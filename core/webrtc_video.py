"""
WebRTC 영상 수신 (save_stream과 동일 시그널링/연결 로직).
수신 프레임은 디스크에 쓰지 않고 on_frame_callback(channel, img, stream_ts) 만 호출한다.

stream_ts(세 번째 인자) 결정 순서 (_receive_video_frames):
  1) 송신 측이 DataChannel JSON type=frame_metadata 로 보낸 timestamp 를 pts 키로 저장
  2) 비디오 frame.pts 와 같은 pts → 그 메타의 timestamp
  3) 없으면 pts 가 가장 가까운 메타(차이 ≤ 5)
  4) 그래도 없으면 가장 최근 메타의 timestamp
  5) 메타가 전혀 없으면 수신 시각 time.time() (로컬)
송신이 frame_metadata 를 주면 대부분 1~2에서 끝난다.
"""
import asyncio
import json
import logging
import time
from typing import Callable, List, Optional

logger = logging.getLogger(__name__)

try:
    import aiohttp
    AIOHTTP_AVAILABLE = True
except ImportError:
    aiohttp = None
    AIOHTTP_AVAILABLE = False

try:
    from aiortc import RTCPeerConnection, RTCSessionDescription
    from aiortc.sdp import candidate_from_sdp
    AIORTC_AVAILABLE = True
except ImportError:
    RTCPeerConnection = RTCSessionDescription = candidate_from_sdp = None
    AIORTC_AVAILABLE = False

if AIORTC_AVAILABLE:
    from aiortc import RTCIceServer, RTCConfiguration

    from core.aiortc_dtls_compat import apply_aiortc_dtls_peer_cert_patch

    apply_aiortc_dtls_peer_cert_patch()


# aioice STUN 로그 필터 (save_stream과 동일)
class AioiceErrorFilter(logging.Filter):
    def filter(self, record):
        if "aioice" in record.name.lower() or "stun" in record.name.lower():
            if "InvalidStateError" in str(record.getMessage()) or "Transaction.__retry" in str(record.getMessage()):
                return False
        if record.name == "asyncio" and "Transaction.__retry" in str(record.getMessage()):
            if "InvalidStateError" in str(record.getMessage()):
                return False
        return True


def _install_aioice_filter():
    logging.getLogger("asyncio").addFilter(AioiceErrorFilter())
    logging.getLogger("aioice").addFilter(AioiceErrorFilter())


# room_name -> channel index (video1=0, video2=1)
def _room_to_channel(room_name: str, room_names: List[str]) -> int:
    try:
        return room_names.index(room_name)
    except ValueError:
        return 0


class WebRTCVideoReceiver:
    """
    save_stream과 동일한 WebRTC 시그널링으로 room별 영상 수신.
    on_frame_callback(channel, img, stream_ts) 만 호출한다. stream_ts 는 모듈 상단 docstring 순서로 결정.
    """

    def __init__(
        self,
        signaling_host: str,
        signaling_port: int,
        room_names: List[str],
        on_frame_callback: Callable[[int, "np.ndarray", float], None],
    ):
        self.signaling_url = f"ws://{signaling_host}:{signaling_port}"
        self.room_names = room_names
        self.on_frame_callback = on_frame_callback
        self.metadata_max_size = 1000

        self.ice_servers = [
            RTCIceServer(urls=["stun:stun.l.google.com:19302"]),
            RTCIceServer(urls=["stun:stun1.l.google.com:19302"]),
            RTCIceServer(urls=["stun:stun2.l.google.com:19302"]),
        ]
        self.rtc_config = RTCConfiguration(iceServers=self.ice_servers)
        self.pcs: dict = {}  # (client_id, room_name) -> RTCPeerConnection
        self.client_room_map: dict = {}
        self.ws_connections: dict = {}  # room_name -> WebSocket
        self.video_tracks: dict = {}  # (client_id, room_name) -> track
        # DataChannel frame_metadata 메시지로 받은 송신측 timestamp 를 pts 로 인덱싱 (비디오 frame.pts 와 조인)
        self.frame_metadata: dict = {}  # (client_id, room_name) -> {pts: {timestamp, time_base}}
        self.running = False
        self.shutdown_requested = False

    async def send_json(self, data: dict, room_name: Optional[str] = None):
        if room_name:
            if room_name in self.ws_connections:
                ws = self.ws_connections[room_name]
                if ws and not ws.closed:
                    await ws.send_json(data)
        else:
            for ws in self.ws_connections.values():
                if ws and not ws.closed:
                    await ws.send_json(data)

    def filter_rtx_from_sdp(self, sdp: str) -> str:
        """SDP에서 RTX 관련 라인 제거 및 aiortc 호환 정규화 (save_stream과 동일)."""
        lines = [ln.rstrip("\r") for ln in sdp.split("\n")]
        filtered_lines = []
        rtx_payload_types = set()
        non_rtx_pt = set()
        for line in lines:
            line_stripped = line.strip()
            if not line_stripped.startswith("a=rtpmap:"):
                continue
            line_lower = line.lower()
            parts = line.split(":")
            if len(parts) <= 1:
                continue
            payload_type = parts[1].split()[0]
            if "rtx/" in line_lower:
                rtx_payload_types.add(payload_type)
            else:
                non_rtx_pt.add(payload_type)
        for line in lines:
            line_stripped = line.strip()
            line_lower = line.lower()
            should_skip = False
            for rtx_pt in rtx_payload_types:
                if line_stripped.startswith(f"a=rtpmap:{rtx_pt}") and "rtx" in line_lower:
                    should_skip = True
                    break
                if line_stripped.startswith(f"a=fmtp:{rtx_pt}") and "apt=" in line_lower:
                    should_skip = True
                    break
                if line_stripped.startswith(f"a=rtcp-fb:{rtx_pt}"):
                    should_skip = True
                    break
            if not should_skip:
                if line_stripped.startswith("m="):
                    parts = line.split()
                    if len(parts) >= 3:
                        parts[2] = parts[2].upper()
                        if len(parts) == 3:
                            if parts[0] in ("m=video", "m=audio"):
                                parts.append("0")
                                filtered_lines.append(" ".join(parts))
                                filtered_lines.append("a=rtpmap:0 H264/90000")
                                if parts[0] == "m=video":
                                    filtered_lines.append("a=fmtp:0 level-asymmetry-allowed=1;packetization-mode=1;profile-level-id=42e01f")
                                continue
                        filtered_parts = [parts[0], parts[1], parts[2]]
                        for pt in parts[3:]:
                            if pt not in rtx_payload_types:
                                filtered_parts.append(pt)
                        keep_pts = set(filtered_parts[3:])
                        need_h264_fallback = (
                            parts[0] == "m=video"
                            and (len(keep_pts) == 0 or not (keep_pts & non_rtx_pt))
                        )
                        if need_h264_fallback:
                            filtered_lines.append(" ".join([parts[0], parts[1], parts[2], "99"]))
                            filtered_lines.append("a=rtpmap:99 H264/90000")
                            filtered_lines.append("a=fmtp:99 level-asymmetry-allowed=1;packetization-mode=1;profile-level-id=42e01f")
                            continue
                        if len(filtered_parts) > 3:
                            added_h264_99 = False
                            if parts[0] == "m=video" and "99" not in filtered_parts:
                                filtered_parts.append("99")
                                added_h264_99 = True
                            filtered_lines.append(" ".join(filtered_parts))
                            if added_h264_99:
                                filtered_lines.append("a=rtpmap:99 H264/90000")
                                filtered_lines.append("a=fmtp:99 level-asymmetry-allowed=1;packetization-mode=1;profile-level-id=42e01f")
                            continue
                filtered_lines.append(line.rstrip("\n"))
        return "\n".join(filtered_lines)

    def _sdp_force_video_h264_only(self, sdp: str) -> str:
        lines = [ln.rstrip("\r") for ln in sdp.split("\n")]
        out = []
        i = 0
        while i < len(lines):
            line = lines[i]
            stripped = line.strip()
            if not stripped.startswith("m="):
                out.append(line)
                i += 1
                continue
            parts = line.split()
            if len(parts) < 3:
                out.append(line)
                i += 1
                continue
            if parts[0].lower() == "m=video":
                port, proto = parts[1], parts[2].upper()
                payloads = list(parts[3:]) if len(parts) > 3 else []
                if "99" not in payloads:
                    payloads.append("99")
                out.append(f"m=video {port} {proto} " + " ".join(payloads))
                out.append("a=rtpmap:99 H264/90000")
                out.append("a=fmtp:99 level-asymmetry-allowed=1;packetization-mode=1;profile-level-id=42e01f")
                i += 1
                while i < len(lines) and not lines[i].strip().startswith("m="):
                    out.append(lines[i])
                    i += 1
                continue
            out.append(line)
            i += 1
        return "\n".join(out)

    def _sdp_fingerprint_sha256_only(self, sdp: str) -> str:
        lines = []
        for line in sdp.split("\n"):
            line = line.rstrip("\r")
            if line.strip().lower().startswith("a=fingerprint:"):
                parts = line.split(":", 1)
                if len(parts) == 2 and " " in parts[1]:
                    alg = parts[1].split(None, 1)[0]
                    if alg.lower() != "sha-256":
                        continue
            lines.append(line)
        return "\n".join(lines)

    def create_pc(self, target_client_id: str, room_name: str):
        pc = RTCPeerConnection(configuration=self.rtc_config)
        pc_key = (target_client_id, room_name)
        self.pcs[pc_key] = pc
        self.client_room_map[pc_key] = room_name

        @pc.on("connectionstatechange")
        async def on_connectionstatechange():
            state = pc.connectionState
            if state == "connected":
                logger.info("[%s] WebRTC connected for %s", room_name, target_client_id)
            if state in ("failed", "closed"):
                if not self.shutdown_requested and pc_key in self.pcs:
                    del self.pcs[pc_key]
                self.video_tracks.pop(pc_key, None)
                self.frame_metadata.pop(pc_key, None)
                self.client_room_map.pop(pc_key, None)

        @pc.on("iceconnectionstatechange")
        async def on_iceconnectionstatechange():
            if pc.iceConnectionState in ("failed", "disconnected", "closed"):
                self.video_tracks.pop(pc_key, None)
                if pc_key in self.pcs:
                    try:
                        await self.pcs[pc_key].close()
                    except Exception:
                        pass
                    self.pcs.pop(pc_key, None)
                self.client_room_map.pop(pc_key, None)

        @pc.on("datachannel")
        def on_datachannel(channel):
            @channel.on("message")
            def on_message(message):
                # frame_metadata: 송신측 timestamp + pts → 비디오 프레임 pts 와 매칭해 stream_ts 로 전달
                try:
                    data = json.loads(message)
                    msg_type = data.get("type")
                    if msg_type == "keepalive":
                        try:
                            channel.send(json.dumps({"type": "keepalive_ack", "timestamp": time.time()}))
                        except Exception:
                            pass
                        return
                    if msg_type == "frame_metadata":
                        ts = data.get("timestamp")
                        pts = data.get("pts")
                        time_base = data.get("time_base")
                        if ts is not None and pts is not None:
                            if pc_key not in self.frame_metadata:
                                self.frame_metadata[pc_key] = {}
                            self.frame_metadata[pc_key][pts] = {"timestamp": ts, "time_base": time_base}
                            if len(self.frame_metadata[pc_key]) > self.metadata_max_size:
                                m = self.frame_metadata[pc_key]
                                sorted_pts = sorted(m.keys())
                                for old in sorted_pts[: len(sorted_pts) - self.metadata_max_size // 2]:
                                    del m[old]
                except json.JSONDecodeError:
                    pass
            return

        @pc.on("track")
        def on_track(track):
            if track.kind == "video":
                self.video_tracks[pc_key] = track
                asyncio.create_task(self._receive_video_frames(target_client_id, track, room_name))

                @track.on("ended")
                async def on_ended():
                    self.video_tracks.pop(pc_key, None)
            return

        @pc.on("icecandidate")
        async def on_icecandidate(event):
            if event.candidate:
                await self.send_json({
                    "type": "ice-candidate",
                    "candidate": event.candidate.toJSON(),
                    "client_id": target_client_id,
                }, room_name=room_name)

        return pc

    async def _receive_video_frames(self, client_id: str, track, room_name: str):
        pc_key = (client_id, room_name)
        channel = _room_to_channel(room_name, self.room_names)
        consecutive_errors = 0
        max_consecutive_errors = 10

        while self.running and not self.shutdown_requested and pc_key in self.video_tracks:
            try:
                if pc_key in self.pcs and self.pcs[pc_key].connectionState in ("failed", "closed", "disconnected"):
                    break
                if getattr(track, "readyState", None) == "ended":
                    break
                try:
                    frame = await asyncio.wait_for(track.recv(), timeout=5.0)
                except asyncio.TimeoutError:
                    await asyncio.sleep(0.5)
                    continue
                if frame is None:
                    break
                consecutive_errors = 0
                current_frame_time = time.time()
                img = frame.to_ndarray(format="bgr24")
                stream_ts = None  # 송신 DataChannel timestamp 우선, 없으면 수신 시각
                frame_pts = getattr(frame, "pts", None)
                if frame_pts is not None and pc_key in self.frame_metadata:
                    m = self.frame_metadata[pc_key]
                    if frame_pts in m:
                        stream_ts = m[frame_pts].get("timestamp")
                    else:
                        closest = min(m.keys(), key=lambda p: abs(p - frame_pts), default=None)
                        if closest is not None and abs(closest - frame_pts) <= 5:
                            stream_ts = m[closest].get("timestamp")
                if stream_ts is None and pc_key in self.frame_metadata and self.frame_metadata[pc_key]:
                    latest_pts = max(self.frame_metadata[pc_key].keys())
                    stream_ts = self.frame_metadata[pc_key][latest_pts].get("timestamp")
                if stream_ts is None:
                    stream_ts = current_frame_time
                try:
                    self.on_frame_callback(channel, img, float(stream_ts))
                except Exception as e:
                    logger.debug("on_frame_callback error: %s", e)
            except asyncio.CancelledError:
                break
            except Exception as e:
                consecutive_errors += 1
                if consecutive_errors >= max_consecutive_errors:
                    logger.warning("WebRTC video receive errors %d, stopping loop: %s", consecutive_errors, e)
                    break
                await asyncio.sleep(0.1)
        self.video_tracks.pop(pc_key, None)

    async def handle_room_connection(self, room_name: str):
        if not AIOHTTP_AVAILABLE or not aiohttp:
            logger.warning("aiohttp not available, WebRTC video disabled")
            return
        try:
            async with aiohttp.ClientSession() as session:
                async with session.ws_connect(self.signaling_url) as ws:
                    self.ws_connections[room_name] = ws
                    logger.info("[%s] Signaling connected: %s", room_name, self.signaling_url)
                    await ws.send_json({"type": "join", "room": room_name})
                    async for msg in ws:
                        if msg.type == aiohttp.WSMsgType.CLOSED:
                            break
                        if msg.type == aiohttp.WSMsgType.ERROR:
                            break
                        if msg.type != aiohttp.WSMsgType.TEXT:
                            continue
                        data = json.loads(msg.data)
                        msg_type = data.get("type")
                        if msg_type == "error":
                            logger.error("[%s] Server error: %s", room_name, data.get("message", data))
                            continue
                        if msg_type == "joined":
                            logger.info("[%s] Joined as %s", room_name, data.get("client_id"))
                            continue
                        if msg_type == "offer":
                            sender_id = data.get("client_id")
                            pc_key = (sender_id, room_name)
                            if pc_key in self.pcs:
                                try:
                                    await self.pcs[pc_key].close()
                                except Exception:
                                    pass
                                self.pcs.pop(pc_key, None)
                                self.video_tracks.pop(pc_key, None)
                                self.frame_metadata.pop(pc_key, None)
                            pc = self.create_pc(sender_id, room_name)
                            filtered_sdp = self.filter_rtx_from_sdp(data["offer"]["sdp"])
                            try:
                                await pc.setRemoteDescription(RTCSessionDescription(
                                    sdp=filtered_sdp,
                                    type=data["offer"]["type"],
                                ))
                            except Exception as sdp_err:
                                if "Failed to set remote video description" in str(sdp_err):
                                    filtered_sdp = self._sdp_force_video_h264_only(filtered_sdp)
                                    await pc.setRemoteDescription(RTCSessionDescription(
                                        sdp=filtered_sdp,
                                        type=data["offer"]["type"],
                                    ))
                                else:
                                    raise
                            answer = await pc.createAnswer()
                            await pc.setLocalDescription(answer)
                            answer_sdp = self._sdp_fingerprint_sha256_only(pc.localDescription.sdp)
                            await self.send_json({
                                "type": "answer",
                                "answer": {"sdp": answer_sdp, "type": pc.localDescription.type},
                                "client_id": sender_id,
                            }, room_name=room_name)
                            logger.info("[%s] Sent answer to %s", room_name, sender_id)
                            continue
                        if msg_type == "ice-candidate" and data.get("candidate"):
                            sender_id = data.get("client_id")
                            pc_key = (sender_id, room_name)
                            if pc_key in self.pcs:
                                cand = candidate_from_sdp(data["candidate"]["candidate"])
                                cand.sdpMid = data["candidate"].get("sdpMid")
                                cand.sdpMLineIndex = data["candidate"].get("sdpMLineIndex")
                                await self.pcs[pc_key].addIceCandidate(cand)
        except Exception as e:
            logger.warning("[%s] WebRTC room connection error: %s", room_name, e)
        finally:
            self.ws_connections.pop(room_name, None)

    async def run(self):
        if not AIOHTTP_AVAILABLE or not AIORTC_AVAILABLE:
            logger.warning("aiohttp or aiortc not available. Install: pip install aiohttp aiortc")
            return
        _install_aioice_filter()
        self.running = True
        tasks = [asyncio.create_task(self.handle_room_connection(r)) for r in self.room_names]
        try:
            await asyncio.gather(*tasks)
        except asyncio.CancelledError:
            pass
        finally:
            self.running = False
            for pc_key in list(self.pcs):
                try:
                    await self.pcs[pc_key].close()
                except Exception:
                    pass
            self.pcs.clear()
            self.video_tracks.clear()
            self.frame_metadata.clear()

    def shutdown(self):
        self.shutdown_requested = True
        self.running = False

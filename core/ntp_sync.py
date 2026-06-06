"""
MQTT `/robot/<id>/time_sync`(req/resp)로 오프셋을 잡은 뒤 `get_robot_time_sec()` 에서
**로봇 기준 시각(초)** 을 돌려준다. WebRTC `stream_ts`(DataChannel+pts)와 축이 다르다.
"""
import json
import logging
import threading
import time
from typing import Optional

logger = logging.getLogger(__name__)

def _monotonic_ns() -> int:
    return time.monotonic_ns()


class RobotTimeSync:
    def __init__(self, robot_id: str = "2"):
        self.robot_id = robot_id
        self.req_topic = f"/robot/{robot_id}/time_sync/req"
        self.resp_topic = f"/robot/{robot_id}/time_sync/resp"
        self._best_offset_ns: int = 0
        self._best_rtt_ns: float = float("inf")
        self._lock = threading.Lock()
        self._pending: dict = {}  # sync_id -> (t1_ns, event, result_holder)
        self._next_sync_id = 1

    def handle_response(self, payload: bytes) -> None:
        """MQTT on_message에서 time_sync/resp 수신 시 호출."""
        try:
            data = json.loads(payload.decode("utf-8"))
            sync_id = data.get("sync_id")
            t1_sent = data.get("t1_server_mono_ns")
            t2_robot = data.get("t2_robot_mono_ns")
            t3_robot = data.get("t3_robot_mono_ns")
            if sync_id is None or t2_robot is None or t3_robot is None:
                return
            t4 = _monotonic_ns()
            with self._lock:
                if sync_id not in self._pending:
                    return
                t1, ev, holder = self._pending[sync_id]
                # t1은 우리가 보낸 값; 프로토콜에 따라 t1_sent와 일치할 수 있음
                offset_ns = ((t2_robot - t1) + (t3_robot - t4)) // 2
                rtt_ns = (t4 - t1) - (t3_robot - t2_robot)
                holder.append({"offset_ns": offset_ns, "rtt_ns": rtt_ns})
                if rtt_ns < self._best_rtt_ns:
                    self._best_rtt_ns = rtt_ns
                    self._best_offset_ns = offset_ns
                    logger.debug(
                        "NTP best offset %.3f ms, RTT %.3f ms",
                        offset_ns / 1e6,
                        rtt_ns / 1e6,
                    )
                ev.set()
                del self._pending[sync_id]
        except Exception as e:
            logger.debug("NTP response parse error: %s", e)

    def sync_once(self, publish_fn) -> Optional[dict]:
        """
        한 번의 req/resp 교환. publish_fn(topic, payload) 호출.
        응답은 같은 MQTT 연결의 on_message에서 handle_response로 들어와야 함.
        """
        sync_id = self._next_sync_id
        self._next_sync_id += 1
        t1 = _monotonic_ns()
        ev = threading.Event()
        holder = []
        with self._lock:
            self._pending[sync_id] = (t1, ev, holder)
        req = {"sync_id": sync_id, "t1_server_mono_ns": t1}
        try:
            publish_fn(self.req_topic, json.dumps(req))
        except Exception as e:
            with self._lock:
                self._pending.pop(sync_id, None)
            logger.warning("NTP publish error: %s", e)
            return None
        if not ev.wait(timeout=5.0):
            with self._lock:
                self._pending.pop(sync_id, None)
            return None
        return holder[0] if holder else None

    def run_sync(self, publish_fn, count: int = 20) -> None:
        """count회 교환 후 best offset 유지."""
        for i in range(count):
            self.sync_once(publish_fn)
            time.sleep(0.05)
        with self._lock:
            if self._best_rtt_ns == float("inf"):
                self._best_offset_ns = 0
                logger.warning("NTP 응답 없음. anchor_time은 서버 monotonic 기준 사용.")
            else:
                logger.info(
                    "NTP sync done: best offset %.3f ms, RTT %.3f ms",
                    self._best_offset_ns / 1e6,
                    self._best_rtt_ns / 1e6,
                )

    def get_offset_ns(self) -> int:
        with self._lock:
            return self._best_offset_ns

    def get_robot_time_sec(self, prediction_delay_sec: float = 0.0) -> float:
        with self._lock:
            offset = self._best_offset_ns
        robot_ns = _monotonic_ns() + offset
        robot_sec = robot_ns / 1e9 - prediction_delay_sec
        return robot_sec

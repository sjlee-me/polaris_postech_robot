"""
브로커·시그널링(예: docker compose)이 준비된 뒤 실행합니다.

약 10초간 WebRTC로 영상을 수신하면서
`outputs/test/<실행 시각>/video_<채널>/` 아래에 PNG를 저장합니다.
동시에 백그라운드 워커가 채널 `DETECT_CHANNEL`의 가장 최근 프레임에 대해
YOLOWorld 추론을 돌리고, bbox가 그려진 결과를 `video_<채널>_det/` 에 저장합니다.
워커는 추론 중 들어온 프레임을 모두 큐잉하지 않고 "최신 한 장"만 들고 가서
추론 속도가 프레임 속도를 못 따라가도 항상 가장 최신을 처리합니다.
fire extinguisher 감지 상태가 바뀔 때마다 MQTT(`ai_avoidance_topic`)로
{"ts", "value": 1|0} 을 발행합니다 (감지=1, 미감지=0).
단 1을 발행한 직후 `DETECT_PUBLISH_COOLDOWN_SEC` 동안은 상태 변화 무시
— 모델 출력이 흔들려도 1 다음 즉시 0이 따라가지 않도록.

시각 정리 (헷갈리지 않게):
  - WebRTC 콜백 세 번째 인자 `stream_ts`: 송신측이 데이터 채널 `frame_metadata`로 준
    타임스탬프를 `core/webrtc_video.py`에서 비디오 `pts`와 맞춰 넣은 값(없으면 수신 시각).
    스트림/카메라 쪽 기준으로 쓸 때 사용.
  - 이 파일의 샘플은 파일명·정렬용으로만 **로봇 시각** `time_sync.get_robot_time_sec()` 를 씀.
    MQTT `ts`도 동일(로봇과 센서 로그를 맞출 때).
  - 스트림 시각으로 저장하려면 `on_frame` 안에서 `stream_ts`를 쓰면 됨.
"""
import argparse
import asyncio
import logging
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Callable

import yaml

from core.motion_mqtt import build_motion_payload, format_motion_topic
from core.mqtt_publisher import MqttPublisher
from core.ntp_sync import RobotTimeSync

print("sample-test: OpenCV / WebRTC(aiortc) / YOLOWorld 로딩 중… (처음엔 수십 초 걸릴 수 있음)", file=sys.stderr, flush=True)
import cv2  # noqa: E402

from core.webrtc_video import WebRTCVideoReceiver  # noqa: E402
from ultralytics import YOLOWorld  # noqa: E402

SAMPLE_ROOT = Path(__file__).resolve().parent
CONFIG_PATH = SAMPLE_ROOT / "config.yml"
OUTPUT_BASE = SAMPLE_ROOT / "outputs" / "test"
CAPTURE_SECONDS = 20.0
_MAX_CHANNELS = 16

DETECT_CHANNEL = 0
DETECT_WEIGHTS = "yolov8x-worldv2.pt"
DETECT_CLASSES = ["fire extinguisher"]
DETECT_PUBLISH_COOLDOWN_SEC = 3.0

logger = logging.getLogger(__name__)


def load_config() -> dict:
    with open(CONFIG_PATH, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def setup_logging(level_name: str) -> None:
    level = getattr(logging, str(level_name).upper(), logging.INFO)
    kwargs = dict(
        level=level,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
        stream=sys.stderr,
    )
    try:
        kwargs["force"] = True  # Python 3.8+: 기존 핸들러가 있어도 덮어씀
    except TypeError:
        pass
    logging.basicConfig(**kwargs)
    # 스트림 초기 협상 단계의 정상적인 노이즈는 숨긴다
    logging.getLogger("aioice.ice").setLevel(logging.WARNING)
    logging.getLogger("aiortc.codecs.h264").setLevel(logging.CRITICAL)
    logging.getLogger("libav.h264").setLevel(logging.CRITICAL)
    logging.getLogger("libav").setLevel(logging.CRITICAL)


def _ai_avoidance_topic(cfg: dict) -> str:
    tpl = str(cfg.get("ai_avoidance_topic", "/robot/{robot_id}/ai-avoidance"))
    return format_motion_topic(tpl, str(cfg.get("robot_id", "2")))


class DetectionWorker:
    """가장 최근에 들어온 프레임 한 장에만 YOLO 추론을 돌리는 백그라운드 워커.

    on_frame 콜백은 latency-sensitive 하므로 추론을 직접 부르지 않고 슬롯에 프레임을
    "덮어쓰기"만 한다. 워커 스레드는 슬롯이 비면 짧게 sleep, 차 있으면 꺼내서 추론한다.
    추론이 끝나기 전에 새 프레임이 여러 장 들어오면 슬롯은 마지막 한 장으로 갱신된다 —
    프레임 드랍을 받아들이는 대신 큐 적체와 stale 결과를 피한다.
    """

    def __init__(
        self,
        dst_dir: Path,
        classes: list,
        weights: str,
        on_state_change: Callable[[bool], None] | None = None,
    ) -> None:
        self._dst_dir = dst_dir
        self._classes = list(classes)
        self._weights = weights
        self._lock = threading.Lock()
        self._latest = None  # (fname, img_bgr_copy)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._dropped = 0  # 슬롯이 덮어써질 때(=드랍될 때) 카운트
        self._on_state_change = on_state_change
        self._last_detected: bool | None = None  # None=한 번도 publish 안 함 → 첫 추론 결과는 무조건 발행
        self._cooldown_until: float = 0.0  # monotonic 시각. 이 시점 전까지는 상태 변화 무시

    def submit(self, fname: str, img) -> None:
        with self._lock:
            if self._latest is not None:
                self._dropped += 1
            # img 버퍼가 재사용될 수 있으니 반드시 copy
            self._latest = (fname, img.copy())

    def start(self) -> None:
        self._dst_dir.mkdir(parents=True, exist_ok=True)
        self._thread = threading.Thread(target=self._run, name="yolo-worker", daemon=True)
        self._thread.start()

    def stop(self, timeout: float = 10.0) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=timeout)
        logger.info("Detection 워커 종료. 드랍 프레임 수=%d", self._dropped)

    def _run(self) -> None:
        logger.info("YOLOWorld 로딩: %s, classes=%s", self._weights, self._classes)
        model = YOLOWorld(self._weights)
        model.set_classes(self._classes)
        logger.info("YOLOWorld 준비 완료, 추론 루프 시작 → %s", self._dst_dir)

        while not self._stop.is_set():
            with self._lock:
                item = self._latest
                self._latest = None
            if item is None:
                time.sleep(0.005)
                continue
            fname, img = item
            try:
                results = model.predict(img, verbose=False)
                r = results[0]
                # set_classes로 fire extinguisher만 예측 대상 → boxes가 비어있지 않으면 감지된 것
                detected = r.boxes is not None and len(r.boxes) > 0
                # 쿨다운 중에는 _last_detected를 그대로 두어 쿨다운 종료 후 정확한 비교가 되도록 함
                now = time.monotonic()
                if now >= self._cooldown_until and detected != self._last_detected:
                    self._last_detected = detected
                    if detected:
                        self._cooldown_until = now + DETECT_PUBLISH_COOLDOWN_SEC
                    if self._on_state_change is not None:
                        try:
                            self._on_state_change(detected)
                        except Exception:
                            logger.exception("on_state_change 콜백 실패")
                annotated = r.plot()  # BGR ndarray
                out_path = self._dst_dir / fname
                if not cv2.imwrite(str(out_path), annotated):
                    logger.warning("Detection 결과 저장 실패: %s", out_path)
            except Exception:
                logger.exception("추론 실패: %s", fname)


def _write_video_from_frames(frames_dir: Path, video_path: Path) -> None:
    """`frames_dir` 안의 PNG 시퀀스를 mp4로 합쳐 같은 폴더에 저장.

    파일명이 `{robot_t}_{idx}.png` 라 사전순 정렬이 곧 시각순이고,
    첫/마지막 파일의 timestamp 차이로 FPS를 추정한다 — 추론이 느려서
    프레임이 듬성듬성 들어와도 실제 캡처 길이와 맞는 영상이 된다.
    """
    frames = sorted(frames_dir.glob("*.png"))
    if len(frames) < 2:
        logger.info("Detection 영상 합성 생략: 프레임 부족 (%d)", len(frames))
        return

    def _ts(p: Path) -> float:
        try:
            return float(p.stem.split("_", 1)[0])
        except (ValueError, IndexError):
            return 0.0

    duration = _ts(frames[-1]) - _ts(frames[0])
    fps = (len(frames) - 1) / duration if duration > 0 else 10.0

    first = cv2.imread(str(frames[0]))
    if first is None:
        logger.warning("Detection 영상 합성 생략: 첫 프레임 읽기 실패 (%s)", frames[0])
        return
    h, w = first.shape[:2]
    writer = cv2.VideoWriter(str(video_path), cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
    if not writer.isOpened():
        logger.warning("VideoWriter 열기 실패: %s", video_path)
        return
    try:
        for p in frames:
            img = cv2.imread(str(p))
            if img is None:
                logger.warning("프레임 읽기 실패, 스킵: %s", p)
                continue
            writer.write(img)
    finally:
        writer.release()
    logger.info("Detection 영상 저장: %s (frames=%d, fps=%.2f)", video_path, len(frames), fps)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="WebRTC 캡처 + fire extinguisher 감지 상태 MQTT 발행")
    p.add_argument(
        "--mqtt-only",
        action="store_true",
        help="WebRTC 캡처 없이 MQTT 연결·NTP 동기화만 수행",
    )
    return p.parse_args()


async def main() -> None:
    args = parse_args()
    cfg = load_config()
    setup_logging(cfg.get("log_level", "INFO"))

    room_names = [r.strip() for r in str(cfg.get("room_names", "room1")).split(",") if r.strip()]
    if not room_names:
        room_names = ["room1"]

    time_sync = RobotTimeSync(robot_id=str(cfg.get("robot_id", "2")))

    def on_mqtt(topic: str, payload: bytes) -> None:
        if topic.endswith("/time_sync/resp"):
            time_sync.handle_response(payload)

    mqtt_pub = MqttPublisher.from_config(cfg, on_message=on_mqtt)
    try:
        mqtt_pub.connect()
    except OSError as e:
        logger.error(
            "MQTT 브로커 연결 실패 (%s:%s). compose 전에 브로커가 떠 있는지 확인하세요: %s",
            cfg.get("mqtt_host"),
            cfg.get("mqtt_port"),
            e,
        )
        raise
    logger.info("MQTT 연결 시도 완료(백그라운드 루프). NTP 동기화 시작…")
    logger.info(
        "로봇이 time_sync/resp를 주지 않으면 sync 한 번당 최대 5초씩 대기합니다 (최대 약 %d초).",
        20 * 5,
    )
    try:
        time_sync.run_sync(mqtt_pub.publish_fn, count=20)
    except Exception as e:
        logger.warning("NTP sync: %s", e)
    logger.info("NTP 단계 종료.")

    topic = _ai_avoidance_topic(cfg)

    def publish_session_marker(stage: str) -> None:
        # 세션 시작/종료 마커: value=-1 (정상 동작 값 0/1과 구분되는 sentinel).
        # 다운스트림은 첫 -1 → 세션 시작, 두 번째 -1 → 세션 종료로 해석.
        payload = {"ts": time_sync.get_robot_time_sec(), "value": -1}
        try:
            mqtt_pub.publish_json(topic, payload)
            logger.info("MQTT 발행(session %s): topic=%s payload=%s", stage, topic, payload)
        except Exception:
            logger.exception("MQTT 세션 마커 발행 실패: stage=%s", stage)

    publish_session_marker("start")

    session_name = datetime.now().strftime("%Y%m%d_%H%M%S")
    session_dir = OUTPUT_BASE / session_name
    session_dir.mkdir(parents=True, exist_ok=True)

    num_ch = int(cfg.get("num_video_channels", len(room_names)))
    for i in range(max(num_ch, len(room_names))):
        (session_dir / f"video_{i}").mkdir(parents=True, exist_ok=True)

    locks = [threading.Lock() for _ in range(_MAX_CHANNELS)]
    frame_idxs = [0] * _MAX_CHANNELS

    if not args.mqtt_only:
        logger.info("WebRTC 수신으로 진행합니다.")
        loop = asyncio.get_running_loop()
        capture_until = loop.time() + CAPTURE_SECONDS

        def publish_detection_state(detected: bool) -> None:
            # 감지=1, 미감지=0. 상태가 바뀐 순간에만 호출됨(워커 스레드).
            payload = build_motion_payload(time_sync.get_robot_time_sec(), 1 if detected else 0)
            try:
                mqtt_pub.publish_json(topic, payload)
                logger.info("MQTT 발행(detection=%s): topic=%s payload=%s", detected, topic, payload)
            except Exception:
                logger.exception("MQTT 발행 실패: topic=%s payload=%s", topic, payload)

        det_worker = DetectionWorker(
            dst_dir=session_dir / f"video_{DETECT_CHANNEL}_det",
            classes=DETECT_CLASSES,
            weights=DETECT_WEIGHTS,
            on_state_change=publish_detection_state,
        )
        det_worker.start()

        def on_frame(channel: int, img, stream_ts: float) -> None:
            # stream_ts: webrtc_video가 데이터 채널 메타+pts로 맞춘 송신측 타임스탬프(초); 미수신 시 수신 시각
            if channel < 0 or channel >= _MAX_CHANNELS:
                return
            if loop.time() >= capture_until:
                return
            # 샘플: 디스크 파일명은 로봇 시각(센서·MQTT ts와 동일 눈금). 스트림 시각 쓰려면 stream_ts 사용.
            robot_t = time_sync.get_robot_time_sec()
            with locks[channel]:
                idx = frame_idxs[channel]
                frame_idxs[channel] = idx + 1
            fname = f"{robot_t:.9f}_{idx:06d}.png"
            out_path = session_dir / f"video_{channel}" / fname
            if not cv2.imwrite(str(out_path), img):
                logger.warning("프레임 저장 실패: %s", out_path)
            if channel == DETECT_CHANNEL:
                det_worker.submit(fname, img)

        receiver = WebRTCVideoReceiver(
            signaling_host=str(cfg.get("signaling_host", "localhost")),
            signaling_port=int(cfg.get("signaling_port", 4732)),
            room_names=room_names,
            on_frame_callback=on_frame,
        )

        run_task = asyncio.create_task(receiver.run())
        try:
            await asyncio.sleep(CAPTURE_SECONDS)
        finally:
            receiver.shutdown()
            run_task.cancel()
            try:
                await run_task
            except asyncio.CancelledError:
                pass
            det_worker.stop()
            det_dir = session_dir / f"video_{DETECT_CHANNEL}_det"
            _write_video_from_frames(det_dir, det_dir / f"video_{DETECT_CHANNEL}_det.mp4")
    else:
        logger.info("--mqtt-only: 영상 캡처 생략")

    publish_session_marker("end")
    mqtt_pub.disconnect()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except Exception:
        if not logging.root.handlers:
            logging.basicConfig(level=logging.DEBUG, stream=sys.stderr)
        logging.exception("sample-test 비정상 종료")
        raise

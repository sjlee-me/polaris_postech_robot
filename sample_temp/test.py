"""
브로커·시그널링(예: docker compose)이 준비된 뒤 실행합니다.

약 10초간 WebRTC로 영상을 수신하면서
`outputs/test/<실행 시각>/video_<채널>/` 아래에 PNG를 저장하고,
완료 후 MQTT(`ai_avoidance_topic`)로 {"ts", "value"} 를 한 번 발행합니다.

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
from datetime import datetime
from pathlib import Path

import yaml

from core.motion_mqtt import build_motion_payload, format_motion_topic, parse_cli_motion
from core.mqtt_publisher import MqttPublisher
from core.ntp_sync import RobotTimeSync

print("sample-test: OpenCV / WebRTC(aiortc) 로딩 중… (처음엔 수십 초 걸릴 수 있음)", file=sys.stderr, flush=True)
import cv2  # noqa: E402

from core.webrtc_video import WebRTCVideoReceiver  # noqa: E402

SAMPLE_ROOT = Path(__file__).resolve().parent
CONFIG_PATH = SAMPLE_ROOT / "config.yml"
OUTPUT_BASE = SAMPLE_ROOT / "outputs" / "test"
CAPTURE_SECONDS = 10.0
_MAX_CHANNELS = 16

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


def _ai_avoidance_topic(cfg: dict) -> str:
    tpl = str(cfg.get("ai_avoidance_topic", "/robot/{robot_id}/ai-avoidance"))
    return format_motion_topic(tpl, str(cfg.get("robot_id", "2")))


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="WebRTC 캡처 후 MQTT로 RUN/STOP 명령 발행")
    p.add_argument(
        "--motion",
        default=None,
        metavar="VAL",
        help='주행 명령: 0|1 또는 RUN|STOP (생략 시 config의 motion_default)',
    )
    p.add_argument(
        "--mqtt-only",
        action="store_true",
        help="WebRTC 캡처 없이 NTP 동기화 후 MQTT 명령만 발행",
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

    if args.motion is not None:
        motion_token = parse_cli_motion(args.motion)
    else:
        motion_raw = cfg.get("motion_default", 1)
        motion_token = parse_cli_motion(str(motion_raw)) if isinstance(motion_raw, str) else motion_raw

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
    else:
        logger.info("--mqtt-only: 영상 캡처 생략")

    topic = _ai_avoidance_topic(cfg)
    payload = build_motion_payload(time_sync.get_robot_time_sec(), motion_token)
    mqtt_pub.publish_json(topic, payload)
    logger.info("MQTT 발행 완료: topic=%s payload=%s", topic, payload)
    mqtt_pub.disconnect()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except Exception:
        if not logging.root.handlers:
            logging.basicConfig(level=logging.DEBUG, stream=sys.stderr)
        logging.exception("sample-test 비정상 종료")
        raise

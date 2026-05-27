"""
MQTT 연결, 토픽 발행, NTP(time_sync) 요청용 publish_fn 제공.
"""
import json
import logging
from typing import Callable, Optional

import paho.mqtt.client as mqtt

logger = logging.getLogger(__name__)


def ai_avoidance_topic(robot_id: str) -> str:
    return f"/robot/{robot_id}/ai-avoidance"


def _connect_rc(reason_code) -> int:
    if isinstance(reason_code, int):
        return reason_code
    return int(getattr(reason_code, "value", reason_code))


class MqttPublisher:
    """브로커 연결, 구독(time_sync/resp), 임의 토픽 발행."""

    def __init__(
        self,
        robot_id: str,
        host: str,
        port: int,
        on_message: Optional[Callable[[str, bytes], None]] = None,
    ):
        self.robot_id = robot_id
        self.host = host
        self.port = int(port)
        self._on_message_cb = on_message
        self._client: Optional[mqtt.Client] = None

    @classmethod
    def from_config(cls, cfg: dict, on_message: Optional[Callable[[str, bytes], None]] = None):
        return cls(
            robot_id=str(cfg.get("robot_id", "2")),
            host=str(cfg.get("mqtt_host", "localhost")),
            port=int(cfg.get("mqtt_port", 1883)),
            on_message=on_message,
        )

    def _make_client(self) -> mqtt.Client:
        cid = f"sample-{self.robot_id}"
        try:
            return mqtt.Client(
                callback_api_version=mqtt.CallbackAPIVersion.VERSION1,
                client_id=cid,
                protocol=mqtt.MQTTv311,
            )
        except (AttributeError, TypeError):
            try:
                return mqtt.Client(client_id=cid, protocol=mqtt.MQTTv311)
            except (AttributeError, TypeError):
                return mqtt.Client(client_id=cid)

    def _on_connect(self, client, userdata, flags, reason_code, properties=None):
        rc = _connect_rc(reason_code)
        if rc != 0:
            logger.warning("MQTT connect failed rc=%s", reason_code)
            return
        resp = f"/robot/{self.robot_id}/time_sync/resp"
        client.subscribe(resp, qos=0)
        logger.info("MQTT subscribed: %s", resp)

    def _on_message(self, client, userdata, msg):
        if self._on_message_cb:
            try:
                self._on_message_cb(msg.topic, msg.payload)
            except Exception as e:
                logger.debug("on_message callback error: %s", e)

    def connect(self) -> None:
        self._client = self._make_client()
        self._client.on_connect = self._on_connect
        self._client.on_message = self._on_message
        self._client.connect(self.host, self.port, keepalive=60)
        self._client.loop_start()

    def disconnect(self) -> None:
        if self._client is not None:
            self._client.loop_stop()
            try:
                self._client.disconnect()
            except Exception:
                pass
            self._client = None

    def publish(self, topic: str, payload: str, qos: int = 0) -> None:
        if self._client is None:
            raise RuntimeError("MQTT not connected")
        self._client.publish(topic, payload.encode("utf-8"), qos=qos)

    def publish_json(self, topic: str, data: dict, qos: int = 0) -> None:
        self.publish(topic, json.dumps(data, ensure_ascii=False), qos=qos)

    def publish_fn(self, topic: str, payload: str) -> None:
        """RobotTimeSync.sync_once에 넘기는 (topic, payload) 발행 함수."""
        self.publish(topic, payload, qos=0)

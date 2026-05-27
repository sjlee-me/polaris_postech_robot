#!/usr/bin/env python3
"""Shared helpers for MQTT protocol testing with a real broker."""

from __future__ import annotations

import json
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional


Callback = Callable[[str, Dict[str, Any]], None]


class TopicTransport(ABC):
    @abstractmethod
    def subscribe(self, topic: str, callback: Callback) -> None:
        raise NotImplementedError

    @abstractmethod
    def publish(self, topic: str, payload: Dict[str, Any]) -> None:
        raise NotImplementedError

    def start(self) -> None:
        return

    def stop(self) -> None:
        return


class PahoMqttTransport(TopicTransport):
    def __init__(self, host: str, port: int, client_id: str, logger: Optional["Logger"] = None) -> None:
        try:
            import paho.mqtt.client as mqtt  # type: ignore
        except ImportError as exc:
            raise RuntimeError(
                "paho-mqtt is required for broker mode. Install with: pip install paho-mqtt"
            ) from exc

        self._mqtt = mqtt
        self._client = mqtt.Client(client_id=client_id)
        self._callbacks: Dict[str, List[Callback]] = {}
        self._host = host
        self._port = port
        self._logger = logger
        self._client.on_connect = self._on_connect
        self._client.on_message = self._on_message

    def _on_connect(self, client: Any, userdata: Any, flags: Any, rc: int) -> None:
        del client, userdata, flags
        if self._logger:
            self._logger.log(0.0, f"mqtt connected rc={rc}")
        for topic in self._callbacks:
            self._client.subscribe(topic)

    def _on_message(self, client: Any, userdata: Any, msg: Any) -> None:
        del client, userdata
        payload = json.loads(msg.payload.decode("utf-8"))
        for callback in self._callbacks.get(msg.topic, []):
            callback(msg.topic, payload)

    def subscribe(self, topic: str, callback: Callback) -> None:
        self._callbacks.setdefault(topic, []).append(callback)
        try:
            self._client.subscribe(topic)
        except Exception:
            pass

    def publish(self, topic: str, payload: Dict[str, Any]) -> None:
        self._client.publish(topic, json.dumps(payload))

    def start(self) -> None:
        self._client.connect(self._host, self._port, keepalive=60)
        self._client.loop_start()

    def stop(self) -> None:
        self._client.loop_stop()
        self._client.disconnect()

@dataclass
class Logger:
    name: str
    entries: List[str] = field(default_factory=list)

    def log(self, sim_time_s: float, message: str) -> None:
        line = f"[{sim_time_s:05.1f}s] [{self.name}] {message}"
        self.entries.append(line)
        print(line)


def robot_topic(robot_id: int, suffix: str) -> str:
    return f"/robot/{robot_id}/{suffix}"


def sample_graph_payload() -> Dict[str, Any]:
    return {
        "vertices": [
            {"id": 101, "x": 0.0, "y": 0.0},
            {"id": 102, "x": 1.0, "y": 0.0},
            {"id": 103, "x": 2.0, "y": 0.0},
            {"id": 104, "x": 3.0, "y": 0.0},
            {"id": 105, "x": 4.0, "y": 0.0},
            {"id": 106, "x": 5.0, "y": 0.0},
            {"id": 201, "x": 2.0, "y": 1.0},
            {"id": 202, "x": 3.0, "y": 1.0},
        ],
        "edges": [
            {"id": 1, "v0": 101, "v1": 102},
            {"id": 2, "v0": 102, "v1": 103},
            {"id": 3, "v0": 103, "v1": 104},
            {"id": 4, "v0": 104, "v1": 105},
            {"id": 5, "v0": 105, "v1": 106},
            {"id": 6, "v0": 103, "v1": 201},
            {"id": 7, "v0": 201, "v1": 202},
            {"id": 8, "v0": 202, "v1": 105},
        ],
    }

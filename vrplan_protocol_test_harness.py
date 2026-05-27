#!/usr/bin/env python3
"""Mock vrplan-side harness to validate the VLA MQTT protocol flow."""

from __future__ import annotations

import argparse
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Set

from mqtt_protocol_sim import (
    Logger,
    PahoMqttTransport,
    TopicTransport,
    robot_topic,
    sample_graph_payload,
)


@dataclass
class ScenarioConfig:
    robot_id: int = 1
    mission_id: str = "mission-001"
    duration_s: float = 12.0
    step_s: float = 0.5


class MockVrplanHarness:
    def __init__(self, bus: TopicTransport, config: ScenarioConfig, logger: Logger) -> None:
        self.bus = bus
        self.config = config
        self.logger = logger
        self.graph = sample_graph_payload()
        self.map_name = "floor_7"
        self.graph_name = "floor_7_main"
        self.active_vertices: Set[int] = set()
        self.disabled_vertices: Set[int] = set()
        self.last_vla_status: Dict[str, Any] | None = None
        self.last_vertex_timestamp_s: float | None = None
        self.last_status_timestamp_s: float | None = None
        self.route_version = 1
        self.driving = False
        self.paused = False
        self.pose_index = 0
        self.poses: List[Dict[str, float]] = [
            {"x": 0.0, "y": 0.0, "theta": 0.0},
            {"x": 0.7, "y": 0.0, "theta": 0.0},
            {"x": 1.4, "y": 0.0, "theta": 0.0},
            {"x": 2.1, "y": 0.0, "theta": 0.0},
            {"x": 2.8, "y": 0.0, "theta": 0.0},
            {"x": 3.5, "y": 0.0, "theta": 0.0},
            {"x": 4.2, "y": 0.0, "theta": 0.0},
        ]

        bus.subscribe(robot_topic(config.robot_id, "get_graph"), self.on_get_graph)
        bus.subscribe(robot_topic(config.robot_id, "vla_status"), self.on_vla_status)
        bus.subscribe(robot_topic(config.robot_id, "active_vertex_list"), self.on_active_vertex_list)

        self._mission_started = False
        self._paused_once = False
        self._resumed_once = False
        self._completed = False

    def on_get_graph(self, topic: str, payload: Dict[str, Any]) -> None:
        del topic
        response = {
            "robotId": self.config.robot_id,
            "timestampMs": float(payload["timestampMs"]),
            "requestId": payload["requestId"],
            "mapName": self.map_name,
            "graphName": self.graph_name,
            "graph": self.graph,
        }
        self.logger.log(float(payload["timestampMs"]), "respond graph payload (v::Graph)")
        self.bus.publish(robot_topic(self.config.robot_id, "graph"), response)

    def on_vla_status(self, topic: str, payload: Dict[str, Any]) -> None:
        del topic
        self.last_vla_status = payload
        self.last_status_timestamp_s = float(payload["timestampMs"])
        self.logger.log(
            self.last_status_timestamp_s,
            f"recv vla_status enable={payload['enable']} state={payload['state']}",
        )

    def on_active_vertex_list(self, topic: str, payload: Dict[str, Any]) -> None:
        del topic
        self.last_vertex_timestamp_s = float(payload["timestampMs"])
        incoming = set(payload.get("vertexIds", []))
        changed = incoming != self.active_vertices
        self.active_vertices = incoming
        all_vertex_ids = {int(vertex["id"]) for vertex in self.graph["vertices"]}
        self.disabled_vertices = all_vertex_ids - self.active_vertices
        self.logger.log(self.last_vertex_timestamp_s, f"recv active_vertex_list {sorted(incoming)}")
        if changed:
            self.route_version += 1
            self.logger.log(
                self.last_vertex_timestamp_s,
                f"overlay updated disabled={sorted(self.disabled_vertices)} -> replan triggered route_version={self.route_version}",
            )

    def publish_init_vertex(self, sim_time_s: float) -> None:
        payload = {
            "robotId": self.config.robot_id,
            "timestampMs": float(sim_time_s),
            "missionId": self.config.mission_id,
            "graphName": self.graph_name,
        }
        self.logger.log(sim_time_s, f"publish init_vertex graphName={self.graph_name}")
        self.bus.publish(robot_topic(self.config.robot_id, "init_vertex"), payload)

    def publish_vla_enable(self, sim_time_s: float, enable: bool) -> None:
        payload = {
            "robotId": self.config.robot_id,
            "timestampMs": float(sim_time_s),
            "missionId": self.config.mission_id,
            "enable": enable,
        }
        self.logger.log(sim_time_s, f"publish vla_enable enable={enable}")
        self.bus.publish(robot_topic(self.config.robot_id, "vla_enable"), payload)

    def publish_robot_infos(self, sim_time_s: float) -> None:
        pose = self.poses[min(self.pose_index, len(self.poses) - 1)]
        payload = {
            "robotId": self.config.robot_id,
            "timestampMs": float(sim_time_s),
            "missionId": self.config.mission_id,
            "pose": pose,
            "mapName": self.map_name,
            "graphName": self.graph_name,
        }
        self.logger.log(sim_time_s, f"publish robot_infos pose={pose}")
        self.bus.publish(robot_topic(self.config.robot_id, "robot_infos"), payload)
        if self.pose_index < len(self.poses) - 1:
            self.pose_index += 1

    def step(self, sim_time_s: float) -> None:
        if not self._mission_started and sim_time_s >= 2.0:
            self.publish_init_vertex(sim_time_s)
            self.publish_vla_enable(sim_time_s, True)
            self.driving = True
            self._mission_started = True

        if not self._paused_once and sim_time_s >= 7.0:
            self.publish_vla_enable(sim_time_s, False)
            self.paused = True
            self.driving = False
            self._paused_once = True

        if self._paused_once and not self._resumed_once and sim_time_s >= 8.0:
            self.publish_init_vertex(sim_time_s)
            self.publish_vla_enable(sim_time_s, True)
            self.paused = False
            self.driving = True
            self._resumed_once = True

        if not self._completed and sim_time_s >= 10.0:
            self.publish_vla_enable(sim_time_s, False)
            self.driving = False
            self._completed = True

        if self.driving and int((sim_time_s * 10) % 10) == 0:
            self.publish_robot_infos(sim_time_s)

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the vrplan/VLA MQTT protocol harness.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=1883)
    parser.add_argument("--robot-id", type=int, default=1)
    parser.add_argument("--mission-id", default="mission-001")
    parser.add_argument("--duration", type=float, default=12.0)
    parser.add_argument("--step", type=float, default=0.5)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config = ScenarioConfig(
        robot_id=args.robot_id,
        mission_id=args.mission_id,
        duration_s=args.duration,
        step_s=args.step,
    )
    vrplan_logger = Logger("mock-vrplan")
    transport = PahoMqttTransport(
        host=args.host,
        port=args.port,
        client_id=f"mock-vrplan-{config.robot_id}",
        logger=vrplan_logger,
    )
    harness = MockVrplanHarness(transport, config, vrplan_logger)
    transport.start()
    start_wall = time.monotonic()
    try:
        while True:
            sim_time_s = time.monotonic() - start_wall
            harness.step(sim_time_s)
            if sim_time_s >= config.duration_s:
                break
            time.sleep(config.step_s)
    finally:
        transport.stop()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Reference VLA node for protocol-first development."""

from __future__ import annotations

import argparse
import math
import os
import time
from typing import Any, Dict, List, Optional, Set, Tuple

from mqtt_protocol_sim import Logger, PahoMqttTransport, TopicTransport, robot_topic


DEFAULT_DETECTION_CLASSES = ["fire extinguisher"]
DEFAULT_DETECTION_DISABLE_RADIUS_M = 0.5
MAX_DETECTED_OBJECT_DISTANCE_M = 10
DETECTED_OBJECT_MERGE_RADIUS_M = 0.15

# 카메라 양 옆(좌/우)과 아래쪽 외곽은 렌즈 왜곡이 커서 좌표 신뢰도가 떨어진다.
# 소화기 bbox 하단 변의 중점(=(cx, y2)) 이 이미지 '가운데 영역' 안에 있을 때만 detect 를 믿는다.
DETECTION_VALID_X_MARGIN_RATIO = 0.15  # 좌/우 각각 가장자리에서 제외할 폭 비율
DETECTION_VALID_BOTTOM_MARGIN_RATIO = 0.0  # 아래쪽 가장자리에서 제외할 높이 비율


def is_detection_pixel_reliable(bbox: Any, image_size: Any) -> bool:
    """bbox 하단 중점이 이미지 가운데 영역(왜곡이 적은 영역) 안에 있는지 확인한다.

    좌/우 가장자리와 아래쪽 가장자리에 잡힌 detection 은 왜곡 때문에 신뢰하지 않는다.
    bbox/imageSize 정보가 부족하면 거르지 않는다(=True).
    """
    if not isinstance(bbox, dict) or not isinstance(image_size, dict):
        return True
    try:
        width = float(image_size.get("width", 0))
        height = float(image_size.get("height", 0))
        pixel_x = float(bbox["cx"])
        pixel_y = float(bbox["y2"])
    except (KeyError, TypeError, ValueError):
        return True
    if width <= 0 or height <= 0:
        return True
    min_x = width * DETECTION_VALID_X_MARGIN_RATIO
    max_x = width * (1.0 - DETECTION_VALID_X_MARGIN_RATIO)
    max_y = height * (1.0 - DETECTION_VALID_BOTTOM_MARGIN_RATIO)
    return min_x <= pixel_x <= max_x and pixel_y <= max_y


class VlaReferenceNode:
    def __init__(
        self,
        bus: TopicTransport,
        robot_id: int,
        logger: Logger,
        view: bool = False,
        save_dir: Optional[str] = None,
        backend: Optional[str] = None,
        detection_classes: Optional[List[str]] = None,
        detection_disable_radius_m: float = DEFAULT_DETECTION_DISABLE_RADIUS_M,
    ) -> None:
        self.bus = bus
        self.robot_id = robot_id
        self.logger = logger
        self.view = view
        self.save_dir = save_dir
        self.backend = backend
        self.enabled = False
        self.state = "initializing"
        self.graph: Optional[Dict[str, Any]] = None
        self.mission_id: Optional[str] = None
        self.latest_pose: Optional[Dict[str, float]] = None
        self.latest_map_name: Optional[str] = None
        self.latest_graph_name: Optional[str] = None
        self.detection_classes = list(detection_classes or DEFAULT_DETECTION_CLASSES)
        self.detection_disable_radius_m = detection_disable_radius_m
        self.latest_detection_status: Optional[Dict[str, Any]] = None
        self.list_detected_object: List[Dict[str, Any]] = []
        self.disabled_vertices: Set[int] = set()
        self.active_vertices: Set[int] = set()
        self._graph_request_interval_s = 3.0
        self._last_graph_request_s = -1.0
        self._last_status_pub_s = -1.0
        self._last_vertex_pub_s = -1.0
        self._last_detection_config_pub_s = -1.0
        self._last_detection_enable_pub_s = -1.0
        self._plot_ready = False
        self._plt = None
        self._fig = None
        self._ax = None
        self._scatter = None
        self._vertex_order: List[int] = []
        self._vertex_positions: Dict[int, Tuple[float, float]] = {}
        self._frame_counter = 0
        self._history: List[Dict[str, Any]] = []

        bus.subscribe(robot_topic(robot_id, "graph"), self.on_graph)
        bus.subscribe(robot_topic(robot_id, "robot_infos"), self.on_robot_infos)
        bus.subscribe(robot_topic(robot_id, "init_vertex"), self.on_init_vertex)
        bus.subscribe(robot_topic(robot_id, "vla_enable"), self.on_vla_enable)
        bus.subscribe(robot_topic(robot_id, "detection_status"), self.on_detection_status)
        bus.subscribe(robot_topic(robot_id, "detected_objects"), self.on_detected_objects)

    def start(self, sim_time_s: float) -> None:
        self.state = "initializing"
        self.request_graph(sim_time_s)
        self.publish_detection_config(sim_time_s)

    def tick(self, sim_time_s: float) -> None:
        if self.graph is None and (
            self._last_graph_request_s < 0.0
            or sim_time_s - self._last_graph_request_s >= self._graph_request_interval_s
        ):
            self.request_graph(sim_time_s)

        self.update_state()

        if sim_time_s - self._last_status_pub_s >= 1.0:
            self.publish_status(sim_time_s)
            self._last_status_pub_s = sim_time_s

        self.maybe_publish_detection_config(sim_time_s)
        self.maybe_publish_detection_enable(sim_time_s)

        if (
            self.state == "working"
            and sim_time_s - self._last_vertex_pub_s >= 1.0
        ):
            self.update_disabled_vertices()
            self.publish_active_vertex_list(sim_time_s)
            self._last_vertex_pub_s = sim_time_s

    def request_graph(self, sim_time_s: float) -> None:
        self._last_graph_request_s = sim_time_s
        payload = {
            "robotId": self.robot_id,
            "timestampMs": float(sim_time_s),
            "requestId": "graph-req-001",
        }
        self.logger.log(sim_time_s, "publish get_graph request")
        self.bus.publish(robot_topic(self.robot_id, "get_graph"), payload)

    def publish_status(self, sim_time_s: float) -> None:
        payload = {
            "robotId": self.robot_id,
            "timestampMs": float(sim_time_s),
            "enable": self.enabled,
            "state": self.state,
        }
        self.bus.publish(robot_topic(self.robot_id, "vla_status"), payload)
        self.logger.log(sim_time_s, f"publish vla_status enable={self.enabled} state={self.state}")

    def publish_detection_config(self, sim_time_s: float) -> None:
        payload = {
            "robotId": self.robot_id,
            "timestampMs": float(sim_time_s),
            "classes": list(self.detection_classes),
        }
        self.bus.publish(robot_topic(self.robot_id, "detection_config"), payload)
        self._last_detection_config_pub_s = sim_time_s
        self.logger.log(sim_time_s, f"publish detection_config classes={self.detection_classes}")

    def publish_detection_enable(self, sim_time_s: float, enable: bool) -> None:
        payload = {
            "robotId": self.robot_id,
            "timestampMs": float(sim_time_s),
            "missionId": self.mission_id or "",
            "enable": bool(enable),
        }
        self.bus.publish(robot_topic(self.robot_id, "detection_enable"), payload)
        self._last_detection_enable_pub_s = sim_time_s
        self.logger.log(sim_time_s, f"publish detection_enable enable={enable} missionId={payload['missionId']}")

    def publish_detected_object_debug(
        self,
        timestamp_ms: float,
        class_name: str,
        local_x: float,
        local_y: float,
        global_x: float,
        global_y: float,
        robot_x: float,
        robot_y: float,
        robot_theta: float,
    ) -> None:
        payload = {
            "robotId": self.robot_id,
            "timestampMs": float(timestamp_ms),
            "missionId": self.mission_id or "",
            "className": class_name,
            "local": {"x": local_x, "y": local_y},
            "global": {"x": global_x, "y": global_y},
            "robot": {"x": robot_x, "y": robot_y, "theta": robot_theta},
        }
        self.bus.publish(robot_topic(self.robot_id, "detected_object_debug"), payload)

    def maybe_publish_detection_config(self, sim_time_s: float) -> None:
        if self.detection_config_ready():
            return
        if sim_time_s - self._last_detection_config_pub_s < 1.0:
            return
        self.publish_detection_config(sim_time_s)

    def maybe_publish_detection_enable(self, sim_time_s: float) -> None:
        if sim_time_s - self._last_detection_enable_pub_s < 1.0:
            return

        if self.enabled:
            if not self.detection_working():
                self.publish_detection_enable(sim_time_s, True)
        elif self.latest_detection_status and bool(self.latest_detection_status.get("enable", False)):
            self.publish_detection_enable(sim_time_s, False)

    def detection_config_ready(self) -> bool:
        if self.latest_detection_status is None:
            return False
        status_classes = self.latest_detection_status.get("classes", [])
        return (
            bool(self.latest_detection_status.get("classesSet", False))
            and self.latest_detection_status.get("state") in {"idle", "working"}
            and isinstance(status_classes, list)
            and status_classes == self.detection_classes
        )

    def detection_idle(self) -> bool:
        return (
            self.detection_config_ready()
            and self.latest_detection_status is not None
            and self.latest_detection_status.get("state") == "idle"
            and not bool(self.latest_detection_status.get("enable", False))
        )

    def detection_working(self) -> bool:
        return (
            self.detection_config_ready()
            and self.latest_detection_status is not None
            and bool(self.latest_detection_status.get("enable", False))
            and self.latest_detection_status.get("state") == "working"
        )

    def update_state(self) -> None:
        if self.enabled:
            if (
                self.graph is not None
                and self.latest_pose is not None
                and bool(self.mission_id)
                and self.detection_working()
            ):
                self.state = "working"
            else:
                self.state = "initializing"
        elif self.graph is not None and self.detection_idle():
            self.state = "idle"
        else:
            self.state = "initializing"

    def publish_active_vertex_list(self, sim_time_s: float) -> None:
        active_vertices = self.current_active_vertices()
        payload = {
            "robotId": self.robot_id,
            "timestampMs": float(sim_time_s),
            "missionId": self.mission_id,
            "mapName": self.latest_map_name,
            "graphName": self.latest_graph_name,
            "vertexIds": sorted(active_vertices),
        }
        self.bus.publish(robot_topic(self.robot_id, "active_vertex_list"), payload)
        self.logger.log(sim_time_s, f"publish active_vertex_list {sorted(active_vertices)}")
        self.record_history(sim_time_s)
        self.update_plot()

    def on_graph(self, topic: str, payload: Dict[str, Any]) -> None:
        del topic
        self.graph = payload["graph"]
        self.latest_map_name = payload.get("mapName")
        self.latest_graph_name = payload.get("graphName")
        vertices = self.graph.get("vertices", []) if isinstance(self.graph, dict) else []
        edges = self.graph.get("edges", []) if isinstance(self.graph, dict) else []
        self.logger.log(
            float(payload.get("timestampMs", 0.0)),
            f"recv graph requestId={payload.get('requestId')} "
            f"mapName={self.latest_map_name} graphName={self.latest_graph_name} "
            f"vertices={len(vertices)} edges={len(edges)}",
        )
        # 그래프 수신 시 각 node 의 global 좌표 전체 로깅.
        for vertex in vertices:
            if not isinstance(vertex, dict):
                continue
            self.logger.log(
                float(payload.get("timestampMs", 0.0)),
                f"  graph node id={vertex.get('id')} "
                f"global=({float(vertex.get('x', 0.0)):.3f}, {float(vertex.get('y', 0.0)):.3f})",
            )
        self.reset_active_vertices()
        self.update_state()
        self.setup_plot_if_needed()
        self.update_plot()

    def on_robot_infos(self, topic: str, payload: Dict[str, Any]) -> None:
        del topic
        pose = payload.get("pose", {})
        self.latest_pose = {
            "x": float(pose.get("x", 0.0)),
            "y": float(pose.get("y", 0.0)),
            "theta": float(pose.get("theta", 0.0)),
        }
        self.mission_id = payload.get("missionId")
        self.latest_map_name = payload.get("mapName")
        self.latest_graph_name = payload.get("graphName")
        self.update_state()

    def on_init_vertex(self, topic: str, payload: Dict[str, Any]) -> None:
        del topic
        graph_name = payload.get("graphName")
        if graph_name and self.latest_graph_name and graph_name != self.latest_graph_name:
            self.logger.log(float(payload["timestampMs"]), f"ignore init_vertex graphName={graph_name}")
            return
        self.reset_active_vertices()
        self.list_detected_object.clear()
        self.logger.log(float(payload["timestampMs"]), f"init_vertex graphName={graph_name}")
        self.update_plot()

    def on_vla_enable(self, topic: str, payload: Dict[str, Any]) -> None:
        del topic
        self.enabled = bool(payload.get("enable", False))
        self.mission_id = payload.get("missionId", self.mission_id)
        self.logger.log(
            float(payload.get("timestampMs", 0.0)),
            f"recv vla_enable enable={self.enabled} missionId={self.mission_id}",
        )
        if self.enabled:
            self.list_detected_object.clear()
        self.update_state()
        self.publish_detection_enable(float(payload.get("timestampMs", 0.0)), self.enabled)
        self.update_plot()

    def on_detection_status(self, topic: str, payload: Dict[str, Any]) -> None:
        del topic
        self.latest_detection_status = payload
        self.update_state()
        self.logger.log(
            float(payload.get("timestampMs", 0.0)),
            f"recv detection_status enable={payload.get('enable')} state={payload.get('state')}",
        )

    def on_detected_objects(self, topic: str, payload: Dict[str, Any]) -> None:
        del topic
        robot_infos = payload.get("robotInfos", {})
        objects = payload.get("objects", [])
        image_size = payload.get("imageSize", {})
        added_count = 0

        if not isinstance(robot_infos, dict) or not isinstance(objects, list):
            objects = []
        else:
            detected_mission_id = robot_infos.get("missionId", "")
            same_mission = not (
                self.mission_id and detected_mission_id and detected_mission_id != self.mission_id
            )
            if same_mission:
                timestamp_ms = payload.get("timestampMs")
                for detected_object in objects:
                    if not isinstance(detected_object, dict):
                        continue

                    class_name = str(detected_object.get("className", "")).strip()
                    if not class_name:
                        continue

                    # 소화기 bbox 하단 중점이 이미지 좌/우 가장자리나 너무 아래쪽이면
                    # 왜곡 때문에 좌표를 믿지 않고 건너뛴다.
                    if not is_detection_pixel_reliable(detected_object.get("bbox"), image_size):
                        self.logger.log(
                            float(payload.get("timestampMs", 0.0)),
                            f"skip detection class={class_name}: bbox bottom-center "
                            f"outside valid region bbox={detected_object.get('bbox')}",
                        )
                        continue

                    local = detected_object.get("local")
                    if not isinstance(local, dict):
                        continue

                    try:
                        local_x = float(local["x"]) + 0.3
                        local_y = float(local["y"])
                    except (KeyError, TypeError, ValueError):
                        continue

                    # # Filter out detections that are too far away in local coordinates to be robust against noisy detections
                    # if math.hypot(local_x, local_y) > MAX_DETECTED_OBJECT_DISTANCE_M:
                    #     continue

                    try:
                        pose = robot_infos.get("pose", {})
                        if not isinstance(pose, dict):
                            continue
                        robot_x = float(pose.get("x", 0.0))
                        robot_y = float(pose.get("y", 0.0))
                        robot_theta = float(pose.get("theta", 0.0))
                        global_theta = math.pi / 2 - robot_theta
                        global_x = robot_x + local_x * math.cos(global_theta) - local_y * math.sin(global_theta)
                        global_y = robot_y + local_x * math.sin(global_theta) + local_y * math.cos(global_theta)
                    except (KeyError, TypeError, ValueError):
                        continue

                    self.logger.log(
                        float(timestamp_ms or 0.0),
                        f"detected_object class={class_name} "
                        f"local=({local_x:.3f}, {local_y:.3f}) "
                        f"global=({global_x:.3f}, {global_y:.3f}) "
                        f"robot=({robot_x:.3f}, {robot_y:.3f}, theta={robot_theta:.3f})",
                    )
                    self.publish_detected_object_debug(
                        float(timestamp_ms or 0.0),
                        class_name,
                        local_x,
                        local_y,
                        global_x,
                        global_y,
                        robot_x,
                        robot_y,
                        robot_theta,
                    )

                    if self.is_same_detected_object(class_name, global_x, global_y):
                        continue

                    stored_object = dict(detected_object)
                    stored_object["className"] = class_name
                    stored_object["global"] = {"x": global_x, "y": global_y}
                    stored_object["timestampMs"] = timestamp_ms
                    self.list_detected_object.append(stored_object)
                    added_count += 1

        self.logger.log(
            float(payload.get("timestampMs", 0.0)),
            f"recv detected_objects count={len(objects)} added={added_count} total={len(self.list_detected_object)}",
        )

    def is_same_detected_object(self, class_name: str, global_x: float, global_y: float) -> bool:
        for stored_object in self.list_detected_object:
            if stored_object.get("className") != class_name:
                continue

            global_coord = stored_object.get("global")
            if not isinstance(global_coord, dict):
                continue

            try:
                dx = float(global_coord["x"]) - global_x
                dy = float(global_coord["y"]) - global_y
            except (KeyError, TypeError, ValueError):
                continue

            if math.hypot(dx, dy) <= DETECTED_OBJECT_MERGE_RADIUS_M:
                return True

        return False

    def compute_disabled_vertices(self) -> Set[int]:
        assert self.graph is not None

        if not self.list_detected_object:
            return set()

        disabled: Set[int] = set()
        for detected_object in self.list_detected_object:
            if not isinstance(detected_object, dict):
                continue

            global_coord = detected_object.get("global")
            if not isinstance(global_coord, dict):
                continue

            try:
                global_x = float(global_coord["x"])
                global_y = float(global_coord["y"])
            except (KeyError, TypeError, ValueError):
                continue

            for vertex in self.graph["vertices"]:
                dx = float(vertex["x"]) - global_x
                dy = float(vertex["y"]) - global_y
                if math.hypot(dx, dy) <= self.detection_disable_radius_m:
                    disabled.add(int(vertex["id"]))

        return disabled

    def current_active_vertices(self) -> Set[int]:
        if self.graph is None:
            return set()
        all_vertices = {int(vertex["id"]) for vertex in self.graph["vertices"]}
        return all_vertices - self.disabled_vertices

    def reset_active_vertices(self) -> None:
        self.disabled_vertices.clear()
        self.active_vertices = self.current_active_vertices()

    def update_disabled_vertices(self) -> None:
        self.disabled_vertices |= self.compute_disabled_vertices()
        self.active_vertices = self.current_active_vertices()

    def setup_plot_if_needed(self) -> None:
        if (not self.view and not self.save_dir) or self.graph is None or self._plot_ready:
            return
        try:
            os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib-codex")
            import matplotlib
            backend = self.backend
            if backend is None:
                backend = "TkAgg" if self.view else "Agg"
            matplotlib.use(backend)
            import matplotlib.pyplot as plt
        except ImportError as exc:
            raise RuntimeError(
                "matplotlib is required for --view mode. Install with: pip install matplotlib"
            ) from exc

        self._plt = plt
        self._plt.ion()
        self._fig, self._ax = self._plt.subplots(figsize=(7, 4))
        self._ax.set_title(f"VLA Graph View - Robot {self.robot_id}")
        self._ax.set_xlabel("x")
        self._ax.set_ylabel("y")
        self._ax.set_aspect("equal", adjustable="box")
        self._ax.grid(True, alpha=0.3)

        self._vertex_order = []
        self._vertex_positions = {}

        for edge in self.graph["edges"]:
            v0 = self._find_vertex(edge["v0"])
            v1 = self._find_vertex(edge["v1"])
            if v0 is None or v1 is None:
                continue
            self._ax.plot(
                [v0["x"], v1["x"]],
                [v0["y"], v1["y"]],
                color="lightgray",
                linewidth=1.5,
                zorder=1,
            )

        xs: List[float] = []
        ys: List[float] = []
        for vertex in self.graph["vertices"]:
            vid = int(vertex["id"])
            x = float(vertex["x"])
            y = float(vertex["y"])
            self._vertex_order.append(vid)
            self._vertex_positions[vid] = (x, y)
            xs.append(x)
            ys.append(y)
            self._ax.text(x, y + 0.08, str(vid), fontsize=8, ha="center", va="bottom")

        colors = self._build_vertex_colors()
        self._scatter = self._ax.scatter(xs, ys, c=colors, s=80, edgecolors="black", zorder=2)
        self._plot_ready = True
        if self.view:
            self._plt.show(block=False)
        self._fig.canvas.draw_idle()
        self._fig.canvas.flush_events()

    def update_plot(self) -> None:
        if not self.view:
            return
        self.setup_plot_if_needed()
        if not self._plot_ready or self._scatter is None or self._fig is None:
            return
        self._scatter.set_color(self._build_vertex_colors())
        title_state = (
            f"state={self.state}, enabled={self.enabled}, "
            f"disabled={sorted(self.disabled_vertices)}, active={sorted(self.current_active_vertices())}"
        )
        self._ax.set_title(f"VLA Graph View - Robot {self.robot_id}\n{title_state}")
        self._fig.canvas.draw_idle()
        self._fig.canvas.flush_events()
        self._plt.pause(0.001)
        self.save_frame_if_needed()

    def save_frame_if_needed(self) -> None:
        if not self.save_dir or not self._plot_ready or self._fig is None:
            return
        os.makedirs(self.save_dir, exist_ok=True)
        output_path = os.path.join(self.save_dir, f"frame_{self._frame_counter:04d}.png")
        self._fig.savefig(output_path, dpi=120, bbox_inches="tight")
        self._frame_counter += 1

    def record_history(self, sim_time_s: float) -> None:
        self._history.append(
            {
                "time": sim_time_s,
                "state": self.state,
                "enabled": self.enabled,
                "disabled_vertices": sorted(self.disabled_vertices),
                "active_vertices": sorted(self.current_active_vertices()),
            }
        )

    def show_history_summary(self) -> None:
        if not self.view or self.graph is None or not self._history:
            return
        self.setup_plot_if_needed()
        if self._plt is None:
            return

        cols = 3
        rows = max(1, math.ceil(len(self._history) / cols))
        fig, axes = self._plt.subplots(rows, cols, figsize=(cols * 4.5, rows * 3.5))
        axes_list = axes.flat if hasattr(axes, "flat") else [axes]

        for ax, snapshot in zip(axes_list, self._history):
            self._draw_snapshot(ax, snapshot)

        for ax in list(axes_list)[len(self._history):]:
            ax.axis("off")

        fig.suptitle(f"VLA Vertex History Summary - Robot {self.robot_id}")
        fig.tight_layout()
        self._plt.ioff()
        self._plt.show()

    def _draw_snapshot(self, ax: Any, snapshot: Dict[str, Any]) -> None:
        active = set(snapshot["active_vertices"])
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, alpha=0.3)
        ax.set_xlabel("x")
        ax.set_ylabel("y")

        for edge in self.graph["edges"]:
            v0 = self._find_vertex(edge["v0"])
            v1 = self._find_vertex(edge["v1"])
            if v0 is None or v1 is None:
                continue
            ax.plot(
                [v0["x"], v1["x"]],
                [v0["y"], v1["y"]],
                color="lightgray",
                linewidth=1.5,
                zorder=1,
            )

        xs: List[float] = []
        ys: List[float] = []
        colors: List[str] = []
        for vertex in self.graph["vertices"]:
            vid = int(vertex["id"])
            x = float(vertex["x"])
            y = float(vertex["y"])
            xs.append(x)
            ys.append(y)
            colors.append("steelblue" if vid in active else "tomato")
            ax.text(x, y + 0.08, str(vid), fontsize=7, ha="center", va="bottom")

        ax.scatter(xs, ys, c=colors, s=70, edgecolors="black", zorder=2)
        ax.set_title(
            f"t={snapshot['time']:.1f}s\nstate={snapshot['state']} enabled={snapshot['enabled']}\nactive={snapshot['active_vertices']}",
            fontsize=9,
        )

    def _build_vertex_colors(self) -> List[str]:
        colors: List[str] = []
        for vid in self._vertex_order:
            if vid in self.current_active_vertices():
                colors.append("steelblue")
            else:
                colors.append("tomato")
        return colors

    def _find_vertex(self, vertex_id: int) -> Optional[Dict[str, Any]]:
        if self.graph is None:
            return None
        for vertex in self.graph["vertices"]:
            if int(vertex["id"]) == int(vertex_id):
                return vertex
        return None


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the VLA reference node on a real MQTT broker.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=1883)
    parser.add_argument("--robot-id", type=int, default=1)
    parser.add_argument("--tick", type=float, default=0.2)
    parser.add_argument("--view", action="store_true", help="Enable matplotlib graph visualization.")
    parser.add_argument("--save-dir", default=None, help="Optional directory to save visualization frames as PNG.")
    parser.add_argument("--backend", default=None, help="Optional matplotlib backend override, e.g. TkAgg or Agg.")
    parser.add_argument("--detection-class", action="append", dest="detection_classes", default=None)
    parser.add_argument("--detection-disable-radius", type=float, default=DEFAULT_DETECTION_DISABLE_RADIUS_M)
    args = parser.parse_args()

    logger = Logger("vla-ref")
    transport = PahoMqttTransport(
        host=args.host,
        port=args.port,
        client_id=f"vla-ref-{args.robot_id}",
        logger=logger,
    )
    node = VlaReferenceNode(
        transport,
        args.robot_id,
        logger,
        view=args.view,
        save_dir=args.save_dir,
        backend=args.backend,
        detection_classes=args.detection_classes,
        detection_disable_radius_m=args.detection_disable_radius,
    )
    transport.start()
    node.start(0.0)

    start_wall = time.monotonic()
    try:
        while True:
            sim_time_s = time.monotonic() - start_wall
            node.tick(sim_time_s)
            time.sleep(args.tick)
    except KeyboardInterrupt:
        logger.log(time.monotonic() - start_wall, "shutdown requested")
    finally:
        transport.stop()


if __name__ == "__main__":
    main()

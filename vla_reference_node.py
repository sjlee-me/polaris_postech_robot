#!/usr/bin/env python3
"""Reference VLA node for protocol-first development."""

from __future__ import annotations

import argparse
import math
import os
import time
from typing import Any, Dict, List, Optional, Set, Tuple

from mqtt_protocol_sim import Logger, PahoMqttTransport, TopicTransport, robot_topic


class VlaReferenceNode:
    def __init__(
        self,
        bus: TopicTransport,
        robot_id: int,
        logger: Logger,
        view: bool = False,
        save_dir: Optional[str] = None,
        backend: Optional[str] = None,
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
        self.disabled_vertices: Set[int] = set()
        self.active_vertices: Set[int] = set()
        self._graph_request_interval_s = 3.0
        self._last_graph_request_s = -1.0
        self._last_status_pub_s = -1.0
        self._last_vertex_pub_s = -1.0
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

    def start(self, sim_time_s: float) -> None:
        self.state = "idle"
        self.request_graph(sim_time_s)

    def tick(self, sim_time_s: float) -> None:
        if self.graph is None and (
            self._last_graph_request_s < 0.0
            or sim_time_s - self._last_graph_request_s >= self._graph_request_interval_s
        ):
            self.request_graph(sim_time_s)

        if sim_time_s - self._last_status_pub_s >= 1.0:
            self.publish_status(sim_time_s)
            self._last_status_pub_s = sim_time_s

        if (
            self.enabled
            and self.graph is not None
            and self.latest_pose is not None
            and self.mission_id
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
        self.reset_active_vertices()
        self.state = "idle"
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
        if self.enabled and self.graph is not None:
            self.state = "working"

    def on_init_vertex(self, topic: str, payload: Dict[str, Any]) -> None:
        del topic
        graph_name = payload.get("graphName")
        if graph_name and self.latest_graph_name and graph_name != self.latest_graph_name:
            self.logger.log(float(payload["timestampMs"]), f"ignore init_vertex graphName={graph_name}")
            return
        self.reset_active_vertices()
        self.logger.log(float(payload["timestampMs"]), f"init_vertex graphName={graph_name}")
        self.update_plot()

    def on_vla_enable(self, topic: str, payload: Dict[str, Any]) -> None:
        del topic
        self.enabled = bool(payload.get("enable", False))
        self.mission_id = payload.get("missionId", self.mission_id)
        self.state = "working" if self.enabled and self.graph is not None else "idle"
        self.update_plot()

    def compute_disabled_vertices(self) -> Set[int]:
        assert self.graph is not None
        assert self.latest_pose is not None

        pose_x = self.latest_pose["x"]
        pose_y = self.latest_pose["y"]
        theta = self.latest_pose["theta"]
        disabled: Set[int] = set()

        for vertex in self.graph["vertices"]:
            dx = float(vertex["x"]) - pose_x
            dy = float(vertex["y"]) - pose_y
            distance = math.hypot(dx, dy)
            heading_error = abs(math.atan2(dy, dx) - theta)
            heading_error = min(heading_error, abs(2 * math.pi - heading_error))
            if 1.2 <= distance <= 2.2 and heading_error <= 0.75:
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

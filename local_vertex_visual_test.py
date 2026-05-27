#!/usr/bin/env python3
"""Reference graph viewer and local-pose-to-vertex visual test."""

from __future__ import annotations

import argparse
import math
import os
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib-codex")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

from coordinate_transform_reference import RobotPose2D, local_to_global_xy
from mqtt_protocol_sim import sample_graph_payload


@dataclass
class VisualCase:
    robot_vertex_id: int
    local_x: float
    local_y: float
    radius: float


class LocalVertexVisualTest:
    def __init__(self, graph: Dict[str, Any]) -> None:
        self.graph = graph
        self.vertex_by_id = {int(v["id"]): v for v in graph["vertices"]}

    def get_vertex(self, vertex_id: int) -> Dict[str, Any]:
        if vertex_id not in self.vertex_by_id:
            raise ValueError(f"unknown vertex_id={vertex_id}")
        return self.vertex_by_id[vertex_id]

    def get_neighbors(self, vertex_id: int) -> List[int]:
        neighbors: List[int] = []
        for edge in self.graph["edges"]:
            v0 = int(edge["v0"])
            v1 = int(edge["v1"])
            if v0 == vertex_id:
                neighbors.append(v1)
            elif v1 == vertex_id:
                neighbors.append(v0)
        return sorted(set(neighbors))

    def choose_forward_neighbor(self, vertex_id: int) -> int:
        neighbors = self.get_neighbors(vertex_id)
        if not neighbors:
            raise ValueError(f"vertex_id={vertex_id} has no neighbor")

        larger = [n for n in neighbors if n > vertex_id]
        if larger:
            return min(larger)
        return neighbors[0]

    def compute_pose_from_vertex(self, vertex_id: int) -> Tuple[float, float, float, int]:
        vertex = self.get_vertex(vertex_id)
        next_vertex_id = self.choose_forward_neighbor(vertex_id)
        next_vertex = self.get_vertex(next_vertex_id)
        theta = math.atan2(
            float(next_vertex["y"]) - float(vertex["y"]),
            float(next_vertex["x"]) - float(vertex["x"]),
        )
        return float(vertex["x"]), float(vertex["y"]), theta, next_vertex_id

    def affected_vertices(self, global_x: float, global_y: float, radius: float) -> List[int]:
        affected: List[int] = []
        for vertex in self.graph["vertices"]:
            dx = float(vertex["x"]) - global_x
            dy = float(vertex["y"]) - global_y
            if math.hypot(dx, dy) <= radius:
                affected.append(int(vertex["id"]))
        return sorted(affected)

    def build_case_result(self, case: VisualCase) -> Dict[str, Any]:
        robot_x, robot_y, theta, next_vertex_id = self.compute_pose_from_vertex(case.robot_vertex_id)
        global_x, global_y = local_to_global_xy(
            RobotPose2D(x=robot_x, y=robot_y, theta=theta),
            case.local_x,
            case.local_y,
        )
        affected = self.affected_vertices(global_x, global_y, case.radius)
        return {
            "case": case,
            "robot_x": robot_x,
            "robot_y": robot_y,
            "theta": theta,
            "next_vertex_id": next_vertex_id,
            "global_x": global_x,
            "global_y": global_y,
            "affected_vertices": affected,
        }

    def show_reference_graph(self, save_path: Optional[str]) -> None:
        fig, ax = plt.subplots(figsize=(8, 4.8))
        self._draw_base_graph(ax)
        ax.set_title("Reference Graph\nChoose robot_vertex_id from this graph")
        fig.tight_layout()
        if save_path:
            fig.savefig(save_path, dpi=150, bbox_inches="tight")
            print(f"saved reference graph: {save_path}")
            plt.close(fig)
            return
        plt.show()

    def show_case_result(self, result: Dict[str, Any], save_path: Optional[str]) -> None:
        fig, ax = plt.subplots(figsize=(8.5, 5.5))
        self._draw_case(ax, result)
        fig.tight_layout()
        if save_path:
            fig.savefig(save_path, dpi=150, bbox_inches="tight")
            print(f"saved result graph: {save_path}")
            plt.close(fig)
            return
        plt.show()

    def _draw_base_graph(self, ax: Any) -> None:
        for edge in self.graph["edges"]:
            v0 = self.get_vertex(int(edge["v0"]))
            v1 = self.get_vertex(int(edge["v1"]))
            ax.plot(
                [float(v0["x"]), float(v1["x"])],
                [float(v0["y"]), float(v1["y"])],
                color="lightgray",
                linewidth=2,
                zorder=1,
            )

        xs: List[float] = []
        ys: List[float] = []
        for vertex in self.graph["vertices"]:
            x = float(vertex["x"])
            y = float(vertex["y"])
            xs.append(x)
            ys.append(y)
            ax.text(x, y + 0.08, str(int(vertex["id"])), fontsize=9, ha="center", va="bottom")

        ax.scatter(xs, ys, c="steelblue", s=100, edgecolors="black", zorder=2)
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, alpha=0.25)
        ax.set_xlabel("map x")
        ax.set_ylabel("map y")

    def _draw_case(self, ax: Any, result: Dict[str, Any]) -> None:
        self._draw_base_graph(ax)

        case: VisualCase = result["case"]
        affected = set(result["affected_vertices"])

        xs: List[float] = []
        ys: List[float] = []
        colors: List[str] = []
        sizes: List[int] = []
        for vertex in self.graph["vertices"]:
            vid = int(vertex["id"])
            x = float(vertex["x"])
            y = float(vertex["y"])
            xs.append(x)
            ys.append(y)
            if vid == case.robot_vertex_id:
                colors.append("gold")
                sizes.append(170)
            elif vid in affected:
                colors.append("tomato")
                sizes.append(130)
            else:
                colors.append("steelblue")
                sizes.append(90)

        ax.scatter(xs, ys, c=colors, s=sizes, edgecolors="black", zorder=3)

        robot_x = result["robot_x"]
        robot_y = result["robot_y"]
        theta = result["theta"]
        global_x = result["global_x"]
        global_y = result["global_y"]

        ax.arrow(
            robot_x,
            robot_y,
            0.65 * math.cos(theta),
            0.65 * math.sin(theta),
            width=0.03,
            head_width=0.18,
            head_length=0.18,
            color="darkorange",
            zorder=4,
            length_includes_head=True,
        )
        ax.text(robot_x, robot_y - 0.18, "robot", fontsize=9, ha="center", va="top", color="darkorange")

        ax.plot([robot_x, global_x], [robot_y, global_y], linestyle="--", color="purple", linewidth=1.8, zorder=4)
        ax.scatter([global_x], [global_y], c="magenta", s=120, marker="x", linewidths=2.2, zorder=5)
        ax.text(global_x, global_y - 0.18, "global(local x,y)", fontsize=9, ha="center", va="top", color="magenta")

        radius_circle = Circle((global_x, global_y), case.radius, color="magenta", fill=False, linewidth=2, alpha=0.7)
        ax.add_patch(radius_circle)

        inset = ax.inset_axes([0.68, 0.08, 0.28, 0.28])
        inset.axhline(0, color="gray", linewidth=1)
        inset.axvline(0, color="gray", linewidth=1)
        inset.arrow(
            0.0,
            0.0,
            case.local_x,
            case.local_y,
            width=0.02,
            head_width=0.10,
            head_length=0.10,
            color="purple",
            length_includes_head=True,
        )
        inset.scatter([0.0], [0.0], c="gold", s=60, edgecolors="black")
        inset.scatter([case.local_x], [case.local_y], c="magenta", s=70, marker="x", linewidths=2.0)
        inset_circle = Circle((case.local_x, case.local_y), case.radius, color="magenta", fill=False, linewidth=1.5, alpha=0.6)
        inset.add_patch(inset_circle)
        inset.set_title("robot local frame", fontsize=8)
        inset.set_xlim(min(-0.5, case.local_x - case.radius - 0.2), max(1.5, case.local_x + case.radius + 0.2))
        inset.set_ylim(min(-1.5, case.local_y - case.radius - 0.2), max(1.5, case.local_y + case.radius + 0.2))
        inset.grid(True, alpha=0.2)
        inset.tick_params(labelsize=7)

        ax.set_title(
            "\n".join(
                [
                    f"robot_vertex={case.robot_vertex_id} -> facing {result['next_vertex_id']}",
                    f"local=({case.local_x:.2f}, {case.local_y:.2f}), radius={case.radius:.2f}",
                    f"global=({global_x:.2f}, {global_y:.2f}), affected={result['affected_vertices']}",
                ]
            ),
            fontsize=10,
        )


def prompt_int(label: str, default: int) -> int:
    raw = input(f"{label} [{default}]: ").strip()
    if not raw:
        return default
    return int(raw)


def prompt_float(label: str, default: float) -> float:
    raw = input(f"{label} [{default}]: ").strip()
    if not raw:
        return default
    return float(raw)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Visualize which graph vertices are affected by a local pose input.")
    parser.add_argument("--show-reference", action="store_true", help="Show the reference graph and exit.")
    parser.add_argument("--save-reference", default=None, help="Optional PNG path for the reference graph.")
    parser.add_argument("--save-result", default=None, help="Optional PNG path for the result graph.")
    parser.add_argument("--robot-vertex", type=int, default=None, help="Robot current vertex id.")
    parser.add_argument("--local-x", type=float, default=None, help="Detected local x in robot frame.")
    parser.add_argument("--local-y", type=float, default=None, help="Detected local y in robot frame.")
    parser.add_argument("--radius", type=float, default=1.5, help="Disable radius in meters.")
    parser.add_argument("--interactive", action="store_true", help="Prompt for robot vertex and local pose after showing the reference graph.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    tester = LocalVertexVisualTest(sample_graph_payload())

    if args.show_reference:
        tester.show_reference_graph(save_path=args.save_reference)
        return

    if args.interactive or args.robot_vertex is None or args.local_x is None or args.local_y is None:
        tester.show_reference_graph(save_path=args.save_reference)
        robot_vertex_id = prompt_int("robot_vertex_id", 103)
        local_x = prompt_float("local_x", 1.0)
        local_y = prompt_float("local_y", 0.0)
        radius = prompt_float("radius", args.radius)
    else:
        robot_vertex_id = args.robot_vertex
        local_x = args.local_x
        local_y = args.local_y
        radius = args.radius

    case = VisualCase(
        robot_vertex_id=robot_vertex_id,
        local_x=local_x,
        local_y=local_y,
        radius=radius,
    )
    result = tester.build_case_result(case)
    print(
        f"robot_vertex={case.robot_vertex_id}, facing={result['next_vertex_id']}, "
        f"local=({case.local_x:.2f}, {case.local_y:.2f}), "
        f"global=({result['global_x']:.2f}, {result['global_y']:.2f}), "
        f"affected={result['affected_vertices']}"
    )
    tester.show_case_result(result, save_path=args.save_result)


if __name__ == "__main__":
    main()

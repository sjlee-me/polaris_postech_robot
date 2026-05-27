#!/usr/bin/env python3
"""Dataset examples for local-to-global coordinate conversion."""

from __future__ import annotations

import json
import math
from typing import Any, Dict, List

from coordinate_transform_reference import local_to_global_from_robot_infos


def dataset_cases() -> List[Dict[str, Any]]:
    return [
        {
            "name": "forward_on_x_axis",
            "robot_infos": {
                "robotId": 1,
                "timestampMs": 0.0,
                "missionId": "",
                "pose": {"x": 2.0, "y": 3.0, "theta": 0.0},
            },
            "local": {"x": 1.0, "y": 0.0},
        },
        {
            "name": "left_of_robot_on_x_axis",
            "robot_infos": {
                "robotId": 1,
                "timestampMs": 0.0,
                "missionId": "",
                "pose": {"x": 2.0, "y": 3.0, "theta": 0.0},
            },
            "local": {"x": 0.0, "y": 1.0},
        },
        {
            "name": "forward_on_y_axis",
            "robot_infos": {
                "robotId": 1,
                "timestampMs": 0.0,
                "missionId": "",
                "pose": {"x": 2.0, "y": 3.0, "theta": math.pi / 2.0},
            },
            "local": {"x": 1.0, "y": 0.0},
        },
        {
            "name": "diagonal_heading_45deg",
            "robot_infos": {
                "robotId": 1,
                "timestampMs": 0.0,
                "missionId": "",
                "pose": {"x": 1.0, "y": 1.0, "theta": math.pi / 4.0},
            },
            "local": {"x": 1.0, "y": 1.0},
        },
    ]


def main() -> None:
    for case in dataset_cases():
        global_x, global_y = local_to_global_from_robot_infos(
            case["robot_infos"],
            float(case["local"]["x"]),
            float(case["local"]["y"]),
        )
        result = {
            "name": case["name"],
            "robot_pose": case["robot_infos"]["pose"],
            "local_xy": case["local"],
            "global_xy": {"x": round(global_x, 6), "y": round(global_y, 6)},
        }
        print(json.dumps(result, ensure_ascii=True))


if __name__ == "__main__":
    main()

### 출력 예시
# {"name": "forward_on_x_axis", "robot_pose": {"x": 2.0, "y": 3.0, "theta": 0.0}, "local_xy": {"x": 1.0, "y": 0.0}, "global_xy": {"x": 3.0, "y": 3.0}}
# {"name": "left_of_robot_on_x_axis", "robot_pose": {"x": 2.0, "y": 3.0, "theta": 0.0}, "local_xy": {"x": 0.0, "y": 1.0}, "global_xy": {"x": 2.0, "y": 4.0}}
# {"name": "forward_on_y_axis", "robot_pose": {"x": 2.0, "y": 3.0, "theta": 1.5707963267948966}, "local_xy": {"x": 1.0, "y": 0.0}, "global_xy": {"x": 2.0, "y": 4.0}}
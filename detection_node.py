#!/usr/bin/env python3
"""Detection node for MQTT protocol testing."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from mqtt_protocol_sim import Logger, PahoMqttTransport, TopicTransport, robot_topic


IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
DEFAULT_CALIBRATION_PATH = Path(__file__).resolve().with_name("calibration.json")
DEFAULT_GROUND_PATH = Path(__file__).resolve().with_name("ground.json")
LOCAL_COORD_PROJECTOR: Optional["GroundPlaneProjector"] = None


def dot(a: List[float], b: List[float]) -> float:
    return sum(x * y for x, y in zip(a, b))


def mat_vec(matrix: List[List[float]], vector: List[float]) -> List[float]:
    return [dot(row, vector) for row in matrix]


def transpose(matrix: List[List[float]]) -> List[List[float]]:
    return [list(row) for row in zip(*matrix)]


def rodrigues_to_matrix(rvec: List[float]) -> List[List[float]]:
    theta = math.sqrt(dot(rvec, rvec))
    if theta <= 1e-12:
        return [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ]

    kx, ky, kz = [value / theta for value in rvec]
    c = math.cos(theta)
    s = math.sin(theta)
    one_c = 1.0 - c
    return [
        [c + kx * kx * one_c, kx * ky * one_c - kz * s, kx * kz * one_c + ky * s],
        [ky * kx * one_c + kz * s, c + ky * ky * one_c, ky * kz * one_c - kx * s],
        [kz * kx * one_c - ky * s, kz * ky * one_c + kx * s, c + kz * kz * one_c],
    ]


# NOTE: 지금 구현은, bounding box 의 하단 변의 중앙 지점의 local 좌표를 반환함.
# 필요에 따라, 하단 변 전체의 좌표를 반환해서 물체의 크기를 추정하도록 할 수 있음.
class GroundPlaneProjector:
    """Project image pixels onto the calibrated ground plane.

    The calibration files use millimeter-scale world coordinates. Returned
    local coordinates are converted to meters to match the protocol examples.
    """

    def __init__(
        self,
        camera_matrix: List[List[float]],
        distortion: List[float],
        rvec: List[float],
        tvec: List[float],
        origin_w: List[float],
        x_axis_w: List[float],
        y_axis_w: List[float],
        world_to_meter_scale: float = 0.001,
    ) -> None:
        self.camera_matrix = camera_matrix
        self.distortion = distortion
        self.rotation = rodrigues_to_matrix(rvec)
        self.rotation_t = transpose(self.rotation)
        self.tvec = tvec
        self.origin_w = origin_w
        self.x_axis_w = x_axis_w
        self.y_axis_w = y_axis_w
        self.world_to_meter_scale = world_to_meter_scale

    @classmethod
    def from_paths(
        cls,
        calibration_path: Path,
        ground_path: Path,
        world_to_meter_scale: float = 0.001,
    ) -> "GroundPlaneProjector":
        with calibration_path.open("r", encoding="utf-8") as file:
            calibration = json.load(file)
        with ground_path.open("r", encoding="utf-8") as file:
            ground = json.load(file)

        distortion_data = calibration.get("distortion_coefficients", {})
        distortion = distortion_data.get("array", [[0.0, 0.0, 0.0, 0.0, 0.0]])[0]
        return cls(
            camera_matrix=[[float(value) for value in row] for row in calibration["camera_matrix"]],
            distortion=[float(value) for value in distortion],
            rvec=[float(value) for value in ground["rvec"]],
            tvec=[float(value) for value in ground["tvec"]],
            origin_w=[float(value) for value in ground["origin_w"]],
            x_axis_w=[float(value) for value in ground["x_axis_w"]],
            y_axis_w=[float(value) for value in ground["y_axis_w"]],
            world_to_meter_scale=world_to_meter_scale,
        )
    
    def project_pixel_to_local(self, pixel_x: float, pixel_y: float) -> Optional[Dict[str, float]]:
        fx = self.camera_matrix[0][0]
        fy = self.camera_matrix[1][1]
        cx = self.camera_matrix[0][2]
        cy = self.camera_matrix[1][2]
        distorted_x = (pixel_x - cx) / fx
        distorted_y = (pixel_y - cy) / fy
        undistorted_x, undistorted_y = self.undistort_normalized(distorted_x, distorted_y)

        ray_camera = [undistorted_x, undistorted_y, 1.0]
        ray_world = mat_vec(self.rotation_t, ray_camera)
        camera_offset_world = mat_vec(self.rotation_t, self.tvec)

        if abs(ray_world[2]) <= 1e-12:
            return None

        scale = camera_offset_world[2] / ray_world[2]
        if scale <= 0.0:
            return None

        ground_point_w = [
            scale * ray_world[index] - camera_offset_world[index]
            for index in range(3)
        ]
        rel = [
            ground_point_w[index] - self.origin_w[index]
            for index in range(3)
        ]
        ground_x = dot(rel, self.x_axis_w) * self.world_to_meter_scale
        ground_y = dot(rel, self.y_axis_w) * self.world_to_meter_scale

        # Protocol convention: local x is robot-forward and local y is robot-left.
        # In the current ground calibration, +ground_x points robot-right and
        # +ground_y points robot-backward, so flip and swap into robot local axes.
        local_x = -ground_y
        local_y = -ground_x
        return {"x": float(local_x), "y": float(local_y)}

    def undistort_normalized(self, distorted_x: float, distorted_y: float) -> Tuple[float, float]:
        k1, k2, p1, p2, k3 = (self.distortion + [0.0] * 5)[:5]
        x = distorted_x
        y = distorted_y
        for _ in range(8):
            r2 = x * x + y * y
            radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
            if abs(radial) <= 1e-12:
                break
            delta_x = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
            delta_y = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
            x = (distorted_x - delta_x) / radial
            y = (distorted_y - delta_y) / radial
        return x, y


def translate_bbox_to_local_coord(
    bbox: Dict[str, float],
    image_size: Dict[str, int],
    class_name: str,
    confidence: float,
    robot_infos: Dict[str, Any],
) -> Optional[Dict[str, float]]:
    """Convert a detected bbox into a robot-local coordinate."""

    del image_size, class_name, confidence, robot_infos
    if LOCAL_COORD_PROJECTOR is None:
        return None

    pixel_x = float(bbox["cx"])
    pixel_y = float(bbox["y2"])
    return LOCAL_COORD_PROJECTOR.project_pixel_to_local(pixel_x, pixel_y)


class DetectionNode:
    def __init__(
        self,
        bus: TopicTransport,
        robot_id: int,
        logger: Logger,
        image_dir: Path,
        weights: str,
        conf: float,
        status_interval_s: float,
    ) -> None:
        self.bus = bus
        self.robot_id = robot_id
        self.logger = logger
        self.image_dir = image_dir
        self.weights = weights
        self.conf = conf
        self.status_interval_s = status_interval_s
        self.enabled = False
        self.state = "initializing"
        self.mission_id: Optional[str] = None
        self.model_loaded = False
        self.target_classes: List[str] = []
        self.classes_set = False
        self.latest_robot_infos: Optional[Dict[str, Any]] = None
        self.latest_image_path: Optional[str] = None
        self._model: Any = None
        self._last_status_pub_s = -1.0
        self._last_image_key: Optional[Tuple[str, int]] = None

        bus.subscribe(robot_topic(robot_id, "detection_config"), self.on_detection_config)
        bus.subscribe(robot_topic(robot_id, "detection_enable"), self.on_detection_enable)
        bus.subscribe(robot_topic(robot_id, "robot_infos"), self.on_robot_infos)

    def start(self, sim_time_s: float) -> None:
        self.load_model(sim_time_s)
        self.update_state()

    def load_model(self, sim_time_s: float) -> None:
        try:
            from ultralytics import YOLOWorld  # type: ignore
        except ImportError as exc:
            raise RuntimeError(
                "ultralytics is required for detection mode. Install with: pip install ultralytics"
            ) from exc

        self.logger.log(sim_time_s, f"load YOLOWorld weights={self.weights}")
        self._model = YOLOWorld(self.weights)
        self.model_loaded = True
        if self.target_classes:
            self.set_model_classes(sim_time_s)
        self.logger.log(sim_time_s, "YOLOWorld model ready")

    def tick(self, sim_time_s: float) -> None:
        self.update_state()

        if sim_time_s - self._last_status_pub_s >= self.status_interval_s:
            self.publish_status(sim_time_s)
            self._last_status_pub_s = sim_time_s

        if self.state == "working":
            self.process_latest_image(sim_time_s)

    def update_state(self) -> None:
        if not self.model_loaded or not self.target_classes or not self.classes_set:
            self.state = "initializing"
        elif self.enabled:
            self.state = "working" if self.latest_robot_infos is not None else "initializing"
        else:
            self.state = "idle"

    def publish_status(self, sim_time_s: float) -> None:
        payload = {
            "robotId": self.robot_id,
            "timestampMs": float(sim_time_s),
            "missionId": self.mission_id,
            "enable": self.enabled,
            "state": self.state,
            "classes": list(self.target_classes),
            "classesSet": self.classes_set,
        }
        self.bus.publish(robot_topic(self.robot_id, "detection_status"), payload)
        self.logger.log(sim_time_s, f"publish detection_status enable={self.enabled} state={self.state}")

    def on_detection_config(self, topic: str, payload: Dict[str, Any]) -> None:
        del topic
        sim_time_s = float(payload.get("timestampMs", 0.0))
        classes = payload.get("classes", [])
        if not isinstance(classes, list):
            self.target_classes = []
            self.classes_set = False
            self.update_state()
            self.logger.log(sim_time_s, "ignore detection_config: classes is not a list")
            return

        target_classes = [str(item).strip() for item in classes if str(item).strip()]
        if not target_classes:
            self.target_classes = []
            self.classes_set = False
            self.update_state()
            self.logger.log(sim_time_s, "ignore detection_config: empty classes")
            return

        if target_classes != self.target_classes:
            self.target_classes = target_classes
            self.classes_set = False
            self._last_image_key = None

        if self.model_loaded:
            self.set_model_classes(sim_time_s)

        self.update_state()
        self.logger.log(sim_time_s, f"update detection_config classes={self.target_classes} classesSet={self.classes_set}")

    def set_model_classes(self, sim_time_s: float) -> None:
        if self._model is None or not self.target_classes:
            self.classes_set = False
            return

        self._model.set_classes(self.target_classes)
        self.classes_set = self.model_classes_match()
        self.logger.log(
            sim_time_s,
            f"set detection classes requested={self.target_classes} modelNames={self.model_class_names()} classesSet={self.classes_set}",
        )

    def model_class_names(self) -> List[str]:
        if self._model is None:
            return []
        names = getattr(self._model, "names", None) # _model.names = {0: 'fire extinguisher'}
        if isinstance(names, dict):
            return [str(names[key]) for key in sorted(names)]
        if isinstance(names, list):
            return [str(item) for item in names]
        return []

    def model_classes_match(self) -> bool:
        return self.model_class_names() == self.target_classes

    def on_detection_enable(self, topic: str, payload: Dict[str, Any]) -> None:
        del topic
        sim_time_s = float(payload.get("timestampMs", 0.0))
        self.enabled = bool(payload.get("enable", False))
        self.mission_id = payload.get("missionId", self.mission_id)
        self._last_image_key = None
        self.update_state()
        self.logger.log(sim_time_s, f"update detection_enable enable={self.enabled} missionId={self.mission_id}")

    def on_robot_infos(self, topic: str, payload: Dict[str, Any]) -> None:
        del topic
        self.latest_robot_infos = self.normalize_robot_infos(payload)
        self.update_state()

    def normalize_robot_infos(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        pose = payload.get("pose", {})
        if not isinstance(pose, dict):
            pose = {}

        return {
            "robotId": int(payload.get("robotId", self.robot_id)),
            "timestampMs": float(payload.get("timestampMs", 0.0)),
            "missionId": payload.get("missionId", ""),
            "pose": {
                "x": float(pose.get("x", 0.0)),
                "y": float(pose.get("y", 0.0)),
                "theta": float(pose.get("theta", 0.0)),
            },
            "mapName": payload.get("mapName"),
            "graphName": payload.get("graphName"),
        }
    
    # TODO: 정확히 어떤 이미지를 읽어서 detection 모델 돌릴지 협의 필요
    def process_latest_image(self, sim_time_s: float) -> None:
        latest = self.find_latest_image()
        if latest is None:
            return

        image_path, mtime_ns = latest
        image_key = (str(image_path), mtime_ns)
        if image_key == self._last_image_key:
            return

        robot_infos = self.latest_robot_infos
        if robot_infos is None:
            return

        image_size = self.read_image_size(image_path)
        self.logger.log(sim_time_s, f"predict image={image_path}")
        results = self._model.predict(str(image_path), verbose=False, conf=self.conf)
        objects = self.build_info_detected_objects(results, image_size, robot_infos)

        payload = {
            "robotId": self.robot_id,
            "timestampMs": float(sim_time_s),
            "imagePath": str(image_path.resolve()),
            "imageSize": image_size,
            "robotInfos": robot_infos,
            "classes": list(self.target_classes),
            "objects": objects,
        }
        self.bus.publish(robot_topic(self.robot_id, "detected_objects"), payload)
        self.latest_image_path = str(image_path.resolve())
        self._last_image_key = image_key
        self.logger.log(sim_time_s, f"publish detected_objects count={len(objects)} image={image_path.name}")

    def find_latest_image(self) -> Optional[Tuple[Path, int]]:
        if not self.image_dir.exists():
            return None

        latest_path: Optional[Path] = None
        latest_mtime_ns = -1
        for path in self.image_dir.iterdir():
            if not path.is_file() or path.suffix.lower() not in IMAGE_SUFFIXES:
                continue
            try:
                mtime_ns = path.stat().st_mtime_ns
            except OSError:
                continue
            if mtime_ns > latest_mtime_ns:
                latest_path = path
                latest_mtime_ns = mtime_ns

        if latest_path is None:
            return None
        return latest_path, latest_mtime_ns

    def read_image_size(self, image_path: Path) -> Dict[str, int]:
        try:
            from PIL import Image  # type: ignore
        except ImportError as exc:
            raise RuntimeError("Pillow is required to read image sizes. Install with: pip install pillow") from exc

        with Image.open(image_path) as image:
            width, height = image.size
        return {"width": int(width), "height": int(height)}

    def build_info_detected_objects(
        self,
        results: Any,
        image_size: Dict[str, int],
        robot_infos: Dict[str, Any],
    ) -> List[Dict[str, Any]]:
        objects: List[Dict[str, Any]] = []
        for result in results:
            boxes = getattr(result, "boxes", None)
            if boxes is None or len(boxes) == 0:
                continue

            for box in boxes:
                x1, y1, x2, y2 = [float(value) for value in box.xyxy[0].tolist()]
                confidence = float(box.conf[0])
                class_id = int(box.cls[0])
                class_name = self.class_name_for_result(result, class_id)
                bbox = {
                    "x1": x1,
                    "y1": y1,
                    "x2": x2,
                    "y2": y2,
                    "cx": (x1 + x2) / 2.0,
                    "cy": (y1 + y2) / 2.0,
                    "width": x2 - x1,
                    "height": y2 - y1,
                }
                local = translate_bbox_to_local_coord(
                    bbox=bbox,
                    image_size=image_size,
                    class_name=class_name,
                    confidence=confidence,
                    robot_infos=robot_infos,
                )
                objects.append(
                    {
                        "className": class_name,
                        "confidence": confidence,
                        "bbox": bbox,
                        "local": local,
                    }
                )
        return objects

    def class_name_for_result(self, result: Any, class_id: int) -> str:
        names = getattr(result, "names", None)
        if isinstance(names, dict) and class_id in names:
            return str(names[class_id])
        if isinstance(names, list) and 0 <= class_id < len(names):
            return str(names[class_id])
        if 0 <= class_id < len(self.target_classes):
            return self.target_classes[class_id]
        return str(class_id)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a detection node on a real MQTT broker.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=1883)
    parser.add_argument("--robot-id", type=int, default=1)
    parser.add_argument("--image-dir", default="./camera_images")
    parser.add_argument("--weights", default="yolov8x-worldv2.pt")
    parser.add_argument("--poll-interval", type=float, default=0.2)
    parser.add_argument("--status-interval", type=float, default=1.0)
    parser.add_argument("--conf", type=float, default=0.5)
    parser.add_argument("--calibration", default=str(DEFAULT_CALIBRATION_PATH))
    parser.add_argument("--ground", default=str(DEFAULT_GROUND_PATH))
    return parser.parse_args()


def main() -> None:
    global LOCAL_COORD_PROJECTOR

    args = parse_args()
    logger = Logger("detection-node")
    image_dir = Path(args.image_dir)
    image_dir.mkdir(parents=True, exist_ok=True)
    LOCAL_COORD_PROJECTOR = GroundPlaneProjector.from_paths(
        calibration_path=Path(args.calibration),
        ground_path=Path(args.ground),
    )
    logger.log(0.0, f"load calibration={args.calibration} ground={args.ground}")

    transport = PahoMqttTransport(
        host=args.host,
        port=args.port,
        client_id=f"detection-node-robotid{args.robot_id}",
        logger=logger,
    )
    node = DetectionNode(
        transport,
        args.robot_id,
        logger,
        image_dir=image_dir,
        weights=args.weights,
        conf=args.conf,
        status_interval_s=args.status_interval,
    )

    transport.start()
    start_wall = time.monotonic()
    try:
        node.start(0.0)
        while True:
            sim_time_s = time.monotonic() - start_wall
            node.tick(sim_time_s)
            time.sleep(args.poll_interval)
    except KeyboardInterrupt:
        logger.log(time.monotonic() - start_wall, "shutdown requested")
    finally:
        transport.stop()


if __name__ == "__main__":
    main()

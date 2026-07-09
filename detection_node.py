#!/usr/bin/env python3
"""Detection node for MQTT protocol testing."""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from mqtt_protocol_sim import Logger, PahoMqttTransport, TopicTransport, robot_topic


DEFAULT_CALIBRATION_PATH = Path(__file__).resolve().with_name("current_camera.json")
DEFAULT_GROUND_PATH = Path(__file__).resolve().with_name("current_ground.json")
LOCAL_COORD_PROJECTOR: Optional["GroundPlaneProjector"] = None

# 카메라 양 옆(좌/우)과 아래쪽 외곽은 렌즈 왜곡이 커서 ground projection 신뢰도가 떨어진다.
# 소화기 bbox 하단 변의 중점(=(cx, y2)) 이 이미지의 '가운데 영역' 안에 있을 때만 좌표를 신뢰한다.
DETECTION_VALID_X_MARGIN_RATIO = 0.15  # 좌/우 각각 가장자리에서 제외할 폭 비율
DETECTION_VALID_BOTTOM_MARGIN_RATIO = 0.0  # 아래쪽 가장자리에서 제외할 높이 비율


def is_pixel_in_valid_region(
    pixel_x: float,
    pixel_y: float,
    image_size: Dict[str, int],
) -> bool:
    """bbox 하단 중점이 이미지 가운데 영역(왜곡이 적은 영역) 안에 있는지 확인한다.

    좌/우 가장자리(DETECTION_VALID_X_MARGIN_RATIO)와 아래쪽 가장자리
    (DETECTION_VALID_BOTTOM_MARGIN_RATIO)는 신뢰하지 않는다.
    이미지 크기를 알 수 없으면 거르지 않는다.
    """
    width = float(image_size.get("width", 0))
    height = float(image_size.get("height", 0))
    if width <= 0 or height <= 0:
        return True
    min_x = width * DETECTION_VALID_X_MARGIN_RATIO
    max_x = width * (1.0 - DETECTION_VALID_X_MARGIN_RATIO)
    max_y = height * (1.0 - DETECTION_VALID_BOTTOM_MARGIN_RATIO)
    return min_x <= pixel_x <= max_x and pixel_y <= max_y


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
        # current_ground.json calibration: +ground_y points robot-forward and
        # +ground_x points robot-right, so map +ground_y -> +x and -ground_x -> +y.
        # (keeps a right-handed robot frame: x forward, y left, z up)
        local_x = ground_y
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

    del class_name, confidence, robot_infos
    if LOCAL_COORD_PROJECTOR is None:
        return None

    pixel_x = float(bbox["cx"])
    pixel_y = float(bbox["y2"])
    # 좌/우 가장자리나 너무 아래쪽에 잡힌 detection 은 왜곡 때문에 좌표를 믿지 않는다.
    if not is_pixel_in_valid_region(pixel_x, pixel_y, image_size):
        return None
    return LOCAL_COORD_PROJECTOR.project_pixel_to_local(pixel_x, pixel_y)


class WebRTCFrameSource:
    """로봇 카메라에서 WebRTC로 들어오는 실시간 프레임 중 '최신 한 장'만 들고 있는다.

    WebRTC 콜백은 단일 슬롯에 프레임을 '덮어쓰기'만 한다
    (추론이 스트림 속도를 못 따라가도 항상 최신 프레임을 처리, 큐 적체/stale 결과 방지).
    WebRTCVideoReceiver.run() 은 코루틴이므로 자체 asyncio 이벤트 루프를 백그라운드 스레드에서 돌려
    DetectionNode 의 동기 tick 루프와 분리한다.
    """

    def __init__(
        self,
        signaling_host: str,
        signaling_port: int,
        room: str,
        detect_channel: int,
        logger: Logger,
    ) -> None:
        self.signaling_host = signaling_host
        self.signaling_port = signaling_port
        self.room = room
        self.detect_channel = detect_channel
        self.logger = logger
        self._lock = threading.Lock()
        self._latest: Optional[Tuple[int, Any, float]] = None  # (seq, img_bgr, stream_ts)
        self._seq = 0
        self._thread: Optional[threading.Thread] = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._run_task: Optional["asyncio.Task[Any]"] = None
        self._receiver: Any = None
        self._receiver_cls: Any = None
        self._last_frame_arrival_s: Optional[float] = None

    def _on_frame(self, channel: int, img: Any, stream_ts: float) -> None:
        # WebRTC 수신 스레드에서 호출됨. detect 채널만 슬롯에 덮어쓴다.
        if channel != self.detect_channel:
            return
        now = time.monotonic()
        with self._lock:
            self._seq += 1
            seq = self._seq
            # img 버퍼가 재사용될 수 있으니 반드시 copy (sample DetectionWorker.submit 과 동일).
            self._latest = (seq, img.copy(), float(stream_ts))
            prev_arrival = self._last_frame_arrival_s
            self._last_frame_arrival_s = now
        # 프레임이 카메라/스트림에서 실제로 몇 초마다 들어오는지 로깅.
        if prev_arrival is None:
            self.logger.log(0.0, f"frame arrival seq={seq} (first frame)")
        else:
            interval = now - prev_arrival
            fps = (1.0 / interval) if interval > 0 else float("inf")
            # self.logger.log(0.0, f"frame arrival seq={seq} interval={interval:.3f}s (~{fps:.2f} fps)")

    def get_latest(self) -> Optional[Tuple[int, Any, float]]:
        with self._lock:
            return self._latest

    def start(self) -> None:
        try:
            from core.webrtc_video import WebRTCVideoReceiver  # type: ignore
        except ImportError as exc:
            raise RuntimeError(
                "WebRTC source requires aiohttp/aiortc and the sample_temp/core package. "
                "Install with: pip install aiohttp aiortc"
            ) from exc

        self._receiver_cls = WebRTCVideoReceiver
        self._thread = threading.Thread(target=self._run, name="webrtc-recv", daemon=True)
        self._thread.start()
        self.logger.log(
            0.0,
            f"webrtc source start room={self.room} channel={self.detect_channel} "
            f"signaling={self.signaling_host}:{self.signaling_port}",
        )

    def _run(self) -> None:
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._receiver = self._receiver_cls(
            signaling_host=self.signaling_host,
            signaling_port=self.signaling_port,
            room_names=[self.room],
            on_frame_callback=self._on_frame,
        )
        self._run_task = self._loop.create_task(self._receiver.run())
        try:
            self._loop.run_until_complete(self._run_task)
        except asyncio.CancelledError:
            pass
        finally:
            self._loop.close()

    def stop(self) -> None:
        if self._receiver is not None:
            self._receiver.shutdown()
        if self._loop is not None and self._run_task is not None:
            self._loop.call_soon_threadsafe(self._run_task.cancel)
        if self._thread is not None:
            self._thread.join(timeout=5.0)


class DetectionNode:
    def __init__(
        self,
        bus: TopicTransport,
        robot_id: int,
        logger: Logger,
        frame_source: "WebRTCFrameSource",
        det_output_dir: Path,
        weights: str,
        conf: float,
        status_interval_s: float,
        device: Optional[str] = None,
        half: bool = False,
    ) -> None:
        self.bus = bus
        self.robot_id = robot_id
        self.logger = logger
        self.frame_source = frame_source
        # run 마다 구분되는 subfolder 생성 (det_output_dir/run_YYYYmmdd_HHMMSS)
        self.det_output_dir = det_output_dir / f"run_{datetime.now():%Y%m%d_%H%M%S}"
        self.weights = weights
        self.conf = conf
        self.status_interval_s = status_interval_s
        # None 이면 ultralytics 자동 선택. "0"/"cuda:0" 로 GPU 강제, "cpu" 로 CPU 강제 가능.
        self.device = device
        # FP16 추론 (GPU 메모리 약 절반). GPU 일 때만 적용.
        self.half = half
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
        self._last_frame_seq: int = 0

        bus.subscribe(robot_topic(robot_id, "detection_config"), self.on_detection_config)
        bus.subscribe(robot_topic(robot_id, "detection_enable"), self.on_detection_enable)
        bus.subscribe(robot_topic(robot_id, "robot_infos"), self.on_robot_infos)

    def start(self, sim_time_s: float) -> None:
        self.load_model(sim_time_s)
        self.update_state()

    def load_model(self, sim_time_s: float) -> None:
        try:
            from ultralytics import YOLOWorld, YOLOE  # type: ignore
        except ImportError as exc:
            raise RuntimeError(
                "ultralytics is required for detection mode. Install with: pip install ultralytics"
            ) from exc

        self.logger.log(sim_time_s, f"load YOLOE weights={self.weights}")
        self._model = YOLOE(self.weights)

        # torch 가 CUDA 를 보는지 확인하고, --device 미지정 시 자동으로 GPU 에 올린다.
        try:
            import torch  # type: ignore

            cuda_ok = torch.cuda.is_available()
            gpu_name = torch.cuda.get_device_name(0) if cuda_ok else "-"
            self.logger.log(
                sim_time_s,
                f"torch={torch.__version__} cuda_available={cuda_ok} "
                f"built_cuda={torch.version.cuda} gpu={gpu_name}",
            )
            # --device 지정 시 그 값 우선, 미지정이면 CUDA 가 있으면 cuda:0, 없으면 cpu.
            if self.device is None:
                self.device = "cuda:0" if cuda_ok else "cpu"

            try:
                self._model.to(self.device)
                self.logger.log(sim_time_s, f"move model to device={self.device}")
            except torch.cuda.OutOfMemoryError as exc:
                # GB10 통합 메모리가 부족(예: llama-server 등이 점유)하면 OOM. CPU 로 폴백.
                self.logger.log(
                    sim_time_s,
                    f"CUDA OOM while moving model to {self.device}: {exc}. fallback to cpu",
                )
                torch.cuda.empty_cache()
                self.device = "cpu"
                self.half = False
                self._model.to(self.device)
        except ImportError:
            self.logger.log(sim_time_s, "torch import failed; running on default (cpu) device")

        # 모델 파라미터가 실제로 올라가 있는 device 를 확인 (cpu 면 추론이 느린 원인).
        try:
            model_device = next(self._model.model.parameters()).device
            self.logger.log(sim_time_s, f"model device={model_device}")
        except (StopIteration, AttributeError):
            pass

        self.model_loaded = True
        if self.target_classes:
            self.set_model_classes(sim_time_s)
        self.logger.log(sim_time_s, "YOLOE model ready")

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
            self._last_frame_seq = 0

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
        self._last_frame_seq = 0
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
    
    def process_latest_image(self, sim_time_s: float) -> None:
        latest = self.frame_source.get_latest()
        if latest is None:
            return

        seq, img, _stream_ts = latest
        if seq == self._last_frame_seq:
            return

        robot_infos = self.latest_robot_infos
        if robot_infos is None:
            return

        height, width = img.shape[:2]
        image_size = {"width": int(width), "height": int(height)}
        # 한 프레임 추론에 걸리는 시간을 측정해서 로깅.
        predict_start = time.monotonic()
        predict_kwargs: Dict[str, Any] = {"verbose": False, "conf": self.conf}
        if self.device is not None:
            predict_kwargs["device"] = self.device
        if self.half and self.device not in (None, "cpu"):
            predict_kwargs["half"] = True
        results = self._model.predict(img, **predict_kwargs)
        predict_elapsed = time.monotonic() - predict_start
        self.logger.log(sim_time_s, f"inference seq={seq} took={predict_elapsed:.3f}s")
        objects = self.build_info_detected_objects(results, image_size, robot_infos)
        image_path = self.save_annotated_frame(results, sim_time_s, seq)

        payload = {
            "robotId": self.robot_id,
            "timestampMs": float(sim_time_s),
            "imagePath": image_path,
            "imageSize": image_size,
            "robotInfos": robot_infos,
            "classes": list(self.target_classes),
            "objects": objects,
        }
        self.bus.publish(robot_topic(self.robot_id, "detected_objects"), payload)
        self.latest_image_path = image_path
        self._last_frame_seq = seq
        self.logger.log(sim_time_s, f"publish detected_objects count={len(objects)} seq={seq}")

    def save_annotated_frame(self, results: Any, sim_time_s: float, seq: int) -> Optional[str]:
        """추론 결과(bbox)가 그려진 프레임을 디스크에 저장하고 경로를 반환한다."""
        try:
            import cv2  # type: ignore
        except ImportError as exc:
            raise RuntimeError(
                "opencv-python is required to save annotated frames. Install with: pip install opencv-python"
            ) from exc

        if not results:
            return None

        annotated = results[0].plot()  # bbox 그려진 BGR ndarray (sample 의 r.plot() 과 동일)
        self.det_output_dir.mkdir(parents=True, exist_ok=True)
        out_path = self.det_output_dir / f"{sim_time_s:.3f}_{seq:06d}.jpg"
        if not cv2.imwrite(str(out_path), annotated):
            self.logger.log(sim_time_s, f"failed to save annotated frame {out_path}")
            return None
        return str(out_path.resolve())

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
    parser.add_argument("--signaling-host", default="localhost")
    parser.add_argument("--signaling-port", type=int, default=4732)
    parser.add_argument("--room", default="room1", help="detection 에 사용할 카메라 room 이름")
    parser.add_argument("--detect-channel", type=int, default=0, help="room_names 내 detect 채널 인덱스")
    parser.add_argument("--det-output-dir", default="./detections", help="bbox 그린 결과 프레임 저장 경로")
    parser.add_argument("--weights", default="yoloe-26x-seg.pt")
    parser.add_argument("--poll-interval", type=float, default=0.2)
    parser.add_argument("--status-interval", type=float, default=1.0)
    parser.add_argument("--conf", type=float, default=0.5)
    parser.add_argument(
        "--device",
        default=None,
        help="추론 device. 예: '0' 또는 'cuda:0' (GPU 강제), 'cpu' (CPU 강제). 미지정 시 자동.",
    )
    parser.add_argument(
        "--half",
        action="store_true",
        help="GPU 에서 FP16 추론 (메모리 약 절반). OOM 회피에 도움.",
    )
    parser.add_argument("--calibration", default=str(DEFAULT_CALIBRATION_PATH))
    parser.add_argument("--ground", default=str(DEFAULT_GROUND_PATH))
    parser.add_argument(
        "--valid-x-margin",
        type=float,
        default=DETECTION_VALID_X_MARGIN_RATIO,
        help="좌/우 각각 가장자리에서 detect 를 신뢰하지 않을 폭 비율 (0~0.5)",
    )
    parser.add_argument(
        "--valid-bottom-margin",
        type=float,
        default=DETECTION_VALID_BOTTOM_MARGIN_RATIO,
        help="아래쪽 가장자리에서 detect 를 신뢰하지 않을 높이 비율 (0~1)",
    )
    return parser.parse_args()


def main() -> None:
    global LOCAL_COORD_PROJECTOR
    global DETECTION_VALID_X_MARGIN_RATIO, DETECTION_VALID_BOTTOM_MARGIN_RATIO

    args = parse_args()
    logger = Logger("detection-node")
    DETECTION_VALID_X_MARGIN_RATIO = args.valid_x_margin
    DETECTION_VALID_BOTTOM_MARGIN_RATIO = args.valid_bottom_margin
    LOCAL_COORD_PROJECTOR = GroundPlaneProjector.from_paths(
        calibration_path=Path(args.calibration),
        ground_path=Path(args.ground),
    )
    logger.log(0.0, f"load calibration={args.calibration} ground={args.ground}")
    logger.log(
        0.0,
        f"detection valid region x_margin={DETECTION_VALID_X_MARGIN_RATIO} "
        f"bottom_margin={DETECTION_VALID_BOTTOM_MARGIN_RATIO}",
    )

    transport = PahoMqttTransport(
        host=args.host,
        port=args.port,
        client_id=f"detection-node-robotid{args.robot_id}",
        logger=logger,
    )
    frame_source = WebRTCFrameSource(
        signaling_host=args.signaling_host,
        signaling_port=args.signaling_port,
        room=args.room,
        detect_channel=args.detect_channel,
        logger=logger,
    )
    node = DetectionNode(
        transport,
        args.robot_id,
        logger,
        frame_source=frame_source,
        det_output_dir=Path(args.det_output_dir),
        weights=args.weights,
        conf=args.conf,
        status_interval_s=args.status_interval,
        device=args.device,
        half=args.half,
    )

    transport.start()
    frame_source.start()
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
        frame_source.stop()
        transport.stop()


if __name__ == "__main__":
    main()

from __future__ import annotations

from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from bridge_interfaces.msg import VisionDetection, VisionDetectionArray
import rclpy
from rclpy.node import Node

from vision.camera_utils import ManagedCamera
from vision.model_utils import (
    DetectionRecord,
    YoloNcnnDetector,
    default_model_path,
    resolve_model_path,
)
from vision.timing_utils import FixedRateScheduler
from vision.traffic_light import classify_traffic_light_color


class VisionNode(Node):
    def __init__(self) -> None:
        super().__init__("vision_node")

        self._share_data_dir = Path(get_package_share_directory("vision")) / "data"
        self._source_data_dir = Path("/ros2_ws/src/vision/data")
        model_default = default_model_path(self._source_data_dir, self._share_data_dir)

        self.declare_parameter("camera_device", "/dev/video10")
        self.declare_parameter("camera_width", 640)
        self.declare_parameter("camera_height", 480)
        self.declare_parameter("camera_fps", 5.0)
        self.declare_parameter("process_rate_hz", 5.0)
        self.declare_parameter("model_path", str(model_default))
        self.declare_parameter("model_imgsz", 640)
        self.declare_parameter("confidence_threshold", 0.25)
        self.declare_parameter("iou_threshold", 0.7)
        self.declare_parameter("max_detections", 20)
        self.declare_parameter("class_filter", "traffic light,stop sign,person")
        self.declare_parameter("ncnn_threads", 4)
        self.declare_parameter("reconnect_delay_sec", 1.0)
        self.declare_parameter("log_interval_sec", 5.0)

        self._camera_device = str(self.get_parameter("camera_device").value)
        self._camera_width = int(self.get_parameter("camera_width").value)
        self._camera_height = int(self.get_parameter("camera_height").value)
        self._camera_fps = float(self.get_parameter("camera_fps").value)
        self._process_rate_hz = float(self.get_parameter("process_rate_hz").value)
        self._model_imgsz = int(self.get_parameter("model_imgsz").value)
        self._confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        self._iou_threshold = float(self.get_parameter("iou_threshold").value)
        self._max_detections = int(self.get_parameter("max_detections").value)
        self._class_filter = str(self.get_parameter("class_filter").value)
        self._ncnn_threads = int(self.get_parameter("ncnn_threads").value)
        self._reconnect_delay_sec = max(0.1, float(self.get_parameter("reconnect_delay_sec").value))
        self._log_interval_sec = max(1.0, float(self.get_parameter("log_interval_sec").value))

        self._publisher = self.create_publisher(VisionDetectionArray, "/vision/detections", 10)
        self._camera = ManagedCamera(
            device=self._camera_device,
            width=self._camera_width,
            height=self._camera_height,
            fps=self._camera_fps,
            reconnect_delay_sec=self._reconnect_delay_sec,
            log_interval_sec=self._log_interval_sec,
            logger=self.get_logger(),
        )
        self._last_loop_summary = 0.0

        model_path = resolve_model_path(
            raw_path=str(self.get_parameter("model_path").value),
            source_data_dir=self._source_data_dir,
            share_data_dir=self._share_data_dir,
        )
        self._detector = YoloNcnnDetector(
            model_path=model_path,
            input_size=self._model_imgsz,
            confidence_threshold=self._confidence_threshold,
            iou_threshold=self._iou_threshold,
            max_detections=self._max_detections,
            class_filter=self._class_filter,
            ncnn_threads=self._ncnn_threads,
        )

        self.get_logger().info(
            "Loaded NCNN YOLO model path=%s max_imgsz=%d classes=%d filter=%s ncnn_threads=%s confidence=%.2f"
            % (
                model_path,
                self._model_imgsz,
                self._detector.class_count,
                self._class_filter or "all",
                self._ncnn_threads if self._ncnn_threads > 0 else "auto",
                self._confidence_threshold,
            )
        )

    def _infer(self, frame) -> list[DetectionRecord]:
        return self._detector.predict(frame)

    def _build_detection_msg(self, record: DetectionRecord) -> VisionDetection:
        detection = VisionDetection()
        detection.class_name = record.class_name
        detection.confidence = float(record.confidence)
        detection.x = int(record.x)
        detection.y = int(record.y)
        detection.width = int(record.width)
        detection.height = int(record.height)
        for attribute in record.attributes:
            detection.attribute_names.append(attribute.name)
            detection.attribute_values.append(attribute.value)
            detection.attribute_scores.append(float(attribute.score))
        return detection

    def _build_detection_array_msg(
        self,
        capture_stamp,
        image_width: int,
        image_height: int,
        records: list[DetectionRecord],
    ) -> VisionDetectionArray:
        message = VisionDetectionArray()
        message.header.stamp = capture_stamp
        message.header.frame_id = "vision_camera"
        message.image_width = int(image_width)
        message.image_height = int(image_height)
        for record in records:
            message.detections.append(self._build_detection_msg(record))
        return message

    def run(self) -> None:
        scheduler = FixedRateScheduler(self._process_rate_hz)

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            if not self._camera.ensure():
                scheduler.reset()
                continue

            scheduler.wait_until_ready()

            ok, frame = self._camera.read()
            if not ok or frame is None:
                self._camera.handle_read_failure()
                scheduler.reset()
                continue

            capture_stamp = self.get_clock().now().to_msg()
            inference_start = time.monotonic()
            try:
                detections = self._infer(frame)

                for detection in detections:
                    object_crop = frame[
                        detection.y : detection.y + detection.height,
                        detection.x : detection.x + detection.width,
                    ]

                    if detection.class_name == "traffic light":
                        traffic_light_crop = object_crop
                        color_label, color_score = classify_traffic_light_color(traffic_light_crop)
                        detection.add_attribute("color", color_label, color_score)
                    elif detection.class_name == "face":
                        face_crop = object_crop
                        # TODO(student): analyze the face crop and attach your own
                        # attributes here, for example gender or customer type.
                        _ = face_crop
                        pass
                    elif detection.class_name == "my_object":
                        custom_object_crop = object_crop
                        # TODO(student): add custom object-specific checks here.
                        _ = custom_object_crop
                        pass

                message = self._build_detection_array_msg(
                    capture_stamp=capture_stamp,
                    image_width=frame.shape[1],
                    image_height=frame.shape[0],
                    records=detections,
                )
                self._publisher.publish(message)
                detection_count = len(message.detections)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"Vision inference failed for one frame: {exc}")
                detection_count = 0
            inference_ms = (time.monotonic() - inference_start) * 1000.0

            now = time.monotonic()
            if now - self._last_loop_summary >= self._log_interval_sec:
                self._last_loop_summary = now
                self.get_logger().info(
                    "Vision frame %dx%d total=%.1fms preprocess=%.1fms ncnn=%.1fms postprocess=%.1fms detections=%d target_rate=%.1fHz"
                    % (
                        frame.shape[1],
                        frame.shape[0],
                        inference_ms,
                        self._detector.last_preprocess_ms,
                        self._detector.last_inference_ms,
                        self._detector.last_postprocess_ms,
                        detection_count,
                        self._process_rate_hz,
                    )
                )

            scheduler.schedule_next()

        self._camera.release()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

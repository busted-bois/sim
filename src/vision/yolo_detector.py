"""YOLOv8 object detector using the Ultralytics runtime.

Follows the same lazy-singleton pattern as ``DepthEstimator``.  The heavy
``YOLO`` model is loaded once on first call and reused for every subsequent
frame.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from src.vision.feed import VisionFrame


@dataclass(frozen=True, slots=True)
class Detection:
    class_name: str
    confidence: float
    x1: int
    y1: int
    x2: int
    y2: int
    nx: float
    ny: float
    size_frac: float


class YoloDetector:
    """Wraps ``ultralytics.YOLO`` for single-image inference."""

    def __init__(
        self,
        model_path: str | Path = "models/yolov8n.pt",
        confidence: float = 0.5,
        classes: list[str] | None = None,
    ) -> None:
        self._model_path = str(model_path)
        self._confidence = confidence
        self._filter_classes = classes
        self._model = None
        self._class_names: dict[int, str] = {}

    def _load(self) -> None:
        from ultralytics import YOLO

        print(f"[yolo] Loading model from {self._model_path} ...")
        self._model = YOLO(self._model_path)
        self._class_names = self._model.names or {}
        print(f"[yolo] Model loaded — {len(self._class_names)} classes")

    def detect(self, frame: VisionFrame) -> list[Detection]:
        if self._model is None:
            self._load()

        results = self._model.predict(
            frame.image_rgb,
            conf=self._confidence,
            verbose=False,
        )

        if not results:
            return []

        w, h = float(frame.width), float(frame.height)
        if w <= 0 or h <= 0:
            return []

        detections: list[Detection] = []
        for result in results:
            boxes = result.boxes
            if boxes is None:
                continue
            for i in range(len(boxes)):
                cls_id = int(boxes.cls[i].item())
                conf = float(boxes.conf[i].item())
                class_name = self._class_names.get(cls_id, str(cls_id))

                if self._filter_classes and class_name not in self._filter_classes:
                    continue

                x1, y1, x2, y2 = boxes.xyxy[i].tolist()
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0
                nx = (cx - 0.5 * w) / (0.5 * w)
                ny = (cy - 0.5 * h) / (0.5 * h)

                box_area = float((x2 - x1) * (y2 - y1))
                size_frac = box_area / (w * h)

                detections.append(
                    Detection(
                        class_name=class_name,
                        confidence=conf,
                        x1=x1,
                        y1=y1,
                        x2=x2,
                        y2=y2,
                        nx=float(nx),
                        ny=float(ny),
                        size_frac=size_frac,
                    )
                )

        detections.sort(key=lambda d: d.size_frac, reverse=True)
        return detections

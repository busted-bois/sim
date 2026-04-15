from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

import airsim


@dataclass(frozen=True, slots=True)
class VisionFrame:
    seq: int
    timestamp_s: float
    frame_age_s: float
    width: int
    height: int
    image_rgb: np.ndarray


@dataclass(frozen=True, slots=True)
class VisionStats:
    capture_attempts: int
    capture_successes: int
    capture_failures: int
    scheduler_dropped_ticks: int
    consumer_dropped_frames: int
    latest_seq: int
    latest_frame_age_s: float
    effective_capture_hz: float


class VisionFeed:
    def __init__(self, client: airsim.MultirotorClient, config: dict) -> None:
        self._client = client
        self._enabled = bool(config.get("enabled", False))
        self._camera_name = str(config.get("camera_name", "0"))
        self._fps = max(1.0, float(config.get("fps", 20.0)))
        self._fov_degrees = float(config.get("fov_degrees", 90.0))
        self._compress = bool(config.get("compress", False))
        self._save_debug_frames = bool(config.get("save_debug_frames", False))
        self._debug_output_dir = Path(str(config.get("debug_output_dir", "logs/vision_frames")))

        self._latest_lock = threading.Lock()
        self._latest: VisionFrame | None = None

        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._seq = 0
        self._last_warning_s = 0.0
        self._start_monotonic_s: float | None = None
        self._capture_attempts = 0
        self._capture_successes = 0
        self._capture_failures = 0
        self._scheduler_dropped_ticks = 0
        self._consumer_dropped_frames = 0
        self._last_consumed_seq = 0

    @property
    def enabled(self) -> bool:
        return self._enabled

    def start(self) -> None:
        if not self._enabled:
            return
        self._start_monotonic_s = time.monotonic()
        self._client.simSetCameraFov(self._camera_name, self._fov_degrees)
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._capture_loop, name="vision_feed", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        if not self._enabled:
            return
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def get_latest(self) -> VisionFrame | None:
        with self._latest_lock:
            if self._latest is None:
                return None
            latest = self._latest
        if self._last_consumed_seq and latest.seq > self._last_consumed_seq + 1:
            self._consumer_dropped_frames += latest.seq - self._last_consumed_seq - 1
        self._last_consumed_seq = latest.seq
        age_s = max(0.0, time.time() - latest.timestamp_s)
        return VisionFrame(
            seq=latest.seq,
            timestamp_s=latest.timestamp_s,
            frame_age_s=age_s,
            width=latest.width,
            height=latest.height,
            image_rgb=latest.image_rgb,
        )

    def get_stats(self) -> VisionStats:
        latest = self.get_latest()
        elapsed_s = 0.0
        if self._start_monotonic_s is not None:
            elapsed_s = max(0.0, time.monotonic() - self._start_monotonic_s)
        effective_capture_hz = 0.0 if elapsed_s <= 0 else self._capture_successes / elapsed_s
        latest_age = 0.0 if latest is None else latest.frame_age_s
        latest_seq = 0 if latest is None else latest.seq
        return VisionStats(
            capture_attempts=self._capture_attempts,
            capture_successes=self._capture_successes,
            capture_failures=self._capture_failures,
            scheduler_dropped_ticks=self._scheduler_dropped_ticks,
            consumer_dropped_frames=self._consumer_dropped_frames,
            latest_seq=latest_seq,
            latest_frame_age_s=latest_age,
            effective_capture_hz=effective_capture_hz,
        )

    def _capture_loop(self) -> None:
        period_s = 1.0 / self._fps
        next_tick = time.time()
        while not self._stop_event.is_set():
            started_s = time.time()
            self._capture_attempts += 1
            try:
                response = self._client.simGetImages(
                    [
                        airsim.ImageRequest(
                            camera_name=self._camera_name,
                            image_type=airsim.ImageType.Scene,
                            pixels_as_float=False,
                            compress=self._compress,
                        )
                    ]
                )[0]
                image_rgb = self._decode_response_rgb(response)
                self._seq += 1
                self._capture_successes += 1
                frame = VisionFrame(
                    seq=self._seq,
                    timestamp_s=started_s,
                    frame_age_s=0.0,
                    width=int(response.width),
                    height=int(response.height),
                    image_rgb=image_rgb,
                )
                with self._latest_lock:
                    self._latest = frame
                if self._save_debug_frames:
                    self._write_debug_frame(frame)
            except Exception as exc:  # noqa: BLE001
                self._capture_failures += 1
                now_s = time.time()
                if now_s - self._last_warning_s > 1.0:
                    print(f"[vision] capture warning: {exc}")
                    self._last_warning_s = now_s

            next_tick += period_s
            sleep_s = next_tick - time.time()
            if sleep_s <= 0:
                missed_ticks = max(1, int(abs(sleep_s) / period_s))
                self._scheduler_dropped_ticks += missed_ticks
                next_tick = time.time()
                continue
            self._stop_event.wait(timeout=sleep_s)

    def _decode_response_rgb(self, response: airsim.ImageResponse) -> np.ndarray:
        width = int(response.width)
        height = int(response.height)
        if width <= 0 or height <= 0:
            raise RuntimeError("vision response has invalid dimensions")
        data = response.image_data_uint8
        if self._compress:
            raise RuntimeError(
                "compress=True is not supported without an image decoder dependency; "
                "set vision.compress=false"
            )
        expected_size = width * height * 3
        if isinstance(data, bytes):
            actual_size = len(data)
            data_array = np.frombuffer(data, dtype=np.uint8)
        else:
            data_array = np.asarray(data, dtype=np.uint8).reshape(-1)
            actual_size = int(data_array.size)
        if actual_size != expected_size:
            raise RuntimeError(
                f"unexpected uncompressed image size={actual_size}, expected={expected_size}"
            )
        return data_array.reshape(height, width, 3).copy()

    def _write_debug_frame(self, frame: VisionFrame) -> None:
        self._debug_output_dir.mkdir(parents=True, exist_ok=True)
        out_path = self._debug_output_dir / f"frame_{frame.seq:06d}.npy"
        np.save(out_path, frame.image_rgb)

"""Vision feed: AirSim RPC capture, UDP video, or disabled stub."""

from __future__ import annotations

import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np

from src.vision.udp_video import UdpVideoReceiver

TrackerCallback = Callable[[np.ndarray, int], None]


@dataclass(frozen=True, slots=True)
class VisionFrame:
    seq: int
    timestamp_s: float
    frame_age_s: float
    sim_time_ns: int
    width: int
    height: int
    image_rgb: np.ndarray


@dataclass(frozen=True, slots=True)
class VisionStats:
    configured_fps: float
    active_fps: float
    capture_attempts: int
    capture_successes: int
    capture_failures: int
    scheduler_dropped_ticks: int
    consumer_dropped_frames: int
    latest_seq: int
    latest_frame_age_s: float
    effective_capture_hz: float
    udp_packets: int = 0
    udp_frames: int = 0


def vision_feed_from_config(
    config: dict,
    *,
    tracker_callback: TrackerCallback | None = None,
    airsim_client: Any | None = None,
) -> VisionFeed:
    vision_cfg = config.get("vision", {})
    udp_cfg = vision_cfg.get("udp_video", {})
    if bool(udp_cfg.get("enabled", False)):
        return UdpVisionFeed(
            port=int(udp_cfg.get("port", 5600)),
            host=str(udp_cfg.get("host", "0.0.0.0")),
            fps=float(vision_cfg.get("fps", 30.0)),
            frame_timeout_s=float(udp_cfg.get("frame_timeout_ms", 200)) / 1000.0,
            tracker_callback=tracker_callback,
        )
    if airsim_client is not None and bool(vision_cfg.get("enabled", False)):
        return AirSimRpcVisionFeed(
            airsim_client,
            vision_cfg,
            tracker_callback=tracker_callback,
        )
    return VisionFeed(config=vision_cfg)


class VisionFeed:
    """Disabled placeholder when vision is off or no capture backend is configured."""

    def __init__(self, client: Any | None = None, config: dict | None = None) -> None:
        _ = client, config
        # Stub only — use vision_feed_from_config() for a real capture backend.
        self._enabled = False

    @property
    def enabled(self) -> bool:
        return self._enabled

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def get_latest(self) -> VisionFrame | None:
        return None

    def get_stats(self) -> VisionStats:
        return VisionStats(
            configured_fps=0.0,
            active_fps=0.0,
            capture_attempts=0,
            capture_successes=0,
            capture_failures=0,
            scheduler_dropped_ticks=0,
            consumer_dropped_frames=0,
            latest_seq=0,
            latest_frame_age_s=0.0,
            effective_capture_hz=0.0,
        )


class AirSimRpcVisionFeed(VisionFeed):
    """FPV frames via AirSim RPC (used with MAVLink control when UDP video is off)."""

    def __init__(
        self,
        client: Any,
        config: dict,
        *,
        tracker_callback: TrackerCallback | None = None,
    ) -> None:
        self._client = client
        self._enabled = bool(config.get("enabled", False))
        self._camera_name = str(config.get("camera_name", "0"))
        self._fps = max(1.0, float(config.get("fps", 30.0)))
        self._compress = bool(config.get("compress", True))
        self._tracker_callback = tracker_callback
        self._lock = threading.Lock()
        self._latest: VisionFrame | None = None
        self._seq = 0
        self._capture_successes = 0
        self._capture_failures = 0
        self._first_mono: float | None = None
        self._last_mono: float | None = None
        self._stop_evt = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if not self._enabled:
            return
        self._stop_evt.clear()
        self._thread = threading.Thread(
            target=self._capture_loop,
            name="airsim-rpc-vision",
            daemon=True,
        )
        self._thread.start()
        print(
            f"[vision] AirSim RPC capture started (camera={self._camera_name!r}, "
            f"target_fps={self._fps:.0f})"
        )

    def stop(self) -> None:
        self._stop_evt.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def _capture_loop(self) -> None:
        import airsim

        min_interval_s = 1.0 / self._fps
        while not self._stop_evt.is_set():
            loop_start = time.monotonic()
            try:
                responses = self._client.simGetImages(
                    [
                        airsim.ImageRequest(
                            self._camera_name,
                            airsim.ImageType.Scene,
                            False,
                            self._compress,
                        )
                    ]
                )
            except Exception:
                with self._lock:
                    self._capture_failures += 1
                time.sleep(min_interval_s)
                continue

            if not responses or responses[0].width == 0 or responses[0].height == 0:
                with self._lock:
                    self._capture_failures += 1
                time.sleep(min_interval_s)
                continue

            response = responses[0]
            img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
            img_bgr = img1d.reshape(response.height, response.width, 3)
            rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
            sim_time_ns = int(getattr(response, "time_stamp", 0) or 0)
            now = time.monotonic()
            with self._lock:
                self._seq += 1
                self._capture_successes += 1
                if self._first_mono is None:
                    self._first_mono = now
                self._last_mono = now
                self._latest = VisionFrame(
                    seq=self._seq,
                    timestamp_s=time.time(),
                    frame_age_s=0.0,
                    sim_time_ns=sim_time_ns,
                    width=int(response.width),
                    height=int(response.height),
                    image_rgb=rgb,
                )
            if self._tracker_callback is not None:
                self._tracker_callback(rgb, sim_time_ns)

            elapsed = time.monotonic() - loop_start
            sleep_s = min_interval_s - elapsed
            if sleep_s > 0:
                time.sleep(sleep_s)

    def get_latest(self) -> VisionFrame | None:
        with self._lock:
            if self._latest is None:
                return None
            frame = self._latest
            age = max(0.0, time.monotonic() - self._last_mono) if self._last_mono else 0.0
            return VisionFrame(
                seq=frame.seq,
                timestamp_s=frame.timestamp_s,
                frame_age_s=age,
                sim_time_ns=frame.sim_time_ns,
                width=frame.width,
                height=frame.height,
                image_rgb=frame.image_rgb,
            )

    def get_stats(self) -> VisionStats:
        with self._lock:
            eff = 0.0
            if (
                self._capture_successes >= 2
                and self._first_mono is not None
                and self._last_mono is not None
                and self._last_mono > self._first_mono
            ):
                eff = (self._capture_successes - 1) / (self._last_mono - self._first_mono)
            return VisionStats(
                configured_fps=self._fps,
                active_fps=min(self._fps, eff),
                capture_attempts=self._capture_successes + self._capture_failures,
                capture_successes=self._capture_successes,
                capture_failures=self._capture_failures,
                scheduler_dropped_ticks=0,
                consumer_dropped_frames=0,
                latest_seq=self._seq,
                latest_frame_age_s=(
                    max(0.0, time.monotonic() - self._last_mono) if self._last_mono else 0.0
                ),
                effective_capture_hz=eff,
            )


class UdpVisionFeed(VisionFeed):
    def __init__(
        self,
        *,
        port: int = 5600,
        host: str = "0.0.0.0",
        fps: float = 30.0,
        frame_timeout_s: float = 0.2,
        tracker_callback: TrackerCallback | None = None,
    ) -> None:
        self._enabled = True
        self._fps = max(1.0, float(fps))
        self._min_interval_s = 1.0 / self._fps
        self._tracker_callback = tracker_callback
        self._receiver = UdpVideoReceiver(
            port=port,
            host=host,
            frame_timeout_s=frame_timeout_s,
        )
        self._lock = threading.Lock()
        self._latest: VisionFrame | None = None
        self._seq = 0
        self._capture_successes = 0
        self._capture_failures = 0
        self._first_mono: float | None = None
        self._last_mono: float | None = None
        self._last_emit_mono: float | None = None

    def start(self) -> None:
        self._receiver.set_frame_callback(self._on_jpeg)
        self._receiver.start()

    def stop(self) -> None:
        self._receiver.stop()

    def _on_jpeg(self, sim_time_ns: int, jpeg_bytes: bytes) -> None:
        now = time.monotonic()
        with self._lock:
            if (
                self._last_emit_mono is not None
                and now - self._last_emit_mono < self._min_interval_s
            ):
                return
        np_buf = np.frombuffer(jpeg_bytes, dtype=np.uint8)
        bgr = cv2.imdecode(np_buf, cv2.IMREAD_COLOR)
        if bgr is None:
            with self._lock:
                self._capture_failures += 1
            return
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        h, w = rgb.shape[:2]
        with self._lock:
            self._seq += 1
            self._capture_successes += 1
            if self._first_mono is None:
                self._first_mono = now
            self._last_mono = now
            self._last_emit_mono = now
            ts_s = sim_time_ns / 1_000_000_000.0
            self._latest = VisionFrame(
                seq=self._seq,
                timestamp_s=ts_s,
                frame_age_s=0.0,
                sim_time_ns=int(sim_time_ns),
                width=w,
                height=h,
                image_rgb=rgb,
            )
        if self._tracker_callback is not None:
            self._tracker_callback(rgb, int(sim_time_ns))

    def get_latest(self) -> VisionFrame | None:
        with self._lock:
            if self._latest is None:
                return None
            frame = self._latest
            age = max(0.0, time.monotonic() - self._last_mono) if self._last_mono else 0.0
            return VisionFrame(
                seq=frame.seq,
                timestamp_s=frame.timestamp_s,
                frame_age_s=age,
                sim_time_ns=frame.sim_time_ns,
                width=frame.width,
                height=frame.height,
                image_rgb=frame.image_rgb,
            )

    def get_stats(self) -> VisionStats:
        with self._lock:
            eff = 0.0
            if (
                self._capture_successes >= 2
                and self._first_mono is not None
                and self._last_mono is not None
                and self._last_mono > self._first_mono
            ):
                eff = (self._capture_successes - 1) / (self._last_mono - self._first_mono)
            udp_stats = self._receiver.stats
            return VisionStats(
                configured_fps=self._fps,
                active_fps=min(self._fps, eff),
                capture_attempts=self._capture_successes + self._capture_failures,
                capture_successes=self._capture_successes,
                capture_failures=self._capture_failures,
                scheduler_dropped_ticks=0,
                consumer_dropped_frames=0,
                latest_seq=self._seq,
                latest_frame_age_s=(
                    max(0.0, time.monotonic() - self._last_mono) if self._last_mono else 0.0
                ),
                effective_capture_hz=eff,
                udp_packets=int(udp_stats.get("packets", 0)),
                udp_frames=int(udp_stats.get("frames", 0)),
            )

"""Vision feed: UDP chunked video or stub when disabled."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass

import cv2
import numpy as np

from src.vision.udp_video import UdpVideoReceiver


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


def vision_feed_from_config(config: dict, *, tracker_callback=None) -> VisionFeed:
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
    return VisionFeed(config=config)


class VisionFeed:
    def __init__(self, client=None, config: dict | None = None) -> None:
        self._enabled = False
        _ = client, config

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


class UdpVisionFeed(VisionFeed):
    def __init__(
        self,
        *,
        port: int = 5600,
        host: str = "0.0.0.0",
        fps: float = 30.0,
        frame_timeout_s: float = 0.2,
        tracker_callback=None,
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

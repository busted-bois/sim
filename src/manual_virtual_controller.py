"""Keyboard -> vJoy bridge used as the missing simple_airsim virtual_controller.

Injected into ``sys.modules['simple_airsim.api.virtual_controller']`` by
``manual_flight_gui.py`` before ``simple_airsim.api.gui_manager`` is imported.

Axes: 1=roll, 2=pitch, 3=throttle, 4=yaw. vJoy axis range is 0x1..0x8000;
neutral is 0x4000. Throttle starts at 0 (bottom).

Default key map:
    W / S          throttle up / down (climb / descend)
    A / D          roll left / right (strafe)
    Up / Down      pitch forward / back
    Left / Right   yaw left / right
"""

from __future__ import annotations

import logging
import threading

try:
    import pyvjoy
except ImportError as e:  # pragma: no cover
    raise RuntimeError(
        "pyvjoy not installed. Run: uv sync --extra manual (and install vJoy driver)"
    ) from e

try:
    from pynput import keyboard
except ImportError as e:  # pragma: no cover
    raise RuntimeError("pynput not installed. Run: uv sync --extra manual") from e


_AXIS_MIN = 0x1
_AXIS_MAX = 0x8000
_AXIS_MID = 0x4000

_AXIS_ROLL = pyvjoy.HID_USAGE_X
_AXIS_PITCH = pyvjoy.HID_USAGE_Y
_AXIS_THROTTLE = pyvjoy.HID_USAGE_Z
_AXIS_YAW = pyvjoy.HID_USAGE_RX

# Sensitivity: fraction of full deflection applied per key press.
# Lower values = gentler control. SimpleFlight tilts proportional to stick.
_TILT_GAIN = 0.30  # roll / pitch
_YAW_GAIN = 0.40
# Throttle is absolute thrust 0..1; 0.5 ~ hover. Nudge above/below for climb/descend.
_THROTTLE_HOVER = 0.50
_THROTTLE_CLIMB = 0.65
_THROTTLE_DESCEND = 0.35

log = logging.getLogger("manual_vjoy")


class VirtualController:
    """Context-managed pynput->pyvjoy bridge matching simple_airsim's API."""

    def __init__(self, device_id: int = 1) -> None:
        self._device_id = device_id
        self._device: pyvjoy.VJoyDevice | None = None
        self._listener: keyboard.Listener | None = None
        self._enabled = False
        self._lock = threading.Lock()
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0
        self._throttle = _THROTTLE_HOVER

    def __enter__(self) -> VirtualController:
        self._device = pyvjoy.VJoyDevice(self._device_id)
        self._reset_axes()
        self._listener = keyboard.Listener(
            on_press=self._on_press, on_release=self._on_release, suppress=False
        )
        self._listener.start()
        log.info("VirtualController attached to vJoy device %d", self._device_id)
        return self

    def __exit__(self, *_args) -> None:
        if self._listener is not None:
            self._listener.stop()
            self._listener = None
        if self._device is not None:
            self._reset_axes()
            self._device = None

    def enable(self) -> None:
        self._enabled = True
        log.info("VirtualController enabled")

    def disable(self) -> None:
        self._enabled = False
        self._reset_axes()
        log.info("VirtualController disabled")

    def _reset_axes(self) -> None:
        if self._device is None:
            return
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0
        self._throttle = _THROTTLE_HOVER
        self._device.set_axis(_AXIS_ROLL, _AXIS_MID)
        self._device.set_axis(_AXIS_PITCH, _AXIS_MID)
        self._device.set_axis(_AXIS_YAW, _AXIS_MID)
        self._device.set_axis(_AXIS_THROTTLE, _scaled(_THROTTLE_HOVER, mid=False))

    def _push(self) -> None:
        if not self._enabled or self._device is None:
            return
        with self._lock:
            roll = _scaled(self._roll, mid=True)
            pitch = _scaled(self._pitch, mid=True)
            yaw = _scaled(self._yaw, mid=True)
            throttle = _scaled(self._throttle, mid=False)
        self._device.set_axis(_AXIS_ROLL, roll)
        self._device.set_axis(_AXIS_PITCH, pitch)
        self._device.set_axis(_AXIS_YAW, yaw)
        self._device.set_axis(_AXIS_THROTTLE, throttle)
        log.debug("axes roll=%d pitch=%d yaw=%d thr=%d", roll, pitch, yaw, throttle)

    def _on_press(self, key) -> None:
        self._update(key, pressed=True)

    def _on_release(self, key) -> None:
        self._update(key, pressed=False)

    def _update(self, key, *, pressed: bool) -> None:
        with self._lock:
            if key == keyboard.Key.up:
                self._pitch = +_TILT_GAIN if pressed else 0.0
            elif key == keyboard.Key.down:
                self._pitch = -_TILT_GAIN if pressed else 0.0
            elif key == keyboard.Key.left:
                self._yaw = -_YAW_GAIN if pressed else 0.0
            elif key == keyboard.Key.right:
                self._yaw = +_YAW_GAIN if pressed else 0.0
            else:
                ch = getattr(key, "char", None)
                if ch is None:
                    return
                ch = ch.lower()
                if ch == "w":
                    self._throttle = _THROTTLE_CLIMB if pressed else _THROTTLE_HOVER
                elif ch == "s":
                    self._throttle = _THROTTLE_DESCEND if pressed else _THROTTLE_HOVER
                elif ch == "a":
                    self._roll = -_TILT_GAIN if pressed else 0.0
                elif ch == "d":
                    self._roll = +_TILT_GAIN if pressed else 0.0
                else:
                    return
        self._push()


def _scaled(value: float, *, mid: bool) -> int:
    """Map [-1, 1] (centered axis) or [0, 1] (throttle) to vJoy axis range."""
    if mid:
        v = max(-1.0, min(1.0, value))
        return int(_AXIS_MID + v * (_AXIS_MAX - _AXIS_MID))
    v = max(0.0, min(1.0, value))
    return int(_AXIS_MIN + v * (_AXIS_MAX - _AXIS_MIN))

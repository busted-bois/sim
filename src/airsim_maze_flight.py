"""AirSim RPC compatibility for the UE 4.16 maze project (opensrc/AirsimSimulation).

That fork often: omits empty trailing strings on the wire, lacks listVehicles / simPause,
uses older RPC tables, and may bind enableApiControl / armDisarm with different arities
(bool vs int, 1-arg vs 2-arg, or reversed parameter order). Flight code uses this module
when AIGP_MAZE=1.
"""

from __future__ import annotations

import json
import os
import sys
import time
from typing import Any

import airsim


def _unwrap_rpc_client(client: Any) -> Any:
    inner = client.client
    while hasattr(inner, "_inner"):
        inner = inner._inner
    return inner


def _vehicle_names_from_sim_settings(client: airsim.MultirotorClient) -> list[str]:
    """Read Vehicles{...} keys from the simulator's merged settings JSON (no listVehicles)."""
    raw = _unwrap_rpc_client(client)
    try:
        s = raw.call("getSettingsString")
    except Exception:
        return []
    if not s or not isinstance(s, str):
        return []
    try:
        data = json.loads(s)
    except json.JSONDecodeError:
        return []
    v = data.get("Vehicles")
    if isinstance(v, dict) and v:
        return [str(k) for k in v.keys()]
    return []


def resolve_maze_vehicle_name(client: airsim.MultirotorClient) -> str:
    """Resolve API vehicle name and register it for all legacy coerced RPC calls."""
    override = os.environ.get("AIGP_AIRSIM_VEHICLE_NAME", "").strip()
    if override:
        airsim.set_maze_default_vehicle(override)
        print(f"Maze flight: AIGP_AIRSIM_VEHICLE_NAME={override!r} (skipping discovery).")
        return override

    settings_names = _vehicle_names_from_sim_settings(client)

    names: list[str] = []
    for attempt in range(1, 6):
        try:
            raw = client.listVehicles()
            names = [str(n) for n in raw] if raw else []
        except Exception:
            time.sleep(0.25 * attempt)
            continue
        if names:
            break
        time.sleep(0.25 * attempt)

    if not names and settings_names:
        chosen = "SimpleFlight" if "SimpleFlight" in settings_names else settings_names[0]
        airsim.set_maze_default_vehicle(chosen)
        print(
            f"Maze flight: listVehicles unavailable; using Vehicles from settings: "
            f"{settings_names!r} → {chosen!r}"
        )
        return chosen

    if not names:
        print(
            "Maze flight: no listVehicles and no Vehicles{} in getSettingsString; "
            "using SimpleFlight. Set AIGP_AIRSIM_VEHICLE_NAME if the drone ignores API.",
            file=sys.stderr,
        )
        airsim.set_maze_default_vehicle("SimpleFlight")
        return "SimpleFlight"

    if len(names) == 1 and names[0] == "SimpleFlight":
        airsim.set_maze_default_vehicle("SimpleFlight")
        return "SimpleFlight"

    if "SimpleFlight" in names and len(names) > 1:
        airsim.set_maze_default_vehicle("SimpleFlight")
        print(
            f"Maze flight: multiple vehicles {names!r}; using 'SimpleFlight'. "
            "Set AIGP_AIRSIM_VEHICLE_NAME to override."
        )
        return "SimpleFlight"

    chosen = "SimpleFlight" if "SimpleFlight" in names else str(names[0])
    airsim.set_maze_default_vehicle(chosen)
    print(f"Maze flight: vehicle name from listVehicles: {names!r} (using {chosen!r}).")
    return chosen


def try_maze_unpause_and_clock(client: airsim.MultirotorClient) -> None:
    """Best-effort: many UE 4.16 builds omit simPause / simContinueForTime; ignore errors."""
    pause_fn = getattr(client, "simPause", None)
    if callable(pause_fn):
        try:
            pause_fn(False)
            print("Maze flight: simPause(False) (sim may not support this RPC).")
        except Exception as exc:
            print(f"Maze flight: simPause skipped ({exc}).")
    is_p = getattr(client, "simIsPause", None) or getattr(client, "simIsPaused", None)
    if callable(is_p):
        try:
            is_p()
        except Exception as exc:
            print(f"Maze flight: simIsPause query skipped ({exc}).")
    cft = getattr(client, "simContinueForTime", None)
    if callable(cft):
        try:
            cft(0.5)
            print("Maze flight: simContinueForTime(0.5).")
        except Exception as exc:
            print(f"Maze flight: simContinueForTime skipped ({exc}).")


def _maze_rpc_try_variants(
    raw: Any, method: str, variants: list[tuple[str, tuple[Any, ...]]]
) -> None:
    """Try several (label, args) tuples until raw.call succeeds."""
    errors: list[str] = []
    last_exc: BaseException | None = None
    for label, args in variants:
        try:
            raw.call(method, *args)
            if label != variants[0][0]:
                print(f"Maze flight: {method} succeeded with variant {label} args={args!r}")
            return
        except BaseException as exc:
            last_exc = exc
            errors.append(f"{label}: {type(exc).__name__}: {exc}")
            continue
    msg = f"Maze flight: {method} failed all variants: " + "; ".join(errors)
    if last_exc is not None:
        raise RuntimeError(msg) from last_exc
    raise RuntimeError(msg)


def maze_enable_api_control(client: airsim.MultirotorClient, enabled: bool, vehicle: str) -> None:
    raw = _unwrap_rpc_client(client)
    ve = (vehicle or "").strip() or "SimpleFlight"
    ei = 1 if enabled else 0
    # UE 4.16 / old rpclib: try int before bool; 1-arg before 2-arg; reversed order last.
    variants: list[tuple[str, tuple[Any, ...]]] = [
        ("1xint", (ei,)),
        ("1xbool", (enabled,)),
        ("2x(int,str)", (ei, ve)),
        ("2x(str,int)", (ve, ei)),
        ("2x(bool,str)", (enabled, ve)),
        ("2x(str,bool)", (ve, enabled)),
    ]
    _maze_rpc_try_variants(raw, "enableApiControl", variants)


def maze_arm_disarm(client: airsim.MultirotorClient, arm: bool, vehicle: str) -> None:
    raw = _unwrap_rpc_client(client)
    ve = (vehicle or "").strip() or "SimpleFlight"
    ai = 1 if arm else 0
    variants: list[tuple[str, tuple[Any, ...]]] = [
        ("2x(int,str)", (ai, ve)),
        ("2x(str,int)", (ve, ai)),
        ("2x(bool,str)", (arm, ve)),
        ("2x(str,bool)", (ve, arm)),
        ("1xint", (ai,)),
        ("1xbool", (arm,)),
    ]
    _maze_rpc_try_variants(raw, "armDisarm", variants)


class MazeFlightClient:
    """Adapter that always sends a non-empty vehicle_name for flight RPCs.

    Some UE 4.16 forks drop trailing empty string args, which breaks methods whose
    signature is `(…, vehicle_name)` when called with defaults.
    """

    def __init__(self, client: airsim.MultirotorClient, vehicle_name: str) -> None:
        self._client = client
        self._vehicle_name = (vehicle_name or "").strip() or "SimpleFlight"
        self._raw = _unwrap_rpc_client(client)
        self._vehicle_bytes = self._vehicle_name.encode("utf-8")
        self._motion_backend: str | None = None
        self._signature_cache: dict[str, int] = {}

    def __getattr__(self, name: str):
        return getattr(self._client, name)

    class _SyncJoinResult:
        __slots__ = ("_result", "_wait_s")

        def __init__(self, result: Any, wait_s: float = 0.0) -> None:
            self._result = result
            self._wait_s = max(0.0, float(wait_s))

        def join(self):
            if self._wait_s > 0.0:
                time.sleep(self._wait_s)
            return self._result

    def _call_variants(self, method: str, variants: list[tuple[str, tuple[Any, ...]]]) -> Any:
        errors: list[str] = []
        last_exc: BaseException | None = None
        for label, args in variants:
            try:
                result = self._raw.call(method, *args)
                if label != variants[0][0]:
                    print(f"Maze flight: {method} succeeded with variant {label} args={args!r}")
                return result
            except BaseException as exc:
                last_exc = exc
                errors.append(
                    f"{label} argc={len(args)} args={args!r}: {type(exc).__name__}: {exc}"
                )
        msg = f"Maze flight: {method} failed all variants: " + "; ".join(errors)
        if last_exc is not None:
            raise RuntimeError(msg) from last_exc
        raise RuntimeError(msg)

    def _call_variants_cached(
        self,
        cache_key: str,
        method: str,
        variants: list[tuple[str, tuple[Any, ...]]],
    ) -> Any:
        cached_idx = self._signature_cache.get(cache_key)
        if cached_idx is not None and 0 <= cached_idx < len(variants):
            label, args = variants[cached_idx]
            try:
                return self._raw.call(method, *args)
            except BaseException:
                self._signature_cache.pop(cache_key, None)
        errors: list[str] = []
        last_exc: BaseException | None = None
        for idx, (label, args) in enumerate(variants):
            try:
                result = self._raw.call(method, *args)
                self._signature_cache[cache_key] = idx
                if idx != 0:
                    print(f"Maze flight: {method} cached variant {label} args={args!r}")
                return result
            except BaseException as exc:
                last_exc = exc
                errors.append(
                    f"{label} argc={len(args)} args={args!r}: {type(exc).__name__}: {exc}"
                )
        msg = f"Maze flight: {method} failed all variants: " + "; ".join(errors)
        if last_exc is not None:
            raise RuntimeError(msg) from last_exc
        raise RuntimeError(msg)

    @staticmethod
    def _normalize_motion_extras(
        drivetrain: Any | None = None,
        yaw_mode: Any | None = None,
    ) -> tuple[int, dict[str, float | bool]]:
        dt = drivetrain if drivetrain is not None else airsim.DrivetrainType.MaxDegreeOfFreedom
        dt_value = int(getattr(dt, "value", dt))
        yaw = yaw_mode if yaw_mode is not None else airsim.YawMode()
        yaw_is_rate = bool(getattr(yaw, "is_rate", False))
        yaw_or_rate = float(getattr(yaw, "yaw_or_rate", 0.0))
        return dt_value, {"is_rate": yaw_is_rate, "yaw_or_rate": yaw_or_rate}

    def _decode_multirotor_state(self, encoded: Any):
        # Stock client expects 9 fields; older forks may return 6-8 fields.
        if isinstance(encoded, (list, tuple)):
            data = list(encoded)
            if len(data) < 9:
                data = data + [False, "", False][: (9 - len(data))]
            return airsim.MultirotorState.from_msgpack(data)
        if isinstance(encoded, dict):
            state = airsim.MultirotorState()
            try:
                kin = encoded.get("kinematics_estimated")
                if kin is not None:
                    state.kinematics_estimated = airsim.KinematicsState.from_msgpack(kin)
            except Exception:
                pass
            ts = encoded.get("timestamp")
            if ts is not None:
                state.timestamp = ts
            ls = encoded.get("landed_state")
            if ls is not None:
                state.landed_state = ls
            return state
        return airsim.MultirotorState.from_msgpack(encoded)

    def _fallback_ground_truth_state(self):
        encoded = self._call_variants(
            "simGetGroundTruthKinematics",
            [
                ("(str)", (self._vehicle_name,)),
                ("(bytes)", (self._vehicle_bytes,)),
                ("()", ()),
            ],
        )
        kin = airsim.KinematicsState.from_msgpack(encoded)
        state = airsim.MultirotorState()
        state.kinematics_estimated = kin
        return state

    def takeoffAsync(self, timeout_sec: float = 20.0):
        result = self._call_variants(
            "takeoff",
            [
                ("(timeout)", (float(timeout_sec),)),
                ("(timeout,str)", (float(timeout_sec), self._vehicle_name)),
                ("(timeout,bytes)", (float(timeout_sec), self._vehicle_bytes)),
            ],
        )
        return self._SyncJoinResult(result)

    def hoverAsync(self):
        result = self._call_variants(
            "hover",
            [
                ("(str)", (self._vehicle_name,)),
                ("(bytes)", (self._vehicle_bytes,)),
                ("()", ()),
            ],
        )
        return self._SyncJoinResult(result)

    def landAsync(self, timeout_sec: float = 60.0):
        result = self._call_variants(
            "land",
            [
                ("(timeout)", (float(timeout_sec),)),
                ("(timeout,str)", (float(timeout_sec), self._vehicle_name)),
                ("(timeout,bytes)", (float(timeout_sec), self._vehicle_bytes)),
            ],
        )
        return self._SyncJoinResult(result)

    def goHomeAsync(self, timeout_sec: float = 3e38):
        result = self._call_variants(
            "goHome",
            [
                ("(timeout)", (float(timeout_sec),)),
                ("(timeout,str)", (float(timeout_sec), self._vehicle_name)),
                ("(timeout,bytes)", (float(timeout_sec), self._vehicle_bytes)),
            ],
        )
        return self._SyncJoinResult(result)

    def getMultirotorState(self):
        encoded = self._call_variants(
            "getMultirotorState",
            [
                ("()", ()),
                ("(str)", (self._vehicle_name,)),
                ("(bytes)", (self._vehicle_bytes,)),
            ],
        )
        try:
            return self._decode_multirotor_state(encoded)
        except Exception as exc:
            kind = type(encoded).__name__
            extra = ""
            if isinstance(encoded, (list, tuple)):
                extra = f" len={len(encoded)}"
            print(
                "Maze flight: getMultirotorState payload decode failed "
                f"({type(exc).__name__}: {exc}); payload type={kind}{extra}. "
                "Falling back to simGetGroundTruthKinematics()."
            )
            return self._fallback_ground_truth_state()

    def getRotorStates(self):
        encoded = self._call_variants(
            "getRotorStates",
            [
                ("()", ()),
                ("(str)", (self._vehicle_name,)),
                ("(bytes)", (self._vehicle_bytes,)),
            ],
        )
        return airsim.RotorStates.from_msgpack(encoded)

    def moveByVelocityAsync(self, vx: float, vy: float, vz: float, duration: float, *args):
        drivetrain = args[0] if len(args) >= 1 else None
        yaw_mode = args[1] if len(args) >= 2 else None
        dt_value, yaw_payload = self._normalize_motion_extras(drivetrain, yaw_mode)
        result = self._call_variants_cached(
            cache_key="moveByVelocity",
            method="moveByVelocity",
            variants=[
                ("(vx,vy,vz,duration)", (vx, vy, vz, duration)),
                (
                    "(vx,vy,vz,duration,drivetrain,yaw)",
                    (vx, vy, vz, duration, dt_value, yaw_payload),
                ),
                ("(vx,vy,vz,duration,int,float)", (vx, vy, vz, duration, dt_value, 0.0)),
                (
                    "(...,str)",
                    (vx, vy, vz, duration, self._vehicle_name),
                ),
                (
                    "(...,bytes)",
                    (vx, vy, vz, duration, self._vehicle_bytes),
                ),
            ],
        )
        return self._SyncJoinResult(result, wait_s=duration)

    def moveByVelocityZAsync(self, vx: float, vy: float, z: float, duration: float, *args):
        drivetrain = args[0] if len(args) >= 1 else None
        yaw_mode = args[1] if len(args) >= 2 else None
        dt_value, yaw_payload = self._normalize_motion_extras(drivetrain, yaw_mode)
        try:
            result = self._call_variants_cached(
                cache_key="moveByVelocityZ",
                method="moveByVelocityZ",
                variants=[
                    ("(vx,vy,z,duration)", (vx, vy, z, duration)),
                    (
                        "(vx,vy,z,duration,drivetrain,yaw)",
                        (vx, vy, z, duration, dt_value, yaw_payload),
                    ),
                    ("(vx,vy,z,duration,int,float)", (vx, vy, z, duration, dt_value, 0.0)),
                    ("(...,str)", (vx, vy, z, duration, self._vehicle_name)),
                    ("(...,bytes)", (vx, vy, z, duration, self._vehicle_bytes)),
                ],
            )
            return self._SyncJoinResult(result)
        except RuntimeError:
            # Legacy fallback ladder for servers with broken velocity-Z endpoint.
            self.movePlanarAsync(vx, vy, duration, z_ref=z).join()
            return self._SyncJoinResult(None)

    def movePlanarAsync(self, vx: float, vy: float, duration: float, z_ref: float | None = None):
        if self._motion_backend is None:
            self._probe_motion_backend(duration=max(0.15, duration))
        backend = self._motion_backend or "moveByRC"
        if backend == "moveByVelocity":
            vz = 0.0
            if z_ref is not None:
                try:
                    z_now = float(self.getMultirotorState().kinematics_estimated.position.z_val)
                    vz = max(-1.5, min(1.5, (z_ref - z_now) * 0.8))
                except Exception:
                    vz = 0.0
            return self.moveByVelocityAsync(vx, vy, vz, duration)
        if backend == "moveToPosition":
            return self._move_to_position_step_async(vx=vx, vy=vy, duration=duration, z_ref=z_ref)
        if backend == "moveByRollPitchYawThrottle":
            return self._move_by_attitude_step_async(vx=vx, vy=vy, duration=duration, z_ref=z_ref)
        self._move_by_rc_fallback(
            vx=vx,
            vy=vy,
            z_ref=z_ref if z_ref is not None else -2.0,
            duration=duration,
        )
        return self._SyncJoinResult(None, wait_s=duration)

    def _probe_motion_backend(self, duration: float = 0.2) -> None:
        probe_dt = max(0.15, min(0.5, duration))
        z_ref = -2.0
        candidates = [
            "moveByVelocity",
            "moveToPosition",
            "moveByRollPitchYawThrottle",
            "moveByRC",
        ]
        best_backend = "moveByRC"
        best_score = -1.0
        for backend in candidates:
            try:
                before = self.getMultirotorState().kinematics_estimated
                x0 = float(before.position.x_val)
                y0 = float(before.position.y_val)
                if backend == "moveByVelocity":
                    self.moveByVelocityAsync(0.6, 0.0, 0.0, probe_dt).join()
                elif backend == "moveToPosition":
                    self._move_to_position_step_async(
                        vx=0.6, vy=0.0, duration=probe_dt, z_ref=z_ref
                    ).join()
                elif backend == "moveByRollPitchYawThrottle":
                    self._move_by_attitude_step_async(
                        vx=0.6, vy=0.0, duration=probe_dt, z_ref=z_ref
                    ).join()
                else:
                    self._move_by_rc_fallback(vx=0.6, vy=0.0, z_ref=z_ref, duration=probe_dt)
                    time.sleep(probe_dt)
                after = self.getMultirotorState().kinematics_estimated
                dx = float(after.position.x_val) - x0
                dy = float(after.position.y_val) - y0
                score = (dx * dx + dy * dy) ** 0.5
                print(f"Maze flight: backend probe {backend} delta=({dx:.3f},{dy:.3f}) "
                      f"score={score:.3f}")
                if score > best_score:
                    best_backend = backend
                    best_score = score
                self.hoverAsync().join()
            except Exception as exc:
                print(f"Maze flight: {backend} probe failed ({type(exc).__name__}: {exc})")
        self._motion_backend = best_backend
        print(
            f"Maze flight: selected motion backend {best_backend} "
            f"(best_probe_score={best_score:.3f})"
        )

    def _move_to_position_step_async(
        self, *, vx: float, vy: float, duration: float, z_ref: float | None
    ) -> _SyncJoinResult:
        s = self.getMultirotorState().kinematics_estimated
        x = float(s.position.x_val) + (vx * duration)
        y = float(s.position.y_val) + (vy * duration)
        z = float(s.position.z_val) if z_ref is None else float(z_ref)
        speed = max(0.5, (vx * vx + vy * vy) ** 0.5)
        timeout = max(0.2, duration)
        result = self._call_variants_cached(
            cache_key="moveToPosition",
            method="moveToPosition",
            variants=[
                ("(x,y,z,vel)", (x, y, z, speed)),
                ("(x,y,z,vel,timeout)", (x, y, z, speed, timeout)),
                ("(...,str)", (x, y, z, speed, timeout, self._vehicle_name)),
                ("(...,bytes)", (x, y, z, speed, timeout, self._vehicle_bytes)),
            ],
        )
        return self._SyncJoinResult(result, wait_s=duration)

    def _move_by_attitude_step_async(
        self, *, vx: float, vy: float, duration: float, z_ref: float | None
    ) -> _SyncJoinResult:
        z_now = float(self.getMultirotorState().kinematics_estimated.position.z_val)
        z_target = z_now if z_ref is None else float(z_ref)
        z_err = z_target - z_now
        throttle = max(0.42, min(0.78, 0.58 + (z_err * 0.08)))
        pitch = max(-0.20, min(0.20, vx * 0.08))
        roll = max(-0.20, min(0.20, vy * 0.08))
        result = self._call_variants_cached(
            cache_key="moveByRollPitchYawThrottle",
            method="moveByRollPitchYawThrottle",
            variants=[
                ("(r,p,y,t,duration)", (roll, pitch, 0.0, throttle, duration)),
                ("(...,str)", (roll, pitch, 0.0, throttle, duration, self._vehicle_name)),
                ("(...,bytes)", (roll, pitch, 0.0, throttle, duration, self._vehicle_bytes)),
            ],
        )
        return self._SyncJoinResult(result, wait_s=duration)

    def movement_backend(self) -> str:
        if self._motion_backend is None:
            self._probe_motion_backend()
        return self._motion_backend or "moveByRC"

    def simGetCollisionInfo(self):
        encoded = self._call_variants_cached(
            cache_key="simGetCollisionInfo",
            method="simGetCollisionInfo",
            variants=[
                ("()", ()),
                ("(str)", (self._vehicle_name,)),
                ("(bytes)", (self._vehicle_bytes,)),
            ],
        )
        return airsim.CollisionInfo.from_msgpack(encoded)

    def _move_with_fallback_chain(
        self, *, vx: float, vy: float, z_ref: float, duration: float
    ) -> bool:
        # 1) Try moveByVelocity without drivetrain/yaw extras.
        vz = 0.0
        try:
            z_now = float(self.getMultirotorState().kinematics_estimated.position.z_val)
            # Gentle P-control on altitude while keeping command stable in legacy RPC.
            vz = max(-1.5, min(1.5, (z_ref - z_now) * 0.8))
        except Exception:
            vz = 0.0
        try:
            self._call_variants(
                "moveByVelocity",
                [
                    ("(vx,vy,vz,duration)", (vx, vy, vz, duration)),
                    ("(...,str)", (vx, vy, vz, duration, self._vehicle_name)),
                    ("(...,bytes)", (vx, vy, vz, duration, self._vehicle_bytes)),
                ],
            )
            return True
        except Exception:
            pass

        # 2) Try moveToPosition based on current state delta.
        try:
            s = self.getMultirotorState().kinematics_estimated
            x = float(s.position.x_val) + (vx * duration)
            y = float(s.position.y_val) + (vy * duration)
            speed = max(0.5, (vx * vx + vy * vy) ** 0.5)
            self._call_variants(
                "moveToPosition",
                [
                    ("(x,y,z,vel)", (x, y, z_ref, speed)),
                    ("(x,y,z,vel,timeout)", (x, y, z_ref, speed, max(0.2, duration))),
                    ("(...,str)", (x, y, z_ref, speed, max(0.2, duration), self._vehicle_name)),
                    ("(...,bytes)", (x, y, z_ref, speed, max(0.2, duration), self._vehicle_bytes)),
                ],
            )
            return True
        except Exception:
            pass

        # 3) Final fallback: moveByRC (if this server exposes it).
        try:
            self._move_by_rc_fallback(vx=vx, vy=vy, z_ref=z_ref, duration=duration)
            return True
        except Exception:
            return False

    def _move_by_rc_fallback(self, *, vx: float, vy: float, z_ref: float, duration: float) -> None:
        s = self.getMultirotorState().kinematics_estimated
        z_now = float(s.position.z_val)
        z_err = z_ref - z_now
        # Gentle mapping from desired velocity/altitude error to manual RC channels.
        pitch = max(-0.55, min(0.55, vx * 0.28))
        roll = max(-0.55, min(0.55, vy * 0.28))
        throttle = max(0.42, min(0.78, 0.58 + (z_err * 0.08)))
        rc = airsim.RCData(
            timestamp=int(time.time() * 1000),
            pitch=pitch,
            roll=roll,
            throttle=throttle,
            yaw=0.0,
            left_z=0.0,
            right_z=0.0,
            switches=0,
            vendor_id="maze_fallback",
            is_initialized=True,
            is_valid=True,
        )
        self._call_variants(
            "moveByRC",
            [
                ("(rc)", (rc,)),
                ("(rc,str)", (rc, self._vehicle_name)),
                ("(rc,bytes)", (rc, self._vehicle_bytes)),
            ],
        )

    def moveByRollPitchYawThrottleAsync(
        self, roll: float, pitch: float, yaw: float, throttle: float, duration: float
    ):
        result = self._call_variants(
            "moveByRollPitchYawThrottle",
            [
                ("(r,p,y,t,duration)", (roll, pitch, yaw, throttle, duration)),
                ("(...,str)", (roll, pitch, yaw, throttle, duration, self._vehicle_name)),
                ("(...,bytes)", (roll, pitch, yaw, throttle, duration, self._vehicle_bytes)),
            ],
        )
        return self._SyncJoinResult(result)

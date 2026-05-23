import os
import sys
from pathlib import Path

import airsim
from src.config import apply_low_end_overrides, load_config, simulator_endpoint
from src.control.algorithms import get_algorithm, list_algorithms
from src.control.flight_client import AirSimAdapter
from src.control.highres_imu import format_highres_imu_health
from src.control.mavlink_client import PymavlinkFlightClient
from src.control.ned_environment import format_ned_environment_health
from src.control.primitives import (
    apply_trace_style,
    land_with_telemetry,
    run_algorithm_with_timeout,
    set_front_camera_pose,
    suppress_api_cleanup_warning,
    wait_until_stationary,
)
from src.internal_mapping import internal_mapping_logger_from_config
from src.log_paths import resolve_log_csv_path
from src.mavlink_endpoints import resolve_control_transport
from src.position_hud import (
    PositionOnScreenHud,
    RpcTrackingHudProvider,
    position_hud_config_from_dict,
)
from src.position_trace import position_trace_store_from_config
from src.session_logs import print_session_log_plan, warn_missing_session_logs
from src.simulator_specs import assert_specification_snapshot_if_required
from src.tracking import local_tracker_from_config
from src.vision import VisionFeed, vision_feed_from_config

ROOT = Path(__file__).resolve().parent


def _format_timesync_ns_ms(value: int | None) -> str:
    if value is None:
        return "none"
    return f"{value / 1_000_000.0:.3f}"


def _format_timesync_event(event) -> str:
    if event is None:
        return "none"
    return f"tc1={event.tc1} ts1={event.ts1}"


def _log_timesync_status(client, label: str) -> None:
    getter = getattr(client, "getTimesyncSnapshot", None)
    if not callable(getter):
        return
    snapshot = getter()
    health = snapshot.sync_health
    print(
        f"[{label}] TIMESYNC messages={snapshot.message_count} "
        f"outbound={snapshot.outbound_request_count} "
        f"matched={snapshot.matched_response_count} "
        f"pending={snapshot.pending_request_count} "
        f"health={health.status} "
        f"reason={health.reason!r} "
        f"last_request={_format_timesync_event(snapshot.last_request)} "
        f"last_response={_format_timesync_event(snapshot.last_response)} "
        f"best_offset_ms={_format_timesync_ns_ms(snapshot.estimated_offset_ns)} "
        f"best_rtt_ms={_format_timesync_ns_ms(snapshot.estimated_rtt_ns)} "
        f"stable_offset_ms={_format_timesync_ns_ms(snapshot.stable_offset_ns)} "
        f"stable_rtt_ms={_format_timesync_ns_ms(snapshot.stable_rtt_ns)} "
        f"jitter_ms={_format_timesync_ns_ms(snapshot.offset_jitter_ns)}"
    )


def _log_ned_environment_status(client, label: str) -> None:
    health_getter = getattr(client, "get_ned_environment_health", None)
    if not callable(health_getter):
        return
    health = health_getter()
    print(f"[{label}] NED {format_ned_environment_health(health)}")


def _log_highres_imu_status(client, label: str) -> None:
    health_getter = getattr(client, "getHighresImuHealth", None)
    sample_getter = getattr(client, "getHighresImu", None)
    if not callable(health_getter) or not callable(sample_getter):
        return
    health = health_getter()
    sample = sample_getter()
    sample_log = "sample=none"
    if sample is not None:
        sample_log = (
            f"sample(id={sample.sensor_id},time_usec={sample.time_usec},"
            f"xacc={sample.xacc},yacc={sample.yacc},zacc={sample.zacc},"
            f"xgyro={sample.xgyro},ygyro={sample.ygyro},zgyro={sample.zgyro})"
        )
    print(f"[{label}] HIGHRES_IMU {format_highres_imu_health(health)} {sample_log}")


def main() -> None:
    config = load_config()
    apply_low_end_overrides(config)
    assert_specification_snapshot_if_required(config)
    sim_cfg = config["simulator"]
    transport = resolve_control_transport(config)
    host, port = simulator_endpoint(config)
    profile = os.environ.get("AIGP_PROFILE", "").strip()
    map_name = str(sim_cfg.get("map_name", "")).strip()
    print(
        "Flight session: "
        f"algorithm={config.algorithm_name!r} "
        f"transport={transport!r} rpc={host}:{port}"
        + (f" profile={profile!r}" if profile else "")
        + (f" map={map_name!r}" if map_name else "")
    )

    mav_cfg = config.get("control", {}).get("mavlink", {})
    hud_cfg = position_hud_config_from_dict(
        mav_cfg.get("position_hud", {}), config=config, transport=transport
    )
    position_trace = None
    local_tracker = None
    position_hud: PositionOnScreenHud | None = None
    airsim_client: airsim.MultirotorClient | None = None
    vision_feed: VisionFeed | None = None

    landing_tel_cfg = config.get("landing", {}).get("telemetry_log", {})
    landing_csv_path = None
    if bool(landing_tel_cfg.get("enabled", False)):
        landing_csv_path = resolve_log_csv_path(
            str(landing_tel_cfg.get("path", "logs/landing_telemetry.csv")),
            ROOT,
            default="logs/landing_telemetry.csv",
        )

    if transport == "mavlink":
        position_trace = position_trace_store_from_config(config, ROOT)
        local_tracker = local_tracker_from_config(config, ROOT)
        timesync_cfg = mav_cfg.get("timesync", {})
        highres_imu_cfg = mav_cfg.get("highres_imu", {})
        endpoint = os.environ.get("AIGP_MAVLINK_ENDPOINT", "").strip() or str(
            mav_cfg.get("endpoint", "udpin:0.0.0.0:14550")
        ).strip()
        client = PymavlinkFlightClient(
            endpoint=endpoint,
            command_rate_hz=float(config.get("control", {}).get("command_rate_hz", 50.0)),
            state_request_hz=float(mav_cfg.get("state_request_hz", 20.0)),
            guided_custom_mode=int(mav_cfg.get("guided_custom_mode", 4)),
            takeoff_altitude_m=float(mav_cfg.get("takeoff_altitude_m", 5.0)),
            land_descent_speed_ms=float(config.get("landing", {}).get("descent_speed_ms", 2.0)),
            source_system=int(mav_cfg.get("source_system", 255)),
            source_component=int(mav_cfg.get("source_component", 1)),
            respond_to_timesync_requests=bool(timesync_cfg.get("respond_to_requests", True)),
            timesync_log_messages=bool(timesync_cfg.get("log_messages", True)),
            send_timesync_requests=bool(timesync_cfg.get("send_requests", True)),
            timesync_request_interval_s=float(timesync_cfg.get("request_interval_seconds", 1.0)),
            highres_imu_enabled=bool(highres_imu_cfg.get("enabled", True)),
            highres_imu_request_hz=float(
                highres_imu_cfg.get("request_hz", mav_cfg.get("state_request_hz", 20.0))
            ),
            highres_imu_log_messages=bool(highres_imu_cfg.get("log_messages", False)),
            highres_imu_max_staleness_ms=float(
                highres_imu_cfg.get("max_staleness_ms", 1000.0)
            ),
            timesync_pending_request_limit=int(timesync_cfg.get("pending_request_limit", 64)),
            timesync_stable_window_size=int(timesync_cfg.get("stable_window_size", 9)),
            timesync_stable_best_subset_size=int(timesync_cfg.get("stable_best_subset_size", 5)),
            timesync_min_stable_samples=int(timesync_cfg.get("min_stable_samples", 3)),
            timesync_max_stable_rtt_ns=int(
                float(timesync_cfg.get("max_stable_rtt_ms", 250.0)) * 1_000_000
            ),
            timesync_max_offset_jitter_ns=int(
                float(timesync_cfg.get("max_offset_jitter_ms", 50.0)) * 1_000_000
            ),
            attitude_target_throttle_body_z=bool(
                mav_cfg.get("attitude_target", {}).get("throttle_body_z", False)
            ),
            sim_config=getattr(config, "_raw", config),
            position_trace=position_trace,
            local_tracker=local_tracker,
        )
        tracker_cb = None
        if local_tracker is not None:

            def tracker_cb(image_rgb, sim_time_ns: int) -> None:
                local_tracker.on_video_frame(image_rgb, sim_time_ns)

        udp_video_enabled = bool(
            config.get("vision", {}).get("udp_video", {}).get("enabled", False)
        )
        allow_airsim_vision = (
            bool(config.get("vision", {}).get("enabled", False))
            and os.environ.get("AIGP_ENABLE_AIRSIM_VISION", "").strip() == "1"
        )
        if udp_video_enabled or local_tracker is not None:
            vision_feed = vision_feed_from_config(config, tracker_callback=tracker_cb)
        elif allow_airsim_vision:
            airsim_client = airsim.MultirotorClient(ip=host, port=port)
            vision_feed = VisionFeed(airsim_client, config.get("vision", {}))
        else:
            config.setdefault("vision", {})["enabled"] = False
    else:
        airsim_client = airsim.MultirotorClient(ip=host, port=port)
        client = AirSimAdapter(
            airsim_client,
            command_rate_hz=float(config.get("control", {}).get("command_rate_hz", 50.0)),
            sim_config=getattr(config, "_raw", config),
        )
        vision_feed = VisionFeed(airsim_client, config.get("vision", {}))

    explore_cfg = config.get("autonomous_explore", {}).get("exploration", {})
    explore_slam_cfg = explore_cfg.get("slam", {})
    internal_mapping_logger = internal_mapping_logger_from_config(config, ROOT, client)
    slam_export_path = str(
        explore_slam_cfg.get("export_path", "logs/slam/exploration_map_{timestamp}.json")
    )
    slam_export_enabled = bool(explore_slam_cfg.get("export_enabled", True))
    internal_mapping_enabled = internal_mapping_logger is not None
    internal_mapping_path = (
        str(internal_mapping_logger.out_path) if internal_mapping_logger is not None else None
    )
    print_session_log_plan(
        transport=transport,
        env_transport=os.environ.get("AIGP_CONTROL_TRANSPORT", ""),
        landing_csv_path=landing_csv_path,
        position_trace_path=position_trace.out_path if position_trace else None,
        local_tracker_path=local_tracker.csv_path if local_tracker else None,
        slam_export_path=slam_export_path,
        slam_export_enabled=slam_export_enabled,
        internal_mapping_enabled=internal_mapping_enabled,
        internal_mapping_path=internal_mapping_path,
    )

    try:
        client.confirmConnection()
        if transport == "airsim":
            try:
                client.reset()
            except Exception as reset_exc:
                print(f"Warning: client.reset() failed (continuing): {reset_exc}", file=sys.stderr)
        client.enableApiControl(True)
        client.armDisarm(True)
        if transport == "airsim":
            wait_until_stationary(client)
        set_front_camera_pose(client, config)
        hud_provider = None
        if hud_cfg.enabled:
            if transport == "mavlink":
                if hud_cfg.data_source == "tracking":
                    if local_tracker is None:
                        print(
                            "Warning: position_hud.data_source=tracking requires "
                            "control.mavlink.tracking.enabled; HUD will stay idle.",
                            file=sys.stderr,
                        )
                    elif hasattr(client, "getTrackingSnapshot"):
                        hud_provider = client.getTrackingSnapshot
                elif position_trace is not None:
                    hud_provider = client.getPositionTraceSnapshot
                else:
                    print(
                        "Warning: position_hud.enabled requires position trace or tracking; "
                        "enable control.mavlink.tracking (data_source=tracking) or "
                        "position_trace (data_source=trace).",
                        file=sys.stderr,
                    )
            elif hud_cfg.data_source == "trace" or transport != "mavlink":
                if hud_cfg.data_source == "tracking" and transport != "mavlink":
                    print(
                        "Warning: position_hud.data_source=tracking requires MAVLink; "
                        "using AirSim RPC position HUD.",
                        file=sys.stderr,
                    )
                hud_provider = RpcTrackingHudProvider(client)
            else:
                print(
                    "Warning: position_hud.data_source=tracking requires MAVLink transport; "
                    "use data_source=trace for AirSim RPC.",
                    file=sys.stderr,
                )
        # #region agent log
        try:
            import json
            import time
            from pathlib import Path as _Path

            _payload = {
                "sessionId": "f6d05d",
                "hypothesisId": "A",
                "location": "main.py:hud_wiring",
                "message": "hud provider resolved",
                "data": {
                    "transport": transport,
                    "data_source": hud_cfg.data_source,
                    "hud_enabled": hud_cfg.enabled,
                    "hud_provider_set": hud_provider is not None,
                    "local_tracker_set": local_tracker is not None,
                },
                "timestamp": int(time.time() * 1000),
            }
            with (_Path(__file__).resolve().parent / "debug-f6d05d.log").open(
                "a", encoding="utf-8"
            ) as _fh:
                _fh.write(json.dumps(_payload, default=str) + "\n")
        except OSError:
            pass
        # #endregion
        if hud_provider is not None:
            position_hud = PositionOnScreenHud(
                host=host,
                port=port,
                snapshot_provider=hud_provider,
                config=hud_cfg,
            )
            position_hud.start()
            print(
                f"On-screen HUD started ({hud_cfg.data_source}, "
                f"{hud_cfg.update_hz:.0f} Hz via AirSim RPC)."
            )
        apply_trace_style(client, config)
        if airsim_client is not None and airsim_client is not client:
            airsim_client.confirmConnection()
            set_front_camera_pose(airsim_client, config)
            apply_trace_style(airsim_client, config)
        _log_timesync_status(client, "startup")
        _log_highres_imu_status(client, "startup")
        if transport == "mavlink":
            _log_ned_environment_status(client, "startup")
        if vision_feed is not None and vision_feed.enabled:
            vision_feed.start()
            udp_port = config.get("vision", {}).get("udp_video", {}).get("port", 5600)
            if transport == "mavlink" and bool(
                config.get("vision", {}).get("udp_video", {}).get("enabled", False)
            ):
                print(f"UDP vision feed started (port {udp_port})")
        if local_tracker is not None:
            health = local_tracker.health()
            print(
                f"[tracking] status={health.status} reason={health.reason!r} "
                f"csv={local_tracker.csv_path}"
            )
        if internal_mapping_logger is not None:
            internal_mapping_logger.start()
            print(f"[startup] internal_mapping CSV: {internal_mapping_logger.out_path}")

        try:
            algo_name = config.algorithm_name
            algo = get_algorithm(algo_name, config)
            algo.set_vision_feed(
                vision_feed if vision_feed is not None and vision_feed.enabled else None
            )
            safety_cfg = config.get("safety", {})
            algo_timeout_seconds = max(
                5.0, float(safety_cfg.get("algorithm_timeout_seconds", 180.0))
            )

            print(f"Algorithm: {algo_name} (available: {', '.join(list_algorithms())})")
            run_algorithm_with_timeout(algo, client, algo_timeout_seconds)

            print("Algorithm complete. Starting landing sequence...")
            land_with_telemetry(client, config, label="main")
            print("Flight client finished normally (landing complete).", file=sys.stderr)
        except Exception as exc:
            print(f"Failsafe triggered: {exc}")
            print("Attempting hover and landing for safe recovery...")
            land_with_telemetry(client, config, label="main")
    finally:
        if internal_mapping_logger is not None:
            internal_mapping_logger.stop()
            print(
                f"[shutdown] internal_mapping path={internal_mapping_logger.out_path} "
                f"rows={internal_mapping_logger.row_count}"
            )
        if position_hud is not None:
            position_hud.stop()
            print(
                f"[shutdown] position_hud updates={position_hud.update_count} "
                f"rpc_errors={position_hud.rpc_error_count}"
            )
        if vision_feed is not None:
            vision_feed.stop()
            if vision_feed.enabled:
                stats = vision_feed.get_stats()
                print(
                    f"[shutdown] vision_feed successes={stats.capture_successes} "
                    f"udp_frames={stats.udp_frames} udp_packets={stats.udp_packets}"
                )
        _log_timesync_status(client, "shutdown")
        _log_highres_imu_status(client, "shutdown")
        if transport == "mavlink":
            _log_ned_environment_status(client, "shutdown")
        if position_trace is not None:
            trace_health = position_trace.health()
            print(
                f"[shutdown] position_trace accepted={trace_health.accepted_count} "
                f"rejected={trace_health.rejected_count} path={position_trace.out_path}"
            )
        if local_tracker is not None:
            local_tracker.flush()
            health = local_tracker.health()
            print(
                f"[shutdown] tracking imu_samples={health.imu_sample_count} "
                f"imu_rate_hz={health.imu_rate_hz} "
                f"vision_corrections={health.vision_correction_count} "
                f"path={local_tracker.csv_path}"
            )
        try:
            client.armDisarm(False)
            client.enableApiControl(False)
        except Exception as cleanup_exc:
            if not suppress_api_cleanup_warning(cleanup_exc):
                print(
                    "Warning: API cleanup failed (often harmless if sim/editor already "
                    f"closed): {cleanup_exc}",
                    file=sys.stderr,
                )
        closer = getattr(client, "close", None)
        if callable(closer):
            closer()

        warn_missing_session_logs(
            transport=transport,
            landing_csv_path=landing_csv_path,
            position_trace_path=position_trace.out_path if position_trace else None,
            internal_mapping_path=(
                internal_mapping_logger.out_path if internal_mapping_logger is not None else None
            ),
            logs_dir=ROOT / "logs",
        )

    if os.environ.get("AIGP_PAUSE_BEFORE_EXIT", "").strip() == "1":
        input("AIGP_PAUSE_BEFORE_EXIT=1 - press Enter to exit the flight client...")


if __name__ == "__main__":
    main()

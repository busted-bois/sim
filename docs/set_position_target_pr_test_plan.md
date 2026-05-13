# SET_POSITION_TARGET_LOCAL_NED PR Test Plan

Use this checklist before merging changes related to MAVLink local-NED setpoint submission.

## Unit and integration checks

- [ ] `uvx ruff check --fix src/control/flight_client.py src/control/mavlink_client.py src/control/algorithms/six_directions.py tests/mavlink_fakes.py tests/test_mavlink_set_position_target_local_ned.py tests/test_mavlink_set_position_target_local_ned_integration.py tests/test_mavlink_client_timesync.py tests/test_mavlink_client_highres_imu.py`
- [ ] `uv run -m unittest tests.test_mavlink_set_position_target_local_ned`
- [ ] `uv run -m unittest tests.test_mavlink_set_position_target_local_ned_integration` (skipped if `AIGP_SKIP_MAVLINK_INTEGRATION=1`)
- [ ] `uv run -m unittest tests.test_mavlink_client_timesync tests.test_mavlink_client_highres_imu`

Expected result: all tests pass with `OK` (integration may show as `skipped` when skip env is set).

## Behavior checks

- [ ] LOCAL_NED and BODY_NED helpers each emit `SET_POSITION_TARGET_LOCAL_NED` with the expected `coordinate_frame`.
- [ ] Legacy `moveByVelocityAsync` still emits the same velocity-only semantics through the shared setpoint sender.
- [ ] `streamSetPositionTargetLocalNedAsync` repeats the same command at `command_rate_hz` for the requested duration.
- [ ] `build_velocity_type_mask()` and `build_position_type_mask()` match expected bitmask defaults.
- [ ] `FlightClient` protocol includes submit/stream helpers; `AirSimAdapter` raises `NotImplementedError` when the wrapped client lacks MAVLink.

## Optional simulator check

- [ ] Set `sim.config.json -> algorithm = "six_directions"` and `six_directions.mavlink_setpoint_demo_enabled = true`.
- [ ] Run with MAVLink transport and verify the vehicle responds to 6-direction motion while using the stream-based setpoint path.

## CI note

Set `AIGP_SKIP_MAVLINK_INTEGRATION=1` in environments where UDP loopback or timing is unreliable; document that in the PR if used.

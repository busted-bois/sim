# SET_POSITION_TARGET_LOCAL_NED PR Test Plan

Use this checklist before merging changes related to MAVLink local-NED setpoint submission.

## Unit and integration checks

- [ ] `uvx ruff check --fix src/control/flight_client.py src/control/mavlink_client.py tests/test_mavlink_set_position_target_local_ned.py tests/test_mavlink_set_position_target_local_ned_integration.py`
- [ ] `uv run -m unittest tests.test_mavlink_set_position_target_local_ned`
- [ ] `uv run -m unittest tests.test_mavlink_set_position_target_local_ned_integration`
- [ ] `uv run -m unittest tests.test_mavlink_client_timesync tests.test_mavlink_client_highres_imu`

Expected result: all tests pass with `OK`.

## Behavior checks

- [ ] LOCAL_NED and BODY_NED helpers each emit `SET_POSITION_TARGET_LOCAL_NED` with the expected `coordinate_frame`.
- [ ] Legacy `moveByVelocityAsync` still emits the same velocity-only semantics through the shared setpoint sender.
- [ ] `build_velocity_type_mask()` and `build_position_type_mask()` match expected bitmask defaults.

## Optional simulator check

- [ ] Set `sim.config.json -> algorithm = "six_directions"` and `six_directions.mavlink_setpoint_demo_enabled = true`.
- [ ] Run with MAVLink transport and verify the vehicle responds to 6-direction motion while using the submit helper path.

# ABI Guide (`motor_abi`)

## Build

```bash
cargo build -p motor_abi --release
```

Artifacts:

- Linux: `target/release/libmotor_abi.so`, `libmotor_abi.a`
- Windows: `target/release/motor_abi.dll`, `motor_abi.lib`
- Header: `motor_abi/include/motor_abi.h`

## Unified Surface

The ABI keeps one unified control surface across vendors.

## ABI Metadata

- `motor_abi_version()` returns the loaded ABI library version string.
- `motor_abi_capabilities_json()` returns a stable JSON capability document
  describing supported transports, vendors, controller lifecycle operations,
  control modes, and vendor-specific features.
- Python exposes the same information through `motorbridge.abi_version()` and
  `motorbridge.abi_capabilities()`.
- C++ exposes the same information through `motorbridge::abi_version()` and
  `motorbridge::abi_capabilities_json()`.
- Binding parity is tracked in `bindings/api_surface.json`.

Unified mode IDs (`motor_handle_ensure_mode`):

- `1 = MIT`
- `2 = POS_VEL`
- `3 = VEL`
- `4 = FORCE_POS`

Unified control units:

- position: `rad`
- velocity: `rad/s`
- torque: `Nm`

Common control/state APIs:

- `motor_handle_enable`
- `motor_handle_disable`
- `motor_handle_clear_error`
- `motor_handle_set_zero_position`
- `motor_handle_ensure_mode`
- `motor_handle_send_mit`
- `motor_handle_send_pos_vel`
- `motor_handle_send_vel`
- `motor_handle_send_force_pos`
- `motor_handle_request_feedback`
- `motor_handle_set_can_timeout_ms`
- `motor_handle_store_parameters`
- `motor_handle_get_state`
- `motor_handle_get_state_with_age` (Damiao)

`motor_handle_get_state_with_age(motor, &state, &age_us, &seq)` returns the
cached state together with its age (µs since the state frame arrived) and a
sequence number (state frames received since the motor was added), read under
one lock. Damiao motors answer every control frame (MIT, pos-vel, ...) and
every feedback request with a state frame, all cached in the same place, so a
caller that sends a control frame every cycle can skip
`motor_handle_request_feedback` and check the age instead. Like the cached
register getters it has three return codes, so do not test `rc != 0` or
`rc < 0`: `0` (state, age and seq written), `1` (no state frame yet; `state` is
written with `has_value = 0`, nothing else) and `-1` (error: null pointer or a
non-Damiao motor). Capability flag: `state_age`. The age counts from when the
frame was processed into the cache: with the background poller and
`motor_handle_poll_feedback_once` both draining the bus, a frame can be
processed after a newer one from the same motor, so the state can be one frame
older than its age says (an age older than some instant still means that no
state frame at all was processed since).

## Vendor Entry Points

- Damiao: `motor_controller_add_damiao_motor(...)`
- Hexfellow: `motor_controller_add_hexfellow_motor(...)` (CAN-FD path via `socketcanfd`)
- RobStride: `motor_controller_add_robstride_motor(...)`
- MyActuator: `motor_controller_add_myactuator_motor(...)`
- HighTorque: `motor_controller_add_hightorque_motor(...)`

## Unified Mode to Native Protocol Mapping

| Unified mode | Damiao native | Hexfellow native | RobStride native | MyActuator native | HighTorque native |
|---|---|---|---|---|---|
| `MIT` | `Mit` | mode `5` | `Mit` | not available | mapped to native pos+vel+tqe |
| `POS_VEL` | `PosVel` | mode `1` | mapped to `Position` (`run_mode=1`, `limit_spd=0x7017`, `loc_ref=0x7016`) | `Position` setpoint flow | mapped to native pos+vel+tqe |
| `VEL` | `Vel` | not available | `Velocity` | `Velocity` setpoint flow | mapped to native velocity command |
| `FORCE_POS` | `ForcePos` | not available | not available | not available | mapped to native pos+vel+tqe |

Behavior rule:

- Unsupported calls return non-zero and a readable message via `motor_last_error_message()`.
- Signatures stay stable even when a vendor ignores part of a signature.
- Example: HighTorque accepts `send_mit(pos, vel, kp, kd, tau)` for interface consistency, but native protocol does not use `kp/kd`.
- Hexfellow ABI path supports MIT and POS_VEL, and reports `VEL` / `FORCE_POS` as unsupported.
- RobStride supports MIT / POS_VEL / VEL on unified APIs; torque/current remains parameter-level (`robstride_write_param_*`), not a unified mode.
- Damiao set-zero sequence rule: call `motor_handle_disable` before `motor_handle_set_zero_position`; otherwise set-zero is rejected by core guard.
- Damiao set-zero settle rule: core applies an internal fixed settle (`~20ms`) after successful `set_zero_position` (no extra ABI parameter).

## Vendor-Specific Extensions

Damiao register APIs:

- `motor_handle_write_register_f32`
- `motor_handle_write_register_u32`
- `motor_handle_get_register_f32`
- `motor_handle_get_register_u32`
- `motor_handle_request_register`
- `motor_handle_get_cached_register_f32`
- `motor_handle_get_cached_register_u32`

Non-blocking register reads: call `motor_handle_request_register` on each motor,
poll feedback (`motor_controller_poll_feedback_once`), then read
`motor_handle_get_cached_register_f32/u32(motor, rid, &value, &age_us, &fresh)`.
Unlike every other ABI call, these getters have three return codes, so do not
test `rc != 0` or `rc < 0`:

- `0`: a reply is cached; `value`, `age_us` (µs since it arrived) and `fresh`
  are written. `fresh` is 1 if the reply answers the most recent request and 0
  if it is left over from an older one (for example, the latest request was
  lost). Check `fresh` before treating `value` as current.
- `1`: no reply has ever arrived for this register; nothing is written.
- `-1`: error (null pointer, non-Damiao motor, or wrong register type).

RobStride extensions:

- `motor_handle_robstride_ping`
- `motor_handle_robstride_ping_host_id`
- `motor_handle_robstride_set_device_id`
- `motor_handle_robstride_set_active_report`
- `motor_handle_robstride_get_fault_report`
- `motor_handle_robstride_get_param_f32_host_id`
- `motor_handle_robstride_write_param_i8/u8/u16/u32/f32`
- `motor_handle_robstride_get_param_i8/u8/u16/u32/f32`

## Typical Call Flow

1. Transport constructor:
   - `motor_controller_new_socketcan(channel)` (general path)
   - `motor_controller_new_socketcanfd(channel)` (CAN-FD path; required by Hexfellow)
   - `motor_controller_new_dm_serial(serial_port, baud)` (Damiao-only serial bridge; cross-platform, e.g. `/dev/ttyACM0` or `COM3`)
   - `motor_controller_new_dm_device(dm_device_type, dm_channel)` (Damiao-only DM_Device SDK path; e.g. `usb2canfd` + `0`, `usb2canfd-dual` + `0|1`, or `linkx4c` + `0|1|2|3`)
2. `motor_controller_add_<vendor>_motor`
3. optional: `motor_controller_enable_all`
4. optional: `motor_handle_ensure_mode`
5. send control commands / read state / vendor-specific operations
6. `motor_controller_shutdown`
   - or `motor_controller_close_bus` when only closing the local session/bus
7. `motor_handle_free`
8. `motor_controller_free`

## Examples

- C ABI demo: `examples/c/c_abi_demo.c`
- C++ ABI demo: `examples/cpp/cpp_abi_demo.cpp`
- Python ctypes demo: `examples/python/python_ctypes_demo.py`

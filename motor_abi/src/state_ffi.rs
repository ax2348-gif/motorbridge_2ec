use super::*;

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_get_state(
    motor: *mut MotorHandle,
    out_state: *mut MotorState,
) -> i32 {
    if motor.is_null() || out_state.is_null() {
        set_last_error("motor or out_state is null");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    let out = unsafe { &mut *out_state };
    match &*motor {
        MotorHandleInner::Damiao(m) => {
            if let Some(state) = m.latest_state() {
                *out = MotorState {
                    has_value: 1,
                    can_id: state.can_id,
                    arbitration_id: state.arbitration_id,
                    status_code: state.status_code,
                    pos: state.pos,
                    vel: state.vel,
                    torq: state.torq,
                    t_mos: state.t_mos,
                    t_rotor: state.t_rotor,
                };
            } else {
                *out = MotorState::default();
            }
        }
        MotorHandleInner::Hexfellow(m) => match m.query_status(Duration::from_millis(20)) {
            Ok(state) => {
                *out = MotorState {
                    has_value: 1,
                    can_id: m.motor_id as u8,
                    arbitration_id: 0,
                    status_code: state.heartbeat_state.unwrap_or(0),
                    pos: state.position_rev * (2.0 * PI),
                    vel: state.velocity_rev_s * (2.0 * PI),
                    torq: state.torque_permille as f32 / 1000.0,
                    t_mos: 0.0,
                    t_rotor: 0.0,
                };
            }
            Err(e) => {
                set_last_error(e.to_string());
                return -1;
            }
        },
        MotorHandleInner::MyActuator(m) => {
            if let Some(state) = m.latest_state() {
                *out = MotorState {
                    has_value: 1,
                    can_id: m.motor_id as u8,
                    arbitration_id: state.arbitration_id,
                    status_code: state.command,
                    pos: state.shaft_angle_deg * (PI / 180.0),
                    vel: state.speed_dps * (PI / 180.0),
                    torq: state.current_a,
                    t_mos: f32::from(state.temperature_c),
                    t_rotor: 0.0,
                };
            } else {
                *out = MotorState::default();
            }
        }
        MotorHandleInner::Robstride(m) => {
            if let Some(state) = m.latest_state() {
                let mut status = 0u8;
                if state.uncalibrated {
                    status |= 1 << 5;
                }
                if state.stall {
                    status |= 1 << 4;
                }
                if state.magnetic_encoder_fault {
                    status |= 1 << 3;
                }
                if state.overtemperature {
                    status |= 1 << 2;
                }
                if state.overcurrent {
                    status |= 1 << 1;
                }
                if state.undervoltage {
                    status |= 1;
                }
                *out = MotorState {
                    has_value: 1,
                    can_id: state.device_id,
                    arbitration_id: state.arbitration_id,
                    status_code: status,
                    pos: state.position,
                    vel: state.velocity,
                    torq: state.torque,
                    t_mos: state.temperature_c,
                    t_rotor: 0.0,
                };
            } else {
                *out = MotorState::default();
            }
        }
        MotorHandleInner::Hightorque(m) => {
            if let Some(state) = m.latest_state() {
                *out = MotorState {
                    has_value: 1,
                    can_id: state.can_id,
                    arbitration_id: state.arbitration_id,
                    status_code: state.status_code,
                    pos: state.pos,
                    vel: state.vel,
                    torq: state.torq,
                    t_mos: state.t_mos,
                    t_rotor: state.t_rotor,
                };
            } else {
                *out = MotorState::default();
            }
        }
    }
    0
}

/// The cached state with its age and sequence number (Damiao only), in one
/// lock so the three always describe the same state frame. Nothing is sent
/// and nothing is waited for.
///
/// Damiao motors answer every control frame (MIT, ...) and every feedback
/// request with a state frame, all cached in the same place; the age shows
/// whether that cache is current, so a caller sending control frames can skip
/// `motor_handle_request_feedback` while the age stays small.
///
/// The age is measured from when the frame was processed (decoded into the
/// cache), not from when it was received: with the background poller and
/// `motor_handle_poll_feedback_once` both draining the bus, a frame can be
/// processed after a newer one from the same motor, so the cached state can be
/// one frame older than its age says. The timestamp is always the latest
/// processing time, though: an age older than some instant means no state
/// frame at all has been processed since.
///
/// Return codes (note: 1 is NOT an error, so don't test `rc != 0` or `rc < 0`):
///   0  a state frame is cached: writes `out_state` (`has_value` = 1),
///      `out_age_us` (µs since it arrived) and `out_seq` (state frames received
///      since the motor was added, this one included)
///   1  no state frame has arrived yet: `out_state` is written with
///      `has_value` = 0; `out_age_us` and `out_seq` are not written
///  -1  error (null pointer or non-Damiao motor)
#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_get_state_with_age(
    motor: *mut MotorHandle,
    out_state: *mut MotorState,
    out_age_us: *mut u64,
    out_seq: *mut u64,
) -> i32 {
    if out_state.is_null() || out_age_us.is_null() || out_seq.is_null() {
        set_last_error("out_state, out_age_us or out_seq is null");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    let cached = match &*motor {
        MotorHandleInner::Damiao(m) => m.cached_state().map_err(|e| e.to_string()),
        _ => Err("state age is only available for Damiao motors".to_string()),
    };
    match cached {
        Ok(Some(c)) => {
            let s = c.state;
            unsafe {
                *out_state = MotorState {
                    has_value: 1,
                    can_id: s.can_id,
                    arbitration_id: s.arbitration_id,
                    status_code: s.status_code,
                    pos: s.pos,
                    vel: s.vel,
                    torq: s.torq,
                    t_mos: s.t_mos,
                    t_rotor: s.t_rotor,
                };
                *out_age_us = c.age.as_micros().min(u64::MAX as u128) as u64;
                *out_seq = c.seq;
            }
            0
        }
        Ok(None) => {
            unsafe { *out_state = MotorState::default() };
            1
        }
        Err(e) => {
            set_last_error(e);
            -1
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use motor_core::bus::CanFrame;
    use motor_core::device::MotorDevice;
    use motor_core::test_support::MockBus;

    fn damiao_handle() -> (Arc<DamiaoMotor>, *mut MotorHandle) {
        let bus: Arc<dyn CanBus> = Arc::new(MockBus::new());
        let motor = Arc::new(DamiaoMotor::new(0x01, 0x11, "4340P", bus).expect("create motor"));
        let handle = Box::into_raw(Box::new(MotorHandle {
            inner: Mutex::new(MotorHandleInner::Damiao(Arc::clone(&motor))),
        }));
        (motor, handle)
    }

    fn state_frame(motor: &DamiaoMotor, t_mos: u8) {
        motor
            .process_feedback_frame(CanFrame {
                arbitration_id: 0x11,
                data: [0x11, 0x80, 0x00, 0x80, 0x08, 0x00, t_mos, 30],
                dlc: 8,
                is_extended: false,
                is_rx: true,
            })
            .expect("process state frame");
    }

    fn last_error() -> String {
        LAST_ERROR.with(|slot| slot.borrow().to_string_lossy().into_owned())
    }

    fn free(handle: *mut MotorHandle) {
        drop(unsafe { Box::from_raw(handle) });
    }

    #[test]
    fn state_with_age_round_trip() {
        let (motor, h) = damiao_handle();
        let mut st = MotorState { has_value: 7, ..MotorState::default() };
        let (mut age, mut seq) = (99u64, 99u64);

        assert_eq!(motor_handle_get_state_with_age(h, &mut st, &mut age, &mut seq), 1);
        assert_eq!((st.has_value, age, seq), (0, 99, 99), "no state yet: only has_value = 0 written");

        state_frame(&motor, 35);
        assert_eq!(motor_handle_get_state_with_age(h, &mut st, &mut age, &mut seq), 0);
        assert_eq!((st.has_value, st.status_code, st.t_mos, seq), (1, 1, 35.0, 1));
        assert!(age < 1_000_000);
        // The same state as the plain getter.
        let mut plain = MotorState::default();
        assert_eq!(motor_handle_get_state(h, &mut plain), 0);
        assert_eq!((plain.pos, plain.vel, plain.t_mos), (st.pos, st.vel, st.t_mos));

        std::thread::sleep(Duration::from_millis(3));
        assert_eq!(motor_handle_get_state_with_age(h, &mut st, &mut age, &mut seq), 0);
        assert!(age >= 3_000 && seq == 1);

        state_frame(&motor, 36);
        assert_eq!(motor_handle_get_state_with_age(h, &mut st, &mut age, &mut seq), 0);
        assert_eq!((st.t_mos, seq), (36.0, 2));
        assert!(age < 3_000);
        free(h);
    }

    #[test]
    fn state_with_age_error_branches() {
        let (_motor, h) = damiao_handle();
        let mut st = MotorState::default();
        let (mut age, mut seq) = (0u64, 0u64);
        assert_eq!(motor_handle_get_state_with_age(ptr::null_mut(), &mut st, &mut age, &mut seq), -1);
        assert_eq!(motor_handle_get_state_with_age(h, ptr::null_mut(), &mut age, &mut seq), -1);
        assert_eq!(motor_handle_get_state_with_age(h, &mut st, ptr::null_mut(), &mut seq), -1);
        assert_eq!(motor_handle_get_state_with_age(h, &mut st, &mut age, ptr::null_mut()), -1);
        free(h);

        let bus: Arc<dyn CanBus> = Arc::new(MockBus::new());
        let rs = Arc::new(RobstrideMotor::new(0x01, 0xFD, "rs-00", bus).expect("create robstride"));
        let h = Box::into_raw(Box::new(MotorHandle {
            inner: Mutex::new(MotorHandleInner::Robstride(rs)),
        }));
        assert_eq!(motor_handle_get_state_with_age(h, &mut st, &mut age, &mut seq), -1);
        assert!(last_error().contains("only available for Damiao"));
        free(h);
    }
}

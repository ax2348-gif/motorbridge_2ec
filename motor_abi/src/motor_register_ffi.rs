use super::*;

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_set_can_timeout_ms(motor: *mut MotorHandle, timeout_ms: u32) -> i32 {
    if motor.is_null() {
        set_last_error("motor is null");
        return -1;
    }
    let reg_value = timeout_ms.saturating_mul(20);
    let motor = lock_motor_inner!(motor, "motor is null");
    let rc = match &*motor {
        MotorHandleInner::Damiao(m) => m
            .write_register_u32(9, reg_value)
            .map_err(|e| e.to_string()),
        MotorHandleInner::Hexfellow(_) => {
            Err("set_can_timeout_ms is not supported for Hexfellow".to_string())
        }
        MotorHandleInner::MyActuator(_) => {
            Err("set_can_timeout_ms is not supported for MyActuator".to_string())
        }
        MotorHandleInner::Robstride(m) => m
            .write_parameter(0x7028, ParameterValue::U32(timeout_ms))
            .map_err(|e| e.to_string()),
        MotorHandleInner::Hightorque(_) => {
            Err("set_can_timeout_ms is not supported for HighTorque".to_string())
        }
    };
    ffi_rc(rc)
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_write_register_f32(
    motor: *mut MotorHandle,
    rid: u8,
    value: f32,
) -> i32 {
    if motor.is_null() {
        set_last_error("motor is null");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    let rc = match &*motor {
        MotorHandleInner::Damiao(m) => m.write_register_f32(rid, value).map_err(|e| e.to_string()),
        _ => Err("Damiao register write is only available for Damiao motors".to_string()),
    };
    ffi_rc(rc)
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_write_register_u32(
    motor: *mut MotorHandle,
    rid: u8,
    value: u32,
) -> i32 {
    if motor.is_null() {
        set_last_error("motor is null");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    let rc = match &*motor {
        MotorHandleInner::Damiao(m) => m.write_register_u32(rid, value).map_err(|e| e.to_string()),
        _ => Err("Damiao register write is only available for Damiao motors".to_string()),
    };
    ffi_rc(rc)
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_get_register_f32(
    motor: *mut MotorHandle,
    rid: u8,
    timeout_ms: u32,
    out_value: *mut f32,
) -> i32 {
    if motor.is_null() || out_value.is_null() {
        set_last_error("motor or out_value is null");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    let out = unsafe { &mut *out_value };
    let rc = match &*motor {
        MotorHandleInner::Damiao(m) => m
            .get_register_f32(rid, Duration::from_millis(timeout_ms as u64))
            .map_err(|e| e.to_string())
            .map(|v| *out = v),
        _ => Err("Damiao register read is only available for Damiao motors".to_string()),
    };
    ffi_rc(rc)
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_get_register_u32(
    motor: *mut MotorHandle,
    rid: u8,
    timeout_ms: u32,
    out_value: *mut u32,
) -> i32 {
    if motor.is_null() || out_value.is_null() {
        set_last_error("motor or out_value is null");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    let out = unsafe { &mut *out_value };
    let rc = match &*motor {
        MotorHandleInner::Damiao(m) => m
            .get_register_u32(rid, Duration::from_millis(timeout_ms as u64))
            .map_err(|e| e.to_string())
            .map(|v| *out = v),
        _ => Err("Damiao register read is only available for Damiao motors".to_string()),
    };
    ffi_rc(rc)
}

/// Send a Damiao register-read request without waiting for the reply.
/// The reply is cached when feedback is polled; read it with
/// `motor_handle_get_cached_register_f32/u32`.
#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_request_register(motor: *mut MotorHandle, rid: u8) -> i32 {
    let motor = lock_motor_inner!(motor, "motor is null");
    let rc = match &*motor {
        MotorHandleInner::Damiao(m) => m.request_register_reading(rid).map_err(|e| e.to_string()),
        _ => Err("Damiao register read is only available for Damiao motors".to_string()),
    };
    ffi_rc(rc)
}

/// Shared body of the cached-register getters.
///
/// Return codes (note: 1 is NOT an error, so don't test `rc != 0` or `rc < 0`):
///   0  a reply is cached: writes the value, its age (µs since it arrived) and
///      `out_fresh` (1 if it answers the most recent request, 0 if it is left
///      over from an older one, e.g. because the latest request was lost)
///   1  no reply has ever arrived for this register; nothing is written
///  -1  error (null pointer, non-Damiao motor, or a register of the other type)
fn get_cached_register<T>(
    motor: *mut MotorHandle,
    rid: u8,
    out_value: *mut T,
    out_age_us: *mut u64,
    out_fresh: *mut u8,
    extract: fn(motor_vendor_damiao::RegisterValue) -> Option<T>,
    type_name: &str,
) -> i32 {
    if out_value.is_null() || out_age_us.is_null() || out_fresh.is_null() {
        set_last_error("out_value, out_age_us or out_fresh is null");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    let cached = match &*motor {
        MotorHandleInner::Damiao(m) => m.cached_register_value(rid).map_err(|e| e.to_string()),
        _ => Err("Damiao register read is only available for Damiao motors".to_string()),
    };
    match cached {
        Ok(Some(cached)) => match extract(cached.value) {
            Some(v) => {
                unsafe {
                    *out_value = v;
                    *out_age_us = cached.age.as_micros().min(u64::MAX as u128) as u64;
                    *out_fresh = u8::from(cached.fresh);
                }
                0
            }
            None => {
                set_last_error(format!("register {rid} is not {type_name}"));
                -1
            }
        },
        Ok(None) => 1,
        Err(e) => {
            set_last_error(e);
            -1
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_get_cached_register_f32(
    motor: *mut MotorHandle,
    rid: u8,
    out_value: *mut f32,
    out_age_us: *mut u64,
    out_fresh: *mut u8,
) -> i32 {
    get_cached_register(motor, rid, out_value, out_age_us, out_fresh, |v| match v {
        motor_vendor_damiao::RegisterValue::Float(f) => Some(f),
        motor_vendor_damiao::RegisterValue::UInt32(_) => None,
    }, "float")
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_get_cached_register_u32(
    motor: *mut MotorHandle,
    rid: u8,
    out_value: *mut u32,
    out_age_us: *mut u64,
    out_fresh: *mut u8,
) -> i32 {
    get_cached_register(motor, rid, out_value, out_age_us, out_fresh, |v| match v {
        motor_vendor_damiao::RegisterValue::UInt32(u) => Some(u),
        motor_vendor_damiao::RegisterValue::Float(_) => None,
    }, "u32")
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_robstride_ping(
    motor: *mut MotorHandle,
    out_device_id: *mut u8,
    out_responder_id: *mut u8,
) -> i32 {
    if motor.is_null() || out_device_id.is_null() || out_responder_id.is_null() {
        set_last_error("motor or output pointer is null");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    let rc = match &*motor {
        MotorHandleInner::Robstride(m) => m
            .ping(Duration::from_millis(500))
            .map_err(|e| e.to_string()),
        MotorHandleInner::Damiao(_)
        | MotorHandleInner::Hexfellow(_)
        | MotorHandleInner::MyActuator(_)
        | MotorHandleInner::Hightorque(_) => {
            Err("robstride_ping requires a RobStride motor".to_string())
        }
    };
    match rc {
        Ok(reply) => {
            unsafe {
                *out_device_id = reply.device_id;
                *out_responder_id = reply.responder_id;
            }
            0
        }
        Err(e) => {
            set_last_error(e);
            -1
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_robstride_ping_host_id(
    motor: *mut MotorHandle,
    host_id: u16,
    timeout_ms: u32,
    out_device_id: *mut u8,
    out_responder_id: *mut u8,
) -> i32 {
    if motor.is_null() || out_device_id.is_null() || out_responder_id.is_null() {
        set_last_error("motor or output pointer is null");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    let rc = match &*motor {
        MotorHandleInner::Robstride(m) => m
            .ping_with_host_id(host_id, Duration::from_millis(timeout_ms as u64))
            .map_err(|e| e.to_string()),
        MotorHandleInner::Damiao(_)
        | MotorHandleInner::Hexfellow(_)
        | MotorHandleInner::MyActuator(_)
        | MotorHandleInner::Hightorque(_) => {
            Err("robstride_ping_host_id requires a RobStride motor".to_string())
        }
    };
    match rc {
        Ok(reply) => {
            unsafe {
                *out_device_id = reply.device_id;
                *out_responder_id = reply.responder_id;
            }
            0
        }
        Err(e) => {
            set_last_error(e);
            -1
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_robstride_get_param_f32_host_id(
    motor: *mut MotorHandle,
    param_id: u16,
    host_id: u16,
    timeout_ms: u32,
    out_value: *mut f32,
) -> i32 {
    if motor.is_null() || out_value.is_null() {
        set_last_error("motor or output pointer is null");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    let out = unsafe { &mut *out_value };
    let rc = match &*motor {
        MotorHandleInner::Robstride(m) => m
            .get_parameter_with_host_id(param_id, host_id, Duration::from_millis(timeout_ms as u64))
            .and_then(|value| match value {
                ParameterValue::F32(v) => Ok(v),
                _ => Err(motor_core::error::MotorError::Protocol(format!(
                    "parameter 0x{param_id:04X} is not f32"
                ))),
            })
            .map_err(|e| e.to_string())
            .map(|v| *out = v),
        MotorHandleInner::Damiao(_)
        | MotorHandleInner::Hexfellow(_)
        | MotorHandleInner::MyActuator(_)
        | MotorHandleInner::Hightorque(_) => {
            Err("robstride_get_param_f32_host_id requires a RobStride motor".to_string())
        }
    };
    ffi_rc(rc)
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_robstride_get_fault_report(
    motor: *mut MotorHandle,
    out_fault_raw: *mut u32,
    out_warning_raw: *mut u32,
) -> i32 {
    if motor.is_null() || out_fault_raw.is_null() || out_warning_raw.is_null() {
        set_last_error("motor or output pointer is null");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    match &*motor {
        MotorHandleInner::Robstride(m) => {
            let report = m.latest_fault_report();
            unsafe {
                *out_fault_raw = report.map(|r| r.fault_raw).unwrap_or(0);
                *out_warning_raw = report.map(|r| r.warning_raw).unwrap_or(0);
            }
            0
        }
        MotorHandleInner::Damiao(_)
        | MotorHandleInner::Hexfellow(_)
        | MotorHandleInner::MyActuator(_)
        | MotorHandleInner::Hightorque(_) => {
            set_last_error("robstride_get_fault_report requires a RobStride motor");
            -1
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_robstride_set_device_id(
    motor: *mut MotorHandle,
    new_device_id: u8,
) -> i32 {
    if motor.is_null() {
        set_last_error("motor is null");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    let rc = match &*motor {
        MotorHandleInner::Robstride(m) => m.set_device_id(new_device_id).map_err(|e| e.to_string()),
        MotorHandleInner::Damiao(_)
        | MotorHandleInner::Hexfellow(_)
        | MotorHandleInner::MyActuator(_)
        | MotorHandleInner::Hightorque(_) => {
            Err("robstride_set_device_id requires a RobStride motor".to_string())
        }
    };
    ffi_rc(rc)
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_robstride_set_active_report(
    motor: *mut MotorHandle,
    enabled: u8,
) -> i32 {
    if motor.is_null() {
        set_last_error("motor is null");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    let rc = match &*motor {
        MotorHandleInner::Robstride(m) => {
            m.set_active_report(enabled != 0).map_err(|e| e.to_string())
        }
        MotorHandleInner::Damiao(_)
        | MotorHandleInner::Hexfellow(_)
        | MotorHandleInner::MyActuator(_)
        | MotorHandleInner::Hightorque(_) => {
            Err("robstride_set_active_report requires a RobStride motor".to_string())
        }
    };
    ffi_rc(rc)
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

    fn reply(motor: &DamiaoMotor, rid: u8, raw: [u8; 4]) {
        motor
            .process_feedback_frame(CanFrame {
                arbitration_id: 0x11,
                data: [0x01, 0x00, 0x33, rid, raw[0], raw[1], raw[2], raw[3]],
                dlc: 8,
                is_extended: false,
                is_rx: true,
            })
            .expect("process reply");
    }

    fn last_error() -> String {
        LAST_ERROR.with(|slot| slot.borrow().to_string_lossy().into_owned())
    }

    fn free(handle: *mut MotorHandle) {
        drop(unsafe { Box::from_raw(handle) });
    }

    #[test]
    fn cached_register_round_trip_and_freshness() {
        let (motor, h) = damiao_handle();
        let (mut v, mut age, mut fresh) = (0.0f32, 0u64, 9u8);

        assert_eq!(motor_handle_get_cached_register_f32(h, 81, &mut v, &mut age, &mut fresh), 1);
        assert_eq!(fresh, 9, "nothing written when no reply is cached");

        assert_eq!(motor_handle_request_register(h, 81), 0);
        reply(&motor, 81, 1.25f32.to_le_bytes());
        assert_eq!(motor_handle_get_cached_register_f32(h, 81, &mut v, &mut age, &mut fresh), 0);
        assert_eq!((v, fresh), (1.25, 1));

        // Re-request without a reply: old value still returned, flagged stale.
        assert_eq!(motor_handle_request_register(h, 81), 0);
        assert_eq!(motor_handle_get_cached_register_f32(h, 81, &mut v, &mut age, &mut fresh), 0);
        assert_eq!((v, fresh), (1.25, 0));
        free(h);
    }

    #[test]
    fn cached_register_u32_reads_u32_registers() {
        let (motor, h) = damiao_handle();
        let (mut v, mut age, mut fresh) = (0u32, 0u64, 0u8);
        assert_eq!(motor_handle_request_register(h, 10), 0);
        reply(&motor, 10, 1u32.to_le_bytes());
        assert_eq!(motor_handle_get_cached_register_u32(h, 10, &mut v, &mut age, &mut fresh), 0);
        assert_eq!((v, fresh), (1, 1));
        free(h);
    }

    #[test]
    fn cached_register_error_branches() {
        let (motor, h) = damiao_handle();
        let (mut f, mut u, mut age, mut fresh) = (0.0f32, 0u32, 0u64, 0u8);

        // Null motor handle.
        assert_eq!(motor_handle_request_register(ptr::null_mut(), 81), -1);
        assert_eq!(
            motor_handle_get_cached_register_f32(ptr::null_mut(), 81, &mut f, &mut age, &mut fresh),
            -1
        );
        // Null output pointers.
        assert_eq!(motor_handle_get_cached_register_f32(h, 81, ptr::null_mut(), &mut age, &mut fresh), -1);
        assert_eq!(motor_handle_get_cached_register_f32(h, 81, &mut f, ptr::null_mut(), &mut fresh), -1);
        assert_eq!(motor_handle_get_cached_register_f32(h, 81, &mut f, &mut age, ptr::null_mut()), -1);
        // Unknown register: rejected before anything is sent.
        assert_eq!(motor_handle_request_register(h, 200), -1);
        assert!(last_error().contains("unknown register"));
        // Wrong type: register 81 is a float.
        assert_eq!(motor_handle_request_register(h, 81), 0);
        reply(&motor, 81, 1.0f32.to_le_bytes());
        assert_eq!(motor_handle_get_cached_register_u32(h, 81, &mut u, &mut age, &mut fresh), -1);
        assert!(last_error().contains("not u32"));
        free(h);

        // Non-Damiao motor.
        let bus: Arc<dyn CanBus> = Arc::new(MockBus::new());
        let rs = Arc::new(RobstrideMotor::new(0x01, 0xFD, "rs-00", bus).expect("create robstride"));
        let h = Box::into_raw(Box::new(MotorHandle {
            inner: Mutex::new(MotorHandleInner::Robstride(rs)),
        }));
        assert_eq!(motor_handle_request_register(h, 81), -1);
        assert_eq!(motor_handle_get_cached_register_f32(h, 81, &mut f, &mut age, &mut fresh), -1);
        assert!(last_error().contains("only available for Damiao"));
        free(h);
    }
}

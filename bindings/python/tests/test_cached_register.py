from __future__ import annotations

import ctypes
from types import SimpleNamespace

import pytest

from motorbridge import CallError
from motorbridge.abi import get_abi
from motorbridge.core import Motor


def test_nonblocking_register_symbols_are_bound() -> None:
    lib = get_abi().lib
    for name in (
        "motor_handle_request_register",
        "motor_handle_get_cached_register_f32",
        "motor_handle_get_cached_register_u32",
    ):
        assert getattr(lib, name).restype is ctypes.c_int32
    assert len(lib.motor_handle_get_cached_register_f32.argtypes) == 5


def _motor_with(fn_name: str, impl) -> Motor:
    motor = Motor.__new__(Motor)
    motor._ptr = 1
    motor._abi = SimpleNamespace(lib=SimpleNamespace(**{fn_name: impl}))
    return motor


def _reply(value, age_us: int, fresh: int):
    def impl(ptr, rid, out, age, fresh_out):
        out._obj.value = value
        age._obj.value = age_us
        fresh_out._obj.value = fresh
        return 0

    return impl


def test_cached_register_f32_returns_value_and_age() -> None:
    motor = _motor_with("motor_handle_get_cached_register_f32", _reply(-0.5, 1500, 1))
    value, age_s = motor.get_cached_register_f32(81)
    assert value == pytest.approx(-0.5)
    assert age_s == pytest.approx(0.0015)


def test_stale_reply_hidden_by_default() -> None:
    motor = _motor_with("motor_handle_get_cached_register_f32", _reply(-0.5, 9000, 0))
    assert motor.get_cached_register_f32(81) is None
    value, age_s = motor.get_cached_register_f32(81, fresh_only=False)
    assert value == pytest.approx(-0.5)
    assert age_s == pytest.approx(0.009)


def test_cached_register_returns_none_when_no_reply_yet() -> None:
    motor = _motor_with("motor_handle_get_cached_register_u32", lambda *a: 1)
    assert motor.get_cached_register_u32(10) is None
    assert motor.get_cached_register_u32(10, fresh_only=False) is None


def test_cached_register_raises_on_error() -> None:
    motor = _motor_with("motor_handle_get_cached_register_f32", lambda *a: -1)
    with pytest.raises(CallError):
        motor.get_cached_register_f32(10)

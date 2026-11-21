from __future__ import annotations

import ctypes
from types import SimpleNamespace

import pytest

from motorbridge import CallError
from motorbridge.abi import get_abi
from motorbridge.core import Motor


def test_state_age_symbol_is_bound() -> None:
    lib = get_abi().lib
    fn = lib.motor_handle_get_state_with_age
    assert fn.restype is ctypes.c_int32
    assert len(fn.argtypes) == 4


def _motor_with(impl) -> Motor:
    motor = Motor.__new__(Motor)
    motor._ptr = 1
    motor._abi = SimpleNamespace(lib=SimpleNamespace(motor_handle_get_state_with_age=impl))
    return motor


def test_state_with_age_returns_state_age_and_seq() -> None:
    def impl(ptr, out_state, out_age, out_seq):
        st = out_state._obj
        st.has_value, st.status_code, st.pos, st.t_mos = 1, 1, 0.25, 35.0
        out_age._obj.value = 2500
        out_seq._obj.value = 42
        return 0

    state, age_s, seq = _motor_with(impl).get_state_with_age()
    assert state.status_code == 1 and state.pos == pytest.approx(0.25) and state.t_mos == 35.0
    assert age_s == pytest.approx(0.0025)
    assert seq == 42


def test_state_with_age_none_before_the_first_state_frame() -> None:
    assert _motor_with(lambda *a: 1).get_state_with_age() is None


def test_state_with_age_raises_on_error() -> None:
    with pytest.raises(CallError):
        _motor_with(lambda *a: -1).get_state_with_age()

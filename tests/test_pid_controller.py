import math
import pytest
from pid_exp.pid_controller import PIDController, normalize_heading_error


def test_normalize_heading_error_no_wrap():
    assert normalize_heading_error(30) == 30
    assert normalize_heading_error(-30) == -30
    assert normalize_heading_error(0) == 0


def test_normalize_heading_error_wrap_positive_to_negative():
    # 350 → -10
    assert normalize_heading_error(350) == pytest.approx(-10)


def test_normalize_heading_error_wrap_negative_to_positive():
    # -350 → +10
    assert normalize_heading_error(-350) == pytest.approx(10)


def test_normalize_heading_error_exact_boundary():
    # 180 应该映射到 -180（公式归到 (-180, 180]，含 -180）
    assert normalize_heading_error(180) == pytest.approx(-180)
    assert normalize_heading_error(-180) == pytest.approx(-180)


def test_pid_init_zero_state():
    pid = PIDController(kp=1.0, ki=0.0, kd=0.0, axis="altitude")
    assert pid.kp == 1.0
    assert pid.ki == 0.0
    assert pid.kd == 0.0
    assert pid.axis == "altitude"


def test_pid_p_only():
    pid = PIDController(kp=2.0, ki=0.0, kd=0.0, axis="altitude")
    result = pid.update(current=100, target=110, dt=0.1)
    # error = 10, P term = 2.0 * 10 = 20
    assert result["error"] == pytest.approx(10)
    assert result["p_term"] == pytest.approx(20)
    assert result["i_term"] == pytest.approx(0)
    assert result["d_term"] == pytest.approx(0)
    assert result["output"] == pytest.approx(20)


def test_pid_i_accumulates():
    pid = PIDController(kp=0.0, ki=1.0, kd=0.0, axis="altitude")
    pid.update(current=100, target=110, dt=0.1)  # integral += 10*0.1 = 1.0
    result = pid.update(current=100, target=110, dt=0.1)  # integral += 10*0.1 = 2.0
    assert result["integral"] == pytest.approx(2.0)
    assert result["i_term"] == pytest.approx(2.0)


def test_pid_d_derivative():
    pid = PIDController(kp=0.0, ki=0.0, kd=1.0, axis="altitude")
    pid.update(current=100, target=110, dt=0.1)  # prev_error = 10
    result = pid.update(current=105, target=110, dt=0.1)  # error=5, derivative=(5-10)/0.1=-50
    assert result["derivative"] == pytest.approx(-50)
    assert result["d_term"] == pytest.approx(-50)


def test_pid_full_combo():
    pid = PIDController(kp=2.0, ki=1.0, kd=0.5, axis="altitude")
    pid.update(current=100, target=110, dt=0.1)  # integral=1.0, prev_error=10
    result = pid.update(current=105, target=110, dt=0.1)
    # error=5, derivative=(5-10)/0.1=-50, integral=1.0+5*0.1=1.5
    # p=2*5=10, i=1*1.5=1.5, d=0.5*(-50)=-25, output=10+1.5-25=-13.5
    assert result["error"] == pytest.approx(5)
    assert result["integral"] == pytest.approx(1.5)
    assert result["p_term"] == pytest.approx(10)
    assert result["i_term"] == pytest.approx(1.5)
    assert result["d_term"] == pytest.approx(-25)
    assert result["output"] == pytest.approx(-13.5)


def test_pid_reset_clears_state():
    pid = PIDController(kp=1.0, ki=1.0, kd=1.0, axis="altitude")
    pid.update(current=100, target=110, dt=0.1)
    pid.update(current=105, target=110, dt=0.1)
    pid.reset()
    result = pid.update(current=100, target=110, dt=0.1)
    # 应该和首次 update 一样，integral 重新从 error*dt 累计
    assert result["integral"] == pytest.approx(1.0)


def test_pid_heading_wrap_in_update():
    # heading 350, target 10 → 真实误差 +20，不是 -340
    pid = PIDController(kp=1.0, ki=0.0, kd=0.0, axis="heading")
    result = pid.update(current=350, target=10, dt=0.1)
    assert result["error"] == pytest.approx(20)
    assert result["output"] == pytest.approx(20)


def test_pid_altitude_no_wrap():
    # altitude 不应做 wrap
    pid = PIDController(kp=1.0, ki=0.0, kd=0.0, axis="altitude")
    result = pid.update(current=350, target=10, dt=0.1)
    assert result["error"] == pytest.approx(-340)


def test_pid_zero_dt_safety():
    # dt=0 时不应 ZeroDivisionError；derivative 应保护性处理
    pid = PIDController(kp=1.0, ki=1.0, kd=1.0, axis="altitude")
    pid.update(current=100, target=110, dt=0.1)
    result = pid.update(current=100, target=110, dt=0.0)
    # 不报错；derivative 用 0 替代
    assert result["derivative"] == pytest.approx(0)

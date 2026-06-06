import json
import math
import pytest
from unittest.mock import patch, MagicMock
from pid_exp import flight_io


def _mock_response(status_code=200, json_data=None):
    resp = MagicMock()
    resp.status_code = status_code
    resp.json.return_value = json_data or {"message": "ok"}
    resp.text = json.dumps(json_data or {"message": "ok"})
    return resp


def test_set_flight_parameter_success(monkeypatch):
    monkeypatch.setattr(flight_io, "API_URL_CTRL", "http://fake/set")
    with patch("pid_exp.flight_io.requests.put") as mock_put:
        mock_put.return_value = _mock_response(200, {"message": "ok"})
        result = flight_io.set_flight_parameter("RUDDER_POSITION", 0.5)
        mock_put.assert_called_once_with(
            "http://fake/set",
            json={"name": "RUDDER_POSITION", "val": 0.5},
            timeout=5.0,
        )
        assert "ok" in result["message"]


def test_set_flight_parameter_retries_then_succeeds(monkeypatch):
    monkeypatch.setattr(flight_io, "API_URL_CTRL", "http://fake/set")
    import requests as _requests
    with patch("pid_exp.flight_io.requests.put") as mock_put:
        mock_put.side_effect = [
            _requests.exceptions.Timeout(),
            _mock_response(200, {"message": "ok"}),
        ]
        result = flight_io.set_flight_parameter("RUDDER_POSITION", 0.5)
        assert mock_put.call_count == 2
        assert "ok" in result["message"]


def test_set_flight_parameter_raises_after_max_retries(monkeypatch):
    monkeypatch.setattr(flight_io, "API_URL_CTRL", "http://fake/set")
    import requests as _requests
    with patch("pid_exp.flight_io.requests.put") as mock_put:
        mock_put.side_effect = _requests.exceptions.Timeout()
        with pytest.raises(flight_io.FlightIOError):
            flight_io.set_flight_parameter("RUDDER_POSITION", 0.5)
        # 1 次首发 + 2 次 retry = 3 次
        assert mock_put.call_count == 3


# 注意：PLANE_HEADING_DEGREES_MAGNETIC 在 bridge 端是【弧度】（名字骗人），
# 这里 val 用 95.5° 对应的弧度，read_state 应把它转回 95.5°。
SAMPLE_GET_RESPONSE = [
    {"name": "PLANE_HEADING_DEGREES_MAGNETIC", "val": math.radians(95.5), "unit": "Radians", "writable": True},
    {"name": "PLANE_ALTITUDE", "val": 5020.3, "unit": "Feet", "writable": True},
    {"name": "PLANE_LATITUDE", "val": 47.6, "unit": "Degrees", "writable": False},
]


def test_read_state_returns_heading_altitude(monkeypatch):
    monkeypatch.setattr(flight_io, "API_URL_GET", "http://fake/get")
    with patch("pid_exp.flight_io.requests.get") as mock_get:
        resp = MagicMock()
        resp.status_code = 200
        resp.json.return_value = SAMPLE_GET_RESPONSE
        mock_get.return_value = resp

        state = flight_io.read_state()
        assert state["heading"] == pytest.approx(95.5)
        assert state["altitude"] == pytest.approx(5020.3)
        assert "raw" in state
        assert "timestamp" in state


def test_read_state_retries_on_http_error(monkeypatch):
    monkeypatch.setattr(flight_io, "API_URL_GET", "http://fake/get")
    import requests as _requests
    with patch("pid_exp.flight_io.requests.get") as mock_get:
        success_resp = MagicMock()
        success_resp.status_code = 200
        success_resp.json.return_value = SAMPLE_GET_RESPONSE
        mock_get.side_effect = [
            _requests.exceptions.ConnectionError(),
            success_resp,
        ]
        state = flight_io.read_state()
        assert mock_get.call_count == 2
        assert state["heading"] == pytest.approx(95.5)


# ------------------------------------------------------------
# heading 弧度/度 转换（bridge 端 PLANE_HEADING_DEGREES_MAGNETIC 实为弧度）
# ------------------------------------------------------------


def test_heading_rad_to_deg_basic():
    assert flight_io._heading_rad_to_deg(0.0) == pytest.approx(0.0)
    assert flight_io._heading_rad_to_deg(math.pi / 2) == pytest.approx(90.0)
    assert flight_io._heading_rad_to_deg(math.pi) == pytest.approx(180.0)


def test_heading_rad_to_deg_normalizes_into_0_360():
    # 负角和 >2π 都要折回 [0, 360)
    assert flight_io._heading_rad_to_deg(-math.pi / 2) == pytest.approx(270.0)
    assert flight_io._heading_rad_to_deg(2 * math.pi + math.pi / 2) == pytest.approx(90.0)


def test_heading_deg_to_rad_roundtrip():
    for deg in (0.0, 90.0, 120.0, 359.0):
        assert flight_io._heading_deg_to_rad(deg) == pytest.approx(math.radians(deg))
        # 往返自洽
        back = flight_io._heading_rad_to_deg(flight_io._heading_deg_to_rad(deg))
        assert back == pytest.approx(deg)


def test_read_state_converts_radians_to_degrees(monkeypatch):
    # 复现 bug：bridge 返回 2.0354 弧度，应转成 ≈116.6° 而非被当成 2.03°
    monkeypatch.setattr(flight_io, "API_URL_GET", "http://fake/get")
    resp_data = [
        {"name": "PLANE_HEADING_DEGREES_MAGNETIC", "val": 2.0354, "unit": "Radians", "writable": True},
        {"name": "PLANE_ALTITUDE", "val": 5000.0, "unit": "Feet", "writable": True},
    ]
    with patch("pid_exp.flight_io.requests.get") as mock_get:
        resp = MagicMock()
        resp.status_code = 200
        resp.json.return_value = resp_data
        mock_get.return_value = resp

        state = flight_io.read_state()
        assert state["heading"] == pytest.approx(math.degrees(2.0354), abs=1e-6)
        assert state["heading"] == pytest.approx(116.6, abs=0.1)


def test_hover_calls_correct_params():
    with patch("pid_exp.flight_io.set_flight_parameter") as mock_set, \
         patch("pid_exp.flight_io.time.sleep"):
        flight_io.hover()
        calls = [c.args for c in mock_set.call_args_list]
        assert ("GENERAL_ENG_THROTTLE_LEVER_POSITION:1", 50) in calls
        assert ("RUDDER_POSITION", 0.0) in calls
        assert ("AILERON_POSITION", 0.0) in calls
        assert ("ELEVATOR_POSITION", 0.0) in calls


def test_move_forward_sequence():
    with patch("pid_exp.flight_io.set_flight_parameter") as mock_set, \
         patch("pid_exp.flight_io.time.sleep") as mock_sleep:
        flight_io.move_forward(0.5)
        # hover, throttle=99, hover; sleep 0.5 then 1.0
        param_calls = [c.args for c in mock_set.call_args_list]
        assert ("GENERAL_ENG_THROTTLE_LEVER_POSITION:1", 99) in param_calls
        # 至少 hover 调用两次 → throttle 50 至少出现 2 次
        throttle_50 = sum(1 for c in param_calls
                          if c == ("GENERAL_ENG_THROTTLE_LEVER_POSITION:1", 50))
        assert throttle_50 >= 2
        sleeps = [c.args[0] for c in mock_sleep.call_args_list]
        assert 0.5 in sleeps  # action_time
        assert 1.0 in sleeps  # 后置 hover 1s


def test_move_ascend_sets_elevator_positive():
    with patch("pid_exp.flight_io.set_flight_parameter") as mock_set, \
         patch("pid_exp.flight_io.time.sleep"):
        flight_io.move_ascend(0.3)
        param_calls = [c.args for c in mock_set.call_args_list]
        assert ("ELEVATOR_POSITION", 1.0) in param_calls


def test_move_descend_sets_elevator_negative_and_clips_time():
    with patch("pid_exp.flight_io.set_flight_parameter") as mock_set, \
         patch("pid_exp.flight_io.time.sleep") as mock_sleep:
        flight_io.move_descend(5.0)  # 应该被截到 2.0
        param_calls = [c.args for c in mock_set.call_args_list]
        assert ("ELEVATOR_POSITION", -1.0) in param_calls
        sleeps = [c.args[0] for c in mock_sleep.call_args_list]
        assert 2.0 in sleeps  # 截断后的 action_time
        assert 1.0 in sleeps  # 后置等待


def test_hover_turn_left_sets_rudder_negative():
    with patch("pid_exp.flight_io.set_flight_parameter") as mock_set, \
         patch("pid_exp.flight_io.time.sleep"):
        flight_io.hover_turn_left(0.4)
        param_calls = [c.args for c in mock_set.call_args_list]
        assert ("RUDDER_POSITION", -0.05) in param_calls


def test_hover_turn_right_sets_rudder_positive():
    with patch("pid_exp.flight_io.set_flight_parameter") as mock_set, \
         patch("pid_exp.flight_io.time.sleep"):
        flight_io.hover_turn_right(0.4)
        param_calls = [c.args for c in mock_set.call_args_list]
        assert ("RUDDER_POSITION", 0.05) in param_calls


def test_reset_to_happy_path():
    with patch("pid_exp.flight_io.set_flight_parameter") as mock_set, \
         patch("pid_exp.flight_io.read_state") as mock_read, \
         patch("pid_exp.flight_io.time.sleep"):
        mock_read.return_value = {
            "heading": 90.5,  # 在 ±5° 容差内
            "altitude": 5010,  # 在 ±50ft 容差内
            "raw": {},
            "timestamp": 0,
        }
        flight_io.reset_to(heading=90.0, altitude=5000.0)
        # 应该 set heading, altitude, 然后 hover（hover 内部又 set 4 个参数）
        param_calls = [c.args for c in mock_set.call_args_list]
        # heading 写回时转成弧度（90° → π/2），altitude 不转
        heading_writes = [c for c in param_calls if c[0] == "PLANE_HEADING_DEGREES_MAGNETIC"]
        assert len(heading_writes) == 1
        assert heading_writes[0][1] == pytest.approx(math.radians(90.0))
        assert ("PLANE_ALTITUDE", 5000.0) in param_calls


def test_reset_to_raises_on_heading_oob():
    with patch("pid_exp.flight_io.set_flight_parameter"), \
         patch("pid_exp.flight_io.read_state") as mock_read, \
         patch("pid_exp.flight_io.time.sleep"):
        mock_read.return_value = {
            "heading": 100,  # 偏 10° > 容差 5°
            "altitude": 5000,
            "raw": {},
            "timestamp": 0,
        }
        with pytest.raises(flight_io.ResetVerificationError):
            flight_io.reset_to(heading=90.0, altitude=5000.0)


def test_reset_to_raises_on_altitude_oob():
    with patch("pid_exp.flight_io.set_flight_parameter"), \
         patch("pid_exp.flight_io.read_state") as mock_read, \
         patch("pid_exp.flight_io.time.sleep"):
        mock_read.return_value = {
            "heading": 90,
            "altitude": 5100,  # 偏 100ft > 容差 50ft
            "raw": {},
            "timestamp": 0,
        }
        with pytest.raises(flight_io.ResetVerificationError):
            flight_io.reset_to(heading=90.0, altitude=5000.0)


def test_reset_to_heading_wrap_in_verification():
    # 目标 5°, 实际 359° → 真实偏差 6° > 容差 5° → 应抛
    with patch("pid_exp.flight_io.set_flight_parameter"), \
         patch("pid_exp.flight_io.read_state") as mock_read, \
         patch("pid_exp.flight_io.time.sleep"):
        mock_read.return_value = {
            "heading": 359,
            "altitude": 5000,
            "raw": {},
            "timestamp": 0,
        }
        with pytest.raises(flight_io.ResetVerificationError):
            flight_io.reset_to(heading=5.0, altitude=5000.0)


def test_reset_to_heading_wrap_within_tolerance():
    # 目标 5°, 实际 2° → 偏差 3° < 容差 5° → 应通过
    with patch("pid_exp.flight_io.set_flight_parameter"), \
         patch("pid_exp.flight_io.read_state") as mock_read, \
         patch("pid_exp.flight_io.time.sleep"):
        mock_read.return_value = {
            "heading": 2,
            "altitude": 5000,
            "raw": {},
            "timestamp": 0,
        }
        flight_io.reset_to(heading=5.0, altitude=5000.0)  # 不应抛

"""Thin HTTP wrapper for MSFS 2024 bridge.

This module is the ONLY place that touches MSFS via HTTP. All other modules
import functions from here and never use ``requests`` directly.
"""

from __future__ import annotations

import logging
import os
import time

import requests
from dotenv import load_dotenv

load_dotenv()
API_URL_CTRL = os.getenv("API_URL_CTRL")
API_URL_GET = os.getenv("API_URL_GET")

logger = logging.getLogger("flight_io")

# HTTP 重试和超时配置
_HTTP_TIMEOUT = 5.0
_HTTP_MAX_RETRIES = 2  # 首发之外再重试 2 次


class FlightIOError(RuntimeError):
    """HTTP 调用最终失败时抛出。"""


def set_flight_parameter(name: str, val) -> dict:
    """PUT to API_URL_CTRL. 自动 retry 2 次。"""
    if API_URL_CTRL is None:
        raise FlightIOError("API_URL_CTRL not set; check .env")
    payload = {"name": name, "val": val}
    last_exc = None
    for attempt in range(_HTTP_MAX_RETRIES + 1):
        try:
            resp = requests.put(API_URL_CTRL, json=payload, timeout=_HTTP_TIMEOUT)
            if resp.status_code == 200:
                return resp.json()
            logger.warning(
                "set_flight_parameter %s=%s got status %s",
                name, val, resp.status_code,
            )
            last_exc = FlightIOError(f"HTTP {resp.status_code}: {resp.text}")
        except requests.exceptions.RequestException as e:
            logger.warning("set_flight_parameter %s=%s attempt %d failed: %s",
                           name, val, attempt + 1, e)
            last_exc = e
    raise FlightIOError(f"set_flight_parameter failed after retries: {last_exc}")


def read_state() -> dict:
    """GET from API_URL_GET. 返回 {heading, altitude, raw, timestamp}。"""
    if API_URL_GET is None:
        raise FlightIOError("API_URL_GET not set; check .env")
    last_exc = None
    for attempt in range(_HTTP_MAX_RETRIES + 1):
        try:
            resp = requests.get(API_URL_GET, timeout=_HTTP_TIMEOUT)
            if resp.status_code == 200:
                data = resp.json()
                state_dict = {item["name"]: item for item in data}
                return {
                    "heading": float(state_dict["PLANE_HEADING_DEGREES_MAGNETIC"]["val"]),
                    "altitude": float(state_dict["PLANE_ALTITUDE"]["val"]),
                    "raw": state_dict,
                    "timestamp": time.time(),
                }
            last_exc = FlightIOError(f"HTTP {resp.status_code}: {resp.text}")
        except requests.exceptions.RequestException as e:
            logger.warning("read_state attempt %d failed: %s", attempt + 1, e)
            last_exc = e
        except (KeyError, ValueError) as e:
            # JSON 缺字段——直接抛，不 retry
            raise FlightIOError(f"read_state: bad JSON shape: {e}")
    raise FlightIOError(f"read_state failed after retries: {last_exc}")


# ============================================================
# 动作函数 — 拷贝自原 flight_operations.py
# 与原版的差异（参见 code_design.md 9 节）：
#  1. 去掉 @mcp.tool() 装饰器
#  2. my_logger.info → logger.info（用模块顶部的 logger）
#  3. 去掉 get_is_on_ground 依赖（不再判断着陆）
# 不拷贝：move_left / move_right / move_forward_and_descend（PID 不用）
# ============================================================


def move_forward(time_s: float) -> None:
    logger.info("move_forward: %.3fs", time_s)
    hover()
    set_flight_parameter("GENERAL_ENG_THROTTLE_LEVER_POSITION:1", 99)
    time.sleep(float(time_s))
    hover()
    time.sleep(1)


def move_backward(time_s: float) -> None:
    logger.info("move_backward: %.3fs", time_s)
    hover()
    set_flight_parameter("GENERAL_ENG_THROTTLE_LEVER_POSITION:1", 0)
    time.sleep(float(time_s))
    hover()


def move_ascend(time_s: float) -> None:
    logger.info("move_ascend: %.3fs", time_s)
    hover()
    set_flight_parameter("ELEVATOR_POSITION", 1.0)
    time.sleep(float(time_s))
    hover()


def move_descend(time_s: float) -> None:
    logger.info("move_descend: %.3fs (capped at 2.0)", time_s)
    hover()
    set_flight_parameter("ELEVATOR_POSITION", -1.0)
    time.sleep(min(float(time_s), 2.0))
    hover()
    time.sleep(1)


def hover() -> None:
    set_flight_parameter("GENERAL_ENG_THROTTLE_LEVER_POSITION:1", 50)
    set_flight_parameter("RUDDER_POSITION", 0.0)
    set_flight_parameter("AILERON_POSITION", 0.0)
    set_flight_parameter("ELEVATOR_POSITION", 0.0)
    time.sleep(0.1)


def hover_turn_left(time_s: float) -> None:
    logger.info("hover_turn_left: %.3fs", time_s)
    hover()
    set_flight_parameter("RUDDER_POSITION", -0.05)
    time.sleep(float(time_s))
    hover()


def hover_turn_right(time_s: float) -> None:
    logger.info("hover_turn_right: %.3fs", time_s)
    hover()
    set_flight_parameter("RUDDER_POSITION", 0.05)
    time.sleep(float(time_s))
    hover()


class ResetVerificationError(FlightIOError):
    """复位后状态超出容差。"""


# 复位验证容差
_HEADING_TOL_DEG = 5.0
_ALTITUDE_TOL_FT = 50.0


def _heading_diff(a: float, b: float) -> float:
    """归一化到 [-180, 180]，处理 wrap-around。"""
    diff = (a - b + 180) % 360 - 180
    return diff


def reset_to(heading: float, altitude: float, settle_seconds: float = 5.0) -> None:
    """Teleport-set heading/altitude，hover 进入稳态，等 settle_seconds，验证。

    Raises:
        ResetVerificationError: 复位后状态超出容差
    """
    set_flight_parameter("PLANE_HEADING_DEGREES_MAGNETIC", heading)
    set_flight_parameter("PLANE_ALTITUDE", altitude)
    hover()
    time.sleep(settle_seconds)

    state = read_state()
    heading_err = abs(_heading_diff(state["heading"], heading))
    altitude_err = abs(state["altitude"] - altitude)

    if heading_err > _HEADING_TOL_DEG:
        raise ResetVerificationError(
            f"reset heading 验证失败: 目标 {heading}, 实际 {state['heading']}, "
            f"偏差 {heading_err:.2f}° > 容差 {_HEADING_TOL_DEG}°"
        )
    if altitude_err > _ALTITUDE_TOL_FT:
        raise ResetVerificationError(
            f"reset altitude 验证失败: 目标 {altitude}, 实际 {state['altitude']}, "
            f"偏差 {altitude_err:.2f}ft > 容差 {_ALTITUDE_TOL_FT}ft"
        )
    logger.info("reset_to(heading=%.2f, altitude=%.2f) OK", heading, altitude)

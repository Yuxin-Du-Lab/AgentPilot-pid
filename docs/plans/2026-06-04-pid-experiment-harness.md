# PID 实验台 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 `D:\work\pilot\aircraft_agent\pid_exp\` 下独立搭一个 PID 超参数 OFAT 敏感性实验台，连真实 MSFS 2024，跑 26 个 step response 实验（13 heading + 13 altitude），产出敏感性表 + 叠加曲线图。

**Architecture:** 7 个 Python 模块（flight_io / pid_controller / experiment / metrics / sweep / analysis / __init__）。命令行入口 `python -m pid_exp.sweep --axis heading`。模块间通过 dataclass 配置和文件传值（log.csv / metrics.json / summary.csv）。

**Tech Stack:** Python ≥3.10, requests, python-dotenv, numpy, pandas, matplotlib, pytest（开发依赖）。

**Spec 来源（实施者必读）:**
- `D:\work\pilot\aircraft_agent\pid_exp\docs\code_design.md` — 模块架构、API、错误处理
- `D:\work\pilot\aircraft_agent\pid_exp\docs\experiment_design.md` — OFAT 测试列表、默认参数、复位
- `D:\work\pilot\aircraft_agent\pid_exp\docs\metric_explanation.md` — 6 个 metric 的含义

---

## File Structure（最终交付）

```
D:\work\pilot\aircraft_agent\pid_exp\
├── docs/                              （已存在，本计划不动）
│   ├── code_design.md
│   ├── experiment_design.md
│   ├── metric_explanation.md
│   └── plans/
│       └── 2026-06-04-pid-experiment-harness.md   ← 本文件
├── pid_exp/                            （Python 包）
│   ├── __init__.py
│   ├── flight_io.py                    Task 3, 4, 5
│   ├── pid_controller.py               Task 2
│   ├── experiment.py                   Task 6
│   ├── metrics.py                      Task 7, 8
│   ├── sweep.py                        Task 9, 10
│   └── analysis.py                     Task 11, 12, 13, 14
├── tests/
│   ├── __init__.py
│   ├── test_pid_controller.py          Task 2
│   ├── test_flight_io.py               Task 3, 4, 5
│   ├── test_experiment.py              Task 6
│   ├── test_metrics.py                 Task 7, 8
│   ├── test_sweep.py                   Task 9, 10
│   └── test_analysis.py                Task 11, 12, 13
├── results/                            （运行时自动创建）
├── .env                                Task 1 （用户填）
├── .env.example                        Task 1
├── requirements.txt                    Task 1
├── pyproject.toml                      Task 1
└── README.md                           Task 15
```

---

## Task 1: 项目脚手架

**Files:**
- Create: `D:\work\pilot\aircraft_agent\pid_exp\requirements.txt`
- Create: `D:\work\pilot\aircraft_agent\pid_exp\.env.example`
- Create: `D:\work\pilot\aircraft_agent\pid_exp\pyproject.toml`
- Create: `D:\work\pilot\aircraft_agent\pid_exp\pid_exp\__init__.py`
- Create: `D:\work\pilot\aircraft_agent\pid_exp\tests\__init__.py`

- [ ] **Step 1: 创建目录结构 + 所有模块的空 stub**

```bash
cd D:/work/pilot/aircraft_agent/pid_exp
mkdir -p pid_exp tests
touch pid_exp/__init__.py tests/__init__.py
touch pid_exp/flight_io.py pid_exp/pid_controller.py
touch pid_exp/experiment.py pid_exp/metrics.py
touch pid_exp/sweep.py pid_exp/analysis.py
```

**为什么所有模块文件都要 touch**：后续任务里有些模块（如 `sweep.py`）在 module-level `from pid_exp import analysis` 引用其他模块。即便此时 analysis.py 是空文件，`import` 也能成功，避免 Task 9/10 完成后 sweep.py 因为 analysis 还没实现而无法 import 的循环。各模块的真实内容会在对应 Task 中填入。

- [ ] **Step 2: 写 requirements.txt**

文件：`D:\work\pilot\aircraft_agent\pid_exp\requirements.txt`

```
requests>=2.31
python-dotenv>=1.0
numpy>=1.26
pandas>=2.0
matplotlib>=3.8
pytest>=7.4
```

- [ ] **Step 3: 写 .env.example**

文件：`D:\work\pilot\aircraft_agent\pid_exp\.env.example`

```dotenv
# MSFS 2024 HTTP bridge endpoints. 实际运行时复制本文件为 .env 并填入真实 URL。
API_URL_CTRL=http://<msfs-bridge-host>:5000/set
API_URL_GET=http://<msfs-bridge-host>:5000/get
```

- [ ] **Step 4: 写 pyproject.toml（最小配置，让 `python -m pid_exp.sweep` 能跑）**

文件：`D:\work\pilot\aircraft_agent\pid_exp\pyproject.toml`

```toml
[build-system]
requires = ["setuptools>=68"]
build-backend = "setuptools.build_meta"

[project]
name = "pid_exp"
version = "0.1.0"
requires-python = ">=3.10"

[tool.setuptools.packages.find]
include = ["pid_exp*"]
exclude = ["tests*"]

[tool.pytest.ini_options]
testpaths = ["tests"]
```

- [ ] **Step 5: 安装依赖并验证**

```bash
cd D:/work/pilot/aircraft_agent/pid_exp
py -m pip install -e .
py -m pip install -r requirements.txt
py -m pytest --collect-only
```

Expected: pytest 输出 "no tests collected"（因为还没写测试），且无 ImportError。

- [ ] **Step 6 (可选): git 初始化 + 首次提交**

如果用户使用 git：
```bash
cd D:/work/pilot/aircraft_agent/pid_exp
git init
git add .gitignore requirements.txt pyproject.toml .env.example pid_exp/ tests/ docs/
git commit -m "feat: 项目脚手架"
```

`.gitignore` 内容（建议）：
```
.env
__pycache__/
*.pyc
.pytest_cache/
results/
```

---

## Task 2: PIDController 类（含 heading wrap-around）

**Files:**
- Create: `D:\work\pilot\aircraft_agent\pid_exp\pid_exp\pid_controller.py`
- Create: `D:\work\pilot\aircraft_agent\pid_exp\tests\test_pid_controller.py`

- [ ] **Step 1: 写 normalize_heading_error 的测试**

文件：`tests/test_pid_controller.py`

```python
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
    # 180 应该保持 180（边界包含）
    assert normalize_heading_error(180) == pytest.approx(-180)  # 因为公式归到 (-180, 180]
    assert normalize_heading_error(-180) == pytest.approx(-180)
```

- [ ] **Step 2: 跑测试验证失败**

```bash
py -m pytest tests/test_pid_controller.py::test_normalize_heading_error_no_wrap -v
```

Expected: FAIL with `ModuleNotFoundError: No module named 'pid_exp.pid_controller'`

- [ ] **Step 3: 实现 normalize_heading_error**

文件：`pid_exp/pid_controller.py`

```python
"""1D PID controller with heading wrap-around support."""


def normalize_heading_error(error: float) -> float:
    """折叠到 (-180, 180]: 350 → -10, -350 → +10."""
    return ((error + 180) % 360) - 180
```

- [ ] **Step 4: 跑测试验证 normalize 通过**

```bash
py -m pytest tests/test_pid_controller.py -v
```

Expected: 4 个 normalize 测试 PASS。

- [ ] **Step 5: 写 PIDController 类的测试**

追加到 `tests/test_pid_controller.py`：

```python
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
```

- [ ] **Step 6: 跑测试验证全部失败（因为 PIDController 还没实现）**

```bash
py -m pytest tests/test_pid_controller.py -v
```

Expected: 4 个 normalize 测试 PASS，9 个 PID 测试 FAIL with `ImportError: cannot import name 'PIDController'`。

- [ ] **Step 7: 实现 PIDController 类**

追加到 `pid_exp/pid_controller.py`：

```python
class PIDController:
    """1D PID controller. 支持 heading wrap-around。"""

    def __init__(self, kp: float, ki: float, kd: float, axis: str):
        if axis not in ("heading", "altitude"):
            raise ValueError(f"axis must be 'heading' or 'altitude', got {axis!r}")
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.axis = axis
        self._integral = 0.0
        self._prev_error: float | None = None

    def reset(self) -> None:
        self._integral = 0.0
        self._prev_error = None

    def _compute_error(self, current: float, target: float) -> float:
        raw = target - current
        if self.axis == "heading":
            return normalize_heading_error(raw)
        return raw

    def update(self, current: float, target: float, dt: float) -> dict:
        error = self._compute_error(current, target)

        # Integral 累积
        self._integral += error * dt

        # Derivative（首次调用或 dt=0 时返回 0）
        if self._prev_error is None or dt <= 0:
            derivative = 0.0
        else:
            derivative = (error - self._prev_error) / dt

        p_term = self.kp * error
        i_term = self.ki * self._integral
        d_term = self.kd * derivative
        output = p_term + i_term + d_term

        self._prev_error = error

        return {
            "output": output,
            "error": error,
            "derivative": derivative,
            "integral": self._integral,
            "p_term": p_term,
            "i_term": i_term,
            "d_term": d_term,
        }
```

- [ ] **Step 8: 跑全部测试验证通过**

```bash
py -m pytest tests/test_pid_controller.py -v
```

Expected: 全部 13 个测试 PASS。

- [ ] **Step 9 (可选): 提交**

```bash
git add pid_exp/pid_controller.py tests/test_pid_controller.py
git commit -m "feat: PIDController with heading wrap-around"
```

---

## Task 3: flight_io HTTP 原语（set_flight_parameter + read_state）

**Files:**
- Create: `D:\work\pilot\aircraft_agent\pid_exp\pid_exp\flight_io.py`
- Create: `D:\work\pilot\aircraft_agent\pid_exp\tests\test_flight_io.py`

- [ ] **Step 1: 写 set_flight_parameter 的测试（mock requests.put）**

文件：`tests/test_flight_io.py`

```python
import json
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
```

- [ ] **Step 2: 写 read_state 的测试**

继续追加到 `tests/test_flight_io.py`：

```python
SAMPLE_GET_RESPONSE = [
    {"name": "PLANE_HEADING_DEGREES_MAGNETIC", "val": 95.5, "unit": "Degrees", "writable": True},
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
```

- [ ] **Step 3: 跑测试验证失败**

```bash
py -m pytest tests/test_flight_io.py -v
```

Expected: 全部 FAIL（模块未实现）。

- [ ] **Step 4: 实现 flight_io.py 的 HTTP 原语部分**

文件：`pid_exp/flight_io.py`

```python
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
```

- [ ] **Step 5: 跑测试验证通过**

```bash
py -m pytest tests/test_flight_io.py -v
```

Expected: 5 个测试全部 PASS。

- [ ] **Step 6 (可选): 提交**

```bash
git add pid_exp/flight_io.py tests/test_flight_io.py
git commit -m "feat: flight_io HTTP primitives (set_flight_parameter, read_state)"
```

---

## Task 4: flight_io 动作函数（拷贝自原 flight_operations.py）

**Files:**
- Modify: `D:\work\pilot\aircraft_agent\pid_exp\pid_exp\flight_io.py`（追加 7 个动作函数）
- Modify: `D:\work\pilot\aircraft_agent\pid_exp\tests\test_flight_io.py`（追加冒烟测试）

参考：源文件 `D:\work\pilot\aircraft_agent\Pilot-FractFlow-main\tools\aircraft\msfs2024tools\flight_operations.py`

- [ ] **Step 1: 写动作函数的冒烟测试（验证调用了正确的 set_flight_parameter）**

追加到 `tests/test_flight_io.py`：

```python
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
```

- [ ] **Step 2: 跑测试验证失败**

```bash
py -m pytest tests/test_flight_io.py -v -k "hover or move"
```

Expected: 全部 FAIL（动作函数未定义）。

- [ ] **Step 3: 实现 7 个动作函数（追加到 flight_io.py）**

追加到 `pid_exp/flight_io.py`：

```python
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
```

- [ ] **Step 4: 跑测试验证通过**

```bash
py -m pytest tests/test_flight_io.py -v
```

Expected: 全部测试 PASS（11 个，包含之前的 + 新增的）。

- [ ] **Step 5 (可选): 提交**

```bash
git add pid_exp/flight_io.py tests/test_flight_io.py
git commit -m "feat: flight_io action functions (move_xxx, hover)"
```

---

## Task 5: flight_io.reset_to + 验证

**Files:**
- Modify: `D:\work\pilot\aircraft_agent\pid_exp\pid_exp\flight_io.py`
- Modify: `D:\work\pilot\aircraft_agent\pid_exp\tests\test_flight_io.py`

- [ ] **Step 1: 写 reset_to 的测试**

追加到 `tests/test_flight_io.py`：

```python
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
        assert ("PLANE_HEADING_DEGREES_MAGNETIC", 90.0) in param_calls
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
```

- [ ] **Step 2: 跑测试验证失败**

```bash
py -m pytest tests/test_flight_io.py -v -k "reset"
```

Expected: 全部 FAIL（reset_to 未实现）。

- [ ] **Step 3: 实现 reset_to**

追加到 `pid_exp/flight_io.py`：

```python
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
```

- [ ] **Step 4: 跑测试验证通过**

```bash
py -m pytest tests/test_flight_io.py -v
```

Expected: 全部测试 PASS（16 个）。

- [ ] **Step 5 (可选): 提交**

```bash
git add pid_exp/flight_io.py tests/test_flight_io.py
git commit -m "feat: flight_io.reset_to with verification"
```

---

## Task 6: experiment.py — 单次实验

**Files:**
- Create: `D:\work\pilot\aircraft_agent\pid_exp\pid_exp\experiment.py`
- Create: `D:\work\pilot\aircraft_agent\pid_exp\tests\test_experiment.py`

- [ ] **Step 1: 写 ExperimentConfig 和 run() 的集成测试（用 fake flight_io）**

文件：`tests/test_experiment.py`

```python
import json
from pathlib import Path
import pytest
from unittest.mock import patch
from pid_exp.experiment import ExperimentConfig, run


class FakeFlightIO:
    """In-memory plant model 模拟飞机。仅用于 experiment.run 的集成测试。
    线性模型：每次动作后按动作时长比例改变状态。
    """
    def __init__(self, initial_heading=90, initial_altitude=5000):
        self.heading = initial_heading
        self.altitude = initial_altitude
        self.action_log = []  # 记录每次动作便于断言

    def read_state(self):
        import time as _t
        return {
            "heading": self.heading,
            "altitude": self.altitude,
            "raw": {},
            "timestamp": _t.time(),
        }

    def reset_to(self, heading, altitude, settle_seconds=5.0):
        self.heading = heading
        self.altitude = altitude

    def hover(self):
        self.action_log.append(("hover", 0))

    def hover_turn_left(self, t):
        self.heading -= t * 3.0  # 假设每秒转 3°
        self.action_log.append(("hover_turn_left", t))

    def hover_turn_right(self, t):
        self.heading += t * 3.0
        self.action_log.append(("hover_turn_right", t))

    def move_ascend(self, t):
        self.altitude += t * 30.0  # 假设每秒升 30ft
        self.action_log.append(("move_ascend", t))

    def move_descend(self, t):
        self.altitude -= t * 30.0
        self.action_log.append(("move_descend", t))


def test_run_writes_log_and_config(tmp_path):
    fake = FakeFlightIO(initial_heading=90, initial_altitude=5000)
    out_dir = tmp_path / "run_001"

    cfg = ExperimentConfig(
        axis="heading",
        kp=2.0, ki=0.1, kd=0.5,
        initial=90, target=120,
        other_axis_value=5000,
        duration_s=5.0,  # 短时长保测试快
        output_dir=out_dir,
        run_label="TestRun",
    )

    # 替换 flight_io 模块为 fake，并把所有 sleep 缩短到 0
    with patch("pid_exp.experiment.flight_io", fake), \
         patch("pid_exp.experiment.time.sleep"):
        run(cfg)

    # 验证 log.csv 存在且至少有 1 行
    log_path = out_dir / "log.csv"
    assert log_path.exists()
    lines = log_path.read_text(encoding="utf-8").strip().split("\n")
    assert len(lines) >= 2  # header + 至少 1 行
    header = lines[0].split(",")
    expected_cols = {
        "t", "target", "current", "error",
        "p_term", "i_term", "d_term",
        "control_raw", "action_time", "saturated",
        "action_cmd", "loop_dt",
    }
    assert expected_cols.issubset(set(header))

    # 验证 config.json 存在并包含 kp/ki/kd
    cfg_path = out_dir / "config.json"
    assert cfg_path.exists()
    cfg_dump = json.loads(cfg_path.read_text(encoding="utf-8"))
    assert cfg_dump["kp"] == 2.0
    assert cfg_dump["ki"] == 0.1
    assert cfg_dump["kd"] == 0.5


def test_run_heading_axis_uses_turn_actions(tmp_path):
    fake = FakeFlightIO(initial_heading=90, initial_altitude=5000)
    out_dir = tmp_path / "run_002"

    cfg = ExperimentConfig(
        axis="heading", kp=2.0, ki=0.0, kd=0.0,
        initial=90, target=120, other_axis_value=5000,
        duration_s=3.0, output_dir=out_dir, run_label="HeadingTest",
    )

    with patch("pid_exp.experiment.flight_io", fake), \
         patch("pid_exp.experiment.time.sleep"):
        run(cfg)

    cmds = [a[0] for a in fake.action_log if a[0] != "hover"]
    # 应该至少有一次 hover_turn_right（heading 要从 90 → 120 增大）
    assert any("hover_turn" in c for c in cmds)


def test_run_altitude_axis_uses_ascend_actions(tmp_path):
    fake = FakeFlightIO(initial_heading=90, initial_altitude=5000)
    out_dir = tmp_path / "run_003"

    cfg = ExperimentConfig(
        axis="altitude", kp=2.0, ki=0.0, kd=0.0,
        initial=5000, target=5200, other_axis_value=90,
        duration_s=3.0, output_dir=out_dir, run_label="AltTest",
    )

    with patch("pid_exp.experiment.flight_io", fake), \
         patch("pid_exp.experiment.time.sleep"):
        run(cfg)

    cmds = [a[0] for a in fake.action_log if a[0] != "hover"]
    assert any("move_ascend" in c or "move_descend" in c for c in cmds)


def test_run_action_time_clipped_to_range(tmp_path):
    """验证 action_time 始终在 [0.1, 1.0] 范围内。"""
    fake = FakeFlightIO(initial_heading=90, initial_altitude=5000)
    out_dir = tmp_path / "run_004"
    cfg = ExperimentConfig(
        axis="heading", kp=100.0, ki=0.0, kd=0.0,  # 故意巨大 kp 触发饱和
        initial=90, target=120, other_axis_value=5000,
        duration_s=3.0, output_dir=out_dir, run_label="SatTest",
    )
    with patch("pid_exp.experiment.flight_io", fake), \
         patch("pid_exp.experiment.time.sleep"):
        run(cfg)

    import csv
    with open(out_dir / "log.csv", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            at = float(row["action_time"])
            assert 0.1 <= at <= 1.0
```

- [ ] **Step 2: 跑测试验证失败**

```bash
py -m pytest tests/test_experiment.py -v
```

Expected: 全部 FAIL（模块未实现）。

- [ ] **Step 3: 实现 experiment.py**

文件：`pid_exp/experiment.py`

```python
"""Single step-response experiment driver."""

from __future__ import annotations

import csv
import json
import logging
import time
from dataclasses import dataclass, asdict
from pathlib import Path

from pid_exp import flight_io
from pid_exp.pid_controller import PIDController

logger = logging.getLogger("experiment")

_LOG_COLUMNS = [
    "t", "target", "current", "error",
    "p_term", "i_term", "d_term",
    "control_raw", "action_time", "saturated",
    "action_cmd", "loop_dt",
]


@dataclass
class ExperimentConfig:
    axis: str                       # "heading" or "altitude"
    kp: float
    ki: float
    kd: float
    initial: float                  # 实验起始值（reset 到这里）
    target: float                   # step 目标
    other_axis_value: float         # 另一轴的固定复位值
    duration_s: float               # 主循环总时长
    output_dir: Path                # 输出目录（log.csv / config.json 写入这里）
    run_label: str                  # 例如 "P+10%"，用于人类阅读


def _pick_action(axis: str, control_output: float):
    """根据 axis 和 PID 输出符号选动作。"""
    if axis == "heading":
        if control_output > 0:
            return "hover_turn_right", flight_io.hover_turn_right
        else:
            return "hover_turn_left", flight_io.hover_turn_left
    else:  # altitude
        if control_output > 0:
            return "move_ascend", flight_io.move_ascend
        else:
            return "move_descend", flight_io.move_descend


def _write_log(rows: list[dict], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=_LOG_COLUMNS)
        writer.writeheader()
        writer.writerows(rows)


def _write_config(cfg: ExperimentConfig, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    dump = asdict(cfg)
    dump["output_dir"] = str(cfg.output_dir)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(dump, f, ensure_ascii=False, indent=2)


def run(cfg: ExperimentConfig) -> None:
    """跑单次 step response：reset → 循环 → 写文件。"""
    logger.info("Experiment start: %s (axis=%s, kp=%.3f, ki=%.3f, kd=%.3f)",
                cfg.run_label, cfg.axis, cfg.kp, cfg.ki, cfg.kd)

    # 1. reset
    if cfg.axis == "heading":
        flight_io.reset_to(heading=cfg.initial, altitude=cfg.other_axis_value)
    else:
        flight_io.reset_to(heading=cfg.other_axis_value, altitude=cfg.initial)

    # 2. 初始化 PID
    pid = PIDController(cfg.kp, cfg.ki, cfg.kd, axis=cfg.axis)

    # 3. 主循环
    rows: list[dict] = []
    t_start = time.time()
    t_prev = t_start

    while time.time() - t_start < cfg.duration_s:
        state = flight_io.read_state()
        current = state[cfg.axis]
        t_now = state["timestamp"]
        dt = t_now - t_prev
        t_prev = t_now

        pid_result = pid.update(current, cfg.target, dt if dt > 0 else 0.01)

        # PID 输出 → 动作时长（保留原映射）
        action_time = max(min(abs(pid_result["output"]) * 0.5, 1.0), 0.1)
        saturated = action_time >= 1.0 - 1e-9

        if pid_result["output"] == 0:
            # 罕见的精确零；跳过本 tick，避免 0 长度动作
            time.sleep(0.1)
            continue

        cmd_name, cmd_fn = _pick_action(cfg.axis, pid_result["output"])
        cmd_fn(action_time)  # 阻塞 action_time + 1s（move_xxx 内部 sleep）

        rows.append({
            "t": t_now - t_start,
            "target": cfg.target,
            "current": current,
            "error": pid_result["error"],
            "p_term": pid_result["p_term"],
            "i_term": pid_result["i_term"],
            "d_term": pid_result["d_term"],
            "control_raw": pid_result["output"],
            "action_time": action_time,
            "saturated": saturated,
            "action_cmd": cmd_name,
            "loop_dt": dt,
        })

    # 4. 写文件
    _write_log(rows, cfg.output_dir / "log.csv")
    _write_config(cfg, cfg.output_dir / "config.json")
    logger.info("Experiment %s done: %d rows logged", cfg.run_label, len(rows))
```

- [ ] **Step 4: 跑测试验证通过**

```bash
py -m pytest tests/test_experiment.py -v
```

Expected: 4 个测试 PASS。

- [ ] **Step 5 (可选): 提交**

```bash
git add pid_exp/experiment.py tests/test_experiment.py
git commit -m "feat: experiment.py single step-response runner"
```

---

## Task 7: metrics.py — 时间响应与精度指标

**Files:**
- Create: `D:\work\pilot\aircraft_agent\pid_exp\pid_exp\metrics.py`
- Create: `D:\work\pilot\aircraft_agent\pid_exp\tests\test_metrics.py`

参考：6 个 metric 定义见 `experiment_design.md` 第 8 节。

- [ ] **Step 1: 写指标计算的测试（用合成的简单响应曲线）**

文件：`tests/test_metrics.py`

```python
import math
import numpy as np
import pandas as pd
import pytest
from pid_exp.metrics import (
    rise_time_10_90,
    peak_time,
    overshoot_pct,
    settling_time_5pct,
    steady_state_error,
    iae,
)


def _make_ideal_step_log(initial=90, target=120, n_steps=50, dt=1.0):
    """造一条理想的 step response：linear rise then flat at target。"""
    rows = []
    step_size = abs(target - initial)
    for i in range(n_steps):
        t = i * dt
        # 前 20 步线性 rise，后面保持
        if i < 20:
            current = initial + (target - initial) * (i / 20)
        else:
            current = target
        rows.append({
            "t": t,
            "target": target,
            "current": current,
            "error": target - current,
            "action_time": 0.5,
            "loop_dt": dt,
        })
    return pd.DataFrame(rows)


def _make_overshoot_log(initial=90, target=120, peak=128, n_steps=50, dt=1.0):
    """造一条带 overshoot 然后衰减的曲线。"""
    rows = []
    for i in range(n_steps):
        t = i * dt
        if i < 15:
            current = initial + (peak - initial) * (i / 15)  # rise to peak
        elif i < 25:
            # 衰减到 target
            current = peak - (peak - target) * ((i - 15) / 10)
        else:
            current = target
        rows.append({
            "t": t,
            "target": target,
            "current": current,
            "error": target - current,
            "action_time": 0.5,
            "loop_dt": dt,
        })
    return pd.DataFrame(rows)


def test_rise_time_ideal_step():
    df = _make_ideal_step_log()
    rt = rise_time_10_90(df, step_size=30, initial=90, target=120)
    # 90% line = error 27 (current=93) → i=2
    # 10% line = error 3 (current=117) → i=18
    # rise_time = (18 - 2) * dt = 16s
    assert rt == pytest.approx(16.0, abs=1.0)


def test_rise_time_no_convergence():
    """完全不动的曲线 → NaN"""
    rows = [{"t": i, "target": 120, "current": 90, "error": 30,
             "action_time": 0.1, "loop_dt": 1.0} for i in range(30)]
    df = pd.DataFrame(rows)
    rt = rise_time_10_90(df, step_size=30, initial=90, target=120)
    assert math.isnan(rt)


def test_peak_time_with_overshoot():
    df = _make_overshoot_log()
    pt = peak_time(df, target=120, initial=90)
    # peak 在 i=15，对应 t=15.0
    assert pt == pytest.approx(15.0, abs=1.0)


def test_peak_time_no_overshoot_equals_rise_time():
    df = _make_ideal_step_log()
    pt = peak_time(df, target=120, initial=90)
    rt = rise_time_10_90(df, step_size=30, initial=90, target=120)
    assert pt == pytest.approx(rt, abs=1.0)


def test_overshoot_pct_with_overshoot():
    df = _make_overshoot_log(initial=90, target=120, peak=128)
    osh = overshoot_pct(df, target=120, initial=90)
    # (128 - 120) / 30 * 100 ≈ 26.67%
    assert osh == pytest.approx(26.67, abs=1.0)


def test_overshoot_pct_no_overshoot():
    df = _make_ideal_step_log()
    osh = overshoot_pct(df, target=120, initial=90)
    assert osh == pytest.approx(0, abs=0.1)


def test_settling_time_ideal():
    df = _make_ideal_step_log()
    st = settling_time_5pct(df, step_size=30, target=120)
    # ±5% step = ±1.5°. current 在 i=18 时 = 117（误差 3 > 1.5 出带），i=19 时 = 118.5（误差 1.5），
    # i=20 时进入并保持。所以 settling ≈ 20s
    assert st == pytest.approx(20.0, abs=1.5)


def test_settling_time_never_settles():
    """长期 oscillate → NaN"""
    rows = []
    for i in range(50):
        # 在 target=120 附近持续 ±5° 振荡（超过 ±5% × 30 = ±1.5°）
        current = 120 + 5 * (1 if i % 2 == 0 else -1)
        rows.append({"t": i, "target": 120, "current": current, "error": 120 - current,
                     "action_time": 0.5, "loop_dt": 1.0})
    df = pd.DataFrame(rows)
    st = settling_time_5pct(df, step_size=30, target=120)
    assert math.isnan(st)


def test_steady_state_error_ideal():
    df = _make_ideal_step_log()
    sse = steady_state_error(df, target=120, last_n_seconds=5)
    # 末段全是 120，error=0
    assert sse == pytest.approx(0, abs=0.01)


def test_steady_state_error_persistent_offset():
    rows = []
    for i in range(50):
        # 整个实验都偏 target 2°
        rows.append({"t": i, "target": 120, "current": 118, "error": 2,
                     "action_time": 0.5, "loop_dt": 1.0})
    df = pd.DataFrame(rows)
    sse = steady_state_error(df, target=120, last_n_seconds=5)
    assert sse == pytest.approx(2.0, abs=0.1)


def test_iae_zero_error():
    rows = [{"t": i, "target": 120, "current": 120, "error": 0,
             "action_time": 0.5, "loop_dt": 1.0} for i in range(30)]
    df = pd.DataFrame(rows)
    assert iae(df) == pytest.approx(0, abs=0.01)


def test_iae_constant_error():
    # 30 行，每行 |error|=2, dt=1 → IAE = 60
    rows = [{"t": i, "target": 120, "current": 118, "error": 2,
             "action_time": 0.5, "loop_dt": 1.0} for i in range(30)]
    df = pd.DataFrame(rows)
    assert iae(df) == pytest.approx(60, abs=0.1)
```

- [ ] **Step 2: 跑测试验证失败**

```bash
py -m pytest tests/test_metrics.py -v
```

Expected: 全部 FAIL（metrics 模块未创建）。

- [ ] **Step 3: 实现 6 个 metric 函数**

文件：`pid_exp/metrics.py`

```python
"""Compute scalar metrics from a single experiment's log.csv."""

from __future__ import annotations

import math
import numpy as np
import pandas as pd


def rise_time_10_90(df: pd.DataFrame, step_size: float, initial: float, target: float) -> float:
    """error 从 90% step → 10% step 用的时间。无法计算 = NaN。"""
    if step_size <= 0 or len(df) == 0:
        return math.nan
    # 直接用 |error| 而非 |current - initial|，统一处理 heading wrap
    abs_err = df["error"].abs().values
    t = df["t"].values
    threshold_90 = 0.9 * step_size  # error >= threshold_90 = 还在远端
    threshold_10 = 0.1 * step_size  # error <= threshold_10 = 接近目标

    # 找第一个 |error| < 90% step 的时刻（即 current 从 initial 进入 "10% close" 区）
    below_90_idx = np.where(abs_err < threshold_90)[0]
    if len(below_90_idx) == 0:
        return math.nan
    i_90 = below_90_idx[0]

    # 然后从 i_90 起找第一个 |error| <= 10% step 的时刻
    below_10_idx = np.where(abs_err[i_90:] <= threshold_10)[0]
    if len(below_10_idx) == 0:
        return math.nan
    i_10 = i_90 + below_10_idx[0]

    return float(t[i_10] - t[i_90])


def peak_time(df: pd.DataFrame, target: float, initial: float) -> float:
    """current 第一次达到极值的时间。不超调则取 rise_time（current 第一次接近 target 的时间）。"""
    if len(df) == 0:
        return math.nan
    current = df["current"].values
    t = df["t"].values
    direction = 1 if target > initial else -1

    # 是否有超调：current 是否突破过 target？
    if direction > 0:
        overshoots = current > target
    else:
        overshoots = current < target

    if overshoots.any():
        # 第一次超过 target 后的第一个局部极值
        i_first_over = np.where(overshoots)[0][0]
        # 从 i_first_over 起找峰值（朝 direction 的极值）
        rest = current[i_first_over:]
        if direction > 0:
            i_peak_rel = np.argmax(rest)
        else:
            i_peak_rel = np.argmin(rest)
        return float(t[i_first_over + i_peak_rel])
    else:
        # 不超调 = 取第一次到达 target 的时刻（用 |error| 最小的点）
        i_min_err = np.argmin(np.abs(df["error"].values))
        return float(t[i_min_err])


def overshoot_pct(df: pd.DataFrame, target: float, initial: float) -> float:
    """(peak_value - target) / step_size × 100；不超调 = 0。"""
    if len(df) == 0:
        return math.nan
    step_size = abs(target - initial)
    if step_size <= 0:
        return 0.0
    current = df["current"].values
    direction = 1 if target > initial else -1
    if direction > 0:
        peak = current.max()
        if peak <= target:
            return 0.0
        return float((peak - target) / step_size * 100)
    else:
        peak = current.min()
        if peak >= target:
            return 0.0
        return float((target - peak) / step_size * 100)


def settling_time_5pct(df: pd.DataFrame, step_size: float, target: float) -> float:
    """error 进入 ±5% step 后剩余时间不再出去的最早时刻。永不 = NaN。"""
    if step_size <= 0 or len(df) == 0:
        return math.nan
    abs_err = df["error"].abs().values
    t = df["t"].values
    band = 0.05 * step_size

    # 从后往前找第一个超出 band 的位置；之后就一直在带内
    out_of_band_idx = np.where(abs_err > band)[0]
    if len(out_of_band_idx) == 0:
        # 全程都在带内
        return float(t[0])
    last_out = out_of_band_idx[-1]
    if last_out == len(abs_err) - 1:
        # 最后一行还在带外 → 永不 settle
        return math.nan
    return float(t[last_out + 1])


def steady_state_error(df: pd.DataFrame, target: float, last_n_seconds: float = 5.0) -> float:
    """末段 last_n_seconds 内的平均 |error|。"""
    if len(df) == 0:
        return math.nan
    t_end = df["t"].iloc[-1]
    mask = df["t"] >= (t_end - last_n_seconds)
    tail = df.loc[mask, "error"].abs()
    if len(tail) == 0:
        return math.nan
    return float(tail.mean())


def iae(df: pd.DataFrame) -> float:
    """∑|error| × loop_dt （梯形积分用 loop_dt 近似）。"""
    if len(df) == 0:
        return math.nan
    return float((df["error"].abs() * df["loop_dt"]).sum())
```

- [ ] **Step 4: 跑测试验证通过**

```bash
py -m pytest tests/test_metrics.py -v
```

Expected: 12 个测试全部 PASS。

- [ ] **Step 5 (可选): 提交**

```bash
git add pid_exp/metrics.py tests/test_metrics.py
git commit -m "feat: 6 step-response metrics"
```

---

## Task 8: metrics.compute() 和 aggregate()

**Files:**
- Modify: `D:\work\pilot\aircraft_agent\pid_exp\pid_exp\metrics.py`
- Modify: `D:\work\pilot\aircraft_agent\pid_exp\tests\test_metrics.py`

- [ ] **Step 1: 写 compute() 和 aggregate() 的测试**

追加到 `tests/test_metrics.py`：

```python
import json
from pathlib import Path
from pid_exp.metrics import compute, aggregate


def _write_fake_run(run_dir: Path, kp=2.0, ki=0.1, kd=0.5,
                    axis="heading", initial=90, target=120,
                    run_label="Nominal", n=30):
    """造一个完整的 run 目录（config.json + log.csv）用于测试 compute/aggregate。"""
    run_dir.mkdir(parents=True, exist_ok=True)
    cfg = {
        "axis": axis, "kp": kp, "ki": ki, "kd": kd,
        "initial": initial, "target": target,
        "other_axis_value": 5000 if axis == "heading" else 90,
        "duration_s": float(n), "output_dir": str(run_dir),
        "run_label": run_label,
    }
    (run_dir / "config.json").write_text(json.dumps(cfg), encoding="utf-8")

    # 写一条理想 rise 曲线
    import csv as _csv
    with open(run_dir / "log.csv", "w", encoding="utf-8", newline="") as f:
        writer = _csv.DictWriter(f, fieldnames=[
            "t", "target", "current", "error",
            "p_term", "i_term", "d_term",
            "control_raw", "action_time", "saturated",
            "action_cmd", "loop_dt",
        ])
        writer.writeheader()
        for i in range(n):
            t = i * 1.0
            if i < 20:
                current = initial + (target - initial) * (i / 20)
            else:
                current = target
            writer.writerow({
                "t": t, "target": target, "current": current,
                "error": target - current,
                "p_term": 0, "i_term": 0, "d_term": 0,
                "control_raw": 1.0, "action_time": 0.5, "saturated": False,
                "action_cmd": "hover_turn_right", "loop_dt": 1.0,
            })


def test_compute_writes_metrics_json(tmp_path):
    run_dir = tmp_path / "run_01"
    _write_fake_run(run_dir)
    result = compute(run_dir)
    assert "rise_time_10_90" in result
    assert "peak_time" in result
    assert "overshoot_pct" in result
    assert "settling_time_5pct" in result
    assert "steady_state_error" in result
    assert "iae" in result
    # 文件也应该写入
    metrics_path = run_dir / "metrics.json"
    assert metrics_path.exists()
    on_disk = json.loads(metrics_path.read_text(encoding="utf-8"))
    assert on_disk == result


def test_aggregate_collects_all_runs(tmp_path):
    sweep_dir = tmp_path / "sweep_xyz"
    runs_dir = sweep_dir / "runs"
    for i, label in enumerate(["Nominal", "P+10%", "P-10%"]):
        run_dir = runs_dir / f"{i+1:02d}_{label}"
        _write_fake_run(run_dir, kp=2.0 * (1 + 0.1 * i), run_label=label)
        compute(run_dir)

    summary_path = aggregate(sweep_dir)
    assert summary_path.exists()
    import pandas as pd
    df = pd.read_csv(summary_path)
    assert len(df) == 3
    assert "run_label" in df.columns
    assert "kp" in df.columns
    assert "rise_time_10_90" in df.columns
    assert set(df["run_label"]) == {"Nominal", "P+10%", "P-10%"}
```

- [ ] **Step 2: 跑测试验证失败**

```bash
py -m pytest tests/test_metrics.py -v -k "compute or aggregate"
```

Expected: FAIL（compute / aggregate 未实现）。

- [ ] **Step 3: 实现 compute 和 aggregate**

追加到 `pid_exp/metrics.py`：

```python
import json
from pathlib import Path


_ALL_METRIC_NAMES = [
    "rise_time_10_90",
    "peak_time",
    "overshoot_pct",
    "settling_time_5pct",
    "steady_state_error",
    "iae",
]


def compute(run_dir: Path) -> dict:
    """从 run_dir/log.csv + config.json 算出 metric dict，写 metrics.json，返回 dict。"""
    run_dir = Path(run_dir)
    log_path = run_dir / "log.csv"
    cfg_path = run_dir / "config.json"
    if not log_path.exists():
        raise FileNotFoundError(f"log.csv not found in {run_dir}")
    if not cfg_path.exists():
        raise FileNotFoundError(f"config.json not found in {run_dir}")

    cfg = json.loads(cfg_path.read_text(encoding="utf-8"))
    df = pd.read_csv(log_path)
    initial = cfg["initial"]
    target = cfg["target"]
    step_size = abs(target - initial)

    metrics_dict = {
        "rise_time_10_90": rise_time_10_90(df, step_size, initial, target),
        "peak_time": peak_time(df, target, initial),
        "overshoot_pct": overshoot_pct(df, target, initial),
        "settling_time_5pct": settling_time_5pct(df, step_size, target),
        "steady_state_error": steady_state_error(df, target, last_n_seconds=5.0),
        "iae": iae(df),
    }
    # 把 NaN 序列化为 null
    serializable = {k: (None if isinstance(v, float) and math.isnan(v) else v)
                    for k, v in metrics_dict.items()}
    (run_dir / "metrics.json").write_text(
        json.dumps(serializable, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    return metrics_dict


def aggregate(sweep_dir: Path) -> Path:
    """读 sweep_dir/runs/*/metrics.json + config.json，写 sweep_dir/summary.csv。返回 csv 路径。"""
    sweep_dir = Path(sweep_dir)
    runs_dir = sweep_dir / "runs"
    if not runs_dir.exists():
        raise FileNotFoundError(f"{runs_dir} not found")

    rows = []
    for run_dir in sorted(runs_dir.iterdir()):
        if not run_dir.is_dir():
            continue
        cfg_path = run_dir / "config.json"
        metrics_path = run_dir / "metrics.json"
        if not cfg_path.exists() or not metrics_path.exists():
            continue
        cfg = json.loads(cfg_path.read_text(encoding="utf-8"))
        metrics_dict = json.loads(metrics_path.read_text(encoding="utf-8"))
        row = {
            "run_id": run_dir.name,
            "run_label": cfg.get("run_label", ""),
            "axis": cfg["axis"],
            "kp": cfg["kp"], "ki": cfg["ki"], "kd": cfg["kd"],
            **{m: metrics_dict.get(m) for m in _ALL_METRIC_NAMES},
        }
        rows.append(row)

    summary_path = sweep_dir / "summary.csv"
    pd.DataFrame(rows).to_csv(summary_path, index=False)
    return summary_path
```

- [ ] **Step 4: 跑全部 metrics 测试**

```bash
py -m pytest tests/test_metrics.py -v
```

Expected: 全部 PASS（14 个）。

- [ ] **Step 5 (可选): 提交**

```bash
git add pid_exp/metrics.py tests/test_metrics.py
git commit -m "feat: metrics.compute() and aggregate()"
```

---

## Task 9: sweep.py — OFAT 列表 + run_sweep

**Files:**
- Create: `D:\work\pilot\aircraft_agent\pid_exp\pid_exp\sweep.py`
- Create: `D:\work\pilot\aircraft_agent\pid_exp\tests\test_sweep.py`

参考：OFAT 测试列表见 `experiment_design.md` 第 5 节，nominal 值见第 3 节。

- [ ] **Step 1: 写 build_ofat_list 的测试**

文件：`tests/test_sweep.py`

```python
from pathlib import Path
import pytest
from pid_exp.sweep import SweepConfig, build_ofat_list


def _basic_cfg(tmp_path, axis="heading"):
    return SweepConfig(
        axis=axis,
        nominal_kp=2.0 if axis == "heading" else 1.5,
        nominal_ki=0.1 if axis == "heading" else 0.05,
        nominal_kd=0.5,
        initial=90 if axis == "heading" else 5000,
        target=120 if axis == "heading" else 5200,
        other_axis_value=5000 if axis == "heading" else 90,
        duration_s=45.0,
        output_root=tmp_path,
    )


def test_ofat_list_length_13(tmp_path):
    cfg = _basic_cfg(tmp_path)
    exp_cfgs = build_ofat_list(cfg)
    assert len(exp_cfgs) == 13


def test_ofat_labels_are_correct(tmp_path):
    cfg = _basic_cfg(tmp_path)
    exp_cfgs = build_ofat_list(cfg)
    labels = [e.run_label for e in exp_cfgs]
    expected = ["Nominal",
                "P-20%", "P-10%", "P+10%", "P+20%",
                "I-20%", "I-10%", "I+10%", "I+20%",
                "D-20%", "D-10%", "D+10%", "D+20%"]
    assert labels == expected


def test_ofat_heading_nominal_values(tmp_path):
    cfg = _basic_cfg(tmp_path)
    exp_cfgs = build_ofat_list(cfg)
    nominal = exp_cfgs[0]
    assert nominal.kp == pytest.approx(2.0)
    assert nominal.ki == pytest.approx(0.1)
    assert nominal.kd == pytest.approx(0.5)


def test_ofat_p_variants(tmp_path):
    cfg = _basic_cfg(tmp_path)
    exp_cfgs = build_ofat_list(cfg)
    # P-20%, P-10%, P+10%, P+20% 是 index 1-4
    assert exp_cfgs[1].kp == pytest.approx(1.6)   # P-20%
    assert exp_cfgs[2].kp == pytest.approx(1.8)   # P-10%
    assert exp_cfgs[3].kp == pytest.approx(2.2)   # P+10%
    assert exp_cfgs[4].kp == pytest.approx(2.4)   # P+20%
    # 这些 run ki/kd 应该都是 nominal
    for i in range(1, 5):
        assert exp_cfgs[i].ki == pytest.approx(0.1)
        assert exp_cfgs[i].kd == pytest.approx(0.5)


def test_ofat_i_variants(tmp_path):
    cfg = _basic_cfg(tmp_path)
    exp_cfgs = build_ofat_list(cfg)
    # I 变体 index 5-8
    assert exp_cfgs[5].ki == pytest.approx(0.08)  # I-20%
    assert exp_cfgs[6].ki == pytest.approx(0.09)  # I-10%
    assert exp_cfgs[7].ki == pytest.approx(0.11)  # I+10%
    assert exp_cfgs[8].ki == pytest.approx(0.12)  # I+20%
    for i in range(5, 9):
        assert exp_cfgs[i].kp == pytest.approx(2.0)
        assert exp_cfgs[i].kd == pytest.approx(0.5)


def test_ofat_d_variants(tmp_path):
    cfg = _basic_cfg(tmp_path)
    exp_cfgs = build_ofat_list(cfg)
    # D 变体 index 9-12
    assert exp_cfgs[9].kd == pytest.approx(0.4)   # D-20%
    assert exp_cfgs[10].kd == pytest.approx(0.45) # D-10%
    assert exp_cfgs[11].kd == pytest.approx(0.55) # D+10%
    assert exp_cfgs[12].kd == pytest.approx(0.6)  # D+20%
    for i in range(9, 13):
        assert exp_cfgs[i].kp == pytest.approx(2.0)
        assert exp_cfgs[i].ki == pytest.approx(0.1)


def test_ofat_altitude_axis(tmp_path):
    cfg = _basic_cfg(tmp_path, axis="altitude")
    exp_cfgs = build_ofat_list(cfg)
    nominal = exp_cfgs[0]
    assert nominal.axis == "altitude"
    assert nominal.kp == pytest.approx(1.5)
    assert nominal.ki == pytest.approx(0.05)
    assert nominal.kd == pytest.approx(0.5)
    # P-20% on altitude
    assert exp_cfgs[1].kp == pytest.approx(1.2)


def test_ofat_output_dirs_unique(tmp_path):
    cfg = _basic_cfg(tmp_path)
    exp_cfgs = build_ofat_list(cfg)
    dirs = [e.output_dir for e in exp_cfgs]
    assert len(set(dirs)) == 13  # 全部不同
```

- [ ] **Step 2: 跑测试验证失败**

```bash
py -m pytest tests/test_sweep.py -v
```

Expected: 全部 FAIL（sweep 模块未实现）。

- [ ] **Step 3: 实现 SweepConfig 和 build_ofat_list**

文件：`pid_exp/sweep.py`

> **注意 import 顺序**：这一步只引入 build_ofat_list 需要的依赖；Task 10 才会把 `analysis`、`flight_io`、`metrics`、`experiment` 这几个 module 引入并实现 run_sweep。这样 Task 9 完成后即可独立通过测试。

```python
"""OFAT sweep driver."""

from __future__ import annotations

import datetime
import json
import logging
from dataclasses import dataclass, asdict
from pathlib import Path

from pid_exp.experiment import ExperimentConfig

logger = logging.getLogger("sweep")


@dataclass
class SweepConfig:
    axis: str                    # "heading" or "altitude"
    nominal_kp: float
    nominal_ki: float
    nominal_kd: float
    initial: float
    target: float
    other_axis_value: float
    duration_s: float
    output_root: Path
    sweep_label: str | None = None  # 默认按时间戳生成


_DELTAS = [
    (-0.20, "20%"),
    (-0.10, "10%"),
    (+0.10, "10%"),
    (+0.20, "20%"),
]


def _make_run_label(prefix: str, delta_pct: float, delta_lbl: str) -> str:
    sign = "+" if delta_pct > 0 else "-"
    return f"{prefix}{sign}{delta_lbl}"


def build_ofat_list(cfg: SweepConfig) -> list[ExperimentConfig]:
    """生成 13 个 ExperimentConfig：1 Nominal + 4 P + 4 I + 4 D。"""
    p0, i0, d0 = cfg.nominal_kp, cfg.nominal_ki, cfg.nominal_kd

    triplets = [(p0, i0, d0, "Nominal")]
    triplets += [(p0 * (1 + δ), i0, d0, _make_run_label("P", δ, lbl))
                 for δ, lbl in _DELTAS]
    triplets += [(p0, i0 * (1 + δ), d0, _make_run_label("I", δ, lbl))
                 for δ, lbl in _DELTAS]
    triplets += [(p0, i0, d0 * (1 + δ), _make_run_label("D", δ, lbl))
                 for δ, lbl in _DELTAS]

    sweep_dir = _ensure_sweep_dir(cfg)

    out = []
    for idx, (kp, ki, kd, label) in enumerate(triplets, start=1):
        # 文件名用安全字符（替换 % 和 +）
        safe_lbl = label.replace("%", "pct").replace("+", "p").replace("-", "n")
        run_dir = sweep_dir / "runs" / f"{idx:02d}_{safe_lbl}"
        out.append(ExperimentConfig(
            axis=cfg.axis,
            kp=kp, ki=ki, kd=kd,
            initial=cfg.initial, target=cfg.target,
            other_axis_value=cfg.other_axis_value,
            duration_s=cfg.duration_s,
            output_dir=run_dir,
            run_label=label,
        ))
    return out


def _ensure_sweep_dir(cfg: SweepConfig) -> Path:
    if cfg.sweep_label is None:
        ts = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M")
        cfg.sweep_label = f"{ts}_{cfg.axis}_OFAT"
    sweep_dir = cfg.output_root / cfg.sweep_label
    sweep_dir.mkdir(parents=True, exist_ok=True)
    return sweep_dir
```

- [ ] **Step 4: 跑测试验证通过**

```bash
py -m pytest tests/test_sweep.py -v
```

Expected: 全部 PASS（8 个）。

- [ ] **Step 5 (可选): 提交**

```bash
git add pid_exp/sweep.py tests/test_sweep.py
git commit -m "feat: sweep OFAT list builder"
```

---

## Task 10: sweep.run_sweep + CLI

**Files:**
- Modify: `D:\work\pilot\aircraft_agent\pid_exp\pid_exp\sweep.py`
- Modify: `D:\work\pilot\aircraft_agent\pid_exp\tests\test_sweep.py`

- [ ] **Step 1: 写 run_sweep 的集成测试（mock experiment.run + analysis.build_outputs）**

追加到 `tests/test_sweep.py`：

```python
from unittest.mock import patch, MagicMock
from pid_exp.sweep import run_sweep


def test_run_sweep_calls_experiment_for_each_run(tmp_path):
    cfg = _basic_cfg(tmp_path)
    # 注意：analysis.build_outputs 真实实现在 Task 14；本测试运行时 analysis.py 可能仍是空 stub，
    # 所以加 create=True 让 patch 容忍属性不存在。其他 patch 的函数已在前面任务实现。
    with patch("pid_exp.sweep.experiment.run") as mock_run, \
         patch("pid_exp.sweep.metrics.compute"), \
         patch("pid_exp.sweep.metrics.aggregate") as mock_agg, \
         patch("pid_exp.sweep.analysis.build_outputs", create=True) as mock_analysis, \
         patch("pid_exp.sweep.flight_io.hover"):
        mock_agg.return_value = tmp_path / "summary.csv"
        sweep_dir = run_sweep(cfg)
        assert mock_run.call_count == 13
        mock_agg.assert_called_once()
        mock_analysis.assert_called_once()
        assert sweep_dir.exists()


def test_run_sweep_tolerates_few_failures(tmp_path):
    cfg = _basic_cfg(tmp_path)
    call_count = {"n": 0}

    def flaky_run(exp_cfg):
        call_count["n"] += 1
        if call_count["n"] == 2:
            raise RuntimeError("simulated failure")

    with patch("pid_exp.sweep.experiment.run", side_effect=flaky_run), \
         patch("pid_exp.sweep.metrics.compute"), \
         patch("pid_exp.sweep.metrics.aggregate") as mock_agg, \
         patch("pid_exp.sweep.analysis.build_outputs", create=True), \
         patch("pid_exp.sweep.flight_io.hover"):
        mock_agg.return_value = tmp_path / "summary.csv"
        sweep_dir = run_sweep(cfg)
        assert call_count["n"] == 13  # 应该继续后续 run
        failed_log = sweep_dir / "failed.json"
        assert failed_log.exists()


def test_run_sweep_aborts_on_too_many_failures(tmp_path):
    cfg = _basic_cfg(tmp_path)
    with patch("pid_exp.sweep.experiment.run", side_effect=RuntimeError("fail")), \
         patch("pid_exp.sweep.metrics.compute"), \
         patch("pid_exp.sweep.metrics.aggregate"), \
         patch("pid_exp.sweep.analysis.build_outputs", create=True), \
         patch("pid_exp.sweep.flight_io.hover"):
        with pytest.raises(RuntimeError, match="失败次数"):
            run_sweep(cfg)
```

- [ ] **Step 2: 跑测试验证失败**

```bash
py -m pytest tests/test_sweep.py -v -k "run_sweep"
```

Expected: FAIL（run_sweep 未实现）。

- [ ] **Step 3: 实现 run_sweep + CLI**

追加到 `pid_exp/sweep.py`：

```python
import argparse
import sys

from pid_exp import flight_io, metrics, experiment, analysis


_MAX_FAILURES = 3


def _save_sweep_config(cfg: SweepConfig, sweep_dir: Path) -> None:
    dump = asdict(cfg)
    dump["output_root"] = str(cfg.output_root)
    (sweep_dir / "sweep_config.json").write_text(
        json.dumps(dump, ensure_ascii=False, indent=2), encoding="utf-8"
    )


def run_sweep(cfg: SweepConfig) -> Path:
    sweep_dir = _ensure_sweep_dir(cfg)
    _save_sweep_config(cfg, sweep_dir)

    exp_cfgs = build_ofat_list(cfg)
    logger.info("Starting sweep %s: %d runs", cfg.sweep_label, len(exp_cfgs))

    failed = []
    for exp_cfg in exp_cfgs:
        try:
            experiment.run(exp_cfg)
            metrics.compute(exp_cfg.output_dir)
        except Exception as e:
            logger.exception("Run %s failed: %s", exp_cfg.run_label, e)
            failed.append({"run_label": exp_cfg.run_label, "error": str(e)})
            if len(failed) > _MAX_FAILURES:
                _write_failed(sweep_dir, failed)
                raise RuntimeError(
                    f"失败次数超过 {_MAX_FAILURES}，中止 sweep"
                ) from e
        # run 之间显式 hover，确保下一次 reset 的初始扰动小
        try:
            flight_io.hover()
        except Exception:
            logger.warning("inter-run hover failed", exc_info=True)

    if failed:
        _write_failed(sweep_dir, failed)

    metrics.aggregate(sweep_dir)
    analysis.build_outputs(sweep_dir)
    return sweep_dir


def _write_failed(sweep_dir: Path, failed: list[dict]) -> None:
    (sweep_dir / "failed.json").write_text(
        json.dumps(failed, ensure_ascii=False, indent=2), encoding="utf-8"
    )


def _make_cli() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="PID OFAT sweep driver")
    parser.add_argument("--axis", choices=["heading", "altitude"], required=True,
                        help="哪个轴做 OFAT")
    parser.add_argument("--kp", type=float, default=None, help="nominal Kp（默认按 axis 取标准值）")
    parser.add_argument("--ki", type=float, default=None, help="nominal Ki")
    parser.add_argument("--kd", type=float, default=None, help="nominal Kd")
    parser.add_argument("--initial", type=float, default=None,
                        help="初始值（默认 heading=90, altitude=5000）")
    parser.add_argument("--target", type=float, default=None,
                        help="step 目标（默认 heading=120, altitude=5200）")
    parser.add_argument("--other-axis", type=float, default=None,
                        help="另一轴固定复位值（默认 heading 实验 alt=5000；alt 实验 heading=90）")
    parser.add_argument("--duration", type=float, default=45.0)
    parser.add_argument("--output-root", type=Path,
                        default=Path("results"))
    parser.add_argument("--sweep-label", type=str, default=None,
                        help="结果目录名（默认时间戳）")
    return parser


_AXIS_DEFAULTS = {
    "heading":  dict(kp=2.0, ki=0.10, kd=0.5, initial=90,   target=120,  other=5000),
    "altitude": dict(kp=1.5, ki=0.05, kd=0.5, initial=5000, target=5200, other=90),
}


def main(argv: list[str] | None = None) -> int:
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s")
    args = _make_cli().parse_args(argv)
    d = _AXIS_DEFAULTS[args.axis]
    cfg = SweepConfig(
        axis=args.axis,
        nominal_kp=args.kp if args.kp is not None else d["kp"],
        nominal_ki=args.ki if args.ki is not None else d["ki"],
        nominal_kd=args.kd if args.kd is not None else d["kd"],
        initial=args.initial if args.initial is not None else d["initial"],
        target=args.target if args.target is not None else d["target"],
        other_axis_value=args.other_axis if args.other_axis is not None else d["other"],
        duration_s=args.duration,
        output_root=args.output_root,
        sweep_label=args.sweep_label,
    )
    sweep_dir = run_sweep(cfg)
    print(f"\nSweep finished. Results in: {sweep_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 4: 跑测试验证通过**

```bash
py -m pytest tests/test_sweep.py -v
```

Expected: 全部 PASS（11 个）。

- [ ] **Step 5: 验证 CLI 帮助能正常输出**

```bash
py -m pid_exp.sweep --help
```

Expected: 看到 argparse 帮助文本，无 error。

- [ ] **Step 6 (可选): 提交**

```bash
git add pid_exp/sweep.py tests/test_sweep.py
git commit -m "feat: run_sweep with retry + CLI entrypoint"
```

---

## Task 11: analysis.py — OFAT 敏感性数值计算

**Files:**
- Create: `D:\work\pilot\aircraft_agent\pid_exp\pid_exp\analysis.py`
- Create: `D:\work\pilot\aircraft_agent\pid_exp\tests\test_analysis.py`

- [ ] **Step 1: 写 compute_sensitivity 的测试**

文件：`tests/test_analysis.py`

```python
import math
import pytest
import pandas as pd
from pid_exp.analysis import compute_sensitivity


def _ofat_summary_with_linear_trend(slope=2.0, intercept=10.0, nominal_kp=2.0):
    """造一份 summary，让 metric 严格线性 = slope * pct + intercept。"""
    rows = []
    deltas = [(-0.20, "P-20%"), (-0.10, "P-10%"), (0.00, "Nominal"),
              (+0.10, "P+10%"), (+0.20, "P+20%")]
    for δ, lbl in deltas:
        kp = nominal_kp * (1 + δ)
        metric_value = slope * δ + intercept
        rows.append({
            "run_label": lbl, "axis": "heading",
            "kp": kp, "ki": 0.1, "kd": 0.5,
            "rise_time_10_90": metric_value,
            "peak_time": 0, "overshoot_pct": 0,
            "settling_time_5pct": 0, "steady_state_error": 0, "iae": 0,
        })
    # 加一个 Nominal 真实条目（已在 deltas 里），还要加 I 和 D 变体（变化都是 0）
    for prefix in ["I", "D"]:
        for δ, suffix in [(-0.20, "20%"), (-0.10, "10%"), (+0.10, "10%"), (+0.20, "20%")]:
            sign = "+" if δ > 0 else "-"
            rows.append({
                "run_label": f"{prefix}{sign}{suffix}",
                "axis": "heading", "kp": 2.0, "ki": 0.1, "kd": 0.5,
                "rise_time_10_90": intercept,
                "peak_time": 0, "overshoot_pct": 0,
                "settling_time_5pct": 0, "steady_state_error": 0, "iae": 0,
            })
    return pd.DataFrame(rows)


def test_compute_sensitivity_linear_perfect_fit():
    df = _ofat_summary_with_linear_trend(slope=2.0, intercept=10.0)
    result = compute_sensitivity(df, metric="rise_time_10_90", param="kp")
    # slope 是 metric 单位 per 100% 变化 = polyfit slope
    assert result["slope"] == pytest.approx(2.0, abs=1e-6)
    # 5 个点：metric = slope * pct + intercept，pct ∈ {-0.2, ..., +0.2}
    # range = max - min = (2*0.2+10) - (2*(-0.2)+10) = 0.8
    assert result["range"] == pytest.approx(0.8, abs=1e-6)
    assert result["r2"] == pytest.approx(1.0, abs=1e-6)


def test_compute_sensitivity_flat_metric():
    df = _ofat_summary_with_linear_trend(slope=0.0, intercept=5.0)
    result = compute_sensitivity(df, metric="rise_time_10_90", param="kp")
    assert result["slope"] == pytest.approx(0.0, abs=1e-6)
    assert result["range"] == pytest.approx(0.0, abs=1e-6)
    # 全部相同 → R² 退化为 NaN（SS_tot = 0）
    assert math.isnan(result["r2"])


def test_compute_sensitivity_unrelated_param():
    """对 ki 求 sensitivity，但 metric 实际只受 kp 影响 → I 变体的 metric 都等于 intercept。"""
    df = _ofat_summary_with_linear_trend(slope=2.0, intercept=10.0)
    # I-variants 的 rise_time 全是 intercept=10，所以 5 个点完全水平
    result = compute_sensitivity(df, metric="rise_time_10_90", param="ki")
    assert result["slope"] == pytest.approx(0.0, abs=1e-6)
    assert result["range"] == pytest.approx(0.0, abs=1e-6)


def test_compute_sensitivity_with_nan_skips():
    """有 NaN 时只用有效点拟合。"""
    df = _ofat_summary_with_linear_trend(slope=2.0, intercept=10.0)
    # P+20% 设为 NaN
    df.loc[df["run_label"] == "P+20%", "rise_time_10_90"] = float("nan")
    result = compute_sensitivity(df, metric="rise_time_10_90", param="kp")
    # 4 点仍是线性 → slope ≈ 2, R² 仍很高
    assert result["slope"] == pytest.approx(2.0, abs=0.01)
    assert result["r2"] > 0.99


def test_compute_sensitivity_all_nan_returns_nan():
    df = _ofat_summary_with_linear_trend(slope=2.0, intercept=10.0)
    df.loc[df["run_label"].str.startswith("P"), "rise_time_10_90"] = float("nan")
    df.loc[df["run_label"] == "Nominal", "rise_time_10_90"] = float("nan")
    result = compute_sensitivity(df, metric="rise_time_10_90", param="kp")
    assert math.isnan(result["slope"])
    assert math.isnan(result["r2"])
    assert math.isnan(result["range"])
```

- [ ] **Step 2: 跑测试验证失败**

```bash
py -m pytest tests/test_analysis.py -v
```

Expected: FAIL（analysis 未创建）。

- [ ] **Step 3: 实现 compute_sensitivity**

文件：`pid_exp/analysis.py`

```python
"""OFAT sensitivity analysis: numerical computation + table/plot generation."""

from __future__ import annotations

import logging
import math
from pathlib import Path

import numpy as np
import pandas as pd

logger = logging.getLogger("analysis")


# OFAT label → (param, fraction) 映射
_LABEL_DELTA = {
    "Nominal": ("nominal", 0.00),
    "P-20%": ("kp", -0.20), "P-10%": ("kp", -0.10),
    "P+10%": ("kp", +0.10), "P+20%": ("kp", +0.20),
    "I-20%": ("ki", -0.20), "I-10%": ("ki", -0.10),
    "I+10%": ("ki", +0.10), "I+20%": ("ki", +0.20),
    "D-20%": ("kd", -0.20), "D-10%": ("kd", -0.10),
    "D+10%": ("kd", +0.10), "D+20%": ("kd", +0.20),
}


def _extract_param_curve(df: pd.DataFrame, metric: str, param: str) -> tuple[np.ndarray, np.ndarray]:
    """从 summary 抽出某参数的 5 个 OFAT 点 (pct, metric_value)。Nominal 是公共原点 0%。"""
    pcts: list[float] = []
    values: list[float] = []
    for _, row in df.iterrows():
        if row["run_label"] not in _LABEL_DELTA:
            continue
        which_param, δ = _LABEL_DELTA[row["run_label"]]
        if which_param == "nominal" or which_param == param:
            pcts.append(δ)
            values.append(row[metric])
    arr_pct = np.array(pcts, dtype=float)
    arr_val = np.array(values, dtype=float)
    # 按 pct 排序，去掉 NaN
    order = np.argsort(arr_pct)
    arr_pct = arr_pct[order]
    arr_val = arr_val[order]
    valid = ~np.isnan(arr_val)
    return arr_pct[valid], arr_val[valid]


def compute_sensitivity(df: pd.DataFrame, metric: str, param: str) -> dict:
    """返回 {slope, range, r2}。slope 单位是 metric per 100% 参数变化。"""
    pcts, values = _extract_param_curve(df, metric, param)

    nan_result = {"slope": math.nan, "range": math.nan, "r2": math.nan}
    if len(pcts) < 2:
        return nan_result

    # 一阶 polyfit
    slope, intercept = np.polyfit(pcts, values, deg=1)
    range_val = float(values.max() - values.min())

    # R²
    pred = slope * pcts + intercept
    ss_res = float(((values - pred) ** 2).sum())
    ss_tot = float(((values - values.mean()) ** 2).sum())
    if ss_tot < 1e-12:
        r2 = math.nan
    else:
        r2 = 1.0 - ss_res / ss_tot

    return {"slope": float(slope), "range": range_val, "r2": r2}
```

- [ ] **Step 4: 跑测试验证通过**

```bash
py -m pytest tests/test_analysis.py -v
```

Expected: 全部 PASS（5 个）。

- [ ] **Step 5 (可选): 提交**

```bash
git add pid_exp/analysis.py tests/test_analysis.py
git commit -m "feat: OFAT sensitivity numerical computation"
```

---

## Task 12: analysis — sensitivity_table.md 生成

**Files:**
- Modify: `D:\work\pilot\aircraft_agent\pid_exp\pid_exp\analysis.py`
- Modify: `D:\work\pilot\aircraft_agent\pid_exp\tests\test_analysis.py`

- [ ] **Step 1: 写 sensitivity_table 生成的测试**

追加到 `tests/test_analysis.py`：

```python
from pid_exp.analysis import write_sensitivity_table


def test_sensitivity_table_writes_markdown(tmp_path):
    df = _ofat_summary_with_linear_trend(slope=2.0, intercept=10.0)
    out_path = tmp_path / "sensitivity_table.md"
    write_sensitivity_table(df, out_path, axis="heading",
                            nominal_kp=2.0, nominal_ki=0.1, nominal_kd=0.5,
                            initial=90, target=120)
    text = out_path.read_text(encoding="utf-8")
    # 包含 6 个 metric 名（至少匹配前缀）
    assert "rise_time_10_90" in text
    assert "peak_time" in text
    assert "overshoot_pct" in text
    assert "settling_time_5pct" in text
    assert "steady_state_error" in text
    assert "iae" in text
    # 包含 Kp/Ki/Kd 列
    assert "Kp 影响" in text or "Kp" in text
    assert "Ki" in text and "Kd" in text
    # 包含 nominal 值的提示
    assert "2.0" in text or "2.00" in text


def test_sensitivity_table_handles_all_nan(tmp_path):
    df = _ofat_summary_with_linear_trend()
    df["rise_time_10_90"] = float("nan")
    out_path = tmp_path / "sensitivity_table.md"
    # 不应抛错
    write_sensitivity_table(df, out_path, axis="heading",
                            nominal_kp=2.0, nominal_ki=0.1, nominal_kd=0.5,
                            initial=90, target=120)
    text = out_path.read_text(encoding="utf-8")
    assert "NaN" in text or "nan" in text
```

- [ ] **Step 2: 跑测试验证失败**

```bash
py -m pytest tests/test_analysis.py -v -k "sensitivity_table"
```

Expected: FAIL。

- [ ] **Step 3: 实现 write_sensitivity_table**

追加到 `pid_exp/analysis.py`：

```python
_METRICS_TO_REPORT = [
    "rise_time_10_90",
    "peak_time",
    "overshoot_pct",
    "settling_time_5pct",
    "steady_state_error",
    "iae",
]

_METRIC_UNITS = {
    "rise_time_10_90": "s",
    "peak_time": "s",
    "overshoot_pct": "%",
    "settling_time_5pct": "s",
    "steady_state_error": "err_unit",
    "iae": "err_unit·s",
}


def _fmt_num(x: float, sig: int = 3) -> str:
    if x is None or (isinstance(x, float) and math.isnan(x)):
        return "NaN"
    if abs(x) < 0.01:
        return f"{x:.4f}"
    return f"{x:.{sig}g}"


def _get_nominal_value(df: pd.DataFrame, metric: str) -> float:
    nominal_rows = df[df["run_label"] == "Nominal"]
    if len(nominal_rows) == 0:
        return math.nan
    val = nominal_rows.iloc[0][metric]
    return float(val) if not pd.isna(val) else math.nan


def write_sensitivity_table(df: pd.DataFrame, out_path: Path, axis: str,
                            nominal_kp: float, nominal_ki: float, nominal_kd: float,
                            initial: float, target: float) -> None:
    """生成数字化的 sensitivity_table.md。"""
    step = abs(target - initial)
    err_unit = "°" if axis == "heading" else "ft"
    step_unit = "°" if axis == "heading" else "ft"

    lines = []
    lines.append(f"# {axis.capitalize()} 轴 PID 超参数敏感性\n")
    lines.append(
        f"Nominal: Kp={nominal_kp}, Ki={nominal_ki}, Kd={nominal_kd}  |  "
        f"Step: {initial} → {target} ({step}{step_unit} 阶跃)\n"
    )

    # 表头
    lines.append("| Metric | Nominal | Kp 影响 (slope/range/R²) | Ki 影响 (slope/range/R²) | Kd 影响 (slope/range/R²) |")
    lines.append("|---|---|---|---|---|")

    for m in _METRICS_TO_REPORT:
        unit = _METRIC_UNITS[m].replace("err_unit", err_unit)
        nominal_val = _get_nominal_value(df, m)
        cells = [f"{m} ({unit})", _fmt_num(nominal_val)]
        for param in ("kp", "ki", "kd"):
            s = compute_sensitivity(df, m, param)
            cells.append(
                f"{_fmt_num(s['slope'])} / {_fmt_num(s['range'])} / {_fmt_num(s['r2'])}"
            )
        lines.append("| " + " | ".join(cells) + " |")

    lines.append("\n阅读说明：")
    lines.append("- slope 单位：metric 单位 per 100% 参数变化（例如 Kp 列 slope=-8.32 表示 Kp 增大 100%，rise_time 约减 8.32s）")
    lines.append("- range = 5 个 OFAT 点（-20/-10/Nominal/+10/+20%）的 max - min")
    lines.append("- R² 接近 1 表示 5 个点线性单调；R² 小说明非线性（如 U 型）")

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text("\n".join(lines), encoding="utf-8")
```

- [ ] **Step 4: 跑测试验证通过**

```bash
py -m pytest tests/test_analysis.py -v
```

Expected: 全部 PASS（7 个）。

- [ ] **Step 5 (可选): 提交**

```bash
git add pid_exp/analysis.py tests/test_analysis.py
git commit -m "feat: sensitivity_table.md generator"
```

---

## Task 13: analysis — 两张图（step_response_overlay + sensitivity_plots）

**Files:**
- Modify: `D:\work\pilot\aircraft_agent\pid_exp\pid_exp\analysis.py`
- Modify: `D:\work\pilot\aircraft_agent\pid_exp\tests\test_analysis.py`

- [ ] **Step 1: 写绘图函数的冒烟测试（只验证 PNG 生成 + 非空）**

追加到 `tests/test_analysis.py`：

```python
from pid_exp.analysis import plot_step_response_overlay, plot_sensitivity_grid


def _make_minimal_run_dir(parent: Path, label: str, kp=2.0, ki=0.1, kd=0.5):
    import json as _json, csv as _csv
    safe_lbl = label.replace("%", "pct").replace("+", "p").replace("-", "n")
    rd = parent / f"01_{safe_lbl}"
    rd.mkdir(parents=True, exist_ok=True)
    (rd / "config.json").write_text(_json.dumps({
        "axis": "heading", "kp": kp, "ki": ki, "kd": kd,
        "initial": 90, "target": 120, "other_axis_value": 5000,
        "duration_s": 30.0, "output_dir": str(rd), "run_label": label,
    }), encoding="utf-8")
    with open(rd / "log.csv", "w", encoding="utf-8", newline="") as f:
        w = _csv.DictWriter(f, fieldnames=[
            "t", "target", "current", "error",
            "p_term", "i_term", "d_term",
            "control_raw", "action_time", "saturated",
            "action_cmd", "loop_dt",
        ])
        w.writeheader()
        for i in range(30):
            current = 90 + min(30, i * 1.2)
            w.writerow({
                "t": i, "target": 120, "current": current, "error": 120 - current,
                "p_term": 0, "i_term": 0, "d_term": 0,
                "control_raw": 1.0, "action_time": 0.5, "saturated": False,
                "action_cmd": "hover_turn_right", "loop_dt": 1.0,
            })
    return rd


def test_plot_step_response_overlay_writes_png(tmp_path):
    runs_dir = tmp_path / "runs"
    for label in ["Nominal", "P-20%", "P-10%", "P+10%", "P+20%",
                  "I-20%", "I-10%", "I+10%", "I+20%",
                  "D-20%", "D-10%", "D+10%", "D+20%"]:
        _make_minimal_run_dir(runs_dir, label)
    out_path = tmp_path / "step_response_overlay.png"
    plot_step_response_overlay(tmp_path, out_path, axis="heading")
    assert out_path.exists()
    assert out_path.stat().st_size > 1000  # 应该是个像样的 PNG


def test_plot_sensitivity_grid_writes_png(tmp_path):
    df = _ofat_summary_with_linear_trend(slope=2.0, intercept=10.0)
    out_path = tmp_path / "sensitivity_plots.png"
    plot_sensitivity_grid(df, out_path, axis="heading")
    assert out_path.exists()
    assert out_path.stat().st_size > 1000
```

- [ ] **Step 2: 跑测试验证失败**

```bash
py -m pytest tests/test_analysis.py -v -k "plot"
```

Expected: FAIL。

- [ ] **Step 3: 实现两个绘图函数**

追加到 `pid_exp/analysis.py`：

```python
import matplotlib
matplotlib.use("Agg")  # 无头模式，不弹窗
import matplotlib.pyplot as plt


_P_VARIANTS = ["P-20%", "P-10%", "Nominal", "P+10%", "P+20%"]
_I_VARIANTS = ["I-20%", "I-10%", "Nominal", "I+10%", "I+20%"]
_D_VARIANTS = ["D-20%", "D-10%", "Nominal", "D+10%", "D+20%"]


def _safe_label(label: str) -> str:
    return label.replace("%", "pct").replace("+", "p").replace("-", "n")


def _load_run_log(sweep_dir: Path, label: str) -> pd.DataFrame | None:
    """根据 run_label 在 sweep_dir/runs/ 下找对应 log.csv，找不到返回 None。"""
    runs_dir = sweep_dir / "runs"
    target = _safe_label(label)
    for child in runs_dir.iterdir():
        if not child.is_dir():
            continue
        if child.name.endswith(f"_{target}"):
            log_path = child / "log.csv"
            if log_path.exists():
                return pd.read_csv(log_path)
    return None


def plot_step_response_overlay(sweep_dir: Path, out_path: Path, axis: str) -> None:
    """3 子图（P/I/D 各一），每子图叠加 5 条响应曲线。"""
    sweep_dir = Path(sweep_dir)
    fig, axes = plt.subplots(1, 3, figsize=(18, 5), sharey=True)
    titles = ["Kp 扫描", "Ki 扫描", "Kd 扫描"]
    variant_sets = [_P_VARIANTS, _I_VARIANTS, _D_VARIANTS]
    colors = ["#7c3aed", "#3b82f6", "#000000", "#f59e0b", "#dc2626"]

    for ax, title, variants in zip(axes, titles, variant_sets):
        for label, color in zip(variants, colors):
            df = _load_run_log(sweep_dir, label)
            if df is None or len(df) == 0:
                continue
            ax.plot(df["t"], df["current"], label=label, color=color,
                    linewidth=1.5 if label == "Nominal" else 1.0)
            # 画 target line
            ax.axhline(df["target"].iloc[0], linestyle="--", color="gray", alpha=0.3)
        ax.set_title(title)
        ax.set_xlabel("t (s)")
        ax.legend(loc="best", fontsize=8)
        ax.grid(True, alpha=0.3)

    axes[0].set_ylabel(f"{axis} ({'°' if axis == 'heading' else 'ft'})")
    fig.suptitle(f"Step response overlay ({axis} axis)")
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=120)
    plt.close(fig)


def plot_sensitivity_grid(df: pd.DataFrame, out_path: Path, axis: str) -> None:
    """6 metric × 3 参数 = 18 子图，每子图是 5 点散点 + 拟合直线。"""
    fig, axes = plt.subplots(len(_METRICS_TO_REPORT), 3,
                             figsize=(13, 2.5 * len(_METRICS_TO_REPORT)))
    params = ("kp", "ki", "kd")

    for row_i, metric in enumerate(_METRICS_TO_REPORT):
        for col_i, param in enumerate(params):
            ax = axes[row_i, col_i]
            pcts, values = _extract_param_curve(df, metric, param)
            if len(pcts) >= 2:
                ax.scatter(pcts * 100, values, color="black", zorder=3)
                # 拟合
                slope, intercept = np.polyfit(pcts, values, deg=1)
                x_fit = np.linspace(-0.25, 0.25, 50)
                ax.plot(x_fit * 100, slope * x_fit + intercept,
                        color="red", alpha=0.6, linewidth=1)
                sense = compute_sensitivity(df, metric, param)
                ax.set_title(f"{metric} vs {param}  R²={_fmt_num(sense['r2'], 2)}",
                             fontsize=9)
            else:
                ax.set_title(f"{metric} vs {param}  (no data)", fontsize=9)
            ax.axvline(0, color="gray", linestyle="--", alpha=0.3)
            ax.set_xlabel(f"Δ{param} (%)", fontsize=8)
            ax.tick_params(labelsize=7)
            ax.grid(True, alpha=0.3)

    fig.suptitle(f"OFAT sensitivity ({axis} axis)")
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=120)
    plt.close(fig)
```

- [ ] **Step 4: 跑测试验证通过**

```bash
py -m pytest tests/test_analysis.py -v
```

Expected: 全部 PASS（9 个）。

- [ ] **Step 5 (可选): 提交**

```bash
git add pid_exp/analysis.py tests/test_analysis.py
git commit -m "feat: step response overlay and sensitivity grid plots"
```

---

## Task 14: analysis.build_outputs 协调器

**Files:**
- Modify: `D:\work\pilot\aircraft_agent\pid_exp\pid_exp\analysis.py`
- Modify: `D:\work\pilot\aircraft_agent\pid_exp\tests\test_analysis.py`

- [ ] **Step 1: 写 build_outputs 的端到端测试**

追加到 `tests/test_analysis.py`：

```python
from pid_exp.analysis import build_outputs


def test_build_outputs_creates_all_files(tmp_path):
    sweep_dir = tmp_path / "sweep_xyz"
    runs_dir = sweep_dir / "runs"
    from pid_exp.metrics import compute, aggregate
    for label in ["Nominal", "P-20%", "P-10%", "P+10%", "P+20%",
                  "I-20%", "I-10%", "I+10%", "I+20%",
                  "D-20%", "D-10%", "D+10%", "D+20%"]:
        rd = _make_minimal_run_dir(runs_dir, label)
        compute(rd)  # 写 metrics.json

    aggregate(sweep_dir)  # 写 summary.csv

    # 写 sweep_config.json
    import json as _json
    (sweep_dir / "sweep_config.json").write_text(_json.dumps({
        "axis": "heading", "nominal_kp": 2.0, "nominal_ki": 0.1, "nominal_kd": 0.5,
        "initial": 90, "target": 120,
    }), encoding="utf-8")

    build_outputs(sweep_dir)
    assert (sweep_dir / "sensitivity_table.md").exists()
    assert (sweep_dir / "step_response_overlay.png").exists()
    assert (sweep_dir / "sensitivity_plots.png").exists()
```

- [ ] **Step 2: 跑测试验证失败**

```bash
py -m pytest tests/test_analysis.py -v -k "build_outputs"
```

Expected: FAIL。

- [ ] **Step 3: 实现 build_outputs**

追加到 `pid_exp/analysis.py`：

```python
import json as _json


def build_outputs(sweep_dir: Path) -> None:
    """读 sweep_dir/summary.csv 和 sweep_config.json，生成 3 个分析产物。"""
    sweep_dir = Path(sweep_dir)
    summary_path = sweep_dir / "summary.csv"
    cfg_path = sweep_dir / "sweep_config.json"

    if not summary_path.exists():
        raise FileNotFoundError(f"{summary_path} not found")
    if not cfg_path.exists():
        raise FileNotFoundError(f"{cfg_path} not found")

    summary = pd.read_csv(summary_path)
    cfg = _json.loads(cfg_path.read_text(encoding="utf-8"))

    write_sensitivity_table(
        summary,
        sweep_dir / "sensitivity_table.md",
        axis=cfg["axis"],
        nominal_kp=cfg["nominal_kp"],
        nominal_ki=cfg["nominal_ki"],
        nominal_kd=cfg["nominal_kd"],
        initial=cfg["initial"],
        target=cfg["target"],
    )
    plot_step_response_overlay(
        sweep_dir,
        sweep_dir / "step_response_overlay.png",
        axis=cfg["axis"],
    )
    plot_sensitivity_grid(
        summary,
        sweep_dir / "sensitivity_plots.png",
        axis=cfg["axis"],
    )
    logger.info("Analysis outputs written to %s", sweep_dir)
```

- [ ] **Step 4: 跑全部 analysis 测试**

```bash
py -m pytest tests/test_analysis.py -v
```

Expected: 全部 PASS（10 个）。

- [ ] **Step 5 (可选): 提交**

```bash
git add pid_exp/analysis.py tests/test_analysis.py
git commit -m "feat: build_outputs orchestrator"
```

---

## Task 15: 端到端冒烟 + README

**Files:**
- Create: `D:\work\pilot\aircraft_agent\pid_exp\README.md`

- [ ] **Step 1: 跑全部测试，确认整体没有回归**

```bash
cd D:/work/pilot/aircraft_agent/pid_exp
py -m pytest tests/ -v
```

Expected: 全部测试 PASS（约 50 个）。

- [ ] **Step 2: 手工冒烟（需要真实 MSFS 在跑 + .env 配好）**

只跑一个 Nominal run，duration_s=15s 以快速验证管线：

```bash
cd D:/work/pilot/aircraft_agent/pid_exp
cp .env.example .env
# 编辑 .env 填入真实的 API_URL_CTRL 和 API_URL_GET

# 先用 python REPL 跑一次单 run
py -c "
import logging
from pathlib import Path
from pid_exp.experiment import ExperimentConfig, run
logging.basicConfig(level=logging.INFO)
cfg = ExperimentConfig(
    axis='heading', kp=2.0, ki=0.1, kd=0.5,
    initial=90, target=120, other_axis_value=5000,
    duration_s=15.0, output_dir=Path('results/_smoke/01_Nominal'),
    run_label='Nominal',
)
run(cfg)
print('单 run 完成')
"
```

Expected: 看到 `Experiment Nominal done: N rows logged`，`results/_smoke/01_Nominal/` 下有 `log.csv` 和 `config.json`。

如果失败：检查 `.env` 是否正确、MSFS 桥接服务是否可达、飞机是否在天上（不在地面）。

- [ ] **Step 3: 跑完整 sweep（需要真实 MSFS，约 11 min）**

```bash
py -m pid_exp.sweep --axis heading --duration 45
```

Expected: 看到 13 个 run 依次完成；结束时打印 `Sweep finished. Results in: results/<timestamp>_heading_OFAT`。

验证：该目录下应有 `sweep_config.json`、`runs/`（13 子目录，每个含 log.csv/config.json/metrics.json）、`summary.csv`、`sensitivity_table.md`、`step_response_overlay.png`、`sensitivity_plots.png`。

如果中途 >3 次失败：sweep 会自动中止并写 `failed.json`。检查那里的错误信息。

- [ ] **Step 4: 写 README.md**

文件：`D:\work\pilot\aircraft_agent\pid_exp\README.md`

````markdown
# PID 超参数 OFAT 敏感性实验台

独立的 PID 实验环境，连真实 MSFS 2024，跑 OFAT (One-Factor-At-a-Time) sweep 测 PID 三个超参数对 step response 的影响。

详细设计见 `docs/code_design.md`、`docs/experiment_design.md`、`docs/metric_explanation.md`。

## 安装

```bash
cd D:/work/pilot/aircraft_agent/pid_exp
py -m pip install -e .
py -m pip install -r requirements.txt
```

## 配置

```bash
cp .env.example .env
# 编辑 .env 填入 MSFS HTTP bridge 的 API_URL_CTRL 和 API_URL_GET
```

## 跑测试

```bash
py -m pytest tests/ -v
```

## 跑 sweep（需要 MSFS 在跑 + 飞机在天上）

```bash
# heading 轴默认 13 run OFAT（约 11 min）
py -m pid_exp.sweep --axis heading

# altitude 轴
py -m pid_exp.sweep --axis altitude

# 自定义 nominal 参数
py -m pid_exp.sweep --axis heading --kp 2.0 --ki 0.1 --kd 0.5
```

## 输出

每次 sweep 在 `results/<timestamp>_<axis>_OFAT/` 下生成：

- `sweep_config.json` — 本次 sweep 的元数据
- `runs/NN_<label>/` — 13 个单 run 目录，各含：
  - `config.json` — 该 run 的 PID 参数和实验条件
  - `log.csv` — 主循环每帧的时间序列（target/current/error/PID 分项/动作）
  - `metrics.json` — 算出来的 6 个标量 metric
- `summary.csv` — 13 行汇总，每行一个 run
- `sensitivity_table.md` — **核心交付物**：每个 metric 对每个超参数的 slope/range/R²
- `step_response_overlay.png` — 3 子图叠加曲线，直观看 P/I/D 各 5 档变体对响应的影响
- `sensitivity_plots.png` — 6×3 子图，每个 metric 对每个参数的 5 点散点 + 拟合

## 故障排查

- **HTTP 超时**：检查 MSFS bridge 是否在跑，URL 是否正确
- **复位验证失败**：飞机可能在地面或处于异常状态，需要先手动飞起来稳定
- **某个 metric 一直是 NaN**：该 run 没收敛（PID 参数不好或时长不够）—— 看 `step_response_overlay.png` 对应曲线
- **>3 个 run 失败导致中止**：看 `failed.json` 里的错误信息

## 项目结构

```
pid_exp/
├── docs/                       # 设计文档（必读）
├── pid_exp/                    # 源代码
│   ├── flight_io.py            # MSFS HTTP wrapper
│   ├── pid_controller.py       # 1D PID with heading wrap
│   ├── experiment.py           # 单次 step response 实验
│   ├── metrics.py              # 6 个 metric 计算
│   ├── sweep.py                # OFAT 驱动 + CLI
│   └── analysis.py             # 敏感性表 + 图生成
├── tests/                      # pytest 测试
├── results/                    # 运行时输出（不入 git）
├── .env                        # 配置（不入 git）
├── .env.example
├── requirements.txt
└── pyproject.toml
```
````

- [ ] **Step 5: 验证 README 完整**

```bash
cat D:/work/pilot/aircraft_agent/pid_exp/README.md | wc -l
```

Expected: 至少 60 行。

- [ ] **Step 6 (可选): 最终提交**

```bash
git add README.md
git commit -m "docs: README with setup and usage instructions"
```

---

## 完成判定

- [ ] 所有 15 个任务的步骤打勾完成
- [ ] `py -m pytest tests/ -v` 全部测试通过（~50 个）
- [ ] 在真实 MSFS 上手工跑过至少 1 个 sweep，得到 3 张产出文件（sensitivity_table.md + 2 张 PNG）
- [ ] README 写好，新人能照着跑起来

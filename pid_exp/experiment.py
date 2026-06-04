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
        # pid_result 是 dict: {output, error, derivative, integral, p_term, i_term, d_term}

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

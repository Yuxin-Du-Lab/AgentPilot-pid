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

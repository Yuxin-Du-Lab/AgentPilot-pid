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
    rt = rise_time_10_90(df, target=120, initial=90)
    # 90% line = error 27 (current=93) → i=2
    # 10% line = error 3 (current=117) → i=18
    # rise_time = (18 - 2) * dt = 16s
    assert rt == pytest.approx(16.0, abs=1.0)


def test_rise_time_no_convergence():
    """完全不动的曲线 → NaN"""
    rows = [{"t": i, "target": 120, "current": 90, "error": 30,
             "action_time": 0.1, "loop_dt": 1.0} for i in range(30)]
    df = pd.DataFrame(rows)
    rt = rise_time_10_90(df, target=120, initial=90)
    assert math.isnan(rt)


def test_peak_time_with_overshoot():
    df = _make_overshoot_log()
    pt = peak_time(df, target=120, initial=90)
    # peak 在 i=15，对应 t=15.0
    assert pt == pytest.approx(15.0, abs=1.0)


def test_peak_time_no_overshoot_equals_rise_time():
    df = _make_ideal_step_log()
    pt = peak_time(df, target=120, initial=90)
    rt = rise_time_10_90(df, target=120, initial=90)
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
    st = settling_time_5pct(df, target=120, initial=90)
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
    st = settling_time_5pct(df, target=120, initial=90)
    assert math.isnan(st)


def test_steady_state_error_ideal():
    df = _make_ideal_step_log()
    sse = steady_state_error(df, last_n_seconds=5)
    # 末段全是 120，error=0
    assert sse == pytest.approx(0, abs=0.01)


def test_steady_state_error_persistent_offset():
    rows = []
    for i in range(50):
        # 整个实验都偏 target 2°
        rows.append({"t": i, "target": 120, "current": 118, "error": 2,
                     "action_time": 0.5, "loop_dt": 1.0})
    df = pd.DataFrame(rows)
    sse = steady_state_error(df, last_n_seconds=5)
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

"""Compute scalar metrics from a single experiment's log.csv."""

from __future__ import annotations

import math
import numpy as np
import pandas as pd


def rise_time_10_90(df: pd.DataFrame, target: float, initial: float) -> float:
    """error 从 90% step → 10% step 用的时间。无法计算 = NaN。"""
    step_size = abs(target - initial)
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
    """current 第一次达到极值的时间。不超调则取 rise_time（与 rise_time_10_90 同值）。"""
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
        # 不超调 = 取 rise_time（与 rise_time_10_90 等价）
        return rise_time_10_90(df, target=target, initial=initial)


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


def settling_time_5pct(df: pd.DataFrame, target: float, initial: float) -> float:
    """error 进入 ±5% step 后剩余时间不再出去的最早时刻。永不 = NaN。"""
    step_size = abs(target - initial)
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


def steady_state_error(df: pd.DataFrame, last_n_seconds: float = 5.0) -> float:
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

    metrics_dict = {
        "rise_time_10_90": rise_time_10_90(df, target=target, initial=initial),
        "peak_time": peak_time(df, target=target, initial=initial),
        "overshoot_pct": overshoot_pct(df, target=target, initial=initial),
        "settling_time_5pct": settling_time_5pct(df, target=target, initial=initial),
        "steady_state_error": steady_state_error(df, last_n_seconds=5.0),
        "iae": iae(df),
    }
    # 把 NaN 序列化为 null
    serializable = {k: (None if isinstance(v, float) and math.isnan(v) else v)
                    for k, v in metrics_dict.items()}
    (run_dir / "metrics.json").write_text(
        json.dumps(serializable, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    return metrics_dict


def _rep_metric_values(config_dir: Path):
    """读 config_dir 下所有 r*/ 的 config.json + metrics.json。返回 (meta, [rep dict])。"""
    reps, meta = [], None
    for rep_dir in sorted(config_dir.iterdir()):
        if not rep_dir.is_dir():
            continue
        cfg_path, metrics_path = rep_dir / "config.json", rep_dir / "metrics.json"
        if not cfg_path.exists() or not metrics_path.exists():
            continue
        cfg = json.loads(cfg_path.read_text(encoding="utf-8"))
        md = json.loads(metrics_path.read_text(encoding="utf-8"))
        if meta is None:
            meta = {"run_label": cfg.get("run_label", ""), "axis": cfg["axis"],
                    "kp": cfg["kp"], "ki": cfg["ki"], "kd": cfg["kd"]}
        reps.append({"rep": rep_dir.name, **md})
    return meta, reps


def _aggregate_stat(values):
    """过滤 None/NaN → (mean, std, n)。n<2→std=0；n==0→全 NaN。"""
    vals = [float(v) for v in values
            if v is not None and not (isinstance(v, float) and math.isnan(v))]
    n = len(vals)
    if n == 0:
        return math.nan, math.nan, 0
    mean = float(np.mean(vals))
    std = float(np.std(vals, ddof=1)) if n >= 2 else 0.0
    return mean, std, n


def aggregate(sweep_dir: Path) -> Path:
    """两级聚合 runs/<config>/r<NN>/。写 summary.csv（每配置 mean/std/n）
    + summary_runs.csv（每 rep 原始值）。返回 summary.csv 路径。"""
    sweep_dir = Path(sweep_dir)
    runs_dir = sweep_dir / "runs"
    if not runs_dir.exists():
        raise FileNotFoundError(f"{runs_dir} not found")

    summary_rows, rep_rows = [], []
    for config_dir in sorted(runs_dir.iterdir()):
        if not config_dir.is_dir():
            continue
        meta, reps = _rep_metric_values(config_dir)
        if meta is None or not reps:
            continue
        for r in reps:
            rep_rows.append({
                "run_id": config_dir.name, "rep": r["rep"],
                "run_label": meta["run_label"], "axis": meta["axis"],
                "kp": meta["kp"], "ki": meta["ki"], "kd": meta["kd"],
                **{m: r.get(m) for m in _ALL_METRIC_NAMES},
            })
        row = {"run_id": config_dir.name, "run_label": meta["run_label"],
               "axis": meta["axis"], "kp": meta["kp"], "ki": meta["ki"], "kd": meta["kd"]}
        for m in _ALL_METRIC_NAMES:
            mean, std, n = _aggregate_stat([r.get(m) for r in reps])
            row[f"{m}_mean"], row[f"{m}_std"], row[f"{m}_n"] = mean, std, n
        summary_rows.append(row)

    summary_path = sweep_dir / "summary.csv"
    pd.DataFrame(summary_rows).to_csv(summary_path, index=False)
    pd.DataFrame(rep_rows).to_csv(sweep_dir / "summary_runs.csv", index=False)
    return summary_path

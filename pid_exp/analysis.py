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

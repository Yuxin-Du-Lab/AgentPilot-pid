import math
from pathlib import Path
import pytest
import pandas as pd
from pid_exp.analysis import compute_sensitivity


def _ofat_summary_with_linear_trend(slope=2.0, intercept=10.0, nominal_kp=2.0, std=0.5):
    """造一份 summary（每 metric 带 _mean/_std/_n），rise_time 严格线性 = slope*pct + intercept。"""
    metric_names = ["rise_time_10_90", "peak_time", "overshoot_pct",
                    "settling_time_5pct", "steady_state_error", "iae"]
    rows = []
    deltas = [(-0.20, "P-20%"), (-0.10, "P-10%"), (0.00, "Nominal"),
              (+0.10, "P+10%"), (+0.20, "P+20%")]
    for δ, lbl in deltas:
        row = {"run_label": lbl, "axis": "heading",
               "kp": nominal_kp * (1 + δ), "ki": 0.1, "kd": 0.5}
        for m in metric_names:
            row[f"{m}_mean"] = (slope * δ + intercept) if m == "rise_time_10_90" else 0.0
            row[f"{m}_std"], row[f"{m}_n"] = std, 10
        rows.append(row)
    # 加 I 和 D 变体（变化都是 0）
    for prefix in ["I", "D"]:
        for δ, suffix in [(-0.20, "20%"), (-0.10, "10%"), (+0.10, "10%"), (+0.20, "20%")]:
            sign = "+" if δ > 0 else "-"
            row = {"run_label": f"{prefix}{sign}{suffix}", "axis": "heading",
                   "kp": 2.0, "ki": 0.1, "kd": 0.5}
            for m in metric_names:
                row[f"{m}_mean"] = intercept if m == "rise_time_10_90" else 0.0
                row[f"{m}_std"], row[f"{m}_n"] = std, 10
            rows.append(row)
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
    df.loc[df["run_label"] == "P+20%", "rise_time_10_90_mean"] = float("nan")
    result = compute_sensitivity(df, metric="rise_time_10_90", param="kp")
    # 4 点仍是线性 → slope ≈ 2, R² 仍很高
    assert result["slope"] == pytest.approx(2.0, abs=0.01)
    assert result["r2"] > 0.99


def test_compute_sensitivity_all_nan_returns_nan():
    df = _ofat_summary_with_linear_trend(slope=2.0, intercept=10.0)
    df.loc[df["run_label"].str.startswith("P"), "rise_time_10_90_mean"] = float("nan")
    df.loc[df["run_label"] == "Nominal", "rise_time_10_90_mean"] = float("nan")
    result = compute_sensitivity(df, metric="rise_time_10_90", param="kp")
    assert math.isnan(result["slope"])
    assert math.isnan(result["r2"])
    assert math.isnan(result["range"])


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
    df["rise_time_10_90_mean"] = float("nan")
    out_path = tmp_path / "sensitivity_table.md"
    # 不应抛错
    write_sensitivity_table(df, out_path, axis="heading",
                            nominal_kp=2.0, nominal_ki=0.1, nominal_kd=0.5,
                            initial=90, target=120)
    text = out_path.read_text(encoding="utf-8")
    assert "NaN" in text or "nan" in text


from pid_exp.analysis import plot_step_response_overlay, plot_sensitivity_grid


def _make_minimal_run_dir(parent: Path, label: str, kp=2.0, ki=0.1, kd=0.5, rep="r01"):
    import json as _json, csv as _csv
    safe_lbl = label.replace("%", "pct").replace("+", "p").replace("-", "n")
    rd = parent / f"01_{safe_lbl}" / rep
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

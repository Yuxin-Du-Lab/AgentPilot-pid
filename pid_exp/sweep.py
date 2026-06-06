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
    repeats: int = 10               # 每个配置重复次数


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
    triplets += [(p0 * (1 + delta), i0, d0, _make_run_label("P", delta, lbl))
                 for delta, lbl in _DELTAS]
    triplets += [(p0, i0 * (1 + delta), d0, _make_run_label("I", delta, lbl))
                 for delta, lbl in _DELTAS]
    triplets += [(p0, i0, d0 * (1 + delta), _make_run_label("D", delta, lbl))
                 for delta, lbl in _DELTAS]

    sweep_dir = _ensure_sweep_dir(cfg)

    out = []
    for idx, (kp, ki, kd, label) in enumerate(triplets, start=1):
        # 文件名用安全字符（替换 % 和 +）
        safe_lbl = label.replace("%", "pct").replace("+", "p").replace("-", "n")
        config_dir = sweep_dir / "runs" / f"{idx:02d}_{safe_lbl}"
        # 分组顺序：同一配置的 repeats 次连续排在一起
        for rep in range(1, cfg.repeats + 1):
            out.append(ExperimentConfig(
                axis=cfg.axis,
                kp=kp, ki=ki, kd=kd,
                initial=cfg.initial, target=cfg.target,
                other_axis_value=cfg.other_axis_value,
                duration_s=cfg.duration_s,
                output_dir=config_dir / f"r{rep:02d}",
                run_label=label,
            ))
    return out


def _ensure_sweep_dir(cfg: SweepConfig) -> Path:
    if cfg.sweep_label is None:
        ts = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")  # 改成秒精度
        cfg.sweep_label = f"{ts}_{cfg.axis}_OFAT"
    sweep_dir = cfg.output_root / cfg.sweep_label
    sweep_dir.mkdir(parents=True, exist_ok=True)
    return sweep_dir


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
    # 失败阈值随总 run 数放大，130-run sweep 容忍零散瞬时失败
    max_failures = max(_MAX_FAILURES, len(exp_cfgs) // 10)
    n_configs = len(exp_cfgs) // cfg.repeats if cfg.repeats else len(exp_cfgs)
    logger.info("Starting sweep %s: %d runs (%d configs × %d reps)",
                cfg.sweep_label, len(exp_cfgs), n_configs, cfg.repeats)

    failed = []
    for exp_cfg in exp_cfgs:
        try:
            experiment.run(exp_cfg)
            metrics.compute(exp_cfg.output_dir)
        except Exception as e:
            logger.exception("Run %s failed: %s", exp_cfg.run_label, e)
            failed.append({"run_label": exp_cfg.run_label,
                           "output_dir": str(exp_cfg.output_dir), "error": str(e)})
            if len(failed) > max_failures:
                _write_failed(sweep_dir, failed)
                raise RuntimeError(
                    f"失败次数超过 {max_failures}，中止 sweep"
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
    parser.add_argument("--repeats", type=int, default=10,
                        help="每个配置重复次数（默认 10）")
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
        repeats=args.repeats,
    )
    sweep_dir = run_sweep(cfg)
    print(f"\nSweep finished. Results in: {sweep_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

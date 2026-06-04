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

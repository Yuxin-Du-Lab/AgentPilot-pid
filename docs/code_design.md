# PID 实验台代码设计方案

> 配套文档：[`experiment_design.md`](./experiment_design.md)（实验流程、参数表、分析输出）
> 实施日期基线：2026-06-04

## 1. 目标与范围

把原 agent (`Pilot-FractFlow-main/tools/aircraft/flight_brain/pid_control.py`) 中的 PID 控制器抽出来，做成一个**独立的、可重复运行的 PID 超参数敏感性实验台**。要求：

- 不依赖原 agent 的任何 Python 模块（不 import `aircraft.*`）
- 真连 MSFS 2024，通过原有的 HTTP API 控制
- 反馈量从 MSFS 直接读 heading 和 altitude（不用视觉跟踪）
- 控制目标由测试脚本批量给出（替代原系统的固定 sliding region）
- 单次实验是单轴 step response；批量执行 OFAT 网格

非目标（YAGNI）：

- 视觉跟踪（bbox tracking）
- VLM 安全检查
- MCP server 包装
- 多轴耦合实验
- 实时 dashboard
- 断点续跑

## 2. 架构总览

```
            ┌──────────────────────────────────────────────────────┐
            │                  sweep.py（驱动）                    │
            │  生成 OFAT (kp,ki,kd) 列表 → 循环调 experiment.run() │
            │  每次 run 前调 flight_io.reset_to()                  │
            │  run 完成后 metrics.compute() → 写文件                │
            │  全部跑完 analysis.build_outputs()                   │
            └────────────┬─────────────────────────────────────────┘
                         │
            ┌────────────▼─────────────┐
            │     experiment.run()     │   一次完整的单轴 step
            │  - reset                 │
            │  - loop:                 │       ┌─────────────────────┐
            │      read_state ─────────┼──────►│   flight_io.py      │
            │      pid.update          │       │  - read_state()     │
            │      issue move_xxx ─────┼──────►│  - move_xxx(t)      │
            │      log row             │       │  - reset_to(...)    │
            │  - until t > duration    │       │  (HTTP wrapper)     │
            │  - 写 log.csv            │       └─────────┬───────────┘
            └────────┬─────────────────┘                 │
                     │ 用                                 │ HTTP
            ┌────────▼─────────┐                  ┌──────▼──────┐
            │ pid_controller.py│                  │  MSFS 2024  │
            │ 1D PID + wrap    │                  │  (real sim) │
            └──────────────────┘                  └─────────────┘
```

## 3. 模块清单

| 文件 | 职责 |
|---|---|
| `pid_exp/flight_io.py` | 唯一接触 MSFS HTTP API 的模块。封装 `read_state()`、`move_xxx(t)`、`reset_to()`、`set_flight_parameter()` |
| `pid_exp/pid_controller.py` | 1D PID 类，正确处理 heading wrap-around |
| `pid_exp/experiment.py` | 单次实验：reset → 循环（read/pid/act/log）→ 写 `log.csv` + `config.json` |
| `pid_exp/metrics.py` | 从 `log.csv` 计算 8 个标量 metric，写 `metrics.json` |
| `pid_exp/sweep.py` | OFAT 扫参驱动：根据 `experiment_design.md` 的测试列表批量调 `experiment.run` |
| `pid_exp/analysis.py` | 跑完后聚合所有 `metrics.json`，生成 `sensitivity_table.md` + `summary.csv` + 图 |
| `pid_exp/.env` | 配置 `API_URL_CTRL` / `API_URL_GET` |
| `pid_exp/__init__.py` | 包标识（空） |

## 4. 模块详细规范

### 4.1 `flight_io.py`

#### 4.1.1 状态读取

```python
def read_state() -> dict:
    """
    Returns:
        {
            "heading": float,    # PLANE_HEADING_DEGREES_MAGNETIC, 度, [0, 360)
            "altitude": float,   # PLANE_ALTITUDE, 英尺
            "raw": dict,         # 原始 API 返回的全字段 dict
            "timestamp": float,  # time.time() 抓的本地时间戳
        }
    """
```

实现方式：GET `API_URL_GET`，返回 JSON 是 simvar 列表（每项 `{name, val, unit, writable}`），转成 dict 后取需要的字段。读取失败重试 2 次，仍失败抛 `FlightIOError`。

> **单位陷阱（heading 是弧度）**：simvar `PLANE_HEADING_DEGREES_MAGNETIC` 名字带 "DEGREES"，但 bridge/SimConnect 实际收发的是**弧度**（官方文档明确："although the name mentions degrees the units used are radians"）。所以 `read_state` 内部用 `_heading_rad_to_deg` 把读到的弧度转成度并归一化到 [0,360)，写回（见 4.1.3 `reset_to`）用 `_heading_deg_to_rad` 把度转成弧度。转换**只在 `flight_io` 这一层做**，对外（PID、experiment、metrics）heading 一律是"度"。`PLANE_ALTITUDE` 是英尺，无需转换。

#### 4.1.2 控制指令（直接拷贝原 `flight_operations.py`）

下列 7 个函数的函数体直接拷贝自原 `Pilot-FractFlow-main/tools/aircraft/msfs2024tools/flight_operations.py`，**仅做以下三点修改**：

1. 删除 `@mcp.tool()` 装饰器及 `from mcp.server.fastmcp import FastMCP` 引用
2. 把所有 `my_logger.info(...)` 替换为 `logger.info(...)`，其中 `logger = logging.getLogger("flight_io")`
3. 不导入 `aircraft.utils.get_gps.get_is_on_ground`（删除 `move_descend` 末尾的着陆检查代码块）

```python
def set_flight_parameter(name: str, val) -> dict
def move_forward(t: float) -> None       # throttle=99 持续 t 秒, 后置 hover
def move_backward(t: float) -> None      # throttle=0
def move_ascend(t: float) -> None        # ELEVATOR_POSITION=1.0
def move_descend(t: float) -> None       # ELEVATOR_POSITION=-1.0, t 截断到 2.0s 上限
def hover() -> None                       # throttle=50, 所有舵面归零
def hover_turn_left(t: float) -> None    # RUDDER_POSITION=-0.05
def hover_turn_right(t: float) -> None   # RUDDER_POSITION=0.05
```

不拷贝：`move_left`、`move_right`、`move_forward_and_descend`（PID 实验用不到）。

`set_flight_parameter` 内部 retry 2 次，仍失败抛 `FlightIOError`。

#### 4.1.3 复位

```python
class ResetVerificationError(Exception): ...

def reset_to(heading: float, altitude: float, settle_seconds: float = 5.0) -> None:
    """
    Teleport-set heading 和 altitude，调 hover 让飞机进入稳态，等 settle_seconds，
    然后读一次状态做验证（容差 heading±5°, altitude±50ft）。验证失败抛 ResetVerificationError。
    """
    set_flight_parameter("PLANE_HEADING_DEGREES_MAGNETIC", heading)
    set_flight_parameter("PLANE_ALTITUDE", altitude)
    hover()                          # throttle=50, 舵面归零
    time.sleep(settle_seconds)
    state = read_state()
    if abs(_heading_diff(state["heading"], heading)) > 5:
        raise ResetVerificationError(...)
    if abs(state["altitude"] - altitude) > 50:
        raise ResetVerificationError(...)
```

`_heading_diff(a, b)` 返回归一化到 [-180, 180] 的差值（处理 wrap-around）。

### 4.2 `pid_controller.py`

```python
class PIDController:
    def __init__(self, kp: float, ki: float, kd: float, axis: str):
        """
        axis ∈ {"heading", "altitude"}
        heading 时 update() 内部用 wrap-around 误差归一化
        """
    def update(self, current: float, target: float, dt: float) -> dict:
        """
        Returns:
            {
                "output": float,      # PID 原始输出（饱和前），单位即误差单位
                "error": float,        # 归一化后的误差（heading 已 wrap）
                "derivative": float,   # (error - prev_error) / dt
                "integral": float,     # 累计积分值（即 ∑error·dt）
                "p_term": float,       # kp × error
                "i_term": float,       # ki × integral
                "d_term": float,       # kd × derivative
            }
        output == p_term + i_term + d_term
        内部状态：integral, prev_error 在每次 update 后更新
        """
    def reset(self) -> None:
        """清零 integral 和 prev_error"""

def normalize_heading_error(error: float) -> float:
    """折叠到 [-180, 180]: 350 → -10, -350 → +10"""
    return ((error + 180) % 360) - 180
```

vs 原 `PIDController` 类的差异：

- 改成 1D（不再是奇怪的 `(x, 0)` / `(0, y)` 二维写法）
- 加 `axis` 参数，heading 轴自动用 wrap-around
- `dt` 由调用方传入（来自 experiment.py 的真实 loop 间隔），不再内部 `time.time()` 算

### 4.3 `experiment.py`

#### 4.3.1 配置

```python
@dataclass
class ExperimentConfig:
    axis: str                      # "heading" or "altitude"
    kp: float
    ki: float
    kd: float
    initial: float                 # 实验起始值（reset 到这里）
    target: float                  # step 目标
    other_axis_value: float        # 另一轴的固定复位值
    duration_s: float = 45.0
    output_dir: Path
    run_label: str                 # 例如 "P+10%" 用于人类阅读
```

#### 4.3.2 主流程

```python
def run(cfg: ExperimentConfig) -> None:
    # 1. reset
    if cfg.axis == "heading":
        flight_io.reset_to(heading=cfg.initial, altitude=cfg.other_axis_value)
    else:
        flight_io.reset_to(heading=cfg.other_axis_value, altitude=cfg.initial)

    # 2. 初始化 PID
    pid = PIDController(cfg.kp, cfg.ki, cfg.kd, axis=cfg.axis)

    # 3. 主循环
    rows = []
    t_start = time.time()
    t_prev = t_start
    while time.time() - t_start < cfg.duration_s:
        state = flight_io.read_state()
        current = state[cfg.axis]
        t_now = state["timestamp"]
        dt = t_now - t_prev
        t_prev = t_now

        pid_result = pid.update(current, cfg.target, dt)
        # pid_result 是 dict: {output, error, derivative, integral, p_term, i_term, d_term}

        # PID 输出 → 动作时长（保留原映射）
        action_time = max(min(abs(pid_result["output"]) * 0.5, 1.0), 0.1)
        saturated = action_time >= 1.0 - 1e-9

        cmd_name, cmd_fn = _pick_action(cfg.axis, pid_result["output"])
        cmd_fn(action_time)   # 阻塞 action_time + 1s

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
```

#### 4.3.3 动作映射

| axis | control_raw 符号 | 调用 |
|---|---|---|
| heading | > 0（target 在右） | `hover_turn_right(t)` |
| heading | < 0 | `hover_turn_left(t)` |
| altitude | > 0（要爬升） | `move_ascend(t)` |
| altitude | < 0 | `move_descend(t)` |
| any | == 0（罕见） | 跳过该 tick（只 sleep 0.1s 等下次） |

#### 4.3.4 不做的事

- 不做 deadband：即使误差很小也照常发动作（最小 0.1s）。让 chattering 真实地反映在 `n_reversals` metric 里。
- 不做早停：固定时长 45s 跑满。不收敛的 run 就让 `converged=False`。
- 不做 anti-windup（保留原代码朴素积分）。

### 4.4 `metrics.py`

#### 4.4.1 输入输出

输入：单个 run 目录下的 `log.csv` + `config.json`
输出：同目录下的 `metrics.json`

#### 4.4.2 标量 Metric 列表

| Metric | 计算方式 | 单位 |
|---|---|---|
| `rise_time_10_90` | error 从 90% step → 10% step 用的时间。永不收敛则 NaN | 秒 |
| `peak_time` | abs(error) 第一次达到极小值之前若 current 超越 target，则取超越后第一个极值点时间；不超越则等于 rise_time | 秒 |
| `overshoot_pct` | (peak_value - target) / step_size × 100；不超越则 0 | 百分比 |
| `settling_time_5pct` | 误差进入 ±5% step_size 后**剩余时间内**不再出去的最早时刻；永不则 NaN | 秒 |
| `steady_state_error` | 末段（最后 5 秒）平均 \|error\| | 误差单位 |
| `iae` | ∑\|error\| × loop_dt（梯形积分） | 误差·秒 |

step_size = abs(target - initial)。

#### 4.4.3 接口

```python
def compute(run_dir: Path) -> dict:
    """读 log.csv + config.json, 返回 metrics dict, 同时写 metrics.json"""

def aggregate(sweep_dir: Path) -> Path:
    """两级聚合 runs/<config>/r<NN>/。写 summary.csv（每配置 mean/std/n）
    + summary_runs.csv（每 rep 原始值）。返回 summary.csv 路径。"""
```

每个配置重复 N 次（见 4.5.1 `repeats`），`aggregate` 做**两级聚合**：外层遍历 `runs/<config>/`，内层读其下所有 `r<NN>/metrics.json`，对每个 metric 跨 rep 求 mean/std/n。

- `summary.csv`：**每配置一行**（13 行）。字段 `run_id, run_label, axis, kp, ki, kd` + 每个 metric 三列 `{metric}_mean, {metric}_std, {metric}_n`。
- `summary_runs.csv`：**每 rep 一行**（N×13 行，全量原始值，便于溯源）。字段 `run_id, rep, run_label, axis, kp, ki, kd, [6 个原始 metric 列]`。
- std 约定：过滤掉 None/NaN 的 rep；`n` = 有效 rep 数；`n≥2` 用样本标准差（ddof=1），`n==1` 记 0.0，`n==0` 记 NaN。

### 4.5 `sweep.py`

#### 4.5.1 配置

```python
@dataclass
class SweepConfig:
    axis: str                            # "heading" or "altitude"
    nominal_kp: float                    # 默认见 experiment_design.md
    nominal_ki: float
    nominal_kd: float
    initial: float                       # 默认 90 (heading) 或 5000 (altitude)
    target: float                        # 默认 120 (heading) 或 5200 (altitude)
    other_axis_value: float              # 默认 5000 (测 heading 时) 或 90 (测 altitude 时)
    duration_s: float = 45.0
    output_root: Path
    sweep_label: str | None = None       # 自动用时间戳生成
    repeats: int = 10                    # 每个配置重复次数
```

#### 4.5.2 OFAT 列表生成

按照 `experiment_design.md` 第 5 节的 13 行 OFAT 表，由代码静态生成：

```python
def build_ofat_list(cfg: SweepConfig) -> list[ExperimentConfig]:
    p, i, d = cfg.nominal_kp, cfg.nominal_ki, cfg.nominal_kd
    deltas = [(-0.20, "P-20%"), (-0.10, "P-10%"), (+0.10, "P+10%"), (+0.20, "P+20%")]
    runs = [(p, i, d, "Nominal")]
    runs += [(p*(1+δ), i, d, lbl) for δ, lbl in deltas]
    runs += [(p, i*(1+δ), d, lbl.replace("P","I")) for δ, lbl in deltas]
    runs += [(p, i, d*(1+δ), lbl.replace("P","D")) for δ, lbl in deltas]
    return [_make_exp_cfg(cfg, kp, ki, kd, label, idx) for idx, (kp,ki,kd,label) in enumerate(runs, 1)]
```

每个配置展开为 `cfg.repeats` 个 `ExperimentConfig`，**分组排列**（同一配置的 N 个 rep 连续排在一起），输出目录嵌套到 rep 一层：`runs/{idx:02d}_{safe_lbl}/r{rep:02d}/`。所以一轴共 `13 × repeats` 个 run（默认 130）。`experiment.run` 写文件前已 `mkdir(parents=True)`，rep 叶子目录自动创建，无需额外处理。

#### 4.5.3 主循环

```python
def run_sweep(cfg: SweepConfig) -> Path:
    sweep_dir = _make_sweep_dir(cfg)            # results/2026-06-04_15-30_heading_OFAT/
    _save_sweep_config(cfg, sweep_dir)

    exp_cfgs = build_ofat_list(cfg)
    # 失败阈值随总 run 数放大：max(3, len(exp_cfgs)//10)，130-run sweep 容忍零散瞬时失败
    max_failures = max(3, len(exp_cfgs) // 10)
    failed = []
    for exp_cfg in exp_cfgs:
        try:
            experiment.run(exp_cfg)
            metrics.compute(exp_cfg.output_dir)
        except Exception as e:
            logger.error(f"run {exp_cfg.run_label} 失败: {e}")
            failed.append({"run_label": exp_cfg.run_label,
                           "output_dir": str(exp_cfg.output_dir), "error": str(e)})
            if len(failed) > max_failures:
                raise RuntimeError(f"失败次数超过 {max_failures}，中止 sweep")
        flight_io.hover()                       # 防止前一个 run 末态污染

    metrics.aggregate(sweep_dir)
    analysis.build_outputs(sweep_dir)
    return sweep_dir
```

#### 4.5.4 命令行

```bash
python -m pid_exp.sweep --axis heading                        # 用默认 nominal，repeats=10
python -m pid_exp.sweep --axis altitude
python -m pid_exp.sweep --axis heading --kp 2.0 --ki 0.1 --kd 0.5  # 显式指定
python -m pid_exp.sweep --axis heading --repeats 2 --duration 5    # 快速冒烟（少 rep 短时长）
```

`--repeats N`（默认 10）控制每个配置重复次数。

### 4.6 `analysis.py`

#### 4.6.1 输入输出

输入：单个 sweep_dir 下的 `summary.csv`
输出：同目录下：
- `sensitivity_table.md` — 数字呈现的敏感性表（见 `experiment_design.md` 第 6 节）
- `step_response_overlay.png` — 叠加曲线图（每参数一张子图，3 张子图横排成 1 张）
- `sensitivity_plots/` — 每个 metric × 每个参数 的 5 点散点 + 拟合曲线（共 12 子图，4×3 网格成 1 张图）

#### 4.6.2 接口

```python
def build_outputs(sweep_dir: Path) -> None:
    summary = pd.read_csv(sweep_dir / "summary.csv")
    _write_sensitivity_table(summary, sweep_dir / "sensitivity_table.md")
    _plot_step_response_overlay(sweep_dir, sweep_dir / "step_response_overlay.png")
    _plot_sensitivity_grid(summary, sweep_dir / "sensitivity_plots.png")
```

#### 4.6.3 OFAT 敏感性数值计算

对每个 (metric, 参数 ∈ {Kp, Ki, Kd}) 组合：

- 提取 5 个点：`pct ∈ {-0.20, -0.10, 0.00, +0.10, +0.20}` → 5 个**每档均值** `{metric}_mean`（同时取出 `{metric}_std` 供画误差棒）
- 用 `numpy.polyfit(pct, mean_values, deg=1)` 拟合一阶直线：`metric = slope × pct + intercept`（拟合用的是均值，不是单 run 原始值）
- **`slope` 的单位是"metric 单位 per 100% 参数变化"**，因为 pct 用 fraction 形式（0.10 = 10%），slope 系数对应 pct=1.0 时（即 +100%）的 metric 增量
- `r2`：用 `1 - SS_res / SS_tot` 算决定系数（5 个均值点对线性拟合的吻合度，单调线性 R² → 1，U/Λ 形 R² 偏小）
- `range = max(mean_values) - min(mean_values)`：5 个均值点实际跨度
- 若该 metric 在 5 个点中均值全为 NaN（如永不收敛 run 的 settling_time），slope/r2/range 全部输出 NaN
- 详见 `experiment_design.md` 第 9.1 节的输出表定义

可视化对 rep 间散布的呈现：

- `sensitivity_plots.png`：散点改用 `ax.errorbar(...)`，纵向误差棒 = 该档 `{metric}_std`
- `step_response_overlay.png`：每档把所有 rep 的 `current(t)` 重采样到公共时间网格，画**均值曲线 + ±std 阴影带**（不再是单条原始曲线）
- `sensitivity_table.md`：Nominal 列显示 `mean (±std)`

## 5. 配置（`.env`）

```dotenv
API_URL_CTRL=http://<msfs-bridge-host>:5000/set
API_URL_GET=http://<msfs-bridge-host>:5000/get
```

复用原 agent 的桥接 server，URL 由用户填。

## 6. 目录布局

```
D:/work/pilot/aircraft_agent/pid_exp/
├── docs/
│   ├── code_design.md             # 本文件
│   └── experiment_design.md
├── pid_exp/
│   ├── __init__.py
│   ├── flight_io.py
│   ├── pid_controller.py
│   ├── experiment.py
│   ├── metrics.py
│   ├── sweep.py
│   └── analysis.py
├── results/
│   └── 2026-06-04_15-30_heading_OFAT/
│       ├── sweep_config.json
│       ├── runs/
│       │   ├── 01_Nominal/           # 每配置一个目录，下含 repeats 个 rep
│       │   │   ├── r01/
│       │   │   │   ├── config.json
│       │   │   │   ├── log.csv
│       │   │   │   └── metrics.json
│       │   │   ├── r02/
│       │   │   └── ...               # 默认 r01..r10
│       │   ├── 02_Pn20pct/
│       │   └── ...
│       ├── summary.csv               # 每配置一行（mean/std/n）
│       ├── summary_runs.csv          # 每 rep 一行（原始值）
│       ├── sensitivity_table.md
│       ├── step_response_overlay.png
│       └── sensitivity_plots.png
├── .env
├── .env.example
├── requirements.txt
├── README.md
└── 超参数设定.jpg
```

## 7. 依赖

`requirements.txt`：

```
requests>=2.31
python-dotenv>=1.0
numpy>=1.26
pandas>=2.0
matplotlib>=3.8
```

不需要 statsmodels / SALib（OFAT 用纯 numpy 拟合即可）。
不需要 scipy（除非后续要做更严格的统计检验）。

## 8. 错误处理策略

| 情况 | 层级 | 处理 |
|---|---|---|
| HTTP 单次调用超时/失败 | `flight_io` | 内部 retry 2 次，仍失败抛 `FlightIOError` |
| 复位后状态超出容差 | `flight_io.reset_to` | 抛 `ResetVerificationError` |
| 单个 run 任意异常 | `sweep.run_sweep` | catch + 记录到 `failed.json`，跳过；累计 > `max(3, 总run数//10)` 次失败则中止 sweep |
| metric 无法计算（如永不收敛） | `metrics.compute` | 该 metric 写为 NaN，`converged=False` |
| 用户 Ctrl+C | `sweep.run_sweep` | 已完成的 run 保留；不做断点续跑（YAGNI） |

## 9. 与原 agent 系统的差异（被砍掉的依赖）

| 原依赖 | 在原系统的作用 | 在实验台中的处理 |
|---|---|---|
| `mcp.server.fastmcp.FastMCP` | 把 PID / flight ops 暴露为 MCP tool | 砍掉，命令行直接调 |
| `aircraft.utils.self_logging.get_my_logger` | 写带 GPS 注入的结构化 log，依赖 `./tmp/log_path.txt` | 砍掉，用 stdlib `logging` |
| `aircraft.utils.logger_config.setup_logger` | 同上的 stdout logger | 砍掉，用 stdlib `logging` |
| `aircraft.utils.get_gps.get_is_on_ground` | `move_descend` 末尾的着陆检查 | 砍掉，实验时飞机一直在空中 |
| `aircraft.safety_tools.video_depth_estimation.stop_PID_control` | 视觉深度突变检测，触发 PID 退出 | 砍掉，实验场景不需要 |
| `aircraft.safety_tools.mode_switch_vlm.stop_PID_control_vlm` | Qwen-VL 安全检查 | 砍掉，实验场景不需要 |
| `./tmp/tracked_view_bbox.json` | 视觉跟踪器写入的 bbox | 砍掉，反馈量改为 heading + altitude |
| `./tmp/control_signal.txt` | PID 写"PID has control"标记给其他模块 | 砍掉，实验台没有其他模块 |

## 10. 测试与验证

| 阶段 | 验证方式 |
|---|---|
| 模块单测 | `pid_controller` 的 wrap-around、积分、微分各写 1 个 unit test（pytest） |
| 集成冒烟 | 跑一个 1 分钟的 Nominal run，看 `log.csv` 是否有数据，`metrics.json` 是否计算成功 |
| 全 sweep 验证 | 先 `--repeats 2 --duration 5` 冒烟（runs/01_Nominal/r01,r02 齐备、summary.csv 13 行带 _mean/_std/_n、summary_runs.csv 26 行），再跑完整 heading sweep（默认 130 run），检查所有产出文件齐备且能打开 |
| 数值合理性 | Nominal run 的 overshoot、settling_time 应该和原 agent 现场观察一致（用户主观判断） |

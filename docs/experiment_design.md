# PID 实验台实验设计方案

> 配套文档：[`code_design.md`](./code_design.md)（架构、模块 API、错误处理）
> 实施日期基线：2026-06-04

## 1. 实验目标

定量评估 PID 三个超参数（$K_p, K_i, K_d$）对该飞行器系统**单轴 step response** 控制质量的影响，最终产出一张直观的敏感性表，展示每个参数对每个性能指标的影响方向、强度与线性度。

## 2. 方法论：OFAT（One-Factor-At-a-Time）

每次实验只让一个参数偏离 nominal，其他保持不变。共 13 个 run / 轴：

```
1   Nominal               (Kp=Kp⁰, Ki=Ki⁰, Kd=Kd⁰)
2-5 Kp 扫描:  ±10%, ±20%  (Ki=Ki⁰, Kd=Kd⁰ 固定)
6-9 Ki 扫描:  ±10%, ±20%  (Kp=Kp⁰, Kd=Kd⁰ 固定)
10-13 Kd 扫描: ±10%, ±20% (Kp=Kp⁰, Ki=Ki⁰ 固定)
```

**为什么选 OFAT 而不是全因子网格（如 3×3×3 / 5×5×5）：**

| 维度 | OFAT (13 run) | 全因子 3×3×3 (27) | 全因子 5×5×5 (125) |
|---|---|---|---|
| 单轴时长（45s/run + 5s reset） | ~11 min | ~23 min | ~108 min |
| 能识别主效应 | 是 | 是 | 是 |
| 能识别交互项 | 否 | 是（粗） | 是（细） |
| 输出可读性 | 高（每参数 5 点单调） | 中 | 低 |

本研究目标是**直观展示每个超参数的独立影响**，不研究交互项，OFAT 是最合理的选择。

## 3. 默认参数（来自原 agent）

来源：`Pilot-FractFlow-main/tools/aircraft/flight_brain/pid_control.py:149-150`

| 轴 | $K_p^0$ | $K_i^0$ | $K_d^0$ |
|---|---|---|---|
| **heading** | 2.0 | 0.10 | 0.5 |
| **altitude** | 1.5 | 0.05 | 0.5 |

> 原代码中 `pid_x` 控制 image-x 轴（通过 `hover_turn_left/right`），对应实验台的 heading；`pid_y` 控制 image-y 轴（通过 `move_ascend/descend`），对应 altitude。这里**沿用原默认值**，不重新调参。

## 4. 单次实验设置

### 4.1 step response 参数

| 项 | heading 实验 | altitude 实验 |
|---|---|---|
| 复位 heading | **90°** | 90°（固定） |
| 复位 altitude | 5000 ft（固定） | **5000 ft** |
| step 目标 heading | **120°**（+30° 阶跃） | 90°（不变） |
| step 目标 altitude | 5000 ft（不变） | **5200 ft**（+200 ft 阶跃） |
| 实验时长 | 45 s | 45 s |
| 复位 settle | 5 s | 5 s |

**为什么 heading step = 30°（不是更大）：** 用户确认真实场景中 heading 修正幅度 ≤45°，30° 在工作包线内，又足够大让 PID 有响应空间。

**为什么 altitude step = 200 ft：** 真实场景中小幅高度修正的代表性数值。

### 4.2 控制循环节拍

每次循环 = `read_state` (HTTP, ~50–100 ms) + `pid.update` (~ms 级) + `move_xxx` 阻塞 (`action_time` + 1s 后置 hover) ≈ **1.1–2.1 s/iter**。

45s 实验约 22–40 个迭代。足够画响应曲线。

### 4.3 PID 输出 → 动作时间映射

保留原 `pid_control.py` 的映射（不是研究对象，固定值）：

```
action_time = clip(abs(control_raw) * 0.5,  min=0.1,  max=1.0)   # 单位：秒
```

`saturated = (action_time >= 1.0)`，记录为日志列。

### 4.4 不做的事

- 不做 deadband
- 不做早停
- 不做 anti-windup

理由见 `code_design.md` 4.3.4。

## 5. OFAT 测试列表

### 5.1 Heading 轴（13 run，nominal: $K_p^0=2.0, K_i^0=0.10, K_d^0=0.50$）

| 序号 | 实验组 | $K_p$ | $K_i$ | $K_d$ |
|---|---|---|---|---|
| 01 | Nominal | 2.00 | 0.100 | 0.500 |
| 02 | P-20% | **1.60** | 0.100 | 0.500 |
| 03 | P-10% | **1.80** | 0.100 | 0.500 |
| 04 | P+10% | **2.20** | 0.100 | 0.500 |
| 05 | P+20% | **2.40** | 0.100 | 0.500 |
| 06 | I-20% | 2.00 | **0.080** | 0.500 |
| 07 | I-10% | 2.00 | **0.090** | 0.500 |
| 08 | I+10% | 2.00 | **0.110** | 0.500 |
| 09 | I+20% | 2.00 | **0.120** | 0.500 |
| 10 | D-20% | 2.00 | 0.100 | **0.400** |
| 11 | D-10% | 2.00 | 0.100 | **0.450** |
| 12 | D+10% | 2.00 | 0.100 | **0.550** |
| 13 | D+20% | 2.00 | 0.100 | **0.600** |

### 5.2 Altitude 轴（13 run，nominal: $K_p^0=1.5, K_i^0=0.05, K_d^0=0.50$）

| 序号 | 实验组 | $K_p$ | $K_i$ | $K_d$ |
|---|---|---|---|---|
| 01 | Nominal | 1.50 | 0.050 | 0.500 |
| 02 | P-20% | **1.20** | 0.050 | 0.500 |
| 03 | P-10% | **1.35** | 0.050 | 0.500 |
| 04 | P+10% | **1.65** | 0.050 | 0.500 |
| 05 | P+20% | **1.80** | 0.050 | 0.500 |
| 06 | I-20% | 1.50 | **0.040** | 0.500 |
| 07 | I-10% | 1.50 | **0.045** | 0.500 |
| 08 | I+10% | 1.50 | **0.055** | 0.500 |
| 09 | I+20% | 1.50 | **0.060** | 0.500 |
| 10 | D-20% | 1.50 | 0.050 | **0.400** |
| 11 | D-10% | 1.50 | 0.050 | **0.450** |
| 12 | D+10% | 1.50 | 0.050 | **0.550** |
| 13 | D+20% | 1.50 | 0.050 | **0.600** |

## 6. 复位机制

每个 run 之间必须把飞机回到一致的初始状态，否则 metric 不可比。

```
1. set_flight_parameter("PLANE_HEADING_DEGREES_MAGNETIC", deg→rad(initial_heading))  # heading 写回需转弧度
2. set_flight_parameter("PLANE_ALTITUDE", initial_altitude)
3. hover()                               # throttle=50, 舵面归零
4. sleep(5 s)                            # 让 MSFS 物理稳态
5. read_state() → 验证：heading 容差 ±5°, altitude 容差 ±50 ft（read_state 已把读到的弧度转回度）
   超出容差则抛 ResetVerificationError，该 run 标记失败
```

> **前提**：用户已确认 `PLANE_HEADING_DEGREES_MAGNETIC` 和 `PLANE_ALTITUDE` 通过现有 HTTP `set_flight_parameter` 接口可写。
>
> **单位陷阱**：`PLANE_HEADING_DEGREES_MAGNETIC` 名字带 "DEGREES" 但实际收发**弧度**（官方文档）。复位写入和验证读取的弧度↔度转换都封装在 `flight_io` 里，本表的 initial_heading / 容差仍按"度"理解。详见 `code_design.md` 4.1.1。

## 7. 时间预算

| 任务 | 时长 |
|---|---|
| 单 run 实验 | 45 s |
| 单 run 复位 | ~5 s |
| 单 run 余量 | ~2 s |
| **单 run 合计** | **~52 s** |
| 单轴 13 run | **~11 min** |
| 双轴 26 run | **~22 min** |

实际跑完总计约 **22 分钟**（连续无失败），适合一次坐下来跑完。

## 8. 标量 Metric 定义

每个 run 跑完后由 `metrics.py` 从 `log.csv` 计算，写入 `metrics.json`。共 6 个 metric，覆盖时间响应、精度、整体跟踪质量三类。

| Metric | 定义 | 单位 | 主要回答的问题 | 主要 PID 影响 |
|---|---|---|---|---|
| `rise_time_10_90` | error 从 90% step → 10% step 用的时间。永不收敛 = NaN | s | 多快接近目标？ | $K_p$（强） |
| `peak_time` | current 第一次达到极值的时间；不超越 = rise_time | s | 何时冲到最高？ | $K_p$ |
| `overshoot_pct` | (peak_value - target) / step_size × 100；不超越 = 0 | % | 冲过头多少？ | $K_p$↑, $K_d$↓ |
| `settling_time_5pct` | 误差进入 ±5% step_size 后**剩余时间内**不再出去的最早时刻；永不 = NaN | s | 多久彻底稳定？ | $K_d$（强） |
| `steady_state_error` | 末段（最后 5 秒）平均 \|error\| | 误差单位 | 长期距离目标多远？ | $K_i$（强） |
| `iae` | ∑\|error\| × loop_dt（梯形积分） | 误差·s | 总体差多少？（均衡） | 综合，U 型 |

step_size = abs(target - initial)。误差单位：heading 度，altitude 英尺。

### 8.1 各 metric 详细说明

#### `rise_time_10_90`（接近速度）
- 例：heading 30° 阶跃 → 90% 点 = error 27°（current ≈ 93°）→ 10% 点 = error 3°（current ≈ 117°）
- 值越小响应越快
- **本系统下限**：bang-bang + post-hover-1s 让飞机不能"瞬间转弯"，即使 $K_p$ 很大 rise_time 也有物理下限

#### `peak_time`（极值时刻）
- 如果不超调，peak_time ≈ rise_time（这种情况下该 metric 无独立信息）
- 主要在有 overshoot 时才有独立含义

#### `overshoot_pct`（超调）
- 教科书强相关：$K_p$ ↑ → overshoot ↑↑↑；$K_d$ ↑ → overshoot ↓↓
- **本系统特殊性**：post-hover-1s 是天然阻尼，所以原系统 overshoot 通常不严重，OFAT 表中数值可能整体偏小

#### `settling_time_5pct`（彻底稳定）
- 必须"进入 ±5% 带后不再出去"，只要后面又超出，settle 重新计算
- **本系统特殊性**：稳态附近 0.1s 最小动作时长导致 chattering 残留，如果幅度超过 ±5% 带，settling_time = NaN

#### `steady_state_error`（稳态误差）
- 教科书核心：$K_i$ ↑ → steady_state_error ↓↓↓
- **本系统特殊性**：受 chattering 影响，即使 Ki 大到理论上能归零，最小动作 0.1s 会让飞机来回小幅摆动，平均 \|error\| 不会真为 0，存在一个"floor"

#### `iae`（综合质量分数）
- $\int_0^T |e(t)|\, dt$ 把"快慢、超调、稳态"全融成一个数
- 通常 PID 参数对 IAE 是 **U 型**（不是单调）——参数过小过大都让 IAE 变大，中间有 sweet spot
- 经典 PID 调参的目标函数

## 9. 分析输出

### 9.1 `sensitivity_table.md`（**核心交付物**）

数字呈现，每个 (metric, 参数) 给出 **slope / range / R²** 三个数：

- **slope**：用 5 个 OFAT 点 (-20, -10, 0, +10, +20%) 一阶 polyfit 得到的斜率，单位 = `<metric 单位> / 100% 参数变化`。展示 metric 对该参数的"灵敏度强度 + 方向"
- **range**：5 个点中 metric 的 max - min，反映**实际经历的变化幅度**（即使非线性也有意义）
- **R²**：5 点拟合的决定系数。R² > 0.95 → 单调线性；R² 接近 0 → 非线性（如 U 型）

文件示例（具体数字以实跑结果填充）：

```markdown
# Heading 轴 PID 超参数敏感性

Nominal: Kp=2.00, Ki=0.10, Kd=0.50
Step: 90° → 120°（30° 阶跃）, duration 45s
Sweep dir: results/2026-06-04_15-30_heading_OFAT

| Metric                  | Nominal |       Kp 影响        |       Ki 影响        |       Kd 影响        |
|                         |  value  | slope / range / R²  | slope / range / R²  | slope / range / R²  |
|-------------------------|---------|----------------------|----------------------|----------------------|
| rise_time_10_90 (s)     |  18.50  | -8.32 / 7.04 / 0.97 | -0.55 / 0.41 / 0.86 | -3.40 / 2.78 / 0.95 |
| peak_time (s)           |  21.30  | -7.10 / 5.80 / 0.94 | -0.40 / 0.30 / 0.81 | -2.20 / 1.85 / 0.92 |
| overshoot_pct (%)       |  12.30  |+10.50 / 8.85 / 0.93 |+0.95 / 0.78 / 0.88  | -3.85 / 3.20 / 0.91 |
| settling_time_5pct (s)  |  28.00  | +4.20 / 3.85 / 0.41 | -2.80 / 2.15 / 0.78 | -7.05 / 5.90 / 0.94 |
| steady_state_error (°)  |   0.80  | +0.05 / 0.04 / 0.62 | -0.62 / 0.51 / 0.96 | +0.02 / 0.02 / 0.45 |
| iae (°·s)               |  145.0  | -3.20 / 11.50 / 0.32| -8.15 / 6.80 / 0.91 | -16.50 /13.00 / 0.93|

阅读说明：
- slope 单位：metric 单位 per 100% 参数变化（即 P+100% 与 P-100% 外推差值）
  例：rise_time 在 Kp 列 slope=-8.32 表示 Kp 增大 100%，rise time 大致减少 8.32 s
- R² 接近 1 表示线性单调；R² 小说明 5 点是非线性关系（看 sensitivity_plots 详查）
- range 是 5 点 max-min 的实际跨度，比 slope 更直接看"在 ±20% 范围内 metric 跳了多少"
```

### 9.2 `summary.csv`

26 行（13 heading + 13 altitude），所有 metric 横向铺开。schema：

```
sweep_id, run_id, run_label, axis,
kp, ki, kd,
rise_time_10_90, peak_time, overshoot_pct, settling_time_5pct,
steady_state_error, iae
```

### 9.3 `step_response_overlay.png`

3 张子图横排（每参数 1 张），每张里叠加 5 条曲线（-20%, -10%, Nominal, +10%, +20%）：

```
   ┌────────── Kp 扫描 ──────────┐ ┌────── Ki 扫描 ──────┐ ┌────── Kd 扫描 ──────┐
   │ heading vs t                │ │ heading vs t        │ │ heading vs t        │
   │  ╱──┬──┬──┬──── target=120 │ │   ── target=120     │ │   ── target=120     │
   │ ╱──┴──┴──                  │ │   ┄┄┄┄┄┄            │ │   ╱──╲              │
   │ ╱  Kp+20%(红)               │ │   Ki+20%(红)        │ │   Kd+20%(红)        │
   │   Kp+10%(橙)                │ │   Ki+10%(橙)        │ │   Kd+10%(橙)        │
   │   Nominal(黑)               │ │   Nominal(黑)       │ │   Nominal(黑)       │
   │   Kp-10%(蓝)                │ │   Ki-10%(蓝)        │ │   Kd-10%(蓝)        │
   │   Kp-20%(紫)                │ │   Ki-20%(紫)        │ │   Kd-20%(紫)        │
   └─────────────────────────────┘ └─────────────────────┘ └─────────────────────┘
```

heading 轴 1 张，altitude 轴 1 张，共 2 张图。**这是最直观看"参数变了曲线变成什么样"的可视化。**

### 9.4 `sensitivity_plots.png`

每个 (metric × 参数) 一个小子图，展示 5 点拟合：

- 横轴：参数变化百分比（-20, -10, 0, +10, +20）
- 纵轴：metric 值
- 5 个散点 + 拟合直线 + R² 标注

6 个 metric × 3 参数 = 18 子图，按 metric 行 × 参数列排成 6×3 网格。每轴 1 张，共 2 张图。

### 9.5 `run_index.csv`

13 行映射表，每行：`run_id, run_label, kp, ki, kd, output_dir`，用于人工查阅。

## 10. 验收标准

实验设计成功的标准：

1. ✅ 26 个 run 全部跑完（或失败 ≤3 个）
2. ✅ 每个 run 有完整的 `log.csv` + `metrics.json` + `config.json`
3. ✅ Nominal run 的响应曲线"看起来正常"（先 rise，可能有 overshoot，最后趋稳）
4. ✅ `sensitivity_table.md` 数字合理：
   - 至少 Kp 列在 rise_time / overshoot 上 R² > 0.7（PID 教科书结论：Kp 应该单调影响这两个）
   - steady_state_error 在 Ki 列 R² > 0.7（Ki 教科书作用）
5. ✅ `step_response_overlay.png` 中 Kp 子图能直观看出"Kp 越大上升越快但 overshoot 越大"

## 11. 后续可能扩展（YAGNI 阶段，先不做）

- 多步长扫描（15° / 30° / 45° heading 各跑一次 OFAT）
- 全因子 3×3×3 网格 + ANOVA 交互项分析
- 双轴同时 step 的耦合实验
- 把 PID-output→action_time 映射的 0.5 系数也作为研究对象
- 实时 dashboard（matplotlib 不是动态的，跑的时候只能等）
- 断点续跑

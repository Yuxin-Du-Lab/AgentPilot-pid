# PID 超参数 OFAT 敏感性实验台

独立的 PID 实验环境，连真实 MSFS 2024，跑 OFAT (One-Factor-At-a-Time) sweep 测 PID 三个超参数对 step response 的影响。

详细设计见 `docs/code_design.md`、`docs/experiment_design.md`、`docs/metric_explanation.md`。

## 安装

你需要做的（真机 MSFS 验证）

# 1. 配置 MSFS bridge URL
```bash
cd path/to/pid_exp
py -m pip install -e .
py -m pip install -r requirements.txt
cp .env.example .env
```
# 编辑 .env 填入真实的 API_URL_CTRL 和 API_URL_GET

# 2. 起飞，让飞机稳定在某个高度（建议 5000ft, heading 90°）

# 3. 跑 heading 轴 sweep（约 11 分钟）
py -m pid_exp.sweep --axis heading

# 4. 查看结果
# results/<时间戳>_heading_OFAT/sensitivity_table.md
# results/<时间戳>_heading_OFAT/step_response_overlay.png
# results/<时间戳>_heading_OFAT/sensitivity_plots.png


## 项目结构

```
pid_exp/
├── docs/                       # 设计文档（必读）
│   ├── code_design.md
│   ├── experiment_design.md
│   ├── metric_explanation.md
│   └── plans/
├── pid_exp/                    # 源代码
│   ├── flight_io.py            # MSFS HTTP wrapper
│   ├── pid_controller.py       # 1D PID with heading wrap
│   ├── experiment.py           # 单次 step response 实验
│   ├── metrics.py              # 6 个 metric 计算
│   ├── sweep.py                # OFAT 驱动 + CLI
│   └── analysis.py             # 敏感性表 + 图生成
├── tests/                      # pytest 测试
├── results/                    # 运行时输出（不入 git）
├── .env                        # 配置（不入 git）
├── .env.example
├── requirements.txt
└── pyproject.toml
```

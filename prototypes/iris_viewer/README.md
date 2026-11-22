# Aether Round 7A · Iris 三维观察与回放

这个原生 MuJoCo Viewer 可以观察 PX4 驱动的 Iris 完成起飞、悬停和降落，也可以在 PX4 完全退出后打开 Round 6 的历史实验。实时模式和回放都使用真值 CSV 的实际采样点。界面显示 PX4 ULog 的飞行模式、原生高度设定值，以及当前记录的解锁状态、位置、速度和姿态。

## 启动与操作

在仓库或交付包根目录执行。需要 Round 6 的 PX4 v1.17.0 原生 SITL 构建和 Python 环境。在 Apple Silicon 上安装 MuJoCo **4.0.1** 后使用其官方 `mjpython` 启动原生被动 Viewer，使 macOS 主线程负责渲染。 [MuJoCo 被动 Viewer 文档](https://mujoco.readthedocs.io/en/stable/python.html#passive-viewer) · [PX4 原生构建与 Round 6 复现](https://github.com/XuanheGuo/Aether/blob/4331543c0abfa3063295f3db5e2932cd245e5142/prototypes/iris_closed_loop/README.md)

```sh
uv venv --python 3.12.10 .venv
uv pip install --python .venv/bin/python -r prototypes/iris_closed_loop/requirements.txt
export PX4_SRC=/absolute/path/to/PX4-Autopilot

# 实时闭环。每次选择新的输出目录。
.venv/bin/python -m prototypes.iris_viewer live --px4-src "$PX4_SRC" --output runs/iris-live --instance 7

# 离线回放。参数是完整实验目录，至少含 truth.csv，ULog 提供模式和设定值。
.venv/bin/mjpython -m prototypes.iris_viewer replay captures/r6-flight-a
```

归档随附 `captures/r6-flight-a`。仓库克隆可从 `prototypes/iris_closed_loop/evidence/captures.zip` 解出同一目录。可选 `--start 14 --paused` 会停在回放的 14 秒处。数字键 0–9 等距跳转，9 为末帧。

| 操作 | 按键 |
| --- | --- |
| 旋转、缩放、平移 | Viewer 原生鼠标操作。拖动旋转，滚轮缩放，右键拖动平移 |
| 自由视角或跟随 Iris | **F** |
| 播放或暂停 | **Space**，只用于回放 |
| 回到开头或跳转 1 秒 | **R** 或 **← / →** |
| 调整速度 | **[ / ]**，支持 0.25× 至 4× |
| 打开记录目录和已生成的误差图 | **O** |
| 关闭窗口 | **Q** 或窗口关闭按钮 |

实时命令运行 Round 6 的标准 Takeoff → Hold → Land。关闭窗口只停止观察，实验继续。需要提前中止实验时，在启动终端按 **Ctrl+C**，由原有运行器关闭子进程。提前中止的记录可能不完整。一次实验完成后自动执行 Round 6 分析，保留 `truth.csv`、`px4.ulg`、`wire.jsonl` 和分析图。关闭 Viewer 后仍可从终端提示的输出目录打开这些文件。

## 数据与进程

实时程序启动原有 Round 6 运行器和一个单独的 Viewer 进程。Viewer 只读取已写入的完整 CSV 行，复制 ENU/FLU 位姿到自己的 MuJoCo 显示模型，调用原生显示同步。它持有独立的 `MjModel` 和 `MjData`，没有发送电机指令或推进物理的接口。渲染以约 30 Hz 为上限，可跳过显示帧。物理步长仍为 4 ms。回放按时间轴选择不晚于光标的原始样本，跳转不改写任何日志。界面标题持续显示 **Recorded Replay**。

完整 Iris 外观来自 PX4 固定版本的上游机身和四桨网格，作为只显示几何加载。地面网格与 E、N、U 三根轴用于定位，青色线来自真值采样轨迹。回放不推算桨叶角度。姿态面板给出 ENU/FLU 的滚转、俯仰和偏航，速度沿用记录的 NED。高度面板同时显示 MuJoCo 相对起点高度及 PX4 本地基准下的设定值。两个基准分别标注。实时 ULog 在独立线程约每秒更新一次，落盘和刷新会使模式或设定值短暂滞后。数据尚未就绪时界面显示等待状态。 [网格来源](assets/README.md)

## M1 实测

在 M1 MacBook Pro 8 GB、原生 macOS arm64、MuJoCo 4.0.1、PX4 v1.17.0 上串行执行三次对照。三个原始实验都通过相同的 Round 6 起降生命周期、时间戳、执行器、误差及落地验收。解锁的每一步都在相同 4 ms 时间戳上取得对应的实际电机指令。独立运行的主机调度和指令到达时刻会变化。

| 指标 | 无 Viewer | Viewer 观察至接地 | 悬停时关闭 Viewer |
| --- | ---: | ---: | ---: |
| 已匹配的解锁步数 | 4,968 | 5,134 | 4,908 |
| 峰值真值高度 | 2.429 m | 2.441 m | 2.413 m |
| Hold 高度跟踪 RMSE | 0.086 m | 0.081 m | 0.088 m |
| HIL_SENSOR 到同一步执行器回复 P95 | 0.849 ms | 0.898 ms | 1.003 ms |
| 同一延迟的最大观测值 | 41.1 ms | 112.8 ms | 213.5 ms |
| 运行墙钟 | 26.03 s | 27.48 s | 26.58 s |
| PX4 / 物理进程峰值 RSS | 6.02 / 50.64 MiB | 5.50 / 48.11 MiB | 5.66 / 49.14 MiB |
| Viewer 进程采样峰值 RSS | — | 631.1 MiB | 482.6 MiB |
| Viewer CPU 采样中位数，相对单核 | — | 118.4% | 113.7% |
| 每秒 Viewer 同步次数 | — | 18.8 | 19.5 |
| 原始样本发送至 Viewer 同步 P95 | — | 91.6 ms | 99.5 ms |

Viewer 的同步次数按应用调用计数。显示延迟从同一主机的 HIL_SENSOR 发送时刻计至 Viewer 同步返回，包含日志缓冲和渲染调度。测试中最慢的主机回包与 GUI 同步有明显抖动，物理时间戳连续。当前对照只测了这台机器上的有限样本，最大延迟易受其它负载影响。打开 Viewer 的单次内存峰值约 631 MiB。

关窗用例在 **15.120 s** 的解锁 Hold 关闭原生窗口，随后物理继续 **8.408 s** 并完成自动上锁。单独测试终端中止时观察进程及仿真子进程均退出，启动器返回 130。接地对照记录了 Takeoff、Hold、Land。每一帧显示的时间戳、位姿和解锁状态都与 CSV 对应采样点完全相同。离线原生键盘试验核对了 32 帧，覆盖播放、暂停、往回跳转、改变倍速和切换跟随相机。原始真值、ULog、MAVLink 文件在操作前后的 SHA-256 一致。

实际窗口截图见 `evidence/live-airborne.png` 和 `evidence/replay.png`。实时截图来自额外的 `r7-live-shot` 完整起降运行，画面显示 13.520 s 的解锁 Hold。

## 回归命令

```sh
.venv/bin/python -m unittest prototypes.iris_viewer.checks -v
.venv/bin/python -m prototypes.iris_viewer.verify \
  --headless captures/r7-headless --live captures/r7-live-a \
  --early-close captures/r7-live-close --replay captures/r6-flight-a \
  --replay-trace evidence/replay-frames.jsonl --out verification.json
```

无界面对照可以直接运行 `prototypes.iris_closed_loop.run`。诊断用例使用实时命令的 `--close-at 15.1`，在该仿真时间之后关闭 Viewer。仓库的实验归档解至工作目录后，使用同名子目录运行上述验收。CI 用原始捕获离线重放核验，真实窗口与键盘实验在 macOS 桌面完成。

Round 7A 分支堆叠在 PR #8 的 `4331543` 上。该提交的三项 CI 成功，GitHub 未记录独立人工审查或未解决行内意见，CodeRabbit 自动审查跳过。使用干净工作树对 Round 6 实验做了单独的无界面复跑。Round 6 仿真及通信文件、上游 PX4、Vehicle-V1 与 S0/S1 均保持原样。 [PR #8](https://github.com/XuanheGuo/Aether/pull/8)

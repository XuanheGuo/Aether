# Aether Round 4 · PX4 SIH 基线

在 Apple Silicon M1 MacBook Pro 8 GB 上原生构建 PX4 **v1.17.0**，以官方 **SIH Quadrotor X**（`10040_sihsim_quadx`）执行自动起飞、悬停、原地降落、降落后自动解除解锁和正常退出。两次运行均由原生 PX4 logger 生成 ULog。运行和分析脚本调用既有控制台、日志和工具库。 [PX4 稳定发布](https://github.com/PX4/PX4-Autopilot/releases/tag/v1.17.0) · [v1.17 SIH 指南](https://docs.px4.io/v1.17/en/sim_sih/)

## PR #5 合并前审查

核对了七个改动文件、既有 CI 和 Apple Silicon 测试。两项工作流均成功，本机重跑五项传感器测试和第二轮五个动力学工况全部通过。动力学摘要与冻结基线逐字节一致。差异检查确认几何、网格、MJCF 和 S0/S1 文件均未变化。审查未发现阻塞项，PR #5 保留待合并。本轮 PX4 实验独立从 `main` 开发。 [PR #5](https://github.com/XuanheGuo/Aether/pull/5) · [既有 CI](https://github.com/XuanheGuo/Aether/actions/runs/45007244921) · [Apple Silicon CI](https://github.com/XuanheGuo/Aether/actions/runs/45007244968)

## 构建与复现

从本目录运行，独立包从解包根目录运行。需要 Apple Silicon 上的 Xcode 命令行工具、Git、Make、CMake、Ninja 和 Python 3.12。实测环境为 macOS 27.3、Apple Clang 21.0.0、CMake 4.4.0 和 Python 3.12.10。以下用 `uv` 创建工作区虚拟环境，已安装 CMake、Ninja 和 ccache 时可直接复用。 [PX4 v1.17 macOS 开发指南](https://docs.px4.io/v1.17/en/dev_setup/dev_env_mac) · [构建测量](evidence/build.json)

```sh
git clone --depth 1 --branch v1.17.0 https://github.com/PX4/PX4-Autopilot.git
PX4_SRC="$PWD/PX4-Autopilot"
git -C "$PX4_SRC" submodule update --init --recursive --jobs 2 -- \
  src/drivers/gps/devices src/lib/heatshrink/heatshrink \
  src/lib/events/libevents src/lib/crypto/monocypher \
  src/modules/mavlink/mavlink src/modules/uxrce_dds_client/Micro-XRCE-DDS-Client
uv venv --python 3.12 .px4-env
uv pip install --python .px4-env/bin/python \
  -r "$PX4_SRC/Tools/setup/requirements.txt" -r requirements.txt
export PATH="$PWD/.px4-env/bin:$PATH"
export CCACHE_DIR="$PWD/.px4-ccache" CCACHE_MAXSIZE=1G
export CMAKE_POLICY_VERSION_MINIMUM=3.5
make -C "$PX4_SRC" -j2 PX4_MAKE_ARGS=-j2 \
  CMAKE_ARGS=-DCMAKE_DISABLE_FIND_PACKAGE_Protobuf=TRUE px4_sitl
python run_sih.py --px4-src "$PX4_SRC" --output runs/quadx --instance 5
ulog_info runs/quadx/flight.ulg
ulog_messages runs/quadx/flight.ulg
python analyze_ulog.py runs/quadx/flight.ulg --output runs/quadx-analysis
```

v1.17 的 SIH 使用标准 `px4_sitl` 构建目标。构建前关闭可选的 Gazebo Protobuf 探测，避免本机新版 Protobuf 的 C++17 要求与该旧组件的 C++14 构建冲突。CMake 4 的最小策略版本兼容设置解决旧子项目的配置要求。PX4 上游源码未修改。官方交互等价入口为 `make px4_sitl sihsim_quadx`，构建时同样传入上述选项。 [上游 SIH 目标定义](https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/src/modules/simulation/simulator_sih/CMakeLists.txt)

脚本为每次运行创建干净的仿真根目录，等待 PX4 原生起飞前检查和位置、速度有效性标志，再通过 `pxh` 发出 `commander takeoff`。PX4 自动进入 Hold，脚本保持约五秒后发出 `commander land`。确认落地并自动解除解锁后停止 logger，执行 `shutdown` 并检查退出码。唯一主动设置的 PX4 参数是 `SDLOG_MODE=2`，用于把地面和飞行阶段写入同一份原生日志。SIH 以 250 Hz 和默认 1×速度运行。 [PX4 日志文档](https://docs.px4.io/v1.17/en/dev_log/logging) · [PX4 PyULog](https://github.com/PX4/pyulog)

已有 ULog 可直接重分析。例如 `python analyze_ulog.py evidence/flight.ulg --output runs/from-evidence`。分析检查飞行阶段、空中估计有效性、失效保护、自动解除解锁、四路电机命令和日志掉包，并输出 CSV、JSON 和误差图。分析以非零退出码报告未达阈值的运行。

## 实际运行结果

| 指标 | 运行一 | 运行二 |
| --- | ---: | ---: |
| ULog 时长 | 26.344 s | 25.704 s |
| 峰值估计高度（相对解锁前） | 2.478 m | 2.481 m |
| 空中 Hold 时段 | 4.84 s | 4.80 s |
| Hold 高度跟踪 RMSE | 0.075 m | 0.071 m |
| Hold 水平跟踪 RMSE | 0.147 m | 0.157 m |
| Hold 姿态跟踪 RMSE | 1.04° | 1.07° |
| Hold 高度估计相对 SIH 真值 RMSE | 0.057 m | 0.062 m |
| Hold 姿态估计相对 SIH 真值 RMSE | 3.86° | 4.20° |
| 日志末尾航向估计相对 SIH 真值偏差 | −16.06° | −16.61° |

两次均记录到解锁期间的 **Takeoff → Hold → Land**，空中位置和速度的四项有效性标志持续为真，无解锁期间失效保护或 ULog dropout。四路电机命令在悬停时均为有限正值。两次均自动解除解锁并以 **0** 退出。SIH 的航向估计误差在飞行后段增大，属于本次基线的观测限制。控制跟踪以设定值与 EKF 状态比较，估计误差以 EKF 状态与 SIH 真值比较。位置采用解锁前约一秒的各自原点对齐，姿态误差使用四元数夹角。所列 RMSE 验收窗为 Hold，航向漂移按全程另行报告。结果限于仿真。 [原生 ULog](evidence/flight.ulg) · [运行指标](evidence/metrics.json) · [复跑指标](evidence/repeat-metrics.json) · [状态与误差图](evidence/flight.png)

双并发构建耗时 **117.02 秒**，`time -l` 报告的最大 RSS 为 **167.5 MiB**，该计数不等于并发进程总内存。生成目录约 202 MiB。两次仿真墙钟分别约 30.2 和 29.7 秒。PX4 进程按约 0.5 秒间隔采样，峰值 RSS **6.20 MiB**，单核心 CPU 百分比的采样中位值为 **20.5% / 16.2%**。 [运行与资源采样记录](evidence/run.json)

## MuJoCo 外部仿真器可行性

**在接口层可行，待做端到端实测。** v1.17 的 `simulator_mavlink` 已提供外部 SITL 通道，可复用 PX4、MAVLink 编解码、状态估计器、控制器和 ULog。现有消息可承载 `HIL_SENSOR`、`HIL_GPS`、独立的 `HIL_STATE_QUATERNION` 真值和回传的 `HIL_ACTUATOR_CONTROLS`。SIH 通过内部 uORB 交换数据。外部验证需要启动独立的 SITL 传输路径并关闭内部 SIH，避免产生两个仿真时钟和两套传感器数据。 [PX4 v1.17 仿真接口](https://docs.px4.io/v1.17/en/simulation/#simulator-mavlink-api) · [v1.17 连接模块源码](https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/src/modules/simulation/simulator_mavlink/SimulatorMavlink.cpp) · [MAVLink 消息定义](https://mavlink.io/en/messages/common.html#HIL_SENSOR)

适配工作集中在三处。

1. **坐标与时间。** 明确 MuJoCo 世界和站点坐标到 PX4 NED/FRD 的变换，校验安装点、比力符号、单位和四元数方向。同步仿真时间戳、各传感器更新率与有效性标志，并验证 lockstep 在启动、断连和退出时的行为。
2. **估计输入与真值。** 复用 MuJoCo 原生 IMU 数据。基于所选标准机型补齐 GNSS、气压和磁场等估计输入。真值独立记录，用于比较控制跟踪和估计误差。
3. **执行器模型。** 对标准 Quadrotor X 先确认回传通道、单位和范围，并把输出接入经过验证的执行器与载荷模型。Vehicle-V1 的推进、传感器与真实执行器物理需要单独辨识。当前基线只覆盖官方标准机型，未验证 Vehicle-V1 的闭环飞行。

下一次接口试验宜用同一官方标准四旋翼对照本轮 ULog，并保留现有 SIH 作为参照。完成这一步所需的是一个薄桥接层。当前评估无需部署 Gazebo 或 ROS 2。

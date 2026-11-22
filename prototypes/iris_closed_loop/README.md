# Aether Round 6 · PX4 Iris × MuJoCo 闭环

两次独立运行都由 PX4 v1.17.0 的原生 **Takeoff → Hold → Land** 完成起飞、悬停、接地和自动上锁。每个已解锁的 4 ms 仿真步使用该步真实返回的四路 PX4 电机输出，MuJoCo 在应用旋翼力后积分。两次运行分别有 **5,034** 和 **5,029** 个已解锁步，均逐步匹配原始 MAVLink 帧，且 PX4 ULog 均无 dropout。

## 模型来源与边界

PX4 使用官方 `10015_gazebo-classic_iris`，物理参数来自 PX4 v1.17.0 固定的 Gazebo Classic Iris SDF，提交 `6697ab169ceab512dc706acea63df4c882662c60`，包元数据声明 BSD。MJCF 是这些参数在 MuJoCo 中的独立移植。Round 5 的 Crazyflie 2 质量约 27 g，使用合成力和力矩接口。此次 Iris 总质量 1.535 kg，并按官方编号布置独立四个旋翼。核对项包括机身、IMU 和旋翼质量与惯量、机身碰撞箱、旋翼位置、电机编号、方向、推力常数、反扭矩系数及转速响应。飞机主体、IMU、四个虚拟电机质量通过 MuJoCo 刚体和焊接子体表示。 [PX4 官方 Iris 配置](https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/ROMFS/px4fmu_common/init.d-posix/airframes/10015_gazebo-classic_iris) · [固定提交 Iris SDF](https://github.com/PX4/PX4-SITL_gazebo-classic/blob/6697ab169ceab512dc706acea63df4c882662c60/models/iris/iris.sdf.jinja) · [上游许可证声明](https://github.com/PX4/PX4-SITL_gazebo-classic/blob/6697ab169ceab512dc706acea63df4c882662c60/package.xml)

前右、后左、前左、后右对应输出 0、1、2、3。解锁时，官方转速目标为 `100 + 1000 × u` rad/s，`u` 是 PX4 非反转归一化输出。上升与下降时间常数分别为 12.5 和 25 ms。推力为 `5.84e-6 × ω²` N，以 MuJoCo 原生 site 执行器施加。反扭矩按旋向取符号，上锁时输出置零。使用 MuJoCo 原生接触。当前模型省略转动桨叶的陀螺效应、旋翼气动阻力和滚转力矩、地效、渲染网格，以及 Gazebo 接触求解器的细节。GNSS、气压模型和 IMU 的部分误差源采用理想化输入。模型精度目前只可追溯到上游 SDF 参数、解析静力检查和本仿真内部一致性，缺少同机型跨引擎轨迹或实机测量误差。 [上游电机实现](https://github.com/PX4/PX4-SITL_gazebo-classic/blob/6697ab169ceab512dc706acea63df4c882662c60/src/gazebo_motor_model.cpp)

## 时间、传感器与故障

世界 ENU 转 NED，机体 FLU 转 FRD，姿态使用 FRD 到 NED 的 wxyz 四元数。比力、角速度和磁场分别使用 m/s²、rad/s 和 gauss。GNSS 经纬度编码为度 × 10⁷、海拔为 mm、速度为 cm/s，气压为 hPa。HIL_STATE_QUATERNION 的加速度按 MAVLink mG 编码。PX4 v1.17 的该真值加速度日志存在已知缩放差异，本轮加速度校验使用原生 IMU。MuJoCo 按 250 Hz 发送主 IMU，GNSS 和独立真值为 50 Hz。磁力计采用上游 100 Hz 和白噪声密度，设定种子 26。4 ms 网格上的磁力计更新间隔在 8 与 12 ms 间交替。首轮诊断使用理想恒定磁场，PX4 日志记录航向对准和本地水平位置未就绪。采用上游传感器配置后，原生起飞前检查正常通过。 [PX4 Simulator MAVLink](https://docs.px4.io/v1.17/en/simulation/#simulator-mavlink-api) · [MAVLink 消息规范](https://mavlink.io/en/messages/common.html#HIL_SENSOR) · [PX4 v1.17 模拟器实现](https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/src/modules/simulation/simulator_mavlink/SimulatorMavlink.cpp)

仿真时间戳从 1,000,000 μs 开始。每次积分前核对 PX4 的时间戳和 lockstep 标志。未解锁的启动期允许零力推进，解锁后等待同一步的输出，超时或断连即停止。配对延迟以同一主机的单调时钟计量，覆盖传输、主机调度和飞控处理。

- **断连。** 2.000 s 时关闭 MuJoCo 连接，累计积分 500 步且全程上锁。1.5 s 探测期间未见 PX4 自动重连。清理旧进程后可用新的运行目录重新启动整个会话。
- **解锁超时。** 对当前测试的 PX4 子进程发送 SIGSTOP，中断同一步的输出。宿主观察到约 0.718 s 后 MuJoCo 以预期状态退出，配置等待上限为 0.6 s，差额含子进程清理和轮询。日志最后一条 IMU 比最后一条已积分记录晚 4 ms，没有积分未获回复的那一步。
- **启动失败记录。** 一次在关闭连接后立即复用实例 7，PX4 启动脚本停在第一路 MAVLink 初始化附近，35 s 内未出现命令提示符。根因未确认。超时试验改用实例 6 后完成；复现命令为不同用例指定不同实例。归档保留此次失败的控制台和初始磁力计试跑。

## 实测结果

| 指标 | Iris A | Iris B |
| --- | ---: | ---: |
| MuJoCo 峰值相对地面高度 | 2.431 m | 2.430 m |
| Hold 时长 | 5.256 s | 5.208 s |
| Hold 高度估计 RMSE（EKF 对 MuJoCo 真值） | 0.0090 m | 0.0086 m |
| Hold 姿态估计 RMSE（四元数夹角） | 0.250° | 0.216° |
| Hold 高度跟踪 RMSE（EKF 对 PX4 设定值） | 0.086 m | 0.084 m |
| Hold 姿态跟踪 RMSE（EKF 对 PX4 设定值） | 0.024° | 0.027° |
| Hold 四旋翼平均总推力，机体轴 | 14.901 N | 14.901 N |
| IMU 发送至同一步执行器返回 P50 / P95 / 最大 | 0.50 / 1.35 / 59.96 ms | 0.45 / 1.18 / 21.18 ms |
| PX4 / MuJoCo 采样峰值 RSS | 6.23 / 60.12 MiB | 5.92 / 58.95 MiB |
| PX4 / MuJoCo 单核 CPU 采样中位数 | 19.4% / 28.9% | 17.1% / 26.1% |
| 端到端墙钟耗时 | 26.733 s | 26.308 s |

实际采样前位置参考由解锁前样本确定。保持高度直到降落命令前，分析脚本按同一仿真时钟对齐 EKF、真值和设定值。悬停均值含进入 Hold 时的高度过渡。离线分析从各自 Hold 段的同一记录初始化两个独立 MuJoCo 状态，各积分一步。回放记录推力时，下一步垂直速度与实测轨迹完全相符；同一步将原生推力置零，垂直速度相差约 **0.039 m/s**。运行中保持原生积分，由控制台发出模式指令。没有注入目标轨迹或设定机体状态。

名义重量为 15.053 N。悬停期间的轴向合力均值与其相差约 1.0%，时窗内有过渡且合力在机体系中统计。这项检查验证量级。两方向 MAVLink 原始帧均可解码且序号连续，已解锁状态全程同时间戳匹配。起动阶段分别有 11 和 12 个未解锁步无执行器回复，均记录为零力。IMU 和磁力计的 PX4 ULog 样本时间按真值网格校验，报告的采样覆盖限于 ULog 的记录窗口。CPU 和 RSS 是两个进程分别采样的结果。

Round 4 的官方 SIH Quadrotor X（`10040`）两次也记录了相同状态序列、自动上锁和正常退出，Hold 分别约 4.84、4.80 s，高度跟踪 RMSE 分别约 0.075、0.071 m。SIH 的控制器经内部 uORB 交换数据，Iris 使用外部 TCP 锁步。机体、传感器、接触和时窗有差异，数字仅供各自基线复核。两批结果限于软件仿真。 [Round 4 SIH 基线](https://github.com/XuanheGuo/Aether/tree/57a1a18533bed5ec08c0ddab3c03ab82c5e21302/prototypes/px4_sih_baseline)

## 复现

实测环境为 M1 MacBook Pro 8 GB、macOS 27.3 arm64、CPython 3.12.10、PX4 v1.17.0（`d6f12ad1c4f70ad3230afd7d86e971421e02fef4`）、MuJoCo 4.0.1 和 PyMAVLink 2.4.50。复用 Round 5 未修改的本机 PX4 SITL 构建。新环境需要 Xcode 命令行工具、Git、Make、CMake、Ninja、Python 和 uv，构建方法见 Round 4。使用双任务和 `CMAKE_POLICY_VERSION_MINIMUM=3.5`，构建命令为 `make -C "$PX4_SRC" -j2 PX4_MAKE_ARGS=-j2 CMAKE_ARGS=-DCMAKE_DISABLE_FIND_PACKAGE_Protobuf=TRUE px4_sitl`。运行无需安装 Gazebo、ROS 2 或 Docker。 [PX4 macOS 构建指南](https://docs.px4.io/v1.17/en/dev_setup/dev_env_mac) · [Round 4 完整依赖与构建命令](https://github.com/XuanheGuo/Aether/blob/57a1a18533bed5ec08c0ddab3c03ab82c5e21302/prototypes/px4_sih_baseline/README.md#构建与复现)

在仓库或独立包根目录运行。每个用例需要新的输出目录和空闲本机实例。仅使用回环仿真端口 `4560 + instance`。普通起降只需运行第一条 `-m prototypes.iris_closed_loop.run` 命令。旧实例退出异常时先确认该实例已清理，再选用其他空闲实例。

```sh
uv venv --python 3.12.10 .venv
uv pip install --python .venv/bin/python -r prototypes/iris_closed_loop/requirements.txt
export PX4_SRC=/absolute/path/to/PX4-Autopilot

.venv/bin/python -m prototypes.iris_closed_loop.run --px4-src "$PX4_SRC" --output runs/iris-a --instance 7
.venv/bin/python -m prototypes.iris_closed_loop.run --px4-src "$PX4_SRC" --output runs/iris-b --instance 7
.venv/bin/python -m prototypes.iris_closed_loop.run --px4-src "$PX4_SRC" --output runs/disconnect --instance 8 --disconnect-test
.venv/bin/python -m prototypes.iris_closed_loop.run --px4-src "$PX4_SRC" --output runs/timeout --instance 6 --timeout-test
.venv/bin/python -m unittest prototypes.iris_closed_loop.checks -v
.venv/bin/python -m prototypes.iris_closed_loop.analyze runs/iris-a runs/iris-b runs/disconnect runs/timeout --out runs/analysis
```

可选的上游参数一致性测试使用 PX4 固定子模块。无需构建或启动 Gazebo。实际验收启用了此项。

```sh
git -C "$PX4_SRC" submodule update --init Tools/simulation/gazebo-classic/sitl_gazebo-classic
export IRIS_SDF="$PX4_SRC/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf.jinja"
.venv/bin/python -m unittest prototypes.iris_closed_loop.checks -v
```

仓库的 `evidence/captures.zip` 和独立交付包的 `captures/` 保留原生 ULog、MuJoCo 真值 CSV、载荷和控制量、原始 MAVLink 十六进制帧、PX4 控制台、配置和运行状态。解包后将四次有效试验的目录传给 `analyze`。两条故障用例、两次正常运行以及早期诊断都保留了原始文件。解析验证、图表和指标可从归档重生。PID、运行调度和日志哈希会随复跑变化。

PR #5、#6、#7 的既有 CI 成功后已依序合入 `main`。Round 6 使用独立分支。此次新增实验仅引用固定 Iris 参数与 Round 5 共用的通信代码，独立于 Vehicle-V1 和 S0/S1。 [PR #5](https://github.com/XuanheGuo/Aether/pull/5) · [PR #6](https://github.com/XuanheGuo/Aether/pull/6) · [PR #7](https://github.com/XuanheGuo/Aether/pull/7)

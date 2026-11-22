# Round 5 · PX4 ↔ MuJoCo 外部仿真通信

在 Apple Silicon M1 8 GB 上，PX4 v1.17.0 与 MuJoCo 4.0.1 作为独立进程通过本机 TCP 完成实际交换。PX4 以通用四旋翼 Iris SITL 配置运行，MuJoCo 使用标准 X 布局的 Menagerie Crazyflie 2 简化模型。整个实验保持 PX4 未解锁、MuJoCo 执行器输入为零。返回通道的零输出用于验证接口和时间戳。Vehicle-V1 及 S0/S1 不参与实验。

## 环境与复现

使用 CPython **3.12.10**、PX4 **v1.17.0**（`d6f12ad1c4f70ad3230afd7d86e971421e02fef4`）、MuJoCo **4.0.1** 和 PyMAVLink **2.4.50**。在 macOS 27.3 arm64 原生验证。PX4 采用 Round 4 的 `make px4_sitl` 构建命令，保留锁步调度器，并用 `CMAKE_ARGS=-DCMAKE_DISABLE_FIND_PACKAGE_Protobuf=TRUE` 跳过与本实验无关的可选组件探测。上游源码无需修改。 [PX4 稳定版](https://github.com/PX4/PX4-Autopilot/releases/tag/v1.17.0) · [Round 4 构建说明](https://github.com/XuanheGuo/Aether/pull/6)

在本目录中准备隔离环境。完成依赖与 PX4 构建后，最后一条命令会启动两个进程、运行断连及重启实验、核对 ULog，并生成结果。

```sh
uv venv --python 3.12.10 .venv
uv pip install --python .venv/bin/python -r requirements.txt
.venv/bin/python run.py --px4-src /path/to/PX4-Autopilot --output runs/round5
```

使用 `--instance 7` 选择 PX4 实例，端口为 `4560 + instance`，仅在 `127.0.0.1` 监听。输出目录须为新路径。可用 `--normal-only` 单独运行正常通信和坏 CRC 检查。运行 `.venv/bin/python -m unittest checks -v` 执行五项自动测试。运行 `.venv/bin/python verify.py runs/round5` 可离线重新解码抓包、核对 ULog 和更新图表。

## 模型、坐标与时钟

- 模型来自官方 MuJoCo Menagerie 的 Crazyflie 2，固定目录提交 `71f066ad0be9cd271f7ed58c030243ef157af9f4`。仓库保留原生 MJCF、39 个网格及 MIT 许可证。模型包含整体推力和机体力矩接口。 [固定提交的模型与来源](https://github.com/google-deepmind/mujoco_menagerie/tree/71f066ad0be9cd271f7ed58c030243ef157af9f4/bitcraze_crazyflie_2) · [随附许可证](vendor/crazyflie2/LICENSE)
- 运行时叠加一个静止固定夹具、重力和理想磁场。前三秒保持静止，随后移除夹具并给予初始平移与角速度。位置、姿态、加速度计和陀螺仪随 MuJoCo 原生物理更新。GNSS 的位置和速度由仿真状态转换而来，气压由理想大气关系计算。定位质量、可见卫星数、温度和磁场是实验假设。观测只验证接口和数值一致性。
- MuJoCo 世界系为 ENU，机体系为 FLU。MAVLink 的世界系和机体系分别为 NED 和 FRD。应用 `(E,N,U) → (N,E,−U)` 和 `(F,L,U) → (F,−L,−U)`。姿态是 FRD 机体转至 NED 世界的 `wxyz` 四元数。HIL_SENSOR 使用 m/s²、rad/s、gauss 和 hPa。位置以度 × 10⁷ 和毫米编码，速度以 cm/s 编码。气压在 PX4 的 uORB 记录中为 Pa。 [MAVLink 消息定义](https://mavlink.io/en/messages/common.html#HIL_SENSOR)
- PX4 使用 `none_iris` 配置启动 v1.17 的 `simulator_mavlink`，通过 TCP 连接桥接器。实测收到 PX4 heartbeat 和真值流间隔请求。桥接器按本实验固定速率发送，GNSS 与真值流均为 50 Hz，主 IMU 为 250 Hz。报文时间为 MuJoCo 仿真时间加 1 秒，以整数微秒表示。PX4 从 ID 0 的 `HIL_SENSOR` 推进自己的仿真时钟。桥接进程以墙钟节拍推进 MuJoCo，未等待执行器返回后再推进下一步。 [PX4 v1.17 仿真接口](https://docs.px4.io/v1.17/en/simulation/#simulator-mavlink-api) · [PX4 v1.17 接收模块](https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/src/modules/simulation/simulator_mavlink/SimulatorMavlink.cpp)

## 实际验证

| 项目 | 本次记录 |
| --- | --- |
| 进程与握手 | 两个独立 PID。PX4 heartbeat 和 HIL 姿态流请求都有原始帧存档 |
| 发送帧 | 1,350 个有效 HIL_SENSOR、271 个 HIL_GPS、271 个 HIL_STATE_QUATERNION，另有 1 个刻意损坏校验和的 IMU 测试帧 |
| PX4 原生接收 | ULog 中实测 IMU、磁力计、气压、GNSS、位置与姿态真值均与仿真状态相符 |
| 执行器返回 | 实收 1,324 个 HIL_ACTUATOR_CONTROLS，约 249 Hz，未解锁标志和通道零值持续成立 |
| 延迟和序号 | 主机单调时钟按同一仿真时间戳匹配传感器发送至执行器返回，P50 为 0.427 ms、P95 为 1.037 ms。两方向的记录均无未解释的 MAVLink 序号缺口 |
| 异常帧 | CRC 坏帧未进入 PX4 IMU ULog，后续有效样本继续被记录 |
| 断连与退出 | 刻意关闭 MuJoCo 套接字后，1.5 秒探测窗内未见原 PX4 进程自动重连。清理旧会话后，以新进程在相同端口恢复交换。正常会话两个进程均退出 0 |

返回延迟包含主机调度和 PX4 处理。ULog 窗口内加速度计和陀螺仪各匹配 1,243 个有效发送样本，磁场和气压捕获率约 99.84%。MAVLink 序号和 ULog 采样覆盖率以各自窗口统计。PX4 v1.17 的主 IMU FIFO 报告相邻已接受样本的梯形平均并量化，按其定义核对后三轴最大误差为 **0.00473 m/s²** 和 **0.00106 rad/s**。GNSS 与真值对照使用 PX4 的发布时刻。 [详细验收和计数](evidence/summary.json) · [PX4 原生 ULog](evidence/exchange.ulg) · [对照与延迟图](evidence/exchange.png)

独立交付包的 `captures/exchange/` 与 `captures/disconnect/` 保存逐帧原始字节（十六进制 JSONL）、主机单调时钟、原生样本 CSV、两端控制台、logger 配置和 PX4 ULog。正常会话的图表和摘要可从这些文件逐项重算。程序输出记录主机时间和仿真时间，重跑的调度延迟与文件哈希会变化。

## PR 与基线复核

先复核 PR #5 的合并前代码范围、五项传感器测试和 CI，再复核 PR #6 的构建和飞行验证。两条 PR 的现有必需检查均通过，未发现需要修复的阻塞项，保留在待合并状态。干净 PX4 工作区重新编译固定版本（双并发，实测 121.91 秒），并从 PR #6 的干净检出复跑官方 SIH Quadrotor X。起飞、悬停、降落、自动上锁和退出均通过。此次悬停高度 RMSE 为 0.039 m，原生 ULog 无 dropout。 [PR #5](https://github.com/XuanheGuo/Aether/pull/5) · [PR #6](https://github.com/XuanheGuo/Aether/pull/6) · [干净构建基线复跑指标](evidence/round4-clean-metrics.json)

## 下一阶段闭环可行性

直接使用 Simulator MAVLink 已满足本次端到端互操作性目标，适合继续验证同类标准四旋翼。当前证明 PX4 能发布外部传感器和真值、推进其仿真时钟并回传执行器帧。未解锁状态下的零输出只验证了回传链路。实际闭环前需选定匹配的标准机型和既有旋翼级模型，核对四个执行器的通道、量纲及反馈边界，然后验证依执行器返回推进的锁步握手、超时及断连恢复。还需评估传感器标定、噪声、启动有效性和故障行为。现有 Crazyflie MJCF 的合成推力与力矩接口和 PX4 Iris 配置存在差异，当前模型及数据不能作为闭环性能基准。

兼容性细节需要单独验证。MAVLink 将 HIL_STATE_QUATERNION 的加速度字段标为 mG，而 v1.17 真值发布代码只除以 1000 后写入标为 m/s² 的本地位置加速度。本轮从 HIL_SENSOR 核对 IMU 加速度，真值消息按位置和姿态验收。接入下一轮前应针对该字段固定转换与回归检查。 [MAVLink 真值消息](https://mavlink.io/en/messages/common.html#HIL_STATE_QUATERNION) · [PX4 v1.17 真值发布代码](https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/src/modules/simulation/simulator_mavlink/SimulatorMavlink.cpp#L551)

v1.17 的一次连接生命周期和本轮观测到的无自动重连行为，需要由监督进程处理。新版桥接方案可在具备匹配机型、明确时间协议或内建恢复能力时对照评估。本轮继续采用直接接口，PX4、MuJoCo 及 MAVLink 编解码均复用既有实现。

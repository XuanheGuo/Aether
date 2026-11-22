# Vehicle-V1 Round 3 传感器验证

在冻结的 Vehicle-V1 上运行 MuJoCo 原生加速度计、陀螺仪和理想速度计。测试覆盖静止、自由落体与无力矩翻滚，三工况各录制理想和带噪数据。几何、质量、惯量及机械限位来自冻结模型。

## 运行和回放

从 `prototypes/aether_v1/` 运行。独立交付包从包根目录运行。使用已安装 MuJoCo、NumPy、SciPy、Matplotlib 的 Python 环境，版本见 [依赖清单](../validation/requirements.txt)。

```sh
python -m unittest sensors.sensor_checks -v
python -m sensors.experiment --output runs/ideal
python -m sensors.experiment --output runs/noisy --seed 31 \
  --accel-std 0.08 --gyro-std 0.004 --velocity-std 0.015
python -m sensors.replay runs/noisy/tumble --output runs/replayed-tumble
```

可用 `--case rest|fall|tumble` 单独记录一组。回放仅依赖 NumPy，生成图表时另需 Matplotlib。可用 `--start`、`--end` 选取时间段，`--stream-rate 1` 按仿真时间输出 JSON 行，`--stream-rate 0` 立即输出所选样本。分析输出包括逐样本误差 CSV、各轴偏差、标准差、RMSE、最大绝对误差和 PNG。输出目录应使用新路径。

## 记录格式与坐标

- `config.json` 保存工况、源 MJCF 与派生 XML 的 SHA‑256、初始状态、采样周期、单位、坐标变换及随机种子。每组 2 秒、500 Hz，共 1,001 帧，CSV 使用同一仿真时钟。
- `truth.csv` 保存世界坐标下的质心位置与速度、机体姿态、安装点的运动和根据刚体运动学投影得到的参考量。`raw.csv` 保存 MuJoCo 原生 `sensordata`。`measurements.csv` 保存理想读数或叠加软件噪声后的读数。回放校验三份 CSV 的时间戳、时间网格及通道。
- 世界系 +Z 朝上，机体系为 CAD 坐标系，+Z 朝机头。安装点在机体系下为 `(0.035, −0.020, 0.850) m`，安装姿态采用外禀 XYZ 欧拉角 `(8°, −12°, 25°)` 定义，四元数按 `wxyz` 保存。若 `R_WB` 和 `R_BS` 分别将机体系转到世界系、安装点局部系转到机体系，则世界矢量乘 `(R_WB R_BS)ᵀ` 得到局部读数。
- 原生 MJCF 使用绑定到同一 `site` 的 `accelerometer`、`gyro`、`velocimeter`。加速度计读数是安装点的比力 `a_site − g`。理想速度计用于观测点速度实验。派生的 `scene.xml` 包含原生传感器定义，可与冻结 MJCF 的 `meshes/` 目录一起加载，初始速度见配置。 [MuJoCo 4.0.1 传感器 XML 参考](https://mujoco.readthedocs.io/en/4.0.1/XMLreference.html#sensor)
- 噪声在原生采样后加入。各轴独立、均值为零、标准差可配，使用 NumPy PCG64。零标准差时原生与测量 CSV 逐项相同。同种子和软件版本下复跑可再现测量。这里的 500 Hz 和噪声水平是仿真设定，实际偏置、延迟、饱和、量化和带宽有待硬件辨识。

## 验收结果

| 工况 | 数值验证 |
| --- | --- |
| 静止 | 倾斜固定，`‖a − g‖ = 9.81 m/s²`，角速度和安装点速度均为零 |
| 自由落体 | 速度随重力变化，比力三轴均小于 2 × 10⁻¹⁵ m/s² |
| 无力矩翻滚 | 三轴角速度随时间变化，偏心安装点产生可测切向和向心加速度 |

三组工况中，原生读数相对参考的逐通道绝对误差均小于 **5.4 × 10⁻¹⁵**，各通道使用各自的 SI 单位。带噪样本的逐轴 RMSE 范围为 **0.0778–0.0826 m/s²**、**0.00389–0.00404 rad/s**、**0.0146–0.0155 m/s**。自动测试另外覆盖噪声种子复现、原生数据完整性、时间窗回放和错误时间戳拒绝。第二轮五组动力学实验复跑后的摘要逐字节一致。 [实验摘要](results.json)

## PX4 与 Gazebo 兼容性评估

| 选项 | Apple Silicon 与可复用接口 | Vehicle-V1 所需适配 |
| --- | --- | --- |
| PX4 SITL + MuJoCo | 当前 PX4 官方 macOS 指南支持 Apple Silicon。PX4 的 Simulator MAVLink API 是外部仿真器候选接口，可复用现成 MAVLink 消息与编解码库。仿真器接口与 MAVSDK/QGroundControl 的客户端接口用途不同。 [PX4 macOS 指南](https://docs.px4.io/main/en/dev_setup/dev_env_mac) · [PX4 仿真架构](https://docs.px4.io/main/en/simulation/#simulator-mavlink-api) | 需要选定 PX4 版本并验证外部 SITL 启动目标。桥接需处理世界、机体与传感器坐标及单位，仿真时间同步、传感器更新标志和有效性，外部传感输入及回传执行器的映射。目前的理想局部速度计不能直接当作 GNSS 或空速数据。 [MAVLink 消息定义](https://mavlink.io/en/messages/common.html#HIL_SENSOR) |
| PX4 SITL + Gazebo | PX4 的 macOS 开发指南提供带 Gazebo Harmonic 的安装路径，模拟依赖由其脚本管理。Gazebo 自身文档另列 Sequoia arm64 的 Harmonic Homebrew 二进制。PX4 使用已有的 `gz_bridge` 与 Gazebo Transport。 [PX4 macOS 指南](https://docs.px4.io/main/en/dev_setup/dev_env_mac) · [Gazebo Harmonic macOS 安装](https://gazebosim.org/docs/harmonic/install_osx/) · [PX4 Gazebo 指南](https://docs.px4.io/main/en/sim_gazebo_gz/) | 要独立建立并校验 SDF、质量惯量、关节、接触和传感器配置。可复用静态显示网格和已有 Gazebo 传感器，仍需验证实际机型和渲染传感器在目标系统上的运行。 |
| PX4 SIH | 官方支持 Apple Silicon 的原生、轻量无界面路径，物理运行在 PX4 内部并经 uORB 交换。适合用已支持机型验证 PX4 自身。 [PX4 SIH 指南](https://docs.px4.io/main/en/sim_sih/) | 其内置物理不直接载入 Vehicle-V1 的 MJCF。现有车辆类型的支持成熟度随版本与类型变化。 |

当前研究继续用 MuJoCo 即可。若后续目标是验证 PX4 自身，先用官方 SITL/SIH 的已支持机型建立基线。若需要 Vehicle-V1 保持原有动力学，再评估最小外部仿真器桥接。仅在必须复用 `gz_bridge`、SDF 场景体系或需要独立引擎对照时，引入 Gazebo 才有明确收益。复用原生传感器、PX4 估计器与日志、MAVLink 编解码及既有桥接，避免重复实现。以上为官方资料评估，本轮执行验证限于 MuJoCo，未在这台 M1 上构建 PX4 或 Gazebo。

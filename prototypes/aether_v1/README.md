# Aether V1 数字样机

本样机用于观察装配、活动面、双轴尾部机构和支脚接触。表内尺寸、材料密度、关节执行器和惯量都是概念参数，未做适航、气动、推进、强度或制造验证。

## 打开和操作

1. 用 macOS ARM64 版 FreeCAD 打开 [Aether-V1.FCStd](model/Aether-V1.FCStd)。已在 FreeCAD 1.1.4 验证。双击树中的 **Concept dimensions** 修改 B 列尺寸，再重新计算。机身壁厚、机头放样、侧面铰链位置、尾罩、支脚位置等由参数表表达式驱动。截面造型和局部机械尺寸可在对应原生草图里编辑。
2. 在 FreeCAD 树中选中任一 **Vane hinge**、**Outer gimbal · X** 或 **Inner nozzle cradle · Y**，在数据面板的 **Kinematics** 下设置 **Angle**。CAD 中的 **Travel** 是建议角度范围，MuJoCo 对该范围施加关节约束。
3. 在已有 MuJoCo 的 Python 环境中，从仓库根目录运行以下命令。拖动右侧 **Control** 面板的六个 `*_target` 滑块，单位为弧度。空格暂停或继续。若控制面板隐藏，可按 Tab 切换侧栏。碰撞代理在渲染组 3，可在可视化设置里查看。

```sh
python prototypes/aether_v1/show.py
```

独立交付包可从包内直接运行 `python show.py`。在新环境中需先安装 MuJoCo Python 包。演示使用 MuJoCo 自带的 managed viewer。 [MuJoCo Python 文档](https://mujoco.readthedocs.io/en/stable/python.html)

## 主要概念参数

| 参数 | 初始值 |
| --- | --- |
| 机身外径 / 直段长度 / 外壳壁厚 | 240 / 1,090 / 5 mm |
| 机头轴向长度 / 整机站立高度 | 350 / 约 1,722 mm |
| 尾罩放样高度 / 尾部旋转中心离地 | 81.2 / 109 mm |
| 活动面展长参数 / 铰链方位 | 132 mm / 每隔 90° |
| 支脚中心半径 / 方位 | 212 mm / 与活动面阵列相差 45° |
| 接缝宽度 | 2 mm |
| 活动面 / 外框 X / 内框 Y 的范围 | ±24° / ±12° / ±15° |
| CAD 体积与概念密度推算的质量 | 约 11.41 kg |

## 结构与重导出

- **Airframe and fairing** 包含空心机身、接缝环和独立曲面机头。机身采用环形草图拉伸，机头由圆截面放样。
- **Four hinged vanes** 包含四组固定座、轴套、销和独立扫掠活动面，旋转轴沿各自的局部径向。
- **Two-axis tail assembly** 包含固定尾罩和支臂、绕 X 轴转动的回转外框，以及嵌套于外框内、绕局部 Y 轴转动的惰性展示喷口。
- **Static display supports** 包含四组放样支腿和脚垫。仿真中的整机自由放在地面上，重力、足地接触和关节执行器驱动其运动。
- 在 FreeCAD 中打开并执行 [Build.FCMacro](Build.FCMacro) 可生成新的工程和 MJCF。它使用 [build_cad.py](build_cad.py) 的原生草图与特征创建装配，并将工程写到宏旁边的 `model/`。运行前请关闭同路径的已打开工程。
- 编辑后将所有铰链的 **Angle** 恢复为 0，保存工程，再打开并执行 [Export.FCMacro](Export.FCMacro)。[export_sim.py](export_sim.py) 从当前工程读取 CAD 局部坐标、关节轴和范围，直接网格化零件，并在工程所在目录生成 [MJCF](model/aether_v1.xml)、STL 网格和 [零件与关节映射](model/assembly.json)。
- CAD 使用毫米。MJCF 将网格及位姿换算为米。渲染网格关闭碰撞和质量贡献。地面接触使用单独的盒体和圆柱代理。各动态刚体的质量和完整惯量张量由局部 CAD 体积、重心和概念密度汇总。机构内部的接触通过抽样几何检查评估，仿真碰撞仅用于整机和地面之间。关节位置执行器有有限的输出力矩，整机只有重力和接触等物理作用。

## 验证与视图

- **FreeCAD 1.1.4** 重新打开工程并验证 41 个单一实体，对直径、机身长度、壁厚、机头长度和活动面展长做参数扰动和重算。尾部检查覆盖 25 个双轴角度组合，在喷口、外框和固定尾罩三者之间未检出体积相交。按喷口 AABB 计算，样本中最小离地间距 12.6 mm。四片活动面各扫描 5 个角度，与机身、各自固定座之间未检出体积相交。这些结果涵盖列出的几何对和离散样本。 [CAD 测试结果](model/cad_checks.json)
- **MuJoCo 4.0.1** 成功加载 41 个 CAD 可视化网格与 6 个铰链。全部动态刚体质量为正，主惯量为正且满足三角条件。四组 CAD 与 MJCF 关节位姿比对的最大旋转四元数误差约为 3.0 × 10⁻¹⁶。5 秒接触仿真中四只脚均与地面接触，六个执行器的最大末态跟踪误差约为 2.76°。 [仿真测试结果](model/sim_checks.json)
- 在 FreeCAD 中执行 [Check.FCMacro](Check.FCMacro) 可复跑 CAD 测试，然后从仓库根目录运行 `python prototypes/aether_v1/check_sim.py`。改动任一基线参数后，应重新导出并复查。
- 两张图由 FreeCAD 实际模型导出。尾部近景中，外框 X 为 9°，喷口局部 Y 为 −12°。 [总览](views/Aether-V1-overview.png) · [尾部近景](views/Aether-V1-tail.png)

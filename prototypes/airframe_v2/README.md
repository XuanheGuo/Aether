# Airframe‑V2 · Round 1

Airframe‑V2 是一个非武器化 EDF 研究项目的概念 CAD 样机。机头朝 FreeCAD 世界坐标 +Z，机身与固定尾罩的接缝位于 Z = 0。尾部内件为惰性形体，四片外部活动面和尾部两轴用于观察几何和装配姿态。模型中的尺寸、展示角度及均质密度均为本轮假设。

## 模型和视图

- 打开 [基线 FreeCAD 工程](evidence/model/Airframe-V2.FCStd)。双击树中的 **Round 1 · Concept dimensions** 编辑 B 列，然后重新计算。活动面和尾部的 **Mechanical display → Angle** 可逐轴编辑。显示角范围 ±12° 属于本轮抽样包络，CAD 属性可自由输入。
- 零件树包含 **Shell and detachable nose**、**Independent exterior panels**、**Inert two-axis tail demonstrator** 和 **Internal support and volume envelopes**。树中的 **Internal support and volume envelopes** 默认隐藏。展开该组可单独显示支撑包络和两块预留设备体积。
- FreeCAD 原生视口截图包括 [正视（从机头看）](evidence/views/front.png)、[侧视](evidence/views/side.png)、[俯视](evidence/views/top.png)、[轴测](evidence/views/axonometric.png)、[纵剖](evidence/views/section.png)、[爆炸装配](evidence/views/assembly.png)、[姿态 A](evidence/views/pose-a.png) 和 [姿态 B](evidence/views/pose-b.png)。纵剖移去 Y ≥ 0 半侧的实体，设备包络保持完整以便辨认。爆炸图按零件分开显示概念接口。

## 参数与构造

| 参数 | 概念值 | 用途 |
| --- | ---: | --- |
| 外径 / 直筒长度 / 名义壁厚 | 180 / 640 / 4 mm | 主壳、机头与尾罩共同外缘 |
| 机头轴长 | 214 mm | 独立空心半椭球，赤道处与直筒相切 |
| 尾罩椭球轴半径 / 开口半径 | 72 / 69 mm | 截取赤道附近曲面，与直筒等径相切 |
| 活动面站位 / 铰轴偏置 / 展长 | 193 / 15 / 52 mm | 四个 90° 方位的径向局部轴 |
| 活动面根部弦长基准 / 厚度基准 | 114 / 8 mm | 使用四个椭圆截面放样 |
| 尾部中心站位 / 外环半径 / 内件基准半径 | −42 / 55 / 32 mm | 外框局部 X，内件随外框绕其局部 Y 转动 |

[完整基线参数](parameters/baseline.json)是生成器的输入。装配局部坐标、原生构造特征和表达式均保存在 FCStd 里，可直接编辑。机头内外椭球均按半轴缩量生成，因此 4 mm 是赤道处名义差值。活动面间隙、尾部支承、内部导轨和设备位置都是概念空间分配。四个活动面固定座与壳体的交叠是预留安装包络。铰链只有运动坐标和形体，后续需决定实际连接界面、支承、紧固和开孔。

## 再生成和交换

在 Apple Silicon 的 FreeCAD 1.1.4 打开并执行 [Build.FCMacro](Build.FCMacro)。宏在脚本旁创建带时间戳的新目录，也可通过 `AETHER_V2_OUTPUT` 指定一个不存在的目标目录。基线由 [build.py](build.py) 建模。[verify.py](verify.py) 计算 BRep 交集、最小间隙与接缝采样。[views.py](views.py) 捕捉 FreeCAD 视口。对手工编辑后的工程，将各轴归零，保存并执行 [Export.FCMacro](Export.FCMacro) 即可重导出。

| 文件 | 用途 |
| --- | --- |
| [FCStd](evidence/model/Airframe-V2.FCStd) | 参数、表达式、原生实体与装配 |
| [STEP](evidence/model/Airframe-V2.step) | 中性 BRep，毫米，保存各实体的中位空间位置 |
| [visual/](evidence/model/visual) | 零件局部坐标 STL，毫米，按清单变换至整机 |
| [assembly.json](evidence/model/assembly.json) | 装配树、关节的局部轴和中位变换，四元数 WXYZ |
| [collision.json](evidence/model/collision.json) | 零件局部包围盒代理，用于粗筛与后续配置 |
| [mass.json](evidence/model/mass.json) | 假定 1000 kg/m³ 的零件级质量、质心与惯量，质量和惯量为 SI，质心为毫米 |

MuJoCo 4.0.1 的[静态导入检查](evidence/mujoco-smoke.json)加载全部 STL 并对比变换后的世界坐标包围盒。可在装有 MuJoCo 的 Python 环境执行 `python smoke_mujoco.py evidence/model`。后续可沿清单层级组装运动模型。箱体代理会填满空心或凹形区域，应按具体接触用途筛选。假定密度计算包含安装包络交叠，整机实测质量和惯量待定。用于 CFD 前需按工况定义外部域、设备开口和分缝，以及处理安装包络的交叠并检查网格边界。STEP 打开后保留独立实体供这一步处理。

## 验证及迭代

[基线检查](evidence/geometry-checks.json)记录全部零件的单实体有效性、离散姿态、干涉、最小间隙与接缝有限差分。抽样包括尾部两轴的 5 × 5 组合、四个活动面的逐轴样本及两组同时动作的姿态。阈值是几何筛查用的数值公差。STEP 往返逐件检查包围盒和体积读数，椭球布尔件的体积积分差异保留在记录中。抽样结论覆盖所列姿态。近接界面使用 BRep 距离，远离界面的条目可用包围盒间距作为保守下界。

[机头加长参数](parameters/longer-nose.json)将机头半轴改为 226 mm。回归直接在重新打开的基线工程中修改参数表，重算并把结果另存为[变体 FCStd](evidence/variant-longer-nose/Airframe-V2-longer-nose.FCStd)，重新打开并导出[变体 STEP](evidence/variant-longer-nose/Airframe-V2-longer-nose.step)。报告给出变化零件、体积和头尖位置，及变体的完整离散检查。

## V1 复用

检查了 `prototypes/aether_v1` 的原生模型、`build_cad.py`、`export_sim.py`、`check_cad.py`、`check_sim.py` 和 `validation/cad_probe.py`。沿用 V1 的 FreeCAD 表达式与嵌套局部坐标表示、中位网格导出、明确单位及保存后回归验证。V2 的模型、宏和记录均位于本目录。基线 V1 原生工程已只读打开并重算，41 个单实体全部有效，识别到 6 个关节，文件 SHA‑256 在检查前后相同。分支的起点是本地缓存的 `origin/main`，提交 `8246024`。

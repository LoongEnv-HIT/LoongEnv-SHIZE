# 机器人模型文件

本目录包含ER15-1400六轴工业机器人的模型文件。

## 文件列表

### URDF模型

#### ER15-1400.urdf
统一机器人描述格式（URDF）模型文件。

**内容**:
- 关节定义（6个旋转关节）
- 连杆几何和惯性参数
- 碰撞和视觉模型
- STL网格引用

**用途**:
- 运动学计算
- 动力学仿真
- 3D可视化
- 碰撞检测

### MJCF模型

#### er15-1400.mjcf.xml
MuJoCo模型格式（MJCF）文件。

**内容**:
- MuJoCo物理引擎配置
- 关节和执行器定义
- 接触参数
- 传感器配置

**用途**:
- MuJoCo仿真
- 物理交互
- 接触力计算

### STL网格文件

3D几何网格文件，用于可视化和碰撞检测。

#### b_link.STL
基座连杆网格。

#### l_1.STL
第1连杆网格。

#### l_2.STL
第2连杆网格。

#### l_3.STL
第3连杆网格。

#### l_4.STL
第4连杆网格。

#### l_5.STL
第5连杆网格。

#### l_6.STL
第6连杆网格（末端执行器）。

## 机器人规格

### ER15-1400 参数

| 参数 | 值 |
|------|-----|
| 自由度 | 6 |
| 负载能力 | 15 kg |
| 工作半径 | 1400 mm |
| 重复定位精度 | ±0.1 mm |
| 总质量 | 206 kg |

### 关节限制

| 关节 | 最小角度 (rad) | 最大角度 (rad) | 最大速度 (rad/s) |
|------|---------------|---------------|-----------------|
| Joint 1 | -2.967 | 2.967 | 3.14 |
| Joint 2 | -2.7925 | 1.5708 | 3.14 |
| Joint 3 | -1.4835 | 3.0543 | 3.14 |
| Joint 4 | -3.316 | 3.316 | 3.14 |
| Joint 5 | -2.2689 | 2.2689 | 3.14 |
| Joint 6 | -6.2832 | 6.2832 | 3.14 |

### 动力学参数

#### 质量 (kg)
- Link 1: 54.52
- Link 2: 11.11
- Link 3: 25.03
- Link 4: 10.81
- Link 5: 4.48
- Link 6: 0.28

#### 质心位置 (m)
各连杆的质心相对于关节坐标系的位置。

#### 惯性张量 (kg·m²)
各连杆的转动惯量矩阵。

## 使用方法

### 加载URDF模型

```python
from robot_motion_control.core.models import RobotModel

robot = RobotModel(
    name="ER15-1400",
    n_joints=6,
    urdf_path="models/ER15-1400.urdf"
)
```

### 使用Pinocchio加载

```python
import pinocchio as pin

model = pin.buildModelFromUrdf("models/ER15-1400.urdf")
data = model.createData()
```

### 在Web界面中使用

URDF和STL文件会自动加载到Web界面的3D查看器中。

文件位置: `ui/public/models/`

## 模型坐标系

### 基座坐标系
- 原点：基座底部中心
- X轴：向前
- Y轴：向左
- Z轴：向上

### DH参数
使用标准DH参数表示法。

## 文件格式

### URDF格式
XML格式，包含：
- `<robot>`: 根元素
- `<link>`: 连杆定义
- `<joint>`: 关节定义
- `<visual>`: 视觉模型
- `<collision>`: 碰撞模型
- `<inertial>`: 惯性参数

### STL格式
二进制STL格式，包含三角网格数据。

## 模型验证

### 检查模型完整性

```python
import pinocchio as pin

# 加载模型
model = pin.buildModelFromUrdf("models/ER15-1400.urdf")

# 打印模型信息
print(f"关节数: {model.njoints}")
print(f"自由度: {model.nv}")
print(f"连杆数: {model.nlinks}")

# 验证质量矩阵
q = pin.neutral(model)
data = model.createData()
M = pin.crba(model, data, q)
print(f"质量矩阵条件数: {np.linalg.cond(M)}")
```

### 可视化模型

```python
from pinocchio.visualize import MeshcatVisualizer

viz = MeshcatVisualizer(model, collision_model, visual_model)
viz.initViewer()
viz.loadViewerModel()
viz.display(q)
```

## 模型修改

### 修改URDF

1. 使用文本编辑器打开URDF文件
2. 修改参数（质量、惯性、几何等）
3. 保存并验证XML格式
4. 重新加载模型测试

### 修改STL网格

1. 使用3D建模软件（Blender、FreeCAD等）
2. 导入STL文件
3. 编辑网格
4. 导出为STL格式（二进制）
5. 替换原文件

## 注意事项

1. **单位**: URDF使用米(m)和千克(kg)
2. **坐标系**: 右手坐标系
3. **文件路径**: 使用相对路径引用STL文件
4. **网格质量**: STL网格应适当简化以提高性能

## 相关工具

- **URDF编辑器**: 
  - ROS URDF Tools
  - Gazebo Model Editor
  
- **3D建模软件**:
  - Blender (开源)
  - FreeCAD (开源)
  - SolidWorks (商业)

- **可视化工具**:
  - RViz
  - Meshcat
  - Gepetto Viewer

## 参考资料

- [URDF规范](http://wiki.ros.org/urdf/XML)
- [Pinocchio文档](https://stack-of-tasks.github.io/pinocchio/)
- [STL格式说明](https://en.wikipedia.org/wiki/STL_(file_format))

## 更新日期

2026-01-29

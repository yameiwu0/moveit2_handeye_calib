
# MoveIt2 Hand-Eye Calibration (moveit2_handeye_calib)

基于 **MoveIt2** 的 **手眼标定 (Hand-Eye Calibration) 工具**，适配 ROS 2 机械臂平台。
目前仓库已在 **TG_ARM 六轴机械臂 + ROS 2 Humble** 平台上完成实现与验证。其他机械臂只需保证 **话题接口一致**（末端位姿、相机 TF 等），即可快速接入使用。

---

## ✨ 功能特性

- 支持 **eye-in-hand**（相机挂载在机械臂末端）和 **eye-to-hand**（相机固定在环境中）两种模式。
- 基于 **MoveIt2 Calibration** 框架，集成 ChArUco / ArUco 标定板采集。
- 通过 MoveIt2 与 RViz2 提供交互式 GUI，标定过程可视化。
- 标定结果以 **TF 发布**，可直接用于机器人视觉任务。
- 支持 TG_ARM 系列机械臂（ARM380、ARM620 等），未来可扩展至其他机械臂平台。

---

## 🔗 参考项目与文档

- **MoveIt2 Hand-Eye Calibration 教程**
  [moveit.picknik.ai 文档](https://moveit.picknik.ai/humble/doc/examples/hand_eye_calibration/hand_eye_calibration_tutorial.html)

- **MoveIt Calibration (ROS2 port)**
  [AndrejOrsula/moveit2_calibration](https://github.com/AndrejOrsula/moveit2_calibration)

---

## ⚙️ 环境依赖

建议配置：
- **操作系统**: Ubuntu 22.04
- **ROS 版本**: ROS 2 Humble (推荐)
- **相机要求**: **RGB-D 深度相机**（出厂已标定内参）
  - ✅ Intel RealSense 系列（D435、D435i、D455 等）
  - ✅ Orbbec 系列（Astra、Gemini 等）
  - ✅ Azure Kinect
  - ❌ 普通 USB 摄像头（需额外标定内参）
- **依赖软件包**:
  - `moveit2`
  - `moveit_calibration`
  - `tf2_ros`, `geometry_msgs`, `sensor_msgs`
  - `OpenCV` (用于 ArUco/ChArUco 检测，需要 contrib 模块)
  - TG_ARM 机械臂 ROS2 驱动（包含在本仓库）
  - `realsense2_camera` 或 `orbbec_camera`（根据相机型号选择）
  - `ros2_socketcan` (用于 CAN 总线通信)

> **为什么需要 RGB-D 相机？**
> - RGB-D 相机出厂已标定内参，驱动自动发布 `camera_info` 话题
> - 手眼标定插件需要读取 `camera_info` 中的内参矩阵来计算标定板位姿
> - 普通摄像头需要先用棋盘格标定内参，并手动发布 `camera_info`，较为繁琐

### 安装依赖：

```bash
# 基础依赖
sudo apt update
sudo apt install ros-humble-realsense2-*
sudo apt install ros-humble-moveit-*
sudo apt install ros-humble-can-msgs

# OpenCV（包含 ArUco 模块）
sudo apt install libopencv-dev libopencv-contrib-dev

# image_geometry
sudo apt install ros-humble-image-geometry

# MoveIt 可视化工具
sudo apt install ros-humble-moveit-visual-tools
sudo apt install ros-humble-moveit-core ros-humble-moveit-ros-planning-interface
```

---

## 📂 系统架构 & 代码结构

```text
moveit2_handeye_calib/
├── TG_ARM/                    # TG_ARM 六轴机械臂 ROS 2 驱动与支持包
│   ├── robot_bringup/         # 启动相关节点与配置
│   ├── robot_config/          # 机械臂配置文件
│   │   ├── arm380_config/     # ARM380 MoveIt2 配置
│   │   ├── arm620_config/     # ARM620 MoveIt2 配置
│   │   └── arm620_dh_config/  # ARM620 DH参数配置
│   ├── robot_control/         # 控制器实现
│   ├── robot_description/     # URDF/SRDF 模型文件
│   │   └── urdf/
│   │       ├── arm380.urdf
│   │       ├── arm620.urdf
│   │       └── arm620_dh.urdf
│   ├── robot_driver/          # 底层驱动接口
│   ├── robot_dynamics/        # 动力学相关
│   ├── robot_interfaces/      # 自定义消息与服务定义
│   ├── robot_sdk/             # 机械臂 SDK 封装
│   ├── robot_state_machine/   # 状态机管理
│   └── ros2_socketcan/        # CAN 总线通信支持
│
├── moveit2_calibration/       # MoveIt2 手眼标定功能包
│   ├── moveit_calibration_demos/   # 标定流程示例
│   ├── moveit_calibration_gui/     # RViz 插件与交互式 GUI
│   └── moveit_calibration_plugins/ # ArUco/ChArUco 插件与算法实现
│
└── README.md                  # 项目说明文档
```

---

### 各模块说明

1. **TG_ARM**
   - 提供 **TG_ARM 六轴机械臂在 ROS2 下的完整支持**，包括硬件接口、运动控制器、状态管理以及 MoveIt2 配置文件。
   - 支持 **ARM380**、**ARM620** 等多种型号机械臂。
   - 通过 **CAN 总线**（ros2_socketcan）与机械臂底层通信，实现实时控制。
   - 集成 MoveIt2 规划与执行接口，可直接用于手眼标定。

2. **moveit2_calibration**
   - MoveIt2 官方的 **手眼标定功能包**，包含标定插件、图形化界面和示例程序。
   - 提供基于 **ArUco/ChArUco** 标定板的采集与求解功能。
   - 与 TG_ARM 的 MoveIt2 配置结合，即可完成 **eye-in-hand** 和 **eye-to-hand** 标定。

3. **顶层集成**
   - 本仓库通过将 **TG_ARM** 与 **moveit2_calibration** 统一放置在同一工作区，实现 TG_ARM 机械臂的手眼标定方案。
   - 支持扩展至 **其他机械臂平台**，只需保证末端位姿与相机 TF 话题对齐即可。

---

## 🚀 使用方法

### 1. 克隆仓库

```bash
cd ~/ros2_ws/src
git clone https://github.com/yameiwu0/moveit2_handeye_calib.git
```

### 2. 编译

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

**注意**：如果在 ARM 平台（如 Jetson）上编译，可能需要指定 OpenCV 路径：

```bash
# 先查找 OpenCV 配置文件路径
find /usr -name "OpenCVConfig.cmake" 2>/dev/null

# 使用正确的路径编译
colcon build --cmake-args -DOpenCV_DIR=/usr/lib/aarch64-linux-gnu/cmake/opencv4
```

### 3. 启动手眼标定系统

本仓库提供了**一键启动所有必要节点**的启动文件，无需单独启动机械臂和相机。

#### 使用 RealSense 相机

```bash
# ARM620 + RealSense 手眼标定
ros2 launch robot_bringup arm620_handeye_calib.launch.py

# ARM380 + RealSense 手眼标定
ros2 launch robot_bringup arm380_handeye_calib.launch.py
```

#### 使用 Orbbec Astra 相机

```bash
# ARM620 + Orbbec Astra 手眼标定
ros2 launch robot_bringup arm620_handeye_calib_orbbec.launch.py

# ARM380 + Orbbec Astra 手眼标定
ros2 launch robot_bringup arm380_handeye_calib_orbbec.launch.py
```

> **其他相机**：只要相机发布标准 ROS2 图像话题（如 `/camera/color/image_raw`），即可参考现有配置文件进行适配。

这个启动文件会**自动启动以下所有组件**：
- ✅ 机械臂 CAN 驱动（robot_driver）
- ✅ 机械臂 SDK 功能（robot_sdk）
- ✅ 机械臂 URDF/TF 发布
- ✅ 机械臂控制器（robot_control）
- ✅ MoveIt2 规划与执行接口（含 RViz）
- ✅ 相机驱动（RealSense / Orbbec）
- ✅ 相机安装位置 TF（eye-in-hand 配置，相机装在末端）

**启动文件位置**：

| 机械臂 | 相机 | 启动文件 |
|--------|------|----------|
| ARM620 | RealSense | [arm620_handeye_calib.launch.py](TG_ARM/robot_bringup/launch/arm620_handeye_calib.launch.py) |
| ARM620 | Orbbec | [arm620_handeye_calib_orbbec.launch.py](TG_ARM/robot_bringup/launch/arm620_handeye_calib_orbbec.launch.py) |
| ARM380 | RealSense | [arm380_handeye_calib.launch.py](TG_ARM/robot_bringup/launch/arm380_handeye_calib.launch.py) |
| ARM380 | Orbbec | [arm380_handeye_calib_orbbec.launch.py](TG_ARM/robot_bringup/launch/arm380_handeye_calib_orbbec.launch.py) |

**重要提示**：
- 使用此启动文件后，**无需再单独启动机械臂驱动或相机节点**
- RViz 会自动打开，并加载手眼标定专用配置
- 相机 TF 已配置为 eye-in-hand 模式（相机装在 Link6 末端）

### 4. 进行手眼标定

启动后，RViz 会自动打开并显示机械臂模型和相机视图。

**已预配置的标定参数**（无需手动设置）：

RViz 配置文件 `arm620_handeye_calib.rviz` 已预设好以下参数，标定人员可直接使用：

| 参数 | 预设值 | 说明 |
|------|--------|------|
| Target Type | `HandEyeTarget/Aruco` | ArUco 标定板 |
| ArUco dictionary | `DICT_ARUCO_ORIGINAL` | ArUco 字典类型 |
| markers, X | 2 | X 方向 marker 数量 |
| markers, Y | 2 | Y 方向 marker 数量 |
| marker size (px) | 400 | marker 像素大小 |
| marker separation (px) | 40 | marker 间距像素 |
| measured marker size (m) | 0.085 | 实际测量的 marker 尺寸（米） |
| measured separation (m) | 0.013 | 实际测量的 marker 间距（米） |
| Calibration Type | eye_in_hand | 相机装在末端 |
| Sensor Frame | camera_color_optical_frame | 相机光学坐标系 |
| End-effector Frame | Link6 | 末端执行器坐标系 |
| Robot Base Frame | base_link | 机器人基座坐标系 |
| Camera Image Topic | /camera/camera/color/image_raw | 相机图像话题 |

> **注意**：如果您的标定板尺寸不同，请在 **Target** 标签页中修改 `measured marker size (m)` 和 `measured separation (m)` 为实际测量值。

**标定步骤**：(推荐手动标定，手动标定的重投影误差可以达到0.2,0.2左右)

1. 在 RViz 左侧找到 **HandEye Calibration** 插件面板
2. 切换到 **Target** 标签页，点击 **Create Target** 生成标定板图像并打印
3. 测量打印后的标定板实际尺寸，如有差异请更新 `measured marker size (m)` 和 `measured separation (m)`
4. 切换到 **Context** 标签页，确认参数配置正确（已预设好，通常无需修改）
5. 切换到 **Calibrate** 标签页
6. 使用 **MoveIt2** 控制机械臂移动到不同姿态，每个姿态下点击 **Take Sample** 采集数据点
7. 采集 **至少 5-10 个不同姿态** 的数据点（越多越好）
8. 点击 **Solve** 计算手眼变换矩阵
9. 查看标定结果的重投影误差，确保精度满足要求
10. 标定结果会通过 **TF 自动发布**到系统中

**管理采集点**：

| 按钮 | 功能 |
|------|------|
| **Take Sample** | 采集当前姿态的数据点 |
| **Delete Selected** | 删除选中的单个采集点 |
| **Clear All** | 清除所有采集点 |
| **Solve** | 计算手眼标定结果 |

如果某个采集点不合适（如标定板检测不准确、机械臂位姿不佳），可以单独删除：
1. 在左侧采集点列表中**点击选中**要删除的采集点（如 Sample 3）
2. 点击 **Delete Selected** 按钮删除该点
3. 剩余采集点会自动重新编号

### 全自动标定模式（自动手眼标定的重投影误差基本在0.8,0.5左右）

除了手动标定外，本项目还提供了**全自动标定功能**，使用球面采样算法自动生成多个标定姿态，一键完成整个标定过程。

**使用方法**：

1. 在 RViz 的 HandEye Calibration 插件面板中，找到 **Full Auto Calibration (Spherical Sampling)** 区域
2. 设置 **Number of poses**（标定姿态数量，建议 20 个，默认 15）
3. **将机械臂移动到一个合适的初始位置**（相机能看到标定板的位置）
4. 点击 **Start Auto Calibrate** 按钮开始自动标定
5. 系统会自动：
   - 基于当前末端位姿，在球面上均匀生成多个目标姿态
   - 依次规划并执行到每个姿态
   - 等待机器人稳定后自动采集样本
   - 采集足够样本后自动计算标定结果
6. 标定过程中可以点击 **Stop** 按钮停止（如果已采集足够样本，会自动计算结果）

**全自动标定参数说明**：

| 参数 | 说明 |
|------|------|
| **Number of poses** | 生成的标定姿态数量（5-30），越多精度越高 |
| **球面半径** | 约 5cm（代码中硬编码），末端执行器在此范围内移动 |
| **姿态倾斜角** | 最大 25 度倾斜，确保标定数据多样性 |

**球面采样算法特点**：

- 使用 **Fibonacci 球面采样**算法，确保姿态在空间中均匀分布
- 自动在末端当前位置周围的上半球面上生成采样点
- 每个姿态包含位置和姿态变化，确保标定数据的多样性
- 失败的姿态会自动跳过，继续尝试下一个
- 采集到 5 个以上样本后即可计算标定结果

**注意事项**：

- 开始自动标定前，确保相机能看到标定板
- 标定板应固定放置（eye-in-hand 模式）
- 确保机械臂周围没有障碍物，有足够的运动空间
- 建议使用较低的运动速度（代码中已设置为 30%）
- 如果规划频繁失败，可能需要调整初始位置或缩小采样范围

**手动标定注意事项**：
- 标定板需要保持静止（eye-in-hand 模式下）
- 采集数据时应覆盖机械臂工作空间的不同位置和角度
- 确保相机能清晰看到标定板，避免遮挡和反光
- 标定板与相机距离建议在 0.3-1.0 米之间
- 每个姿态下确保标定板完全在相机视野内且检测框稳定

### 5. 保存标定结果

标定完成后，需要将标定结果永久保存到启动文件中，以便后续使用。

**步骤 1：查看标定结果**

在 RViz 的 HandEye Calibration 插件面板中，标定完成后会显示变换结果，包括：
- Translation（平移）: x, y, z
- Rotation（旋转）: 四元数 qx, qy, qz, qw

或者使用命令查看 TF：
```bash
# eye-in-hand 模式：查看相机光学坐标系到末端的变换
ros2 run tf2_ros tf2_echo Link6 camera_color_optical_frame
```

**步骤 2：创建标定结果启动文件**

创建一个新的启动文件来发布标定结果。参考示例 [TG_ARM/robot_bringup/launch/bd01.launch.py](TG_ARM/robot_bringup/launch/bd01.launch.py)：

```python
""" Static transform publisher acquired via MoveIt 2 hand-eye calibration """
""" EYE-IN-HAND: Link6 -> camera_color_optical_frame """
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    nodes = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="log",
            arguments=[
                "--frame-id",
                "Link6",
                "--child-frame-id",
                "camera_color_optical_frame",
                "--x",
                "-0.0319045",        # 替换为你的标定结果
                "--y",
                "0.0530007",         # 替换为你的标定结果
                "--z",
                "-0.0141292",        # 替换为你的标定结果
                "--qx",
                "0.0315969",         # 替换为你的标定结果（四元数）
                "--qy",
                "0.00402934",        # 替换为你的标定结果
                "--qz",
                "-0.0201579",        # 替换为你的标定结果
                "--qw",
                "0.999289",          # 替换为你的标定结果
            ],
        ),
    ]
    return LaunchDescription(nodes)
```

**步骤 3：使用标定结果**

保存文件后，在需要使用手眼标定结果的场景中启动这个文件：

```bash
# 启动标定结果 TF 发布
ros2 launch robot_bringup bd01.launch.py
```

或者将其集成到你的主启动文件中，例如在 `arm620_handeye_calib.launch.py` 中替换原有的 `camera_mount_tf` 节点。

**重要说明**：
- 使用 **四元数** (`--qx`, `--qy`, `--qz`, `--qw`) 而不是欧拉角，精度更高
- 标定结果的 `child-frame-id` 应该是 `camera_color_optical_frame`（相机光学坐标系），而不是 `camera_link`
- 每次重新标定后，更新这个文件中的数值即可

### 6. 转换为标定矩阵（可选）

如果你需要将标定结果转换为 **4×4 齐次变换矩阵**（用于视觉定位、坐标转换等应用），可以使用提供的工具脚本。

**使用方法**：

```bash
# 确保标定结果已发布（启动 bd01.launch.py 或包含标定 TF 的启动文件）
ros2 launch robot_bringup bd01.launch.py

# 在另一个终端运行脚本读取标定矩阵
python3 /home/orin/work/moveit2_handeye_calib/TG_ARM/robot_bringup/scripts/read_calib_matrix.py
```

**脚本输出示例**：

```
======================================================================
手眼标定矩阵 (Link6 -> camera_color_optical_frame)
======================================================================

平移向量 (Translation):
  x: -0.0319045 m
  y: 0.0530007 m
  z: -0.0141292 m

旋转 (Rotation) - 四元数:
  qx: 0.0315969
  qy: 0.0040293
  qz: -0.0201579
  qw: 0.9992890

旋转 (Rotation) - 欧拉角 (XYZ, 度):
  roll:  3.6341°
  pitch: 0.3885°
  yaw:   -2.3240°

4x4 齐次变换矩阵:
[[ 0.99789  -0.04043   0.05074  -0.03190]
 [ 0.03996   0.99913   0.00880   0.05300]
 [-0.05108  -0.00648   0.99868  -0.01413]
 [ 0.00000   0.00000   0.00000   1.00000]]

======================================================================
用于视觉定位的应用示例:
======================================================================
# 将相机坐标系下的点转换到机械臂末端坐标系
import numpy as np
T = np.array([[0.9979, -0.0404, 0.0507, -0.0319], ...])
P_camera = np.array([x, y, z, 1])  # 相机坐标系下的点
P_link6 = T @ P_camera              # 转换到 Link6 坐标系
======================================================================
```

**矩阵含义**：

这个 4×4 齐次变换矩阵 `T` 描述了相机光学坐标系相对于机械臂末端 Link6 的变换：
- **左上角 3×3**：旋转矩阵（相机相对于 Link6 的姿态）
- **右上角 3×1**：平移向量（相机相对于 Link6 的位置，单位：米）
- **底部 1×4**：齐次坐标 [0 0 0 1]

**应用示例**：

在机器视觉应用中，如果相机检测到一个物体在相机坐标系下的位置为 `P_camera = [0.1, 0.05, 0.5, 1]`，可以通过标定矩阵将其转换到机械臂末端坐标系：

```python
import numpy as np

# 使用上面脚本输出的矩阵
T = np.array([
    [ 0.99789, -0.04043,  0.05074, -0.03190],
    [ 0.03996,  0.99913,  0.00880,  0.05300],
    [-0.05108, -0.00648,  0.99868, -0.01413],
    [ 0.00000,  0.00000,  0.00000,  1.00000]
])

# 相机坐标系下检测到的点
P_camera = np.array([0.1, 0.05, 0.5, 1])

# 转换到 Link6 坐标系
P_link6 = T @ P_camera

print(f"物体在 Link6 坐标系下的位置: {P_link6[:3]}")
```

这样就可以将视觉检测结果转换到机械臂坐标系，用于抓取规划等任务。

### 7. 多平台使用

如果使用其他机械臂（如 UR、xArm）：

- 确保提供 **末端位姿** 话题以及 **MoveIt2 控制接口**
- 提供正确的 URDF/SRDF 模型
- 其他部分无需修改

---

## 🔧 常见问题与解决方案

### 问题一：找不到 OpenCV ArUco 模块

```
CMake Error: Could NOT find OpenCV (missing: aruco) (found version "4.8.0")
```

**解决办法**：

```bash
# 1. 更新软件包列表
sudo apt update

# 2. 安装完整的 OpenCV（核心 + contrib模块）
sudo apt install libopencv-dev libopencv-contrib-dev

# 3. 验证安装
pkg-config --modversion opencv4

# 4. 查找正确的 OpenCVConfig.cmake 路径
find /usr -name "OpenCVConfig.cmake" 2>/dev/null

# 5. 清理缓存并重新编译
cd ~/ros2_ws
rm -rf build install log
colcon build --cmake-args -DOpenCV_DIR=/usr/lib/aarch64-linux-gnu/cmake/opencv4
```

### 问题二：找不到 image_geometry

```
CMake Error: Could not find a package configuration file provided by "image_geometry"
```

**解决办法**：

```bash
# 安装
sudo apt install ros-humble-image-geometry

# 重新编译
colcon build --cmake-args -DOpenCV_DIR=/usr/lib/aarch64-linux-gnu/cmake/opencv4
```

### 问题三：找不到 moveit_visual_tools

```
CMake Error: Could not find a package configuration file provided by "moveit_visual_tools"
```

**解决办法**：

```bash
# 安装 moveit_visual_tools 及其依赖
sudo apt update
sudo apt install ros-humble-moveit-visual-tools

# 安装其他 MoveIt 核心包
sudo apt install ros-humble-moveit-core ros-humble-moveit-ros-planning-interface

# 安装常见的 MoveIt 工具包
sudo apt install ros-humble-moveit-servo ros-humble-moveit-task-constructor
```

### 问题四：moveit_calibration_gui 编译失败（cv_bridge 类型不匹配）

如果遇到 `cv_bridge::CvImage` 构造函数类型不匹配的错误，这是由于系统中存在 ROS 1 版本的 cv_bridge 与 ROS 2 版本冲突。

**解决办法（推荐）**：

```bash
# 1. 移除系统版本的 cv_bridge（可能与 ROS 2 冲突）
sudo apt remove libcv-bridge-dev libcv-bridge2d

# 2. 确保 ROS 2 版本已安装
sudo apt install --reinstall ros-humble-cv-bridge

# 3. 清理并重新编译
cd ~/work/moveit2_handeye_calib
rm -rf build/moveit_calibration_gui
colcon build --packages-select moveit_calibration_gui --cmake-args -DOpenCV_DIR=/usr/lib/aarch64-linux-gnu/cmake/opencv4
```

**备选方案**：如果上述方法仍有问题，可以暂时跳过 GUI 包编译：

```bash
colcon build --packages-skip moveit_calibration_gui
```

标定功能不依赖 GUI 包，核心功能不受影响。

---

## 📺 演示视频

查看项目手眼标定演示视频：
[![演示视频](https://img.youtube.com/vi/BV18FYqz9ECG/0.jpg)](https://www.bilibili.com/video/BV18FYqz9ECG)
B站视频
www.bilibili.com/video/BV18FYqz9ECG/
---

## 📜 许可证

MIT License

---

## 🙏 致谢

- [MoveIt Calibration](https://github.com/ros-planning/moveit_calibration) 提供的标定框架
- [AndrejOrsula/moveit2_calibration](https://github.com/AndrejOrsula/moveit2_calibration) 提供的 ROS 2 移植版本
- 以及所有开源社区贡献者

---

💡 **欢迎贡献**：如果你在其他机械臂平台上完成了适配，欢迎提交 PR！

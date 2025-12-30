# 集成新相机指南

本文档介绍如何将新的相机集成到手眼标定系统中。

**前提条件**：已安装并测试好相机的 ROS2 驱动。

---

## 快速概览

集成新相机需要创建/修改 **2 个文件**：

| 文件类型 | 路径 | 作用 |
|---------|------|------|
| Launch 文件 | `TG_ARM/robot_bringup/launch/arm620_handeye_calib_新相机.launch.py` | 启动相机驱动和配置 TF |
| RViz 配置 | `TG_ARM/robot_bringup/rviz/arm620_handeye_calib_新相机.rviz` | 配置图像话题和坐标系 |

---

## 第一步：获取相机信息

在集成之前，先单独启动相机驱动，获取以下信息：

```bash
# 1. 启动相机驱动
ros2 launch <相机驱动包名> <相机launch文件>.py

# 2. 查看相机发布的话题
ros2 topic list | grep -E "image|camera"

# 3. 查看相机的 TF 坐标系
ros2 run tf2_ros tf2_echo camera_link camera_color_optical_frame
```

**需要记录的信息**：

| 项目 | 示例 (Orbbec Gemini2) | 你的相机 |
|------|----------------------|---------|
| 驱动包名 | `orbbec_camera` | |
| Launch 文件 | `gemini2.launch.py` | |
| 彩色图像话题 | `/camera/color/image_raw` | |
| 相机链接坐标系 | `camera_link` | |
| 光学坐标系 | `camera_color_optical_frame` | |

---

## 第二步：创建 Launch 文件

### 2.1 复制模板

```bash
cd /home/w/work/moveit2_handeye_calib/moveit2_handeye_calib/TG_ARM/robot_bringup/launch

# 以 Orbbec 版本为模板
cp arm620_handeye_calib_orbbec.launch.py arm620_handeye_calib_新相机.launch.py
```

### 2.2 修改 Launch 文件

打开新文件，修改以下 **3 处**：

#### 修改 1：文件描述（第 2 行）

```python
# 修改前
"""TG_ARM ARM620 手眼标定启动文件 - Orbbec Astra 相机版 (Eye-in-Hand 配置)"""

# 修改后
"""TG_ARM ARM620 手眼标定启动文件 - 新相机 相机版 (Eye-in-Hand 配置)"""
```

#### 修改 2：RViz 配置文件路径（第 19-22 行）

```python
# 修改前
default_rviz_config = os.path.join(
    get_package_share_directory('robot_bringup'),
    'rviz', 'arm620_handeye_calib_orbbec.rviz'
)

# 修改后
default_rviz_config = os.path.join(
    get_package_share_directory('robot_bringup'),
    'rviz', 'arm620_handeye_calib_新相机.rviz'
)
```

#### 修改 3：相机驱动启动（第 67-73 行）

```python
# 修改前
# 6. 启动相机驱动 (Orbbec Astra)
camera_node = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('orbbec_camera'),
                    'launch', 'astra.launch.py')
    )
)

# 修改后
# 6. 启动相机驱动 (新相机)
camera_node = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('你的相机驱动包名'),
                    'launch', '你的相机launch文件.py')
    )
)
```

#### 修改 4（可选）：TF 坐标系名称（第 90 行）

如果你的相机 `camera_link` 名称不同，需要修改：

```python
# 修改前
"--child-frame-id", "camera_link",

# 修改后（如果你的相机用其他名称）
"--child-frame-id", "你的相机_link",
```

---

## 第三步：创建 RViz 配置文件

### 3.1 复制模板

```bash
cd /home/w/work/moveit2_handeye_calib/moveit2_handeye_calib/TG_ARM/robot_bringup/rviz

# 以 Orbbec 版本为模板
cp arm620_handeye_calib_orbbec.rviz arm620_handeye_calib_新相机.rviz
```

### 3.2 修改 RViz 配置

打开新文件，搜索并修改以下 **2 处**：

#### 修改 1：图像话题（搜索 `image_topic`）

```yaml
# 修改前
image_topic: /camera/color/image_raw

# 修改后（根据你相机实际发布的话题）
image_topic: /你的相机/color/image_raw
```

#### 修改 2：传感器坐标系（搜索 `sensor:`）

```yaml
# 修改前
sensor: camera_color_optical_frame

# 修改后（如果你的相机用其他名称）
sensor: 你的相机_color_optical_frame
```

---

## 第四步：测试

### 4.1 重新编译（如果需要）

```bash
cd ~/ros2_ws
colcon build --packages-select robot_bringup
source install/setup.bash
```

### 4.2 启动测试

```bash
ros2 launch robot_bringup arm620_handeye_calib_新相机.launch.py
```

### 4.3 检查清单

- [ ] RViz 正常打开
- [ ] 能看到相机图像（Target Detection 窗口）
- [ ] TF 树中有 `camera_link` 和 `camera_color_optical_frame`
- [ ] HandEye Calibration 面板能检测到标定板

---

## 常见问题

### Q1: RViz 中看不到相机图像

**检查步骤**：

```bash
# 1. 确认图像话题存在
ros2 topic list | grep image

# 2. 确认话题有数据
ros2 topic hz /camera/color/image_raw

# 3. 检查 RViz 配置中的 image_topic 是否正确
```

### Q2: TF 报错找不到 camera_link

**检查步骤**：

```bash
# 1. 查看相机驱动发布的 TF
ros2 run tf2_ros tf2_echo base_link camera_link

# 2. 确认 launch 文件中 --child-frame-id 与相机驱动发布的一致
```

### Q3: 标定板检测不到

**检查步骤**：

1. 确认 `sensor` 参数（光学坐标系）设置正确
2. 确认相机内参话题 (`camera_info`) 正常发布：

```bash
ros2 topic echo /camera/camera_info --once
```

---

## 已支持的相机配置

| 相机 | 驱动包 | Launch 文件 | 图像话题 |
|------|--------|-------------|---------|
| RealSense D435/D435i | `realsense2_camera` | `rs_launch.py` | `/camera/camera/color/image_raw` |
| Orbbec Astra | `orbbec_camera` | `astra.launch.py` | `/camera/color/image_raw` |
| Orbbec Gemini2 | `orbbec_camera` | `gemini2.launch.py` | `/camera/color/image_raw` |

---

## 完整示例：Gemini2 相机

以下是 Gemini2 相机的完整配置示例，供参考：

### Launch 文件关键部分

```python
# 6. 启动相机驱动 (Orbbec Gemini2)
camera_node = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('orbbec_camera'),
                    'launch', 'gemini2.launch.py')
    )
)

# 7. 相机安装位置静态 TF 发布
camera_mount_tf = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="camera_mount_tf",
    arguments=[
        "--x", "0.0",
        "--y", "0.058",
        "--z", "0.02",
        "--frame-id", "Link6",
        "--child-frame-id", "camera_link",
    ],
)
```

### RViz 配置关键部分

```yaml
image_topic: /camera/color/image_raw
sensor: camera_color_optical_frame
```

---

## 总结

| 步骤 | 操作 | 需要修改的内容 |
|------|------|---------------|
| 1 | 获取相机信息 | 记录话题名、坐标系名 |
| 2 | 创建 Launch 文件 | 驱动包名、launch 文件名、rviz 路径 |
| 3 | 创建 RViz 配置 | image_topic、sensor |
| 4 | 测试 | 启动并验证 |

如有问题，可参考现有的 `arm620_handeye_calib_orbbec.launch.py` 和 `arm620_handeye_calib_gemini2.launch.py` 配置。

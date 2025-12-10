# 障碍物配置使用说明

## 功能概述

电子围栏系统现在支持以下类型的障碍物配置：

1. **圆柱体 (Cylinder)** - 适合表示柱子、圆形障碍物
2. **长方体 (Box)** - 适合表示桌子、箱子等方形障碍物
3. **球体 (Sphere)** - 适合表示圆形物体
4. **圆锥体 (Cone)** - 适合表示锥形障碍物
5. **STL网格 (Mesh)** - 支持导入复杂的STL 3D模型文件

## 配置方法

### 1. 基本配置

在YAML配置文件中，通过以下参数启用障碍物功能：

```yaml
virtual_fence_node:
  ros__parameters:
    obstacles:
      enable: true  # 启用障碍物功能
```

### 2. 配置圆柱体

```yaml
obstacles:
  num_cylinders: 2  # 圆柱体数量

  cylinder_0:
    radius: 0.05    # 半径 (米)
    height: 0.3     # 高度 (米)
    x: 0.2          # X坐标
    y: 0.15         # Y坐标
    z: 0.15         # Z坐标 (通常为height/2)

  cylinder_1:
    radius: 0.03
    height: 0.2
    x: 0.4
    y: 0.4
    z: 0.1
```

### 3. 配置长方体

```yaml
obstacles:
  num_boxes: 1

  box_0:
    size_x: 0.1     # X方向尺寸
    size_y: 0.15    # Y方向尺寸
    size_z: 0.08    # Z方向尺寸
    x: 0.25         # X坐标
    y: 0.35         # Y坐标
    z: 0.04         # Z坐标 (通常为size_z/2)
```

### 4. 配置球体

```yaml
obstacles:
  num_spheres: 1

  sphere_0:
    radius: 0.04    # 半径
    x: 0.35         # X坐标
    y: 0.25         # Y坐标
    z: 0.1          # Z坐标
```

### 5. 配置圆锥体

```yaml
obstacles:
  num_cones: 1

  cone_0:
    radius: 0.05    # 底部半径
    height: 0.2     # 高度
    x: 0.15         # X坐标
    y: 0.25         # Y坐标
    z: 0.1          # Z坐标
```

### 6. 配置STL网格模型

```yaml
obstacles:
  num_meshes: 1

  mesh_0:
    file_path: "/path/to/your/model.stl"  # STL文件绝对路径
    x: 0.3          # X坐标
    y: 0.3          # Y坐标
    z: 0.0          # Z坐标
    scale: 0.001    # 缩放比例 (如果STL文件单位是mm，用0.001转换为m)
```

**注意事项：**
- STL文件必须是二进制格式
- `file_path` 必须是绝对路径
- `scale` 参数用于调整模型尺寸，默认为1.0

## 使用示例

### 方法1：使用提供的启动文件

```bash
ros2 launch robot_virtual_fence virtual_fence_with_obstacles.launch.py
```

### 方法2：直接运行节点并指定配置文件

```bash
ros2 run robot_virtual_fence virtual_fence_node --ros-args \
  --params-file /path/to/your/config.yaml
```

### 方法3：通过命令行参数配置

```bash
ros2 run robot_virtual_fence virtual_fence_node --ros-args \
  -p obstacles.enable:=true \
  -p obstacles.num_cylinders:=1 \
  -p obstacles.cylinder_0.radius:=0.05 \
  -p obstacles.cylinder_0.height:=0.3 \
  -p obstacles.cylinder_0.x:=0.2 \
  -p obstacles.cylinder_0.y:=0.2 \
  -p obstacles.cylinder_0.z:=0.15
```

## 完整配置示例

参考文件：`config/obstacles_example.yaml`

该文件包含了所有类型障碍物的配置示例，可以根据实际需求修改。

## 坐标系说明

- 所有坐标均相对于 `frame_id` 参数指定的坐标系（默认为 `base_link`）
- Z坐标建议设置为障碍物高度的一半，使障碍物底部位于地面
- 对于圆柱体和圆锥体，Z坐标表示中心点高度

## 调试技巧

1. 在RViz中查看障碍物：
   - 添加 `PlanningScene` 显示
   - 确保 `Fixed Frame` 设置为 `frame_id` 对应的坐标系

2. 检查障碍物是否正确加载：
   - 查看节点日志输出，会显示每个添加的障碍物信息

3. 如果STL文件加载失败：
   - 检查文件路径是否正确
   - 确认文件格式为二进制STL
   - 检查文件权限

## 性能建议

- 避免添加过多复杂的STL网格，会影响碰撞检测性能
- 优先使用简单几何形状（圆柱、球体、长方体）组合来近似复杂形状
- STL网格的三角面数量建议控制在几千以内

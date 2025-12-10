# 电子围栏障碍物配置快速指南

## 快速开始

### 1. 编译
```bash
cd /home/zwh/work/1.code_git/1112
colcon build --packages-select robot_virtual_fence
source install/setup.bash
```

### 2. 运行示例
```bash
# 使用完整示例配置（包含多种障碍物）
ros2 launch robot_virtual_fence virtual_fence_with_obstacles.launch.py

# 或使用简单测试配置（只有一个圆柱体）
ros2 run robot_virtual_fence virtual_fence_node --ros-args \
  --params-file src/robot_virtual_fence/config/simple_test.yaml
```

## 支持的障碍物类型

| 类型 | 参数 | 说明 |
|-----|------|-----|
| **圆柱体** | radius, height | 适合柱子、管道 |
| **长方体** | size_x, size_y, size_z | 适合桌子、箱子 |
| **球体** | radius | 适合圆形物体 |
| **圆锥体** | radius, height | 适合锥形物体 |
| **STL网格** | file_path, scale | 复杂3D模型 |

## 配置文件位置

- `config/obstacles_example.yaml` - 完整示例，包含所有类型
- `config/simple_test.yaml` - 简单测试，只有一个圆柱体

## 参数说明

### 启用障碍物
```yaml
obstacles:
  enable: true  # 必须设置为true
```

### 圆柱体配置
```yaml
num_cylinders: 1
cylinder_0:
  radius: 0.05   # 半径(米)
  height: 0.3    # 高度(米)
  x: 0.2         # X坐标
  y: 0.2         # Y坐标
  z: 0.15        # Z坐标(通常=height/2)
```

### STL文件配置
```yaml
num_meshes: 1
mesh_0:
  file_path: "/absolute/path/to/model.stl"
  x: 0.3
  y: 0.3
  z: 0.0
  scale: 0.001  # mm转m用0.001
```

## 注意事项

1. **坐标系**: 所有坐标相对于`frame_id`（默认`base_link`）
2. **Z坐标**: 对于对称形状，通常设为高度的一半
3. **STL文件**: 必须是二进制格式的绝对路径
4. **性能**: 避免过多复杂STL网格，优先使用简单几何形状

详细文档请参考：`OBSTACLES_USAGE.md`

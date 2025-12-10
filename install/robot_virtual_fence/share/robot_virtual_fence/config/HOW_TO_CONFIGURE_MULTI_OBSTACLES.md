# 如何一次配置多个障碍物

## 核心步骤

### 1. 设置障碍物数量
在YAML文件中，首先指定每种类型障碍物的数量：

```yaml
obstacles:
  enable: true
  num_cylinders: 3    # 3个圆柱体
  num_boxes: 2        # 2个长方体
  num_spheres: 2      # 2个球体
  num_cones: 1        # 1个圆锥体
  num_meshes: 0       # 0个STL网格
```

### 2. 逐个配置障碍物参数
按照索引（从0开始）配置每个障碍物：

```yaml
# 圆柱体0
cylinder_0:
  radius: 0.05
  height: 0.4
  x: 0.3
  y: 0.3
  z: 0.2

# 圆柱体1
cylinder_1:
  radius: 0.03
  height: 0.3
  x: 0.7
  y: 0.4
  z: 0.15

# 圆柱体2
cylinder_2:
  radius: 0.04
  height: 0.35
  x: 0.5
  y: 0.7
  z: 0.175
```

## 完整示例

我已经创建了一个包含**多个障碍物**的配置文件示例：
- **文件位置**: `config/multi_obstacles.yaml`
- **包含内容**:
  - 3个圆柱体
  - 2个长方体
  - 2个球体
  - 1个圆锥体

## 使用方法

### 方法1: 使用启动文件（推荐）

```bash
source install/setup.bash
ros2 launch robot_virtual_fence multi_obstacles.launch.py
```

### 方法2: 直接运行节点

```bash
source install/setup.bash
ros2 run robot_virtual_fence virtual_fence_node --ros-args \
  --params-file src/robot_virtual_fence/config/multi_obstacles.yaml
```

### 方法3: 使用已安装的配置文件

```bash
source install/setup.bash
ros2 run robot_virtual_fence virtual_fence_node --ros-args \
  --params-file install/robot_virtual_fence/share/robot_virtual_fence/config/multi_obstacles.yaml
```

## 配置规则

### 索引规则
- 索引从 **0** 开始
- 必须连续（不能跳号）
- 例如：如果 `num_cylinders: 3`，必须配置 `cylinder_0`, `cylinder_1`, `cylinder_2`

### 数量匹配
```yaml
# ✅ 正确示例
num_cylinders: 3
cylinder_0: {...}
cylinder_1: {...}
cylinder_2: {...}

# ❌ 错误示例（缺少cylinder_1）
num_cylinders: 3
cylinder_0: {...}
cylinder_2: {...}
cylinder_3: {...}
```

### 不需要的类型设为0
```yaml
# 如果不需要某种类型，直接设为0，不用配置参数
num_spheres: 0
# 不需要配置sphere_0, sphere_1等
```

## 实际场景示例

### 场景1: 工作台环境
```yaml
obstacles:
  enable: true

  # 4个桌腿（圆柱体）
  num_cylinders: 4
  cylinder_0: {radius: 0.025, height: 0.7, x: 0.4, y: 0.3, z: 0.35}
  cylinder_1: {radius: 0.025, height: 0.7, x: 0.6, y: 0.3, z: 0.35}
  cylinder_2: {radius: 0.025, height: 0.7, x: 0.4, y: 0.5, z: 0.35}
  cylinder_3: {radius: 0.025, height: 0.7, x: 0.6, y: 0.5, z: 0.35}

  # 1个桌面（长方体）
  num_boxes: 1
  box_0: {size_x: 0.3, size_y: 0.3, size_z: 0.02, x: 0.5, y: 0.4, z: 0.71}

  num_spheres: 0
  num_cones: 0
  num_meshes: 0
```

### 场景2: 仓库货架
```yaml
obstacles:
  enable: true

  # 3个货架立柱
  num_cylinders: 3
  cylinder_0: {radius: 0.05, height: 1.5, x: 0.3, y: 0.3, z: 0.75}
  cylinder_1: {radius: 0.05, height: 1.5, x: 0.5, y: 0.3, z: 0.75}
  cylinder_2: {radius: 0.05, height: 1.5, x: 0.7, y: 0.3, z: 0.75}

  # 2层货架（长方体）
  num_boxes: 2
  box_0: {size_x: 0.5, size_y: 0.3, size_z: 0.05, x: 0.5, y: 0.3, z: 0.5}
  box_1: {size_x: 0.5, size_y: 0.3, size_z: 0.05, x: 0.5, y: 0.3, z: 1.0}

  num_spheres: 0
  num_cones: 0
  num_meshes: 0
```

## 添加STL模型

如果需要添加复杂的3D模型：

```yaml
num_meshes: 2

mesh_0:
  file_path: "/home/zwh/models/machine_part.stl"
  x: 0.4
  y: 0.4
  z: 0.0
  scale: 0.001  # mm转m

mesh_1:
  file_path: "/home/zwh/models/toolbox.stl"
  x: 0.6
  y: 0.6
  z: 0.0
  scale: 0.001
```

**注意**: STL文件路径必须是绝对路径！

## 调试建议

1. **从少到多**: 先配置1-2个障碍物测试，确认正确后再增加
2. **使用RViz可视化**: 启动后在RViz中添加 `PlanningScene` 显示
3. **查看日志**: 节点会输出每个添加的障碍物信息
4. **逐步调整位置**: 先设置大概位置，然后微调

## 提示

- 所有尺寸单位都是**米(m)**
- 坐标相对于 `frame_id`（默认 `base_link`）
- Z坐标通常设为障碍物高度的一半
- 可以混合使用不同类型的障碍物

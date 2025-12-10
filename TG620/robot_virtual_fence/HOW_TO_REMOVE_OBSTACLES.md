# 如何让障碍物失效

## 方法总览

| 方法 | 说明 | 推荐度 |
|-----|------|-------|
| **ROS服务调用** | 动态移除/启用障碍物 | ⭐⭐⭐⭐⭐ 推荐 |
| **停止节点** | 直接关闭virtual_fence_node | ⭐⭐⭐ |
| **配置文件** | 设置enable: false | ⭐⭐ |

---

## 方法1: 使用ROS服务（推荐）✨

节点提供了3个服务来动态控制障碍物：

### 1.1 移除所有障碍物（包括围栏）

```bash
ros2 service call /remove_all_obstacles std_srvs/srv/Trigger
```

**效果**: 移除所有障碍物和虚拟围栏

### 1.2 只移除自定义障碍物（保留围栏）

```bash
ros2 service call /remove_custom_obstacles std_srvs/srv/Trigger
```

**效果**: 只移除圆柱、长方体、球体、圆锥、STL等自定义障碍物，围栏保留

### 1.3 临时禁用/启用自定义障碍物

```bash
# 禁用自定义障碍物
ros2 service call /enable_obstacles std_srvs/srv/SetBool "{data: false}"

# 重新启用自定义障碍物
ros2 service call /enable_obstacles std_srvs/srv/SetBool "{data: true}"
```

**效果**: 可以动态切换障碍物的启用状态

---

## 方法2: 停止节点

### 2.1 直接停止节点

```bash
# 找到节点进程
ros2 node list | grep virtual_fence

# 停止节点 (Ctrl+C 或 kill)
```

**效果**: 节点停止后，障碍物信息不会主动从规划场景删除，但新的规划会话会丢失这些信息

### 2.2 重启后清除

```bash
# 重启move_group节点可以清除所有规划场景中的障碍物
# 具体命令取决于你的launch文件
```

---

## 方法3: 配置文件

在YAML配置文件中设置：

```yaml
obstacles:
  enable: false  # 禁用障碍物
  num_cylinders: 0
  num_boxes: 0
  num_spheres: 0
  num_cones: 0
  num_meshes: 0
```

然后重启节点：

```bash
ros2 run robot_virtual_fence virtual_fence_node --ros-args \
  --params-file your_config.yaml
```

---

## 实际使用场景示例

### 场景1: 调试时临时禁用障碍物

```bash
# 1. 启动节点
ros2 launch robot_virtual_fence multi_obstacles.launch.py

# 2. 规划运动（会考虑障碍物）
# ...

# 3. 临时禁用障碍物进行测试
ros2 service call /enable_obstacles std_srvs/srv/SetBool "{data: false}"

# 4. 规划运动（不会考虑障碍物）
# ...

# 5. 重新启用障碍物
ros2 service call /enable_obstacles std_srvs/srv/SetBool "{data: true}"
```

### 场景2: 只保留围栏，移除其他障碍物

```bash
# 移除所有自定义障碍物，只保留围栏
ros2 service call /remove_custom_obstacles std_srvs/srv/Trigger
```

### 场景3: 完全清空规划场景

```bash
# 移除所有障碍物和围栏
ros2 service call /remove_all_obstacles std_srvs/srv/Trigger
```

---

## Python脚本示例

如果需要在代码中控制障碍物：

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger, SetBool

class ObstacleController(Node):
    def __init__(self):
        super().__init__('obstacle_controller')

        # 创建服务客户端
        self.remove_all_client = self.create_client(Trigger, 'remove_all_obstacles')
        self.remove_custom_client = self.create_client(Trigger, 'remove_custom_obstacles')
        self.enable_client = self.create_client(SetBool, 'enable_obstacles')

    def remove_all_obstacles(self):
        request = Trigger.Request()
        future = self.remove_all_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def disable_obstacles(self):
        request = SetBool.Request()
        request.data = False
        future = self.enable_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def enable_obstacles(self):
        request = SetBool.Request()
        request.data = True
        future = self.enable_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main():
    rclpy.init()
    controller = ObstacleController()

    # 示例：禁用障碍物
    response = controller.disable_obstacles()
    print(f"Result: {response.message}")

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## C++代码示例

```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>

class ObstacleController
{
public:
    ObstacleController(rclcpp::Node::SharedPtr node) : node_(node)
    {
        remove_all_client_ = node_->create_client<std_srvs::srv::Trigger>("remove_all_obstacles");
        enable_client_ = node_->create_client<std_srvs::srv::SetBool>("enable_obstacles");
    }

    bool removeAllObstacles()
    {
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = remove_all_client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = future.get();
            RCLCPP_INFO(node_->get_logger(), "%s", response->message.c_str());
            return response->success;
        }
        return false;
    }

    bool enableObstacles(bool enable)
    {
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = enable;
        auto future = enable_client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = future.get();
            RCLCPP_INFO(node_->get_logger(), "%s", response->message.c_str());
            return response->success;
        }
        return false;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr remove_all_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr enable_client_;
};
```

---

## 服务说明

| 服务名 | 服务类型 | 参数 | 说明 |
|-------|---------|------|------|
| `/remove_all_obstacles` | `std_srvs/srv/Trigger` | 无 | 移除所有障碍物和围栏 |
| `/remove_custom_obstacles` | `std_srvs/srv/Trigger` | 无 | 只移除自定义障碍物 |
| `/enable_obstacles` | `std_srvs/srv/SetBool` | `data: bool` | 启用/禁用自定义障碍物 |

---

## 注意事项

1. **服务调用要求节点运行中**: 必须确保 `virtual_fence_node` 正在运行
2. **移除是永久的**: 使用 `remove_*` 服务移除后，需要重启节点或调用 `enable_obstacles` 来恢复
3. **围栏和障碍物独立**: `remove_custom_obstacles` 只移除自定义障碍物，围栏保持不变
4. **规划场景共享**: 所有使用MoveIt的节点都会看到这些变化

---

## 快速参考

```bash
# 查看可用服务
ros2 service list | grep obstacles

# 移除所有
ros2 service call /remove_all_obstacles std_srvs/srv/Trigger

# 只移除自定义障碍物
ros2 service call /remove_custom_obstacles std_srvs/srv/Trigger

# 禁用
ros2 service call /enable_obstacles std_srvs/srv/SetBool "{data: false}"

# 启用
ros2 service call /enable_obstacles std_srvs/srv/SetBool "{data: true}"
```

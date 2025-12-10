import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from robot_interfaces.srv import ComputeGravity
from robot_interfaces.msg import ToolConfig
from std_msgs.msg import Float64MultiArray
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
from geometry_msgs.msg import Twist

import numpy as np
import os
import tempfile

class GravityCompensator(Node):
    def __init__(self):
        super().__init__('gravity_compensator')

        # === 从参数服务器读取 robot_description（URDF 字符串）===
        self.declare_parameter('robot_description', '')
        urdf_string = self.get_parameter('robot_description').get_parameter_value().string_value
        
        if not urdf_string:
            self.get_logger().error(" 参数 'robot_description' 未设置！")
            return

        try:
            self.model = pin.buildModelFromXML(urdf_string)
            self.data = self.model.createData()
            
            # 获取关节名称
            self.joint_names = [
                name for idx, name in enumerate(self.model.names)
                if idx > 0 and self.model.joints[idx].nq > 0
            ]
            
            self.get_logger().info(f"成功加载模型，关节数量: {len(self.joint_names)}")

            # 打印Link6的质量信息，用于判断URDF是否包含gripper
            for i in range(self.model.nbodies):
                if 'joint6' in self.model.names[i]:
                    link6_mass = self.model.inertias[i].mass
                    link6_com = self.model.inertias[i].lever
                    self.get_logger().info(f"URDF中Link6质量: {link6_mass:.6f} kg, 质心: {link6_com}")
                    if link6_mass > 0.5:
                        self.get_logger().warn("Link6质量>0.5kg，URDF可能已包含gripper！请检查是否需要外部工具配置")
                    else:
                        self.get_logger().info("Link6质量<0.5kg，URDF不含gripper，需要外部工具配置")
            
        except Exception as e:
            self.get_logger().error(f"模型构建失败: {str(e)}")
            return
        self.q = np.zeros(self.model.nq)
        self.dq_max = 2.0
        self.dq_singularity_threshold = 3.0  # 奇异点检测阈值

        # 工具配置
        self.tool_mass = 0.0
        self.tool_com = np.zeros(3)
        self.tcp_offset_transform = pin.SE3.Identity()

        # === 订阅与发布 ===
        self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.create_subscription(ToolConfig, '/tool_config', self.tool_config_callback, 10)
        self.torque_pub = self.create_publisher(JointState, '/gravity_torque', 10)
        
        self.srv   = self.create_service(
            ComputeGravity,
            'compute_gravity',
            self.handle_compute_gravity
        )

        # 订阅笛卡尔速度
        self.create_subscription(Twist, '/cartesian_velocity_cmd', self.velocity_callback, 10)

        # 发布关节速度
        self.joint_velocity_pub = self.create_publisher(Float64MultiArray, '/joint_velocity', 10)

        self.get_logger().info("Gravity compensator node started with robot_description.")

    def tool_config_callback(self, msg: ToolConfig):
        try:
            self.tool_mass = msg.mass
            self.tool_com = np.array([msg.com.x, msg.com.y, msg.com.z])

            # 从geometry_msgs/Pose转换为Pinocchio SE3
            tcp_pos = np.array([msg.tcp_offset.position.x,
                               msg.tcp_offset.position.y,
                               msg.tcp_offset.position.z])

            # 四元数 -> 旋转矩阵
            quat = pin.Quaternion(msg.tcp_offset.orientation.w,
                                 msg.tcp_offset.orientation.x,
                                 msg.tcp_offset.orientation.y,
                                 msg.tcp_offset.orientation.z)
            tcp_rot = quat.toRotationMatrix()

            self.tcp_offset_transform = pin.SE3(tcp_rot, tcp_pos)

            self.get_logger().info(f"更新工具配置: 质量={self.tool_mass:.3f}kg, 质心=[{self.tool_com[0]:.3f}, {self.tool_com[1]:.3f}, {self.tool_com[2]:.3f}]m")
        except Exception as e:
            self.get_logger().error(f"工具配置回调失败: {e}")

    def compute_tool_gravity_torque(self, q, debug=False):
        """
        通过修改joint6的惯性参数来计算工具重力力矩
        这种方法直接使用Pinocchio的computeGeneralizedGravity，避免了雅可比方法的误差
        """
        try:
            # 找到joint6的索引
            joint6_idx = 6

            # 备份原始惯性
            original_inertia = self.model.inertias[joint6_idx].copy()

            # 计算工具质心相对Link6的位置
            # TCP offset + tool COM (相对TCP)
            tcp_offset = self.tcp_offset_transform.translation
            tcp_rot = self.tcp_offset_transform.rotation

            # 工具COM在TCP坐标系中，需要转换到Link6坐标系
            # tool_com相对Link6 = tcp_offset + tcp_rot * tool_com
            tool_com_rel_link6 = tcp_offset + tcp_rot @ self.tool_com

            # 计算组合惯性参数
            m1 = original_inertia.mass
            com1 = original_inertia.lever
            m2 = self.tool_mass
            com2 = tool_com_rel_link6

            m_total = m1 + m2
            if m_total > 0:
                com_total = (m1 * com1 + m2 * com2) / m_total
            else:
                com_total = com1

            # 创建新的惯性（保持原惯性张量，只修改质量和质心）
            new_inertia = pin.Inertia(
                m_total,
                com_total,
                original_inertia.inertia
            )

            # 临时修改模型
            self.model.inertias[joint6_idx] = new_inertia

            # 计算带工具的重力
            tau_with_tool = pin.computeGeneralizedGravity(self.model, self.data, q)

            # 恢复原始惯性
            self.model.inertias[joint6_idx] = original_inertia

            # 计算不带工具的重力
            tau_without_tool = pin.computeGeneralizedGravity(self.model, self.data, q)

            # 工具贡献 = 带工具 - 不带工具
            tool_torque = tau_with_tool - tau_without_tool

            if debug:
                self.get_logger().info(f"[DEBUG] Joint angles (deg): {np.rad2deg(q)}")
                self.get_logger().info(f"[DEBUG] Tool mass: {self.tool_mass} kg")
                self.get_logger().info(f"[DEBUG] Tool COM (relative to TCP): {self.tool_com}")
                self.get_logger().info(f"[DEBUG] Tool COM (relative to Link6): {tool_com_rel_link6}")
                self.get_logger().info(f"[DEBUG] Original Link6 mass: {m1:.6f} kg, COM: {com1}")
                self.get_logger().info(f"[DEBUG] Combined mass: {m_total:.6f} kg, COM: {com_total}")
                self.get_logger().info(f"[DEBUG] Tool torque: {tool_torque}")

            return tool_torque

        except Exception as e:
            self.get_logger().error(f"计算工具重力力矩失败: {e}")
            return np.zeros(self.model.nq)

    def joint_callback(self, msg: JointState):
        try:
            q = np.zeros(self.model.nq)

            for i, name in enumerate(self.joint_names):
                if name in msg.name:
                    idx = msg.name.index(name)
                    q[i] = msg.position[idx]
            self.q = q
            g = pin.computeGeneralizedGravity(self.model, self.data, q)

            # 计算工具重力的影响
            if self.tool_mass > 0.0:
                tool_torque = self.compute_tool_gravity_torque(q)
                g += tool_torque

            torque_msg = JointState()
            torque_msg.header.stamp = self.get_clock().now().to_msg()
            torque_msg.name = self.joint_names
            torque_msg.effort = g.tolist()
            self.torque_pub.publish(torque_msg)

        except Exception as e:
            self.get_logger().error(f"关节状态处理失败: {e}")
            
    def handle_compute_gravity(self, request, response):
        N  = len(request.joint_trajectory)
        nq = self.model.nq

        q_matrix = np.zeros((N, nq))
        names    = request.joint_trajectory[0].name

        for i, js in enumerate(request.joint_trajectory):
            q_matrix[i, :] = np.array(js.position)

        # 逐点计算
        tau_matrix = np.zeros((N, nq))
        for i in range(N):
            # 计算机械臂本体的重力补偿
            robot_gravity = pin.computeGeneralizedGravity(self.model, self.data, q_matrix[i])
            tau_matrix[i, :] = robot_gravity

            # 加上工具重力的影响
            if self.tool_mass > 0.0:
                tool_torque = self.compute_tool_gravity_torque(q_matrix[i], debug=(i == N - 1))
                tau_matrix[i, :] += tool_torque

                # 打印详细调试信息（仅打印最后一个点，避免过多输出）
                if i == N - 1:
                    self.get_logger().info(
                        f"Robot gravity torque: [{robot_gravity[0]:.4f}, {robot_gravity[1]:.4f}, "
                        f"{robot_gravity[2]:.4f}, {robot_gravity[3]:.4f}, {robot_gravity[4]:.4f}, "
                        f"{robot_gravity[5]:.4f}] Nm"
                    )
                    self.get_logger().info(
                        f"Tool gravity torque:  [{tool_torque[0]:.4f}, {tool_torque[1]:.4f}, "
                        f"{tool_torque[2]:.4f}, {tool_torque[3]:.4f}, {tool_torque[4]:.4f}, "
                        f"{tool_torque[5]:.4f}] Nm"
                    )
                    self.get_logger().info(
                        f"Total gravity torque: [{tau_matrix[i,0]:.4f}, {tau_matrix[i,1]:.4f}, "
                        f"{tau_matrix[i,2]:.4f}, {tau_matrix[i,3]:.4f}, {tau_matrix[i,4]:.4f}, "
                        f"{tau_matrix[i,5]:.4f}] Nm"
                    )
                    self.get_logger().info(
                        f"Tool config: mass={self.tool_mass:.3f} kg, "
                        f"COM=[{self.tool_com[0]:.4f}, {self.tool_com[1]:.4f}, {self.tool_com[2]:.4f}] m"
                    )

        # 返回一批力矩
        for i in range(N):
            eff = JointState()
            eff.name = names
            eff.position = q_matrix[i, :].tolist()
            eff.effort = tau_matrix[i, :].tolist()
            response.efforts.append(eff)
        last_pos = q_matrix[-1, 1]
        last_tau = tau_matrix[-1, 1]
        self.get_logger().info(f"终点第2轴位置: {last_pos:.6f}, 力矩: {last_tau:.6f}")

        self.get_logger().info(f"Batch gravity computed for {N} points")
        return response
    
    def velocity_callback(self, msg: Twist):
        try:
            v_ee = np.array([
                msg.linear.x,
                msg.linear.y,
                msg.linear.z,
                msg.angular.x,
                msg.angular.y,
                msg.angular.z
            ])
            if len(v_ee) != 6:
                self.get_logger().error("输入笛卡尔速度不是六维向量！")
                return

            # 打印当前关节角度
            q_deg = np.rad2deg(self.q)
            self.get_logger().info(
                f"当前关节角度: [{q_deg[0]:.2f}, {q_deg[1]:.2f}, {q_deg[2]:.2f}, "
                f"{q_deg[3]:.2f}, {q_deg[4]:.2f}, {q_deg[5]:.2f}] deg",
                throttle_duration_sec=1.0
            )

            # 打印输入速度
            self.get_logger().info(
                f"输入笛卡尔速度: 线速度=[{v_ee[0]:.4f}, {v_ee[1]:.4f}, {v_ee[2]:.4f}] m/s, "
                f"角速度=[{v_ee[3]:.4f}, {v_ee[4]:.4f}, {v_ee[5]:.4f}] rad/s",
                throttle_duration_sec=1.0
            )

            # 使用Pinocchio计算雅可比矩阵
            ee_frame = self.model.njoints - 1
            J = pin.computeJointJacobian(self.model, self.data, self.q, ee_frame)  # 6xN

            # SVD分解计算可操作度
            s = np.linalg.svd(J, compute_uv=False)  # 只计算奇异值，不需要U和Vt
            manipulability = np.prod(s)  # 可操作度 = 所有奇异值的乘积
            min_singular_value = s[-1]   # 最小奇异值

            # 根据可操作度自适应调整阻尼系数
            lambda_min = 0.001        # 最小阻尼（远离奇异点）
            lambda_max = 0.1          # 最大阻尼（奇异点附近）
            w_threshold = 0.01        # 可操作度阈值

            if manipulability < w_threshold:
                # 接近奇异点，增加阻尼
                lambda_damping = lambda_min + (lambda_max - lambda_min) * \
                               (1.0 - manipulability / w_threshold)
            else:
                # 远离奇异点，使用最小阻尼
                lambda_damping = lambda_min

            # 阻尼最小二乘法求解
            # dq = J^T * (J*J^T + λ^2*I)^(-1) * v
            JJT = J @ J.T
            damped_inverse = np.linalg.inv(JJT + lambda_damping**2 * np.eye(6))
            dq = J.T @ damped_inverse @ v_ee

            # 奇异点检测和警告
            if manipulability < w_threshold:
                self.get_logger().warn(
                    f"接近奇异点！可操作度: {manipulability:.6f}, 最小奇异值: {min_singular_value:.6f}, "
                    f"使用阻尼系数: {lambda_damping:.6f}",
                    throttle_duration_sec=1.0
                )

            # 检查关节速度是否过大
            max_joint_velocity = np.max(np.abs(dq))
            if max_joint_velocity > self.dq_singularity_threshold:
                exceeded_joints = np.where(np.abs(dq) > self.dq_singularity_threshold)[0]
                max_joint_idx = np.argmax(np.abs(dq))
                exceeded_info = ", ".join([f"Joint{j+1}={dq[j]:.3f}" for j in exceeded_joints])

                self.get_logger().warn(
                    f"关节速度过大！Joint{max_joint_idx+1}: {dq[max_joint_idx]:.3f} rad/s "
                    f"(阈值: {self.dq_singularity_threshold:.3f})\n"
                    f"超过阈值的关节: {exceeded_info}\n"
                    f"可操作度: {manipulability:.6f}, 阻尼: {lambda_damping:.6f}",
                    throttle_duration_sec=1.0
                )
                # 发布零速度，停止机器人
                out_msg = Float64MultiArray()
                out_msg.data = [0.0] * self.model.nq
                self.joint_velocity_pub.publish(out_msg)
                return

            # 限幅
            dq = np.clip(dq, -self.dq_max, self.dq_max)

            # 发布关节速度
            out_msg      = Float64MultiArray()
            out_msg.data = dq.tolist()
            self.joint_velocity_pub.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f"计算关节速度失败: {e}")

def main(args=None): 
    rclpy.init(args=args)
    node = GravityCompensator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


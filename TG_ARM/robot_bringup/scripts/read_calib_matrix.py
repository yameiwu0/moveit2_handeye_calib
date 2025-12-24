#!/usr/bin/env python3
"""
读取手眼标定结果并转换为 4x4 齐次变换矩阵
从 TF 树中读取标定后的相机到机械臂末端的变换
"""
import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros import Buffer, TransformListener
import numpy as np
from scipy.spatial.transform import Rotation as R


class CalibMatrixReader(Node):
    def __init__(self):
        super().__init__('calib_matrix_reader')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info('等待 TF 树建立...')

        # 等待一段时间让 TF 树建立
        self.timer = self.create_timer(2.0, self.read_transform)
        self.read_count = 0

    def read_transform(self):
        try:
            # 读取 TF 变换
            trans = self.tf_buffer.lookup_transform(
                'Link6',
                'camera_color_optical_frame',
                rclpy.time.Time()
            )

            # 提取平移和旋转
            t = trans.transform.translation
            q = trans.transform.rotation

            translation = [t.x, t.y, t.z]
            quaternion = [q.x, q.y, q.z, q.w]

            self.get_logger().info('成功读取 TF 变换！')

            # 转换为旋转矩阵
            rot_matrix = R.from_quat(quaternion).as_matrix()

            # 构建 4x4 齐次变换矩阵
            T = np.eye(4)
            T[:3, :3] = rot_matrix
            T[:3, 3] = translation

            # 打印结果
            print("\n" + "="*70)
            print("手眼标定矩阵 (Link6 -> camera_color_optical_frame)")
            print("="*70)
            print("\n平移向量 (Translation):")
            print(f"  x: {translation[0]:.7f} m")
            print(f"  y: {translation[1]:.7f} m")
            print(f"  z: {translation[2]:.7f} m")

            print("\n旋转 (Rotation) - 四元数:")
            print(f"  qx: {quaternion[0]:.7f}")
            print(f"  qy: {quaternion[1]:.7f}")
            print(f"  qz: {quaternion[2]:.7f}")
            print(f"  qw: {quaternion[3]:.7f}")

            # 转换为欧拉角（仅供参考）
            euler = R.from_quat(quaternion).as_euler('xyz', degrees=True)
            print("\n旋转 (Rotation) - 欧拉角 (XYZ, 度):")
            print(f"  roll:  {euler[0]:.4f}°")
            print(f"  pitch: {euler[1]:.4f}°")
            print(f"  yaw:   {euler[2]:.4f}°")

            print("\n4x4 齐次变换矩阵:")
            print(T)

            print("\n" + "="*70)
            print("用于视觉定位的应用示例:")
            print("="*70)
            print("# 将相机坐标系下的点转换到机械臂末端坐标系")
            print("import numpy as np")
            print(f"T = np.array({T.tolist()})")
            print("P_camera = np.array([x, y, z, 1])  # 相机坐标系下的点")
            print("P_link6 = T @ P_camera              # 转换到 Link6 坐标系")
            print("="*70 + "\n")

            # 关闭节点和 ROS
            self.timer.cancel()
            self.destroy_node()
            rclpy.shutdown()

        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.read_count += 1
            if self.read_count == 1:
                self.get_logger().warn(f'等待 TF 变换可用... ({e})')
            elif self.read_count > 10:
                self.get_logger().error('无法读取 TF 变换，请确保：')
                self.get_logger().error('  1. 机械臂和相机节点已启动')
                self.get_logger().error('  2. 手眼标定结果已发布 (运行 bd01.launch.py 或类似启动文件)')
                self.get_logger().error('  3. TF 树中存在 Link6 -> camera_color_optical_frame 的变换')
                self.timer.cancel()
                self.destroy_node()
                rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    print("\n" + "="*70)
    print("手眼标定矩阵读取工具")
    print("="*70)
    print("从 TF 树读取: Link6 -> camera_color_optical_frame")
    print("="*70 + "\n")

    try:
        node = CalibMatrixReader()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n用户中断")
    except Exception as e:
        print(f"\n错误: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

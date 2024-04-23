#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        self.publisher = self.create_publisher(PoseStamped, 'arm_right_cartesian_motion_controller/target_frame', 10)
        self.timer = self.create_timer(1.0, self.publish_pose)
        self.get_logger().info("Pose Publisher has been started.")

    def publish_pose(self):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "base"

        # 设置目标位姿
        pose.pose.position.x = 0.4  # 0.5 米前进
        pose.pose.position.y = -0.2  # 保持不变
        pose.pose.position.z = 0.1 # 0.1 米抬高

        pose.pose.orientation.x = 0.748178  # 无旋转
        pose.pose.orientation.y = 0.607012  # 无旋转
        pose.pose.orientation.z = 0.231413  # 无旋转
        pose.pose.orientation.w = 0.30162  # 无旋转

        self.publisher.publish(pose)
        self.get_logger().info("Published new pose to 'target_frame'")

def main(args=None):
    rclpy.init(args=args)
    node = PosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

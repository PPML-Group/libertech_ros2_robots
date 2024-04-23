#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np
import time
import math
from transformations import quaternion_from_euler, quaternion_from_matrix
from math import sin, cos   
    
class ArcTrajectory:
    def __init__(self):
        pass

    @classmethod
    def __ComputeQuinticCoeffs(cls, t0, t1, theta0, theta1, v0=0.0, v1=0.0, a0=0.0, a1=0.0):
        T = t1 - t0
        h = theta1 - theta0
        k0 = theta0
        k1 = v0
        k2 = 0.5 * a0
        k3 = (20. * h - (8. * v1 + 12. * v0) * T - (3 * a0 - a1) * T**2) / (2. * T**3)
        k4 = (-30. * h + (14 * v1 + 16 * v0) * T + (3 * a0 - 2 * a1) * T**2) / (2. * T**4)
        k5 = (12 * h - 6 * (v1 + v0) * T + (a1 - a0) * T**2) / (2. * T**5)
        return (k0, k1, k2, k3, k4, k5)

    def generate_trajectory(self, t0, t1, theta0, theta1, r, num_points=100):
        coeffs = self.__ComputeQuinticCoeffs(t0, t1, theta0, theta1)
        theta_poly = np.poly1d(coeffs[::-1])
        ts = np.linspace(t0, t1, num_points)
        thetas = theta_poly(ts)
        xs = r * np.cos(thetas)
        ys = r * np.sin(thetas)
        return ts, xs, ys, thetas

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        self.right_publisher = self.create_publisher(PoseStamped, 'arm_right_cartesian_motion_controller/target_frame', 10)
        self.left_publisher = self.create_publisher(PoseStamped, 'arm_left_cartesian_motion_controller/target_frame', 10)
        
        pose_right = PoseStamped()
        pose_right.header.stamp = self.get_clock().now().to_msg()
        pose_right.header.frame_id = "base"
        pose_right.pose.position.x = 0.55
        pose_right.pose.position.y = -0.24
        pose_right.pose.position.z = 0.25     
        # pose_right.pose.orientation.x = 0.430477  # 无旋转
        # pose_right.pose.orientation.y = 0.560972  # 无旋转
        # pose_right.pose.orientation.z = 0.430477  # 无旋转
        # pose_right.pose.orientation.w = 0.560972
        rot_right = np.array([[0,0,1],[1,0,0],[0,1,0]])
        rotation_matrix_4x4 = np.eye(4)
        rotation_matrix_4x4[:3, :3] = rot_right
        quat = quaternion_from_matrix(rotation_matrix_4x4)
        pose_right.pose.orientation.x = quat[0]  # 无旋转
        pose_right.pose.orientation.y = quat[1] # 无旋转
        pose_right.pose.orientation.z = quat[2]  # 无旋转
        pose_right.pose.orientation.w = quat[3]
        
        pose_left = PoseStamped()
        pose_left.header.stamp = self.get_clock().now().to_msg()
        pose_left.header.frame_id = "base"
        pose_left.pose.position.x = 0.55
        pose_left.pose.position.y = 0.24
        pose_left.pose.position.z = 0.25     
        # pose_right.pose.orientation.x = 0.430477  # 无旋转
        # pose_right.pose.orientation.y = 0.560972  # 无旋转
        # pose_right.pose.orientation.z = 0.430477  # 无旋转
        # pose_right.pose.orientation.w = 0.560972
        rot_left = np.array([[0,0,1],[-1,0,0],[0,-1,0]])
        rotation_matrix_4x4 = np.eye(4)
        rotation_matrix_4x4[:3, :3] = rot_left
        quat = quaternion_from_matrix(rotation_matrix_4x4)
        pose_left.pose.orientation.x = quat[0]  # 无旋转
        pose_left.pose.orientation.y = quat[1] # 无旋转
        pose_left.pose.orientation.z = quat[2]  # 无旋转
        pose_left.pose.orientation.w = quat[3]
        
        self.right_publisher.publish(pose_right)
        self.left_publisher.publish(pose_left)
        self.get_logger().info(f"Published trajectory 1 succ")
        time.sleep(5)
        
        ## ############################################################
        t0, t1 = 0, 5.0
        theta0, theta1 = 0, np.pi/3
        r = 0.24
        num_points = 100
        self.arc_planner = ArcTrajectory()
        ts, xs, ys, thetas = self.arc_planner.generate_trajectory(t0, t1,theta0, theta1, r, num_points)
        self.get_logger().info(f"xs:{xs}")
        self.get_logger().info(f"ys:{ys}")
        self.get_logger().info(f"thetas:{thetas}")
        for i in range(len(ts)):
            pose_right = PoseStamped()
            pose_right.header.stamp = self.get_clock().now().to_msg()
            pose_right.header.frame_id = "base"
            pose_right.pose.position.x = 0.50
            pose_right.pose.position.y = -xs[i]
            pose_right.pose.position.z = 0.25+ys[i]  # Keep Z constant
            rot_right = np.array([[0,0,1],[cos(-thetas[i]),sin(-thetas[i]),0],[-sin(-thetas[i]),cos(-thetas[i]),0]])
            # rot_right = np.array([[0,0,1],[-cos(thetas[i]),sin(thetas[i]),0],[-sin(thetas[i]),-cos(thetas[i]),0]])
            # rot_right = np.array([[0,0,1],[1,0,0],[0,1,0]])
            rotation_matrix_4x4_right = np.eye(4)
            rotation_matrix_4x4_right[:3, :3] = rot_right
            quaternion_right = quaternion_from_matrix(rotation_matrix_4x4_right)
            pose_right.pose.orientation.x = quaternion_right[0]  # 无旋转
            pose_right.pose.orientation.y = quaternion_right[1] # 无旋转
            pose_right.pose.orientation.z = quaternion_right[2]  # 无旋转
            pose_right.pose.orientation.w = quaternion_right[3]
            # pose.pose.orientation.x = 0.430477  # 无旋转
            # pose.pose.orientation.y = 0.560972  # 无旋转
            # pose.pose.orientation.z = 0.430477  # 无旋转
            # pose.pose.orientation.w = 0.560972
            
            pose_left = PoseStamped()
            pose_left.header.stamp = self.get_clock().now().to_msg()
            pose_left.header.frame_id = "base"
            pose_left.pose.position.x = 0.50
            pose_left.pose.position.y = xs[i]
            pose_left.pose.position.z = 0.25-ys[i]  # Keep Z constant
            rot_left = np.array([[0,0,1],[-cos(thetas[i]),sin(thetas[i]),0],[-sin(thetas[i]),-cos(thetas[i]),0]])
            # rot_left = np.array([[0,0,1],[cos(thetas[i]),sin(thetas[i]),0],[-sin(thetas[i]),cos(thetas[i]),0]])
            # rot_left = np.array([[0,0,1],[-1,0,0],[0,-1,0]])
            rotation_matrix_4x4_left = np.eye(4)
            rotation_matrix_4x4_left[:3, :3] = rot_left
            quaternion_left = quaternion_from_matrix(rotation_matrix_4x4_left )
            pose_left.pose.orientation.x = quaternion_left[0]  # 无旋转
            pose_left.pose.orientation.y = quaternion_left[1] # 无旋转
            pose_left.pose.orientation.z = quaternion_left[2]  # 无旋转
            pose_left.pose.orientation.w = quaternion_left[3]
            
            self.right_publisher.publish(pose_right)
            self.left_publisher.publish(pose_left)
            
            time.sleep((t1-t0)/num_points)
        self.get_logger().info(f"Published trajectory 2 succ")
            

def main(args=None):
    rclpy.init(args=args)

    node = PosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

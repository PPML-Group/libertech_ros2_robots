#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState, Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from controller_manager_services import *
from aruco_utils import ArucoDetector
from cv_bridge import CvBridge
import cv2
import time
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from model_rtb import ModelRtb
from ament_index_python.packages import get_package_share_directory
import os
from spatialmath import SO3, SE3
import numpy as np
from transformations import quaternion_from_euler, quaternion_from_matrix
from scipy.spatial.transform import Rotation


class GraspCubeAruco(Node):
    def __init__(self):
        super().__init__("grasp_cube_aruco")
        
        self.declare_parameter("arm_controller", f"")
        arm_controller = self.get_parameter("arm_controller").value
        
        self.declare_parameter("hand_controller", f"")
        hand_controller = self.get_parameter("hand_controller").value
        
        publish_topic = "/" + arm_controller + "/target_frame"
        self.get_logger().info('Publishing arm goal on topic "{}"'.format(publish_topic))
        self.arm_pub = self.create_publisher(PoseStamped, publish_topic, 1) 
        
        publish_topic = "/" + hand_controller + "/joint_trajectory"
        self.get_logger().info('Publishing hand goal on topic "{}"'.format(publish_topic))
        self.hand_pub = self.create_publisher(JointTrajectory, publish_topic, 1) 
        
        self.declare_parameter("hand_joints", rclpy.Parameter.Type.STRING_ARRAY)
        self.hand_joints = self.get_parameter("hand_joints").value
        self.get_logger().info('hand_joints "{}"'.format(self.hand_joints))
        
        # self.joint_state_sub = self.create_subscription(
        #     JointState, "joint_states", self.joint_state_callback, 10
        # )
        # self.timer = self.create_timer(5, self.timer_callback)
        
        self.declare_parameter("aruco_id", 0)
        self.aruco_id = self.get_parameter("aruco_id").value
        
        self.camera_info_received = False
        
        # self.declare_parameter("urdf_file", f"")
        # urdf_file = self.get_parameter("urdf_file").value
        # package_share_directory = get_package_share_directory("libertech_humanoid_description")
        # self.urdf_file = os.path.join(package_share_directory, "urdf", urdf_file)
        # # print(self.urdf_file)
        # self.model = ModelRtb(self.urdf_file)
        # print(self.model.n)
        # q = np.zeros(self.model.n)
        # q[12] = -0.5 # camera init
        # self.T_base_cam = self.model.fkine(q,
        #                           start=self.model.link_dict["base"],
        #                           end=self.model.link_dict["head_camera_zedm_optical_frame"]) # SE3
        # print(self.T_base_cam)
        
        
        self.T_base_cam = SE3([0.00,0.00,0.3558+0.05]) * SE3(SO3.RPY(np.array([-np.pi/2-np.pi/12,0,-np.pi/2])))
        # /controller_manager/
        # result = list_controllers(self,"/controller_manager")
        # print("List of controllers:")
        # for controller in result.controller:
        #     print(controller)
            
        # result = switch_controllers(self,"/controller_manager",
        #                             ["arm_trajectory_controller"],
        #                             ["arm_velocity_controller"],
        #                             1,
        #                             True,
        #                             10)
        
        # result = switch_controllers(self,"/controller_manager",
        #                             ["arm_velocity_controller"],
        #                             ["arm_trajectory_controller"],
        #                             1,
        #                             True,
        #                             10)                
        
        # 图像回调，识别二维码        
        self.image_sub = self.create_subscription(
                Image,
                # '/camera/camera/color/image_raw',
                '/camera/color/image_raw',
                self.image_callback,
                2)
        self.camera_info_sub = self.create_subscription(
                CameraInfo,
                '/camera/color/camera_info',
                self.camera_info_callback,
                1)
        self.cv_bridge = CvBridge()
        
        self.move_flag = False
        self.grasp_flag = False

    def camera_info_callback(self, info: CameraInfo): 
        if self.camera_info_received is False:
            self.camera_d = info.d
            self.camera_height = info.height
            self.camera_width = info.width
            self.camera_k = info.k
            self.camera_r = info.r
            self.binning_x = info.binning_x
            self.get_logger().info(f'camera_height: {self.camera_height}\n \
                    camera_width: {self.camera_width}\n \
                    camera_d: {self.camera_d}\n \
                    camera_k: {self.camera_k}\n \
                    camera_r: {self.camera_r}\n \
                    binning_x: {self.binning_x}\n')
            self.camera_info_received = True
            
            self.aruco_detetor = ArucoDetector(self.aruco_id,
                                               'DICT_6X6_1000',
                                               self.camera_k,
                                               self.camera_d,
                                               0.1)
        else:
            return
        
    def image_callback(self, imgmsd):
        if self.camera_info_received is False:
            return
        
        img = self.cv_bridge.imgmsg_to_cv2(imgmsd)
        # opencv 使用BGR
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        ret = self.aruco_detetor.detect_and_estimate_pose(img.copy())
        if ret is not None:
            rvecs, tvecs, image = ret
            cv2.imshow("aruco detection",image)  
            # self.get_logger().info(f'rvecs: {rvecs}')
            # self.get_logger().info(f'tvecs: {tvecs}')
            key = cv2.waitKey(1)
            
            rotation_matrix, _ = cv2.Rodrigues(rvecs[0])
            # self.get_logger().info(f'rotation_matrix: {rotation_matrix}')
            
            rot = SE3(SO3(rotation_matrix))
            # self.get_logger().info(f'rot: {rot}')
            T_cam_obj = SE3(tvecs[0]) * rot
            self.get_logger().info(f'T_cam_obj: {T_cam_obj}')
            
            T_cam_grasp = T_cam_obj * (SE3([0.00,0,0.02]) * SE3(SO3.Eul(np.array([0,np.pi,0]))))
            # self.get_logger().info(f'T_cam_grasp: {T_cam_grasp}')
            # self.get_logger().info(f'T_cam_grasp: {T_cam_grasp.t}')
            # self.get_logger().info(f'T_cam_grasp: {T_cam_grasp.R}')
            T_base_grasp = self.T_base_cam * T_cam_grasp
            self.get_logger().info(f'T_base_grasp: {T_base_grasp}')
            # self.get_logger().info(f'T_cam_grasp.t: {T_base_grasp.x}')
            # self.get_logger().info(f'T_cam_grasp.t: {T_base_grasp.y}')
            # self.get_logger().info(f'T_cam_grasp.t: {T_base_grasp.z}')
            # self.get_logger().info(f'T_cam_grasp.R: {T_base_grasp.R}')
            
            # SE3().
            if self.move_flag is False:
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = "baseout"
                pose.pose.position.x = T_base_grasp.x
                pose.pose.position.y = T_base_grasp.y
                pose.pose.position.z = T_base_grasp.z
                # self.get_logger().info(f'position: {pose.pose.position}')
                # rotation_matrix_4x4 = np.eye(4)
                # rotation_matrix_4x4[:3, :3] = T_base_grasp.R
                # quat = quaternion_from_matrix(rotation_matrix_4x4)
                # self.get_logger().info(f'quat: {quat}')
                r = Rotation.from_matrix(T_base_grasp.R)
                quat = r.as_quat()
                # self.get_logger().info(f'quat: {quat}')
                # self.get_logger().info(f'T_cam_grasp.R: {T_cam_grasp.R}')
                
                pose.pose.orientation.x = quat[0]
                pose.pose.orientation.y = quat[1]
                pose.pose.orientation.z = quat[2]
                pose.pose.orientation.w = quat[3]
                
                self.arm_pub.publish(pose)
                # self.arm_pub.publish(pose)
                # self.arm_pub.publish(pose)
                # self.move_flag = True
                self.get_logger().info(f"Published trajectory 1 succ")
        else:
            cv2.imshow("aruco detection",img)  
            key = cv2.waitKey(1)
                
    def timer_callback(self):
        self.get_logger().info(f"timer for grasp")
        if self.move_flag is True and self.grasp_flag is False:
            self.get_logger().info(f"grasping")
            time.sleep(5)
            
            traj = JointTrajectory()
            traj.joint_names = self.hand_joints
            point = JointTrajectoryPoint()
            pos_a = [0,0.01,0.01,0.01,
                    0,0.01,0.01,0.01,
                    0,0.01,0.01,0.01,
                    0,0.01,0.01,0.01,
                    1.8,0,0.0,0.0]
            for idx, name in enumerate(traj.joint_names):
                pos = (pos_a[idx])
                point.positions.append(pos)

            point.velocities = [0.0] * len(point.positions)
            point.time_from_start = Duration(sec=0)
            traj.points.append(point)
            self.hand_pub.publish(traj)
            time.sleep(2)
            
            traj = JointTrajectory()
            traj.joint_names = self.hand_joints
            point = JointTrajectoryPoint()
            pos_a = [0,1.4,0.8,0.8,
                    0,1.5,0.8,0.8,
                    0,1.0,1.0,1.0,
                    0,0.8,0.8,0.8,
                    1.8,0.0,0.0,0.0]
            for idx, name in enumerate(traj.joint_names):
                pos = (pos_a[idx])
                point.positions.append(pos)

            point.velocities = [0.0] * len(point.positions)
            point.time_from_start = Duration(sec=1)
            traj.points.append(point)
            self.hand_pub.publish(traj)
            
            self.grasp_flag = True

        
def main(args=None):
    rclpy.init(args=args)
    publisher_joint_trajectory = GraspCubeAruco()
    rclpy.spin(publisher_joint_trajectory)
    publisher_joint_trajectory.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

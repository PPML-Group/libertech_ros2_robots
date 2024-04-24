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

class GraspCubeAruco(Node):
    def __init__(self):
        super().__init__("grasp_cube_aruco")
        
        self.declare_parameter("arm_controller", f"")
        arm_controller = self.get_parameter("arm_controller").value
        
        self.declare_parameter("hand_controller", f"")
        hand_controller = self.get_parameter("hand_controller").value
        
        publish_topic = "/" + arm_controller + "/target_frame"
        self.get_logger().info('Publishing arm goal on topic "{}"'.format(publish_topic))
        self.arm_pub = self.create_publisher(JointTrajectory, publish_topic, 1) 
        
        publish_topic = "/" + hand_controller + "/joint_trajectory"
        self.get_logger().info('Publishing hand goal on topic "{}"'.format(publish_topic))
        self.hand_pub = self.create_publisher(JointTrajectory, publish_topic, 1) 
          
        self.declare_parameter("aruco_id", 0)
        self.aruco_id = self.get_parameter("aruco_id").value
        
        
        self.camera_info_received = False
        
        # /controller_manager/
        # result = list_controllers(self,"/controller_manager")
        # print("List of controllers:")
        # for controller in result.controller:
        #     print(controller)
            
        # result = switch_controllers(self,"/controller_manager",
        #                             ["arm_right_trajectory_controller"],
        #                             ["arm_right_velocity_controller"],
        #                             1,
        #                             True,
        #                             10)
        
        # result = switch_controllers(self,"/controller_manager",
        #                             ["arm_right_velocity_controller"],
        #                             ["arm_right_trajectory_controller"],
        #                             1,
        #                             True,
        #                             10)                
        
        # 图像回调，识别二维码        
        self.image_sub = self.create_subscription(
                Image,
                '/zedm/image',
                self.image_callback,
                2)
        self.camera_info_sub = self.create_subscription(
                CameraInfo,
                '/zedm/camera_info',
                self.camera_info_callback,
                1)
        self.cv_bridge = CvBridge()
        
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=10),node=self)
        self.tf_listener = TransformListener(self.tf_buffer, node=self, spin_thread=False)
        transform = self.get_relative_pose()
        print(transform)
    
    def print_all_transforms(self):
        # 获取所有的转换数据
        all_transforms = self.tf_buffer.all_frames_as_yaml()
        self.get_logger().info('All transforms:\n{}'.format(all_transforms))
        
    def get_relative_pose(self):
        max_attempts = 1000
        attempts = 0
        while attempts < max_attempts:
            try:
                transform = self.tf_buffer.lookup_transform('base', 'head_camera_zedm', rclpy.time.Time())
                self.get_logger().info('Relative Pose:\n{}'.format(transform))
                return transform
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().warn('Failed to lookup transform: {}'.format(e))
                attempts += 1
                time.sleep(5)  # 等待一段时间后重试
        
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
        pass
        
        
def main(args=None):
    rclpy.init(args=args)
    publisher_joint_trajectory = GraspCubeAruco()
    rclpy.spin(publisher_joint_trajectory)
    publisher_joint_trajectory.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

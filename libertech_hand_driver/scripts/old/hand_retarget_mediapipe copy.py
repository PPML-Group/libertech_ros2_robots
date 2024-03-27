#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, IntegerRange
from sensor_msgs.msg import JointState

import math
import numpy as np
# from libertech_hand_driver.hand_control import HandAPI
from libertech_hand_driver.hand_model_kinematics import HandModelKinematics, logger
from libertech_hand_driver.hand_model_vision import HandModelVision
import threading
import asyncio
import cv2

CHANGE_THRESHOLD = 0.1

class HandRetarget(Node):
    def __init__(self):
        super().__init__('freehand_ros_retarget_node')
        
        self.declare_parameter('urdf_file', value='')
        self.declare_parameter('action_file', value='')
        self.declare_parameter('ip', value='')
        
        urdf_file = self.get_parameter('urdf_file').value
        action_file = self.get_parameter('action_file').value
        ip = self.get_parameter('ip').value
        
        self.get_logger().info(f'load {urdf_file}')
        
        self.joint_states_pub = self.create_publisher(JointState,"/joint_states",10)
        self.joint_states = JointState()
        self.hand = HandModelKinematics(urdf_file)
        self.hand_vision_tool = HandModelVision(
            urdf_file,
            static_image_mode=False)
        self.joint_states.name = self.hand.qname_
        # self.thread_ = threading.Thread(target=self.run_loop, daemon=True)
        # self.thread_.start()
        
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("无法打开摄像头")
            return
        
        # 只提取5个末端点
        self.ph = None 
        self.ph_last = None
        self.q_retarget = np.zeros(13)
        self.filter_k = 0.5
        self.timer_image = self.create_timer(1./30, self.timer_callback_image)
        self.timer_joint_pub = self.create_timer(0.005, self.timer_callback_joint_pub)
        self.timer_retarget = self.create_timer(0.1, self.timer_callback_retarget)
        
    def timer_callback_image(self):
        # !!!!!!!!!!!!!! 注意： 手必须与相机平行，竖直向上
        ret, img = self.cap.read()
        if ret:
            # cv2.imshow("img", img)
            # img = cv2.flip(img, 1) # 左右手是有问题的mediapipe, 但此处不用翻转？
            hand_positions, img = self.hand_vision_tool.pred_hand_pose_mediapipe(img[...,::-1])
            
            if hand_positions['right']:
                # self.hand_vision_tool.plot_hands(hand_positions)
                rot = np.array([[0,0,-1],[1,0,0],[0,-1,0]])
                hand_positions_rot = self.hand_vision_tool.transform_hand_pose(hand_positions['right'][0],rot)
                ph = self.hand_vision_tool.extract_endpoints(hand_positions_rot)
                if self.ph_last is None:
                    self.ph = ph
                else:
                    # 列表
                    self.ph = [self.filter_k * ph_i + (1 - self.filter_k) * ph_last_i for ph_i, ph_last_i in zip(ph, self.ph_last)]
 
                self.ph_last = self.ph
            else:
                self.ph = None
                self.ph_last = None
                # TODO: 判断跟丢了多久
            
            # self.get_logger().info(f'ph: {self.ph}')
            cv2.imshow("img cap", img[...,::-1])

            cv2.waitKey(1)
    
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.cap.release()
                cv2.destroyAllWindows()
                self.get_logger().info("摄像头已关闭，节点终止")
                rclpy.shutdown()
    
    def timer_callback_retarget(self):
        if self.ph is None:
            self.q_retarget = np.zeros(13)
        else:
            self.q_retarget = self.hand.ik_e2q_retarget(self.ph, q_last=self.q_retarget)
        # self.get_logger().info(f'ph: {self.ph}')
    
    def timer_callback_joint_pub(self):
        # 不断发布 joint state
        position = []
        # for finger, value in self.hand.finger_joint_index.items():
        #     if finger in ["index", "middle"]:
        #         q1 = 0.
        #         q2 = 0.
        #         q3 = 0.
        #         q4 = self.hand.four_bar_pip2dip(q3, finger)
        #         q  = np.array([q1, q2, q3])
        #         position.extend([q1, q2, q3, q4])
        #     elif finger in ["fourth", "little"]:
        #         q1 = 0.
        #         q2 = 0.
        #         q3 = self.hand.four_bar_mcp2pip(q2, finger)
        #         q4 = self.hand.four_bar_pip2dip(q3, finger)
        #         q  = np.array([q1, q2])                    
        #         position.extend([q1, q2, q3, q4])
        #     else:
        #         q1 = 0.
        #         q2 = 0.
        #         q3 = 0.
        #         q4 = self.hand.four_bar_pip2dip(q3, finger)
        #         q  = np.array([q1, q2, q3])
        #         position.extend([q1, q2, q3, q4])
        qa = self.hand.fk_q2q_all(self.q_retarget)
        position = qa.tolist()
        # self.get_logger().info(f'position: {position}')
        # self.get_logger().info(f'{len(position)}')
        # self.get_logger().info(f'name: {self.joint_states.name}')
        self.joint_states.header.stamp = self.get_clock().now().to_msg()
        self.joint_states.header.frame_id = ""
        self.joint_states.position = position
        self.joint_states.velocity = []
        self.joint_states.effort = []
        self.joint_states_pub.publish(self.joint_states)

def main(args=None):
    rclpy.init(args=args)
    node = HandRetarget()
    rclpy.spin(node)

if __name__ == '__main__':
    main()


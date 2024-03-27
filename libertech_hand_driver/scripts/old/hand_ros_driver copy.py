#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, IntegerRange
from sensor_msgs.msg import JointState

import math
import numpy as np
from libertech_hand_driver.hand_control import HandAPI
import threading
import asyncio

CHANGE_THRESHOLD = 0.1

class HandDriver(Node):
    def __init__(self):
        super().__init__('freehand_ros_driver')
        
        self.declare_parameter('urdf_file', value='')
        self.declare_parameter('action_file', value='')
        self.declare_parameter('ip', value='')
        
        urdf_file = self.get_parameter('urdf_file').value
        action_file = self.get_parameter('action_file').value
        ip = self.get_parameter('ip').value
        
        self.get_logger().info(f'load {urdf_file}')
        
        self.joint_states_pub = self.create_publisher(JointState,"/joint_states",10)
        self.joint_states = JointState()
        self.hand = HandAPI(urdf_file, action_file=action_file, target_ip=ip)
        self.joint_states.name = self.hand.hand_model_.qname_
        
        self.joint_name_param = ['f11', 'f12', 'f13', \
            'f21', 'f22', 'f23', \
            'f31', 'f32',  \
            'f41', 'f42',  \
            'f51', 'f52', 'f53']
        # self.joint_name_param_yaw = ['index_q1', 'middle_q1', 'fourth_q1', 'little_q1', 'thumb_q1']
        self.joint_name_param = [f'{self.hand.hand_model_.hand_name_}_' + x for x in self.joint_name_param]
        joint_limit_low = np.trunc(np.degrees(self.hand.hand_model_.qlim_act_[0])).tolist()
        joint_limit_up  = np.trunc(np.degrees(self.hand.hand_model_.qlim_act_[1])).tolist()

        self.get_logger().info(f'joint_states.name: {self.joint_states.name}')
        self.get_logger().info(f'joint_name_param: {self.joint_name_param}')
        self.get_logger().info(f'{joint_limit_low}')
        self.get_logger().info(f'{joint_limit_up}')


        # for i in range(len(self.joint_name_param)):
        #     descriptor =  ParameterDescriptor(description='q (rad)',
        #                                      type = 3)
        #     ran = FloatingPointRange()
        #     ran.from_value = joint_limit_low[i]
        #     ran.to_value = joint_limit_up[i]
                
        #     ran.step = 1
        #     descriptor.floating_point_range = [ran]
        #     self.declare_parameter(self.joint_name_param[i], 0, descriptor)

        for i in range(len(self.joint_name_param)):
            descriptor =  ParameterDescriptor(description='q (deg)',
                                             type = 5)
            ran = IntegerRange()
            ran.from_value = int(joint_limit_low[i])
            ran.to_value = int(joint_limit_up[i])
                
            ran.step = 1
            descriptor.integer_range = [ran]
            self.declare_parameter(self.joint_name_param[i], 0, descriptor)
        # self.hand.add_queue(np.array([0,1,1]), "index")
        # for i in range(5):
        #     self.get_logger().info(f'{self.hand.queue_[i].get()}')
        # return
        
        self.previous_position = {
            "index": np.zeros(3),
            "middle": np.zeros(3),
            "fourth": np.zeros(2),
            "little": np.zeros(2),
            "thumb": np.zeros(3),
        }
        
        self.thread_ = threading.Thread(target=self.run_loop, daemon=True)
        self.thread_.start()
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        
    def run_loop(self):
        asyncio.run(self.hand.run())

    def timer_callback(self):
        # 逻辑，滑动动态调参（满足耦合关系，有点难，毕竟每个都可以拖动）——发送逆解指令——反馈正解（满足耦合关系）——显示rviz
        
        def get_param_rad(name):
            return np.radians(self.get_parameter(name).get_parameter_value().integer_value)
            
        position = []
        for finger, value in self.hand.hand_model_.finger_joint_index.items():
            # num = self.hand.hand_model_.finger_driver_num[finger]
            if finger in ["index", "middle"]:
                q1 = get_param_rad(self.joint_name_param[value])
                q2 = get_param_rad(self.joint_name_param[value+1])
                q3 = get_param_rad(self.joint_name_param[value+2])
                q4 = self.hand.hand_model_.four_bar_pip2dip(q3, finger)
                q  = np.array([q1, q2, q3])
                
                # self.get_logger().info(f'q_now: {q}')
                
                if self.previous_position is not None and \
                    np.linalg.norm(q - self.previous_position[finger]) > CHANGE_THRESHOLD:
                    self.hand.add_queue(q, finger)
                    self.previous_position[finger] = q
                    # self.get_logger().info(f'发送: {q}')

                
                position.extend([q1, q2, q3, q4])
                # self.get_logger().info(f'{finger}: {[q1, q2, q3, q4]}')
            elif finger in ["fourth", "little"]:
                q1 = get_param_rad(self.joint_name_param[value])
                q2 = get_param_rad(self.joint_name_param[value+1])
                q3 = self.hand.hand_model_.four_bar_mcp2pip(q2, finger)
                q4 = self.hand.hand_model_.four_bar_pip2dip(q3, finger)
                q  = np.array([q1, q2])
                
                if self.previous_position is not None and \
                    np.linalg.norm(q - self.previous_position[finger]) > CHANGE_THRESHOLD:
                    self.hand.add_queue(q, finger)
                    self.previous_position[finger] = q
                    
                position.extend([q1, q2, q3, q4])
                # self.get_logger().info(f'{finger}: {[q1, q2, q3, q4]}')
            else:
                q1 = get_param_rad(self.joint_name_param[value])
                q2 = get_param_rad(self.joint_name_param[value+1])
                q3 = get_param_rad(self.joint_name_param[value+2])
                q4 = self.hand.hand_model_.four_bar_pip2dip(q3, finger)
                q  = np.array([q1, q2, q3])
                
                if self.previous_position is not None and \
                    np.linalg.norm(q - self.previous_position[finger]) > CHANGE_THRESHOLD:
                    self.hand.add_queue(q, finger)
                    self.previous_position[finger] = q
                    
                position.extend([q1, q2, q3, q4])
                # self.get_logger().info(f'{finger}: {[q1, q2, q3, q4]}')
                
        # for i in range(5):
        #     self.get_logger().info(f'{self.hand.queue_[i].get()}')

        self.joint_states.header.stamp = self.get_clock().now().to_msg()
        self.joint_states.header.frame_id = ""
        self.joint_states.position = position
        self.joint_states.velocity = []
        self.joint_states.effort = []
        self.joint_states_pub.publish(self.joint_states)


def main(args=None):
    rclpy.init(args=args)
    
    node = HandDriver()
        
    rclpy.spin(node)

if __name__ == '__main__':
    main()
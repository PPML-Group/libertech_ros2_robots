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
from std_msgs.msg import Float64MultiArray

CHANGE_THRESHOLD = 0.1

class HandDriver(Node):
    def __init__(self):
        super().__init__('hand_ros_driver')
        
        self.declare_parameter('urdf_file', value='')
        self.declare_parameter('action_file', value='')
        self.declare_parameter('ip', value='')
        self.declare_parameter('sim', value=True)
        
        urdf_file = self.get_parameter('urdf_file').value
        action_file = self.get_parameter('action_file').value
        ip = self.get_parameter('ip').value
        self.sim = self.get_parameter('sim').value
        
        self.get_logger().info(f'Load {urdf_file}')
        
        self.joint_states_pub = self.create_publisher(
            JointState,
            "/joint_states",
            10)
        self.joint_states = JointState()
        self.hand = HandAPI(
            urdf_file, 
            action_file=action_file, 
            target_ip=ip,
            plan=0
        )
        self.joint_states.name = self.hand.qname
        
        self.cmds_sub = self.create_subscription(
            Float64MultiArray,
            '/hand_joint_cmds',
            self.cmds_callback,
            10)
        
        if self.sim:
            self.get_logger().info(f'Sim...')
        else:
            self.thread_ = threading.Thread(target=self.run_loop, daemon=True)
            self.thread_.start()
            self.get_logger().info(f'Real Hand Start...')

        self.q_last = None
                
    def run_loop(self):
        asyncio.run(self.hand.run())

    def cmds_callback(self, msg):
        q = msg.data
        if len(q) != 13:
            self.get_logger().warn('Received a Float64MultiArray message, but it is not 13-dimensional')
            return
        
        q = np.array(q)

        if self.sim:
            qa = self.hand.hand_model_.fk_q2q_all(q)
        else:
            # if self.q_last is not None:
            #     qe = q - self.q_last
            #     if np.linalg.norm(qe[:3]) > CHANGE_THRESHOLD:
            #         self.hand.add_queue(q[:3], "index")
            #         self.q_last[:3] = q[:3]
            #     if np.linalg.norm(qe[3:6]) > CHANGE_THRESHOLD:
            #         self.hand.add_queue(q[3:6], "middle")
            #         self.q_last[3:6] = q[3:6]
            #     if np.linalg.norm(qe[6:8]) > CHANGE_THRESHOLD:
            #         self.hand.add_queue(q[6:8], "fourth")
            #         self.q_last[6:8] = q[6:8]
            #     if np.linalg.norm(qe[8:10]) > CHANGE_THRESHOLD:
            #         self.hand.add_queue(q[8:10], "little")
            #         self.q_last[8:10] = q[8:10]
            #     if np.linalg.norm(qe[10:]) > CHANGE_THRESHOLD:
            #         self.hand.add_queue(q[10:], "thumb")
            #         self.q_last[10:] = q[10:]     
            # else:
            #     self.hand.add_queue(q[:3], "index")
            #     self.hand.add_queue(q[3:6], "middle")
            #     self.hand.add_queue(q[6:8], "fourth")
            #     self.hand.add_queue(q[8:10], "little")
            #     self.hand.add_queue(q[10:], "thumb")
            #     self.q_last = q
            
            # Define the joint names and their corresponding indices
            joint_names = ["index", "middle", "fourth", "little", "thumb"]
            joint_indices = [(0, 3), (3, 6), (6, 8), (8, 10), (10, None)]
            if self.q_last is not None:
                qe = q - self.q_last
                for name, (start, end) in zip(joint_names, joint_indices):
                    if end is None:
                        end = len(q)

                    if np.linalg.norm(qe[start:end]) > CHANGE_THRESHOLD:
                        self.hand.add_queue(q[start:end], name)
                        self.q_last[start:end] = q[start:end]
            else:
                for name, (start, end) in zip(joint_names, joint_indices):
                    if end is None:
                        end = len(q)
                    self.hand.add_queue(q[start:end], name)
                    self.q_last[start:end] = q[start:end]
            
            # TODO: 从实际电机读取
            # q_m
            qa = self.hand.hand_model_.fk_q2q_all(q)

        position = qa.tolist()   
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


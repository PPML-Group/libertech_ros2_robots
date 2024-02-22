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
        asyncio.run(self.hand.demo())      



def main(args=None):
    rclpy.init(args=args)
    
    node = HandDriver()
        
    rclpy.spin(node)

if __name__ == '__main__':
    main()


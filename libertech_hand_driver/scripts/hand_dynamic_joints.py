#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, IntegerRange
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import math
import numpy as np
from libertech_hand_driver.hand_control import HandAPI

CHANGE_THRESHOLD = 0.1

class HandDynamicJoints(Node):
    def __init__(self):
        super().__init__('hand_dynamic_joints')
        
        self.declare_parameter('urdf_file', value='')
        self.declare_parameter('action_file', value='')

        urdf_file = self.get_parameter('urdf_file').value
        action_file = self.get_parameter('action_file').value
        
        self.get_logger().info(f'Load {urdf_file}')
        
        self.hand = HandAPI(urdf_file, action_file=action_file, display=False)
        
        self.joint_name_param = ['f11', 'f12', 'f13', \
            'f21', 'f22', 'f23', \
            'f31', 'f32',  \
            'f41', 'f42',  \
            'f51', 'f52', 'f53']
        self.joint_name_param = [f'{self.hand.hand_name}_' + x for x in self.joint_name_param]
        joint_limit_low = np.trunc(np.degrees(self.hand.qlim[0])).tolist()
        joint_limit_up  = np.trunc(np.degrees(self.hand.qlim[1])).tolist()

        self.get_logger().info(f'joint_name_param: {self.joint_name_param}')
        self.get_logger().debug(f'{joint_limit_low}')
        self.get_logger().debug(f'{joint_limit_up}')
        
        self.cmds_pub = self.create_publisher(
            Float64MultiArray, 
            'hand_joint_cmds', 
            10)

        # 使用float可能无法滑动到刻度
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
                
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        # 滑动动态调参
        def get_param_rad(name):
            return np.radians(self.get_parameter(name).get_parameter_value().integer_value)
        
        q = [get_param_rad(name) for name in self.joint_name_param]

        msg = Float64MultiArray()
        msg.data = q
        self.cmds_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = HandDynamicJoints()
    rclpy.spin(node)

if __name__ == '__main__':
    main()


#!/usr/bin/env python3
import rclpy

from rclpy.node import Node


class HandDriver(Node):
    def __init__(self):
        super().__init__('freehand_ros_driver')
        pass

def main():
    rclpy.init()

    node = HandDriver()
    
    # Get the parameter
    your_parameter = node.get_parameter('your_parameter').value
    print(your_parameter)
    
    # Use the parameter in your node
    # ...
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

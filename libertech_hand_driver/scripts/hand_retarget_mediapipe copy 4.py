#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import numpy as np
from libertech_hand_driver.hand_model_kinematics import HandModelKinematics, logger
from libertech_hand_driver.hand_model_vision import HandModelVision
import cv2
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import queue


class HandRetargetNode(Node):
    def __init__(self):
        super().__init__('hand_retarget_node')
        
        self.declare_parameter('urdf_file', value='')
        urdf_file = self.get_parameter('urdf_file').value
        
        self.hand = HandModelKinematics(
            urdf_file,
            display=False
        )
        self.hand_vision_tool = HandModelVision(
            urdf_file,
            static_image_mode=False
        )
        
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/color/image_rect_raw/compressed',
            self.timer_callback_image,
            2
        )
        self.cv_bridge = CvBridge()
        
        self.retarget_pub = self.create_publisher(
            Float64MultiArray, 
            '/liberhand/joints_retarget', 
            2
        )
                
        # 只提取5个末端点
        self.ph_last = None
        self.q_retarget = np.zeros(13)
        self.filter_k = 0.7
        self.timer_retarget = self.create_timer(0.001, self.timer_callback_retarget)
                
        self.ph_queue = queue.Queue()
            
    def timer_callback_image(self, msg):
        img = self.cv_bridge.compressed_imgmsg_to_cv2(msg)
        # cv2.imshow("img", img)
        # img = cv2.flip(img, 1) # 左右手是有问题的mediapipe, 但此处不用翻转？
        hand_positions, img = self.hand_vision_tool.pred_hand_pose_mediapipe(img[...,::-1])

        if hand_positions['right']:
            # self.hand_vision_tool.plot_hands(hand_positions)
            rot = np.array([[0,0,-1],[1,0,0],[0,-1,0]])
            hand_positions_rot = self.hand_vision_tool.transform_hand_pose(hand_positions['right'][0],rot)
            ph = self.hand_vision_tool.extract_endpoints(hand_positions_rot)
            if self.ph_last is not None:
                # 列表
                ph = [self.filter_k * ph_i + (1 - self.filter_k) * ph_last_i for ph_i, ph_last_i in zip(ph, self.ph_last)]

            self.ph_queue.put(ph)
            self.ph_last = ph
        else:
            self.ph_last = None
            # TODO: 判断跟丢了多久

        # # self.get_logger().info(f'ph: {self.ph}')
        # cv2.imshow("img cap", img[...,::-1])
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     cv2.destroyAllWindows()
        #     self.get_logger().info("关闭图像，节点终止")
        #     rclpy.shutdown()
    
    def timer_callback_retarget(self):
        if not self.ph_queue.empty():
            self.q_retarget = self.hand.ik_e2q_retarget(self.ph_queue.get(), q_last=self.q_retarget)
            self.get_logger().info(f'q_retarget: {self.q_retarget}')
            msg = Float64MultiArray()
            msg.data = self.q_retarget.tolist()
            self.retarget_pub.publish(msg)
            
        
class HandRetargetPubNode(Node):
    def __init__(self):
        super().__init__('hand_retarget_pub_node')
        self.q_retarget = np.zeros(13)
        self.retarget_sub = self.create_subscription(
            Float64MultiArray, 
            '/liberhand/joints_retarget', 
            self.retarget_callback, 
            2
        )
        
        self.cmds_pub = self.create_publisher(
            Float64MultiArray, 
            '/liberhand/joints_cmd', 
            2
        )
        self.timer_pub = self.create_timer(0.001, self.timer_callback_pub)
        
        self.q_retarget_queue = queue.Queue()
        # self.q_trajectory_queue = queue.Queue()

    def timer_callback_pub(self):
        msg = Float64MultiArray()
        msg.data = self.q_retarget.tolist()
        self.cmds_pub.publish(msg)
        # self.get_logger().info(f'='*10)
        
    def retarget_callback(self, msg: Float64MultiArray):
        self.q_retarget = msg.data  # 更新接收到的数据
        
def main(args=None):
    rclpy.init(args=args)    
    
    retarget_node = HandRetargetNode()
    pub_node = HandRetargetPubNode()

    # 创建两个不同的执行者
    retarget_executor = MultiThreadedExecutor()
    retarget_executor.add_node(retarget_node)
    retarget_executor.add_node(pub_node)

    # pub_executor = MultiThreadedExecutor()
    # pub_executor.add_node(pub_node)

    try:
        # 分别运行两个执行者
        retarget_executor.spin()
        # pub_executor.spin()
    except KeyboardInterrupt:
        pass
    
    

if __name__ == '__main__':
    main()


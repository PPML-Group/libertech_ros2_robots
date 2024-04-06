#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
import numpy as np
from libertech_hand_driver.hand_model_kinematics import HandModelKinematics, logger
from libertech_hand_driver.hand_model_vision import HandModelVision
import cv2
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

class HandRetarget(Node):
    def __init__(self):
        super().__init__('freehand_ros_retarget_node')
        
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
        self.cmds_pub = self.create_publisher(
            Float64MultiArray, 
            'hand_joint_cmds', 
            10)
        
        # 只提取5个末端点
        self.ph = None 
        self.ph_last = None
        self.q_retarget = np.zeros(13)
        self.filter_k = 0.9
        self.timer_retarget = self.create_timer(0.05, self.timer_callback_retarget)
        self.timer_pub = self.create_timer(0.01, self.timer_callback_pub)
        
    def timer_callback_image(self, msg):
        # !!!!!!!!!!!!!! 注意： 手必须与相机平行，竖直向上
        img = self.cv_bridge.compressed_imgmsg_to_cv2(msg)

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
        # cv2.imshow("img cap", img[...,::-1])

        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     cv2.destroyAllWindows()
        #     self.get_logger().info("摄像头已关闭，节点终止")
        #     rclpy.shutdown()
    
    def timer_callback_retarget(self):
        if self.ph is None:
            self.q_retarget = np.zeros(13)
        else:
            self.q_retarget = self.hand.ik_e2q_retarget(self.ph, q_last=self.q_retarget)
        # self.get_logger().info(f'ph: {self.ph}')

    def timer_callback_pub(self):
        msg = Float64MultiArray()
        msg.data = self.q_retarget.tolist()
        self.cmds_pub.publish(msg)
        
        
def main(args=None):
    rclpy.init(args=args)
    node = HandRetarget()
    rclpy.spin(node)

if __name__ == '__main__':
    main()


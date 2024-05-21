#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from message_filters import Subscriber, TimeSynchronizer


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.bridge = CvBridge()
        self.depth_sub = Subscriber(self, Image, '/camera/camera/aligned_depth_to_color/image_raw')
        self.color_sub = Subscriber(self, Image, '/camera/camera/color/image_raw')
        self.ts = TimeSynchronizer([self.depth_sub, self.color_sub], 10)
        self.ts.registerCallback(self.callback)
        self.i = 0

    def callback(self, depth_msg, color_msg):
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg)
        color_image = self.bridge.imgmsg_to_cv2(color_msg)
        # overlay_image = cv2.addWeighted(color_image, 0.7, depth_image, 0.3, 0)
        # cv2.imshow("Overlay Image", overlay_image)
        # cv2.waitKey(1)
        max_depth = depth_image.max()
        min_depth = depth_image.min()
        mean_depth = depth_image.mean()
        # Print max, min, and mean depth values
        self.get_logger().info(f"Max Depth: {max_depth}")
        self.get_logger().info(f"Min Depth: {min_depth}")
        self.get_logger().info(f"Mean Depth: {mean_depth}")
        
        # Convert depth image to pseudo-color for visualization
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # Resize depth image to match the size of the color image
        depth_colormap_resized = cv2.resize(depth_colormap, (color_image.shape[1], color_image.shape[0]))

        # Overlay color image and depth pseudo-color image
        overlay_image = cv2.addWeighted(color_image, 0.7, depth_colormap_resized, 0.3, 0)
        
        concat_image = cv2.hconcat([color_image, depth_colormap_resized, overlay_image])
        concat_image = cv2.resize(concat_image, (0, 0), fx=0.5, fy=0.5)

        cv2.imshow("Overlay Image", concat_image)
        # cv2.imwrite(f"results/align/{self.i:05d}.png", overlay_image)
        cv2.waitKey(1)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

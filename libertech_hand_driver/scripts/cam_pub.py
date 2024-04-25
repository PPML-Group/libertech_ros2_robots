#!/usr/bin/env python3
import cv2
import rclpy
from sensor_msgs.msg import CompressedImage
from rclpy.node import Node

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(CompressedImage, '/camera/color/image_rect_raw/compressed', 10)
        timer_period = 1.0 / 30.0  # 30Hz
        self.timer = self.create_timer(timer_period, self.publish_image)
        
        self.cap = cv2.VideoCapture(0)

    def publish_image(self):
        # Capture image from the webcam
        ret, frame = self.cap.read()
        
        # Compress the image
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
        _, compressed_img = cv2.imencode('.jpg', frame, encode_param)

        # Create a CompressedImage message
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = compressed_img.tobytes()

        # Publish the message
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
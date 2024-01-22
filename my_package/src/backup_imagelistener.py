#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageListener(Node):
    def __init__(self):
        super().__init__('image_listener')
        self.subscription = self.create_subscription(
            Image,
            '/my_camera/image_raw',
            self.image_callback,
            5
        )
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # self.get_logger().info('Received an image!')
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(str(e))
        else:
            # Save your OpenCV2 image as a jpeg 
            cv2.imwrite('camera_image.jpeg', cv2_img)

def main():
    rclpy.init()
    image_listener = ImageListener()
    rclpy.spin(image_listener)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
import sys
import cv2
import numpy as np
import time

class ImageListener(Node):
    def __init__(self):
        super().__init__('image_listener')
        #Retrieve topic argument from launch file
        topic = sys.argv[1]
        self.subscription = self.create_subscription(
            Image,
            topic,
            self.image_callback,
            5
        )
        self.bridge = CvBridge()
        pub_topic = str(topic) + "/offset"
        self.publisher_ = self.create_publisher(String, pub_topic, 5)  

    def image_callback(self, msg):
        pub_str = String()
        centre_offset_x, centre_offset_y = int(sys.argv[2]), int(sys.argv[3])
        threshold_value = 125 

        gray_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        # Get the dimensions (rows, columns)
        rows, cols = gray_image.shape

        #image centre coordinates
        centre_x= int(cols/2+ centre_offset_x)
        centre_y= int(rows/2+centre_offset_y)

        # Apply thresholding to create a binary mask
        _, binary_mask = cv2.threshold(gray_image, threshold_value, 255, cv2.THRESH_BINARY)

        #  Finds contours in binary mask
        contours, hierarchy = cv2.findContours(binary_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # Filter circular contours
        filtered_circles = []
        for contour in contours:
            # Calculate contour area
            area = cv2.contourArea(contour)

            # Approximate the contour to a polygon
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            # Check if the polygon is circular based on the number of vertices and within expected area
            if len(approx) >=6 and area >= 26 and area <= 40:
                filtered_circles.append(contour)

        for filtered_contour in filtered_circles:
            # Calculate the centroid of the circular contour
            M = cv2.moments(filtered_contour)
            # Check if the contour has a valid area (m00 is not zero)
            if M['m00'] != 0:
                # Calculate centroid coordinates
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                pub_str.data = f"Beacon x: {cx} Beacon y: {cy} Centre x: {centre_x} Centre y: {centre_y} X_offset: {cx-centre_x} Y_offset: {cy-centre_y}"
                self.publisher_.publish(pub_str)
     
def main():
    rclpy.init()
    image_listener = ImageListener()
    rclpy.spin(image_listener)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

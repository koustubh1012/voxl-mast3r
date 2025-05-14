#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

class ImageCaptureNode(Node):
    def __init__(self):
        super().__init__('image_capture_node')
        self.subscription = self.create_subscription(
            Image,
            '/hires_small_color',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.image_counter = 1
        self.declare_parameter('image_save_path', '/home/koustubh/ros_ws/src/voxl_mast3r/images/')
        self.image_save_path = self.get_parameter('image_save_path').get_parameter_value().string_value
        self.get_logger().info(f'Image save path: {self.image_save_path}')
        self.image1 = None
        self.image2 = None
        self.first_image_received = False
        self.sift = cv2.SIFT_create()
        self.flann = cv2.FlannBasedMatcher(dict(algorithm=1, trees=5), dict(checks=50))

    def listener_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Check if the image is blurry
        if self.is_blurry(cv_image):
            self.get_logger().info('Image is blurry, skipping...')
            return

        # Check if the first image has been received
        if not self.first_image_received and not cv_image is None:
            self.image1 = cv_image
            self.first_image_received = True
            self.get_logger().info('First image received')
            self.save_image(self.image1, f'first_image_{self.image_counter}.jpg')
            self.image_counter += 1
            cv2.imshow('First Image', self.image1)
            cv2.waitKey(1)
            return
        
        # If the first image has been received, compare with the second image
        if self.first_image_received and not cv_image is None:
            self.image2 = cv_image
            cv2.imshow('Second Image', self.image2)
            cv2.waitKey(1)
            start_time = time.time()
            self.compare_images()
            end_time = time.time()
            elapsed_time = end_time - start_time
            self.get_logger().info(f'Comparison time: {elapsed_time:.2f} seconds')

        
    def is_blurry(self, image, threshold=375):
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # Compute the Laplacian variance
        laplacian_var = cv2.Laplacian(gray, cv2.CV_64F).var()
        # Check if the variance is below the threshold
        return laplacian_var < threshold
    
    def save_image(self, image, image_name):
        # Save the image to the specified path
        cv2.imwrite(f"{self.image_save_path}{image_name}", image)
        self.get_logger().info(f'Saved image: {self.image_save_path}{image_name}')
    
    def compare_images(self):
        # Compare the two images using SIFT
        kp1, des1 = self.sift.detectAndCompute(self.image1, None)
        kp2, des2 = self.sift.detectAndCompute(self.image2, None)
        matches = self.flann.knnMatch(des1, des2, k=2)
        # Apply Lowe's ratio test
        good_matches = []
        for m, n in matches:
            if m.distance < 0.7 * n.distance:
                good_matches.append(m)
        # Display the matches
        if len(good_matches) > 0:
            self.get_logger().info(f'Found {len(good_matches)} good matches')

    
def main(args=None):
    rclpy.init(args=args)
    image_capture_node = ImageCaptureNode()

    try:
        rclpy.spin(image_capture_node)
    except KeyboardInterrupt:
        pass
    finally:
        image_capture_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

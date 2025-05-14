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
        self.orb = cv2.ORB_create()
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

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
            # self.image1 = cv2.imread('/home/koustubh/test/image_3.jpg')
            self.first_image_received = True
            self.get_logger().info('First image received')
            self.save_image(self.image1, f'image_{self.image_counter}.jpg')
            self.image_counter += 1
            return
        
        # If the first image has been received, compare with the second image
        if self.first_image_received and not cv_image is None:
            self.image2 = cv_image
            cv2.imshow('First Image', self.image1)
            cv2.imshow('Second Image', self.image2)
            cv2.waitKey(1)
            # start_time = time.time()
            good_matches = self.compare_images()
            if good_matches < 50:
                self.get_logger().info(f'Found {good_matches} good matches')
                self.save_image(self.image2, f'image_{self.image_counter}.jpg')
                self.image1 = self.image2
                self.image_counter += 1
            # end_time = time.time()
            # elapsed_time = end_time - start_time
            # self.get_logger().info(f'Comparison time: {elapsed_time:.2f} seconds')

        
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
        # Compare the two images using ORB
        kp1, des1 = self.orb.detectAndCompute(self.image1, None)
        kp2, des2 = self.orb.detectAndCompute(self.image2, None)
        # Match descriptors using BFMatcher
        matches = self.bf.match(des1, des2)
        # Sort matches based on distance
        matches = sorted(matches, key=lambda x: x.distance)
        # Define a distance threshold for "good" matches
        GOOD_MATCH_THRESHOLD = 50
        # Filter matches based on the distance threshold
        good_matches = [m for m in matches if m.distance < GOOD_MATCH_THRESHOLD]
        # self.get_logger().info(f'Number of good matches: {len(good_matches)}')
        return len(good_matches)
    
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

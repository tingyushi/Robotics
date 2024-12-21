"""
Save captured images over a certain period of time
"""


import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
import argparse


class Multiple_Image_CameraSubscriber(Node):
    def __init__(self, duration, folder_path):
        super().__init__('camera_subscriber')
        
        self.duration = duration
        self.folder_path = folder_path

        self.subscription = self.create_subscription(
            Image,
            'camera_0',
            self.image_callback,
            10
        )
        self.subscription
        self.bridge = CvBridge()
        self.image_count = 0
        
        # set how many seconds to capture image
        self.timer = self.create_timer(self.duration, self.stop_capture)


    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Save image
            filename = os.path.join(self.folder_path, f'image_{self.image_count:04d}.jpg')
            cv2.imwrite(filename, cv_image)
            self.get_logger().info(f'Saved: {filename}')
            self.image_count += 1

        except Exception as e:
            self.get_logger().error(f'Failed to save image: {e}')


    def stop_capture(self):
        self.get_logger().info('Stopping capture')
        rclpy.shutdown()



def main(args=None):

    parser = argparse.ArgumentParser(description='Capture multiple images from the camera.')
    parser.add_argument('--duration', type=float, default=5.0, help='The duration to capture images')
    input_args = parser.parse_args()

    rclpy.init(args=args)

    camera_subscriber = Multiple_Image_CameraSubscriber(duration=input_args.duration, folder_path="checkerboard_images")

    try:
        rclpy.spin(camera_subscriber)
    except KeyboardInterrupt:
        pass

    camera_subscriber.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
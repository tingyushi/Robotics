"""
Save a single captured image
"""

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import argparse
import os
import time
import itertools



class Single_Image_CameraSubscriber(Node):
    def __init__(self, filename, folder_path):
        super().__init__('camera_subscriber')

        self.filename = filename
        self.folder_path = folder_path

        self.subscription = self.create_subscription(
            Image,
            'camera_0',
            self.image_callback,
            10  
        )
        self.subscription
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Save image
            save_path = os.path.join(self.folder_path, self.filename)
            cv2.imwrite(save_path, cv_image)
            self.get_logger().info(f'Saved: {save_path}')

            # Shut down the node after saving the image
            self.get_logger().info('Image captured')
            rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f'Error saving image: {e}')


def main(filename, folder_path, args=None):

    # parser = argparse.ArgumentParser(description='Capture a single image from the camera.')
    # parser.add_argument('--filename', type=str, default="image.jpg", help='Image File Name')
    # input_args = parser.parse_args()

    rclpy.init(args=args)

    camera_subscriber = Single_Image_CameraSubscriber(filename=filename, folder_path=folder_path)

    try:
        rclpy.spin(camera_subscriber)
    except KeyboardInterrupt:
        pass

    camera_subscriber.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':

    # taking images from different positions and angles for camera calibration

    poss = ['front', 'back', 'left', 'right', 'up', 'down']
    dircs = ['parallel', 'faceleft', 'faceright', 'faceup', 'facedown']
    combinations = list(itertools.product(poss, dircs))
    folder_path = "checkerboard_images"

    for pos, dirc in combinations:

        filename = f"image_{pos}_{dirc}.jpg"
        print(f"Taking {filename}")
        for i in range(5):
            print(f"Wait {i+1} seconds")
            time.sleep(1)
        main(filename, folder_path)

        print() ; print()
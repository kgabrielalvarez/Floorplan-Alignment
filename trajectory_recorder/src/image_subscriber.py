#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
import os


class ImageSub(Node):

    def __init__(self):
        super().__init__('img_subscriber')
        self.subscription1 = self.create_subscription(
            Image,
            '/cam0/image_raw',
            self.listener_callback1,
            10)
        self.subscription1  # prevent unused variable warning

        self.subscription2 = self.create_subscription(Image, 'cam1/image_raw', self.listener_callback2,10)
        self.subscription2

    def listener_callback1(self, msg):
        print("[LOG] Received front fisheye image")
        img_front = np.array(msg.data).reshape(msg.height, msg.width, 3)
        img_front = cv2.flip(img_front,0)
        cv2.imwrite("src/Floorplan-Alignment/trajectory_recorder/images/front_fisheye_img.png", img_front)

    def listener_callback2(self, msg):
        print("[LOG] Received rear fisheye image")
        img_rear = np.array(msg.data).reshape(msg.height, msg.width, 3)
        
        img_rear = cv2.flip(img_rear,0)
        dir_path = os.path.dirname(os.path.realpath(__file__))
        cv2.imwrite("src/Floorplan-Alignment/trajectory_recorder/images/rear_fisheye_img.png", img_rear)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = ImageSub()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
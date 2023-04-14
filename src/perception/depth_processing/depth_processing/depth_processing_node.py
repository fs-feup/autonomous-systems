#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rclpy.duration import Duration
from rclpy.time import Time

from sensor_msgs.msg import Image, RegionOfInterest
from bboxes_ex_msgs.msg import BoundingBoxes, BoundingBox
from cv_bridge import CvBridge

import numpy as np
import cv2

class DepthProcessing(Node):
    def __init__(self):
        super().__init__('depth_processing')

        self.bridge = CvBridge()
        self.subscription_yolov5 = self.create_subscription(
            BoundingBoxes,
            '/yolov5/bounding_boxes',
            self.yolov5_callback,
            qos_profile=qos_profile_sensor_data)
        self.subscription_depth = self.create_subscription(
            Image,
            'zed/depth/image_raw',
            self.depth_callback,
            qos_profile=qos_profile_sensor_data)
        
        self.depth_image = None
        self.bounding_boxes_msgs = []
        
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.process)

    def yolov5_callback(self, msg):
        #print("yolov5 callback")
        self.bounding_boxes_msgs.append(msg)

    def depth_callback(self, msg):
        #print("depth callback")
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

    def process(self):
        if self.depth_image is None or len(self.bounding_boxes_msgs) == 0:
            return

        for bounding_boxes in self.bounding_boxes_msgs:
            for bounding_box in bounding_boxes.bounding_boxes:
                #check if confidence is high enough
                if bounding_box.probability < 0.8:
                    continue
                #get depth from bounding box's center
                x = int(bounding_box.xmin + (bounding_box.xmax - bounding_box.xmin)/2)
                y = int(bounding_box.ymin + (bounding_box.ymax - bounding_box.ymin)/2)
                roi = self.depth_image[y:y+1, x:x+1]
                #print(roi)
                print(bounding_box.class_id + " ID " + str(bounding_box.class_id_int) + ": " + str(np.mean(roi)) + "m")

        print("--------------------")
        self.bounding_boxes_msgs = []
        self.depth_image = None

def main(args=None):
    rclpy.init(args=args)

    node = DepthProcessing()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

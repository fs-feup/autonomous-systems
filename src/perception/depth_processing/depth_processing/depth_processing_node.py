#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import numpy as np
import cv2

from sensor_msgs.msg import CameraInfo, Image
from custom_interfaces.msg import BoundingBoxes, Cone, ConeArray, Point2d
from cv_bridge import CvBridge

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
        
        # self.subscription_pointcloud = self.create_subscription(
        #     PointCloud2,
        #     'zed/points',
        #     self.pointcloud_callback,
        #     qos_profile=qos_profile_sensor_data)

        #subscribe to zed/camera_info to get camera matrix
        self.subcription_camera_info = self.create_subscription(
            CameraInfo,
            'zed/camera_info',
            self.camera_info_callback,
            qos_profile=qos_profile_sensor_data)
        
        self.depth_image = None
        self.bounding_boxes_msgs = []
        #self.pointcloud = None
        self.camera_matrix = None

        self.pub_cone_coordinates = self.create_publisher(ConeArray, 
                                                          'perception/cone_coordinates', 
                                                          10)
        
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.process)

    def yolov5_callback(self, msg):
        self.bounding_boxes_msgs.append(msg)

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

    # def pointcloud_callback(self, msg):
    #     self.pointcloud = msg

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.p).reshape(3, 4)

    def process(self):
        if self.camera_matrix is None or self.depth_image is None or len(self.bounding_boxes_msgs) == 0:  # noqa: E501
                return

        cone_array = ConeArray()
        points = []
        
        imgPts = np.float32([[430, 325], [483, 310], [790, 325], [736, 310]])
        objPts = np.float32([[-2.795184, 6.102409], [-3.088784, 8.926933], [1.836005, 6.092231], [1.77159, 8.985139]]) # noqa: E501

        # Apply perspective transformation function of openCV2.
        # This function will return the matrix which you can feed into warpPerspective 
        # function to get the warped image.
        matrix = cv2.getPerspectiveTransform(imgPts, objPts)
    
        for bounding_boxes in self.bounding_boxes_msgs:
            for bounding_box in bounding_boxes.bounding_boxes:
                #check if confidence is high enough
                if bounding_box.probability < 0.50:
                    continue

                x = int(bounding_box.xmin + (bounding_box.xmax - bounding_box.xmin)/2)
                y = bounding_box.ymax

                #get depth from all points in bounding box that are not inf
                roi = self.depth_image[bounding_box.ymin:bounding_box.ymax, 
                                       bounding_box.xmin:bounding_box.xmax]
                roi = roi[~np.isinf(roi)]

                if len(roi) == 0:
                    continue
            
                bb_point = np.float32([[[x, y]]])
                point = cv2.perspectiveTransform(bb_point, matrix)
                x = float(point[0][0][0])
                y = float(point[0][0][1])
                points.append((point, bounding_box.class_id))

                #publish cone coordinates
                cone = Cone()
                position = Point2d()
                position.x = x
                position.y = y
                cone.position = position
                cone.color = bounding_box.class_id
                cone_array.cone_array.append(cone)

                self.get_logger().info("({},\t{})\td={}m\t{}"
                    .format(x, y, roi.min(), bounding_box.class_id))
                
        self.pub_cone_coordinates.publish(cone_array)
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

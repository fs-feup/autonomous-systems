#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from bboxes_ex_msgs.msg import BoundingBoxes
from cone_coordinates_msg.msg import ConeCoordinates
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge

import numpy as np

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

        self.pub_cone_coordinates = self.create_publisher(ConeCoordinates, 
                                                          'perception/cone_coordinates', 
                                                          10)
        
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.process)

    def yolov5_callback(self, msg):
        #print("yolov5 callback")
        self.bounding_boxes_msgs.append(msg)

    def depth_callback(self, msg):
        #print("depth callback")
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

    # def pointcloud_callback(self, msg):
    #     self.pointcloud = msg

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.p).reshape(3, 4)

    def process(self):
        # if self.depth_image is None or len(self.bounding_boxes_msgs) == 0 
        # or self.pointcloud is None:
        #     return
        
        
        if self.camera_matrix is None or self.depth_image is None or len(self.bounding_boxes_msgs) == 0:  # noqa: E501
                return
    
        for bounding_boxes in self.bounding_boxes_msgs:
            for bounding_box in bounding_boxes.bounding_boxes:
                #check if confidence is high enough
                if bounding_box.probability < 0.70:
                    continue
                #get bounding box's center
                x = int(bounding_box.xmin + (bounding_box.xmax - bounding_box.xmin)/2)
                y = int(bounding_box.ymin + (bounding_box.ymax - bounding_box.ymin)/2)
                # roi = self.depth_image[y-1:y+1, x-1:x+1]

                #get depth from all points in bounding box that are not inf
                roi = self.depth_image[bounding_box.ymin:bounding_box.ymax, 
                                       bounding_box.xmin:bounding_box.xmax]
                roi = roi[~np.isinf(roi)]

                if len(roi) == 0:
                    continue
                
                # Given a 3D point [X Y Z]', the projection (x, y) of the point onto 
                # the rectified image is given by:
                #[u v w]' = P * [X Y Z 1]'
                #x = u / w
                #y = v / w
                #where P is the 3x4 projection matrix, and [u v w]' is the homogeneous 
                # image coordinate of the projected point.
                z = roi.min()
                x = (x - self.camera_matrix[0, 2]) * z / self.camera_matrix[0, 0]
                y = (y - self.camera_matrix[1, 2]) * z / self.camera_matrix[1, 1]
                print("x: " + str(x) + " y: " + str(y) + " z: " + str(z))
                print(bounding_box.class_id + " ID " + str(bounding_box.class_id_int) 
                      + ": " + str(roi.min()) + "m")

                #publish cone coordinates
                cone_coordinates = ConeCoordinates()
                cone_coordinates.header = bounding_boxes.header
                cone_coordinates.x = x
                cone_coordinates.y = y
                cone_coordinates.class_id = bounding_box.class_id
                cone_coordinates.class_id_int = bounding_box.class_id_int
                self.pub_cone_coordinates.publish(cone_coordinates)
                
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

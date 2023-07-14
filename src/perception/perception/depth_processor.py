#!/usr/bin/env python3

import numpy as np
import cv2

from custom_interfaces.msg import Cone, ConeArray, Point2d
from cv_bridge import CvBridge


class DepthProcessor():
    def __init__(self, logger):
        self.logger = logger
        self.bridge = CvBridge()

        # Calculate stereo depth
        self.image_left=[]
        self.image_right=[]
        self.depthmap = []

    def process(self, msg, img):
        cone_array = ConeArray()
        points = []
        
        imgPts = np.float32([
            [430, 325],
            [483, 310],
            [790, 325],
            [736, 310]
        ])
        objPts = np.float32([
            [-2.795184, 6.102409],
            [-3.088784, 8.926933],
            [1.836005, 6.092231],
            [1.77159, 8.985139]
        ])

        # Apply perspective transformation function of openCV2.
        # This function will return the matrix which you can feed into warpPerspective 
        # function to get the warped image.
        matrix = cv2.getPerspectiveTransform(imgPts, objPts)
    
        for bounding_box in msg.bounding_boxes:
            #check if confidence is high enough
            if bounding_box.probability < 0.50:
                continue

            x = int(bounding_box.xmin + (bounding_box.xmax - bounding_box.xmin)/2)
            y = bounding_box.ymax

            #get depth from all points in bounding box that are not inf
            roi = img[bounding_box.ymin:bounding_box.ymax, 
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

            # rotate coordinates -90 degrees so they match localisation logic
            position.x = y
            position.y = x
            cone.position = position
            cone.color = bounding_box.class_id
            cone_array.cone_array.append(cone)

            self.logger.info("({},\t{})\t{}"
                .format(position.x, position.y, bounding_box.class_id))

        self.bounding_boxes_msgs = []
        self.image = None
        return cone_array

    def recv_stereo_img(self, image, side):
        if side == 1:
            self.image_left = self.bridge.imgmsg_to_cv2(image, "bgr8")
            if self.image_right != []:
                self.calculate_depthmap()

        if side == 0:
            self.image_right = self.bridge.imgmsg_to_cv2(image, "bgr8")
            if self.image_left != []:
                self.calculate_depthmap()


    def calculate_depthmap(self):
        # Convert images to gray scale
        left_gray = cv2.cvtColor(self.image_left, cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(self.image_right, cv2.COLOR_BGR2GRAY)

        # Set up stereo parameters
        stereo = cv2.StereoBM_create(numDisparities=64, blockSize=25)

        # Compute disparity map - 1e4 offset to protect divs by 0
        disparity_map = stereo.compute(left_gray, right_gray) + 0.0001

        # Convert disparity map to depth map
        # focal length (in pixels), B = baseline (distance between the cameras), p = pixel size in real world coords
        f = 700  # 700 for zed or zed mini
        B = 0.120  # m
        p = 0.004 # mm
        
        self.depth_map = (f) * (B / disparity_map)

        # Plot depth map
        cv2.imshow('Depth Map', self.depth_map)
        cv2.waitKey(0)  
        cv2.destroyAllWindows() 

#!/usr/bin/env python3

import numpy as np
import cv2

from custom_interfaces.msg import Cone, ConeArray, Point2d
from cv_bridge import CvBridge

class DepthProcessor():
    def __init__(self, logger):
        self.logger = logger
        self.bridge = CvBridge()

        self.imgPts = np.float32([
            [430, 325],
            [483, 310],
            [790, 325],
            [736, 310]
        ])
        self.objPts = np.float32([
            [-2.795184, 6.102409],
            [-3.088784, 8.926933],
            [1.836005, 6.092231],
            [1.77159, 8.985139]
        ])

    def process(self, msg, img, point_cloud=None):
        cone_array = ConeArray()
        points = []

        # Apply perspective transformation function of openCV2.
        # This function will return the matrix which you can feed into warpPerspective 
        # function to get the warped image.
        matrix = cv2.getPerspectiveTransform(self.imgPts, self.objPts)
    
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
        
            if point_cloud != None:
                point3D = point_cloud.get_value(x, y)
                x = point3D[1][0]
                y = point3D[1][2]
            else:
                bb_point = np.float32([[[x, y]]])
                point = cv2.perspectiveTransform(bb_point, matrix)
                x = float(point[0][0][0])
                y = float(point[0][0][1])

            #publish cone coordinates
            cone = Cone()
            position = Point2d()

            # rotate coordinates 90 degrees so they match localization logic
            position.x = float(y)
            position.y = float(-x)
            cone.position = position
            cone.color = bounding_box.class_id
            cone_array.cone_array.append(cone)

            self.logger.debug("({},{})\t{}"
                .format(position.x, position.y, bounding_box.class_id))

        self.bounding_boxes_msgs = []
        self.image = None
        return cone_array

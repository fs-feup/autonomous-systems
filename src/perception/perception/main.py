#ADAPTED FROM https://github.com/Ar-Ray-code/YOLOv5-ROS

import os
import sys
from pathlib import Path

import cv2
import numpy as np

import torch
import torch.backends.cudnn as cudnn

from .utils.datasets import letterbox
from .utils.general import (check_img_size, check_imshow, LOGGER,
                                      non_max_suppression, scale_coords, xyxy2xywh)
from .utils.plots import Annotator, colors
from perception.utils.torch_utils import time_sync

from .depth_processor import DepthProcessor
from .adapter import PerceptionAdapter

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from custom_interfaces.msg import BoundingBoxes, BoundingBox, ConeArray
from std_msgs.msg import Header
from cv_bridge import CvBridge

class yolov5():
    def __init__(self,  weights,
                        imagez_height,
                        imagez_width,
                        conf_thres,
                        iou_thres,
                        max_det,
                        view_img,
                        classes,
                        agnostic_nms,
                        line_thickness,
                        half,
                        dnn
                        ):
        self.weights = weights
        self.imagez_height = imagez_height
        self.imagez_width = imagez_width
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres
        self.max_det = max_det
        self.view_img = view_img
        self.classes = classes
        self.agnostic_nms = agnostic_nms
        self.line_thickness = line_thickness
        self.half = half
        self.dnn = dnn

        self.s = str()

        self.load_model()

    def load_model(self):
        imgsz = (self.imagez_height, self.imagez_width)

        # Load model
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = torch.hub.load('ultralytics/yolov5','custom', path=self.weights)
        stride, self.names, pt = self.model.stride, self.model.names, self.model.pt
        imgsz = check_img_size(imgsz, s=stride)  # check image size

        # Half
        # FP16 supported on limited backends with CUDA
        self.half &= (pt) and self.device.type != 'cpu'  
        if pt:
            self.model.model.half() if self.half else self.model.model.float()

        # Dataloader
        webcam = True
        if webcam:
            check_imshow()
            cudnn.benchmark = True
        bs = 1
        self.vid_path, self.vid_writer = [None] * bs, [None] * bs

        self.dt, self.seen = [0.0, 0.0, 0.0], 0

    # return ---------------------------------------
    # 1. class (str)                                +
    # 2. confidence (float)                         +
    # 3. x_min, y_min, x_max, y_max (float)         +
    # ----------------------------------------------
    def image_callback(self, image_raw):
        class_list = []
        confidence_list = []
        x_min_list = []
        y_min_list = []
        x_max_list = []
        y_max_list = []

        # im is  NDArray[_SCT@ascontiguousarray
        # im = im.transpose(2, 0, 1)
        self.stride = 32  # stride
        self.img_size = 640
        img = letterbox(image_raw, self.img_size, stride=self.stride)[0]

        # Convert
        img = img.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
        im = np.ascontiguousarray(img)

        t1 = time_sync()
        im = torch.from_numpy(im).to(self.device)
        im = im.half() if self.half else im.float()  # uint8 to fp16/32
        im /= 255  # 0 - 255 to 0.0 - 1.0
        if len(im.shape) == 3:
            im = im[None]  # expand for batch dim
        t2 = time_sync()
        self.dt[0] += t2 - t1

        # Inference
        pred = self.model(im, augment=False)
        t3 = time_sync()
        self.dt[1] += t3 - t2

        # NMS
        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, self.classes, 
                                   self.agnostic_nms, max_det=self.max_det)
        self.dt[2] += time_sync() - t3

        # Process predictions
        for i, det in enumerate(pred):
            im0 = image_raw
            self.s += f'{i}: '

            # p = Path(str(p))  # to Path
            self.s += '%gx%g ' % im.shape[2:]  # print string
            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
            annotator = Annotator(im0, line_width=self.line_thickness, 
                                  example=str(self.names))
            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(im.shape[2:], det[:, :4], im0.shape).round()

                # Print results
                for c in det[:, -1].unique():
                    # detections per class
                    n = (det[:, -1] == c).sum()
                    # add to string
                    self.s += f"{n} {self.names[int(c)]}{'s' * (n > 1)}, "  

                for *xyxy, conf, cls in reversed(det):
                    # normalized xywh
                    (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  
                    
                    # Add bbox to image
                    c = int(cls)  # integer class
                    label = f'{self.names[c]} {conf:.2f}'
                    annotator.box_label(xyxy, label, color=colors(c, True))

                    # print(xyxy, label)
                    class_list.append(self.names[c])
                    confidence_list.append(conf)
                    # tensor to float
                    x_min_list.append(xyxy[0].item())
                    y_min_list.append(xyxy[1].item())
                    x_max_list.append(xyxy[2].item())
                    y_max_list.append(xyxy[3].item())

            # Stream results
            im0 = annotator.result()
            if self.view_img:
                cv2.imshow("yolov5", im0)
                cv2.waitKey(1)  # 1 millisecond

            return class_list, confidence_list, x_min_list, y_min_list, x_max_list, y_max_list  # noqa: E501

class perception(Node):
    def __init__(self):
        super().__init__('perception')

        self.bridge = CvBridge()

        self.adapter = PerceptionAdapter("eufs", self) # 3rd arg as true to get stereo cam depth map
        self.pub_cone_coordinates = self.create_publisher(ConeArray, 
                                                          'perception/cone_coordinates', 
                                                          10)

        # parameter
        FILE = Path(__file__).resolve()
        ROOT = FILE.parents[0]
        if str(ROOT) not in sys.path:
            sys.path.append(str(ROOT))  # add ROOT to PATH
        ROOT = Path(os.path.relpath(ROOT, Path.cwd()))

        self.weights = str(ROOT) + '/config/best_cones.pt'
        self.imagez_height = 640
        self.imagez_width = 640
        self.conf_thres = 0.25
        self.iou_thres = 0.45
        self.max_det = 1000
        self.view_img = False
        self.classes = None
        self.agnostic_nms = False
        self.line_thickness = 2
        self.half = False
        self.dnn = False

        self.yolov5 = yolov5(self.weights,
                                self.imagez_height,
                                self.imagez_width,
                                self.conf_thres,
                                self.iou_thres,
                                self.max_det,
                                self.view_img,
                                self.classes,
                                self.agnostic_nms,
                                self.line_thickness,
                                self.half,
                                self.dnn)

        self.depth_processor = DepthProcessor(LOGGER)
    
    def yolovFive2bboxes_msgs(self, bboxes:list, scores:list, cls:list, 
                              img_header:Header):
        bboxes_msg = BoundingBoxes()
        bboxes_msg.header = img_header
        i = 0
        for score in scores:
            one_box = BoundingBox()
            one_box.xmin = int(bboxes[0][i])
            one_box.ymin = int(bboxes[1][i])
            one_box.xmax = int(bboxes[2][i])
            one_box.ymax = int(bboxes[3][i])
            one_box.probability = float(score)
            one_box.class_id = cls[i]
            one_box.class_id_int = i
            bboxes_msg.bounding_boxes.append(one_box)
            i = i+1
        
        return bboxes_msg

    def image_callback(self, image:Image):
        image_raw = self.bridge.imgmsg_to_cv2(image, "bgr8")
        class_list, confidence_list,\
        x_min_list, y_min_list, x_max_list, y_max_list =\
            self.yolov5.image_callback(image_raw)
        
        msg = self.yolovFive2bboxes_msgs(bboxes=[x_min_list, y_min_list, 
                                                 x_max_list, y_max_list], 
                                                 scores=confidence_list, 
                                                 cls=class_list, 
                                                 img_header=image.header)
        
        cone_array = self.depth_processor.process(msg, image_raw)
        self.pub_cone_coordinates.publish(cone_array)
        LOGGER.info("")

    def left_callback(self, image:Image):
        self.depth_processor.recv_stereo_img(image, 1) # 1 for left

    def right_callback(self, image:Image):
        self.depth_processor.recv_stereo_img(image, 0) # 0 for right

def ros_main(args=None):
    rclpy.init(args=args)
    perception_node = perception()
    rclpy.spin(perception_node)
    perception_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    ros_main()
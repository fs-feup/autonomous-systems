import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile


class ObjectDetector(Node):
    def __init__(self):
        super().__init__("object_detector")
        self.subscription = self.create_subscription(
            Image,
            "/zed/left/image_rect_color",
            self.callback,
             QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                depth=5)
        )
        yolo_config_path = '/home/diogo/FS/driverless/src/perception/object_detector/object_detector/yolov3.cfg'
        yolo_weights_path = '/home/diogo/FS/driverless/src/perception/object_detector/object_detector/yolov3.weights'
        yolo_classes_path = '/home/diogo/FS/driverless/src/perception/object_detector/object_detector/custom_classes.txt'
        self.cv_bridge = CvBridge()
        self.net = cv2.dnn.readNetFromDarknet(yolo_config_path, yolo_weights_path)
        #self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
        #self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
        self.labels = open(yolo_classes_path).read().strip().split("\n")

    def callback(self, msg):
        yolo_confidence_threshold = 0.5
        yolo_nms_threshold = 0.4
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        blob = cv2.dnn.blobFromImage(cv_image, 1 / 255.0, (416, 416), swapRB=True, crop=False)
        self.net.setInput(blob)
        layer_names = self.net.getLayerNames()
        output_layers = [layer_names[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]
        outputs = self.net.forward(output_layers)
        boxes = []
        confidences = []
        class_ids = []
        width = cv_image.shape[1]
        height = cv_image.shape[0]
        for output in outputs:
            for detection in output:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > yolo_confidence_threshold:
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)
                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)
        indices = cv2.dnn.NMSBoxes(boxes, confidences, yolo_confidence_threshold, yolo_nms_threshold)
        for i in indices:
            i = i[0]
            box = boxes[i]
            x = box[0]
            y = box[1]
            w = box[2]
            h = box[3]
            label = self.labels[class_ids[i]]
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(cv_image, label, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.imshow("Object detection", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    detector = ObjectDetector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
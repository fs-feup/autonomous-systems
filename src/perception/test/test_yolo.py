import cv2 as cv
import os
import unittest
import rclpy
from perception.main import perception
import time

# It is necessary to keep this function. It will be overrided 
original_imread = cv.imread 


class TestPerceptionNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init(args=None)

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = perception()

    def test_yolo(self):

        file = open('../performance/exec_time/perception.csv', 'a')
        
        if (os.stat("../performance/exec_time/perception.csv").st_size == 0):
            file.write("Module,Scope,Scenario,Execution Time (ms)\n")

        dir = os.path.dirname(os.path.abspath(__file__))

        for filename in os.listdir(dir + '/images'):
            image = original_imread(dir + '/images/' + filename)

            sumTime = 0.0
            n_times = 10
            for i in range (n_times):
                start_time = time.time()
                self.node.yolov5.image_callback(image)
                end_time = time.time()
                sumTime += (end_time - start_time) * 1000

            averageTime = sumTime / n_times

            file.write(f"Perception,Yolov5,{filename},{averageTime}\n")

        file.close()
        self.node.destroy_node()

if __name__ == '__main__':
    unittest.main()

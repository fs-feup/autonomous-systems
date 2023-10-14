import cv2 as cv
import os
original_imread = cv.imread # It is necessary to keep this function. It will be overrided 
import unittest
import rclpy
from rclpy.node import Node
from custom_interfaces.msg import ConeArray
from perception.main import perception
import numpy as np
import base64
import time


class TestPerceptionNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init(args=None)

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = perception()

    def test_wholeNode(self):

        file = open('test/output/output.csv', 'a')

        if (os.stat("test/output/output.csv").st_size == 0):
            file.write("Module,Scope,Scenario,Execution Time (ms)\n")

        dir = os.path.dirname(os.path.abspath(__file__))

        for filename in os.listdir(dir + '/images'):
            image = original_imread(dir + '/images/' + filename)

            sumTime = 0.0

            n_times = 10
            for i in range (n_times):
                start_time = time.time()
                cones = self.node.image_callback(image, False)
                end_time = time.time()
                sumTime += (end_time - start_time) * 1000

            averageTime = sumTime / n_times

            file.write(f"Perception,All,{filename}: {len(cones.cone_array)} cones,{averageTime}\n")

        file.close()
        self.node.destroy_node()

if __name__ == '__main__':
    unittest.main()

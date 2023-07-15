from enum import Enum
from eufs_msgs.msg import CanState

import math
import numpy as np

from .config import Params

class DDTStateA(Enum):
    START = 0
    TURNING_LEFT = 1
    TURNING_RIGHT = 2
    TURNING_CENTER = 3
    RPM200 = 4
    STOP = 5
    FINISH = 6

class DDTStateB(Enum):
    START = 0
    RPM50 = 1
    EBS = 2
    FINISH = 3

P = Params()
A_STATE = DDTStateA.START
B_STATE = DDTStateB.START

def rpm_to_velocity(rpm):
    return rpm * P.tire_diam * math.pi / 60

def ddt_inspection_a(node):
    global P
    global A_STATE

    if (A_STATE == DDTStateA.START):
        node.adapter.publish_cmd(steering_angle=P.MAX_STEER)
        A_STATE = DDTStateA.TURNING_LEFT

    elif (A_STATE == DDTStateA.TURNING_LEFT):
        if (node.steering_angle_actual >= P.MAX_STEER):
            node.adapter.publish_cmd(steering_angle=-P.MAX_STEER)
            A_STATE = DDTStateA.TURNING_RIGHT

    elif (A_STATE == DDTStateA.TURNING_RIGHT):
        if (node.steering_angle_actual <= P.MAX_STEER):
            node.adapter.publish_cmd(steering_angle=0)
            A_STATE = DDTStateA.TURNING_CENTER

    elif (A_STATE == DDTStateA.TURNING_CENTER):
        if (node.steering_angle_actual == 0):
            node.adapter.publish_cmd(speed=rpm_to_velocity(200))
            A_STATE = DDTStateA.RPM200

    elif (A_STATE == DDTStateA.RPM200):
        if (node.velocity_actual >= rpm_to_velocity(200)):
            node.adapter.publish_cmd()
            A_STATE = DDTStateA.FINISH

    elif (A_STATE == DDTStateA.STOP):
        if (node.velocity_actual == 0):
            node.adapter.set_mission_state(
                CanState.AMI_DDT_INSPECTION_A, CanState.AS_FINISHED)

def ddt_inspection_b(node):
    global P
    global B_STATE

    if (B_STATE == DDTStateB.START):
        node.adapter.publish_cmd(speed=rpm_to_velocity(50))
        B_STATE = DDTStateB.RPM50

    elif (B_STATE == DDTStateB.RPM50):
        if (node.velocity_actual >= rpm_to_velocity(50)):
            node.adapter.ebs()
            B_STATE = DDTStateB.EBS

    elif (B_STATE == DDTStateB.EBS):
        if (node.velocity_actual == 0):
            B_STATE = DDTStateB.FINISH

def autonomous_demo(node):
    return
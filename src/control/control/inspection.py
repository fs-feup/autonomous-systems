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

class AutonomousState(Enum):
    START = 0
    TURNING_LEFT = 1
    TURNING_RIGHT = 2
    TURNING_CENTER = 3
    KMH15_M10_1 = 4
    BRAKING = 5
    KMH15_M10_2 = 6
    EBS = 7
    FINISH = 8

P = Params()
A_STATE = DDTStateA.START
B_STATE = DDTStateB.START
AUTONOMOUS_STATE = AutonomousState.START
START_POINT = np.array([0, 0])

def rpm_to_velocity(rpm):
    return rpm * P.tire_diam * math.pi / 60

def kmh_to_ms(kmh):
    return kmh / 3.6

def ddt_inspection_a(node):
    global P
    global A_STATE

    node.get_logger().debug("State: {} - Steering angle actual: {} - Velocity actual: {} - Max steer: {}"
        .format(A_STATE, node.steering_angle_actual, node.velocity_actual, P.MAX_STEER))

    if (A_STATE == DDTStateA.START):
        node.adapter.publish_cmd(steering_angle=P.MAX_STEER)
        A_STATE = DDTStateA.TURNING_LEFT

    elif (A_STATE == DDTStateA.TURNING_LEFT):
        if (node.steering_angle_actual >= P.MAX_STEER):
            node.adapter.publish_cmd(steering_angle=-P.MAX_STEER)
            A_STATE = DDTStateA.TURNING_RIGHT

    elif (A_STATE == DDTStateA.TURNING_RIGHT):
        if (node.steering_angle_actual <= -P.MAX_STEER):
            node.adapter.publish_cmd(steering_angle=0.)
            A_STATE = DDTStateA.TURNING_CENTER

    elif (A_STATE == DDTStateA.TURNING_CENTER):
        if (node.steering_angle_actual == 0.):
            node.adapter.publish_cmd(accel=2.)
            A_STATE = DDTStateA.RPM200

    elif (A_STATE == DDTStateA.RPM200):
        if (node.wheel_speed >= 250.):
            node.adapter.publish_cmd(accel=-2.)
            A_STATE = DDTStateA.FINISH

    elif (A_STATE == DDTStateA.STOP):
        if (node.wheel_speed == 0.):
            node.adapter.set_mission_state(
                CanState.AMI_DDT_INSPECTION_A, CanState.AS_FINISHED)

def ddt_inspection_b(node):
    global P
    global B_STATE

    print(B_STATE)

    if (B_STATE == DDTStateB.START):
        node.adapter.publish_cmd(accel=2.)
        B_STATE = DDTStateB.RPM50

    elif (B_STATE == DDTStateB.RPM50):
        if (node.wheel_speed >= 100.):
            node.adapter.ebs()
            B_STATE = DDTStateB.EBS

    elif (B_STATE == DDTStateB.EBS):
        if (node.wheel_speed == 0.):
            B_STATE = DDTStateB.FINISH

def autonomous_demo(node):
    global P
    global AUTONOMOUS_STATE
    global START_POINT

    if (AUTONOMOUS_STATE == AutonomousState.START):
        node.adapter.publish_cmd(steering_angle=P.MAX_STEER)
        AUTONOMOUS_STATE = AutonomousState.TURNING_LEFT

    elif (AUTONOMOUS_STATE == AutonomousState.TURNING_LEFT):
        if (node.steering_angle_actual >= P.MAX_STEER):
            node.adapter.publish_cmd(steering_angle=-P.MAX_STEER)
            AUTONOMOUS_STATE = AutonomousState.TURNING_RIGHT

    elif (AUTONOMOUS_STATE == AutonomousState.TURNING_RIGHT):
        if (node.steering_angle_actual <= -P.MAX_STEER):
            node.adapter.publish_cmd(steering_angle=0)
            AUTONOMOUS_STATE = AutonomousState.TURNING_CENTER

    elif (AUTONOMOUS_STATE == AutonomousState.TURNING_CENTER):
        if (node.steering_angle_actual == 0):
            START_POINT = np.array([node.position.x, node.position.y])
            node.adapter.publish_cmd(speed=kmh_to_ms(20))
            AUTONOMOUS_STATE = AutonomousState.KMH15_M10_1

    elif (AUTONOMOUS_STATE == AutonomousState.KMH15_M10_1):
        dist = np.linalg.norm(
            START_POINT - np.array([node.position.x, node.position.y]))
        if (node.velocity_actual >= kmh_to_ms(15) and dist >= 10):
            node.adapter.publish_cmd()
            AUTONOMOUS_STATE = AutonomousState.BRAKING
        else:
            node.adapter.publish_cmd(speed=kmh_to_ms(20))

    elif (AUTONOMOUS_STATE == AutonomousState.BRAKING):
        if (node.velocity_actual <= 0):
            START_POINT = np.array([node.position.x, node.position.y])
            node.adapter.publish_cmd(speed=kmh_to_ms(20))
            AUTONOMOUS_STATE = AutonomousState.KMH15_M10_2

    elif (AUTONOMOUS_STATE == AutonomousState.KMH15_M10_2):
        dist = np.linalg.norm(
            START_POINT - np.array([node.position.x, node.position.y]))
        if (node.velocity_actual >= kmh_to_ms(15) and dist >= 10):
            node.adapter.ebs()
            AUTONOMOUS_STATE = AutonomousState.EBS
        else:
            node.adapter.publish_cmd(speed=kmh_to_ms(20))

    elif (AUTONOMOUS_STATE == AutonomousState.EBS):
        if (node.velocity_actual == 0):
            AUTONOMOUS_STATE = AutonomousState.FINISH
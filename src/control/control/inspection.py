from enum import Enum
from eufs_msgs.msg import CanState

import math
import numpy as np
import time

from .config import Params

class DDTStateA(Enum):
    START = 0
    TURNING_LEFT = 1
    TURNING_RIGHT = 2
    TURNING_CENTER = 3
    RPM200 = 4
    ROLLING = 5
    STOP = 6
    FINISH = 7

class DDTStateB(Enum):
    START = 0
    RPM50 = 1
    ROLLING = 2
    EBS = 3
    FINISH = 4

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
START_POINT = [0, 0]
TIMER = 0

def rpm_to_velocity(rpm):
    return rpm * P.tire_diam * math.pi / 60

def kmh_to_ms(kmh):
    return kmh / 3.6

def ddt_inspection_a(node):
    global P
    global A_STATE
    global TIMER

    print(A_STATE, node.steering_angle_actual, node.velocity_actual, P.MAX_STEER)

    if (A_STATE == DDTStateA.START):
        node.adapter.publish_cmd(steering_angle=P.MAX_STEER)
        A_STATE = DDTStateA.TURNING_LEFT

    elif (A_STATE == DDTStateA.TURNING_LEFT):
        if (node.steering_angle_actual >= P.MAX_STEER - 0.05):
            node.adapter.publish_cmd(steering_angle=-P.MAX_STEER)
            A_STATE = DDTStateA.TURNING_RIGHT
        else:
            node.adapter.publish_cmd(steering_angle=P.MAX_STEER)

    elif (A_STATE == DDTStateA.TURNING_RIGHT):
        if (node.steering_angle_actual <= -P.MAX_STEER + 0.05):
            node.adapter.publish_cmd(steering_angle=0.)
            A_STATE = DDTStateA.TURNING_CENTER
        else:
            node.adapter.publish_cmd(steering_angle=-P.MAX_STEER)

    elif (A_STATE == DDTStateA.TURNING_CENTER):
        if (node.steering_angle_actual == 0.):
            node.adapter.publish_cmd(accel=2.)
            A_STATE = DDTStateA.RPM200
        else:
            node.adapter.publish_cmd(steering_angle=0.)

    elif (A_STATE == DDTStateA.RPM200):
        if (node.wheel_speed >= 250.):
            node.adapter.publish_cmd(accel=0.)
            A_STATE = DDTStateA.ROLLING
            TIMER = time.perf_counter()
        else:
            node.adapter.publish_cmd(accel=10.)

    elif (A_STATE == DDTStateA.ROLLING):
        if (time.perf_counter() - TIMER > 3):
            node.adapter.publish_cmd(accel=-4.)
            A_STATE = DDTStateA.STOP
        else:
            node.adapter.publish_cmd(accel=0.)

    elif (A_STATE == DDTStateA.STOP):
        if (node.wheel_speed <= 0.05 and node.wheel_speed >= -0.05):
            node.adapter.eufs_mission_finished()
            node.adapter.publish_cmd(accel=0.)
            A_STATE = DDTStateA.FINISH
        else:
            node.adapter.publish_cmd(accel=-4.)

def ddt_inspection_b(node):
    global P
    global B_STATE
    global TIMER

    print(B_STATE, node.wheel_speed)

    if (B_STATE == DDTStateB.START):
        node.adapter.publish_cmd(accel=0.1)
        B_STATE = DDTStateB.RPM50

    elif (B_STATE == DDTStateB.RPM50):
        if (node.wheel_speed >= 50.):
            node.adapter.publish_cmd(accel=0.1)
            B_STATE = DDTStateB.ROLLING
            TIMER = time.perf_counter()
            print(TIMER)
        else:
            node.adapter.publish_cmd(accel=.5)

    elif (B_STATE == DDTStateB.ROLLING):
        print(time.perf_counter() - TIMER, time.perf_counter(), TIMER)
        if (time.perf_counter() - TIMER > 3):
            node.adapter.ebs()
            B_STATE = DDTStateB.EBS
        else:
            node.adapter.publish_cmd(accel=.1)

    elif (B_STATE == DDTStateB.EBS):
        if (node.wheel_speed == 0.):
            node.adapter.eufs_mission_finished()
            B_STATE = DDTStateB.FINISH

def distance(p1, p2):
    return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

def autonomous_demo(node):
    global P
    global AUTONOMOUS_STATE
    global START_POINT

    print(AUTONOMOUS_STATE)

    if (AUTONOMOUS_STATE == AutonomousState.START):
        node.adapter.publish_cmd(steering_angle=P.MAX_STEER)
        AUTONOMOUS_STATE = AutonomousState.TURNING_LEFT

    elif (AUTONOMOUS_STATE == AutonomousState.TURNING_LEFT):
        if (node.steering_angle_actual >= P.MAX_STEER - 0.05):
            node.adapter.publish_cmd(steering_angle=-P.MAX_STEER)
            AUTONOMOUS_STATE = AutonomousState.TURNING_RIGHT
        else:
            node.adapter.publish_cmd(steering_angle=P.MAX_STEER)

    elif (AUTONOMOUS_STATE == AutonomousState.TURNING_RIGHT):
        if (node.steering_angle_actual <= -P.MAX_STEER + 0.05):
            node.adapter.publish_cmd(steering_angle=0.)
            AUTONOMOUS_STATE = AutonomousState.TURNING_CENTER
        else:
            node.adapter.publish_cmd(steering_angle=-P.MAX_STEER)

    elif (AUTONOMOUS_STATE == AutonomousState.TURNING_CENTER):
        if (node.steering_angle_actual == 0):
            START_POINT = [node.position[0], node.position[1]]
            node.adapter.publish_cmd(accel=0.2)
            AUTONOMOUS_STATE = AutonomousState.KMH15_M10_1
        else:
            node.adapter.publish_cmd(steering_angle=0.)

    elif (AUTONOMOUS_STATE == AutonomousState.KMH15_M10_1):
        dist = distance(START_POINT, [node.position[0], node.position[1]])
        if (node.velocity_actual >= kmh_to_ms(15) and dist >= 10):
            node.adapter.publish_cmd(accel=-2)
            AUTONOMOUS_STATE = AutonomousState.BRAKING
        else:
            node.adapter.publish_cmd(accel=0.2)

    elif (AUTONOMOUS_STATE == AutonomousState.BRAKING):
        if (node.velocity_actual <= 0.):
            START_POINT = [node.position[0], node.position[1]]
            node.adapter.publish_cmd(accel=0.2)
            AUTONOMOUS_STATE = AutonomousState.KMH15_M10_2
        else:
            node.adapter.publish_cmd(accel=-2.)


    elif (AUTONOMOUS_STATE == AutonomousState.KMH15_M10_2):
        dist = distance(START_POINT, [node.position[0], node.position[1]])
        if (node.velocity_actual >= kmh_to_ms(15) and dist >= 10):
            node.adapter.ebs()
            AUTONOMOUS_STATE = AutonomousState.EBS
        else:
            node.adapter.publish_cmd(accel=.2)

    elif (AUTONOMOUS_STATE == AutonomousState.EBS):
        if (node.velocity_actual == 0.):
            node.adapter.eufs_mission_finished()
            AUTONOMOUS_STATE = AutonomousState.FINISH
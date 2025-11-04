#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node

from evdev import InputDevice, ecodes, list_devices

from pacsim.msg import StampedScalar
import custom_interfaces.msg._control_command

NODE_NAME = "ps3"

TOPIC_CTRL = "/control/command"
TOPIC_PACSIM_STEER = "/pacsim/steering_setpoint"
TOPIC_PACSIM_THROTTLE = "/pacsim/throttle_setpoint"

PUBLISH_RATE_HZ = 100.0
MAX_STEERING_RAD = math.radians(20.0)  # 20 deg

LX = ecodes.ABS_X
L2 = ecodes.ABS_Z
R2 = ecodes.ABS_RZ


def clamp(v, a, b): return max(a, min(b, v))

def norm_0_1(raw: int, info) -> float:
    if info and info.max != info.min:
        v = (raw - info.min) / float(info.max - info.min)
    else:
        v = raw / 255.0
    return clamp(v, 0.0, 1.0)

def norm_lx_neg1_pos1(raw: int, info) -> float:
    v01 = norm_0_1(raw, info)
    return (v01 - 0.5) * 2.0

def find_ps3_device() -> Optional[InputDevice]:
    for path in list_devices():
        dev = InputDevice(path)
        name = (dev.name or "").lower()
        if any(k in name for k in ["ps3", "playstation", "dualshock", "sony", "controller", "gamepad"]):
            return dev
    return None


class PS3(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        # Parameters
        self.declare_parameter("device", "")  # e.g. "/dev/input/eventX"
        self.declare_parameter("rate_hz", PUBLISH_RATE_HZ)
        self.declare_parameter("max_steering_deg", 20.0)

        rate_hz = float(self.get_parameter("rate_hz").get_parameter_value().double_value)
        max_deg = float(self.get_parameter("max_steering_deg").get_parameter_value().double_value)
        self.max_steer_rad = math.radians(max_deg)
        self.device_param = self.get_parameter("device").get_parameter_value().string_value.strip()

        # Publishers
        self.ctrl_pub = self.create_publisher(
            custom_interfaces.msg._control_command.ControlCommand, TOPIC_CTRL, 10
        )
        self.pacsim_steer_pub = self.create_publisher(StampedScalar, TOPIC_PACSIM_STEER, 10)
        self.pacsim_throttle_pub = self.create_publisher(StampedScalar, TOPIC_PACSIM_THROTTLE, 10)

        # State
        self._lx = 0.0
        self._l2 = 0.0
        self._r2 = 0.0
        self._lock = threading.Lock()

        # Device
        self._dev_path: Optional[str] = self.device_param if self.device_param else None
        self._dev: Optional[InputDevice] = None
        self._absinfo = {}

        # Threads & timers
        self._stop_evt = threading.Event()
        self._reader = threading.Thread(target=self._reader_thread, daemon=True)
        self._reader.start()

        self._period = 1.0 / float(rate_hz)
        self._timer = self.create_timer(self._period, self._publish_tick)

        self.get_logger().info(
            f"ps3 running @{rate_hz:.1f} Hz | max_steer={self.max_steer_rad:.3f} rad | device='{self.device_param or 'auto'}'"
        )

    def _open_device(self) -> bool:
        try:
            dev = InputDevice(self._dev_path) if self._dev_path else find_ps3_device()
            if not dev:
                self.get_logger().warn("No PS3-like controller found. Retrying in 1s...")
                return False
            self._dev = dev
            caps = dict(dev.capabilities(absinfo=True))
            abs_caps = caps.get(ecodes.EV_ABS, [])
            self._absinfo = {code: info for code, info in abs_caps}
            self.get_logger().info(f"Using input device: {dev.path} ({dev.name})")
            return True
        except Exception as e:
            self.get_logger().warn(f"Failed to open controller: {e}. Retrying in 1s...")
            return False

    def _reader_thread(self):
        while not self._stop_evt.is_set():
            if not self._dev and not self._open_device():
                time.sleep(1.0)
                continue
            try:
                for ev in self._dev.read_loop():
                    if self._stop_evt.is_set():
                        break
                    if ev.type != ecodes.EV_ABS:
                        continue
                    info = self._absinfo.get(ev.code)
                    if ev.code == LX and info:
                        with self._lock:
                            self._lx = norm_lx_neg1_pos1(ev.value, info)
                    elif ev.code == L2 and info:
                        with self._lock:
                            self._l2 = norm_0_1(ev.value, info)
                    elif ev.code == R2 and info:
                        with self._lock:
                            self._r2 = norm_0_1(ev.value, info)
            except (OSError, IOError):
                self.get_logger().warn("Controller read error/disconnect. Reconnecting...")
                try:
                    if self._dev:
                        self._dev.close()
                except Exception:
                    pass
                self._dev = None
                time.sleep(1.0)
            except Exception as e:
                self.get_logger().error(f"Reader exception: {e}")
                time.sleep(0.2)

    def _publish_tick(self):
        with self._lock:
            lx, l2, r2 = self._lx, self._l2, self._r2

        steering = float(lx * self.max_steer_rad)  # radians
        throttle = float(r2 - l2)                  # [-1, +1]

        msg = custom_interfaces.msg._control_command.ControlCommand()
        msg.throttle_rl = throttle
        msg.throttle_rr = throttle
        msg.steering = steering
        self.ctrl_pub.publish(msg)

        lat = StampedScalar(); lat.value = steering
        lon = StampedScalar(); lon.value = throttle
        self.pacsim_steer_pub.publish(lat)
        self.pacsim_throttle_pub.publish(lon)

    def destroy_node(self):
        self._stop_evt.set()
        try:
            if self._reader.is_alive():
                self._reader.join(timeout=1.0)
        except Exception:
            pass
        try:
            if self._dev:
                self._dev.close()
        except Exception:
            pass
        return super().destroy_node()


def main():
    rclpy.init()
    node = PS3()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

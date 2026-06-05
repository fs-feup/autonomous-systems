#!/usr/bin/env python3
"""ROS2 node that publishes Logitech G923 HID inputs as invictasim control commands.

- Steering/throttle/brake come from /dev/hidraw directly.
- Custom force feedback is sent through /dev/hidraw directly.
- Autocenter is disabled through HID, not /dev/input/event*.
- Runtime FFB sends only the custom constant force.
- Road/motor vibration is added as a software torque overlay.
"""

from __future__ import annotations

import array
import fcntl
import glob
import math
import os
import select
import struct
import sys
import threading
import time
from dataclasses import dataclass


def bootstrap_ros_environment() -> None:
    python_tag = f"python{sys.version_info.major}.{sys.version_info.minor}"
    workspace_root = os.path.dirname(os.path.abspath(__file__))
    install_root = os.path.join(workspace_root, "install")

    python_paths = [
        f"/opt/ros/humble/local/lib/{python_tag}/dist-packages",
        f"/opt/ros/humble/lib/{python_tag}/site-packages",
    ]
    python_paths.extend(
        sorted(glob.glob(os.path.join(install_root, "*", "local", "lib", python_tag, "dist-packages")))
    )

    lib_paths = ["/opt/ros/humble/lib"]
    lib_paths.extend(sorted(glob.glob(os.path.join(install_root, "*", "lib"))))

    ament_paths = ["/opt/ros/humble"]
    ament_paths.extend(sorted(glob.glob(os.path.join(install_root, "*"))))

    for path in reversed([path for path in python_paths if os.path.isdir(path)]):
        if path not in sys.path:
            sys.path.insert(0, path)

    def prepend_env(name: str, paths: list[str]) -> None:
        existing = [path for path in os.environ.get(name, "").split(":") if path]
        merged = [path for path in paths if os.path.isdir(path)]
        merged.extend(path for path in existing if path not in merged)
        os.environ[name] = ":".join(merged)

    prepend_env("PYTHONPATH", python_paths)
    prepend_env("LD_LIBRARY_PATH", lib_paths)
    prepend_env("AMENT_PREFIX_PATH", ament_paths)
    os.environ.setdefault("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4")

    if os.environ.get("WHEEL_NODE_BOOTSTRAPPED") != "1":
        os.environ["WHEEL_NODE_BOOTSTRAPPED"] = "1"
        os.execvpe(sys.executable, [sys.executable, *sys.argv], os.environ)


bootstrap_ros_environment()

import rclpy
from custom_interfaces.msg import ControlCommand, TireForces, VehicleStateVector
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


# =============================================================================
# Configuration
# =============================================================================

MAX_STEERING_RAD = 0.39
STEERING_WHEEL_CUTOFF = 0.30

CONTROL_TOPIC = "/control/command"
SELF_ALIGNING_MOMENT_TOPIC = "/invictasim/vehicle_model/tire/forces"
VEHICLE_STATE_TOPIC = "/invictasim/vehicle_model/status"

PUBLISH_PERIOD_SECONDS = 0.005  # 200 Hz

SELF_ALIGNING_MOMENT_TO_WHEEL_TORQUE = 2.5
MAX_FORCE_FEEDBACK_TORQUE_NM = 2.3
# Master force-feedback scale applied to the final torque (steering Mz + vibration).
# 1.0 = full, lower = weaker. Tune to taste.
FORCE_FEEDBACK_GAIN = 0.8
FORCE_FEEDBACK_INVERT = True
FORCE_FEEDBACK_USE_FRONT_MZ_ONLY = True

# Reduces punches from abrupt Mz changes. Kept high to minimize felt delay: at 150 Nm/s
# a full-scale change settles in ~15 ms (vs ~90 ms at 25). Lower it if FFB feels notchy.
FORCE_FEEDBACK_MAX_SLEW_NM_PER_SECOND = 150.0
FORCE_FEEDBACK_DEADBAND_NM = 0.015

# If tire force messages stop, ramp force back to zero.
FORCE_FEEDBACK_TIMEOUT_SECONDS = 0.10

# If vehicle state messages stop, vibration fades to zero.
VEHICLE_STATE_TIMEOUT_SECONDS = 0.25

# Asphalt / motor vibration model.
ENABLE_FORCE_FEEDBACK_VIBRATION = True

# Start conservative. Increase gradually if it is too weak.
MAX_VIBRATION_TORQUE_NM = 0.8

VIBRATION_MIN_SPEED_MPS = 0.5
VIBRATION_FULL_SPEED_MPS = 25.0
VIBRATION_FULL_RPM = 5500.0

ASPHALT_BASE_FREQUENCY_HZ = 10.0
ASPHALT_SPEED_FREQUENCY_GAIN = 2

# If rpms are wheel rpms, 2.0 is a reasonable tactile order.
MOTOR_VIBRATION_ORDER = 2.0

ASPHALT_VIBRATION_WEIGHT = 0.65
MOTOR_VIBRATION_WEIGHT = 0.35

# Smooth vibration amplitude changes.
VIBRATION_AMPLITUDE_TIME_CONSTANT = 0.20

# HID behavior.
HIDRAW_REPORT_ID = 0x08  # Logitech input report id
HIDRAW_INPUT_REPORT_SIZE = 12
HIDRAW_OUTPUT_REPORT_SIZE = 16
HIDRAW_READ_SIZE = 64  # generous read buffer; one full HID report is returned per read

# Thrustmaster input report (e.g. T598 "Advance Racer"), little-endian:
#   byte 0     report id (0x07)
#   bytes 1-2  steering  (X,  u16, centered ~32768)
#   bytes 3-4  brake     (Y,  u16, 1023 = released .. 0 = pressed)
#   bytes 5-6  throttle  (Rz, u16, 1023 = released .. 0 = pressed)
#   bytes 7-8  clutch    (Slider, u16) -- unused
TM_HIDRAW_REPORT_ID = 0x07

# Logitech neutral force byte. 128 is approximately zero.
LG4FF_NEUTRAL_FORCE = 128

# Evdev force feedback (used for Thrustmaster wheels driven by hid-tmff2). This is the
# standard Linux input FF_CONSTANT effect; the kernel driver translates it to the wheel's
# protocol. struct ff_effect is 48 bytes on x86-64.
EV_FF = 0x15
FF_CONSTANT = 0x52
FF_EFFECT_SIZE = 48
FF_DIRECTION = 0x4000  # 90 deg; the sign of the level selects torque direction


def _ioc_write(type_char: str, nr: int, size: int) -> int:
    # asm-generic _IOC for a write-direction ioctl.
    return (1 << 30) | (size << 16) | (ord(type_char) << 8) | nr


EVIOCSFF = _ioc_write("E", 0x80, FF_EFFECT_SIZE)
EVIOCRMFF = _ioc_write("E", 0x81, 4)

# Joystick fallback constants. Used only if HID input is unavailable.
ABS_X = 0
ABS_Z = 2
ABS_RZ = 5
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80
JS_EVENT_FORMAT = "IhBB"
JS_EVENT_SIZE = struct.calcsize(JS_EVENT_FORMAT)
JSIOCGAXES = 0x80016A11
JSIOCGNAME = 0x81006A13
JSIOCGAXMAP = 0x80406A32

LOGITECH_KEYWORDS = (
    "g923",
    "g29",
    "g920",
    "driving force",
)

THRUSTMASTER_KEYWORDS = (
    "thrustmaster",
    "advance racer",
    "t598",
    "t300",
    "t248",
    "t150",
    "tmx",
    "ts-xw",
    "t-gt",
)

# Union, plus a generic term. Used only to decide "is this a wheel at all".
WHEEL_NAME_KEYWORDS = LOGITECH_KEYWORDS + THRUSTMASTER_KEYWORDS + ("racing wheel",)


# =============================================================================
# Utility
# =============================================================================

@dataclass
class WheelState:
    steering: float = 0.0
    throttle: float = 0.0
    brake: float = 0.0


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def _name_matches(name: str, keywords: tuple[str, ...]) -> bool:
    lowered = name.lower()
    return any(keyword in lowered for keyword in keywords)


def is_wheel_name(name: str) -> bool:
    return _name_matches(name, WHEEL_NAME_KEYWORDS)


def is_logitech_name(name: str) -> bool:
    return _name_matches(name, LOGITECH_KEYWORDS)


def is_thrustmaster_name(name: str) -> bool:
    return _name_matches(name, THRUSTMASTER_KEYWORDS)


def pedal_from_raw_255(value: int) -> float:
    return clamp((255.0 - float(value)) / 255.0, 0.0, 1.0)


def pedal_from_raw_1023(value: int) -> float:
    # Thrustmaster pedals report 1023 = released, 0 = fully pressed.
    return clamp((1023.0 - float(value)) / 1023.0, 0.0, 1.0)


def pedal_from_axis(value: int, released: int, minimum: int = -32767, maximum: int = 32767) -> float:
    if abs(released - minimum) <= abs(maximum - released):
        return clamp((float(value) - float(released)) / (float(maximum) - float(released)), 0.0, 1.0)

    return clamp((float(released) - float(value)) / (float(released) - float(minimum)), 0.0, 1.0)


def steering_from_raw_u16(value: int) -> float:
    return clamp((32768.0 - float(value)) / 32767.0, -1.0, 1.0)


def steering_to_radians(normalized_steering: float) -> float:
    cutoff = max(abs(STEERING_WHEEL_CUTOFF), 1e-9)
    scaled = clamp(normalized_steering / cutoff, -1.0, 1.0)
    return scaled * MAX_STEERING_RAD


def force_byte_from_torque(torque_nm: float) -> int:
    if abs(torque_nm) < FORCE_FEEDBACK_DEADBAND_NM:
        torque_nm = 0.0

    normalized = clamp(
        torque_nm / max(MAX_FORCE_FEEDBACK_TORQUE_NM, 1e-9),
        -1.0,
        1.0,
    )

    return int(round((normalized + 1.0) * 127.5))


def hidraw_info(path: str) -> tuple[str, str]:
    name = ""
    phys = ""

    with open(f"/sys/class/hidraw/{os.path.basename(path)}/device/uevent", encoding="utf-8") as f:
        for line in f:
            key, _, value = line.strip().partition("=")
            if key == "HID_NAME":
                name = value
            elif key == "HID_PHYS":
                phys = value

    return name, phys


# =============================================================================
# HID input reader
# =============================================================================

class WheelReader:
    def read(self) -> WheelState:
        raise NotImplementedError

    def close(self) -> None:
        pass


class HidrawWheelReader(WheelReader):
    """Direct HID input reader (Logitech report layout).

    This is the preferred path. Controls come from HID, not evdev.
    """

    report_id = HIDRAW_REPORT_ID
    read_size = HIDRAW_READ_SIZE

    def __init__(self, path: str):
        self.path = path
        self.fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        self.state = WheelState()
        self.lock = threading.Lock()
        self.running = True

        self.thread = threading.Thread(
            target=self._read_loop,
            name="hidraw-wheel-reader",
            daemon=True,
        )
        self.thread.start()

    def _read_loop(self) -> None:
        while self.running:
            try:
                readable, _, _ = select.select([self.fd], [], [], 0.05)
            except OSError:
                return

            if not readable:
                continue

            while self.running:
                try:
                    data = os.read(self.fd, self.read_size)
                except BlockingIOError:
                    break
                except OSError:
                    return

                self._handle_report(data)

    def _handle_report(self, data: bytes) -> None:
        if len(data) < HIDRAW_INPUT_REPORT_SIZE:
            return

        if data[0] != self.report_id:
            return

        steering_raw = data[4] | (data[5] << 8)

        state = WheelState(
            steering=steering_from_raw_u16(steering_raw),
            throttle=pedal_from_raw_255(data[6]),
            brake=pedal_from_raw_255(data[7]),
        )

        with self.lock:
            self.state = state

    def read(self) -> WheelState:
        with self.lock:
            return WheelState(
                steering=self.state.steering,
                throttle=self.state.throttle,
                brake=self.state.brake,
            )

    def close(self) -> None:
        self.running = False
        self.thread.join(timeout=0.2)

        try:
            os.close(self.fd)
        except OSError:
            pass


class ThrustmasterHidrawWheelReader(HidrawWheelReader):
    """Direct HID input reader for Thrustmaster wheels (e.g. T598).

    Reuses the threaded read loop; only the report parsing differs. See the
    TM_HIDRAW_REPORT_ID notes for the report 0x07 layout.
    """

    report_id = TM_HIDRAW_REPORT_ID

    def _handle_report(self, data: bytes) -> None:
        # Need bytes 0..6 (report id, steering, brake, throttle).
        if len(data) < 7 or data[0] != self.report_id:
            return

        steering_raw = data[1] | (data[2] << 8)
        brake_raw = data[3] | (data[4] << 8)
        throttle_raw = data[5] | (data[6] << 8)

        state = WheelState(
            steering=steering_from_raw_u16(steering_raw),
            throttle=pedal_from_raw_1023(throttle_raw),
            brake=pedal_from_raw_1023(brake_raw),
        )

        with self.lock:
            self.state = state


class JoystickWheelReader(WheelReader):
    """Fallback only. HID should be used when available."""

    def __init__(self, path: str, axis_map: list[int]):
        self.path = path
        self.fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        self.state = WheelState()
        self.lock = threading.Lock()
        self.running = True

        self.steering_axis = axis_map.index(ABS_X)
        self.throttle_axis = axis_map.index(ABS_Z)
        self.brake_axis = axis_map.index(ABS_RZ)

        self.throttle_released = -32767
        self.brake_released = 32767

        self.drain_initial_events()

        self.thread = threading.Thread(
            target=self._read_loop,
            name="js-wheel-reader",
            daemon=True,
        )
        self.thread.start()

    def drain_initial_events(self) -> None:
        while True:
            try:
                data = os.read(self.fd, JS_EVENT_SIZE)
            except BlockingIOError:
                break
            except OSError:
                break

            if len(data) != JS_EVENT_SIZE:
                break

            self._handle_event(data, calibrating=True)

    def _read_loop(self) -> None:
        while self.running:
            try:
                readable, _, _ = select.select([self.fd], [], [], 0.05)
            except OSError:
                return

            if not readable:
                continue

            while self.running:
                try:
                    data = os.read(self.fd, JS_EVENT_SIZE)
                except BlockingIOError:
                    break
                except OSError:
                    return

                if len(data) != JS_EVENT_SIZE:
                    break

                self._handle_event(data, calibrating=False)

    def _handle_event(self, data: bytes, calibrating: bool) -> None:
        _, value, event_type, axis = struct.unpack(JS_EVENT_FORMAT, data)
        event_type &= ~JS_EVENT_INIT

        if event_type != JS_EVENT_AXIS:
            return

        with self.lock:
            if axis == self.steering_axis:
                self.state.steering = clamp(-float(value) / 32767.0, -1.0, 1.0)

            elif axis == self.throttle_axis:
                if calibrating:
                    self.throttle_released = value
                    self.state.throttle = 0.0
                else:
                    self.state.throttle = pedal_from_axis(value, self.throttle_released)

            elif axis == self.brake_axis:
                if calibrating:
                    self.brake_released = value
                    self.state.brake = 0.0
                else:
                    self.state.brake = pedal_from_axis(value, self.brake_released)

    def read(self) -> WheelState:
        with self.lock:
            return WheelState(
                steering=self.state.steering,
                throttle=self.state.throttle,
                brake=self.state.brake,
            )

    def close(self) -> None:
        self.running = False
        self.thread.join(timeout=0.2)

        try:
            os.close(self.fd)
        except OSError:
            pass


def find_hidraw_reader() -> tuple[WheelReader, str] | None:
    for path in sorted(glob.glob("/dev/hidraw*")):
        try:
            name, phys = hidraw_info(path)
        except OSError:
            continue

        if not is_wheel_name(name):
            continue

        # Your working wheel is input0. Keep this exactly.
        if not phys.endswith("/input0"):
            continue

        reader_cls = (
            ThrustmasterHidrawWheelReader if is_thrustmaster_name(name) else HidrawWheelReader
        )

        try:
            return reader_cls(path), f"{path}: {name} via HID raw input"
        except OSError:
            continue

    return None


def js_name(path: str) -> str:
    with open(path, "rb") as device:
        name = array.array("b", [0]) * 128
        fcntl.ioctl(device, JSIOCGNAME, name, True)

    return name.tobytes().split(b"\x00", 1)[0].decode("utf-8", errors="replace")


def js_axis_map(path: str) -> list[int]:
    with open(path, "rb") as device:
        axes = array.array("B", [0])
        fcntl.ioctl(device, JSIOCGAXES, axes, True)

        axis_map = array.array("B", [0]) * max(axes[0], 64)
        fcntl.ioctl(device, JSIOCGAXMAP, axis_map, True)

    return list(axis_map[: axes[0]])


def find_joystick_reader() -> tuple[WheelReader, str] | None:
    for path in sorted(glob.glob("/dev/input/js*")):
        try:
            name = js_name(path)
            axis_map = js_axis_map(path)
        except OSError:
            continue

        if is_wheel_name(name) and all(axis in axis_map for axis in (ABS_X, ABS_Z, ABS_RZ)):
            return JoystickWheelReader(path, axis_map), f"{path}: {name} via joystick fallback"

    return None


def find_wheel_reader() -> tuple[WheelReader, str, bool]:
    hidraw_reader = find_hidraw_reader()
    if hidraw_reader is not None:
        reader, description = hidraw_reader
        return reader, description, False

    joystick_reader = find_joystick_reader()
    if joystick_reader is not None:
        reader, description = joystick_reader
        return reader, description, True

    raise RuntimeError("No readable Logitech/Thrustmaster wheel input was found")


# =============================================================================
# HID force feedback
# =============================================================================

class ForceFeedback:
    def set_torque(self, torque_nm: float) -> None:
        raise NotImplementedError

    def close(self) -> None:
        pass


class HidrawLg4ffForceFeedback(ForceFeedback):
    """Direct HID Logitech force feedback.

    Runtime behavior:
    - Do not repeatedly clear effects.
    - Do not repeatedly send autocenter disable commands.
    - Only update the constant-force slot.

    Road/motor vibration is generated by the ROS node as a small torque overlay.
    """

    def __init__(self, path: str):
        self.path = path
        self.fd = os.open(path, os.O_WRONLY | os.O_NONBLOCK)
        self.last_force_byte: int | None = None

        self._initialize()

    def _write_command(self, command: list[int], delay_seconds: float = 0.0) -> bool:
        report = bytearray(HIDRAW_OUTPUT_REPORT_SIZE)
        report[: len(command)] = bytes(command)

        try:
            os.write(self.fd, report)
        except OSError:
            return False

        if delay_seconds > 0.0:
            time.sleep(delay_seconds)

        return True

    def _disable_autocenter_once(self) -> None:
        # HID command used by Logitech LG4FF-style wheels to deactivate autocenter.
        # Send only at startup/exit. Do not spam it during runtime.
        self._write_command([0xF5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], 0.010)

    def _stop_effect_slots_once(self) -> None:
        # Stop logical effect slots once.
        # These commands must not be sent periodically while driving.
        self._write_command([0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], 0.004)
        self._write_command([0x23, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], 0.004)
        self._write_command([0x43, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], 0.004)
        self._write_command([0x83, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], 0.004)

    def _start_constant_force_slot(self) -> None:
        # Start slot 0 as a neutral constant-force effect.
        self._write_command(
            [0x11, 0x00, LG4FF_NEUTRAL_FORCE, 0x00, 0x00, 0x00, 0x00],
            0.010,
        )
        self.last_force_byte = LG4FF_NEUTRAL_FORCE

    def _initialize(self) -> None:
        # Reset/init.
        self._write_command([0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], 0.020)

        # Disable passive centering.
        self._disable_autocenter_once()

        # Clear old effects only once, before starting our constant-force slot.
        self._stop_effect_slots_once()

        # Start one neutral constant-force effect. Runtime only updates this slot.
        self._start_constant_force_slot()

    def set_torque(self, torque_nm: float) -> None:
        force_byte = force_byte_from_torque(torque_nm)

        if force_byte == self.last_force_byte:
            return

        # Update slot 0 constant-force level.
        # This is the only FFB command sent during runtime.
        if self._write_command([0x1C, 0x00, force_byte, 0x00, 0x00, 0x00, 0x00]):
            self.last_force_byte = force_byte

    def close(self) -> None:
        try:
            # Neutralize custom torque.
            self._write_command([0x1C, 0x00, LG4FF_NEUTRAL_FORCE, 0x00, 0x00, 0x00, 0x00], 0.010)

            # Stop our slot.
            self._write_command([0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], 0.010)

            # Leave autocenter disabled.
            self._disable_autocenter_once()

            # Original shutdown command.
            self._write_command([0xF3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], 0.010)
        except OSError:
            pass

        try:
            os.close(self.fd)
        except OSError:
            pass


class EvdevForceFeedback(ForceFeedback):
    """Standard Linux input FF_CONSTANT force feedback.

    Works with any FF-capable evdev device, including Thrustmaster wheels driven by
    hid-tmff2 (the Logitech LG4FF HID command set does not apply to those). The node
    outputs a normalized force; the wheelbase's own gain sets the physical Nm, so this
    is correct for a direct-drive base like the T598 without hard-coding its torque.
    """

    def __init__(self, path: str):
        self.path = path
        self.fd = os.open(path, os.O_RDWR)
        self.effect_id = -1
        self.last_level: int | None = None
        self._upload(0)  # allocate the effect (raises OSError if the device has no FF)
        self._play()

    def _pack_effect(self, level: int) -> bytearray:
        buf = bytearray(FF_EFFECT_SIZE)
        # type, id, direction, trigger.button, trigger.interval, replay.length, replay.delay
        struct.pack_into(
            "<HhHHHHH", buf, 0, FF_CONSTANT, self.effect_id, FF_DIRECTION, 0, 0, 0, 0
        )
        # ff_constant_effect.level lives at offset 16 (after 2 bytes of union alignment pad).
        struct.pack_into("<h", buf, 16, level)
        return buf

    def _upload(self, level: int) -> None:
        buf = self._pack_effect(level)
        fcntl.ioctl(self.fd, EVIOCSFF, buf, True)  # writes the assigned id back into buf
        self.effect_id = struct.unpack_from("<h", buf, 2)[0]

    def _play(self) -> None:
        # input_event{ time(16), type, code, value } -> start the effect, play indefinitely.
        os.write(self.fd, struct.pack("<qqHHi", 0, 0, EV_FF, self.effect_id, 1))

    def set_torque(self, torque_nm: float) -> None:
        normalized = clamp(torque_nm / max(MAX_FORCE_FEEDBACK_TORQUE_NM, 1e-9), -1.0, 1.0)
        level = int(round(normalized * 32767.0))

        if level == self.last_level:
            return

        try:
            self._upload(level)  # re-uploading the same id updates the live force
            self.last_level = level
        except OSError:
            pass

    def close(self) -> None:
        try:
            self.set_torque(0.0)
            fcntl.ioctl(self.fd, EVIOCRMFF, self.effect_id)
        except OSError:
            pass

        try:
            os.close(self.fd)
        except OSError:
            pass


class NullForceFeedback(ForceFeedback):
    def set_torque(self, torque_nm: float) -> None:
        return


def find_hidraw_force_feedback() -> tuple[ForceFeedback, str] | None:
    for path in sorted(glob.glob("/dev/hidraw*")):
        try:
            name, phys = hidraw_info(path)
        except OSError:
            continue

        # The LG4FF command set is Logitech-specific. Never send it to other wheels
        # (e.g. Thrustmaster) -- it does nothing useful and could confuse the device.
        # Thrustmaster FFB needs the hid-tmff2 driver + evdev FF_CONSTANT instead.
        if not is_logitech_name(name):
            continue

        # Use the same HID interface as the input reader.
        if not phys.endswith("/input0"):
            continue

        try:
            return HidrawLg4ffForceFeedback(path), f"{path}: {name} via direct HID FFB"
        except OSError:
            continue

    return None


def find_evdev_force_feedback() -> tuple[ForceFeedback, str] | None:
    """Force feedback via evdev FF_CONSTANT (Thrustmaster / hid-tmff2)."""
    for sysdir in sorted(glob.glob("/sys/class/input/event*")):
        try:
            with open(os.path.join(sysdir, "device", "name"), encoding="utf-8") as f:
                name = f.read().strip()
        except OSError:
            continue

        # Only Thrustmaster here; Logitech keeps its working LG4FF hidraw path below.
        if not is_thrustmaster_name(name):
            continue

        node = os.path.join("/dev/input", os.path.basename(sysdir))
        try:
            return EvdevForceFeedback(node), f"{node}: {name} via evdev FF_CONSTANT"
        except OSError:
            # No FF yet (driver not loaded) or node/permission missing.
            continue

    return None


def find_force_feedback() -> tuple[ForceFeedback, str] | None:
    return find_evdev_force_feedback() or find_hidraw_force_feedback()


# =============================================================================
# ROS node
# =============================================================================

class WheelNode(Node):
    def __init__(self):
        super().__init__("wheel_node")

        self.reader, description, using_joystick = find_wheel_reader()

        force_feedback = find_force_feedback()
        if force_feedback is None:
            self.force_feedback: ForceFeedback = NullForceFeedback()
            force_feedback_description = ""
        else:
            self.force_feedback, force_feedback_description = force_feedback

        self.publisher = self.create_publisher(ControlCommand, CONTROL_TOPIC, 1)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.self_aligning_moment_subscription = self.create_subscription(
            TireForces,
            SELF_ALIGNING_MOMENT_TOPIC,
            self.self_aligning_moment_callback,
            qos,
        )

        self.vehicle_state_subscription = self.create_subscription(
            VehicleStateVector,
            VEHICLE_STATE_TOPIC,
            self.vehicle_state_callback,
            qos,
        )

        self.force_lock = threading.Lock()
        self.target_wheel_torque_nm = 0.0
        self.current_wheel_torque_nm = 0.0
        self.last_force_feedback_time = 0.0
        self.last_force_update_time = time.monotonic()

        self.vehicle_state_lock = threading.Lock()
        self.vehicle_velocity_x = 0.0
        self.vehicle_average_rpm = 0.0
        self.last_vehicle_state_time = 0.0

        self.asphalt_vibration_phase = 0.0
        self.motor_vibration_phase = 0.0
        self.current_vibration_amplitude_nm = 0.0

        self.get_logger().info(f"Using wheel input {description}")

        if using_joystick:
            self.get_logger().warn(
                "HID raw input was not used. Falling back to /dev/input/js*. "
                "Run with sudo or fix /dev/hidraw* permissions."
            )

        self.get_logger().info(
            f"Publishing {CONTROL_TOPIC} at {1.0 / PUBLISH_PERIOD_SECONDS:.0f} Hz, "
            f"max steering={MAX_STEERING_RAD:.3f} rad, "
            f"cutoff={STEERING_WHEEL_CUTOFF:.3f}"
        )

        self.get_logger().info(f"Subscribing {SELF_ALIGNING_MOMENT_TOPIC} for self-aligning force")
        self.get_logger().info(f"Subscribing {VEHICLE_STATE_TOPIC} for speed/rpm vibration")

        if force_feedback is None:
            self.get_logger().warn(
                "No HID force-feedback output was found. "
                "Mz subscription is active but wheel torque is disabled."
            )
        else:
            self.get_logger().info(
                f"Sending custom force feedback through {force_feedback_description}, "
                f"max torque={MAX_FORCE_FEEDBACK_TORQUE_NM:.2f} Nm"
            )

    def run(self) -> None:
        next_publish = time.monotonic()

        while rclpy.ok():
            self.publish_command()
            rclpy.spin_once(self, timeout_sec=0.0)
            self.update_force_feedback()

            next_publish += PUBLISH_PERIOD_SECONDS
            sleep_time = next_publish - time.monotonic()

            if sleep_time > 0.0:
                time.sleep(sleep_time)
            else:
                next_publish = time.monotonic()

    def publish_command(self) -> None:
        wheel_state = self.reader.read()

        throttle_command = clamp(wheel_state.throttle - wheel_state.brake, -1.0, 1.0)
        steering_command = steering_to_radians(wheel_state.steering)

        msg = ControlCommand()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.throttle_fl = 0.0
        msg.throttle_fr = 0.0
        msg.throttle_rl = throttle_command
        msg.throttle_rr = throttle_command
        msg.steering = steering_command

        self.publisher.publish(msg)

    def self_aligning_moment_callback(self, msg: TireForces) -> None:
        mz = msg.fl_wrench.torque.z + msg.fr_wrench.torque.z

        if not FORCE_FEEDBACK_USE_FRONT_MZ_ONLY:
            mz += msg.rl_wrench.torque.z + msg.rr_wrench.torque.z

        wheel_torque = mz * SELF_ALIGNING_MOMENT_TO_WHEEL_TORQUE

        if FORCE_FEEDBACK_INVERT:
            wheel_torque = -wheel_torque

        wheel_torque = clamp(
            wheel_torque,
            -MAX_FORCE_FEEDBACK_TORQUE_NM,
            MAX_FORCE_FEEDBACK_TORQUE_NM,
        )

        with self.force_lock:
            self.target_wheel_torque_nm = wheel_torque
            self.last_force_feedback_time = time.monotonic()

    def vehicle_state_callback(self, msg: VehicleStateVector) -> None:
        rpm_values = [
            abs(float(msg.fl_rpm)),
            abs(float(msg.fr_rpm)),
            abs(float(msg.rl_rpm)),
            abs(float(msg.rr_rpm)),
        ]

        average_rpm = sum(rpm_values) / max(len(rpm_values), 1)

        with self.vehicle_state_lock:
            self.vehicle_velocity_x = abs(float(msg.velocity_x))
            self.vehicle_average_rpm = average_rpm
            self.last_vehicle_state_time = time.monotonic()

    def compute_vibration_torque(self, dt: float, now: float) -> float:
        if not ENABLE_FORCE_FEEDBACK_VIBRATION:
            return 0.0

        with self.vehicle_state_lock:
            velocity_x = self.vehicle_velocity_x
            average_rpm = self.vehicle_average_rpm
            last_vehicle_state_time = self.last_vehicle_state_time

        if last_vehicle_state_time <= 0.0 or now - last_vehicle_state_time > VEHICLE_STATE_TIMEOUT_SECONDS:
            target_amplitude = 0.0
            velocity_x = 0.0
            average_rpm = 0.0
        else:
            speed_factor = clamp(
                (velocity_x - VIBRATION_MIN_SPEED_MPS)
                / max(VIBRATION_FULL_SPEED_MPS - VIBRATION_MIN_SPEED_MPS, 1e-9),
                0.0,
                1.0,
            )

            rpm_factor = clamp(
                average_rpm / max(VIBRATION_FULL_RPM, 1e-9),
                0.0,
                1.0,
            )

            combined_factor = clamp(
                ASPHALT_VIBRATION_WEIGHT * speed_factor
                + MOTOR_VIBRATION_WEIGHT * rpm_factor,
                0.0,
                1.0,
            )

            target_amplitude = MAX_VIBRATION_TORQUE_NM * combined_factor

        alpha = clamp(
            dt / max(VIBRATION_AMPLITUDE_TIME_CONSTANT, 1e-9),
            0.0,
            1.0,
        )

        self.current_vibration_amplitude_nm += alpha * (
            target_amplitude - self.current_vibration_amplitude_nm
        )

        if self.current_vibration_amplitude_nm <= 1e-5:
            return 0.0

        asphalt_frequency_hz = clamp(
            ASPHALT_BASE_FREQUENCY_HZ + ASPHALT_SPEED_FREQUENCY_GAIN * velocity_x,
            15.0,
            85.0,
        )

        motor_frequency_hz = clamp(
            (average_rpm / 60.0) * MOTOR_VIBRATION_ORDER,
            8.0,
            95.0,
        )

        two_pi = 2.0 * math.pi

        self.asphalt_vibration_phase += two_pi * asphalt_frequency_hz * dt
        self.motor_vibration_phase += two_pi * motor_frequency_hz * dt

        self.asphalt_vibration_phase %= two_pi
        self.motor_vibration_phase %= two_pi

        asphalt_signal = (
            0.75 * math.sin(self.asphalt_vibration_phase)
            + 0.25 * math.sin(2.37 * self.asphalt_vibration_phase)
        )

        motor_signal = math.sin(self.motor_vibration_phase)

        vibration_signal = (
            ASPHALT_VIBRATION_WEIGHT * asphalt_signal
            + MOTOR_VIBRATION_WEIGHT * motor_signal
        )

        return self.current_vibration_amplitude_nm * vibration_signal

    def update_force_feedback(self) -> None:
        now = time.monotonic()
        dt = max(now - self.last_force_update_time, 1e-6)
        self.last_force_update_time = now

        with self.force_lock:
            target_base_torque = self.target_wheel_torque_nm
            last_msg_time = self.last_force_feedback_time

        if last_msg_time <= 0.0 or now - last_msg_time > FORCE_FEEDBACK_TIMEOUT_SECONDS:
            target_base_torque = 0.0

        max_delta = FORCE_FEEDBACK_MAX_SLEW_NM_PER_SECOND * dt
        delta = clamp(
            target_base_torque - self.current_wheel_torque_nm,
            -max_delta,
            max_delta,
        )

        self.current_wheel_torque_nm += delta

        if abs(self.current_wheel_torque_nm) < FORCE_FEEDBACK_DEADBAND_NM:
            self.current_wheel_torque_nm = 0.0

        vibration_torque = self.compute_vibration_torque(dt, now)

        final_torque = (self.current_wheel_torque_nm + vibration_torque) * FORCE_FEEDBACK_GAIN

        final_torque = clamp(
            final_torque,
            -MAX_FORCE_FEEDBACK_TORQUE_NM,
            MAX_FORCE_FEEDBACK_TORQUE_NM,
        )

        self.force_feedback.set_torque(final_torque)

    def destroy_node(self) -> bool:
        try:
            self.force_feedback.set_torque(0.0)
        except Exception:
            pass

        try:
            self.force_feedback.close()
        except Exception:
            pass

        try:
            self.reader.close()
        except Exception:
            pass

        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = WheelNode()

    try:
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
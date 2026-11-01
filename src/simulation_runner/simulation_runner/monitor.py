"""ROS node that watches a running simulation and decides when the run is over."""

from __future__ import annotations

import math
from typing import List, Optional

from custom_interfaces.msg import (
    ConeArray,
    ControlCommand,
    LapSummary,
    PathPointArray,
    PerceptionOutput,
    Pose,
    VehicleStateVector,
    Velocities,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64

from .results import LapRecord

#: Completed-lap statistics, latched so a late subscriber still sees every lap.
LAP_SUMMARY_TOPIC = "/invictasim/statistics/lap_summary"

#: Vehicle state, used for the speed the stop condition is tested against.
VEHICLE_STATUS_TOPIC = "/invictasim/vehicle_model/status"

#: The pipeline in order, so the first stage with no output localises a failure. Each entry is
#: (stage, ros node that should be alive, topic it must publish, message type).
PIPELINE = (
    ("simulator", "invictasim_output", "/invictasim/vehicle_model/status", VehicleStateVector),
    ("perception", "invictasim_output", "/invictasim/perception/cones", PerceptionOutput),
    ("velocity estimation", "velocity_estimation", "/state_estimation/velocities", Velocities),
    ("slam pose", "slam", "/state_estimation/vehicle_pose", Pose),
    ("slam map", "slam", "/state_estimation/map", ConeArray),
    ("lap counter", "slam", "/state_estimation/lap_counter", Float64),
    ("planning", "planning_adapter", "/path_planning/path", PathPointArray),
    ("control", "control_adapter", "/invictasim/input", ControlCommand),
)

_LATCHED_QOS = QoSProfile(
    depth=10,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class RunMonitor(Node):
    """Tracks a run's progress and reports when its finish conditions are met."""

    def __init__(self, min_laps: int, stop_speed: float, wait_for_stop: bool,
                 context=None) -> None:
        """Watch for min_laps, and for the car dropping below stop_speed when asked."""
        super().__init__("simulation_run_monitor", context=context)

        self._min_laps = min_laps
        self._stop_speed = stop_speed
        self._wait_for_stop = wait_for_stop

        self._laps: List[LapRecord] = []
        self._speed = 0.0
        self._peak_speed = 0.0
        self._has_moved = False

        self.create_subscription(
            LapSummary, LAP_SUMMARY_TOPIC, self._lap_summary_callback, _LATCHED_QOS
        )
        self.create_subscription(
            VehicleStateVector, VEHICLE_STATUS_TOPIC, self._vehicle_status_callback, 10
        )

        # One counter per pipeline stage: a stage with zero messages is where things broke.
        self._counts = {stage: 0 for stage, _, _, _ in PIPELINE}
        for stage, _, topic, message_type in PIPELINE:
            self.create_subscription(
                message_type, topic, self._make_counter(stage), 10
            )

    def _make_counter(self, stage: str):
        """Build a callback that only records that the stage produced something."""
        def callback(_message) -> None:
            self._counts[stage] += 1
        return callback

    def pipeline_report(self) -> List[str]:
        """Describe which nodes never appeared and which stages published nothing."""
        problems: List[str] = []

        running = set(self.get_node_names())
        missing = [node for _, node, _, _ in PIPELINE if node not in running]
        for node in sorted(set(missing)):
            problems.append(f"node '{node}' is not running")

        for stage, node, topic, _ in PIPELINE:
            if self._counts[stage] or node in missing:
                continue
            problems.append(f"{stage} published nothing on {topic}")

        return problems

    @property
    def laps(self) -> List[LapRecord]:
        """Completed laps seen so far."""
        return list(self._laps)

    @property
    def speed(self) -> float:
        """Latest vehicle speed, in m/s."""
        return self._speed

    @property
    def peak_speed(self) -> float:
        """Highest speed seen, which tells a timeout apart from a car that never drove."""
        return self._peak_speed

    @property
    def started_moving(self) -> bool:
        """Whether the car has exceeded the stop speed at least once."""
        return self._has_moved

    def _lap_summary_callback(self, message: LapSummary) -> None:
        """Store the completed-lap history the simulator publishes."""
        self._laps = [LapRecord.from_lap_statistics(row) for row in message.rows]

    def _vehicle_status_callback(self, message: VehicleStateVector) -> None:
        """Track the vehicle speed and latch that the car has started driving."""
        self._speed = math.hypot(message.velocity_x, message.velocity_y)
        self._peak_speed = max(self._peak_speed, self._speed)
        # The car starts at a standstill, so the stop test only counts once it has driven.
        if self._speed > self._stop_speed:
            self._has_moved = True

    def finish_reason(self) -> Optional[str]:
        """Why the run should end now, or ``None`` while it should keep going."""
        if len(self._laps) < self._min_laps:
            return None

        if not self._wait_for_stop:
            return "completed"

        if self._has_moved and self._speed <= self._stop_speed:
            return "completed"

        return None

"""ROS node that watches a running simulation and decides when the run is over."""

from __future__ import annotations

import math
from typing import List, Optional

from custom_interfaces.msg import LapSummary, VehicleStateVector
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from .results import LapRecord

#: Completed-lap statistics, latched so a late subscriber still sees every lap.
LAP_SUMMARY_TOPIC = "/invictasim/statistics/lap_summary"

#: Vehicle state, used for the speed the stop condition is tested against.
VEHICLE_STATUS_TOPIC = "/invictasim/vehicle_model/status"

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
        self._has_moved = False

        self.create_subscription(
            LapSummary, LAP_SUMMARY_TOPIC, self._lap_summary_callback, _LATCHED_QOS
        )
        self.create_subscription(
            VehicleStateVector, VEHICLE_STATUS_TOPIC, self._vehicle_status_callback, 10
        )

    @property
    def laps(self) -> List[LapRecord]:
        """Completed laps seen so far."""
        return list(self._laps)

    @property
    def speed(self) -> float:
        """Latest vehicle speed, in m/s."""
        return self._speed

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

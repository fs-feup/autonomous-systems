"""Metrics collected from a simulation run and the pass/fail verdict derived from them."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

from .scenarios import UPPER_BOUND_CRITERIA, Scenario


@dataclass
class LapRecord:
    """One completed lap, as written by the simulator's statistics."""

    lap: int
    lap_time: float
    cones_hit: int
    mean_tracking_error: float
    peak_tracking_error: float
    mean_velocity_error: float

    @classmethod
    def from_lap_statistics(cls, row) -> "LapRecord":
        """Build a record from one ``custom_interfaces/LapStatistics`` row."""
        return cls(
            lap=int(row.lap_number),
            lap_time=float(row.time),
            cones_hit=int(row.cones_hit),
            mean_tracking_error=float(row.avg_tracking_error_distance),
            peak_tracking_error=float(row.max_tracking_error_distance),
            mean_velocity_error=float(row.avg_velocity_error),
        )


@dataclass
class RunMetrics:
    """Aggregated metrics for a whole run, in the units the criteria are expressed in."""

    laps_completed: int = 0
    cones_hit: int = 0
    total_time: float = 0.0
    best_lap_time: float = 0.0
    mean_lap_time: float = 0.0
    mean_tracking_error: float = 0.0
    peak_tracking_error: float = 0.0
    mean_velocity_error: float = 0.0

    @classmethod
    def from_laps(cls, laps: List[LapRecord]) -> "RunMetrics":
        """Aggregate per-lap records into whole-run metrics."""
        if not laps:
            return cls()

        lap_times = [lap.lap_time for lap in laps]
        return cls(
            laps_completed=len(laps),
            cones_hit=sum(lap.cones_hit for lap in laps),
            total_time=sum(lap_times),
            best_lap_time=min(lap_times),
            mean_lap_time=sum(lap_times) / len(lap_times),
            mean_tracking_error=sum(lap.mean_tracking_error for lap in laps) / len(laps),
            peak_tracking_error=max(lap.peak_tracking_error for lap in laps),
            mean_velocity_error=sum(lap.mean_velocity_error for lap in laps) / len(laps),
        )


@dataclass
class CriterionCheck:
    """The outcome of one criterion applied to a run."""

    name: str
    limit: float
    value: float
    passed: bool


@dataclass
class ScenarioResult:
    """Everything known about one executed scenario."""

    scenario: Scenario
    finish_reason: str
    duration: float
    laps: List[LapRecord] = field(default_factory=list)
    metrics: RunMetrics = field(default_factory=RunMetrics)
    checks: List[CriterionCheck] = field(default_factory=list)
    failures: List[str] = field(default_factory=list)
    log_path: Optional[Path] = None

    @property
    def passed(self) -> bool:
        """Whether the scenario met every criterion and finished cleanly."""
        return not self.failures

    def as_dict(self) -> Dict[str, Any]:
        """Serializable form used for the machine-readable report."""
        return {
            "name": self.scenario.name,
            "description": self.scenario.description,
            "track": self.scenario.track,
            "mission": self.scenario.mission,
            "required": self.scenario.required,
            "passed": self.passed,
            "finish_reason": self.finish_reason,
            "duration_seconds": round(self.duration, 1),
            "metrics": {
                "laps_completed": self.metrics.laps_completed,
                "cones_hit": self.metrics.cones_hit,
                "total_time": round(self.metrics.total_time, 3),
                "best_lap_time": round(self.metrics.best_lap_time, 3),
                "mean_lap_time": round(self.metrics.mean_lap_time, 3),
                "mean_tracking_error": round(self.metrics.mean_tracking_error, 4),
                "peak_tracking_error": round(self.metrics.peak_tracking_error, 4),
                "mean_velocity_error": round(self.metrics.mean_velocity_error, 4),
            },
            "checks": [
                {
                    "name": check.name,
                    "limit": check.limit,
                    "value": round(check.value, 4),
                    "passed": check.passed,
                }
                for check in self.checks
            ],
            "failures": list(self.failures),
            "log": str(self.log_path) if self.log_path else None,
        }


def evaluate(scenario: Scenario, metrics: RunMetrics) -> List[CriterionCheck]:
    """Apply a scenario's criteria to the metrics of its run."""
    checks: List[CriterionCheck] = []
    for criterion, limit in sorted(scenario.criteria.items()):
        value = float(getattr(metrics, UPPER_BOUND_CRITERIA[criterion]))
        checks.append(
            CriterionCheck(name=criterion, limit=limit, value=value, passed=value <= limit)
        )
    return checks

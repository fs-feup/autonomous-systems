"""Loading and validation of the simulation scenario definitions."""

from __future__ import annotations

import copy
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

import yaml

#: Scenario criterion -> the RunMetrics field it bounds. Every criterion is an upper bound.
UPPER_BOUND_CRITERIA = {
    "max_cones_hit": "cones_hit",
    "max_total_time": "total_time",
    "max_best_lap_time": "best_lap_time",
    "max_mean_lap_time": "mean_lap_time",
    "max_mean_tracking_error": "mean_tracking_error",
    "max_peak_tracking_error": "peak_tracking_error",
    "max_mean_velocity_error": "mean_velocity_error",
}


class ScenarioError(Exception):
    """Raised when the scenario file is malformed."""


@dataclass(frozen=True)
class Scenario:
    """One full-stack simulation run and the criteria it has to satisfy."""

    name: str
    track: str
    mission: str
    min_laps: int
    stop_speed: float
    wait_for_stop: bool
    preloaded_map: bool
    timeout: float
    required: bool
    criteria: Dict[str, float]
    description: str = ""

    @property
    def summary(self) -> str:
        """One-line human description of what this scenario runs."""
        ending = "then stop" if self.wait_for_stop else "no stop needed"
        return f"{self.mission} on {self.track}, {self.min_laps} lap(s), {ending}"


@dataclass
class ScenarioFile:
    """The parsed contents of a scenario definition file."""

    path: Path
    scenarios: List[Scenario] = field(default_factory=list)

    def select(self, names: Optional[List[str]]) -> List[Scenario]:
        """Return the named scenarios, or all of them when no names are given."""
        if not names:
            return list(self.scenarios)

        by_name = {scenario.name: scenario for scenario in self.scenarios}
        unknown = [name for name in names if name not in by_name]
        if unknown:
            available = ", ".join(sorted(by_name))
            raise ScenarioError(
                f"Unknown scenario(s): {', '.join(unknown)}. Available: {available}"
            )
        return [by_name[name] for name in names]


def _merge_defaults(defaults: Dict[str, Any], entry: Dict[str, Any]) -> Dict[str, Any]:
    """Merge a scenario entry over the defaults, one level deep for ``criteria``."""
    merged = copy.deepcopy(defaults)
    for key, value in entry.items():
        if key == "criteria" and isinstance(value, dict):
            criteria = copy.deepcopy(merged.get("criteria", {}))
            for criterion, limit in value.items():
                if limit is None:
                    criteria.pop(criterion, None)
                else:
                    criteria[criterion] = limit
            merged["criteria"] = criteria
        else:
            merged[key] = value
    return merged


def _build_scenario(entry: Dict[str, Any], index: int) -> Scenario:
    """Turn one merged scenario mapping into a :class:`Scenario`."""
    missing = [key for key in ("name", "track", "mission") if not entry.get(key)]
    if missing:
        raise ScenarioError(f"Scenario #{index} is missing required key(s): {', '.join(missing)}")

    criteria = entry.get("criteria") or {}
    unknown_criteria = sorted(set(criteria) - set(UPPER_BOUND_CRITERIA))
    if unknown_criteria:
        raise ScenarioError(
            f"Scenario '{entry['name']}' has unknown criteria: {', '.join(unknown_criteria)}. "
            f"Available: {', '.join(sorted(UPPER_BOUND_CRITERIA))}"
        )

    min_laps = int(entry.get("min_laps", 1))
    if min_laps < 1:
        raise ScenarioError(
            f"Scenario '{entry['name']}' has min_laps={min_laps}; at least one lap is required "
            "or the run would end before the car moves."
        )

    timeout = float(entry.get("timeout", 600))
    if timeout <= 0:
        raise ScenarioError(f"Scenario '{entry['name']}' needs a positive timeout")

    return Scenario(
        name=str(entry["name"]),
        track=str(entry["track"]),
        mission=str(entry["mission"]),
        min_laps=min_laps,
        stop_speed=float(entry.get("stop_speed", 0.5)),
        wait_for_stop=bool(entry.get("wait_for_stop", True)),
        preloaded_map=bool(entry.get("preloaded_map", False)),
        timeout=timeout,
        required=bool(entry.get("required", True)),
        criteria={key: float(value) for key, value in criteria.items()},
        description=str(entry.get("description", "")),
    )


def load_scenarios(path: Path) -> ScenarioFile:
    """Read and validate a scenario definition file."""
    if not path.is_file():
        raise ScenarioError(f"Scenario file not found: {path}")

    document = yaml.safe_load(path.read_text()) or {}
    defaults = document.get("defaults") or {}
    entries = document.get("scenarios")

    if not entries:
        raise ScenarioError(f"No scenarios defined in {path}")

    scenarios = [
        _build_scenario(_merge_defaults(defaults, entry), index)
        for index, entry in enumerate(entries)
    ]

    names = [scenario.name for scenario in scenarios]
    duplicates = sorted({name for name in names if names.count(name) > 1})
    if duplicates:
        raise ScenarioError(f"Duplicate scenario name(s) in {path}: {', '.join(duplicates)}")

    return ScenarioFile(path=path, scenarios=scenarios)

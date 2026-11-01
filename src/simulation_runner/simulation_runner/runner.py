"""Execution of a single simulation scenario against the full InvictaSim stack."""

from __future__ import annotations

import os
import re
import shutil
import signal
import subprocess
import time
from pathlib import Path
from typing import List, Tuple

import rclpy
import yaml
from rclpy.executors import SingleThreadedExecutor

from .config_overrides import AppliedOverrides, simulation_overrides
from .monitor import RunMonitor
from .results import LapRecord, RunMetrics, ScenarioResult, evaluate
from .scenarios import Scenario

#: Brings up simulator, velocity estimation, SLAM, planning and control.
LAUNCH_PACKAGE = "launcher"
LAUNCH_FILE = "invictasim_full_stack.launch.py"

#: Killed before and after every run: a leftover node publishes onto the same topics.
STACK_PROCESSES = (
    "install/invictasim/lib",
    "install/velocity_estimation/lib",
    "install/slam/lib",
    "install/planning/lib",
    "install/control/lib",
    f"ros2 launch {LAUNCH_PACKAGE} {LAUNCH_FILE}",
)

#: How often the monitor is serviced while a run is in flight, in seconds.
_POLL_INTERVAL = 0.1

#: Exit codes of a normal shutdown; anything else is a crash.
_CRASH_PATTERN = re.compile(r"process has died \[pid \d+, exit code (-?\d+)")
_CLEAN_EXIT_CODES = {0, -2, -15}

#: Settings that make a run impossible, checked up front so the failure is a clear message.
INCOMPATIBLE_SETTINGS = (
    (
        "config/planning/invictasim.yaml",
        ("planning", "planning_using_full_map"),
        True,
        "planning waits for a complete map that only a mapping lap can produce, so the car "
        "never receives a path and never moves. Set planning_using_full_map to false.",
    ),
)


class RunnerError(Exception):
    """Raised when a scenario cannot be executed at all."""


def kill_stack_processes(settle_timeout: float = 10.0) -> None:
    """Hard kill any stack process still running, and wait for them to actually go."""
    for pattern in STACK_PROCESSES:
        subprocess.run(["pkill", "-9", "-f", pattern], check=False,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # Everything shares one ROS domain, so a process still dying would be discovered by the run
    # about to start.
    deadline = time.monotonic() + settle_timeout
    while True:
        survivors = [
            pattern for pattern in STACK_PROCESSES
            if subprocess.run(["pgrep", "-f", pattern], stdout=subprocess.DEVNULL,
                              stderr=subprocess.DEVNULL).returncode == 0
        ]
        if not survivors:
            return
        if time.monotonic() >= deadline:
            print(f"WARNING: stack processes survived being killed: {', '.join(survivors)}. "
                  "The next run shares a ROS domain with them.", flush=True)
            return
        time.sleep(0.2)


def preflight_check(workspace: Path) -> List[str]:
    """Report configuration values that would stop a full-stack run from working."""
    problems: List[str] = []
    for relative_path, key_path, bad_value, explanation in INCOMPATIBLE_SETTINGS:
        path = workspace / relative_path
        if not path.is_file():
            continue

        document = yaml.safe_load(path.read_text()) or {}
        node = document
        for key in key_path:
            if not isinstance(node, dict) or key not in node:
                node = None
                break
            node = node[key]

        if node == bad_value:
            problems.append(
                f"{relative_path}: {'.'.join(key_path)} is {bad_value!r}: {explanation}"
            )
    return problems


def find_crashes(log_text: str) -> List[str]:
    """Extract node crashes from a launch log."""
    crashes = [
        f"a node died with exit code {code}"
        for code in (int(match) for match in _CRASH_PATTERN.findall(log_text))
        if code not in _CLEAN_EXIT_CODES
    ]
    return sorted(set(crashes))


def _terminate(process: subprocess.Popen) -> None:
    """Shut a launch down, escalating to SIGKILL if it does not go quietly."""
    for sig, grace in ((signal.SIGINT, 15.0), (signal.SIGTERM, 10.0), (signal.SIGKILL, 5.0)):
        try:
            os.killpg(os.getpgid(process.pid), sig)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=grace)
            return
        except subprocess.TimeoutExpired:
            continue


def _drive_run(scenario: Scenario, workspace: Path, log_path: Path,
               context: rclpy.Context) -> Tuple[str, float, List[LapRecord], float]:
    """Launch the stack and watch it until the scenario's finish conditions are met."""
    environment = dict(os.environ)
    # The simulator opens an SDL window; automated runs have no display.
    environment.setdefault("SDL_VIDEODRIVER", "dummy")

    monitor = RunMonitor(
        min_laps=scenario.min_laps,
        stop_speed=scenario.stop_speed,
        wait_for_stop=scenario.wait_for_stop,
        context=context,
    )
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(monitor)

    started = time.monotonic()
    with log_path.open("w") as log_file:
        process = subprocess.Popen(
            ["ros2", "launch", LAUNCH_PACKAGE, LAUNCH_FILE],
            cwd=workspace,
            env=environment,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL,
            start_new_session=True,
        )

        try:
            while True:
                executor.spin_once(timeout_sec=_POLL_INTERVAL)

                reason = monitor.finish_reason()
                if reason is not None:
                    break
                if process.poll() is not None:
                    reason = "aborted"
                    break
                if time.monotonic() - started >= scenario.timeout:
                    reason = "timeout"
                    break

            laps = monitor.laps
            peak_speed = monitor.peak_speed
            duration = time.monotonic() - started
        finally:
            executor.remove_node(monitor)
            monitor.destroy_node()
            if process.poll() is None:
                _terminate(process)

    return reason, duration, laps, peak_speed


def run_scenario(scenario: Scenario, workspace: Path, output_directory: Path,
                 context: rclpy.Context) -> ScenarioResult:
    """Execute one scenario and evaluate it against its criteria."""
    if shutil.which("ros2") is None:
        raise RunnerError("ros2 is not on PATH; source the ROS and workspace setup files first")

    output_directory.mkdir(parents=True, exist_ok=True)
    log_path = output_directory / "launch.log"

    overrides = simulation_overrides(
        track=scenario.track,
        mission=scenario.mission,
        preloaded_map=scenario.preloaded_map,
    )

    kill_stack_processes()
    try:
        with AppliedOverrides(workspace, overrides):
            finish_reason, duration, laps, peak_speed = _drive_run(
                scenario, workspace, log_path, context)
    finally:
        kill_stack_processes()

    metrics = RunMetrics.from_laps(laps)
    checks = evaluate(scenario, metrics)

    failures = [
        f"{check.name}: {check.value:.3f} exceeds the limit of {check.limit:g}"
        for check in checks
        if not check.passed
    ]
    failures.extend(find_crashes(log_path.read_text() if log_path.is_file() else ""))

    if finish_reason == "timeout":
        failures.append(
            f"timed out after {scenario.timeout:g}s of wall clock, peak speed "
            f"{peak_speed:.2f} m/s"
        )
    elif finish_reason == "aborted":
        failures.append("the stack exited before the run reached its finish conditions")

    if metrics.laps_completed < scenario.min_laps:
        failures.append(
            f"completed {metrics.laps_completed} of the {scenario.min_laps} required lap(s)"
        )

    return ScenarioResult(
        scenario=scenario,
        finish_reason=finish_reason,
        duration=duration,
        laps=laps,
        metrics=metrics,
        checks=checks,
        failures=failures,
        log_path=log_path,
    )

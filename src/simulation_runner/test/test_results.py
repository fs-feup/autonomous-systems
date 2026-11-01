"""Tests for metric aggregation and criteria evaluation."""

from types import SimpleNamespace

from simulation_runner.results import LapRecord, RunMetrics, evaluate
from simulation_runner.scenarios import Scenario


def _scenario(**criteria):
    return Scenario(
        name="trackdrive", track="FSG25", mission="trackdrive", min_laps=2,
        stop_speed=0.5, wait_for_stop=True, preloaded_map=False, timeout=500.0,
        required=True, criteria=criteria,
    )


def test_metrics_are_aggregated_across_laps():
    laps = [
        LapRecord(1, 35.5, 1, 0.08, 0.99, 2.5),
        LapRecord(2, 33.9, 0, 0.10, 1.20, 2.6),
    ]

    metrics = RunMetrics.from_laps(laps)

    assert metrics.laps_completed == 2
    assert metrics.cones_hit == 1, "cone hits accumulate over the whole run"
    assert metrics.best_lap_time == 33.9
    assert metrics.total_time == 69.4
    assert metrics.peak_tracking_error == 1.20


def test_metrics_of_a_run_that_never_finished_a_lap():
    metrics = RunMetrics.from_laps([])

    assert metrics.laps_completed == 0
    assert metrics.best_lap_time == 0.0


def test_lap_records_are_built_from_the_published_statistics():
    row = SimpleNamespace(
        lap_number=2, time=33.9, cones_hit=1, total_time=69.4, best_time=33.9,
        avg_velocity=35.1, max_velocity=62.5, avg_tracking_error_distance=0.10,
        max_tracking_error_distance=1.20, avg_velocity_error=2.6, max_velocity_error=6.0,
    )

    lap = LapRecord.from_lap_statistics(row)

    assert lap.lap == 2
    assert lap.cones_hit == 1
    assert lap.peak_tracking_error == 1.20


def test_criteria_are_upper_bounds():
    metrics = RunMetrics.from_laps([LapRecord(1, 35.5, 1, 0.08, 0.99, 2.5)])

    checks = {check.name: check for check in evaluate(_scenario(max_cones_hit=0), metrics)}
    assert checks["max_cones_hit"].passed is False

    checks = {check.name: check for check in evaluate(_scenario(max_cones_hit=2), metrics)}
    assert checks["max_cones_hit"].passed is True


def test_a_scenario_without_criteria_produces_no_checks():
    assert evaluate(_scenario(), RunMetrics()) == []


def test_killing_the_stack_reports_processes_that_survive(capsys):
    """A survivor would be discovered by the run about to start, so it must be visible."""
    from unittest import mock

    from simulation_runner import runner

    alive = mock.Mock(returncode=0)
    with mock.patch.object(runner.subprocess, "run", return_value=alive), \
            mock.patch.object(runner.time, "sleep"):
        runner.kill_stack_processes(settle_timeout=0.0)

    assert "survived being killed" in capsys.readouterr().out

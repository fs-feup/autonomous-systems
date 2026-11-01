"""Tests for the finish logic the runner applies to a live run."""

from types import SimpleNamespace

import pytest

rclpy = pytest.importorskip("rclpy")

from simulation_runner.monitor import RunMonitor  # noqa: E402


def _lap(lap_number):
    return SimpleNamespace(
        lap_number=lap_number, time=33.9, cones_hit=0, total_time=33.9, best_time=33.9,
        avg_velocity=35.1, max_velocity=62.5, avg_tracking_error_distance=0.1,
        max_tracking_error_distance=1.0, avg_velocity_error=2.6, max_velocity_error=6.0,
    )


@pytest.fixture
def monitor_factory():
    context = rclpy.Context()
    context.init()
    created = []

    def make(min_laps=1, stop_speed=0.5, wait_for_stop=True):
        monitor = RunMonitor(min_laps, stop_speed, wait_for_stop, context=context)
        created.append(monitor)
        return monitor

    yield make

    for monitor in created:
        monitor.destroy_node()
    context.try_shutdown()


def _drive(monitor, speed):
    monitor._vehicle_status_callback(SimpleNamespace(velocity_x=speed, velocity_y=0.0))


def _complete_laps(monitor, count):
    monitor._lap_summary_callback(SimpleNamespace(rows=[_lap(i + 1) for i in range(count)]))


def test_a_run_does_not_end_before_the_required_laps(monitor_factory):
    monitor = monitor_factory(min_laps=2)
    _complete_laps(monitor, 1)
    _drive(monitor, 10.0)
    _drive(monitor, 0.0)

    assert monitor.finish_reason() is None


def test_a_run_ends_once_the_car_stops_after_the_required_laps(monitor_factory):
    monitor = monitor_factory(min_laps=1)
    _complete_laps(monitor, 1)
    _drive(monitor, 10.0)
    assert monitor.finish_reason() is None, "still driving"

    _drive(monitor, 0.1)
    assert monitor.finish_reason() == "completed"


def test_a_stationary_start_does_not_end_the_run(monitor_factory):
    """The car begins at a standstill, which must not count as having stopped."""
    monitor = monitor_factory(min_laps=1)
    _complete_laps(monitor, 1)
    _drive(monitor, 0.0)

    assert monitor.finish_reason() is None
    assert monitor.started_moving is False


def test_without_wait_for_stop_the_lap_count_is_enough(monitor_factory):
    monitor = monitor_factory(min_laps=1, wait_for_stop=False)
    _complete_laps(monitor, 1)

    assert monitor.finish_reason() == "completed"


def test_laps_are_taken_from_the_published_history(monitor_factory):
    monitor = monitor_factory(min_laps=3)
    _complete_laps(monitor, 3)

    assert [lap.lap for lap in monitor.laps] == [1, 2, 3]

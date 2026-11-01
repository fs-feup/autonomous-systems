"""Tests for scenario loading and validation."""

import pytest

from simulation_runner.scenarios import ScenarioError, load_scenarios

VALID = """
defaults:
  stop_speed: 0.5
  timeout: 300
  criteria:
    max_cones_hit: 0
scenarios:
  - name: acceleration
    track: Acceleration
    mission: acceleration
    min_laps: 1
  - name: trackdrive
    track: FSG25
    mission: trackdrive
    min_laps: 4
    timeout: 500
    criteria:
      max_mean_lap_time: 40.0
"""


def _write(tmp_path, text):
    path = tmp_path / "scenarios.yaml"
    path.write_text(text)
    return path


def test_defaults_are_merged_into_every_scenario(tmp_path):
    scenarios = load_scenarios(_write(tmp_path, VALID)).scenarios

    assert [scenario.name for scenario in scenarios] == ["acceleration", "trackdrive"]
    assert scenarios[0].stop_speed == 0.5
    assert scenarios[0].timeout == 300
    assert scenarios[1].timeout == 500, "a scenario must be able to override a default"


def test_criteria_are_merged_rather_than_replaced(tmp_path):
    trackdrive = load_scenarios(_write(tmp_path, VALID)).scenarios[1]

    assert trackdrive.criteria == {"max_cones_hit": 0.0, "max_mean_lap_time": 40.0}


def test_selecting_scenarios_by_name(tmp_path):
    scenario_file = load_scenarios(_write(tmp_path, VALID))

    assert [scenario.name for scenario in scenario_file.select(["trackdrive"])] == ["trackdrive"]
    assert len(scenario_file.select(None)) == 2

    with pytest.raises(ScenarioError, match="Unknown scenario"):
        scenario_file.select(["does_not_exist"])


def test_unknown_criterion_is_rejected(tmp_path):
    text = VALID.replace("max_mean_lap_time: 40.0", "max_altitude: 40.0")

    with pytest.raises(ScenarioError, match="unknown criteria"):
        load_scenarios(_write(tmp_path, text))


def test_zero_laps_is_rejected(tmp_path):
    text = VALID.replace("min_laps: 1", "min_laps: 0")

    with pytest.raises(ScenarioError, match="min_laps"):
        load_scenarios(_write(tmp_path, text))


def test_duplicate_names_are_rejected(tmp_path):
    text = VALID.replace("name: trackdrive", "name: acceleration")

    with pytest.raises(ScenarioError, match="Duplicate scenario name"):
        load_scenarios(_write(tmp_path, text))


def test_missing_file_is_reported(tmp_path):
    with pytest.raises(ScenarioError, match="not found"):
        load_scenarios(tmp_path / "absent.yaml")


def test_repository_scenarios_are_valid():
    """The scenarios actually used by CI have to parse."""
    from pathlib import Path

    repository_file = Path(__file__).resolve().parents[3] / "config/simulation/scenarios.yaml"
    if not repository_file.is_file():
        pytest.skip("running outside the repository")

    scenarios = load_scenarios(repository_file).scenarios
    names = {scenario.name for scenario in scenarios}
    assert {"acceleration", "skidpad", "autocross", "trackdrive"} <= names


def test_a_scenario_can_drop_an_inherited_criterion(tmp_path):
    text = VALID.replace("max_mean_lap_time: 40.0", "max_cones_hit: null")

    trackdrive = load_scenarios(_write(tmp_path, text)).scenarios[1]

    assert trackdrive.criteria == {}, "a null criterion removes the default, not overrides it"

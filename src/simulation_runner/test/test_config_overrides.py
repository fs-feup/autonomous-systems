"""Tests for the temporary configuration edits a run needs."""

import pytest

from simulation_runner.config_overrides import (
    AppliedOverrides,
    ConfigOverrideError,
    Override,
    set_yaml_value,
    simulation_overrides,
)

DOCUMENT = """global:
  adapter: "vehicle" # Options: "vehicle", "pacsim" or "invictasim"
  use_simulated_se: true # use simulated state estimator
  nested:
    value: 1.0    # a nested scalar
other:
  adapter: "keep me"
"""


def test_value_is_replaced_and_comment_kept():
    result = set_yaml_value(DOCUMENT, ("global", "adapter"), "invictasim")

    assert '  adapter: "invictasim" # Options: "vehicle", "pacsim" or "invictasim"' in result
    assert '  adapter: "keep me"' in result, "a same-named key elsewhere must not be touched"


def test_booleans_use_yaml_spelling():
    result = set_yaml_value(DOCUMENT, ("global", "use_simulated_se"), False)

    assert "  use_simulated_se: false # use simulated state estimator" in result


def test_nested_keys_are_addressable():
    result = set_yaml_value(DOCUMENT, ("global", "nested", "value"), 2.5)

    assert "    value: 2.5    # a nested scalar" in result


def test_missing_key_is_reported():
    with pytest.raises(ConfigOverrideError, match="not found"):
        set_yaml_value(DOCUMENT, ("global", "absent"), 1)


def test_overrides_are_reverted_on_exit(tmp_path):
    config = tmp_path / "config" / "global"
    config.mkdir(parents=True)
    (config / "global_config.yaml").write_text(DOCUMENT)

    override = Override("config/global/global_config.yaml", ("global", "adapter"), "invictasim")

    with AppliedOverrides(tmp_path, [override]):
        assert 'adapter: "invictasim"' in (config / "global_config.yaml").read_text()

    assert (config / "global_config.yaml").read_text() == DOCUMENT


def test_overrides_are_reverted_when_the_body_raises(tmp_path):
    config = tmp_path / "config" / "global"
    config.mkdir(parents=True)
    (config / "global_config.yaml").write_text(DOCUMENT)

    override = Override("config/global/global_config.yaml", ("global", "adapter"), "invictasim")

    with pytest.raises(RuntimeError):
        with AppliedOverrides(tmp_path, [override]):
            raise RuntimeError("boom")

    assert (config / "global_config.yaml").read_text() == DOCUMENT


def test_only_setup_parameters_are_overridden():
    """Performance tuning belongs to the pull request, so it must never be rewritten."""
    touched = {
        (override.relative_path, override.key_path)
        for override in simulation_overrides("FSG25", "trackdrive")
    }
    touched_files = {path for path, _ in touched}

    assert touched_files == {
        "config/global/global_config.yaml",
        "config/invictasim/global.yaml",
        "config/slam/invictasim.yaml",
    }
    assert ("config/global/global_config.yaml", ("global", "use_simulated_se")) in touched
    assert ("config/invictasim/global.yaml", ("invictasim", "track_name")) in touched
    assert not any("finish_conditions" in key_path for _, key_path in touched), (
        "when a run ends is the runner's business, not something written into the simulator"
    )

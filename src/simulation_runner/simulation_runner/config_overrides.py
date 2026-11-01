"""Temporary, comment-preserving edits to the repository configuration files."""

from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Sequence, Tuple

#: Backup written while overrides are applied, so a killed run can be recovered on the next start.
BACKUP_SUFFIX = ".simulation_runner_backup"

_KEY_PATTERN = re.compile(r"^(\s*)([A-Za-z_][A-Za-z0-9_-]*):(.*)$")
_COMMENT_PATTERN = re.compile(r"(\s+#.*)$")


class ConfigOverrideError(Exception):
    """Raised when a configuration key the runner depends on cannot be found."""


@dataclass(frozen=True)
class Override:
    """A single value to write into a configuration file."""

    relative_path: str
    key_path: Tuple[str, ...]
    value: Any


def format_scalar(value: Any) -> str:
    """Render a Python value the way these YAML files write it."""
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (int, float)):
        return repr(value)
    return f'"{value}"'


def set_yaml_value(text: str, key_path: Sequence[str], value: Any) -> str:
    """Replace one scalar in a YAML document, leaving the rest of the file byte for byte intact."""
    lines = text.splitlines(keepends=True)
    path = tuple(key_path)
    stack: List[Tuple[int, str]] = []

    for index, raw_line in enumerate(lines):
        line = raw_line.rstrip("\n")
        if not line.strip() or line.strip().startswith("#"):
            continue

        match = _KEY_PATTERN.match(line)
        if not match:
            continue

        indent, key, remainder = len(match.group(1)), match.group(2), match.group(3)
        while stack and stack[-1][0] >= indent:
            stack.pop()
        stack.append((indent, key))

        if tuple(entry[1] for entry in stack) != path:
            continue

        comment_match = _COMMENT_PATTERN.search(remainder)
        comment = comment_match.group(1) if comment_match else ""
        newline = "\n" if raw_line.endswith("\n") else ""
        lines[index] = f"{' ' * indent}{key}: {format_scalar(value)}{comment}{newline}"
        return "".join(lines)

    raise ConfigOverrideError(f"Key '{'.'.join(path)}' not found")


def simulation_overrides(track: str, mission: str, preloaded_map: bool = False) -> List[Override]:
    """Build the set of setup overrides a scenario needs."""
    global_config = "config/global/global_config.yaml"
    simulator_config = "config/invictasim/global.yaml"
    slam_config = "config/slam/invictasim.yaml"

    return [
        # Point the whole stack at the simulator.
        Override(global_config, ("global", "adapter"), "invictasim"),
        Override(global_config, ("global", "discipline"), mission),
        # Exercise the real SLAM and velocity estimator, fed by the simulated sensors.
        Override(global_config, ("global", "use_simulated_se"), False),
        Override(global_config, ("global", "use_simulated_velocities"), False),
        Override(global_config, ("global", "use_simulated_perception"), True),
        Override(global_config, ("global", "use_simulated_planning"), False),
        # Simulator run setup.
        Override(simulator_config, ("invictasim", "track_name"), track),
        Override(simulator_config, ("invictasim", "control_mode"), "autonomous"),
        Override(simulator_config, ("invictasim", "input_adapter"), "ros"),
        Override(simulator_config, ("invictasim", "output_adapter"), "ros"),
        Override(simulator_config, ("invictasim", "initial_go_signal"), True),
        # Skidpad and acceleration run on a known layout; the maps live in src/slam/maps/.
        Override(slam_config, ("slam", "using_preloaded_map"), preloaded_map),
    ]


class AppliedOverrides:
    """Context manager applying overrides to the working tree and restoring them afterwards."""

    def __init__(self, workspace: Path, overrides: Sequence[Override]) -> None:
        """Apply the given overrides under the repository at workspace."""
        self._workspace = workspace
        self._overrides = list(overrides)
        self._originals: Dict[Path, str] = {}

    def __enter__(self) -> "AppliedOverrides":
        restore_stale_backups(self._workspace)

        by_file: Dict[Path, List[Override]] = {}
        for override in self._overrides:
            by_file.setdefault(self._workspace / override.relative_path, []).append(override)

        try:
            for path, overrides in by_file.items():
                if not path.is_file():
                    raise ConfigOverrideError(f"Configuration file not found: {path}")

                original = path.read_text()
                self._originals[path] = original
                path.with_suffix(path.suffix + BACKUP_SUFFIX).write_text(original)

                text = original
                for override in overrides:
                    try:
                        text = set_yaml_value(text, override.key_path, override.value)
                    except ConfigOverrideError as error:
                        raise ConfigOverrideError(f"{path}: {error}") from error
                path.write_text(text)
        except Exception:
            self.restore()
            raise

        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        """Restore every file the context modified."""
        self.restore()

    def restore(self) -> None:
        """Put the original contents back and drop the on-disk backups."""
        for path, original in self._originals.items():
            path.write_text(original)
        self._originals.clear()

        for override in self._overrides:
            backup = self._workspace / (override.relative_path + BACKUP_SUFFIX)
            backup.unlink(missing_ok=True)


def restore_stale_backups(workspace: Path) -> List[Path]:
    """Restore configuration files left modified by a run that was killed."""
    restored: List[Path] = []
    for backup in sorted((workspace / "config").rglob(f"*{BACKUP_SUFFIX}")):
        original = backup.with_name(backup.name[: -len(BACKUP_SUFFIX)])
        original.write_text(backup.read_text())
        backup.unlink()
        restored.append(original)
    return restored

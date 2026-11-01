"""Rendering of simulation results for humans and for machines."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Dict, List

from .results import ScenarioResult

_STATUS_ICON = {True: "PASS", False: "FAIL"}


def _format_number(value: float) -> str:
    """Render a metric compactly, dropping the decimals of whole numbers."""
    if value == int(value):
        return str(int(value))
    return f"{value:.3f}".rstrip("0").rstrip(".")


def _headline(results: List[ScenarioResult]) -> str:
    """Summarise the outcome, distinguishing failures that block a merge from the rest."""
    blocking = [r for r in results if not r.passed and r.scenario.required]
    optional = [r for r in results if not r.passed and not r.scenario.required]

    if blocking:
        headline = (
            f"{len(blocking)} of {len(results)} scenarios failed and block merging: "
            f"{', '.join(r.scenario.name for r in blocking)}."
        )
    else:
        headline = "Every required scenario passed."

    if optional:
        headline += (
            f" {len(optional)} optional scenario(s) also failed "
            f"({', '.join(r.scenario.name for r in optional)}), which does not block merging."
        )
    return headline


def render_markdown(results: List[ScenarioResult]) -> str:
    """Render the results as a markdown report."""
    lines = [
        "## Simulation results",
        "",
        _headline(results),
        "",
        "| Scenario | Result | Laps | Cones | Best lap | Mean lap | Mean error | Peak error |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]

    for result in results:
        metrics = result.metrics
        name = result.scenario.name if result.scenario.required else f"{result.scenario.name} *"
        lines.append(
            f"| {name} | {_STATUS_ICON[result.passed]} "
            f"| {metrics.laps_completed} | {metrics.cones_hit} "
            f"| {_format_number(metrics.best_lap_time)} s "
            f"| {_format_number(metrics.mean_lap_time)} s "
            f"| {_format_number(metrics.mean_tracking_error)} m "
            f"| {_format_number(metrics.peak_tracking_error)} m |"
        )

    if any(not result.scenario.required for result in results):
        lines.extend(["", "`*` optional scenario, its result does not block merging."])

    failed = [result for result in results if not result.passed]
    if failed:
        lines.extend(["", "### Failures", ""])
        for result in failed:
            optional = "" if result.scenario.required else " - optional, does not block merging"
            lines.append(f"**{result.scenario.name}** ({result.scenario.summary}){optional}")
            lines.extend(f"- {failure}" for failure in result.failures)
            lines.append("")

    lines.extend(["", "<details><summary>Criteria checked</summary>", ""])
    for result in results:
        lines.append(f"**{result.scenario.name}** - finished as `{result.finish_reason}` "
                     f"in {result.duration:.0f}s of wall clock")
        if result.checks:
            lines.append("")
            lines.append("| Criterion | Limit | Value | Result |")
            lines.append("| --- | ---: | ---: | --- |")
            for check in result.checks:
                lines.append(
                    f"| {check.name} | {_format_number(check.limit)} "
                    f"| {_format_number(check.value)} | {_STATUS_ICON[check.passed]} |"
                )
        lines.append("")
    lines.append("</details>")

    return "\n".join(lines) + "\n"


def render_console(results: List[ScenarioResult]) -> str:
    """Render a compact plain-text summary for the terminal."""
    lines = []
    for result in results:
        metrics = result.metrics
        lines.append(
            f"{_STATUS_ICON[result.passed]:4}  {result.scenario.name:<14} "
            f"laps={metrics.laps_completed:<3} cones={metrics.cones_hit:<3} "
            f"best={_format_number(metrics.best_lap_time)}s "
            f"mean_err={_format_number(metrics.mean_tracking_error)}m "
            f"({result.finish_reason})"
        )
        lines.extend(f"        - {failure}" for failure in result.failures)
    return "\n".join(lines)


def build_json(results: List[ScenarioResult]) -> Dict:
    """Build the machine-readable report."""
    return {
        "passed": all(result.passed for result in results),
        "scenarios": [result.as_dict() for result in results],
    }


def write_reports(results: List[ScenarioResult], output_directory: Path) -> Dict[str, Path]:
    """Write the JSON and markdown reports into the output directory."""
    output_directory.mkdir(parents=True, exist_ok=True)

    json_path = output_directory / "results.json"
    json_path.write_text(json.dumps(build_json(results), indent=2) + "\n")

    markdown_path = output_directory / "summary.md"
    markdown_path.write_text(render_markdown(results))

    return {"json": json_path, "markdown": markdown_path}

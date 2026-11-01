"""Command line entry point that runs the simulation scenarios and reports on them."""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path
from typing import List, Optional

import rclpy

from .config_overrides import restore_stale_backups
from .notifications import (
    WEBHOOK_ENVIRONMENT_VARIABLE,
    WEBHOOK_FILE,
    NotificationError,
    RunContext,
    notify,
    resolve_webhook,
)
from .report import render_console, render_markdown, write_reports
from .results import ScenarioResult
from .runner import RunnerError, kill_stack_processes, preflight_check, run_scenario
from .scenarios import ScenarioError, load_scenarios

DEFAULT_SCENARIO_FILE = Path("config/simulation/scenarios.yaml")
DEFAULT_OUTPUT_DIRECTORY = Path("simulation_results")
DEFAULT_DISCORD_USERS_FILE = Path("config/simulation/discord_users.yaml")

#: Domain the stack runs on, matching the rest of the team's tooling.
DEFAULT_DOMAIN_ID = 42


def _parse_arguments(argv: Optional[List[str]]) -> argparse.Namespace:
    """Build and parse the command line."""
    parser = argparse.ArgumentParser(
        prog="simulation_runner",
        description=(
            "Run the InvictaSim full stack through the configured scenarios and check the "
            "resulting driving performance against their criteria."
        ),
    )
    parser.add_argument(
        "--workspace", type=Path, default=Path.cwd(),
        help="Repository root (default: the current directory)",
    )
    parser.add_argument(
        "--scenarios", type=Path, default=None,
        help=f"Scenario definition file (default: {DEFAULT_SCENARIO_FILE})",
    )
    parser.add_argument(
        "--output", type=Path, default=None,
        help=f"Directory for logs and reports (default: {DEFAULT_OUTPUT_DIRECTORY})",
    )
    parser.add_argument(
        "--only", nargs="+", metavar="NAME", default=None,
        help="Run only the named scenarios instead of all of them",
    )
    parser.add_argument(
        "--list", action="store_true",
        help="List the defined scenarios and exit",
    )
    parser.add_argument(
        "--keep-going", action="store_true",
        help="Run every scenario even after one fails (default: stop at the first failure)",
    )
    parser.add_argument(
        "--ros-domain-id", type=int, default=DEFAULT_DOMAIN_ID,
        help=f"ROS domain id the stack runs on (default: {DEFAULT_DOMAIN_ID})",
    )

    discord = parser.add_argument_group("Discord notification")
    discord.add_argument("--author", default="", help="GitHub login or email of the author")
    discord.add_argument("--branch", default="", help="Branch the run belongs to")
    discord.add_argument("--pull-request", default="", help="Pull request the run belongs to")
    discord.add_argument("--run-url", default="", help="Link included in the message")
    discord.add_argument(
        "--discord-users", type=Path, default=None,
        help=f"GitHub to Discord user mapping (default: {DEFAULT_DISCORD_USERS_FILE})",
    )
    discord.add_argument(
        "--discord-webhook-url", default="",
        help=f"Webhook to post to (default: ${WEBHOOK_ENVIRONMENT_VARIABLE}, else {WEBHOOK_FILE})",
    )
    discord.add_argument(
        "--discord-dry-run", action="store_true",
        help="Print the Discord message instead of posting it",
    )
    return parser.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    """Run the scenarios and return the process exit code."""
    arguments = _parse_arguments(argv)
    workspace = arguments.workspace.resolve()
    scenario_path = arguments.scenarios or workspace / DEFAULT_SCENARIO_FILE
    output_directory = (arguments.output or workspace / DEFAULT_OUTPUT_DIRECTORY).resolve()

    try:
        scenario_file = load_scenarios(scenario_path)
        scenarios = scenario_file.select(arguments.only)
    except ScenarioError as error:
        print(f"Scenario configuration error: {error}", file=sys.stderr)
        return 2

    # A killed run leaves the configuration rewritten; put it back before doing anything else.
    restored = restore_stale_backups(workspace)
    for path in restored:
        print(f"Restored configuration left behind by an interrupted run: {path}")

    if arguments.list:
        for scenario in scenarios:
            required = "required" if scenario.required else "optional"
            print(f"{scenario.name:<14} {scenario.summary:<34} {required}")
        return 0

    problems = preflight_check(workspace)
    if problems:
        print("The committed configuration cannot run the full stack:", file=sys.stderr)
        for problem in problems:
            print(f"  - {problem}", file=sys.stderr)
        _notify_discord(arguments, workspace, [])
        return 2

    os.environ["ROS_DOMAIN_ID"] = str(arguments.ros_domain_id)
    print(f"Running on ROS_DOMAIN_ID={arguments.ros_domain_id}", flush=True)

    context = rclpy.Context()
    context.init()

    results: List[ScenarioResult] = []
    try:
        for scenario in scenarios:
            print(f"\n=== {scenario.name}: {scenario.summary} ===", flush=True)
            try:
                result = run_scenario(
                    scenario, workspace, output_directory / scenario.name, context
                )
            except RunnerError as error:
                print(f"Could not run scenario '{scenario.name}': {error}", file=sys.stderr)
                _notify_discord(arguments, workspace, results)
                return 2

            results.append(result)
            print(render_console([result]), flush=True)

            if not result.passed and scenario.required and not arguments.keep_going:
                print("\nStopping after the first required failure "
                      "(pass --keep-going to run the rest).", flush=True)
                break
    finally:
        kill_stack_processes()
        context.try_shutdown()

    reports = write_reports(results, output_directory)
    print(f"\nReports written to {reports['json']} and {reports['markdown']}")

    summary_file = os.environ.get("GITHUB_STEP_SUMMARY")
    if summary_file:
        with open(summary_file, "a") as handle:
            handle.write(render_markdown(results))

    _notify_discord(arguments, workspace, results)

    failed = [result for result in results if not result.passed and result.scenario.required]
    if failed:
        print(f"\n{len(failed)} required scenario(s) failed: "
              f"{', '.join(result.scenario.name for result in failed)}")
        return 1

    if len(results) < len(scenarios):
        print("\nNot every scenario ran.")
        return 1

    print("\nEvery required scenario passed.")
    return 0


def _notify_discord(arguments: argparse.Namespace, workspace: Path,
                    results: List[ScenarioResult]) -> None:
    """Post the run's outcome to Discord, never failing the run if that does not work."""
    user_map_path = arguments.discord_users or workspace / DEFAULT_DISCORD_USERS_FILE
    context = RunContext(
        author=arguments.author,
        branch=arguments.branch,
        pull_request=arguments.pull_request,
        url=arguments.run_url,
    )

    try:
        message = notify(
            results, context, user_map_path,
            resolve_webhook(arguments.discord_webhook_url), arguments.discord_dry_run,
        )
    except NotificationError as error:
        print(f"Discord notification failed: {error}", file=sys.stderr)
        return

    if arguments.discord_dry_run:
        print("\nDiscord message (not sent):\n" + (message or ""))
    elif message:
        print("\nPosted the results to Discord.")


if __name__ == "__main__":
    sys.exit(main())

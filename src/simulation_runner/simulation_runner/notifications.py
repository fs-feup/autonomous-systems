"""Discord notification for a finished simulation run."""

from __future__ import annotations

import json
import os
import urllib.error
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional

import yaml

from .results import ScenarioResult

#: Discord rejects messages longer than this.
MESSAGE_LIMIT = 2000

#: Environment variable holding the webhook the message is posted to.
WEBHOOK_ENVIRONMENT_VARIABLE = "DISCORD_WEBHOOK_URL"

#: Read when the environment variable is unset, so a local webhook survives a reboot.
WEBHOOK_FILE = Path.home() / ".config" / "invictasim" / "discord_webhook"

#: Discord rejects the default urllib user agent with 403, so send a descriptive one.
USER_AGENT = "InvictaSimSimulationRunner (https://github.com/, 1.0)"


class NotificationError(Exception):
    """Raised when a notification could not be delivered."""


@dataclass(frozen=True)
class RunContext:
    """Who and what a run belongs to, used to address the message."""

    author: str = ""
    branch: str = ""
    pull_request: str = ""
    url: str = ""


def resolve_webhook(explicit: str = "") -> str:
    """Find the webhook: the argument, then the environment, then the local file."""
    if explicit:
        return explicit

    from_environment = os.environ.get(WEBHOOK_ENVIRONMENT_VARIABLE, "").strip()
    if from_environment:
        return from_environment

    if WEBHOOK_FILE.is_file():
        return WEBHOOK_FILE.read_text().strip()
    return ""


def load_user_map(path: Path) -> Dict[str, str]:
    """Read the GitHub identity to Discord user id mapping, keyed case insensitively."""
    if not path.is_file():
        return {}

    document = yaml.safe_load(path.read_text()) or {}
    users = document.get("users") or {}
    return {str(key).strip().lower(): str(value).strip() for key, value in users.items()}


def mention_for(author: str, user_map: Dict[str, str]) -> str:
    """Return the Discord mention for an author, or their plain name when unmapped."""
    if not author:
        return "Someone"

    entry = user_map.get(author.strip().lower())
    if not entry:
        return author
    # Only a numeric user id pings; anything else is shown as text rather than a broken mention.
    return f"<@{entry}>" if entry.isdigit() else f"@{entry}"


def _scenario_line(result: ScenarioResult) -> str:
    """Render one scenario as a single line."""
    metrics = result.metrics
    status = "PASS" if result.passed else "FAIL"
    cones = "1 cone" if metrics.cones_hit == 1 else f"{metrics.cones_hit} cones"

    detail = f"{metrics.total_time:.2f}s, {cones}"
    if result.finish_reason in ("timeout", "aborted"):
        detail = f"{detail} ({result.finish_reason})"
    if not result.scenario.required:
        detail = f"{detail} [optional]"

    return f"{status:4}  {result.scenario.name:<13} {detail}"


def build_message(results: List[ScenarioResult], context: RunContext,
                  user_map: Dict[str, str]) -> str:
    """Build the Discord message for a finished run."""
    blocking = [r for r in results if not r.passed and r.scenario.required]
    optional = [r for r in results if not r.passed and not r.scenario.required]

    if not results:
        headline = "**FAILED**, no scenario ran"
    elif blocking:
        headline = "**FAILED**"
    elif optional:
        headline = "**PASSED**, but optional scenarios failed"
    else:
        headline = "**PASSED**"

    where = context.pull_request or context.branch
    subject = f"Pull request {where}" if context.pull_request else (where or "Your changes")

    lines = [
        f"{mention_for(context.author, user_map)} {subject} has been tested: {headline}",
        "```",
        *[_scenario_line(result) for result in results],
        "```",
    ]

    for result in blocking + optional:
        for failure in result.failures:
            lines.append(f"- `{result.scenario.name}`: {failure}")

    if context.url:
        lines.append(context.url)

    message = "\n".join(lines)
    if len(message) > MESSAGE_LIMIT:
        message = message[: MESSAGE_LIMIT - 3] + "..."
    return message


def send_message(webhook_url: str, message: str, timeout: float = 15.0) -> None:
    """Post a message to a Discord webhook."""
    payload = json.dumps({"content": message}).encode()
    request = urllib.request.Request(
        webhook_url,
        data=payload,
        headers={"Content-Type": "application/json", "User-Agent": USER_AGENT},
        method="POST",
    )

    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            if response.status >= 300:
                raise NotificationError(f"Discord replied with status {response.status}")
    except urllib.error.HTTPError as error:
        detail = error.read().decode(errors="replace").strip()
        raise NotificationError(f"Discord rejected the message ({error.code}): {detail}") from error
    except urllib.error.URLError as error:
        raise NotificationError(f"Could not reach Discord: {error.reason}") from error
    except OSError as error:
        # Timeouts are not URLErrors, and must not take the run down with them.
        raise NotificationError(f"Could not reach Discord: {error}") from error


def notify(results: List[ScenarioResult], context: RunContext, user_map_path: Path,
           webhook_url: Optional[str], dry_run: bool = False) -> Optional[str]:
    """Build and deliver the Discord message, returning what was sent."""
    message = build_message(results, context, load_user_map(user_map_path))

    if dry_run:
        return message
    if not webhook_url:
        return None

    send_message(webhook_url, message)
    return message

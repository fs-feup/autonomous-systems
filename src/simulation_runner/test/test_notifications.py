"""Tests for the Discord notification."""

import json
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

import pytest

from simulation_runner.notifications import (
    MESSAGE_LIMIT,
    NotificationError,
    RunContext,
    build_message,
    load_user_map,
    mention_for,
    notify,
    send_message,
)
from simulation_runner.results import LapRecord, RunMetrics, ScenarioResult
from simulation_runner.scenarios import Scenario


def _result(name, passed, required=True, cones=0, finish_reason="completed", failures=None):
    scenario = Scenario(
        name=name, track="FSG25", mission=name, min_laps=1, stop_speed=0.5,
        wait_for_stop=True, preloaded_map=False, timeout=300.0, required=required, criteria={},
    )
    laps = [LapRecord(1, 35.5, cones, 0.1, 1.0, 2.5)]
    return ScenarioResult(
        scenario=scenario, finish_reason=finish_reason, duration=40.0, laps=laps,
        metrics=RunMetrics.from_laps(laps), checks=[],
        failures=list(failures or ([] if passed else ["something went wrong"])),
    )


def test_an_unmapped_author_is_not_tagged():
    assert mention_for("octocat", {}) == "octocat"
    assert mention_for("", {}) == "Someone"


def test_a_mapped_author_is_tagged_case_insensitively():
    user_map = {"octocat": "123"}

    assert mention_for("OctoCat", user_map) == "<@123>"


def test_user_map_reads_logins_and_emails(tmp_path):
    path = tmp_path / "discord_users.yaml"
    path.write_text('users:\n  OctoCat: "1"\n  Octo@Example.com: "2"\n')

    user_map = load_user_map(path)

    assert user_map == {"octocat": "1", "octo@example.com": "2"}
    assert load_user_map(tmp_path / "absent.yaml") == {}


def test_message_reports_every_scenario_and_the_overall_verdict():
    results = [_result("acceleration", True), _result("skidpad", False, required=False, cones=1)]

    context = RunContext(author="octocat", pull_request="#42")
    message = build_message(results, context, {"octocat": "9"})

    assert message.startswith("<@9> Pull request #42 has been tested: **PASSED**")
    assert "acceleration" in message and "skidpad" in message
    assert "1 cone" in message and "[optional]" in message


def test_a_failing_required_scenario_makes_the_run_fail_and_lists_why():
    results = [_result("trackdrive", False, failures=["max_cones_hit: 4.000 exceeds 3"])]

    message = build_message(results, RunContext(author="octocat"), {})

    assert "**FAILED**" in message
    assert "max_cones_hit: 4.000 exceeds 3" in message


def test_long_messages_are_truncated_to_the_discord_limit():
    results = [_result(f"scenario{i}", False, failures=["x" * 200]) for i in range(40)]

    message = build_message(results, RunContext(), {})

    assert len(message) <= MESSAGE_LIMIT


def test_without_a_webhook_nothing_is_sent(tmp_path):
    results = [_result("acceleration", True)]

    assert notify(results, RunContext(), tmp_path / "absent.yaml", webhook_url="") is None


class _Recorder(BaseHTTPRequestHandler):
    received = []
    user_agents = []
    status = 204

    def do_POST(self):  # noqa: N802
        length = int(self.headers["Content-Length"])
        _Recorder.received.append(json.loads(self.rfile.read(length)))
        _Recorder.user_agents.append(self.headers.get("User-Agent", ""))
        self.send_response(_Recorder.status)
        self.end_headers()
        if _Recorder.status >= 400:
            self.wfile.write(b'{"message": "no"}')

    def log_message(self, *args):
        pass


@pytest.fixture
def webhook_server():
    _Recorder.received = []
    _Recorder.user_agents = []
    _Recorder.status = 204
    server = HTTPServer(("127.0.0.1", 0), _Recorder)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    yield f"http://127.0.0.1:{server.server_port}/webhook", _Recorder
    server.shutdown()
    server.server_close()


def test_the_message_is_posted_as_discord_expects(webhook_server):
    url, recorder = webhook_server

    send_message(url, "hello")

    assert recorder.received == [{"content": "hello"}]


def test_an_unreachable_webhook_is_reported(tmp_path):
    with pytest.raises(NotificationError):
        send_message("http://127.0.0.1:1/webhook", "hello", timeout=1.0)


def test_a_descriptive_user_agent_is_sent(webhook_server):
    """Discord's edge rejects the default urllib user agent with 403."""
    url, recorder = webhook_server

    send_message(url, "hello")

    assert recorder.user_agents and "urllib" not in recorder.user_agents[0].lower()


def test_a_rejected_message_reports_what_discord_said(webhook_server):
    url, recorder = webhook_server
    recorder.status = 403

    with pytest.raises(NotificationError, match="403"):
        send_message(url, "hello")


def test_a_username_instead_of_an_id_is_shown_as_text():
    """<@name> renders as a broken mention in Discord; only numeric ids ping."""
    assert mention_for("octocat", {"octocat": "someusername"}) == "@someusername"


def test_the_webhook_falls_back_from_argument_to_environment_to_file(tmp_path, monkeypatch):
    """An exported variable dies with the shell, so a local file has to work too."""
    from simulation_runner import notifications

    webhook_file = tmp_path / "discord_webhook"
    monkeypatch.setattr(notifications, "WEBHOOK_FILE", webhook_file)
    monkeypatch.delenv("DISCORD_WEBHOOK_URL", raising=False)

    assert notifications.resolve_webhook("https://explicit") == "https://explicit"
    assert notifications.resolve_webhook() == ""

    webhook_file.write_text("https://from-file\n")
    assert notifications.resolve_webhook() == "https://from-file"

    monkeypatch.setenv("DISCORD_WEBHOOK_URL", "https://from-env")
    assert notifications.resolve_webhook() == "https://from-env"

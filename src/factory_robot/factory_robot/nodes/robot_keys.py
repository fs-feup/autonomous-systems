#!/usr/bin/env python3
from __future__ import annotations
import os
import sys
import tty
import termios
import fcntl
import select
from contextlib import contextmanager

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String


@contextmanager
def raw_nonblocking_stdin():
    """
    Put the current TTY (stdin) in raw + non-blocking mode and restore on exit.
    We read with os.read() guarded by select() to avoid TextIOWrapper quirks.
    """
    fd = sys.stdin.fileno()
    old_term = termios.tcgetattr(fd)
    old_fl = fcntl.fcntl(fd, fcntl.F_GETFL)
    try:
        tty.setraw(fd)
        fcntl.fcntl(fd, fcntl.F_SETFL, old_fl | os.O_NONBLOCK)
        yield fd
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_term)
        fcntl.fcntl(fd, fcntl.F_SETFL, old_fl)


class RobotKeys(Node):
    """
    Publishes std_msgs/String to /control/cmd.
    Arrows/WASD = UP/DOWN/LEFT/RIGHT (viewer referential), P = PICK, D or X = DROP, Q or Ctrl-C = quit.
    """
    def __init__(self):
        super().__init__('robot_keys')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pub = self.create_publisher(String, 'control/cmd', qos)

        # enter raw, non-blocking mode and remember fd
        self.stdin_ctx = raw_nonblocking_stdin()
        self.fd = self.stdin_ctx.__enter__()

        # poll at ~30 Hz
        self.timer = self.create_timer(1.0 / 30.0, self._poll_keys)

        # buffer for escape sequences (arrow keys)
        self._esc_buf = ""

    def destroy_node(self):
        try:
            self.stdin_ctx.__exit__(None, None, None)
        except Exception:
            pass
        return super().destroy_node()

    # ---------- key polling ----------
    def _poll_keys(self):
        # Use select to avoid read() when no data is available.
        rlist, _, _ = select.select([self.fd], [], [], 0.0)
        if not rlist:
            return

        try:
            # Read a chunk of bytes; decode safely.
            data_bytes = os.read(self.fd, 1024)
        except BlockingIOError:
            return
        except OSError:
            return

        if not data_bytes:
            return

        try:
            data = data_bytes.decode('utf-8', errors='ignore')
        except Exception:
            # Fallback: ISO-8859-1 (very permissive)
            data = data_bytes.decode('latin1', errors='ignore')

        for ch in data:
            # Ctrl-C in raw mode appears as '\x03'
            if ch == '\x03':
                self.get_logger().info("Ctrl-C pressed; quitting robot_keys")
                rclpy.shutdown()
                return
            cmd = self._to_cmd(ch)
            if cmd:
                self.pub.publish(String(data=cmd))

    def _to_cmd(self, ch: str) -> str | None:
        """Translate raw chars (incl. ESC sequences) into commands."""
        # Arrow keys: ESC [ A/B/C/D
        if ch == '\x1b':            # ESC
            self._esc_buf = '\x1b'
            return None
        if self._esc_buf == '\x1b':
            if ch == '[':
                self._esc_buf = '\x1b['
                return None
            self._esc_buf = ""
            return None
        if self._esc_buf == '\x1b[':
            self._esc_buf = ""
            return {'A': 'UP', 'B': 'DOWN', 'C': 'RIGHT', 'D': 'LEFT'}.get(ch)

        # Single keys
        if ch.lower() == 'w': return 'UP'
        if ch.lower() == 's': return 'DOWN'
        if ch.lower() == 'a': return 'LEFT'
        if ch == 'd':         return 'RIGHT'   # lowercase d = move right
        if ch == 'D' or ch.lower() == 'x': return 'DROP'  # uppercase D or X = drop
        if ch.lower() == 'p': return 'PICK'
        if ch.lower() == 'q':
            self.get_logger().info("Quitting robot_keys...")
            # Graceful shutdown
            rclpy.shutdown()
            return None
        return None


def main(args=None):
    rclpy.init(args=args)
    node = RobotKeys()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

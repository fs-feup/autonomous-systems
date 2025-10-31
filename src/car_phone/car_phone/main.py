#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import print_function
import threading
import time
import math
import asyncio
import json
import socket
import traceback
from datetime import datetime
from pacsim.msg import StampedScalar


import rclpy
from rclpy.node import Node
import custom_interfaces.msg._control_command

WS_HOST = "0.0.0.0"
WS_PORT = 6969
PUBLISH_RATE_HZ = 100.0
TOPIC = "/control/command"
NODE_NAME = "car_phone"

LOG_EVERY_MESSAGE = False
PRINT_RX_RATE_EVERY_S = 2.0

try:
    import websockets
    # Try to import the new Response class (websockets >= 12/13)
    try:
        from websockets.http import Response as WSHTTPResponse
        HAVE_WS_RESPONSE = True
    except Exception:
        HAVE_WS_RESPONSE = False
except ImportError:
    raise SystemExit("Install websockets:\n  python3 -m pip install websockets")


def get_host_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "UNKNOWN"


class PublishThread(threading.Thread):
    def __init__(self, rate_hz: float):
        super().__init__(daemon=True)
        self.node = Node(NODE_NAME)
        self.control_publisher = self.node.create_publisher(
            custom_interfaces.msg._control_command.ControlCommand, TOPIC, 10
        )
        self.steering_publisher = self.node.create_publisher(
            StampedScalar, "/pacsim/steering_setpoint", 10
        )
        self.acceleration_publisher = self.node.create_publisher(
            StampedScalar, "/pacsim/throttle_setpoint", 10
        )
        self.rate = float(rate_hz)
        self._lock = threading.Lock()
        self._accel = 0.0
        self._steer = 0.0
        self._stop = threading.Event()
        self.start()

    def update(self, accel: float, steer_rad: float):
        accel = max(-1.0, min(1.0, float(accel)))
        steer_rad = float(steer_rad)
        with self._lock:
            self._accel = accel
            self._steer = steer_rad

    def run(self):
        period = 1.0 / self.rate
        try:
            while not self._stop.is_set():
                with self._lock:
                    acc = self._accel
                    st = self._steer
                msg = custom_interfaces.msg._control_command.ControlCommand()
                msg.throttle_rl = acc
                msg.throttle_rr = acc
                msg.steering = st
                self.control_publisher.publish(msg)
                
                lateral_command_msg = StampedScalar()
                longitudinal_command_msg = StampedScalar()
                lateral_command_msg.value = st
                longitudinal_command_msg.value = acc
                self.acceleration_publisher.publish(longitudinal_command_msg)
                self.steering_publisher.publish(lateral_command_msg)
                        
                time.sleep(period)
        finally:
            msg = custom_interfaces.msg._control_command.ControlCommand()
            msg.throttle_rl = 0.0
            msg.throttle_rr = 0.0
            msg.steering = 0.0
            self.control_publisher.publish(msg)
            lateral_command_msg = StampedScalar()
            longitudinal_command_msg = StampedScalar()
            lateral_command_msg.value = st
            longitudinal_command_msg.value = acc
            self.acceleration_publisher.publish(longitudinal_command_msg)
            self.steering_publisher.publish(lateral_command_msg)
                        

    def stop(self):
        self._stop.set()
        self.join()


async def websocket_main(pub_thread: PublishThread):
    clients = set()

    # Serve a tiny health page only for HTTP GETs to "/", "/healthz", "/info".
    # Return None for everything else so WS upgrades proceed normally.
    async def process_request(path, request_headers):
        if path in ("/", "/healthz", "/info"):
            body = (
                "car_phone OK\n"
                f"time: {datetime.utcnow().isoformat()}Z\n"
                f"topic: {TOPIC}\n"
                f"node: {NODE_NAME}\n"
                f"clients: {len(clients)}\n"
            ).encode("utf-8")
            headers = [("Content-Type", "text/plain; charset=utf-8"),
                       ("Content-Length", str(len(body)))]
            if HAVE_WS_RESPONSE:
                return WSHTTPResponse(200, headers, body)
            else:
                # Older websockets accepts (status, headers, body)
                return (200, headers, body)
        # Return None to continue with WS handshake
        return None

    # Handler compatible with both websockets APIs (ws) and (ws, path)
    async def handler(*args):
        if len(args) == 1:
            ws = args[0]
            path = None
        else:
            ws, path = args[0], args[1]

        peer = getattr(ws, "remote_address", None)
        clients.add(ws)
        print(f"[{NODE_NAME}] Client connected: {peer} path={path}")
        rx = 0
        t0 = time.time()
        try:
            async for message in ws:
                rx += 1
                if LOG_EVERY_MESSAGE:
                    print(f"[{NODE_NAME}] RX {peer}: {message[:200]}{'...' if len(message)>200 else ''}")
                try:
                    data = json.loads(message)
                    steering_deg = data.get("steering_deg", 0.0)
                    torque = data.get("torque", 0.0)
                    pub_thread.update(float(torque), float(steering_deg) * math.pi / 180.0)
                except Exception as e:
                    print(f"[{NODE_NAME}] Bad payload from {peer}: {e}")

                now = time.time()
                if now - t0 >= PRINT_RX_RATE_EVERY_S:
                    rps = rx / (now - t0)
                    print(f"[{NODE_NAME}] RX rate {peer}: {rps:.1f} msg/s")
                    rx, t0 = 0, now
        except websockets.ConnectionClosed as e:
            print(f"[{NODE_NAME}] Client closed {peer}: code={e.code} reason={e.reason}")
        except Exception as e:
            print(f"[{NODE_NAME}] handler exception for {peer}: {e}")
            traceback.print_exc()
        finally:
            clients.discard(ws)
            print(f"[{NODE_NAME}] Client disconnected: {peer}")

    # Start server
    async with websockets.serve(
        handler,
        WS_HOST,
        WS_PORT,
        max_size=2**20,
        process_request=process_request,
        ping_interval=20,
        ping_timeout=20,
    ):
        lan_ip = get_host_ip()
        print("\n-------------------------------")
        print(f" ✅ {NODE_NAME} WebSocket Running")
        print(f" 📡 Connect your phone to: ws://{lan_ip}:{WS_PORT}")
        print(f" 🔎 Health check:     curl http://{lan_ip}:{WS_PORT}/healthz")
        print(f" 🧪 WS test (host):  npx wscat -c ws://{lan_ip}:{WS_PORT}")
        print("-------------------------------\n")
        await asyncio.Future()


def main():
    rclpy.init()
    pub_thread = PublishThread(PUBLISH_RATE_HZ)
    try:
        asyncio.run(websocket_main(pub_thread))
    except KeyboardInterrupt:
        pass
    except Exception:
        traceback.print_exc()
    finally:
        pub_thread.stop()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

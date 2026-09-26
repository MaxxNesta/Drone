"""
ESP32 Node Simulator
====================
Simulates two ESP32 motor-controller nodes so the full pipeline works
without any physical hardware.

Each virtual node:
  - Listens for motor commands (x,y) and mode commands on its UDP port
  - Updates a virtual heading (pan) and pitch (tilt) with simple physics
  - Sends sensor packets to Unity on port 4444 every 100 ms

Node 0  ->  listens :3333  |  Node 1  ->  listens :3334
Both send  ->  Unity :4444
"""

import socket
import threading
import time
import sys
import argparse

ap = argparse.ArgumentParser(description="ESP32 Node Simulator")
ap.add_argument("--unity-ip", default="127.0.0.1",
                help="IP of the machine running Unity (default: 127.0.0.1)")
args = ap.parse_args()

UNITY_IP   = args.unity_ip
UNITY_PORT = 4444

# Physics constants
PAN_SPEED    = 40.0   # degrees/sec pan at full error (norm_x = 1.0)
TILT_SPEED   = 25.0   # degrees/sec tilt at full error
SEARCH_SPEED = 15.0   # degrees/sec slow pan when searching
SEARCH_DELAY = 0.5    # seconds at zero error before entering search mode
PITCH_MIN, PITCH_MAX = -45.0, 45.0
CMD_TIMEOUT  = 0.3    # treat stale commands as zero after this many seconds


class NodeSimulator:
    def __init__(self, node_id, listen_port, lat, lon, heading=90.0):
        self.node_id     = node_id
        self.listen_port = listen_port
        self.lat         = lat
        self.lon         = lon
        self.heading     = heading
        self.pitch       = 0.0

        self.is_active    = False
        self.is_searching = False
        self.error_x      = 0.0
        self.error_y      = 0.0
        self._last_cmd    = time.time()
        self._zero_start  = None

        self._rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._rx.bind(("0.0.0.0", listen_port))
        self._rx.settimeout(0.1)

        self._tx      = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.running  = True

    # ── Start all threads ────────────────────────────────────────────────────
    def start(self):
        threading.Thread(target=self._recv_loop,    daemon=True).start()
        threading.Thread(target=self._physics_loop, daemon=True).start()
        threading.Thread(target=self._send_loop,    daemon=True).start()
        return self

    # ── Receive motor/mode commands from Python tracker ──────────────────────
    def _recv_loop(self):
        while self.running:
            try:
                data, _ = self._rx.recvfrom(256)
                msg = data.decode().strip()

                if msg.lower() == "active":
                    self.is_active    = True
                    self.is_searching = False
                    _log(f"Node {self.node_id} → ACTIVE")

                elif msg.lower() == "disabled":
                    self.is_active    = False
                    self.is_searching = False
                    self.error_x      = 0.0
                    self.error_y      = 0.0
                    _log(f"Node {self.node_id} → DISABLED")

                else:
                    # "norm_x,norm_y"  motor command
                    parts = msg.split(",")
                    if len(parts) == 2:
                        self.error_x   = float(parts[0])
                        self.error_y   = float(parts[1])
                        self._last_cmd = time.time()

            except (socket.timeout, ValueError):
                pass
            except Exception as e:
                _log(f"Node {self.node_id} recv error: {e}")

    # ── Update heading / pitch based on error commands ────────────────────────
    def _physics_loop(self):
        prev = time.time()
        while self.running:
            now = time.time()
            dt  = now - prev
            prev = now

            # Stale-command timeout
            if now - self._last_cmd > CMD_TIMEOUT:
                self.error_x = self.error_y = 0.0

            if self.is_active:
                near_zero = abs(self.error_x) < 0.05 and abs(self.error_y) < 0.05

                if near_zero:
                    if self._zero_start is None:
                        self._zero_start = now
                    elif now - self._zero_start > SEARCH_DELAY:
                        self.is_searching = True
                else:
                    self._zero_start  = None
                    self.is_searching = False

                if self.is_searching:
                    # Slow continuous pan
                    self.heading = (self.heading + SEARCH_SPEED * dt) % 360
                else:
                    self.heading = (self.heading + self.error_x * PAN_SPEED  * dt) % 360
                    self.pitch   = max(PITCH_MIN,
                                   min(PITCH_MAX,
                                       self.pitch - self.error_y * TILT_SPEED * dt))

            time.sleep(0.02)

    # ── Send sensor packet to Unity ───────────────────────────────────────────
    def _send_loop(self):
        while self.running:
            if not self.is_active:
                mode = "D"
            elif self.is_searching:
                mode = "S"
            else:
                mode = "T"

            pkt = (f"ID:{self.node_id},M:{mode},"
                   f"P:{self.pitch:.1f},"
                   f"MAG:{self.heading:.1f},"
                   f"LAT:{self.lat:.6f},"
                   f"LON:{self.lon:.6f}")
            try:
                self._tx.sendto(pkt.encode(), (UNITY_IP, UNITY_PORT))
            except Exception:
                pass
            time.sleep(0.1)

    def stop(self):
        self.running = False
        self._rx.close()
        self._tx.close()

    @property
    def status(self):
        if not self.is_active:
            mode = "DISABLED"
        elif self.is_searching:
            mode = "SEARCH "
        else:
            mode = "TRACK  "
        return (f"Node{self.node_id} [{mode}]  "
                f"H={self.heading:6.1f}  P={self.pitch:+6.1f}  "
                f"({self.lat:.5f}, {self.lon:.5f})")


_log_lock = threading.Lock()

def _log(msg):
    with _log_lock:
        print(f"\n  >> {msg}")


def main():
    print("=" * 58)
    print("  ESP32 Node Simulator")
    print("=" * 58)
    print(f"  Node 0  listen :3333   send → {UNITY_IP}:{UNITY_PORT}")
    print(f"  Node 1  listen :3334   send → {UNITY_IP}:{UNITY_PORT}")
    print("  Ctrl+C to stop")
    print("=" * 58)
    print()

    nodes = [
        NodeSimulator(0, 3333, lat=37.774929, lon=-122.419418, heading=45.0).start(),
        NodeSimulator(1, 3334, lat=37.774960, lon=-122.419390, heading=225.0).start(),
    ]

    try:
        while True:
            line = "  |  ".join(n.status for n in nodes)
            sys.stdout.write(f"\r{line}   ")
            sys.stdout.flush()
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\n\nShutting down...")
        for n in nodes:
            n.stop()


if __name__ == "__main__":
    main()

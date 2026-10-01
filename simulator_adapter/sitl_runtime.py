"""Optional receive-only collector. Requires Gazebo Python bindings + pymavlink.

There is intentionally no publish/request/send/mavlink_connection API here.
"""
import argparse
import base64
import fcntl
import json
import os
from pathlib import Path
import socket
import tempfile
import time
from .sitl import RealSitlAdapter, SitlConfig


def atomic_json(path, record):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix='.sitl-', dir=path.parent)
    try:
        with os.fdopen(fd, 'w') as stream:
            json.dump(record, stream, allow_nan=False)
            stream.write('\n')
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


class PassiveMavlink:
    """Decoder only; no writable MAVLink file or sender is supplied."""
    def __init__(self):
        from pymavlink.dialects.v20 import common
        self.common = common

    def decode(self, data):
        if len(data) > 65507:
            raise ValueError('Oversized UDP datagram')
        # UDP datagrams must contain complete packets. Do not carry fragments
        # or bad sender data into subsequent datagrams.
        parser = self.common.MAVLink(None)
        return parser.parse_buffer(data) or []


class Collector:
    def __init__(self, adapter, node, clock_type, pose_type, decoder, capture=None):
        self.adapter, self.node, self.decoder = adapter, node, decoder
        self.capture = capture
        self.peer = None
        self.topics = []
        world = adapter.config.world
        callbacks = [(clock_type, '/world/'+world+'/clock', self.clock),
                     (pose_type, '/world/'+world+'/dynamic_pose/info', self.pose)]
        try:
            for message_type, topic, callback in callbacks:
                if not node.subscribe(message_type, topic, callback):
                    raise RuntimeError('Unable to subscribe to '+topic)
                self.topics.append(topic)
        except Exception:
            self.close()
            raise

    def close(self):
        for topic in self.topics:
            self.node.unsubscribe(topic)
        self.topics.clear()

    def _record(self, kind, data, received):
        if self.capture:
            self.capture(kind, data, received)

    def clock(self, message):
        now = time.monotonic()
        try:
            with self.adapter.lock:
                if not message.HasField('sim'):
                    raise ValueError('Missing simulation clock')
                self._record('gazebo_clock_protobuf', message.SerializeToString(), now)
                self.adapter.clock({'sim': {'sec': message.sim.sec, 'nsec': message.sim.nsec}}, now)
        except (ValueError, KeyError, TypeError, AttributeError):
            with self.adapter.lock:
                self.adapter.reject('invalid_gazebo_clock')

    def pose(self, message):
        now = time.monotonic()
        try:
            with self.adapter.lock:
                if not message.HasField('header') or not message.header.HasField('stamp'):
                    raise ValueError('Missing pose timestamp')
                if len(message.pose) > 4096:
                    raise ValueError('Too many poses')
                self._record('gazebo_pose_v_protobuf', message.SerializeToString(), now)
                packet = {'stamp': {'sec': message.header.stamp.sec, 'nsec': message.header.stamp.nsec},
                          'poses': [{'name': p.name, 'id': p.id, 'position': (p.position.x, p.position.y, p.position.z),
                                     'orientation_wxyz': (p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z)}
                                    for p in message.pose]}
                self.adapter.pose(packet, now)
        except (ValueError, KeyError, TypeError, AttributeError):
            with self.adapter.lock:
                self.adapter.reject('invalid_gazebo_pose')

    def datagram(self, data, peer, now):
        with self.adapter.lock:
            if peer[0] != '127.0.0.1' or self.peer is not None and peer != self.peer:
                return self.adapter.reject('udp_peer')
            try:
                messages = self.decoder.decode(data)
            except Exception:
                return self.adapter.reject('invalid_mavlink_datagram')
            for message in messages:
                try:
                    accepted = self.adapter.mavlink(message.get_type(), message.to_dict(),
                                                    message.get_srcSystem(), message.get_srcComponent(), now)
                    if accepted:
                        self.peer = peer
                except (ValueError, KeyError, TypeError, AttributeError):
                    self.adapter.reject('invalid_mavlink_message')
            self._record('mavlink_udp', data, now)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--capture', type=Path)
    parser.add_argument('--duration', type=float, default=60)
    parser.add_argument('--confirmed-isolated-sitl', action='store_true')
    args = parser.parse_args()
    if not args.confirmed_isolated_sitl:
        parser.error('Requires an isolated single unarmed SITL environment; never attach to hardware')
    if not 1 <= args.duration <= 3600:
        parser.error('Duration must be in [1,3600] seconds')
    config = SitlConfig(**json.loads(args.config.read_text()))
    # Must match the separately launched Gazebo process. No external discovery.
    os.environ['GZ_IP'] = '127.0.0.1'
    os.environ['GZ_PARTITION'] = 'skyview-sitl'
    from gz.transport13 import Node
    from gz.msgs10.clock_pb2 import Clock
    from gz.msgs10.pose_v_pb2 import Pose_V
    decoder = PassiveMavlink()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.with_suffix('.lock').open('a') as owner:
        fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
        adapter = RealSitlAdapter(config)
        capture = args.capture.open('x') if args.capture else None
        captured = 0
        def record(kind, data, received):
            nonlocal captured
            if capture is None:
                return
            row = json.dumps({'kind': kind, 'received_monotonic_s': received,
                              'encoding': 'base64', 'data': base64.b64encode(data).decode('ascii')})+'\n'
            if captured+len(row) <= 10*1024*1024:
                capture.write(row)
                captured += len(row)
        collector = None
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                sock.bind(('127.0.0.1', config.udp_port))
                sock.settimeout(.1)
                collector = Collector(adapter, Node(), Clock, Pose_V, decoder, record)
                deadline = time.monotonic()+args.duration
                while time.monotonic() < deadline:
                    try:
                        data, peer = sock.recvfrom(65535)
                        collector.datagram(data, peer, time.monotonic())
                    except socket.timeout:
                        pass
                    atomic_json(args.output, adapter.telemetry(time.monotonic()))
        except KeyboardInterrupt:
            pass
        finally:
            if collector:
                collector.close()
            with adapter.lock:
                adapter.blocked = 'collector_stopped'
                atomic_json(args.output, adapter.telemetry(time.monotonic()))
                if capture:
                    capture.close()


if __name__ == '__main__':
    main()

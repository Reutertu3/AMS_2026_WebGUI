#!/usr/bin/env python3
"""Web control for the EduRob: manual driving, live odometry trail, lidar view, gyro calibration,
running movement scripts from scripts/, switching the ROS domain.

usage: ./start.sh && ./ros.sh python3 webui.py             -> open http://localhost:8080
       ./ros.sh python3 webui.py --host 0.0.0.0            (reachable from phone/other PCs: anyone on the WLAN can drive!)
       ./ros.sh --turtle python3 webui.py                  (turtlesim instead of the robot)
"""
import argparse
import base64
import collections
import gzip
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from turtlesim_msgs.msg import Pose as TurtlePose

try:  # built from src/ with colcon, see README.md
    from edurob_interfaces.srv import CalibrateGyro
except ImportError:
    CalibrateGyro = None
try:  # only in the edurob-slam image (start.sh uses it for the web server once it's built)
    from slam_toolbox.srv import Reset as SlamReset
except ImportError:
    SlamReset = None

CMD_INTERVAL = 0.5  # s, the robot only handles ~2 commands/s, faster ones queue up (README.md)
DEADMAN = 0.8       # s without a command from the browser -> slow down to 0 (tab closed, WiFi gone)
TICK = 0.05         # s, ramp update rate
# Real robot: if a wheel has to go faster than it can, the robot stalls for seconds (see WHEEL_MAX in
# edurobot.py, same values). Commands are scaled so the fastest mecanum wheel stays within WHEEL_MAX:
# |x| + |y| + WHEEL_LEVER * |turn| <= WHEEL_MAX. Not for the turtle.
WHEEL_MAX = 0.16    # m/s
WHEEL_LEVER = 0.25  # m
HERE = os.path.dirname(os.path.abspath(__file__))
HTML = os.path.join(HERE, 'webui.html')
SCRIPTS = os.path.join(HERE, 'scripts')
MAPS = os.path.join(HERE, 'maps')
# ROS args given to webui.py (ros.sh --turtle adds the turtlesim remaps), passed on to scripts
ROS_ARGS = sys.argv[sys.argv.index('--ros-args'):] if '--ros-args' in sys.argv else []


def step_towards(cur, target, max_step):
    return cur + max(-max_step, min(max_step, target - cur))


class ScriptRunner:
    """Runs one script from scripts/ as its own process (so Stop always works), keeps its output."""

    def __init__(self, on_exit):
        self.proc = None
        self.name = None
        self.code = None
        self.stopped = False
        self.log = collections.deque(maxlen=2000)
        self.log_start = 0  # number of lines dropped from the front of `log`
        self.run_id = 0     # counts the runs, so the page notices a new one
        self.lock = threading.Lock()
        self.on_exit = on_exit

    @staticmethod
    def list():
        libraries = ('motion.py', 'edurobot.py')  # importable helpers, not programs to run
        return sorted(f for f in os.listdir(SCRIPTS)
                      if f.endswith('.py') and not f.startswith('_') and f not in libraries)

    def running(self):
        return self.proc is not None and self.proc.poll() is None

    def start(self, name):
        if name not in self.list():  # only files from scripts/, no paths
            raise ValueError(f'unknown script {name!r}')
        if self.running():
            raise ValueError(f'{self.name} is still running')
        with self.lock:
            self.log.clear()
            self.log_start = 0
        self.name, self.code, self.stopped = name, None, False
        self.run_id += 1
        self._add(f'$ python3 scripts/{name}')
        env = dict(os.environ, PYTHONPATH=os.pathsep.join(filter(None, [HERE, os.environ.get('PYTHONPATH')])),
                   PYTHONUNBUFFERED='1')
        self.proc = subprocess.Popen([sys.executable, os.path.join(SCRIPTS, name)] + ROS_ARGS, cwd=HERE, env=env,
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        threading.Thread(target=self._read, args=(self.proc,), daemon=True).start()

    def stop(self):
        """Like Ctrl+C: the script's edurobot stops the robot on the way out."""
        p = self.proc
        if p is not None and p.poll() is None:
            self.stopped = True
            p.send_signal(signal.SIGINT)
            try:
                p.wait(3)
            except subprocess.TimeoutExpired:
                p.kill()

    def _add(self, line):
        with self.lock:
            if len(self.log) == self.log.maxlen:
                self.log_start += 1
            self.log.append(line)

    def _read(self, proc):
        for line in proc.stdout:
            self._add(line.rstrip('\n'))
        self.code = proc.wait()
        self._add('[stopped]' if self.stopped else f'[finished, exit code {self.code}]')
        self.on_exit()

    def state(self, since, run_id):
        if run_id != self.run_id:
            since = 0  # the page still shows an older run: send everything
        with self.lock:
            end = self.log_start + len(self.log)
            lines = list(self.log)[max(0, since - self.log_start):]
        return {'name': self.name, 'run': self.run_id, 'running': self.running(), 'code': self.code,
                'stopped': self.stopped,
                'log': lines, 'log_from': end - len(lines), 'log_end': end}


class Bridge(Node):
    def __init__(self):
        super().__init__('webui')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.create_subscription(TurtlePose, 'turtle_pose', self.on_turtle, 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        # slam_toolbox (start.sh --slam) publishes /map "latched": also get the last one when joining later
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, 'map', self.on_map, map_qos)
        self.map = None
        self.map_seq = 0
        self.map_time = 0.0
        self.reset_client = self.create_client(SlamReset, 'slam_toolbox/reset') if SlamReset else None
        self.lock = threading.Lock()
        self.target = [0.0, 0.0, 0.0]  # what the browser wants: x m/s, y m/s, turn rad/s
        self.cur = [0.0, 0.0, 0.0]     # ramped value that is actually sent
        self.accel = [0.0, 0.0]        # m/s², rad/s², 0 = no ramp
        self.last_request = 0.0
        self.last_pub = 0.0
        self.sent = (0.0, 0.0, 0.0)
        self.turtle = None
        self.scan = None
        self.scan_seq = 0
        self.scan_time = 0.0
        self.scan_times = []
        self.calib = {'state': 'idle', 'msg': ''}  # idle / running / done / error
        self.calib_client = self.create_client(CalibrateGyro, 'calibrategyro') if CalibrateGyro else None
        self.script = ScriptRunner(on_exit=lambda: self.publish((0.0, 0.0, 0.0)))
        self.create_timer(TICK, self.tick)

    # ---- inputs ----
    def on_turtle(self, m):
        self.turtle = (m.x, m.y, m.theta)

    def on_scan(self, m):
        now = time.monotonic()
        self.scan_times = [t for t in self.scan_times if now - t < 2] + [now]
        self.scan_time = now
        r = [round(v, 3) if math.isfinite(v) and m.range_min <= v <= m.range_max else 0 for v in m.ranges]
        self.scan_seq += 1
        self.scan = {'seq': self.scan_seq, 'amin': m.angle_min, 'ainc': m.angle_increment, 'r': r}

    def on_map(self, m):
        self.map_seq += 1
        self.map_time = time.monotonic()
        self.map = m

    def map_json(self):
        m = self.map
        if m is None:
            return None
        info = m.info
        data = np.asarray(m.data, dtype=np.int8).tobytes()  # 0..100 occupancy, -1 unknown -> byte 255
        return {'seq': self.map_seq, 'w': info.width, 'h': info.height, 'res': info.resolution,
                'ox': info.origin.position.x, 'oy': info.origin.position.y, 'data': base64.b64encode(data).decode()}

    def reset_map(self):
        """Throw the current SLAM map away and start a new one where the robot is now."""
        if not self.reset_client:
            raise ValueError('not available: web server runs without slam_toolbox (run ./start.sh --slam once, then restart it)')
        if not self.reset_client.service_is_ready():
            raise ValueError('SLAM is not running (./start.sh --slam)')
        self.reset_client.call_async(SlamReset.Request())
        self.map = None       # forget the old map at once, the page clears it
        self.map_seq += 1
        print('map reset', flush=True)

    def save_map(self, name):
        """Write maps/<name>.pgm + .yaml in the map_server format (usable by nav2 / slam_toolbox)."""
        if not name or not all(c.isalnum() or c in '-_' for c in name):
            raise ValueError('name: only letters, digits, - and _')
        m = self.map
        if m is None:
            raise ValueError('no map yet (started with ./start.sh --slam?)')
        w, h = m.info.width, m.info.height
        rows = []
        for y in range(h - 1, -1, -1):  # image top = largest y
            row = m.data[y * w:(y + 1) * w]
            rows.append(bytes(205 if v < 0 else 0 if v >= 65 else 254 if v <= 25 else 205 for v in row))
        os.makedirs(MAPS, exist_ok=True)
        with open(os.path.join(MAPS, name + '.pgm'), 'wb') as f:
            f.write(f'P5\n{w} {h}\n255\n'.encode() + b''.join(rows))
        o = m.info.origin.position
        with open(os.path.join(MAPS, name + '.yaml'), 'w') as f:
            f.write(f'image: {name}.pgm\nmode: trinary\nresolution: {m.info.resolution}\n'
                    f'origin: [{o.x}, {o.y}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
        return f'maps/{name}.pgm ({w}x{h} cells, {w * m.info.resolution:.1f} x {h * m.info.resolution:.1f} m)'

    def busy(self):
        # a calibration whose answer never came (robot rebooted, WiFi lost) must not block forever
        if self.calib['state'] == 'running' and time.monotonic() > self.calib.get('deadline', math.inf):
            self.calib = {'state': 'error', 'msg': 'no answer from the robot (rebooted? WiFi?), try again'}
            print('gyro calibration: no answer, given up', flush=True)
        if self.script.running():
            return f'script {self.script.name} is running'
        if self.calib['state'] == 'running':
            return 'gyro calibration is running'
        return None

    def set_target(self, x, y, turn_deg, lin_acc, ang_acc_deg, client=''):
        if self.busy():
            return  # don't fight a script / move during calibration
        with self.lock:
            if (x, y, turn_deg) == (0, 0, 0) and self.target != [0.0, 0.0, 0.0]:
                print(f'released in the browser ({client})', flush=True)
            elif (x, y, turn_deg) != (0, 0, 0) and self.target == [0.0, 0.0, 0.0]:
                print(f'drive x={x:.2f} y={y:.2f} turn={turn_deg:.0f} ({client})', flush=True)
            self.target = [x, y, math.radians(turn_deg)]
            self.accel = [max(0.0, lin_acc), math.radians(max(0.0, ang_acc_deg))]
            self.last_request = time.monotonic()

    def stop_now(self):
        """STOP button: manual driving and a running script."""
        with self.lock:
            self.target = [0.0, 0.0, 0.0]
            self.cur = [0.0, 0.0, 0.0]
        self.publish((0.0, 0.0, 0.0))
        self.script.stop()

    def run_script(self, name):
        if self.busy():  # never replace a running script by another one: press STOP first
            raise ValueError(self.busy())
        self.stop_now()
        self.script.start(name)

    def calibrate(self, seconds):
        if not self.calib_client:
            raise ValueError('edurob_interfaces not built (see README.md)')
        if self.busy():
            raise ValueError(self.busy())
        if not self.calib_client.service_is_ready():
            raise ValueError('no /calibrategyro service, is the robot connected?')
        self.stop_now()  # the robot has to stand still
        req = CalibrateGyro.Request()
        req.time = max(1, min(255, int(seconds)))
        self.calib = {'state': 'running', 'msg': f'calibrating for {req.time} s, keep the robot still',
                      'deadline': time.monotonic() + req.time + 10}
        self.calib_client.call_async(req).add_done_callback(self.on_calibrated)

    def on_calibrated(self, future):
        if self.calib['state'] != 'running':
            return  # answer came after we gave up
        try:
            o = future.result().offsets
            self.calib = {'state': 'done', 'msg': f'offsets x={o.x:.5f} y={o.y:.5f} z={o.z:.5f}'}
        except Exception as e:
            self.calib = {'state': 'error', 'msg': str(e)}

    # ---- output ----
    def tick(self):
        now = time.monotonic()
        with self.lock:
            if now - self.last_request > DEADMAN and self.target != [0.0, 0.0, 0.0]:
                self.target = [0.0, 0.0, 0.0]
                print(f'dead-man stop: no command from the browser for {now - self.last_request:.1f} s', flush=True)
            for i in range(3):
                a = self.accel[0 if i < 2 else 1]
                self.cur[i] = step_towards(self.cur[i], self.target[i], a * TICK if a > 0 else math.inf)
            cmd = tuple(self.cur)
        if cmd == (0.0, 0.0, 0.0):
            if self.sent != cmd:
                self.publish(cmd)  # stop always goes out at once
        elif now - self.last_pub >= CMD_INTERVAL:
            self.publish(cmd)      # repeated, the robot stops by itself after ~2 s without commands

    def publish(self, cmd):
        if self.turtle is None:  # real robot
            demand = (abs(cmd[0]) + abs(cmd[1]) + WHEEL_LEVER * abs(cmd[2])) / WHEEL_MAX
            if demand > 1:
                cmd = tuple(v / demand for v in cmd)
        m = Twist()
        m.linear.x, m.linear.y, m.angular.z = (float(v) for v in cmd)
        self.pub.publish(m)
        self.sent = cmd
        self.last_pub = time.monotonic()

    # ---- state for the browser ----
    def state(self, log_since, run_id):
        pose, source, frame = None, None, None
        # With SLAM: position on the map (map -> base_fix), otherwise raw odometry
        for parent, child in (('map', 'base_fix'), ('odom', 'base_link')):
            try:
                t = self.tf.lookup_transform(parent, child, rclpy.time.Time()).transform
            except Exception:
                continue
            q = t.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            pose, source, frame = [t.translation.x, t.translation.y, yaw], 'robot', parent
            break
        if pose is None and self.turtle:
            pose, source, frame = list(self.turtle), 'turtle', 'turtle'
        with self.lock:
            cmd = list(self.cur)
        now = time.monotonic()
        scan_age = now - self.scan_time if self.scan else None
        scan_hz = sum(1 for t in self.scan_times if now - t < 2) / 2
        busy = self.busy()  # also ends a calibration that timed out
        calib = {k: v for k, v in self.calib.items() if k != 'deadline'}
        return {'pose': pose, 'source': source, 'frame': frame, 'map_seq': self.map_seq,
                'map_age': now - self.map_time if self.map else None, 'cmd': cmd, 'scan': self.scan, 'calib': calib,
                'scan_hz': scan_hz, 'scan_age': scan_age, 'busy': busy,
                'domain': int(os.environ.get('ROS_DOMAIN_ID', 0)), 'script': self.script.state(log_since, run_id)}


def make_handler(bridge, restart):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def reply(self, code, body, ctype='application/json'):
            data = body if isinstance(body, bytes) else json.dumps(body).encode()
            self.send_response(code)
            if len(data) > 100_000 and 'gzip' in self.headers.get('Accept-Encoding', ''):
                data = gzip.compress(data, 3)  # maps (MBs at 1 cm cells) compress ~10x, the browser unpacks it
                self.send_header('Content-Encoding', 'gzip')
            self.send_header('Content-Type', ctype)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            url = urlparse(self.path)
            if url.path in ('/', '/index.html'):
                with open(HTML, 'rb') as f:
                    self.reply(200, f.read(), 'text/html; charset=utf-8')
            elif url.path == '/state':
                q = parse_qs(url.query)
                self.reply(200, bridge.state(int(q.get('log', ['0'])[0] or 0), int(q.get('run', ['0'])[0] or 0)))
            elif url.path == '/map':
                self.reply(200, {'map': bridge.map_json()})
            elif url.path == '/scripts':
                self.reply(200, {'scripts': ScriptRunner.list()})
            else:
                self.reply(404, {'error': 'not found'})

        def do_POST(self):
            try:
                body = json.loads(self.rfile.read(int(self.headers.get('Content-Length', 0))) or b'{}')
                if self.path == '/cmd':
                    vals = [float(body.get(k, 0)) for k in ('x', 'y', 'turn', 'lin_acc', 'ang_acc')]
                    if not all(math.isfinite(v) for v in vals):
                        raise ValueError('not a number')
                    # hard limits, whatever the page sends
                    vals[0], vals[1] = (max(-1.0, min(1.0, v)) for v in vals[:2])
                    vals[2] = max(-180.0, min(180.0, vals[2]))
                    bridge.set_target(*vals, client=self.client_address[0])
                elif self.path == '/stop':
                    print(f'STOP pressed ({self.client_address[0]})', flush=True)
                    bridge.stop_now()
                elif self.path == '/calibrate':
                    bridge.calibrate(float(body.get('time', 5)))
                elif self.path == '/savemap':
                    return self.reply(200, {'ok': True, 'msg': bridge.save_map(str(body.get('name', '')).strip())})
                elif self.path == '/resetmap':
                    bridge.reset_map()
                elif self.path == '/run':
                    bridge.run_script(str(body.get('name', '')))
                elif self.path == '/domain':
                    domain = int(body.get('domain', -1))
                    if not 0 <= domain <= 232:
                        raise ValueError('domain must be 0..232')
                    bridge.stop_now()
                    self.reply(200, {'ok': True})
                    restart(domain)
                    return
                else:
                    return self.reply(404, {'error': 'not found'})
                self.reply(200, {'ok': True})
            except (ValueError, TypeError, json.JSONDecodeError) as e:
                self.reply(400, {'error': str(e)})

    return Handler


def main():
    # Own Ctrl+C / docker stop handling, so the final stop is still sent before ROS shuts down
    rclpy.init(args=sys.argv, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    p = argparse.ArgumentParser()
    p.add_argument('--port', type=int, default=8080)
    p.add_argument('--host', default='127.0.0.1')
    args = p.parse_args(remove_ros_args(sys.argv)[1:])

    bridge = Bridge()
    new_domain = []

    def restart(domain):
        # A ROS process can't change its domain: restart this program with the new one (after the reply is sent)
        new_domain.append(domain)
        threading.Thread(target=lambda: (time.sleep(0.3), os.kill(os.getpid(), signal.SIGINT)), daemon=True).start()

    ThreadingHTTPServer.allow_reuse_address = True
    try:
        server = ThreadingHTTPServer((args.host, args.port), make_handler(bridge, restart))
    except OSError as e:
        print(f'Port {args.port} not available ({e.strerror}). webui.py already running? '
              f'Stop it: docker ps, docker stop <name>. Or use another port: --port 8081', file=sys.stderr)
        rclpy.try_shutdown()
        sys.exit(1)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    host = 'localhost' if args.host == '127.0.0.1' else args.host
    print(f'EduRob web control: http://{host}:{args.port}  (ROS domain {os.environ.get("ROS_DOMAIN_ID", 0)}, '
          f'Ctrl+C to quit)', flush=True)
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.script.stop()
        bridge.publish((0.0, 0.0, 0.0))
        time.sleep(0.2)
        server.shutdown()
        server.server_close()
        bridge.destroy_node()
        rclpy.try_shutdown()
    if new_domain:
        print(f'Switching to ROS domain {new_domain[0]}...', flush=True)
        os.environ['ROS_DOMAIN_ID'] = str(new_domain[0])
        os.execv(sys.executable, [sys.executable] + sys.argv)


if __name__ == '__main__':
    main()

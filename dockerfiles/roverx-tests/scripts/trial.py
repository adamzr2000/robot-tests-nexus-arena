#!/usr/bin/env python3
"""Run one RoverX floor-test trial (T1/T2/T3) and write its trial.json.

  python3 trial.py --test T1 --set speed=0.5 [--set distance=3] --out DIR [--id ID]

Sequence: prepare the robot (lidar on, wheels powered), stay still settle_s
(gyro bias, gravity, robot-laptop clock offset), drive the profile at rate_hz,
then hold v=0 until the wheels report rest, cut wheel power and write results.
Prints a live line every 0.2 s. Ctrl+C / SIGTERM stops the robot the same way
(status ABORTED). Normally started by api.py, which also records the bag.
"""
import argparse
import csv
import json
import math
import os
import signal
import statistics
import sys
import time
import traceback

import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool

import profiles

HERE = os.path.dirname(os.path.abspath(__file__))
# Space limit for this location (start_roverx_tests.sh --max-travel): runs whose planned
# distance exceeds it are refused, and odometry beyond it + TRAVEL_MARGIN_M aborts.
MAX_TRAVEL_M = float(os.environ["MAX_TRAVEL_M"]) if os.environ.get("MAX_TRAVEL_M") else None
TRAVEL_MARGIN_M = 1.0
SENSOR_QOS = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
PRINT_PERIOD_S = 0.2
WAIT_DATA_S = 10.0
SERVICE_WAIT_S = 15.0
SERVICE_TIMEOUT_S = 10.0  # turning the lidar on takes ~4.5 s


class Abort(Exception):
    pass


def load_protocol():
    return yaml.safe_load(open(os.path.join(HERE, "protocol.yaml")))


def load_params(test, overrides):
    cfg = load_protocol()
    if test not in cfg["tests"]:
        raise ValueError(f"unknown test {test}; available: {sorted(cfg['tests'])}")
    params = {k: v for k, v in cfg["tests"][test].items() if k not in ("name", "values", "repetitions")}
    cond = params["condition"]
    params[cond] = None
    for key, value in overrides.items():
        if key not in params or key == "condition":
            raise ValueError(f"{test} has no setting '{key}'; settings: {sorted(params)}")
        params[key] = value if isinstance(params[key], str) else float(value)
    if params[cond] is None:
        raise ValueError(f"{test} needs its condition: {cond}=<value>")
    for key, value in params.items():
        if isinstance(value, (int, float)) and not (math.isfinite(value) and value > 0):
            raise ValueError(f"{key} must be a positive number, got {value}")
    if test == "T3" and params["stop_mode"] not in ("active", "coast"):
        raise ValueError("stop_mode must be 'active' or 'coast'")
    if MAX_TRAVEL_M is not None:
        c = cfg["control"]
        planned = profiles.plan(test, params, c["crawl_speed"], c["rate_hz"])["distance_m"]
        if planned > MAX_TRAVEL_M + 0.01:  # the plan's last 50 Hz step lands a hair past the target
            hint = {"T1": "distance", "T2": "speed or hold_s", "T3": "trigger"}[test]
            raise ValueError(f"planned travel {planned:.2f} m exceeds the max travel {MAX_TRAVEL_M:g} m "
                             f"set for this location; lower {hint}")
    return cfg, params


class Rover(Node):
    """Robot interface: odometry (with extrapolated distance), IMU samples,
    relay status, /cmd_vel and /trial/events publishers, SetBool services."""

    def __init__(self, cfg):
        super().__init__("roverx_trial")
        r, c = cfg["robot"], cfg["control"]
        self.c = c
        self.z_sign = r["imu_z_sign"]
        self.cmd_pub = self.create_publisher(Twist, r["cmd_vel"], 10)
        self.event_pub = self.create_publisher(String, "/trial/events", 10)
        self.odom = None            # (x, y, v, monotonic rx time)
        self.stamp_offsets = []     # robot header stamp - laptop receive time
        self.imu = []               # (monotonic, ax, ay, az, gz, IMU stamp)
        self.odom_speeds = []       # every wheel speed received (stillness check)
        self.imu_rx = None          # monotonic time of the last IMU sample
        self.relays = None
        self.origin = None
        self.gz_bias = None         # set after settling; then heading integrates
        self.heading = 0.0          # rad since the bias was set
        self._pending = []          # fire-and-forget service calls
        self.create_subscription(Odometry, r["odom"], self._on_odom, SENSOR_QOS)
        self.create_subscription(Imu, r["imu"], self._on_imu, SENSOR_QOS)
        self.create_subscription(Bool, r["relay_status"], lambda m: setattr(self, "relays", m.data), 10)

    def _on_odom(self, m):
        p = m.pose.pose.position
        self.odom = (p.x, p.y, m.twist.twist.linear.x, time.monotonic())
        self.odom_speeds.append(m.twist.twist.linear.x)
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.stamp_offsets.append(stamp - time.time())

    def _on_imu(self, m):
        a = m.linear_acceleration
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        sample = (time.monotonic(), a.x, a.y, a.z, self.z_sign * m.angular_velocity.z, stamp)
        if self.gz_bias is not None and self.imu:
            dt = stamp - self.imu[-1][5]  # the IMU's own clock: steady 4 ms, unlike network arrival
            if 0 < dt < 0.1:
                self.heading += (sample[4] - self.gz_bias) * dt
        self.imu.append(sample)
        self.imu_rx = sample[0]

    def speed(self):
        return self.odom[2]

    def distance(self, extrapolate=True):
        """Straight-line distance from the motion start, extrapolated between
        odometry samples (~20 Hz) with the measured speed."""
        x, y, v, rx = self.odom
        d = math.hypot(x - self.origin[0], y - self.origin[1])
        if extrapolate:
            d += abs(v) * min(time.monotonic() - rx, self.c["extrapolate_max_s"])
        return d

    def odom_age(self):
        return time.monotonic() - self.odom[3]

    def imu_age(self):
        return time.monotonic() - self.imu_rx

    def send(self, v):
        msg = Twist()
        msg.linear.x = float(v)
        self.cmd_pub.publish(msg)

    def event(self, name, **data):
        data.update(event=name, wall=time.time())
        self.event_pub.publish(String(data=json.dumps(data)))
        return data

    def spin_for(self, seconds, until=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)
            if until is not None and until():
                return True
        return until is None

    def request_bool(self, name, value):
        """Call a SetBool service without waiting (the reply is ignored)."""
        client = self.create_client(SetBool, name)
        self._pending.append((client, client.call_async(SetBool.Request(data=value))))

    def set_bool(self, name, value):
        client = self.create_client(SetBool, name)
        try:
            if not client.wait_for_service(timeout_sec=SERVICE_WAIT_S):
                return False, "service not found"
            for _ in range(2):
                future = client.call_async(SetBool.Request(data=value))
                rclpy.spin_until_future_complete(self, future, timeout_sec=SERVICE_TIMEOUT_S)
                if future.done():
                    return future.result().success, future.result().message
                client.remove_pending_request(future)
            return False, "no reply"
        finally:
            self.destroy_client(client)

    def get_params(self, node, names):
        """One request per name: the driver rejects a whole request if any name is unknown."""
        client = self.create_client(GetParameters, f"{node}/get_parameters")
        out = {}
        try:
            if not client.wait_for_service(timeout_sec=5.0):
                return out
            for name in names:
                future = client.call_async(GetParameters.Request(names=[name]))
                rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
                values = future.result().values if future.done() else []
                v = values[0] if values else None
                out[name] = {2: v.integer_value, 3: v.double_value, 1: v.bool_value, 4: v.string_value}.get(v.type) if v else None
            return out
        finally:
            self.destroy_client(client)


def run(cfg, test, params, result, out_dir, verbose=True):
    """Drive one trial; fills `result` as it goes so an abort keeps what is known."""
    c, r = cfg["control"], cfg["robot"]
    node = Rover(cfg)
    log = (lambda s: print(s, flush=True)) if verbose else (lambda s: None)
    prof = profiles.make(test, params, c["crawl_speed"])
    if MAX_TRAVEL_M is not None:
        prof.max_distance = min(prof.max_distance, MAX_TRAVEL_M + TRAVEL_MARGIN_M)
    result["max_travel_m"] = MAX_TRAVEL_M
    result["imu_forward_deg"] = r["imu_forward_deg"]
    deadline = 2 * profiles.plan(test, params, c["crawl_speed"], c["rate_hz"])["motion_s"] + 5
    period = 1.0 / c["rate_hz"]
    steady = profiles.Steady(params["speed"], c["steady_tolerance"], round(c["steady_window_s"] * c["rate_hz"]))
    samples = []        # (t, cmd, v, d, phase, at target speed)
    events = result["events"]
    moving = False
    try:
        # --- prepare -------------------------------------------------------
        if not node.spin_for(WAIT_DATA_S, lambda: node.odom is not None):
            raise Abort(f"no odometry on {r['odom']} within {WAIT_DATA_S:g}s")
        for name, value in r["prepare"]:
            ok, msg = node.set_bool(name, value)
            log(f"{name} {value}: {msg}")
            if not ok:
                raise Abort(f"robot preparation failed: {name} {value}: {msg}")
        if not node.spin_for(WAIT_DATA_S, lambda: node.imu and node.relays):
            raise Abort("no IMU data" if not node.imu else "wheels not powered (/relay_status false)")
        node.imu.clear()
        node.stamp_offsets.clear()
        node.odom_speeds.clear()
        node.spin_for(c["settle_s"])
        rest = node.imu[:]
        # the gyro bias and gravity below are only valid if nothing moved the robot meanwhile
        moved = max((abs(v) for v in node.odom_speeds), default=0.0)
        gz_mean = statistics.fmean(s[4] for s in rest)
        turned = max(abs(s[4] - gz_mean) for s in rest)
        if moved > c["rest_speed"] or turned > c["still_gyro_rad_s"]:
            raise Abort(f"robot not still during the {c['settle_s']:g}s still period "
                        f"(wheel speed up to {moved:.3f} m/s, gyro z {turned:.3f} rad/s off its mean)")
        result["rest"] = {
            "gz_bias": statistics.fmean(s[4] for s in rest),
            "accel_mean": [statistics.fmean(s[i] for s in rest) for i in (1, 2, 3)],
            "accel_std": [statistics.pstdev(s[i] for s in rest) for i in (1, 2, 3)],
            "imu_samples": len(rest),
        }
        result["clock_offset_s"] = statistics.median(node.stamp_offsets) if node.stamp_offsets else None
        result["driver_params"] = node.get_params(r["driver_node"], r["driver_params"])

        # --- motion --------------------------------------------------------
        node.origin = node.odom[:2]
        node.imu.clear()
        node.gz_bias = result["rest"]["gz_bias"]
        t0 = time.monotonic()
        events.append(node.event("motion_start"))
        moving = True
        log(f"{'time':>6} {'phase':<7} {'cmd':>6} {'wheels':>7} {'odom':>8} {'heading':>9}")
        next_tick = next_print = t0
        cmd, unresponsive_since = 0.0, None
        while True:
            rclpy.spin_once(node, timeout_sec=max(0.0, next_tick - time.monotonic()))
            now = time.monotonic()
            if now < next_tick:
                continue
            next_tick = max(next_tick + period, now)  # never burst to catch up
            t, d, v = now - t0, node.distance(), node.speed()
            if node.odom_age() > c["odom_stale_s"]:
                raise Abort(f"odometry silent for {node.odom_age():.2f}s")
            if node.imu_age() > c["imu_stale_s"]:
                raise Abort(f"IMU silent for {node.imu_age():.2f}s")
            if d > prof.max_distance:
                raise Abort(f"odometry {d:.2f} m beyond the guard {prof.max_distance:.2f} m")
            if t > deadline:
                raise Abort(f"still running after {deadline:.0f}s")
            if cmd >= 0.1 and abs(v) < 0.02:  # last command was clearly moving, wheels aren't
                unresponsive_since = unresponsive_since if unresponsive_since is not None else t
                if t - unresponsive_since > c["response_timeout_s"]:
                    raise Abort(f"wheels not responding: commanded {cmd:.2f} m/s, measured {v:+.3f} m/s")
            else:
                unresponsive_since = None
            at_speed = steady.update(t, v)
            if ("time_to_speed_s" not in result["metrics"]
                    and abs(abs(v) - params["speed"]) <= c["steady_tolerance"] * params["speed"]):
                result["metrics"]["time_to_speed_s"] = t  # first measured sample within the band
            cmd = prof.command(t, d, v, period)
            if cmd is None:
                break
            node.send(cmd)
            samples.append((t, cmd, v, d, prof.phase, at_speed))
            if now >= next_print:
                next_print += PRINT_PERIOD_S
                log(f"{t:5.1f}s {prof.phase:<7} {cmd:+.3f} {v:+.3f}  {d:6.3f} m "
                    f"{math.degrees(node.heading):+6.1f} deg{'  at speed' if at_speed else ''}")
        node.send(0.0)
        stop = {"t": time.monotonic() - t0, "odom_m": node.distance(), "speed": node.speed()}
        if test == "T3":
            stop["steady_s"] = steady.held(stop["t"])
            stop["speed_avg"] = sum(steady.recent) / len(steady.recent)  # last 1 s, not one jittery sample
            stop["mode"] = params["stop_mode"]
            if params["stop_mode"] == "coast":
                node.request_bool(r["finish"][0][0], False)  # cut wheel power now; zeros keep going
        events.append(node.event("stop_command", **stop))
        result["stop_command"] = stop
        log(f"Stop command at {stop['t']:.2f}s, odometry {stop['odom_m']:.3f} m, wheels {stop['speed']:+.3f} m/s")
        result["status"] = "DONE"
    except (Abort, KeyboardInterrupt, Exception) as exc:
        if isinstance(exc, Abort):
            reason = str(exc)
        elif isinstance(exc, KeyboardInterrupt):
            reason = "stopped by operator"
        else:
            reason = f"error: {exc!r}"
            traceback.print_exc()
        result.update(status="ABORTED", reason=reason)
        log(f"ABORTED: {reason}")
    finally:
        try:
            if moving:
                stopping = []  # (t, odometry, wheel speed) while v=0 is held
                at_rest, still_since = hold_until_rest(node, c, period, t0, stopping)
                t_rest = (still_since if at_rest else time.monotonic()) - t0
                result["rest_after"] = {"t": t_rest, "odom_m": node.distance(extrapolate=False), "at_rest": at_rest,
                                        "wheels_powered": bool(node.relays)}
                events.append(node.event("at_rest", **result["rest_after"]))
                log(f"At rest at {t_rest:.2f}s: odometry {result['rest_after']['odom_m']:.3f} m"
                    + ("" if at_rest else " (rest NOT confirmed)"))
                metrics(test, params, result, samples, stopping, node.imu, node.heading, t0)
                write_samples(out_dir, samples, stopping)
        finally:
            for name, value in r["finish"]:
                ok, msg = node.set_bool(name, value)
                log(f"{name} {value}: {msg}")
            node.destroy_node()


def hold_until_rest(node, c, period, t0, stopping):
    """Publish v=0 at the control rate until the wheels report rest for rest_hold_s,
    appending (t, odometry, wheel speed) to `stopping`. Returns (confirmed, monotonic
    time the stillness began)."""
    end = time.monotonic() + c["rest_timeout_s"]
    still_since = None
    while time.monotonic() < end:
        node.send(0.0)
        node.spin_for(period)
        stopping.append((time.monotonic() - t0, node.distance(extrapolate=False), node.speed()))
        if abs(node.speed()) < c["rest_speed"]:
            still_since = still_since or time.monotonic()
            if time.monotonic() - still_since >= c["rest_hold_s"]:
                return True, still_since
        else:
            still_since = None
    return False, None


def write_samples(out_dir, samples, stopping):
    """The controller's own log at the control rate, for analysis without ROS tools:
    commanded and measured speed, odometry distance, phase. 'stopping' = v=0 held
    until rest (the T3 braking, the end of T1/T2)."""
    with open(os.path.join(out_dir, "samples.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "phase", "cmd_mps", "wheel_speed_mps", "odom_m", "at_speed"])
        for t, cmd, v, d, phase, at_speed in samples:
            w.writerow([f"{t:.3f}", phase, f"{cmd:.4f}", f"{v:.4f}", f"{d:.4f}", int(at_speed)])
        for t, d, v in stopping:
            w.writerow([f"{t:.3f}", "stopping", "0.0000", f"{v:.4f}", f"{d:.4f}", 0])


def imu_mean(m, name, imu, t0, start, end, base, forward_deg):
    """Mean IMU acceleration (gravity and rest bias removed) between start and end
    seconds after motion start. Means, not peaks: the lidar's vibration adds ±1-2 m/s²
    at 0.1 s scale, which averages out over a phase. imu_fwd_* is the component along
    the robot's forward axis (imu_forward_deg in the IMU x-y plane)."""
    if start is None or end is None or end <= start:
        return
    sel = [s for s in imu if start <= s[0] - t0 <= end]
    if sel:
        ax = statistics.fmean(s[1] - base[0] for s in sel)
        ay = statistics.fmean(s[2] - base[1] for s in sel)
        f = math.radians(forward_deg)
        m[f"imu_ax_{name}_mean"], m[f"imu_ay_{name}_mean"] = ax, ay
        m[f"imu_fwd_{name}_mean"] = ax * math.cos(f) + ay * math.sin(f)
        # direction of the mean acceleration: speed-up should read ~imu_forward_deg,
        # slow-down/braking ~180° from it; a drift would mean the mounting changed
        m[f"imu_{name}_dir_deg"] = math.degrees(math.atan2(ay, ax))


def speed_held(samples, target, c):
    """Measured speeds while commanded at target (cruise/hold) and the measured speed,
    averaged over a window centred on each sample, is within tolerance. Centred (after
    the run), unlike the live check, so it neither starts late nor runs into the slow-down."""
    half = c["steady_window_s"] / 2
    times = [s[0] for s in samples]
    speeds = [abs(s[2]) for s in samples]
    out, lo, hi, acc = [], 0, 0, 0.0
    for i, s in enumerate(samples):
        while hi < len(samples) and times[hi] <= s[0] + half:
            acc += speeds[hi]
            hi += 1
        while times[lo] < s[0] - half:
            acc -= speeds[lo]
            lo += 1
        if s[4] in ("cruise", "hold") and abs(acc / (hi - lo) - target) <= c["steady_tolerance"] * target:
            out.append(speeds[i])
    return out


def metrics(test, params, result, samples, stopping, imu, heading_rad, t0):
    m = result["metrics"]
    rest = result.get("rest_after", {})
    m["motion_time_s"] = rest.get("t")
    m["odom_end_m"] = rest.get("odom_m")
    m["max_speed"] = max((abs(s[2]) for s in samples), default=None)
    m.setdefault("time_to_speed_s", None)  # set during the run when first at target speed
    held = speed_held(samples, params["speed"], result["control"])
    m["speed_held_mean"] = statistics.fmean(held) if held else None  # measured, only while at target
    m["speed_held_s"] = len(held) / result["control"]["rate_hz"]
    m["heading_change_deg"] = math.degrees(heading_rad)
    stop = result.get("stop_command")
    base = result["rest"]["accel_mean"]
    fwd = result["imu_forward_deg"]
    # speed-up: motion start until measured at speed; slow-down: first slow-down command until rest
    imu_mean(m, "speedup", imu, t0, 0.0, m["time_to_speed_s"], base, fwd)
    t_slow = next((s[0] for s in samples if s[4] == "decel"), stop["t"] if stop else None)
    if test != "T3":
        imu_mean(m, "slowdown", imu, t0, t_slow, rest.get("t"), base, fwd)
    if test == "T1":
        if stop and rest:
            m["coast_after_stop_m"] = rest["odom_m"] - stop["odom_m"]
    elif test == "T2":
        target = params["speed"]
        t10 = next((s[0] for s in samples if abs(s[2]) >= 0.1 * target), None)
        t90 = next((s[0] for s in samples if abs(s[2]) >= 0.9 * target), None)
        m["t10_s"], m["t90_s"] = t10, t90
        m["planned_t90_s"] = 0.9 * target / params["accel"]
        m["accel_measured"] = 0.8 * target / (t90 - t10) if t10 is not None and t90 and t90 > t10 else None
    elif test == "T3" and stop and rest:
        m["trigger_odom_m"] = stop["odom_m"]
        m["speed_at_trigger"] = stop["speed_avg"]
        m["wheels_powered_at_rest"] = rest["wheels_powered"]  # false: power was cut while still braking
        m["steady_before_trigger_s"] = stop["steady_s"]
        m["steady_ok"] = stop["steady_s"] >= params["min_steady_s"]
        m["braking_odom_m"] = rest["odom_m"] - stop["odom_m"]
        m["braking_time_s"] = rest["t"] - stop["t"]  # until confirmed rest (|v| < rest_speed)
        # until 95% of the braking distance: robust to the wheel-speed jitter near zero
        target = stop["odom_m"] + 0.95 * m["braking_odom_m"]
        m["braking_t95_s"] = next((s[0] for s in stopping if s[1] >= target), rest["t"]) - stop["t"]
        imu_mean(m, "braking", imu, t0, stop["t"], rest["t"], base, fwd)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--test", required=True, help="T1, T2 or T3")
    ap.add_argument("--set", action="append", default=[], metavar="KEY=VALUE",
                    help="Condition and overrides of protocol.yaml values, e.g. speed=0.5 distance=3")
    ap.add_argument("--out", required=True, help="Trial folder; trial.json is written there")
    ap.add_argument("--id", default=None, help="Trial ID (default: folder name)")
    ap.add_argument("--repeat", type=int, default=None)
    ap.add_argument("--code-version", default=None)
    args = ap.parse_args()

    overrides = dict(kv.split("=", 1) for kv in args.set)
    try:
        cfg, params = load_params(args.test, overrides)
    except (ValueError, KeyError) as exc:
        sys.exit(f"ERROR: {exc}")
    os.makedirs(args.out, exist_ok=True)
    path = os.path.join(args.out, "trial.json")
    result = {
        "id": args.id or os.path.basename(os.path.normpath(args.out)),
        "test": args.test, "name": cfg["tests"][args.test]["name"],
        "condition": {params["condition"]: params[params["condition"]]},
        "repeat": args.repeat, "params": params, "control": cfg["control"],
        "code_version": args.code_version, "started": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "status": "RUNNING", "events": [], "metrics": {}, "manual": {},
    }
    write(path, result)

    # Both stop the robot cleanly; set explicitly in case SIGINT was inherited as ignored
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    try:
        run(cfg, args.test, params, result, args.out)
    finally:
        result["finished"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
        write(path, result)
        rclpy.shutdown()
    print(json.dumps({"status": result["status"], **result["metrics"]}, default=str), flush=True)
    return 0 if result["status"] == "DONE" else 1


def write(path, result):
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(result, f, indent=2, default=str)
    os.replace(tmp, path)


if __name__ == "__main__":
    sys.exit(main())

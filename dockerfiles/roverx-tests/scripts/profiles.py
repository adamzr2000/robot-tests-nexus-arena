"""Speed profiles for the RoverX floor tests, free of ROS so they can be dry-run.

A profile is asked for a command every control period with the current state
(time since motion start, progress, measured speed) and returns None when its
motion is over; the runner then holds zero until the robot is at rest. For the
straight tests the command is forward speed (m/s) and progress the odometry
distance (m); for T4 (`angular = True`) the command is the turn rate (rad/s,
signed) and progress the heading turned in that direction (rad, IMU).
`speed` is the target magnitude; `phase` names what the profile is doing.
"""
import math
from collections import deque


class Steady:
    """Whether the measured speed holds `target`: its average over the last
    `window` updates is within `tolerance` (fraction) of the target. `since`
    is when that started (None while not steady)."""

    def __init__(self, target, tolerance, window):
        self.target, self.tolerance = target, tolerance * target
        self.recent = deque(maxlen=window)
        self.since = None

    def update(self, t, v):
        self.recent.append(abs(v))
        avg = sum(self.recent) / len(self.recent)
        ok = len(self.recent) == self.recent.maxlen and abs(avg - self.target) <= self.tolerance
        self.since = (self.since if self.since is not None else t) if ok else None
        return ok

    def held(self, t):
        return 0.0 if self.since is None else t - self.since


class Straight:
    """T1 and T2: accelerate at `accel` to `speed`, cruise, then decelerate at
    `decel` (T1: same as accel) down to a crawl, and stop when odometry reads
    `distance`."""
    angular = False

    def __init__(self, p, crawl):
        self.speed, self.accel, self.distance = p["speed"], p["accel"], p["distance"]
        self.decel = p.get("decel", self.accel)
        self.crawl = min(crawl, self.speed)
        self.max_distance = self.distance + p["overshoot_guard"]
        self.cmd = 0.0
        self.phase = "accel"

    def command(self, t, d, v, dt):
        remaining = self.distance - d
        if remaining <= 0:
            self.phase = "stop"
            return None
        up = min(self.speed, self.cmd + self.accel * dt)
        down = max(self.crawl, math.sqrt(2 * self.decel * remaining))
        self.cmd = min(up, down)
        if self.cmd >= self.speed:
            self.phase = "cruise"
        elif up < down:
            self.phase = "accel"
        else:
            self.phase = "crawl" if self.cmd <= self.crawl else "decel"
        return self.cmd


class T3Stop:
    """As T1 up to `distance` (accelerate at `accel` to `speed`, cruise), but no
    slow-down: when odometry reads `distance` return None, the sudden stop command.
    The runner's Steady tracker tells how long the robot held the speed, so a run
    not at speed at the stop is flagged."""
    angular = False

    def __init__(self, p, crawl):
        self.speed, self.accel, self.stop_at = p["speed"], p["accel"], p["distance"]
        self.max_distance = self.stop_at + p["braking_guard"]
        self.cmd = 0.0
        self.phase = "accel"

    def command(self, t, d, v, dt):
        if d >= self.stop_at:
            self.phase = "stop"
            return None
        self.cmd = min(self.speed, self.cmd + self.accel * dt)
        self.phase = "cruise" if self.cmd >= self.speed else "accel"
        return self.cmd


class T4Rotate:
    """In place (v = 0): ramp the turn rate at `ang_accel` to `yaw_rate` (signed,
    + counter-clockwise), hold it, and when the heading has turned `angle_deg`
    return None, the stop command (v = 0, w = 0 at once; the overshoot is measured)."""
    angular = True

    def __init__(self, p, crawl):
        self.rate, self.accel = p["yaw_rate"], p["ang_accel"]
        self.speed = abs(self.rate)
        self.stop_at = math.radians(p["angle_deg"])
        self.max_distance = self.stop_at + math.radians(p["heading_guard_deg"])
        self.cmd = 0.0
        self.phase = "accel"

    def command(self, t, d, v, dt):
        if d >= self.stop_at:
            self.phase = "stop"
            return None
        self.cmd = min(self.speed, self.cmd + self.accel * dt)
        self.phase = "cruise" if self.cmd >= self.speed else "accel"
        return math.copysign(self.cmd, self.rate)


PROFILES = {"T1": Straight, "T2": Straight, "T3": T3Stop, "T4": T4Rotate}


def make(test, params, crawl):
    return PROFILES[test](params, crawl)


def plan(test, params, crawl, rate_hz):
    """Run the profile against an ideal robot (moves exactly as commanded) and
    report what it commands: duration, distance, top speed, time per phase.
    Real robots lag, slip and (T3) brake over an unknown distance."""
    prof = make(test, params, crawl)
    dt = 1.0 / rate_hz
    t = d = v = top = 0.0
    phases = {}
    steady = Steady(prof.speed, 0.05, 1)
    while True:
        steady.update(t, v)
        cmd = prof.command(t, d, v, dt)
        if cmd is None:
            break
        phases[prof.phase] = phases.get(prof.phase, 0.0) + dt
        v = abs(cmd)
        top = max(top, v)
        d += v * dt
        t += dt
        if t > 600:
            raise ValueError("profile does not end within 600 s")
    if prof.angular:
        return {"motion_s": round(t, 2), "angle_deg": round(math.degrees(d), 1), "top_rate": round(top, 3),
                "phases_s": {k: round(s, 2) for k, s in phases.items()},
                "abort_beyond_deg": round(math.degrees(prof.max_distance), 1),
                "note": "plus the overshoot after the stop command (measured)"}
    out = {"motion_s": round(t, 2), "distance_m": round(d, 3), "top_speed": round(top, 3),
           "phases_s": {k: round(s, 2) for k, s in phases.items()},
           "abort_beyond_m": round(prof.max_distance, 2)}
    if test == "T3":
        out["steady_at_stop_s"] = round(steady.held(t), 2)
        out["note"] = "plus the braking distance after the stop command (measured, unknown in advance)"
    return out

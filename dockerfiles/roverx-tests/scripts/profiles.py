"""Speed profiles for the RoverX floor tests, free of ROS so they can be dry-run.

A profile is asked for a forward speed command every control period with the
current state (time since motion start, odometry distance, measured speed) and
returns None when its motion is over; the runner then holds v=0 until the robot
is at rest. `phase` names what the profile is doing, for logs and metrics.
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


class T1Straight:
    """Accelerate at `accel` to `speed`, cruise, then decelerate at `accel`
    (down to a crawl) and stop when odometry reads `distance`."""

    def __init__(self, p, crawl):
        self.speed, self.accel, self.distance = p["speed"], p["accel"], p["distance"]
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
        down = max(self.crawl, math.sqrt(2 * self.accel * remaining))
        self.cmd = min(up, down)
        if self.cmd >= self.speed:
            self.phase = "cruise"
        elif up < down:
            self.phase = "accel"
        else:
            self.phase = "crawl" if self.cmd <= self.crawl else "decel"
        return self.cmd


class T2Accel:
    """From rest, ramp at `accel` to `speed`, hold `hold_s`, slow down at
    `decel`. Purely timed: the command is the setpoint under test."""

    def __init__(self, p, crawl):
        self.speed, self.accel, self.decel = p["speed"], p["accel"], p["decel"]
        self.t_up = self.speed / self.accel
        self.t_hold = self.t_up + p["hold_s"]
        self.t_end = self.t_hold + self.speed / self.decel
        planned = (self.speed ** 2 / (2 * self.accel) + self.speed * p["hold_s"]
                   + self.speed ** 2 / (2 * self.decel))
        self.max_distance = planned + p["distance_guard"]
        self.phase = "accel"

    def command(self, t, d, v, dt):
        if t < self.t_up:
            self.phase = "accel"
            return self.accel * t
        if t < self.t_hold:
            self.phase = "hold"
            return self.speed
        if t < self.t_end:
            self.phase = "decel"
            return self.speed - self.decel * (t - self.t_hold)
        self.phase = "stop"
        return None


class T3Stop:
    """Accelerate at `accel` to `speed` and cruise; at odometry `trigger`
    return None (the stop command). The runner's Steady tracker tells how
    long the robot held the speed, so a run not at speed at the trigger is flagged."""

    def __init__(self, p, crawl):
        self.speed, self.accel, self.trigger = p["speed"], p["accel"], p["trigger"]
        self.max_distance = self.trigger + p["braking_guard"]
        self.cmd = 0.0
        self.phase = "accel"

    def command(self, t, d, v, dt):
        if d >= self.trigger:
            self.phase = "stop"
            return None
        self.cmd = min(self.speed, self.cmd + self.accel * dt)
        self.phase = "cruise" if self.cmd >= self.speed else "accel"
        return self.cmd


PROFILES = {"T1": T1Straight, "T2": T2Accel, "T3": T3Stop}


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
    steady = Steady(params["speed"], 0.05, 1)
    while True:
        steady.update(t, v)
        cmd = prof.command(t, d, v, dt)
        if cmd is None:
            break
        phases[prof.phase] = phases.get(prof.phase, 0.0) + dt
        v = cmd
        top = max(top, v)
        d += v * dt
        t += dt
        if t > 600:
            raise ValueError("profile does not end within 600 s")
    out = {"motion_s": round(t, 2), "distance_m": round(d, 3), "top_speed": round(top, 3),
           "phases_s": {k: round(s, 2) for k, s in phases.items()},
           "abort_beyond_m": round(prof.max_distance, 2)}
    if test == "T3":
        out["steady_at_trigger_s"] = round(steady.held(t), 2)
        out["note"] = "plus the braking distance after the trigger (measured, unknown in advance)"
    return out

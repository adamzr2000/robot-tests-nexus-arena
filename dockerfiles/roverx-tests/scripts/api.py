#!/usr/bin/env python3
"""HTTP API for the RoverX floor tests (port 8090). Stdlib only.

  POST /t1  {"speed": 0.5}      T1 straight-line baseline
  POST /t2  {"accel": 0.2}      T2 acceleration response
  POST /t3  {"speed": 1.0}      T3 stopping response
            optional: any protocol.yaml setting to override (e.g. "distance": 3),
            "record": false (no bag, nothing saved, not counted), "dry_run": true (plan only),
            "notes": "..."
  POST /stop                    stop the running trial (the robot stops and powers off)
  GET  /status                  running trial, last result, DONE count per condition

One trial at a time: trial.py runs as a subprocess (live output in
`docker logs -f roverx-tests`) while the API records its bag. Recorded trials
go to results/<session>/<trial id>/ (trial.json + bag/), <session> = date
unless SESSION is set. Trial IDs: <test>_<v|a><setpoint>_r<repeat>_<HHMMSS>,
e.g. T1_v0.50_r3_101530; only DONE trials count as repeats.

Tape measurements go in results/<session>/manual.csv (one row per trial, added
automatically; edit it in Excel/LibreOffice, the API never overwrites what you
type). run_log.csv = trial.json results + manual.csv, rebuilt after every trial
and on GET /status.
"""
import csv
import glob
import hashlib
import json
import math
import os
import signal
import subprocess
import sys
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import profiles
import trial

HERE = os.path.dirname(os.path.abspath(__file__))
RESULTS = os.environ.get("RESULTS_DIR", os.path.expanduser("~/results"))
PORT = int(os.environ.get("PORT", "8090"))
BAG_WARMUP_S = 1.5   # recorder discovers topics before the trial starts
BAG_TAIL_S = 1.0     # keep recording after the trial ends
STOP_WAIT_S = 30.0
SETPOINT_LABEL = {"speed": "v", "accel": "a"}

# manual.csv: the API owns the first columns, the operator the rest
MANUAL_AUTO = ["trial_id", "test", "setpoint", "repeat", "status"]
MANUAL_OPERATOR = ["dx_cm", "dy_cm", "total_cm", "override_status", "observations", "notes"]

_lock = threading.Lock()
_current = {}  # id, test, proc, bag, dir, record, started, watcher
_last = {}


def session_dir():
    return os.path.join(RESULTS, os.environ.get("SESSION") or time.strftime("%Y-%m-%d"))


def code_version():
    h = hashlib.sha256()
    for path in sorted(glob.glob(os.path.join(HERE, "*.py")) + glob.glob(os.path.join(HERE, "*.yaml"))):
        h.update(open(path, "rb").read())
    return h.hexdigest()[:12]


def load_trials():
    out = []
    for path in sorted(glob.glob(os.path.join(session_dir(), "*", "trial.json"))):
        try:
            out.append(json.load(open(path)))
        except (OSError, ValueError):
            pass
    return out


def setpoint(t):
    return next(iter(t["condition"].values()))


# --- manual.csv ----------------------------------------------------------------

def read_manual():
    """{trial_id: row} plus the file's delimiter. Accepts ',' ';' or tab and
    decimal commas, as Excel/LibreOffice may save it."""
    path = os.path.join(session_dir(), "manual.csv")
    if not os.path.exists(path):
        return {}, ","
    text = open(path, encoding="utf-8-sig").read()
    try:
        delim = csv.Sniffer().sniff(text.splitlines()[0], delimiters=",;\t").delimiter
    except (csv.Error, IndexError):
        delim = ","
    rows = {r["trial_id"]: r for r in csv.DictReader(text.splitlines(), delimiter=delim) if r.get("trial_id")}
    return rows, delim


def number(value):
    try:
        return float(str(value).strip().replace(",", ".")) if str(value).strip() else None
    except ValueError:
        return None


def update_manual(trials):
    """Add a row for every recorded trial, refresh the API's columns, keep the operator's."""
    rows, delim = read_manual()
    path = os.path.join(session_dir(), "manual.csv")
    with open(path + ".tmp", "w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=MANUAL_AUTO + MANUAL_OPERATOR, delimiter=delim, extrasaction="ignore")
        w.writeheader()
        for t in trials:
            row = rows.get(t["id"], {})
            row.update(trial_id=t["id"], test=t["test"], setpoint=setpoint(t), repeat=t.get("repeat"),
                       status=t.get("status") + (f" ({t['reason']})" if t.get("reason") else ""))
            w.writerow(row)
    os.replace(path + ".tmp", path)


def effective_status(t, manual_row):
    return (manual_row or {}).get("override_status") or t.get("status")


# --- run log -------------------------------------------------------------------

CSV_COLUMNS = [
    "trial_id", "test", "setpoint", "repeat", "status", "reason", "started", "overrides",
    "motion_time_s", "time_to_speed_s", "speed_held_s", "speed_held_mean", "max_speed",
    "odom_end_m", "heading_change_deg",
    "coast_after_stop_m",                                                       # T1
    "t10_s", "t90_s", "planned_t90_s", "accel_measured",                        # T2
    "stop_mode", "trigger_odom_m", "speed_at_trigger", "steady_before_trigger_s", "steady_ok",
    "braking_odom_m", "braking_time_s", "braking_t95_s", "wheels_powered_at_rest",                        # T3
    "imu_fwd_speedup_mean", "imu_speedup_dir_deg", "imu_fwd_slowdown_mean", "imu_slowdown_dir_deg",
    "imu_fwd_braking_mean", "imu_braking_dir_deg", "clock_offset_s",
    "dx_cm", "dy_cm", "endpoint_error_cm", "odom_error_cm",                     # T1 tape
    "total_cm", "stop_distance_cm",                                             # T3 tape
    "observations", "notes", "bag",
]


def csv_row(t, man):
    m, p = t.get("metrics", {}), t["params"]
    row = {k: m.get(k) for k in CSV_COLUMNS if k in m}
    row.update(trial_id=t["id"], test=t["test"], setpoint=setpoint(t), repeat=t.get("repeat"),
               status=effective_status(t, man), reason=t.get("reason"), started=t.get("started"),
               overrides=json.dumps(t.get("overrides", {})), clock_offset_s=t.get("clock_offset_s"),
               stop_mode=p.get("stop_mode"), observations=man.get("observations"),
               notes="; ".join(x for x in (t.get("notes"), man.get("notes")) if x),
               bag=os.path.join(t["id"], "bag"))
    dx, dy, total = number(man.get("dx_cm")), number(man.get("dy_cm")), number(man.get("total_cm"))
    row.update(dx_cm=dx, dy_cm=dy, total_cm=total)
    if t["test"] == "T1" and dx is not None and dy is not None:
        row["endpoint_error_cm"] = math.hypot(dx, dy)
        if m.get("odom_end_m") is not None:  # real travel (distance + dx) minus what odometry says
            row["odom_error_cm"] = p["distance"] * 100 + dx - m["odom_end_m"] * 100
    if t["test"] == "T3" and total is not None and m.get("trigger_odom_m") is not None:
        row["stop_distance_cm"] = total - m["trigger_odom_m"] * 100
    return {k: (round(v, 4) if isinstance(v, float) else v) for k, v in row.items()}


def refresh_logs():
    """Rebuild manual.csv rows and run_log.csv from the trial.json files. Caller holds _lock."""
    trials = load_trials()
    if not trials:
        return
    update_manual(trials)
    manual_rows, _ = read_manual()
    path = os.path.join(session_dir(), "run_log.csv")
    with open(path + ".tmp", "w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=CSV_COLUMNS)
        w.writeheader()
        for t in trials:
            w.writerow(csv_row(t, manual_rows.get(t["id"], {})))
    os.replace(path + ".tmp", path)


def done_counts():
    cfg, trials = trial.load_protocol(), load_trials()
    manual_rows, _ = read_manual()
    out = {}
    for test, spec in cfg["tests"].items():
        done = [setpoint(t) for t in trials
                if t["test"] == test and effective_status(t, manual_rows.get(t["id"])) == "DONE"]
        out[test] = {f"{spec['condition']} {v:g}": f"{sum(math.isclose(d, v) for d in done)}/{spec['repetitions']}"
                     for v in spec["values"]}
    return out


# --- trials --------------------------------------------------------------------

def start_trial(test, body):
    """Validate, then start the bag (if recording) and the trial. Returns (http status, payload)."""
    body = dict(body)
    dry_run = body.pop("dry_run", False)
    record = body.pop("record", True)
    notes = body.pop("notes", "")
    if not isinstance(record, bool) or not isinstance(dry_run, bool):
        return 400, {"error": "'record' and 'dry_run' must be true/false"}
    try:
        cfg, params = trial.load_params(test, body)
    except (ValueError, KeyError, TypeError) as exc:
        return 400, {"error": str(exc)}
    c = cfg["control"]
    plan = profiles.plan(test, params, c["crawl_speed"], c["rate_hz"])
    if dry_run:
        return 200, {"dry_run": True, "test": test, "params": params, "plan": plan}

    with _lock:
        if _current.get("proc") and _current["proc"].poll() is None:
            return 409, {"error": f"trial {_current['id']} is running; POST /stop first"}
        cond = params["condition"]
        label = f"{SETPOINT_LABEL.get(cond, cond)}{params[cond]:.2f}"
        stamp = time.strftime("%H%M%S")
        if record:
            manual_rows, _ = read_manual()
            repeat = 1 + sum(1 for t in load_trials() if t["test"] == test and math.isclose(setpoint(t), params[cond])
                             and effective_status(t, manual_rows.get(t["id"])) == "DONE")
            tid = f"{test}_{label}_r{repeat}_{stamp}"
            tdir = os.path.join(session_dir(), tid)
        else:
            repeat = None
            tid = f"{test}_{label}_test_{stamp}"
            tdir = os.path.join(tempfile.gettempdir(), "roverx-tests", tid)
        os.makedirs(tdir)
        bag = None
        if record:
            bag = subprocess.Popen(["ros2", "bag", "record", "-o", os.path.join(tdir, "bag"), *cfg["bag_topics"]],
                                   stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
            time.sleep(BAG_WARMUP_S)
            if bag.poll() is not None:
                return 502, {"error": f"bag recorder exited with code {bag.returncode}; trial not started"}
        cmd = [sys.executable, os.path.join(HERE, "trial.py"), "--test", test, "--out", tdir, "--id", tid,
               "--code-version", code_version(), *(["--repeat", str(repeat)] if repeat else []),
               *[a for k, v in body.items() for a in ("--set", f"{k}={v}")]]
        proc = subprocess.Popen(cmd)
        watcher = threading.Thread(target=_watch, args=(proc,), daemon=True)
        _current.clear()
        _current.update(id=tid, test=test, proc=proc, bag=bag, dir=tdir, record=record, started=time.time(),
                        watcher=watcher, notes=notes, overrides={k: v for k, v in body.items() if k != cond})
        watcher.start()
    return 200, {"started": tid, "recorded": record, "params": params, "plan": plan,
                 "follow": "docker logs -f roverx-tests"}


def _watch(proc):
    proc.wait()
    with _lock:
        if _current.get("proc") is not proc:
            return
        bag = _current["bag"]
        if bag:
            time.sleep(BAG_TAIL_S)
            bag.send_signal(signal.SIGINT)
            try:
                bag.wait(timeout=20)
            except subprocess.TimeoutExpired:
                bag.kill()
        path = os.path.join(_current["dir"], "trial.json")
        try:
            t = json.load(open(path))
        except (OSError, ValueError) as exc:
            print(f"[api] {_current['id']}: no readable trial.json ({exc}); trial.py exit code "
                  f"{proc.returncode}", flush=True)
            _current.clear()
            return
        if t.get("status") == "RUNNING":  # trial.py died without finishing
            t.update(status="FAILED", reason=f"trial.py exited with code {proc.returncode}")
        t.update(notes=_current["notes"], overrides=_current["overrides"], recorded=_current["record"])
        trial.write(path, t)
        _last.clear()
        _last.update(id=t["id"], recorded=_current["record"], status=t.get("status"), reason=t.get("reason"),
                     metrics=t.get("metrics"))
        print(f"[api] {t['id']} finished: {t.get('status')} {t.get('reason') or ''}"
              + ("" if _current["record"] else " (not recorded)"), flush=True)
        record = _current["record"]
        _current.clear()
        if record:
            refresh_logs()


def stop_trial(wait_bag=False):
    """Stop the running trial (trial.py stops the robot, then exits). With
    wait_bag, also wait until its bag is closed and results written."""
    with _lock:
        proc, watcher = _current.get("proc"), _current.get("watcher")
    if not proc:
        return False
    running = proc.poll() is None
    if running:
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=STOP_WAIT_S)
        except subprocess.TimeoutExpired:
            proc.kill()
    if wait_bag and watcher:
        watcher.join(timeout=STOP_WAIT_S)
    return running


class Handler(BaseHTTPRequestHandler):
    def _send(self, status, payload):
        body = json.dumps(payload, indent=1, default=str).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _body(self):
        n = int(self.headers.get("Content-Length", 0))
        data = json.loads(self.rfile.read(n)) if n else {}
        if not isinstance(data, dict):
            raise ValueError("body must be a JSON object")
        return data

    def do_GET(self):
        if self.path != "/status":
            self._send(404, {"error": "not found; endpoints: POST /t1 /t2 /t3 /stop, GET /status"})
            return
        with _lock:
            running = bool(_current.get("proc")) and _current["proc"].poll() is None
            cur = {"id": _current["id"], "recorded": _current["record"],
                   "elapsed_s": round(time.time() - _current["started"], 1)} if running else None
            refresh_logs()  # picks up manual.csv edits
            counts = done_counts()
        self._send(200, {"session": session_dir(), "running": cur, "last": _last or None, "done": counts})

    def do_POST(self):
        try:
            body = self._body()
        except ValueError as exc:
            self._send(400, {"error": f"bad JSON body: {exc}"})
            return
        test = {"/t1": "T1", "/t2": "T2", "/t3": "T3"}.get(self.path)
        if test:
            self._send(*start_trial(test, body))
        elif self.path == "/stop":
            self._send(200, {"stopped": stop_trial()})
        else:
            self._send(404, {"error": "not found; endpoints: POST /t1 /t2 /t3 /stop, GET /status"})

    def log_message(self, fmt, *args):
        print(f"[api] {self.address_string()} " + fmt % args, flush=True)


def main():
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.default_int_handler)  # docker stop: stop the trial cleanly
    os.makedirs(RESULTS, exist_ok=True)
    server = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    print(f"RoverX tests API on :{PORT}, results in {RESULTS}, code {code_version()}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop_trial(wait_bag=True)
        server.server_close()


if __name__ == "__main__":
    main()

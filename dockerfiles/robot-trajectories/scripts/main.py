#!/usr/bin/env python3
"""Minimal HTTP API that triggers trajectory scripts in this folder.

Stdlib only, no ROS2 package: launches a script (e.g. circle_walk.py) as a
subprocess, translating the JSON body into its CLI flags 1:1 ("angular_speed":
0.5 -> --angular-speed 0.5; a boolean becomes a flag or nothing). Starting a
trajectory stops whatever was running, since two scripts can't share a
cmd_vel topic. Add a new one with one line in TRAJECTORIES below.

Two body keys are for the API itself, not the script: "record_bag": true
records a bag (via the bags-collector API at BAGS_URL) for exactly as long as
the trajectory runs, with the robot profile's bag_topics plus any explicit
topic/odom_topic; "tag" is added to the bag name, <robot>_<trajectory>_<tag>.

Endpoints:
  GET  /trajectories             list available trajectory names
  POST /trajectories/<name>/run  start <name>; JSON body = its CLI flags
  POST /stop                     stop whatever is running
  GET  /status                   currently running trajectory and bag, if any

Example:
  curl -X POST localhost:8080/trajectories/circle/run \
       -d '{"robot": "go2", "radius": 1.0, "angular_speed": 0.5, "laps": 2, "use_odom": true,
            "record_bag": true, "tag": "exp1"}'
"""
import argparse
import json
import os
import signal
import subprocess
import threading
import time
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from robot_profiles import ROBOT_PROFILES

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# Add a new trajectory by mapping its name to its script filename here.
TRAJECTORIES = {
    "circle": "circle_walk.py",
    "square": "square_walk.py",
}

# Fast-fail check: wait this long after launch, then see if the script
# already died (e.g. a bad flag). Not a guarantee it'll run forever after.
STARTUP_CHECK_S = 0.3

BAGS_URL = os.environ.get("BAGS_URL", "http://localhost:8081")
BAG_TAIL_S = 1.0  # keep recording this long after the trajectory ends, to catch the robot stopping

_lock = threading.Lock()
_current = {"name": None, "proc": None, "bag": None}
_last = {"trajectory": None, "exit_code": None}  # last finished run; 0 = completed


def _params_to_args(params):
    args = []
    for key, value in params.items():
        flag = "--" + key.replace("_", "-")
        if isinstance(value, bool):
            if value:
                args.append(flag)
            # false -> omit the flag; scripts default to off
        else:
            args += [flag, str(value)]
    return args


def _is_alive(proc):
    return proc is not None and proc.poll() is None


def _bags_post(path, body):
    """POST to the bags-collector API; returns (ok, response JSON or error text)."""
    req = urllib.request.Request(BAGS_URL + path, data=json.dumps(body).encode(), method="POST",
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=20) as resp:
            return True, json.loads(resp.read())
    except urllib.error.HTTPError as exc:
        return False, exc.read().decode(errors="replace")
    except (urllib.error.URLError, OSError, ValueError) as exc:
        return False, str(exc)


def _bag_topics(params):
    """Profile bag_topics plus explicit topic overrides; never everything."""
    topics = list(ROBOT_PROFILES.get(params.get("robot"), {}).get("bag_topics", []))
    extra = [params.get("topic") or "/cmd_vel", params.get("odom_topic")]
    return topics + [t for t in extra if t and t not in topics]


def _start_bag(params, name, tag):
    label = "_".join(part for part in (params.get("robot"), name, tag) if part)
    topics = _bag_topics(params)
    ok, reply = _bags_post("/start", {"tag": label, "topics": topics})
    if not ok:
        raise RuntimeError(f"bags-collector at {BAGS_URL} refused to record: {reply}")
    return reply["started"]


def _stop_current_locked():
    """Caller must hold _lock. Stops the trajectory, then its bag."""
    proc = _current["proc"]
    if _is_alive(proc):
        proc.send_signal(signal.SIGINT)  # same as Ctrl+C: scripts stop the robot on this
        try:
            proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            proc.kill()
    if _current["bag"]:
        time.sleep(BAG_TAIL_S)
        ok, reply = _bags_post("/stop", {})
        print(f"[api] bag {_current['bag']} " + ("saved" if ok else f"stop failed: {reply}"), flush=True)
    if proc is not None:
        _last.update(trajectory=_current["name"], exit_code=proc.returncode)
        print(f"[api] {_current['name']} exited with code {proc.returncode}", flush=True)
    _current.update(name=None, proc=None, bag=None)


def _watch(proc):
    """Ends the bag as soon as a trajectory finishes on its own."""
    proc.wait()
    with _lock:
        if _current["proc"] is proc:
            _stop_current_locked()


class Handler(BaseHTTPRequestHandler):
    def _send_json(self, status, payload):
        body = json.dumps(payload).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_json_body(self):
        length = int(self.headers.get("Content-Length", 0))
        if length == 0:
            return {}
        return json.loads(self.rfile.read(length))

    def do_GET(self):
        if self.path == "/trajectories":
            self._send_json(200, {"trajectories": sorted(TRAJECTORIES)})
        elif self.path == "/status":
            with _lock:
                running = _is_alive(_current["proc"])
                self._send_json(200, {
                    "running": running,
                    "trajectory": _current["name"] if running else None,
                    "bag": _current["bag"] if running else None,
                    "last": _last,
                })
        else:
            self._send_json(404, {"error": "not found"})

    def do_POST(self):
        if self.path == "/stop":
            with _lock:
                was_running = _is_alive(_current["proc"])
                _stop_current_locked()
            self._send_json(200, {"stopped": was_running})
            return

        if self.path.startswith("/trajectories/") and self.path.endswith("/run"):
            name = self.path[len("/trajectories/"):-len("/run")]
            script = TRAJECTORIES.get(name)
            if script is None:
                self._send_json(404, {"error": f"unknown trajectory: {name}",
                                       "available": sorted(TRAJECTORIES)})
                return

            try:
                params = self._read_json_body()
                record_bag = params.pop("record_bag", False)
                tag = params.pop("tag", "")
                if not isinstance(record_bag, bool) or not isinstance(tag, str):
                    raise ValueError("'record_bag' must be true/false and 'tag' a string")
                cli_args = _params_to_args(params)
            except ValueError as exc:
                self._send_json(400, {"error": f"bad JSON body: {exc}"})
                return

            cmd = ["python3", os.path.join(SCRIPT_DIR, script), *cli_args]
            with _lock:
                _stop_current_locked()
                bag = None
                if record_bag:
                    try:
                        bag = _start_bag(params, name, tag)
                    except RuntimeError as exc:
                        self._send_json(502, {"error": str(exc)})
                        return
                proc = subprocess.Popen(cmd)
                _current.update(name=name, proc=proc, bag=bag)
                threading.Thread(target=_watch, args=(proc,), daemon=True).start()

            time.sleep(STARTUP_CHECK_S)
            exit_code = proc.poll()
            if exit_code is not None:
                self._send_json(400, {
                    "error": f"{name} exited immediately with code {exit_code}; "
                             "check `docker logs` for details",
                    "command": cmd,
                })
                return

            self._send_json(200, {"started": name, "command": cmd, "bag": bag})
            return

        self._send_json(404, {"error": "not found"})

    def log_message(self, fmt, *args):
        print(f"[api] {self.address_string()} " + fmt % args, flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", type=int, default=8080, help="HTTP port to listen on (default: 8080)")
    args = parser.parse_args()

    server = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)
    print(f"Trajectory API listening on :{args.port} — trajectories: {sorted(TRAJECTORIES)}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        with _lock:
            _stop_current_locked()
        server.server_close()


if __name__ == "__main__":
    main()

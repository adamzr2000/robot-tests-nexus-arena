#!/usr/bin/env python3
"""Minimal HTTP API to start/stop ros2 bag recording.

Stdlib only, no ROS2 package: launches `ros2 bag record` as a subprocess.
Stopping it with SIGINT is the normal way to end a recording (same as
Ctrl+C), so rosbag2 flushes its metadata properly instead of leaving a
corrupt bag.

Endpoints:
  POST /start   start recording; JSON body: {"tag": "...", "topics": [...]}
  POST /stop    stop the current recording
  GET  /status  currently recording bag, if any
  GET  /bags    list bags under BAGS_DIR, with sizes

Bag folders are named <timestamp-with-microseconds>_<tag> under BAGS_DIR (default
/home/robot/bags, bind-mount this to the host for easy export). "topics" is
optional: omit it to use DEFAULT_TOPICS below, or record everything
(`ros2 bag record -a`) if that's empty too. "tag" is optional and used to
label the run; it's sanitized to safe filename characters.

Only one recording runs at a time: /start refuses (409) if one is already
in progress rather than silently stopping it, since that could cut an
active recording short without the caller noticing.

Example:
  curl -X POST localhost:8081/start -d '{"tag": "exp1", "topics": ["/cmd_vel", "/go2/scan"]}'
  curl localhost:8081/status
  curl -X POST localhost:8081/stop
  curl localhost:8081/bags
"""
import argparse
import json
import os
import re
import signal
import subprocess
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

BAGS_DIR = os.environ.get("BAGS_DIR", os.path.expanduser("~/bags"))

# Used when a /start request omits "topics". Empty means record everything.
# Edit this to your project's usual topic set, e.g. ["/cmd_vel", "/go2/scan"].
DEFAULT_TOPICS = []

STARTUP_CHECK_S = 0.5
SAFE_TAG_RE = re.compile(r"[^A-Za-z0-9_-]")

_lock = threading.Lock()
_current = {"name": None, "path": None, "proc": None, "topics": None}


def _sanitize_tag(tag):
    tag = SAFE_TAG_RE.sub("", tag or "")
    return tag or "run"


def _is_alive(proc):
    return proc is not None and proc.poll() is None


def _stop_current_locked(timeout=15):
    """Caller must hold _lock. Returns whether something was actually
    stopped, and clears _current either way."""
    proc = _current["proc"]
    was_recording = _is_alive(proc)
    if was_recording:
        proc.send_signal(signal.SIGINT)  # same as Ctrl+C: lets rosbag2 flush cleanly
        try:
            proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            proc.kill()
    _current.update(name=None, path=None, proc=None, topics=None)
    return was_recording


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
        if self.path == "/status":
            with _lock:
                running = _is_alive(_current["proc"])
                self._send_json(200, {
                    "recording": running,
                    "name": _current["name"] if running else None,
                    "path": _current["path"] if running else None,
                    "topics": _current["topics"] if running else None,
                })
        elif self.path == "/bags":
            os.makedirs(BAGS_DIR, exist_ok=True)
            bags = []
            for entry in sorted(os.listdir(BAGS_DIR)):
                full = os.path.join(BAGS_DIR, entry)
                if os.path.isdir(full):
                    size = sum(os.path.getsize(os.path.join(dp, f))
                               for dp, _, files in os.walk(full) for f in files)
                    bags.append({"name": entry, "bytes": size})
            self._send_json(200, {"bags": bags})
        else:
            self._send_json(404, {"error": "not found"})

    def do_POST(self):
        if self.path == "/start":
            with _lock:
                if _is_alive(_current["proc"]):
                    self._send_json(409, {"error": "already recording", "name": _current["name"]})
                    return

                try:
                    body = self._read_json_body()
                except ValueError as exc:
                    self._send_json(400, {"error": f"bad JSON body: {exc}"})
                    return

                topics = body.get("topics") or DEFAULT_TOPICS
                if not isinstance(topics, list) or not all(isinstance(t, str) for t in topics):
                    self._send_json(400, {"error": "'topics' must be a list of strings"})
                    return
                tag = body.get("tag") or ""
                if not isinstance(tag, str):
                    self._send_json(400, {"error": "'tag' must be a string"})
                    return

                # Microseconds, not just seconds: two runs with the same tag
                # started within the same second would otherwise collide on
                # this folder name, and ros2 bag record refuses to reuse one.
                name = f"{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}_{_sanitize_tag(tag)}"
                bag_path = os.path.join(BAGS_DIR, name)
                os.makedirs(BAGS_DIR, exist_ok=True)

                cmd = ["ros2", "bag", "record", "-o", bag_path]
                cmd += topics if topics else ["-a"]

                proc = subprocess.Popen(cmd)
                _current.update(name=name, path=bag_path, proc=proc, topics=topics or ["(all)"])

            time.sleep(STARTUP_CHECK_S)
            exit_code = proc.poll()
            if exit_code is not None:
                with _lock:
                    _current.update(name=None, path=None, proc=None, topics=None)
                self._send_json(400, {
                    "error": f"ros2 bag record exited immediately with code {exit_code}; "
                             "check `docker logs` for details",
                    "command": cmd,
                })
                return

            self._send_json(200, {"started": name, "path": bag_path, "command": cmd})
            return

        if self.path == "/stop":
            with _lock:
                name = _current["name"]
                was_recording = _stop_current_locked()
            self._send_json(200, {"stopped": was_recording, "name": name if was_recording else None})
            return

        self._send_json(404, {"error": "not found"})

    def log_message(self, fmt, *args):
        print(f"[api] {self.address_string()} " + fmt % args, flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", type=int, default=8081, help="HTTP port to listen on (default: 8081)")
    args = parser.parse_args()

    os.makedirs(BAGS_DIR, exist_ok=True)
    server = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)
    print(f"Bag-collector API listening on :{args.port}, bags dir: {BAGS_DIR}", flush=True)
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

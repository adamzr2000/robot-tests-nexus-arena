#!/usr/bin/env python3
"""HTTP API for the lidar floor tests (port 8091): RPLiDAR A3, static, nothing moves.

  POST /l1  {}   L1 empty:  lidar on the floor, no box
  POST /l2  {}   L2 box:    lidar on the floor, box at `distance` in direction `box_angle`
  POST /l3  {}   L3 raised: same box, same place, lidar raised
  Surface tests (does the floor's reflection affect the sensor?), each a pair: a = bare floor, b = a thin
  cardboard sheet on the floor, nothing else changed:
  POST /l4a /l4b  vertical:   lidar on its side, its box-side beam straight down to the floor (~0.4 m)
  POST /l5a /l5b  tilted:     lidar tilted down, its box-side beam hits the floor ~1 m away
  POST /l6a /l6b  horizontal: lidar flat on the floor, box at 1 m; the sheet covers the floor in between
            optional: "notes": "...", or any PROTOCOL setting to override (e.g. "duration": 60);
            an override other than duration/delay makes it a different condition
  POST /stop     abort the running recording (saved as ABORTED, not counted)
  GET  /peek     live check: scan rate, what is at box_angle, nearest object per 30 deg sector
  GET  /status   running recording, last result, DONE count per experiment

Each run waits `delay` s (step away), records /scan for `duration` s, then writes
results/<session>/<trial id>/: trial.json (setup + metrics), scans.csv (one row per
scan), angles.csv (one row per beam angle) and bag/scan.bag (raw /scan).
Trial IDs: <test>_<name>_d<distance>_r<repeat>_<HHMMSS> (e.g. L2_box_d2.00_r1_140502); only DONE runs count.

Operator notes go in results/<session>/manual.csv (a row per run is added automatically;
the API never overwrites what you type). run_log.csv = trial.json metrics + manual.csv,
summary.csv = mean/sd per experiment, surface_results.csv = only the L4-L6 numbers (distance, noise,
outliers, returns per run + mean per experiment); all rebuilt after every run and on GET /status.
"""
import csv
import glob
import hashlib
import json
import math
import os
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import rosbag
import rospy
from sensor_msgs.msg import LaserScan

HERE = os.path.dirname(os.path.abspath(__file__))
RESULTS = os.environ.get("RESULTS_DIR", os.path.expanduser("~/results"))
PORT = int(os.environ.get("PORT", "8091"))
SCAN_TOPIC = "/scan"
MAX_GAP_S = 0.5  # a longer pause between scans fails the run (driver/USB hiccup)

PROTOCOL = {
    "repetitions": 5,
    "defaults": {
        "duration": 30.0,          # s of /scan recorded per run (~360 scans at 12 Hz)
        "delay": 5.0,              # s between the request and the recording (operator steps away)
        "distance": 2.0,           # m, lidar centre to box face (placed with /peek); L1 uses it too, so all share the zone
        "box_angle": 180.0,        # deg in the /scan frame where the box stands; the driver's 0 deg points
                                   # away from the box side we use (checked with the box, 2026-11-03)
        "zone_half_angle": 10.0,   # zone = box_angle +- this, ranges up to distance + zone_margin
        "zone_margin": 0.5,        # m
        "center_half_angle": 2.0,  # box_angle +- this: the box face centre, used for its range
    },
    "tests": {  # "defaults": this experiment's own values for the settings above
        "L1": {"name": "empty", "setup": "lidar floor, no box", "target": False},
        "L2": {"name": "box", "setup": "lidar floor, box", "target": True},
        "L3": {"name": "raised", "setup": "lidar raised, box", "target": True},
        # Surface tests (asked by the supervisor, 2026-11-04: does the floor's reflection affect the sensor?).
        # Each pair: a = bare floor, b = a thin cardboard sheet laid where the beam meets the floor ("sheet").
        # L4 vertical: lidar on its side (scan plane vertical), box side straight down, ~0.4 m up: the
        #    target is the floor itself, hit head-on
        # L5 tilted: lidar on boxes ~40 cm up, tilted down, box side hits the floor ~1 m away (slant)
        # L6 horizontal: lidar flat on the floor, box at 1 m; the sheet covers the floor under the beam path
        "L4a": {"name": "floor", "setup": "lidar on its side, bare floor", "target": True, "pair": "L4",
                "sheet": False, "defaults": {"distance": 0.4}},
        "L4b": {"name": "card", "setup": "lidar on its side, cardboard sheet", "target": True, "pair": "L4",
                "sheet": True, "defaults": {"distance": 0.4}},
        "L5a": {"name": "floor", "setup": "lidar tilted down, bare floor", "target": True, "pair": "L5",
                "sheet": False, "defaults": {"distance": 1.0}},
        "L5b": {"name": "card", "setup": "lidar tilted down, cardboard sheet", "target": True, "pair": "L5",
                "sheet": True, "defaults": {"distance": 1.0}},
        "L6a": {"name": "box", "setup": "lidar flat on the floor, box at 1 m, bare floor", "target": True,
                "pair": "L6", "sheet": False, "defaults": {"distance": 1.0}},
        "L6b": {"name": "sheet", "setup": "lidar flat on the floor, box at 1 m, cardboard sheet between",
                "target": True, "pair": "L6", "sheet": True, "defaults": {"distance": 1.0}},
    },
    # "target": the box/floor must be seen at box_angle; a run seeing it in fewer scans than this fails
    "min_detection": 0.5,
}
SAME_CONDITION_IGNORES = ("duration", "delay")  # these may change between repeats

# manual.csv: the API owns the first columns, the operator the rest
MANUAL_AUTO = ["trial_id", "test", "setup", "repeat", "status"]
MANUAL_OPERATOR = ["override_status", "observations", "notes"]

RUN_LOG_COLUMNS = [
    "trial_id", "test", "setup", "distance", "box_angle", "repeat", "status", "reason", "started",
    "overrides", "duration_s", "scans", "scan_hz", "max_gap_s", "valid_fraction",
    "zone_points_mean", "zone_min_m", "detection_rate", "range_median_m", "range_std_mm", "beam_std_mm",
    "outlier_fraction", "range_min_m", "range_max_m", "observations", "notes",
]
SUMMARY_METRICS = ["zone_points_mean", "detection_rate", "range_median_m", "range_std_mm", "beam_std_mm",
                   "outlier_fraction", "valid_fraction"]
OUTLIER_M = 0.02  # a centre-beam return this far from that beam's median counts as an outlier

_lock = threading.Lock()
_current = {}   # id, dir, started, stop (Event), thread
_last = {}
_live = {"msg": None, "stamps": []}   # latest scan + recent arrival times, for /peek and checks
_buffer = None  # list of (receive time, LaserScan) while recording, else None


# --- ROS -----------------------------------------------------------------------

def on_scan(msg):
    now = time.time()
    _live["msg"] = msg
    _live["stamps"] = [t for t in _live["stamps"] if now - t < 2.0] + [now]
    if _buffer is not None:
        _buffer.append((now, msg))


def live_hz():
    s = _live["stamps"]
    return (len(s) - 1) / (s[-1] - s[0]) if len(s) > 2 and s[-1] > s[0] else 0.0


def scan_alive():
    return bool(_live["stamps"]) and time.time() - _live["stamps"][-1] < 1.0


def angles_deg(msg):
    return np.degrees(msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment)


def angle_diff(a, b):
    """a - b wrapped to [-180, 180) deg."""
    return (np.asarray(a) - b + 180.0) % 360.0 - 180.0


def valid_mask(msg, r):
    return np.isfinite(r) & (r >= msg.range_min) & (r <= msg.range_max)


# --- results files ---------------------------------------------------------------

def session_dir():
    return os.path.join(RESULTS, os.environ.get("SESSION") or "lidar-benchmark")


def code_version():
    return hashlib.sha256(open(os.path.abspath(__file__), "rb").read()).hexdigest()[:12]


def write_json(path, data):
    with open(path + ".tmp", "w") as f:
        json.dump(data, f, indent=1, default=str)
    os.replace(path + ".tmp", path)


def load_trials():
    out = []
    for path in sorted(glob.glob(os.path.join(session_dir(), "*", "trial.json"))):
        try:
            out.append(json.load(open(path)))
        except (OSError, ValueError):
            pass
    return out


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


def effective_status(t, manual_row):
    return ((manual_row or {}).get("override_status") or "").strip().upper() or t.get("status")


def setup_label(t):
    return PROTOCOL["tests"][t["test"]]["setup"]


def test_defaults(test):
    return {**PROTOCOL["defaults"], **PROTOCOL["tests"][test].get("defaults", {})}


def update_manual(trials):
    """Add a row for every recorded run, refresh the API's columns, keep the operator's.
    Rows with operator values but no trial.json (e.g. a renamed folder) are kept at the end."""
    rows, delim = read_manual()
    ids = {t["id"] for t in trials}
    orphans = [r for i, r in rows.items() if i not in ids and any((r.get(k) or "").strip() for k in MANUAL_OPERATOR)]
    path = os.path.join(session_dir(), "manual.csv")
    with open(path + ".tmp", "w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=MANUAL_AUTO + MANUAL_OPERATOR, delimiter=delim, extrasaction="ignore",
                           lineterminator="\n")
        w.writeheader()
        for t in trials:
            row = rows.get(t["id"], {})
            row.update(trial_id=t["id"], test=t["test"], setup=setup_label(t), repeat=t.get("repeat"),
                       status=t.get("status") + (f" ({t['reason']})" if t.get("reason") else ""))
            w.writerow(row)
        for row in orphans:
            print(f"[api] manual.csv: {row['trial_id']} has no trial.json; row kept, not used", flush=True)
            w.writerow(row)
    os.replace(path + ".tmp", path)


def run_log_row(t, man):
    m, p = t.get("metrics", {}), t["params"]
    row = {k: m.get(k) for k in RUN_LOG_COLUMNS if k in m}
    row.update(trial_id=t["id"], test=t["test"], setup=setup_label(t), distance=p["distance"],
               box_angle=p["box_angle"], repeat=t.get("repeat"), status=effective_status(t, man),
               reason=t.get("reason"), started=t.get("started"), overrides=json.dumps(t.get("overrides", {})),
               observations=man.get("observations"),
               notes="; ".join(x for x in (t.get("notes"), man.get("notes")) if x))
    return {k: (round(v, 4) + 0.0 if isinstance(v, float) else v) for k, v in row.items()}  # + 0.0: no "-0.0"


def refresh_logs():
    """Rebuild manual.csv rows and run_log.csv from the trial.json files. Caller holds _lock."""
    trials = load_trials()
    if not trials:
        return
    update_manual(trials)
    manual_rows, _ = read_manual()
    rows = [run_log_row(t, manual_rows.get(t["id"], {})) for t in trials]
    path = os.path.join(session_dir(), "run_log.csv")
    with open(path + ".tmp", "w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=RUN_LOG_COLUMNS, lineterminator="\n")
        w.writeheader()
        w.writerows(rows)
    os.replace(path + ".tmp", path)

    # summary.csv: one row per experiment, mean and sd over its DONE runs with the protocol settings
    path = os.path.join(session_dir(), "summary.csv")
    with open(path + ".tmp", "w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=["test", "setup", "runs"] + [c for k in SUMMARY_METRICS for c in (k, k + "_sd")],
                           lineterminator="\n")
        w.writeheader()
        for test in PROTOCOL["tests"]:
            default = condition(test, test_defaults(test))
            sel = [r for t, r in zip(trials, rows)
                   if t["test"] == test and r["status"] == "DONE" and condition(test, t["params"]) == default]
            out = {"test": test, "setup": setup_label({"test": test}), "runs": len(sel)}
            for k in SUMMARY_METRICS:
                v = [r[k] for r in sel if r.get(k) is not None]
                out[k] = round(float(np.mean(v)), 4) + 0.0 if v else None
                out[k + "_sd"] = round(float(np.std(v, ddof=1)), 4) + 0.0 if len(v) > 1 else None
            w.writerow(out)
    os.replace(path + ".tmp", path)
    write_surface_results(trials, rows)


SURFACE_COLUMNS = ["trial_id", "test", "setup", "sheet", "repeat", "status", "distance_mm", "noise_mm", "outliers_pct",
                   "returns_pct", "notes"]


def write_surface_results(trials, rows):
    """surface_results.csv: the four numbers of the surface tests (L4-L6) per run, then the mean per
    experiment over the DONE runs with the protocol settings. Caller holds _lock."""
    surface = [t for t, spec in PROTOCOL["tests"].items() if "pair" in spec]
    l4 = [(t, r) for t, r in zip(trials, rows) if t["test"] in surface]
    if not l4:
        return

    def numbers(r):
        mm = lambda v, k=1000: None if v in (None, "") else round(float(v) * k, 2)
        return {"distance_mm": mm(r.get("range_median_m")), "noise_mm": mm(r.get("beam_std_mm"), 1),
                "outliers_pct": mm(r.get("outlier_fraction"), 100), "returns_pct": mm(r.get("detection_rate"), 100)}

    label = lambda test: {"test": test, "setup": PROTOCOL["tests"][test]["setup"],
                          "sheet": "yes" if PROTOCOL["tests"][test]["sheet"] else "no"}
    path = os.path.join(session_dir(), "surface_results.csv")
    with open(path + ".tmp", "w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=SURFACE_COLUMNS, lineterminator="\n")
        w.writeheader()
        for t, r in sorted(l4, key=lambda tr: tr[0]["test"]):
            w.writerow({"trial_id": t["id"], **label(t["test"]), "repeat": t.get("repeat"),
                        "status": r["status"], "notes": r.get("notes"), **numbers(r)})
        for test in surface:
            default = condition(test, test_defaults(test))
            sel = [numbers(r) for t, r in l4 if t["test"] == test and r["status"] == "DONE"
                   and condition(test, t["params"]) == default]
            if sel:
                mean = {k: round(float(np.mean([x[k] for x in sel if x[k] is not None])), 2)
                        if any(x[k] is not None for x in sel) else None for k in sel[0]}
                w.writerow({"trial_id": f"MEAN {test} ({len(sel)} runs)", **label(test), **mean})
    os.replace(path + ".tmp", path)


def condition(test, params):
    return (test,) + tuple(sorted((k, v) for k, v in params.items() if k not in SAME_CONDITION_IGNORES))


def done_counts():
    trials, (manual_rows, _) = load_trials(), read_manual()
    out = {}
    for test, spec in PROTOCOL["tests"].items():
        default = condition(test, test_defaults(test))
        done = [t for t in trials if t["test"] == test and effective_status(t, manual_rows.get(t["id"])) == "DONE"]
        label = f"{test} {spec['name']} ({spec['setup']})"
        out[label] = f"{sum(condition(test, t['params']) == default for t in done)}/{PROTOCOL['repetitions']}"
        other = sum(condition(test, t["params"]) != default for t in done)
        if other:
            out[label] += f" (+{other} with overridden settings)"
    return out


# --- metrics -------------------------------------------------------------------

def analyse(scans, p):
    """scans: [(receive time, LaserScan)]. Returns (metrics, per-scan rows, per-angle rows)."""
    times = np.array([t for t, _ in scans])
    gaps = np.diff(times) if len(times) > 1 else np.array([0.0])
    zone_max = p["distance"] + p["zone_margin"]
    per_scan, center = [], []
    for t, msg in scans:
        r = np.asarray(msg.ranges, dtype=float)
        a = angle_diff(angles_deg(msg), p["box_angle"])
        ok = valid_mask(msg, r)
        zone = ok & (np.abs(a) <= p["zone_half_angle"]) & (r <= zone_max)
        mid = zone & (np.abs(a) <= p["center_half_angle"])
        c = float(np.median(r[mid])) if mid.any() else None
        center.append(c)
        per_scan.append({"t_s": round(t - times[0], 4), "stamp": msg.header.stamp.to_sec(), "points": len(r),
                         "valid": int(ok.sum()), "zone_points": int(zone.sum()),
                         "zone_min_m": round(float(r[zone].min()), 4) if zone.any() else None,
                         "center_points": int(mid.sum()), "center_range_m": None if c is None else round(c, 4)})
    hits = np.array([c for c in center if c is not None])
    zone_mins = [s["zone_min_m"] for s in per_scan if s["zone_min_m"] is not None]
    metrics = {
        "duration_s": round(float(times[-1] - times[0]), 3),
        "scans": len(scans),
        "scan_hz": round((len(scans) - 1) / (times[-1] - times[0]), 3) if len(scans) > 1 else None,
        "max_gap_s": round(float(gaps.max()), 3),
        "valid_fraction": round(float(np.mean([s["valid"] / s["points"] for s in per_scan])), 4),
        "zone_points_mean": round(float(np.mean([s["zone_points"] for s in per_scan])), 2),
        "zone_min_m": min(zone_mins) if zone_mins else None,
        "detection_rate": round(len(hits) / len(scans), 4),
        "range_median_m": round(float(np.median(hits)), 4) if len(hits) else None,
        "range_std_mm": round(float(np.std(hits) * 1000), 2) if len(hits) > 1 else None,
        "range_min_m": round(float(hits.min()), 4) if len(hits) else None,
        "range_max_m": round(float(hits.max()), 4) if len(hits) else None,
    }

    # per beam angle over the whole run; with angle_compensate the A3 gives the same
    # 1440 angles every scan, otherwise fall back to 1-degree bins
    first = scans[0][1]
    same_grid = all(len(m.ranges) == len(first.ranges) and m.angle_min == first.angle_min
                    and m.angle_increment == first.angle_increment for _, m in scans)
    R = np.array([np.where(valid_mask(m, np.asarray(m.ranges, dtype=float)), m.ranges, np.nan) for _, m in scans]
                 if same_grid else [])
    if same_grid:
        bins, grid = angles_deg(first), R
    else:
        bins = np.arange(-180.0, 180.0)
        grid = np.full((len(scans), len(bins)), np.nan)
        for i, (_, m) in enumerate(scans):
            r = np.asarray(m.ranges, dtype=float)
            ok = valid_mask(m, r)
            idx = (np.round(angle_diff(angles_deg(m), 0.0)).astype(int) + 180) % 360
            for j in np.unique(idx[ok]):  # median of the beams falling in that degree
                grid[i, j] = np.median(r[ok & (idx == j)])
    # noise of single beams: each centre beam's scan-to-scan std (returns in the zone, beams with a
    # return in at least half the scans), averaged over those beams; and the share of those returns
    # more than OUTLIER_M from their beam's median (occasional wrong readings, e.g. from reflections)
    stds, n_out, n_all = [], 0, 0
    for j in np.flatnonzero(np.abs(angle_diff(bins, p["box_angle"])) <= p["center_half_angle"]):
        col = grid[:, j]
        v = col[np.isfinite(col) & (col <= zone_max)]
        if len(v) >= max(2, len(col) / 2):
            stds.append(np.std(v))
            n_out += int(np.sum(np.abs(v - np.median(v)) > OUTLIER_M))
            n_all += len(v)
    metrics["beam_std_mm"] = round(float(np.mean(stds)) * 1000, 2) if stds else None
    metrics["outlier_fraction"] = round(n_out / n_all, 5) if n_all else None

    per_angle = []
    with np.errstate(all="ignore"):
        for j, ang in enumerate(bins):
            col = grid[:, j]
            v = col[np.isfinite(col)]
            per_angle.append({"angle_deg": round(float(ang), 3), "valid_fraction": round(len(v) / len(col), 4),
                              **{k: (round(float(f(v)), 4) if len(v) else None) for k, f in
                                 (("mean_m", np.mean), ("median_m", np.median), ("std_m", np.std),
                                  ("min_m", np.min), ("max_m", np.max))}})
    return metrics, per_scan, per_angle


def write_csv(path, rows):
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()), lineterminator="\n")
        w.writeheader()
        w.writerows(rows)


# --- runs ----------------------------------------------------------------------

def start_run(test, body):
    body = dict(body)
    notes = str(body.pop("notes", ""))
    params = test_defaults(test)
    for k, v in body.items():
        if k not in params:
            return 400, {"error": f"unknown setting '{k}'; settings: {sorted(params)} and notes"}
        try:
            params[k] = float(v)
        except (TypeError, ValueError):
            return 400, {"error": f"'{k}' must be a number"}
    if params["duration"] <= 0 or params["delay"] < 0 or params["distance"] <= 0:
        return 400, {"error": "duration and distance must be > 0, delay >= 0"}
    if not scan_alive():
        return 503, {"error": f"no {SCAN_TOPIC} in the last second; is the rplidar-lidar container running? "
                              "(docker logs rplidar-lidar)"}
    with _lock:
        if _current:
            return 409, {"error": f"run {_current['id']} is in progress; POST /stop first"}
        manual_rows, _ = read_manual()
        cond = condition(test, params)
        # lowest repeat number without a DONE run: a retry fills the gap left by an
        # ABORTED/FAILED run or one marked INVALID in manual.csv
        valid = {t.get("repeat") for t in load_trials() if t["test"] == test and condition(test, t["params"]) == cond
                 and effective_status(t, manual_rows.get(t["id"])) == "DONE"}
        repeat = next(n for n in range(1, len(valid) + 2) if n not in valid)
        spec = PROTOCOL["tests"][test]
        tid = f"{test}_{spec['name']}_d{params['distance']:.2f}_r{repeat}_{time.strftime('%H%M%S')}"
        tdir = os.path.join(session_dir(), tid)
        os.makedirs(tdir)
        stop = threading.Event()
        thread = threading.Thread(target=_run, args=(test, tid, tdir, repeat, params, notes, body, stop), daemon=True)
        _current.update(id=tid, dir=tdir, started=time.time(), stop=stop, thread=thread,
                        phase="delay", record_s=params["duration"])
        thread.start()
    return 200, {"started": tid, "setup": spec["setup"],
                 "params": params, "recording_in_s": params["delay"],
                 "follow": "docker logs -f rplidar-tests"}


def _run(test, tid, tdir, repeat, params, notes, overrides, stop):
    global _buffer
    t = {"id": tid, "test": test, "setup": PROTOCOL["tests"][test], "repeat": repeat, "status": "RUNNING",
         "reason": None, "started": time.strftime("%Y-%m-%dT%H:%M:%S%z"), "params": params,
         "overrides": overrides, "notes": notes, "code_version": code_version()}
    print(f"[api] {tid}: recording starts in {params['delay']:g} s - step away and stay still", flush=True)
    status, reason, scans = "DONE", None, []
    if stop.wait(params["delay"]):
        status, reason = "ABORTED", "stopped before recording"
    else:
        _current["phase"] = "recording"
        _current["record_start"] = time.time()
        print(f"[api] {tid}: recording {params['duration']:g} s", flush=True)
        _buffer = []
        if stop.wait(params["duration"]):
            status, reason = "ABORTED", "POST /stop"
        scans, _buffer = _buffer, None
    if len(scans) < 2:
        status, reason = ("FAILED", f"only {len(scans)} scans received") if status == "DONE" else (status, reason)
    else:
        try:
            rospy_driver = rospy.get_param("/rplidarNode", {})
        except Exception as exc:  # master gone: keep the data anyway
            rospy_driver = {"error": str(exc)}
        first = scans[0][1]
        t["lidar"] = {"driver_params": rospy_driver, "frame_id": first.header.frame_id,
                      "points_per_scan": len(first.ranges), "angle_min_deg": round(math.degrees(first.angle_min), 4),
                      "angle_increment_deg": round(math.degrees(first.angle_increment), 5),
                      "range_min_m": first.range_min, "range_max_m": first.range_max}
        os.makedirs(os.path.join(tdir, "bag"))
        with rosbag.Bag(os.path.join(tdir, "bag", "scan.bag"), "w") as bag:
            for _, msg in scans:
                bag.write(SCAN_TOPIC, msg, msg.header.stamp)
        metrics, per_scan, per_angle = analyse(scans, params)
        write_csv(os.path.join(tdir, "scans.csv"), per_scan)
        write_csv(os.path.join(tdir, "angles.csv"), per_angle)
        t["metrics"] = metrics
        if status == "DONE" and metrics["max_gap_s"] > MAX_GAP_S:
            status, reason = "FAILED", f"{metrics['max_gap_s']} s without scans"
        elif (status == "DONE" and PROTOCOL["tests"][test]["target"]
              and metrics["detection_rate"] < PROTOCOL["min_detection"]):
            status, reason = "FAILED", (f"target not seen at {params['box_angle']:g} deg within "
                                        f"{params['distance'] + params['zone_margin']:g} m "
                                        f"(detection {metrics['detection_rate']}); check /peek")
    t.update(status=status, reason=reason)
    write_json(os.path.join(tdir, "trial.json"), t)
    m = t.get("metrics", {})
    print(f"[api] {tid} finished: {status} {reason or ''} | scans {m.get('scans')} @ {m.get('scan_hz')} Hz, "
          f"zone points/scan {m.get('zone_points_mean')}, detection {m.get('detection_rate')}, "
          f"range {m.get('range_median_m')} m (std {m.get('range_std_mm')} mm)", flush=True)
    with _lock:
        _last.clear()
        _last.update(id=tid, status=status, reason=reason, metrics=m)
        _current.clear()
        refresh_logs()


def stop_run():
    with _lock:
        stop, thread = _current.get("stop"), _current.get("thread")
    if not stop:
        return False
    stop.set()
    thread.join(timeout=30)
    return True


def peek():
    msg = _live["msg"]
    if msg is None or not scan_alive():
        return 503, {"error": f"no {SCAN_TOPIC} in the last second"}
    p = PROTOCOL["defaults"]
    r = np.asarray(msg.ranges, dtype=float)
    a = angle_diff(angles_deg(msg), 0.0)
    ok = valid_mask(msg, r)
    mid = ok & (np.abs(angle_diff(a, p["box_angle"])) <= p["center_half_angle"])
    sectors = {}
    for c in range(-180, 180, 30):  # nearest return in each 30-degree sector, centred on c
        s = ok & (np.abs(angle_diff(a, c)) <= 15)
        if s.any():
            i = np.flatnonzero(s)[np.argmin(r[s])]
            sectors[f"{c:+d}"] = f"{r[i]:.3f} m at {a[i]:+.1f} deg"
        else:
            sectors[f"{c:+d}"] = "nothing"
    out = {"scan_hz": round(live_hz(), 2), "points": len(r), "valid": int(ok.sum()),
           f"at_box_angle_{p['box_angle']:g}deg": (f"{np.median(r[mid]):.3f} m ({int(mid.sum())} points)"
                                                    if mid.any() else "nothing")}
    if mid.any():
        out["box_face"] = box_face(r, angle_diff(a, p["box_angle"]), ok, float(np.median(r[mid])))
    out["nearest_per_sector"] = sectors
    return 200, out


def box_face(r, off, ok, r0):
    """Setup aid: the surface seen at box_angle, followed beam by beam to each side while the
    range stays within 15 cm of the previous beam (up to 4 beams without a return are skipped)."""
    order = np.argsort(off)
    off, r, ok = off[order], r[order], ok[order]
    start = int(np.argmin(np.abs(off)))
    ends = []
    for step in (-1, 1):
        i, last, edge, misses = start, r0, start, 0
        while 0 <= i + step < len(r) and abs(off[i + step]) < 60:
            i += step
            if not ok[i]:
                misses += 1
                if misses > 4:
                    break
                continue
            if abs(r[i] - last) > 0.15:
                break
            last, edge, misses = r[i], i, 0
        ends.append(float(off[edge]))
    centre = (ends[0] + ends[1]) / 2
    return {"from_deg": round(ends[0], 1), "to_deg": round(ends[1], 1),
            "width_m": round(2 * r0 * math.tan(math.radians((ends[1] - ends[0]) / 2)), 2),
            "centre_offset_deg": round(centre, 1),
            "check": "centred" if abs(centre) <= 2 else
                     f"slide the box ~{abs(r0 * math.tan(math.radians(centre))) * 100:.0f} cm sideways "
                     "until centre_offset_deg is within +-2 (if it grows, the other way)"}


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
        if self.path == "/peek":
            self._send(*peek())
            return
        if self.path != "/status":
            self._send(404, {"error": "not found; endpoints: POST /l1 /l2 /l3 /l4a /l4b /l5a /l5b /l6a /l6b /stop, GET /peek /status"})
            return
        with _lock:
            cur = None
            if _current:
                cur = {"id": _current["id"], "phase": _current["phase"],
                       "elapsed_s": round(time.time() - _current["started"], 1)}
            refresh_logs()  # picks up manual.csv edits
            counts = done_counts()
        self._send(200, {"session": session_dir(), "lidar_alive": scan_alive(), "scan_hz": round(live_hz(), 2),
                         "running": cur, "last": _last or None, "done": counts})

    def do_POST(self):
        try:
            body = self._body()
        except ValueError as exc:
            self._send(400, {"error": f"bad JSON body: {exc}"})
            return
        test = {"/" + t.lower(): t for t in PROTOCOL["tests"]}.get(self.path)
        if test:
            self._send(*start_run(test, body))
        elif self.path == "/stop":
            self._send(200, {"stopped": stop_run()})
        else:
            self._send(404, {"error": "not found; endpoints: POST /l1 /l2 /l3 /l4a /l4b /l5a /l5b /l6a /l6b /stop, GET /peek /status"})

    def log_message(self, fmt, *args):
        print(f"[api] {self.address_string()} " + fmt % args, flush=True)


def main():
    rospy.init_node("lidar_tests", anonymous=False, disable_signals=True)  # waits for the master
    rospy.Subscriber(SCAN_TOPIC, LaserScan, on_scan, queue_size=50)
    os.makedirs(session_dir(), exist_ok=True)
    server = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    signal.signal(signal.SIGTERM, lambda *_: threading.Thread(target=server.shutdown).start())
    print(f"Lidar tests API on :{PORT}, results in {session_dir()}, code {code_version()}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop_run()
        server.server_close()
        rospy.signal_shutdown("exit")


if __name__ == "__main__":
    main()

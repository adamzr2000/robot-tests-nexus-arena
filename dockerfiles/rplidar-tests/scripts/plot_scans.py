#!/usr/bin/env python3
"""Figures for the lidar floor tests, from the recorded bags (run in the rplidar-viz image):

  setup_sketch.png    side view of the three setups (L1, L2, L3)
  scans_top_view.png  what the lidar saw: every return of the 5 runs, seen from above
  box_edges.png       share of scans with a return on the box, beam by beam (L2 vs L3)
  surface_sketch.png  side view of the surface tests L4 (vertical) and L5 (tilted)
  setup_sketch.svg, surface_sketch.svg  the two sketches as editable vector files (Inkscape, PowerPoint)
  surface_summary.png L4/L5 from the measured readings: how the readings spread (floor vs cardboard),
                      one beam over 30 s, and the spread of every run
  surface_analysis.csv  the numbers behind surface_summary.png, per run and all runs together
  l4_surface.png      per surface test (L4/L5/L6, once its a and b runs exist): distance, noise and
  l5_surface.png      returns of each beam, bare floor (a) vs cardboard sheet (b)
  l6_surface.png

Only DONE runs are used (not ABORTED/FAILED, nor marked INVALID in manual.csv) with the
same settings as the campaign.

Beam angles: with angle_compensate the rplidar_ros driver fills 1440 slots of 0.25 deg (360 deg)
but labels them as if they spanned 359 deg (angle_max = 359 deg in node.cpp), so its published
angles drift by up to 0.85 deg and leave a fake 1 deg gap at 180 deg, where the box stands.
The figures place each slot at its real 0.25 deg position.

Usage: plot_scans.py <session dir> [<output dir>, default <session dir>/figures]
"""
import csv
import glob
import json
import math
import os
import sys
import warnings
from collections import Counter

import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams["svg.fonttype"] = "none"  # SVG text stays text, editable
import matplotlib.pyplot as plt
import numpy as np
import rosbag
from matplotlib.patches import Rectangle, Wedge

BOX_WIDTH_M = 0.98  # the operator's box (measured by hand, 2026-11-03); only used to draw its edges
NAMES = {"L1": "L1: lidar on the floor, no box",
         "L2": "L2: lidar on the floor, box at 2 m",
         "L3": "L3: lidar raised 25 cm, box at 2 m",
         "L4a": "L4a: lidar on its side, bare floor",
         "L4b": "L4b: lidar on its side, cardboard sheet",
         "L5a": "L5a: lidar tilted down, bare floor",
         "L5b": "L5b: lidar tilted down, cardboard sheet",
         "L6a": "L6a: lidar flat on the floor, box at 1 m, bare floor",
         "L6b": "L6b: lidar flat on the floor, box at 1 m, sheet between"}
COLOURS = {"L1": "#7a7a7a", "L2": "#d9480f", "L3": "#1c7ed6",
           **{f"L{n}a": "#2b8a3e" for n in (4, 5, 6)}, **{f"L{n}b": "#ae3ec9" for n in (4, 5, 6)}}
PAIRS = {  # surface test: (what the target is, plot range in deg from the centre beam, x-axis label)
    "L4": ("floor", 60, "angle from the straight-down beam = angle at which the beam hits the floor (deg)"),
    "L5": ("floor", 60, "angle along the floor line from the centre beam (deg)"),
    "L6": ("box", 30, "angle from the box centre (deg)"),
}


def off_deg(angle, box_angle):
    return (np.asarray(angle) - box_angle + 180.0) % 360.0 - 180.0


def beam_angles(m):
    """Real angle (deg, scan frame) of each published slot; see the module docstring."""
    n = len(m.ranges)
    published = np.degrees(m.angle_min + np.arange(n) * m.angle_increment)
    if n == 1440 and math.isclose(math.degrees(m.angle_increment), 359.0 / 1439, rel_tol=1e-4):
        return 180.0 - (n - 1 - np.arange(n)) * 360.0 / n  # slot j = device angle j * 0.25, stored reversed
    return published


def select_runs(session):
    """{test: [run dirs]} of the runs to use, plus {test: its campaign settings} (each experiment's
    most common settings among its DONE runs)."""
    manual = {}
    path = os.path.join(session, "manual.csv")
    if os.path.exists(path):
        manual = {r["trial_id"]: r for r in csv.DictReader(open(path, encoding="utf-8-sig"))}
    trials = [json.load(open(p)) for p in sorted(glob.glob(os.path.join(session, "*", "trial.json")))]
    done = [t for t in trials if ((manual.get(t["id"], {}).get("override_status") or "").strip().upper()
                                  or t.get("status")) == "DONE"]
    key = lambda t: json.dumps({k: v for k, v in t["params"].items() if k not in ("duration", "delay")}, sort_keys=True)
    campaign = {test: Counter(key(t) for t in done if t["test"] == test).most_common(1)[0][0]
                for test in {t["test"] for t in done}}
    used = {test: [] for test in NAMES}
    for t in trials:
        if t in done and t["test"] in used and key(t) == campaign[t["test"]]:
            used[t["test"]].append(os.path.join(session, t["id"]))
        else:
            print(f"skipped {t['id']} ({t.get('status')}, or other settings)")
    return used, {test: json.loads(k) for test, k in campaign.items()}


def load_points(run_dir, box_angle):
    """x, y of every valid return of every scan, rotated so the box direction points up."""
    xs, ys = [], []
    with rosbag.Bag(os.path.join(run_dir, "bag", "scan.bag")) as bag:
        for _, m, _ in bag.read_messages():
            r = np.asarray(m.ranges, dtype=float)
            ok = np.isfinite(r) & (r >= m.range_min) & (r <= m.range_max)
            phi = np.radians(off_deg(beam_angles(m)[ok], box_angle) + 90.0)  # box direction -> +y
            xs.append(r[ok] * np.cos(phi))
            ys.append(r[ok] * np.sin(phi))
    return np.concatenate(xs), np.concatenate(ys)


def box_hit_rate(run_dir, box_angle, max_range):
    """Per slot: (angle from the box centre, share of scans with a return closer than max_range)."""
    hits, n = None, 0
    with rosbag.Bag(os.path.join(run_dir, "bag", "scan.bag")) as bag:
        for _, m, _ in bag.read_messages():
            r = np.asarray(m.ranges, dtype=float)
            h = np.isfinite(r) & (r >= m.range_min) & (r < max_range)
            hits = h.astype(float) if hits is None else hits + h
            n += 1
    return off_deg(beam_angles(m), box_angle), hits / n


def beam_stats(run_dir, box_angle, max_range):
    """Per slot: (angle from the box centre, mean range, scan-to-scan std, share of scans with a
    return closer than max_range); range statistics only over those returns."""
    rows = []
    with rosbag.Bag(os.path.join(run_dir, "bag", "scan.bag")) as bag:
        for _, m, _ in bag.read_messages():
            rows.append(np.asarray(m.ranges, dtype=float))
    R = np.array(rows)
    R[~(np.isfinite(R) & (R >= m.range_min) & (R < max_range))] = np.nan
    with warnings.catch_warnings():  # angles that never got a return: nan, on purpose
        warnings.simplefilter("ignore", RuntimeWarning)
        return (off_deg(beam_angles(m), box_angle), np.nanmean(R, axis=0), np.nanstd(R, axis=0),
                np.mean(np.isfinite(R), axis=0))


def pair_surface(pair, used, campaign, out):
    a, b = pair + "a", pair + "b"
    target, half, xlabel = PAIRS[pair]
    p = campaign[a]
    # floor targets: the whole floor line (beams farther out meet it farther away and more slanted)
    max_range = 3.0 if target == "floor" else p["distance"] + p["zone_margin"]
    fig, axes = plt.subplots(3, 1, figsize=(9, 8.5), sharex=True)
    for test in (a, b):
        stats = [beam_stats(d, p["box_angle"], max_range) for d in used[test]]
        o = stats[0][0]
        sel = np.abs(o) <= half
        order = np.argsort(o[sel])
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)
            mean_r, std_r, rate = (np.nanmean([s[k] for s in stats], axis=0)[sel][order] for k in (1, 2, 3))
        x = o[sel][order]
        axes[0].plot(x, mean_r, color=COLOURS[test], lw=1.4, label=NAMES[test])
        axes[1].plot(x, 1000 * std_r, color=COLOURS[test], lw=1.4, label=NAMES[test])
        axes[2].plot(x, 100 * rate, color=COLOURS[test], lw=1.4, label=NAMES[test])
    axes[0].set_ylabel(f"distance to the\n{target} (m)")
    axes[1].set_ylabel("noise: scan-to-scan\nstd per beam (mm)")
    axes[2].set_ylabel("scans with\na return (%)")
    axes[2].set_ylim(-3, 105)
    axes[2].set_xlabel(xlabel)
    for ax in axes:
        ax.axvspan(-p["center_half_angle"], p["center_half_angle"], color="#ffd43b", alpha=0.25, lw=0)
        ax.grid(alpha=0.3)
    axes[0].legend(fontsize=9)
    axes[0].set_title(f"{pair}: bare floor vs cardboard sheet (mean of {len(used[a])} / {len(used[b])} runs; "
                      "yellow = centre beams used in the numbers)", fontsize=10)
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)


def sketch(out):
    fig, axes = plt.subplots(3, 1, figsize=(9, 7.5), sharex=True)
    for ax, test in zip(axes, ("L1", "L2", "L3")):
        base = 0.25 if test == "L3" else 0.0
        plane = base + 0.03
        ax.axhspan(-0.08, 0, color="#c8b89a")                               # ground
        ax.text(2.9, -0.05, "arena surface", ha="right", va="center", fontsize=8)
        if test == "L3":
            ax.add_patch(Rectangle((-0.12, 0), 0.24, base, color="#adb5bd"))  # stand
            ax.text(0.15, base / 2, "stand 25 cm", va="center", fontsize=8)
        ax.add_patch(Rectangle((-0.038, base), 0.076, 0.041, color="black"))  # lidar
        ax.plot([-1.6, 3.0], [plane, plane], ls="--", color="red", lw=1)
        ax.text(0.25, plane + 0.03, f"laser plane, ~{plane * 100:.0f} cm above the ground", color="red", fontsize=8)
        if test != "L1":
            ax.add_patch(Rectangle((2.0, 0), 0.35, 0.45, facecolor="#e9c46a", edgecolor="#8d6e2f"))
            ax.text(2.175, 0.47, "box", ha="center", fontsize=8)
        else:
            ax.add_patch(Rectangle((2.0, 0), 0.35, 0.45, fill=False, ls=":", edgecolor="#8d6e2f"))
            ax.text(2.175, 0.47, "no box (zone empty)", ha="center", fontsize=8)
        ax.annotate("", xy=(2.0, -0.13), xytext=(0, -0.13), arrowprops=dict(arrowstyle="<->", lw=0.8))
        ax.text(1.0, -0.2, "2.0 m", ha="center", fontsize=8)
        ax.add_patch(Rectangle((-1.45, 0), 0.25, 0.03, color="#495057"))       # laptop
        ax.text(-1.33, 0.13, "you + laptop", ha="center", fontsize=8)
        ax.set_title(NAMES[test], loc="left", fontsize=10, color=COLOURS[test])
        ax.set_xlim(-1.7, 3.0)
        ax.set_ylim(-0.25, 0.6)
        ax.set_yticks([0, 0.25, 0.5])
        ax.set_ylabel("height (m)")
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
    axes[-1].set_xlabel("distance from the lidar (m)   (side view, not to scale; box size illustrative)")
    fig.suptitle("Lidar floor tests: only one thing changes from one experiment to the next", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(out, dpi=130)
    fig.savefig(os.path.splitext(out)[0] + ".svg")
    plt.close(fig)


def centre_beams(run_dir, p):
    """The run's centre beams exactly as the API defines them (published angles within
    center_half_angle of box_angle, returns closer than distance + zone_margin): (times, [(t, readings, return share)])."""
    T, R, first = [], [], None
    with rosbag.Bag(os.path.join(run_dir, "bag", "scan.bag")) as bag:
        for _, m, t in bag.read_messages():
            first = first or m
            T.append(t.to_sec())
            R.append(np.asarray(m.ranges, dtype=float))
    T, R = np.array(T) - T[0], np.array(R)
    pub = np.degrees(first.angle_min + np.arange(R.shape[1]) * first.angle_increment)
    with np.errstate(invalid="ignore"):
        ok = np.isfinite(R) & (R >= first.range_min) & (R <= p["distance"] + p["zone_margin"])
    cols = np.flatnonzero(np.abs(off_deg(pub, p["box_angle"])) <= p["center_half_angle"])
    return T, [(T[ok[:, j]], R[ok[:, j], j], ok[:, j].mean()) for j in cols]


def numbers(used, returns):
    """Numbers from the returns of the used beams (one array per beam) and the return share of every
    centre beam. Deviation = a return minus its own beam's median. noise = each beam's std, averaged over
    the beams; spread_99 = 99% of the deviations are within +-this; off_1cm = share of deviations over
    1 cm, off_1cm_far = share of those that are positive (too far)."""
    dev = np.concatenate([v - np.median(v) for v in used])
    off = np.abs(dev) > 0.01
    return {"readings": len(dev),
            "distance_mm": round(float(np.median([np.median(v) for v in used])) * 1000, 1),
            "noise_mm": round(float(np.mean([np.std(v) for v in used])) * 1000, 2),
            "spread_99_mm": round(float(np.percentile(np.abs(dev), 99)) * 1000, 1),
            "min_dev_mm": round(float(dev.min()) * 1000, 1),
            "max_dev_mm": round(float(dev.max()) * 1000, 1),
            "off_1cm_pct": round(float(off.mean()) * 100, 2),
            "off_1cm_far_pct": round(float((dev[off] > 0).mean()) * 100, 1) if off.any() else "",
            "beam_returns_pct": round(float(np.mean(returns)) * 100, 1)}


def run_numbers(beams, n_scans):
    """One run: its used beams are the centre beams with a return in at least half the scans (as the API).
    Returns the numbers, the per-beam deviations and (used, returns) to pool runs with numbers()."""
    used = [v for _, v, _ in beams if len(v) >= max(2, n_scans / 2)]
    returns = [r for _, _, r in beams]
    return numbers(used, returns), [v - np.median(v) for v in used], (used, returns)


def surface_summary(used, campaign, out_png, out_csv):
    pairs = [q for q in ("L4", "L5") if used[q + "a"] and used[q + "b"]]
    fig, axes = plt.subplots(len(pairs), 3, figsize=(15, 4.4 * len(pairs)), squeeze=False)
    rows = []
    for row, pair in zip(axes, pairs):
        per_run = {}
        for test in (pair + "a", pair + "b"):
            p = campaign[test]
            label = "bare floor" if test.endswith("a") else "cardboard"
            alldev, per_run[test], pool_used, pool_returns = [], [], [], []
            for i, d in enumerate(used[test]):
                T, beams = centre_beams(d, p)
                nums, devs, (u, r) = run_numbers(beams, len(T))
                rows.append({"test": test, "trial_id": os.path.basename(d), "surface": label, **nums})
                per_run[test].append(nums)
                alldev += list(np.concatenate(devs))
                pool_used += u
                pool_returns += r
                if i == 0:  # one typical beam over the run: the centre beam whose noise is the median one
                    full = [(t, v) for t, v, _ in beams if len(v) >= max(2, len(T) / 2)]
                    t, v = sorted(full, key=lambda b: np.std(b[1]))[len(full) // 2]
                    dv = 1000 * (v - np.median(v))
                    row[1].plot(t, dv, ".", ms=2, color=COLOURS[test],
                                label=f"{label} ({dv.min():+.0f} to {dv.max():+.0f} mm)")
            rows.append({"test": test, "trial_id": f"ALL ({len(used[test])} runs)", "surface": label,
                         **numbers(pool_used, pool_returns)})
            row[0].hist(1000 * np.array(alldev), bins=np.arange(-15.5, 66, 1), color=COLOURS[test], alpha=0.55,
                        label=f"{label} ({len(alldev)} readings)")
            x = np.arange(1, len(per_run[test]) + 1) + (0.1 if test.endswith("b") else -0.1)
            row[2].plot(x, [n["spread_99_mm"] for n in per_run[test]], "o", color=COLOURS[test], label=label)
        name = {"L4": "L4 (beam straight down)", "L5": "L5 (beam at a slant)"}[pair]
        row[0].set_yscale("log")
        for edge in (-10, 10):
            row[0].axvline(edge, color="#868e96", ls=":", lw=1)
        row[0].set_ylim(bottom=0.5)
        row[0].text(22, 300, "outside the dotted lines:\noff by > 1 cm", fontsize=8, color="#495057")
        row[0].set_xlabel("reading minus that beam's usual value (mm)")
        row[0].set_ylabel("number of readings (log)")
        row[0].set_title(f"{name}: how the readings spread, centre beams, all runs", fontsize=10)
        row[1].set_xlabel("time in the run (s)")
        row[1].set_ylabel("reading minus usual value (mm)")
        row[1].set_title(f"{name}: one typical centre beam over run 1", fontsize=10)
        row[2].set_xlabel("run")
        row[2].set_ylabel("99% of readings within \u00b1 (mm)")
        row[2].set_xticks(range(1, 1 + max(len(v) for v in per_run.values())))
        row[2].set_ylim(bottom=0)
        row[2].set_title(f"{name}: spread of every run", fontsize=10)
        for ax in row:
            ax.grid(alpha=0.3)
            ax.legend(fontsize=8)
    fig.suptitle("Nothing moves during a run: every reading should equal its beam's usual value (median). "
                 "Spread = 99% of the readings are within \u00b1 this", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(out_png, dpi=130)
    plt.close(fig)
    with open(out_csv, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()), lineterminator="\n")
        w.writeheader()
        w.writerows(rows)


def boxes(ax, x0, x1, top):
    ax.add_patch(Rectangle((x0, 0), x1 - x0, top, facecolor="#e9c46a", edgecolor="#8d6e2f"))
    ax.text((x0 + x1) / 2, top / 2, "boxes", ha="center", va="center", fontsize=8)


def surface_sketch(out, height=0.4, slant=1.0):
    """Side views of L4 (vertical) and L5 (tilted); the brown sheet is there in b only."""
    fig, axes = plt.subplots(2, 1, figsize=(9, 6.2))
    brown = "#5c3d12"
    for ax in axes:
        ax.axhspan(-0.06, 0, color="#c8b89a")
        ax.text(1.75, -0.03, "arena surface", ha="right", va="center", fontsize=8)
        ax.set_xlim(-0.75, 1.8)
        ax.set_ylim(-0.08, 0.7)
        ax.set_aspect("equal")
        ax.set_ylabel("height (m)")
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)

    ax = axes[0]  # L4 vertical: lidar on its side, base taped to the boxes' vertical side
    boxes(ax, -0.4, 0, height + 0.08)
    ax.add_patch(Rectangle((0, height - 0.038), 0.041, 0.076, color="black"))
    ax.plot([0.03, 0.03], [0, height + 0.25], color="red", ls=":", lw=0.9)
    ax.annotate("", xy=(0.03, 0.006), xytext=(0.03, height - 0.045), arrowprops=dict(arrowstyle="->", color="red", lw=1.5))
    ax.add_patch(Rectangle((0, 0), 0.4, 0.012, facecolor="none", edgecolor=brown, ls="--"))
    ax.text(0.08, height, "lidar on its side, base taped to the side of the boxes;\nbeam straight down, ~%.1f m"
            % height, fontsize=8, va="center")
    ax.text(0.45, 0.03, "b: sheet here (against the boxes, under the lidar)", fontsize=8, color=brown)
    ax.set_title("L4 vertical: L4a bare floor / L4b cardboard sheet", loc="left", fontsize=10)

    ax = axes[1]  # L5 tilted: lidar on top of the boxes, at the front edge, tilted down
    tilt = math.asin(height / slant)
    reach = slant * math.cos(tilt)
    boxes(ax, -0.4, 0.04, height - 0.03)
    ax.add_patch(Rectangle((-0.038, height - 0.03), 0.076, 0.041, color="black", angle=-math.degrees(tilt)))
    ax.plot([0, reach], [height, 0], color="red", lw=1.2)
    ax.add_patch(Rectangle((reach - 0.2, 0), 0.4, 0.012, facecolor="none", edgecolor=brown, ls="--"))
    ax.text(0.1, height + 0.12, "lidar on top of the boxes at the front edge, tilted down ~%.0f\N{DEGREE SIGN}\n"
            "(a ~3.5 cm piece under the edge facing you); beam hits the floor ~%.1f m away"
            % (math.degrees(tilt), slant), fontsize=8)
    ax.text(reach, 0.03, "b: sheet here (~%.2f m out)" % reach, ha="center", fontsize=8, color=brown)
    ax.set_title("L5 tilted: L5a bare floor / L5b cardboard sheet", loc="left", fontsize=10)

    axes[-1].set_xlabel("m   (side views; in each pair only the sheet changes)")
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    fig.savefig(os.path.splitext(out)[0] + ".svg")
    plt.close(fig)


def top_view(used, p, out):
    fig, axes = plt.subplots(1, 3, figsize=(15, 5.6))
    for ax, test in zip(axes, ("L1", "L2", "L3")):
        for d in used[test]:
            x, y = load_points(d, p["box_angle"])
            ax.scatter(x, y, s=0.4, color=COLOURS[test], alpha=0.25, linewidths=0)
        ax.add_patch(Wedge((0, 0), p["distance"] + p["zone_margin"], 90 - p["zone_half_angle"],
                           90 + p["zone_half_angle"], color="#ffd43b", alpha=0.25, lw=0))
        ax.plot(0, 0, marker="o", color="black", ms=5)
        ax.text(0.08, -0.25, "lidar", fontsize=8)
        ax.text(0, -1.6, "you + laptop side", ha="center", fontsize=8, color="#495057")
        ax.text(0.45, 2.35, "zone", fontsize=8, color="#b08900")
        ax.set_title(f"{NAMES[test]}\n({len(used[test])} runs, every return of every scan)", fontsize=10,
                     color=COLOURS[test])
        ax.set_xlim(-4, 4)
        ax.set_ylim(-2.5, 4)
        ax.set_aspect("equal")
        ax.grid(alpha=0.3)
        ax.set_xlabel("m")
    axes[0].set_ylabel("m   (box direction = up)")
    fig.suptitle("What the lidar saw, from above (each dot = one return; yellow = zone where the box stands)",
                 fontsize=11)
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)


def box_edges(used, p, out):
    fig, ax = plt.subplots(figsize=(9, 4.4))
    max_range = p["distance"] + p["zone_margin"]
    ranges = []
    for test in ("L2", "L3"):
        rates = [box_hit_rate(d, p["box_angle"], max_range) for d in used[test]]
        o = rates[0][0]
        mean = np.mean([r for _, r in rates], axis=0)
        sel = np.abs(o) <= 20
        order = np.argsort(o[sel])
        # visible width: outermost slots hit in at least half the scans, at the measured distance
        rng = np.median([json.load(open(os.path.join(d, "trial.json")))["metrics"]["range_median_m"] for d in used[test]])
        ranges.append(rng)
        seen = o[sel][mean[sel] >= 0.5]
        width = rng * (math.tan(math.radians(-seen.min())) + math.tan(math.radians(seen.max())))
        ax.plot(o[sel][order], 100 * mean[sel][order], color=COLOURS[test], lw=1.6,
                label=f"{NAMES[test]}: box seen {width * 100:.0f} cm wide")
    half = math.degrees(math.atan(BOX_WIDTH_M / 2 / np.mean(ranges)))
    for edge in (-half, half):
        ax.axvline(edge, color="#8d6e2f", ls=":", lw=1)
    ax.text(0, 45, f"real box face ({BOX_WIDTH_M * 100:.0f} cm) between the dotted lines", ha="center",
            fontsize=9, color="#8d6e2f")
    ax.annotate("dip at 0: driver artefact\n(first slot of each scan), both setups", xy=(0, 88), xytext=(3.5, 70),
                fontsize=8, arrowprops=dict(arrowstyle="->", lw=0.7))
    ax.set_xlabel("angle from the box centre (deg)")
    ax.set_ylabel("scans with a return on the box (%)")
    ax.set_ylim(-3, 105)
    ax.set_title(f"Box ends: seen less often with the lidar on the floor (mean of {len(used['L2'])} runs each)",
                 fontsize=10)
    ax.legend(loc="lower center", fontsize=9)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)


def main():
    session = sys.argv[1]
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(session, "figures")
    os.makedirs(out, exist_ok=True)
    used, campaign = select_runs(session)
    print("runs used: " + ", ".join(f"{t} {len(d)}" for t, d in used.items()))
    sketch(os.path.join(out, "setup_sketch.png"))
    surface_sketch(os.path.join(out, "surface_sketch.png"))
    made = ["setup_sketch.png", "setup_sketch.svg", "surface_sketch.png", "surface_sketch.svg"]
    if all(used[t] for t in ("L1", "L2", "L3")):
        top_view(used, campaign["L2"], os.path.join(out, "scans_top_view.png"))
        box_edges(used, campaign["L2"], os.path.join(out, "box_edges.png"))
        made += ["scans_top_view.png", "box_edges.png"]
    if any(used[q + "a"] and used[q + "b"] for q in ("L4", "L5")):
        surface_summary(used, campaign, os.path.join(out, "surface_summary.png"), os.path.join(out, "surface_analysis.csv"))
        made += ["surface_summary.png", "surface_analysis.csv"]
    for pair in PAIRS:
        if used[pair + "a"] and used[pair + "b"]:
            name = f"{pair.lower()}_surface.png"
            pair_surface(pair, used, campaign, os.path.join(out, name))
            made.append(name)
    print(f"figures in {out}: " + " ".join(made))


if __name__ == "__main__":
    main()

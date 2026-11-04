# Lidar floor tests (RPLiDAR A3, off the robot)

Questions: **does lying on the arena floor change what the lidar sees?** (L1–L3) and **does the floor
produce reflections that affect the sensor?** (L4–L6: is the distance noisier on the bare floor than on a
thin cardboard sheet, measured three ways)

**Status:** L1–L3 done (15/15, 2026-11-03). Next: the **surface tests L4–L6**, in their own section below.

Static experiments, 5 runs each. The lidar, the target and you stay in the same place; only one thing
changes from one experiment to the next.

| Exp. | Lidar | Target | Shows |
| --- | --- | --- | --- |
| **L1** | on the floor | none | the surface alone: any return where the box would stand comes from the ground |
| **L2** | on the floor | box at 2.0 m | how a real object is seen from the surface |
| **L3** | raised 20–30 cm | box at 2.0 m | the same without the surface (control): L2 vs L3 = surface effect |
| **L4a / L4b** | on its side (scan vertical), taped to a box ~40 cm up | the floor straight below / a thin cardboard sheet there | distance lidar → floor, hit head-on |
| **L5a / L5b** | on top of boxes ~40 cm up, tilted down | the floor ~1 m away / the sheet there | distance lidar → floor, hit at a slant |
| **L6a / L6b** | flat on the floor | box at 1 m, bare floor / the sheet covering the floor in between | distance lidar → box, with and without the floor under the beam |

L1–L3 are in **Steps**; the surface tests L4–L6 (asked by the supervisor) have their own section after
them, with their own list of what you need.

## You need

- The lidar on USB to the laptop (laptop charged).
- A cardboard box with a flat face, at least 10 cm taller than the raised lidar (e.g. ≥ 40 cm tall),
  with something heavy inside so wind can't move it.
- A stable stand for L3 (box/stool, 20–30 cm).
- Floor tape (or chalk outdoors) to mark positions. No tape measure needed for L1–L3 (L4–L6 need one).
- A flat patch of the surface you want to test, with nothing else within ~3 m of the box side.

## Steps

### 1. Start

Plug the lidar's USB into the laptop first (it starts spinning when the driver starts). Then, in a terminal:

```bash
cd ~/robot-tests-nexus-arena
./start_rplidar_tests.sh
curl -s localhost:8091/status         # wait ~10 s: "lidar_alive": true, "scan_hz": ~12.7
docker logs -f rplidar-tests          # in a second terminal: live output, leave it open
```

If `lidar_alive` stays `false`: `./stop_rplidar_tests.sh`, unplug and replug the USB, start again.

### 2. Set up (once)

1. Lidar flat on the floor. You and the laptop on one side of it: that is your spot for every run.
2. Box on the **other** side, about 2 m away, its face square to the lidar
   (the same side of the lidar as in the indoor check).
3. `curl -s localhost:8091/peek` and adjust until it looks like this:

   ```text
   "at_box_angle_180deg": "2.003 m (16 points)",   <- ~2.00 m (move the box closer/farther; +-5 cm is fine)
   "box_face": {
     "width_m": 0.69,                              <- about your box's real width
     "check": "centred"                            <- otherwise slide the box sideways as it says
   ```

   If it shows `"nothing"`, the box is not on the lidar's 180° side: rotate the **lidar** (not the box)
   until it appears. `nearest_per_sector` shows where things are.
4. Mark the outlines of the lidar and the box on the ground. From now on, don't move the lidar.

### 3. Run the tests (L1 → L2 → L3, 5 runs each, back to back)

How every run works: you type the command and then keep still at your spot. After 5 s the API records
for 30 s. When the log (second terminal) prints a `finished` line, start the next run.

- If something disturbs a run (someone walks through, wind moves the box), run
  `curl -s -X POST localhost:8091/stop`. It's saved as ABORTED and not counted; repeat the same command.
- A run that ends `FAILED` is not counted either; repeat the same command. In L2, L3 and L4 a run also
  fails if the target isn't seen in at least half the scans (`target not seen`): check `/peek` first.
- Progress at any time: `curl -s localhost:8091/status` → `"done"`.

#### L1: lidar on the floor, no box (~4 min)

1. Lift the box off its marks and put it far away, behind you. Don't touch the lidar.
2. Run 5 times, one after the other:

   ```bash
   curl -s -X POST localhost:8091/l1 -d '{}'
   ```

3. Good run: `finished: DONE`, `zone points/scan` ~0 (more than 0 = returns from the ground).
4. `/status` → `"L1 empty (lidar floor, no box)": "5/5"`.

#### L2: lidar on the floor, box at 2 m (~4 min)

1. Put the box back exactly on its marks. Don't touch the lidar.
2. Check: `curl -s localhost:8091/peek` → `at_box_angle_180deg` ~2.00 m and `"check": "centred"`.
3. Run 5 times:

   ```bash
   curl -s -X POST localhost:8091/l2 -d '{}'
   ```

4. Good run: `finished: DONE`, `detection` ~1.0, `range` ~2.0 m.
5. `/status` → `"L2 box (lidar floor, box)": "5/5"`.

#### L3: lidar raised, same box (~5 min)

1. Leave the box on its marks.
2. Put the stand on the lidar's mark and the lidar flat on top of it: centred above the mark, facing the
   same way as before. Keep the USB cable out of the way.
3. Check: `curl -s localhost:8091/peek` → `at_box_angle_180deg` ~2.00 m and `"check": "centred"`.
   If not, rotate or shift the **lidar** slightly on the stand, not the box.
4. Run 5 times:

   ```bash
   curl -s -X POST localhost:8091/l3 -d '{}'
   ```

5. Good run: `finished: DONE`, `detection` ~1.0, `range` ~2.0 m.
6. `/status` → `"L3 raised (lidar raised, box)": "5/5"`. Done!

### 4. Stop

```bash
./stop_rplidar_tests.sh
```

Optionally open `results/lidar-benchmark/manual.csv` and add `observations` (see below).
The comparison is in `results/lidar-benchmark/summary.csv`.

## Surface tests L4–L6: does the floor's reflection affect the sensor? (L4 + L5: 20 runs, ~40 min)

Each test is a pair: **a** = bare arena floor, **b** = a thin cardboard sheet laid where the beam meets the
floor, **nothing else touched**. Cardboard is the matte reference: if the floor's reflections disturb the
sensor, **a** shows more noise, more outliers or fewer returns than **b**. The three tests do it three ways,
so you get the whole picture. Picture: `results/lidar-benchmark/figures/surface_sketch.png`.

| Test | Lidar | The beam measures | Recommended distance |
| --- | --- | --- | --- |
| **L6** horizontal | flat on the floor (as in L2) | the box at 1 m; in **b** the sheet covers the floor under the beam | box **1.0 m** away |
| **L5** tilted (like the supervisor's drawing) | on top of boxes ~40 cm high, tilted down | the floor ~1 m away, at a slant | **1.0 m** to the floor (spot ~0.9 m out) |
| **L4** vertical | on its side, taped to the side of a box | the floor straight below, head-on | **~40 cm** |

**Recommended: L5 and L4** (20 runs, ~40 min): they measure the distance lidar → floor, as the supervisor
asked. **L6 is optional**: it is yesterday's L2 (box at 1 m instead of 2 m) plus a sheet on the floor in
between; it doesn't measure the floor itself. For each test: 5 × a, then 5 × b.

### You need

- The lidar on USB to the laptop (laptop charged), the 98 cm box (L6 target).
- **Boxes to raise the lidar ~40 cm** (one box or a stack), with a flat top and a flat vertical side.
- **A separate flat cardboard sheet** (don't cut the box, L6 needs it): **~100 × 40 cm** (minimum 60 × 30 cm),
  one flat layer, 3–5 mm, plain matte brown, no tape or print in the middle.
- A ~3.5 cm thick piece to tilt the lidar (L5: folded cardboard, a small block), strong tape, a shoe, a tape measure.
- A flat patch of arena floor, about 2 × 1.5 m, nothing on it.

### Before the first test (once, ~5 min)

Ignore `box_face` in the `/peek` output in L4 and L5: it is meant for a box, not the floor.

1. Plug the lidar into the laptop, then `./start_rplidar_tests.sh`; wait for `"lidar_alive": true`
   (`curl -s localhost:8091/status`). Second terminal: `docker logs -f rplidar-tests`.
2. **Find and mark the lidar's 180° side** (the side all tests use). Lidar flat on the ground, a shoe ~50 cm
   from it; `curl -s localhost:8091/peek`: if `at_box_angle_180deg` reads ~0.5 m, the shoe is on that side;
   if not, rotate the lidar until it does. Put a piece of tape on the lidar's **base** on that side.

Every run: type the command, keep still at your spot until the log prints `finished` (~35 s), then the next.
Disturbed run: `curl -s -X POST localhost:8091/stop` and repeat (not counted). `FAILED (target not seen ...)`:
the target wasn't where expected; check `/peek`, fix, repeat. Never tape the lidar's top: it spins.

### L6 horizontal (optional, ~10 min)

Setup:
1. Lidar flat on the floor, marked side towards the box. Box **1.0 m** away, face square to the lidar.
2. `/peek` → `at_box_angle_180deg` **~1.00 m** and `"check": "centred"` (slide the box if not).
3. Mark the lidar and the box on the floor. You and the laptop ≥ 1.5 m behind the lidar.

Runs:
- **L6a** (bare floor between lidar and box), 5 times: `curl -s -X POST localhost:8091/l6a -d '{}'`.
  Good run: `DONE`, `detection` ~1.0, `range` ~1.0 m.
- **L6b**: lay the sheet flat on the floor **between** the lidar and the box: long side along the beam, from
  the lidar's front edge to the box, centred on the line between them. Don't touch the lidar or the box.
  `/peek` still ~1.00 m. 5 times: `curl -s -X POST localhost:8091/l6b -d '{}'`.

### L5 tilted (~10 min)

Setup:
1. Lidar **flat** on top of the boxes (~40 cm high), its marked side **at the front edge** of the top,
   facing the open floor (otherwise the tilted beam hits the box top instead of the floor).
2. Slide the ~3.5 cm piece under the lidar's edge **facing you**, so the marked side tips down (about 24°).
   Tape the lidar's base. `/peek` → `at_box_angle_180deg` must read **1.00 m ± 0.05**: more → a thicker
   piece, less → thinner. `"nothing"`: the wrong edge is raised, or the beam hits the box top (move the lidar
   to the front edge).
3. Mark the beam spot on the floor: **~90 cm** out from the front of the boxes, straight ahead of the lidar.
   From now on don't touch the lidar or the boxes. You and the laptop behind the boxes.

Runs:
- **L5a** (bare floor), 5 times: `curl -s -X POST localhost:8091/l5a -d '{}'`.
  Good run: `DONE`, `detection` ~1.0, `range` ~1.0 m.
- **L5b**: lay the sheet flat, centred on the spot, long side **across** the beam (left-right as seen from the
  lidar). `/peek` reads **~1 cm less** (the sheet's thickness, seen at a slant; expected). If it reads the same,
  slide the sheet along the beam. 5 times: `curl -s -X POST localhost:8091/l5b -d '{}'`.

### L4 vertical (~10 min)

Setup:
1. Tape the lidar's **base** flat against a **vertical side** of the boxes, the lidar **on its side**, its
   centre ~40 cm above the floor, marked side **pointing straight down**. Below the lidar nothing may stick out
   from that side more than ~2 cm (the beam passes ~3 cm in front of it).
2. `/peek` → `at_box_angle_180deg` ≈ **your height** (~0.40 m to the lidar's centre).
   Clearly shorter: the beam hits something below the lidar. `nearest_per_sector` → `"-180"`: the nearest
   point should be at **±175–180°** (e.g. `"0.402 m at -178.6 deg"`); if not, turn the lidar a little around
   its own axis (straight down is where the floor reads shortest).
3. You and the laptop ≥ 1 m in front of the boxes, not beside the lidar along the box side (that is inside
   the scan).

Runs:
- **L4a** (bare floor), 5 times: `curl -s -X POST localhost:8091/l4a -d '{}'`.
  Good run: `DONE`, `detection` ~1.0, `range` ≈ your height.
- **L4b**: slide the sheet in flat, pushed against the bottom of the boxes, long side along the box side,
  centred under the lidar, without touching the lidar. `/peek` reads **3–5 mm less** (the sheet's thickness). 5 times:
  `curl -s -X POST localhost:8091/l4b -d '{}'`.

### At the end

`curl -s localhost:8091/status` → L4a, L4b, L5a, L5b (and L6a, L6b if you did L6) at `5/5`. Then `./lidar_viz.sh plot` (makes
`l4_surface.png`, `l5_surface.png`, `l6_surface.png`) and `./stop_rplidar_tests.sh`. Optionally write the
heights, the sheet's size/thickness and the weather in `manual.csv` → `observations`.

### The results: `results/lidar-benchmark/surface_results.csv`

One row per run plus a **MEAN** row per experiment (L4a, L4b, L5a, …), with only these numbers. Compare
**a with b** within each test:

| Column | Meaning | Reflections from the floor would show as |
| --- | --- | --- |
| `noise_mm` | how much a single beam's distance jumps from scan to scan | higher in a (bare floor) than in b |
| `outliers_pct` | % of readings more than 2 cm off the usual value | higher in a |
| `returns_pct` | % of scans where the centre beam got a return | lower in a |
| `distance_mm` | lidar → floor (L4, L5) or → box (L6) | L4/L5: a − b far from the sheet's thickness (L4: 3–5 mm, L5: ~1 cm); L6: a ≠ b |

If a and b are the same (within the run-to-run spread) in all three tests, the floor's reflection does not
affect the sensor. The `l4/l5/l6_surface.png` figures show the same numbers beam by beam (L4/L5 out to 60°,
where beams meet the floor more and more slanted, which is where reflections usually matter most).

## What you write yourself (optional)

Nothing to measure. In `manual.csv` you may write `observations` (e.g. sunny / cloudy, wind, surface type,
stand height) and `notes`. To discard a finished run, write `INVALID` in its `override_status` and run it
again (it reuses the repeat number).

## What the API measures (automatically, every run)

**Zone** = where the box stands: 180° ± 10°, closer than 2.5 m (L4: 0.9 m, L5 and L6: 1.5 m). It is the
same zone in L1, L2 and L3.

| Number | Meaning | Compare |
| --- | --- | --- |
| `zone_points_mean` | points per scan in the zone | L1: should be 0, any point is a false return (nothing is there). L2 vs L3: how much of the box face is seen |
| `detection_rate` | share of scans that see the box centre (180° ± 2°) | L2 vs L3 (1.0 = every scan) |
| `range_median_m` | measured distance to the box centre | L2 vs L3: the same = the floor doesn't bias it |
| `range_std_mm` | how much that distance varies between scans (median of the ±2° beams per scan) | L2 vs L3: larger = noisier |
| `beam_std_mm` | the same per single beam, averaged over the ±2° beams (added for L4; empty for older runs) | L4a vs L4b: larger = noisier |
| `outlier_fraction` | share of those single-beam readings more than 2 cm from their usual value (added for L4) | L4a vs L4b: more = occasional wrong readings, e.g. reflections |
| `valid_fraction` | share of all 1440 beams (360°) with a return | L1/L2 vs L3: floor hits or blocked beams in other directions |
| `scan_hz`, `max_gap_s` | driver health | ~12.7 Hz; a gap > 0.5 s makes the run FAILED |

Intensity is not measured: in this mode the A3 only reports 0 (no return) or 47 (return).

## Results: `results/lidar-benchmark/`

- `summary.csv`: **the comparison**, one row per experiment (mean and sd of its 5 runs).
- `surface_results.csv`: **L4–L6 only**, the four numbers that matter per run + the mean per experiment.
- `run_log.csv`: one row per run (all numbers above + your observations/notes).
- `manual.csv`: your observations/notes and INVALID marks.
- `<run id>/` (e.g. `L2_box_d2.00_r3_101530`, time in UTC):
  - `trial.json`: settings, driver parameters, metrics
  - `scans.csv`: one row per scan
  - `angles.csv`: one row per beam angle (return rate, range statistics), for 360° plots
  - `bag/scan.bag`: raw `/scan` (ROS 1 bag; not in git, like the RoverX bags)

## Look at the data

```bash
./lidar_viz.sh plot                                                 # figures -> results/lidar-benchmark/figures/
./lidar_viz.sh rviz results/lidar-benchmark/L2_box_d2.00_r1_140502  # RViz replaying one run (loops; close the window to stop)
./lidar_viz.sh rviz                                                 # RViz on the live lidar (stack running)
```

Figures: `setup_sketch.png` (L1–L3 setups, side view), `scans_top_view.png` (every return of the
5 runs, from above), `box_edges.png` (how often each beam across the box got a return, L2 vs L3),
`surface_sketch.png` (L4 and L5 setups), `surface_summary.png` (L4/L5 from the measured readings: how the
readings spread on floor vs cardboard, one typical beam over a run, the noise of every run; numbers in
`surface_analysis.csv`), `l4_surface.png` / `l5_surface.png` (a vs b beam by beam: distance, noise, returns).
In RViz the red axis is the scan's 0°; the box is on the opposite side. The viewer image (`rplidar-viz`,
RViz + matplotlib) is built automatically the first time.

## Notes

- Outdoors: strong direct sun can reduce the lidar's returns. That is why the runs being compared go
  back to back (L1–L3 together, L4a and L4b together), and why the weather goes in `observations`.
- Don't change `distance`/`box_angle` during the campaign: a run with other values is not counted in the 5/5.
  (L4–L6's `distance` is nominal; your exact height/distance doesn't matter, only that a and b share it.)
  `"notes": "..."` in the request body is always fine.
- The box side is the scan's 180° (the driver's 0° points the other way); measured 2026-11-03.
- The lidar driver is the unchanged `dockerfiles/rplidar` container (ROS 1 Melodic, Sensitivity mode,
  1440 points/scan). The test API (`scripts/api.py`, ROS 1 Noetic) is mounted, so an edit only needs
  `docker restart rplidar-tests`. On a new PC, install the udev rule once:
  `cd dockerfiles/rplidar && ./apply_lidar_rules.sh`.

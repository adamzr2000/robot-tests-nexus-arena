# L4 ground-truth runs

The lidar is on its side (scan plane vertical), with its 180° beam pointing straight down at the floor.
Each run compares the lidar's distance readings with the distance measured by hand (ground truth).

## Running

```bash
./start_rplidar_tests.sh                       # session lidar-l4 is the default
curl -s localhost:8091/peek                    # "at_box_angle_180deg" ≈ your ground truth
curl -s -X POST localhost:8091/l4a -d '{"ground_truth_mm": 400}'     # bare floor
curl -s -X POST localhost:8091/l4b -d '{"ground_truth_mm": 397}'     # cardboard: to the top of the sheet
```

- **Ground truth:** measured by hand from the lidar's rotation centre (the centre of the round housing)
  to the surface directly below. With the sheet (L4b), measure to the top of the sheet. It is saved in
  each run's `trial.json`, and the API refuses an L4 run without it. If the lidar moves, measure again
  and pass the new value.
- Each run waits 5 s, then records 30 s (~390 scans). Stay still and out of the beam.

## Analysis

```bash
./lidar_viz.sh plot                            # writes figures/l4_error.csv, l4_error_runs.csv, l4_error.png
```

Uses every DONE run of L4a/L4b (not marked INVALID in `manual.csv`), only the recorded distances and
the ground truth of each run.

The table and how each value is calculated: [figures/README.md](figures/README.md).

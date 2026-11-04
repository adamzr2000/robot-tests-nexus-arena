# surface_analysis.csv

How steady the lidar's distance readings are on the bare floor (L4a, L5a) and on a cardboard sheet
(L4b, L5b). L4: beam straight down; L5: beam at a slant. Nothing moves during a run, so a perfectly
steady surface would give the same reading every scan.

All values come only from the distances the lidar recorded; no hand-measured value is used.

```bash
./lidar_viz.sh plot        # from the repo root: recreates this file from the runs' bag files
```

Script: [`plot_scans.py`](../../../dockerfiles/rplidar-tests/scripts/plot_scans.py) (`centre_beams`, `run_numbers`, `numbers`).
Input: `<run>/bag/scan.bag` of every run with status DONE (and not marked INVALID in `manual.csv`).

## How the numbers are computed

1. **Beams.** In each scan, take the 14 beams within ±2° of the beam pointing at the surface (scan
   angle 180°). A reading counts if it is 0.15 m or more and at most the test distance + 0.5 m.
2. **Used beams.** Keep the beams with a reading in at least half of the run's scans (about 390 scans in 30 s).
3. **Beam median.** For each used beam, the median of its readings in that run.
4. **Deviation.** Each reading minus its beam median, e.g. 406 mm − 391 mm = +15 mm.

Beam medians are always taken within one run. The lidar reports distances in steps of 1 mm at 0.4 m
and 2 mm at 1 m.

## Rows

- **Run rows** (`trial_id` = run folder): the numbers of that one run.
- **`ALL (4 runs)` rows**: the readings of the 4 runs of a test put together, then the same numbers
  computed again. They are not averages of the run rows.

## Fields

| Field | Meaning |
| --- | --- |
| `test` | L4a, L4b, L5a, L5b |
| `trial_id` | run folder, or `ALL (4 runs)` |
| `surface` | `bare floor` or `cardboard` |
| `readings` | number of readings of the used beams |
| `distance_mm` | median of the beam medians: the measured distance to the surface |
| `noise_mm` | standard deviation of each used beam's readings (around that beam's mean), averaged over the used beams |
| `spread_99_mm` | at least 99% of the readings are within ± this value of their beam median |
| `min_dev_mm` | largest deviation towards the lidar (reading too close) |
| `max_dev_mm` | largest deviation away from the lidar (reading too far) |
| `off_1cm_pct` | % of readings more than 10 mm from their beam median |
| `off_1cm_far_pct` | of the readings more than 10 mm off, the % that are too far; empty if there are none |
| `beam_returns_pct` | % of scans in which a beam had a reading, averaged over all 14 beams (used or not) |

`noise_mm` is an average: a few large jumps among many steady readings change it little.
`spread_99_mm`, `min_dev_mm`, `max_dev_mm` and `off_1cm_pct` show the large jumps directly.

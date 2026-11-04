# l4_error.csv

L4: lidar on its side, beam straight down, 5 runs on the bare floor (L4a) and 5 on a 7 mm cardboard
sheet (L4b). One row per surface. Computed only from the recorded distances and the hand-measured
ground truth.

```bash
./lidar_viz.sh plot        # from the repo root: recreates this file from the runs' bag files
```

## Data used

For each surface:

- g = ground truth: distance from the lidar's rotation centre to the surface, measured by hand
  (bare floor 400 mm, cardboard 400 − 7 = 393 mm)
- rᵢ = every distance the lidar measured within ±2° of straight down (14 per scan), in every scan of
  all 5 runs (i = 1 … N)
- eᵢ = rᵢ − g, the error of each reading

All readings count equally.

## Fields

| Field | Formula | Meaning |
| --- | --- | --- |
| `surface` | | `bare floor` or `cardboard` |
| `ground_truth_mm` | g | hand-measured distance |
| `readings` | N | number of readings |
| `lidar_mean_mm` | r̄ = (1/N) Σ rᵢ | the lidar's average distance |
| `error_mean_mm` | ē = (1/N) Σ eᵢ = r̄ − g | how far off the lidar is on average (negative = reads shorter) |
| `error_std_mm` | σ = √((1/N) Σ (eᵢ − ē)²) | how much the readings vary, within and between runs |

`l4_error_runs.csv` gives the same fields for each run separately.

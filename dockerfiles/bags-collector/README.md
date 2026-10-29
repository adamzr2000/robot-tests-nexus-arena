# bags-collector

Records `ros2 bag` over HTTP (`scripts/main.py`, port 8081). Normally driven by
`robot-trajectories` with `"record_bag": true`; call it directly only to record
outside a trajectory.

## Endpoints

| Method | Path      | Description                                         |
|--------|-----------|------------------------------------------------------|
| POST   | `/start`  | Start recording; JSON body: `{"tag": "...", "topics": [...]}` |
| POST   | `/stop`   | Stop the current recording                           |
| GET    | `/status` | Currently recording bag, if any                      |
| GET    | `/bags`   | List bags under `BAGS_DIR`, with sizes               |

Both `tag` and `topics` are optional. `topics` omitted uses `DEFAULT_TOPICS`
in `scripts/main.py` (empty by default, meaning record everything via
`ros2 bag record -a`); edit that list to your project's usual topic set.
`tag` labels the run and is folded into the bag's folder name as
`<timestamp>_<tag>`, sanitized to safe filename characters.

`/start` refuses with `409` if a recording is already in progress rather than
silently stopping it, since that could cut a run short without you noticing.

### Example

```bash
curl -X POST localhost:8081/start -d '{"tag": "exp1", "topics": ["/cmd_vel", "/go2/scan"]}'

curl localhost:8081/status

curl -X POST localhost:8081/stop

curl localhost:8081/bags
```

## Exporting data

`BAGS_DIR` (default `/home/robot/bags`) is bind-mounted to `./bags` at the
repo root in `docker-compose-go2.yml` and `docker-compose-roverx.yml`, so recordings land there directly on
the host, ready to copy off without needing to reach into the container.

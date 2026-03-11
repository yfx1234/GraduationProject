# PythonClient Layout

## Active structure

- `gradsim/`: reusable TCP client library and thin agent wrappers.
- `Drone/`: active drone-side scripts, including `auto_spawn_visual_intercept.py`.
- `Turret/`: legacy compatibility entry points kept only to avoid breaking old commands.
- `YOLO/`: third-party and dataset/training assets.

## Recommended entry points

- Main intercept script: `PythonClient/Drone/auto_spawn_visual_intercept.py`
- Legacy wrapper: `PythonClient/Turret/auto_spawn_visual_intercept.py`
- Reusable client API: `PythonClient/gradsim/client.py`

## Cleanup policy

- Runtime snapshots are written to `PythonClient/Drone/snapshots/` and are ignored by git.
- `__pycache__/` and local editor settings are ignored.
- New drone-control scripts should live under `PythonClient/Drone/`.

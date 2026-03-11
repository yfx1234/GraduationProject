# Drone Scripts

## Active script

- `auto_spawn_visual_intercept.py`: spawn interceptor + target, keep target on a stable orbit reference, and call the C++ `auto_intercept` loop with optional visualization.

## Runtime output

- `snapshots/`: optional local images generated during debugging or visualization experiments.

## Notes

- The script depends on `PythonClient/gradsim/client.py`.
- The legacy path `PythonClient/Turret/auto_spawn_visual_intercept.py` simply forwards to this script.

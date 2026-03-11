import runpy
from pathlib import Path

if __name__ == "__main__":
    target = Path(__file__).resolve().parents[1] / "Drone" / "auto_spawn_visual_intercept.py"
    runpy.run_path(str(target), run_name="__main__")

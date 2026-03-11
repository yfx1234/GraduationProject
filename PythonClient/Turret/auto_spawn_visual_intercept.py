"""兼容旧入口的转发脚本。

当前自动拦截实现统一维护在 `PythonClient/Drone` 目录下，这里仅保留一个
极薄的入口，避免已有启动方式失效。
"""

import runpy
from pathlib import Path

if __name__ == "__main__":
    # 用 runpy 保持“作为脚本执行”的语义，避免直接导入时跳过 argparse 入口。
    target = Path(__file__).resolve().parents[1] / "Drone" / "auto_spawn_visual_intercept.py"
    runpy.run_path(str(target), run_name="__main__")
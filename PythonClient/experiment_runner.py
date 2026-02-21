"""
ExperimentRunner — 批量自动化实验
3 种制导方法 × 3 种飞行模式 × N 次重复 → CSV 输出

使用方法:
  python experiment_runner.py [--repeats 5] [--duration 10] [--output results.csv]
"""

import argparse, csv, os, time, json
from datetime import datetime
import numpy as np
from sim_client import SimClient
from guidance_runner import GuidanceRunner

METHODS = ["direct", "proportional", "predictive"]
PATTERNS = ["straight", "curved", "evasive"]


def run_experiments(host="127.0.0.1", port=9000,
                    repeats=5, duration=10.0, dt=0.1,
                    fire_interval=10, output="results.csv"):
    """运行完整实验矩阵"""

    client = SimClient(host, port)
    runner = GuidanceRunner(client, muzzle_speed=400.0)

    # CSV header
    fieldnames = [
        "timestamp", "method", "pattern", "trial",
        "mean_error", "final_error", "fire_count",
        "elapsed", "num_steps"
    ]

    # 创建输出目录
    output_dir = os.path.dirname(output)
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir)

    with open(output, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()

        total = len(METHODS) * len(PATTERNS) * repeats
        count = 0

        for method in METHODS:
            for pattern in PATTERNS:
                for trial in range(repeats):
                    count += 1
                    print(f"\n[{count}/{total}] method={method}, pattern={pattern}, trial={trial+1}")

                    try:
                        result = runner.run_trial(
                            method=method,
                            pattern=pattern,
                            duration=duration,
                            dt=dt,
                            fire_interval=fire_interval,
                        )

                        row = {
                            "timestamp": datetime.now().isoformat(),
                            "method": method,
                            "pattern": pattern,
                            "trial": trial + 1,
                            "mean_error": f"{result['mean_error']:.4f}",
                            "final_error": f"{result['final_error']:.4f}",
                            "fire_count": result["fire_count"],
                            "elapsed": f"{result['elapsed']:.2f}",
                            "num_steps": len(result["positions"]),
                        }
                        writer.writerow(row)
                        f.flush()

                        print(f"  mean_err={result['mean_error']:.2f}, "
                              f"final_err={result['final_error']:.2f}, "
                              f"fires={result['fire_count']}")

                    except Exception as e:
                        print(f"  [ERROR] {e}")
                        row = {
                            "timestamp": datetime.now().isoformat(),
                            "method": method,
                            "pattern": pattern,
                            "trial": trial + 1,
                            "mean_error": "ERROR",
                            "final_error": "ERROR",
                            "fire_count": 0,
                            "elapsed": 0,
                            "num_steps": 0,
                        }
                        writer.writerow(row)
                        f.flush()

                    # 试验间隔
                    time.sleep(2)

    client.close()
    print(f"\n[DONE] Results saved to {output}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="GraduationProject 批量实验")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9000)
    parser.add_argument("--repeats", type=int, default=5, help="每组重复次数")
    parser.add_argument("--duration", type=float, default=10.0, help="单次实验时长(秒)")
    parser.add_argument("--dt", type=float, default=0.1, help="采样间隔(秒)")
    parser.add_argument("--fire-interval", type=int, default=10, help="每N步开火一次")
    parser.add_argument("--output", default="results/results.csv", help="CSV 输出路径")

    args = parser.parse_args()
    run_experiments(
        host=args.host, port=args.port,
        repeats=args.repeats, duration=args.duration,
        dt=args.dt, fire_interval=args.fire_interval,
        output=args.output,
    )

"""
ResultPlotter — 读取实验 CSV 绘制论文图表
  - 命中率/瞄准误差对比柱状图
  - 误差收敛曲线
  - 方法×飞行模式热力图

使用方法:
  python result_plotter.py [--input results/results.csv] [--output results/]
"""

import argparse, os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
matplotlib.rcParams["font.family"] = ["SimHei", "sans-serif"]
matplotlib.rcParams["axes.unicode_minus"] = False

METHOD_LABELS = {"direct": "直接瞄准", "proportional": "比例导引", "predictive": "预测制导"}
PATTERN_LABELS = {"straight": "直线", "curved": "弧线", "evasive": "S型机动"}


def plot_mean_error_bar(df, output_dir):
    """柱状图：各制导方法平均误差"""
    fig, ax = plt.subplots(figsize=(10, 6))

    methods = df["method"].unique()
    patterns = df["pattern"].unique()
    x = np.arange(len(methods))
    width = 0.25

    for i, pattern in enumerate(patterns):
        means = []
        stds = []
        for method in methods:
            subset = df[(df["method"] == method) & (df["pattern"] == pattern)]
            vals = pd.to_numeric(subset["mean_error"], errors="coerce").dropna()
            means.append(vals.mean())
            stds.append(vals.std())
        ax.bar(x + i * width, means, width, yerr=stds, label=PATTERN_LABELS.get(pattern, pattern), capsize=5)

    ax.set_xlabel("制导方法")
    ax.set_ylabel("平均瞄准误差 (cm)")
    ax.set_title("不同制导方法在各飞行模式下的平均瞄准误差")
    ax.set_xticks(x + width)
    ax.set_xticklabels([METHOD_LABELS.get(m, m) for m in methods])
    ax.legend()
    ax.grid(axis="y", alpha=0.3)

    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, "mean_error_bar.png"), dpi=150)
    plt.close()
    print("[Plot] mean_error_bar.png saved")


def plot_heatmap(df, output_dir):
    """热力图：方法 × 飞行模式"""
    methods = list(METHOD_LABELS.keys())
    patterns = list(PATTERN_LABELS.keys())

    data = np.zeros((len(methods), len(patterns)))
    for i, method in enumerate(methods):
        for j, pattern in enumerate(patterns):
            subset = df[(df["method"] == method) & (df["pattern"] == pattern)]
            vals = pd.to_numeric(subset["mean_error"], errors="coerce").dropna()
            data[i, j] = vals.mean() if len(vals) > 0 else 0

    fig, ax = plt.subplots(figsize=(8, 5))
    im = ax.imshow(data, cmap="YlOrRd", aspect="auto")

    ax.set_xticks(range(len(patterns)))
    ax.set_xticklabels([PATTERN_LABELS[p] for p in patterns])
    ax.set_yticks(range(len(methods)))
    ax.set_yticklabels([METHOD_LABELS[m] for m in methods])

    # 数值标注
    for i in range(len(methods)):
        for j in range(len(patterns)):
            ax.text(j, i, f"{data[i,j]:.1f}", ha="center", va="center", color="black", fontsize=12)

    ax.set_title("瞄准误差热力图 (cm)")
    fig.colorbar(im, ax=ax, shrink=0.8)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, "error_heatmap.png"), dpi=150)
    plt.close()
    print("[Plot] error_heatmap.png saved")


def plot_summary_table(df, output_dir):
    """统计摘要表格"""
    summary = df.groupby(["method", "pattern"]).agg(
        mean_error=("mean_error", lambda x: pd.to_numeric(x, errors="coerce").mean()),
        std_error=("mean_error", lambda x: pd.to_numeric(x, errors="coerce").std()),
        final_error=("final_error", lambda x: pd.to_numeric(x, errors="coerce").mean()),
        fire_count=("fire_count", "mean"),
        trials=("trial", "count"),
    ).reset_index()

    summary["method_cn"] = summary["method"].map(METHOD_LABELS)
    summary["pattern_cn"] = summary["pattern"].map(PATTERN_LABELS)

    print("\n" + "=" * 70)
    print("实验结果摘要")
    print("=" * 70)
    print(summary[["method_cn", "pattern_cn", "mean_error", "std_error", "final_error", "fire_count", "trials"]].to_string(index=False))
    print("=" * 70)

    summary.to_csv(os.path.join(output_dir, "summary.csv"), index=False, encoding="utf-8-sig")
    print(f"[Table] summary.csv saved")


def main():
    parser = argparse.ArgumentParser(description="实验结果绘图")
    parser.add_argument("--input", default="results/results.csv")
    parser.add_argument("--output", default="results/")
    args = parser.parse_args()

    if not os.path.exists(args.input):
        print(f"[ERROR] File not found: {args.input}")
        return

    os.makedirs(args.output, exist_ok=True)

    df = pd.read_csv(args.input)
    print(f"[Data] Loaded {len(df)} rows from {args.input}")

    plot_mean_error_bar(df, args.output)
    plot_heatmap(df, args.output)
    plot_summary_table(df, args.output)


if __name__ == "__main__":
    main()

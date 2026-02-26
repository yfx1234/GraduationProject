"""
train_yolo.py — 用采集的数据训练 YOLO 模型

先运行 collect_data.py 采集数据，然后运行本脚本训练。

使用:
  python train_yolo.py                     # 默认使用上次训练出的 best.pt 继续训练
  python train_yolo.py --epochs 150        # 继续训练 150 轮
  python train_yolo.py --model yolo11n.pt  # 重新从官方轻量模型开始训练
"""

import os, sys, argparse, shutil

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# ultralytics 路径
_ultra_path = os.path.join(SCRIPT_DIR, "YOLO", "ultralytics")
if os.path.exists(_ultra_path):
    sys.path.insert(0, _ultra_path)

from ultralytics import YOLO


def create_data_yaml(dataset_dir, output_path):
    """生成 YOLO 数据集配置文件"""
    abs_path = os.path.abspath(dataset_dir).replace("\\", "/")
    yaml_content = f"""# YOLO 无人机检测数据集
path: {abs_path}
train: images/train
val: images/train  # 小数据集可以同训练集，大数据集应该分开

nc: 1
names:
  0: drone
"""
    with open(output_path, "w", encoding="utf-8") as f:
        f.write(yaml_content)
    print(f"[Config] {output_path} 已生成")
    return output_path


def split_dataset(dataset_dir, val_ratio=0.2):
    """将训练集按比例分出验证集"""
    import random

    train_img = os.path.join(dataset_dir, "images", "train")
    train_lbl = os.path.join(dataset_dir, "labels", "train")
    val_img = os.path.join(dataset_dir, "images", "val")
    val_lbl = os.path.join(dataset_dir, "labels", "val")
    os.makedirs(val_img, exist_ok=True)
    os.makedirs(val_lbl, exist_ok=True)

    files = [f for f in os.listdir(train_img) if f.endswith(".jpg")]
    random.shuffle(files)
    n_val = max(1, int(len(files) * val_ratio))

    for f in files[:n_val]:
        name = os.path.splitext(f)[0]
        shutil.move(os.path.join(train_img, f), os.path.join(val_img, f))
        lbl = name + ".txt"
        if os.path.exists(os.path.join(train_lbl, lbl)):
            shutil.move(os.path.join(train_lbl, lbl), os.path.join(val_lbl, lbl))

    print(f"[Split] train={len(files)-n_val}, val={n_val}")


def get_best_model():
    """在 PythonClient/YOLO/ 目录下搜索之前训练的最佳模型"""
    for name in ["drone_detect2", "drone_detect"]:
        p = os.path.join(SCRIPT_DIR, "YOLO", "runs", "detect", name, "weights", "best.pt")
        if os.path.exists(p):
            return p
    return "yolo11n.pt"

def main():
    parser = argparse.ArgumentParser(description="训练 YOLO 无人机检测模型")
    parser.add_argument("--dataset", default=os.path.join(SCRIPT_DIR, "YOLO", "dataset"), help="数据集目录")
    parser.add_argument("--model", default=get_best_model(), help="预训练模型 (默认寻找 runs 下的 best.pt，没有则用 yolo11n.pt)")
    parser.add_argument("--epochs", type=int, default=100, help="训练轮数")
    parser.add_argument("--imgsz", type=int, default=640, help="输入尺寸")
    parser.add_argument("--batch", type=int, default=16, help="批大小")
    parser.add_argument("--name", default="drone_detect", help="实验名称")
    parser.add_argument("--no-split", action="store_true", help="不分验证集")
    args = parser.parse_args()

    if not os.path.exists(os.path.join(args.dataset, "images", "train")):
        print("[ERROR] 数据集不存在，请先运行: python collect_data.py --num 500")
        return

    n_imgs = len([f for f in os.listdir(os.path.join(args.dataset, "images", "train"))
                  if f.endswith(".jpg")])
    print(f"[Data] 训练图像: {n_imgs} 张")

    if n_imgs < 20:
        print("[WARN] 图像太少，建议至少 200 张")

    # 分验证集
    if not args.no_split:
        val_dir = os.path.join(args.dataset, "images", "val")
        if not os.path.exists(val_dir) or len(os.listdir(val_dir)) == 0:
            split_dataset(args.dataset)

    # 生成配置
    yaml_path = os.path.join(args.dataset, "data.yaml")
    create_data_yaml(args.dataset, yaml_path)

    # 更新 yaml 如果有 val 集
    val_dir = os.path.join(args.dataset, "images", "val")
    if os.path.exists(val_dir) and len(os.listdir(val_dir)) > 0:
        abs_path = os.path.abspath(args.dataset).replace("\\", "/")
        with open(yaml_path, "w", encoding="utf-8") as f:
            f.write(f"path: {abs_path}\ntrain: images/train\nval: images/val\n\n")
            f.write("nc: 1\nnames:\n  0: drone\n")

    # 训练
    print(f"\n{'='*50}")
    print(f"  开始训练 YOLO 无人机检测模型")
    print(f"  模型: {args.model}")
    print(f"  轮数: {args.epochs}")
    print(f"  批大小: {args.batch}")
    print(f"  图像尺寸: {args.imgsz}")
    print(f"{'='*50}\n")

    model = YOLO(args.model)
    results = model.train(
        data=yaml_path,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        name=args.name,
        project=os.path.join(SCRIPT_DIR, "YOLO", "runs", "detect"),  # 输出到 PythonClient/YOLO/runs/detect/
        device=0,  # GPU
        workers=4,
        patience=20,
        save=True,
        plots=True,
    )

    # 输出结果
    best_model = os.path.join(SCRIPT_DIR, "YOLO", "runs", "detect", args.name, "weights", "best.pt")
    print(f"\n{'='*50}")
    print(f"  训练完成！")
    print(f"  最佳模型: {best_model}")
    print(f"{'='*50}")
    print(f"\n使用训练好的模型:")
    print(f"  python yolo_guidance.py --model {best_model}")


if __name__ == "__main__":
    main()

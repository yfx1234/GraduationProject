from __future__ import annotations

import json
import os
import sys
from pathlib import Path


IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}


def ensure_ultralytics_import_path(project_root):
    source_root = project_root / "PythonClient" / "YOLO" / "ultralytics"
    settings_root = project_root / "PythonClient" / ".ultralytics_config"
    settings_root.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("YOLO_CONFIG_DIR", str(settings_root))
    os.environ.setdefault("YOLO_AUTOINSTALL", "False")
    os.environ.setdefault("YOLO_VERBOSE", "False")
    source_text = str(source_root)
    if source_text not in sys.path:
        sys.path.insert(0, source_text)


def list_samples(dataset_root, split):
    images_dir = dataset_root / "images" / split
    labels_dir = dataset_root / "labels" / split
    samples = []
    if not images_dir.exists():
        return samples
    for image_path in sorted(images_dir.iterdir()):
        if image_path.is_file() and image_path.suffix.lower() in IMAGE_EXTS:
            samples.append((image_path, labels_dir / f"{image_path.stem}.txt"))
    return samples


def ensure_label_file(path):
    path.parent.mkdir(parents=True, exist_ok=True)
    if not path.exists():
        path.write_text("", encoding="utf-8")


def move_sample(sample, dataset_root, dst_split):
    image_path, label_path = sample
    dst_image_dir = dataset_root / "images" / dst_split
    dst_label_dir = dataset_root / "labels" / dst_split
    dst_image_dir.mkdir(parents=True, exist_ok=True)
    dst_label_dir.mkdir(parents=True, exist_ok=True)
    dst_image = dst_image_dir / image_path.name
    dst_label = dst_label_dir / label_path.name
    image_path.replace(dst_image)
    if label_path.exists():
        label_path.replace(dst_label)
    else:
        dst_label.write_text("", encoding="utf-8")


def write_data_yaml(dataset_root, use_train_as_val=False):
    yaml_path = dataset_root / "data.yaml"
    yaml_path.write_text(
        "path: " + dataset_root.as_posix() + "\n"
        "train: images/train\n"
        f"val: {'images/train' if use_train_as_val else 'images/val'}\n\n"
        "nc: 1\n"
        "names:\n"
        "  0: drone\n",
        encoding="utf-8",
    )
    return yaml_path


def prepare_dataset(dataset_root):
    for split in ("train", "val"):
        (dataset_root / "images" / split).mkdir(parents=True, exist_ok=True)
        (dataset_root / "labels" / split).mkdir(parents=True, exist_ok=True)

    train_samples = list_samples(dataset_root, "train")
    val_samples = list_samples(dataset_root, "val")
    if not train_samples and val_samples:
        move_sample(val_samples[0], dataset_root, "train")
    train_samples = list_samples(dataset_root, "train")
    val_samples = list_samples(dataset_root, "val")
    if not val_samples and len(train_samples) > 1:
        move_sample(train_samples[-1], dataset_root, "val")

    for split in ("train", "val"):
        for _, label_path in list_samples(dataset_root, split):
            ensure_label_file(label_path)

    train_count = len(list_samples(dataset_root, "train"))
    val_count = len(list_samples(dataset_root, "val"))
    if train_count <= 0:
        raise RuntimeError(f"dataset has no training images: {dataset_root}")

    yaml_path = write_data_yaml(dataset_root, use_train_as_val=(val_count <= 0))
    return yaml_path, train_count, val_count


def scan_split(dataset_root, split):
    stats = {
        "images": 0,
        "labeled_images": 0,
        "empty_labels": 0,
        "missing_labels": 0,
        "invalid_labels": 0,
        "boxes": 0,
        "min_area_ratio": 0.0,
        "max_area_ratio": 0.0,
        "mean_area_ratio": 0.0,
    }
    area_ratios = []
    for image_path, label_path in list_samples(dataset_root, split):
        stats["images"] += 1
        if not label_path.exists():
            stats["missing_labels"] += 1
            ensure_label_file(label_path)
            stats["empty_labels"] += 1
            continue
        text = label_path.read_text(encoding="utf-8").strip()
        if not text:
            stats["empty_labels"] += 1
            continue
        valid_image = False
        for raw_line in text.splitlines():
            parts = raw_line.strip().split()
            if len(parts) != 5:
                stats["invalid_labels"] += 1
                continue
            try:
                _, _, _, bw, bh = [float(item) for item in parts]
            except ValueError:
                stats["invalid_labels"] += 1
                continue
            if bw <= 0.0 or bh <= 0.0 or bw > 1.0 or bh > 1.0:
                stats["invalid_labels"] += 1
                continue
            valid_image = True
            stats["boxes"] += 1
            area_ratios.append(bw * bh)
        if valid_image:
            stats["labeled_images"] += 1
    if area_ratios:
        stats["min_area_ratio"] = min(area_ratios)
        stats["max_area_ratio"] = max(area_ratios)
        stats["mean_area_ratio"] = sum(area_ratios) / len(area_ratios)
    return stats


def scan_dataset(dataset_root):
    train_stats = scan_split(dataset_root, "train")
    val_stats = scan_split(dataset_root, "val")
    return {
        "train": train_stats,
        "val": val_stats,
        "total_boxes": int(train_stats["boxes"] + val_stats["boxes"]),
    }


def pick_model_path(weights_dir):
    preferred = weights_dir / "yolo26n.pt"
    if preferred.exists():
        return preferred
    candidates = sorted(weights_dir.glob("*.pt"))
    if not candidates:
        raise RuntimeError(f"no pretrained weights found in: {weights_dir}")
    return candidates[0]


def run():
    project_root = Path(__file__).resolve().parents[2]
    dataset_root = project_root / "PythonClient" / "YOLO" / "dataset"
    weights_dir = project_root / "PythonClient" / "YOLO" / "weights"
    project_dir = project_root / "PythonClient" / "YOLO" / "runs" / "detect"
    results_root = project_root / "PythonClient" / "YOLO" / "results"

    epochs = 100
    batch = 16
    imgsz = 640
    workers = 4
    device = None
    run_name = "collect_train"
    patience = 20

    yaml_path, train_count, val_count = prepare_dataset(dataset_root)
    dataset_stats = scan_dataset(dataset_root)
    if dataset_stats["train"]["boxes"] <= 0:
        raise RuntimeError("dataset has no positive training labels")
    if dataset_stats["train"]["invalid_labels"] > 0 or dataset_stats["val"]["invalid_labels"] > 0:
        raise RuntimeError(f"dataset has invalid labels: {dataset_stats}")

    batch = min(batch, max(2, train_count))
    model_path = pick_model_path(weights_dir)

    project_dir.mkdir(parents=True, exist_ok=True)
    results_root.mkdir(parents=True, exist_ok=True)
    ensure_ultralytics_import_path(project_root)

    from ultralytics import YOLO

    model = YOLO(str(model_path))
    result = model.train(
        data=str(yaml_path),
        epochs=epochs,
        batch=batch,
        imgsz=imgsz,
        device=device,
        workers=workers,
        project=str(project_dir),
        name=run_name,
        exist_ok=True,
        verbose=True,
        patience=patience,
        cos_lr=True,
        close_mosaic=10,
        degrees=0.0,
        translate=0.06,
        scale=0.35,
        fliplr=0.5,
        mosaic=0.20,
        mixup=0.0,
        copy_paste=0.0,
    )

    save_dir = Path(str(getattr(result, "save_dir", project_dir / run_name)))
    weights_out = save_dir / "weights"
    best_weights = weights_out / "best.pt"
    last_weights = weights_out / "last.pt"
    summary = {
        "dataset_root": str(dataset_root),
        "data_yaml": str(yaml_path),
        "dataset_stats": dataset_stats,
        "train_images": train_count,
        "val_images": val_count,
        "model": str(model_path),
        "epochs": epochs,
        "batch": batch,
        "imgsz": imgsz,
        "save_dir": str(save_dir),
        "best_weights": str(best_weights) if best_weights.exists() else "",
        "last_weights": str(last_weights) if last_weights.exists() else "",
    }
    summary_path = results_root / f"{run_name}_summary.json"
    summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    return summary


if __name__ == "__main__":
    print(json.dumps(run(), ensure_ascii=False, indent=2))

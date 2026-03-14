from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Tuple


IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}


def _resolve_path(root: Path, path_text: str) -> Path:
    path = Path(path_text)
    if not path.is_absolute():
        path = root / path
    return path.resolve()


def _ensure_ultralytics_import_path(project_root: Path) -> None:
    source_root = project_root / "PythonClient" / "YOLO" / "ultralytics"
    if not source_root.exists():
        raise RuntimeError(f"local ultralytics source not found: {source_root}")
    settings_root = project_root / "PythonClient" / ".ultralytics_config"
    settings_root.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("YOLO_CONFIG_DIR", str(settings_root))
    os.environ.setdefault("YOLO_AUTOINSTALL", "False")
    os.environ.setdefault("YOLO_VERBOSE", "False")
    source_text = str(source_root)
    if source_text not in sys.path:
        sys.path.insert(0, source_text)


def _write_data_yaml(dataset_root: Path, use_train_as_val: bool = False) -> Path:
    yaml_path = dataset_root / "data.yaml"
    yaml_text = (
        f"path: {dataset_root.as_posix()}\n"
        "train: images/train\n"
        f"val: {'images/train' if use_train_as_val else 'images/val'}\n\n"
        "nc: 1\n"
        "names:\n"
        "  0: drone\n"
    )
    yaml_path.write_text(yaml_text, encoding="utf-8")
    return yaml_path


def _list_samples(dataset_root: Path, split: str) -> List[Tuple[Path, Path]]:
    images_dir = dataset_root / "images" / split
    labels_dir = dataset_root / "labels" / split
    samples: List[Tuple[Path, Path]] = []
    if not images_dir.exists():
        return samples
    for image_path in sorted(images_dir.iterdir()):
        if not image_path.is_file() or image_path.suffix.lower() not in IMAGE_EXTS:
            continue
        label_path = labels_dir / f"{image_path.stem}.txt"
        samples.append((image_path, label_path))
    return samples


def _move_sample(sample: Tuple[Path, Path], dst_images_dir: Path, dst_labels_dir: Path) -> Dict[str, str]:
    image_path, label_path = sample
    dst_image = dst_images_dir / image_path.name
    dst_label = dst_labels_dir / label_path.name
    dst_images_dir.mkdir(parents=True, exist_ok=True)
    dst_labels_dir.mkdir(parents=True, exist_ok=True)
    image_path.replace(dst_image)
    if label_path.exists():
        label_path.replace(dst_label)
    else:
        dst_label.write_text("", encoding="utf-8")
    return {
        "image": str(dst_image),
        "label": str(dst_label),
    }


def _rebalance_dataset(dataset_root: Path) -> Dict[str, object]:
    moves: List[Dict[str, str]] = []
    train_samples = _list_samples(dataset_root, "train")
    val_samples = _list_samples(dataset_root, "val")

    if not train_samples and len(val_samples) > 1:
        moves.append(
            _move_sample(
                val_samples[0],
                dataset_root / "images" / "train",
                dataset_root / "labels" / "train",
            )
        )
    train_samples = _list_samples(dataset_root, "train")
    val_samples = _list_samples(dataset_root, "val")
    if not val_samples and len(train_samples) > 1:
        moves.append(
            _move_sample(
                train_samples[-1],
                dataset_root / "images" / "val",
                dataset_root / "labels" / "val",
            )
        )

    return {
        "train_images": len(_list_samples(dataset_root, "train")),
        "val_images": len(_list_samples(dataset_root, "val")),
        "moves": moves,
    }


def prepare_dataset(dataset_root: Path) -> Dict[str, object]:
    for split in ("train", "val"):
        (dataset_root / "images" / split).mkdir(parents=True, exist_ok=True)
        (dataset_root / "labels" / split).mkdir(parents=True, exist_ok=True)

    rebalance = _rebalance_dataset(dataset_root)
    train_images = int(rebalance["train_images"])
    val_images = int(rebalance["val_images"])
    if train_images <= 0:
        raise RuntimeError(f"dataset has no training images: {dataset_root}")

    use_train_as_val = val_images <= 0
    yaml_path = _write_data_yaml(dataset_root, use_train_as_val=use_train_as_val)
    return {
        "dataset_root": str(dataset_root),
        "data_yaml": str(yaml_path),
        "train_images": train_images,
        "val_images": val_images,
        "val_source": "train" if use_train_as_val else "val",
        "rebalance_moves": rebalance["moves"],
    }


def run_training(args: argparse.Namespace, dataset_yaml: Path) -> Dict[str, object]:
    project_root = Path(__file__).resolve().parents[2]
    _ensure_ultralytics_import_path(project_root)
    from ultralytics import YOLO

    model_path = _resolve_path(project_root, args.model)
    project_dir = _resolve_path(project_root, args.project)
    results_root = _resolve_path(project_root, args.results_root)
    project_dir.mkdir(parents=True, exist_ok=True)
    results_root.mkdir(parents=True, exist_ok=True)

    run_name = args.name.strip() if args.name else f"collect_train_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    model = YOLO(str(model_path))
    result = model.train(
        data=str(dataset_yaml),
        epochs=args.epochs,
        batch=args.batch,
        imgsz=args.imgsz,
        device=None if args.device in (None, "", "auto", "none") else args.device,
        workers=args.workers,
        project=str(project_dir),
        name=run_name,
        resume=args.resume,
        exist_ok=True,
        verbose=True,
    )

    save_dir_value = getattr(result, "save_dir", None)
    save_dir = Path(str(save_dir_value)) if save_dir_value else None
    copied_results_csv = ""
    if save_dir is not None and save_dir.exists():
        results_csv = save_dir / "results.csv"
        if results_csv.exists():
            results_copy = results_root / f"{run_name}_results.csv"
            shutil.copy2(results_csv, results_copy)
            copied_results_csv = str(results_copy)

    summary = {
        "run_name": run_name,
        "model": str(model_path),
        "dataset_yaml": str(dataset_yaml),
        "save_dir": str(save_dir) if save_dir is not None else "",
        "results_csv": copied_results_csv,
    }
    summary_path = results_root / f"{run_name}_summary.json"
    summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    summary["summary_json"] = str(summary_path)
    return summary


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Train YOLO directly from PythonClient/YOLO/dataset")
    parser.add_argument("--dataset-root", default="PythonClient/YOLO/dataset")
    parser.add_argument("--model", default="PythonClient/YOLO/weights/yolo26n.pt")
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--project", default="PythonClient/YOLO/runs/detect")
    parser.add_argument("--results-root", default="PythonClient/YOLO/results")
    parser.add_argument("--name", default="collect_train")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--no-train", dest="train", action="store_false")
    parser.set_defaults(train=True)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    project_root = Path(__file__).resolve().parents[2]
    try:
        dataset_root = _resolve_path(project_root, args.dataset_root)
        summary = prepare_dataset(dataset_root)
        if args.train:
            summary["train"] = run_training(args, Path(summary["data_yaml"]))
        print(json.dumps(summary, ensure_ascii=False, indent=2))
        return 0
    except Exception as exc:
        print(json.dumps({"exit_reason": f"error:{exc}"}, ensure_ascii=False, indent=2))
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

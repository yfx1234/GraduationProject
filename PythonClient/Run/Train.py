from __future__ import annotations

import argparse
import json
import os
import random
import shutil
import sys
from pathlib import Path


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


def _iter_session_dirs(paths, project_root: Path):
    for path_text in paths:
        root = _resolve_path(project_root, path_text)
        if root.is_dir() and (root / "images").is_dir() and (root / "labels").is_dir():
            yield root
            continue
        if not root.is_dir():
            continue
        for child in sorted(root.iterdir()):
            if child.is_dir() and (child / "images").is_dir() and (child / "labels").is_dir():
                yield child.resolve()


def _collect_samples(session_dir: Path):
    images_dir = session_dir / "images"
    labels_dir = session_dir / "labels"
    samples = []
    for image_path in sorted(images_dir.iterdir()):
        if image_path.suffix.lower() not in IMAGE_EXTS:
            continue
        label_path = labels_dir / f"{image_path.stem}.txt"
        if not label_path.exists():
            continue
        samples.append((image_path.resolve(), label_path.resolve(), session_dir.name))
    return samples


def _write_dataset_yaml(dataset_dir: Path) -> Path:
    yaml_path = dataset_dir / "dataset.yaml"
    yaml_text = (
        f"path: {dataset_dir.as_posix()}\n"
        "train: images/train\n"
        "val: images/val\n"
        "names:\n"
        "  0: drone\n"
    )
    yaml_path.write_text(yaml_text, encoding="utf-8")
    return yaml_path


def _copy_samples(samples, dataset_dir: Path, val_ratio: float, seed: int):
    rng = random.Random(seed)
    ordered = list(samples)
    rng.shuffle(ordered)
    val_count = int(round(len(ordered) * val_ratio))
    val_count = max(1, val_count) if len(ordered) > 1 and val_ratio > 0.0 else 0
    val_set = set(range(val_count))

    for split in ("train", "val"):
        (dataset_dir / "images" / split).mkdir(parents=True, exist_ok=True)
        (dataset_dir / "labels" / split).mkdir(parents=True, exist_ok=True)

    manifest = []
    for index, (image_path, label_path, session_name) in enumerate(ordered):
        split = "val" if index in val_set else "train"
        stem = f"{session_name}_{image_path.stem}"
        dst_image = dataset_dir / "images" / split / f"{stem}{image_path.suffix.lower()}"
        dst_label = dataset_dir / "labels" / split / f"{stem}.txt"
        shutil.copy2(image_path, dst_image)
        shutil.copy2(label_path, dst_label)
        manifest.append({
            "split": split,
            "source_image": str(image_path),
            "source_label": str(label_path),
            "image": str(dst_image),
            "label": str(dst_label),
        })
    return manifest


def build_dataset(args) -> tuple[Path, Path, list]:
    project_root = Path(__file__).resolve().parents[2]
    session_dirs = list(dict.fromkeys(_iter_session_dirs(args.inputs, project_root)))
    if not session_dirs:
        raise RuntimeError("no collection sessions found")

    samples = []
    for session_dir in session_dirs:
        samples.extend(_collect_samples(session_dir))
    if not samples:
        raise RuntimeError("no labeled samples found in the selected sessions")

    dataset_root = _resolve_path(project_root, args.dataset_root)
    dataset_name = args.dataset_name.strip() if args.dataset_name else f"dataset_{Path(session_dirs[0]).name}"
    dataset_dir = dataset_root / dataset_name
    if dataset_dir.exists():
        if not args.force_rebuild:
            raise RuntimeError(f"dataset directory already exists: {dataset_dir}")
        shutil.rmtree(dataset_dir)
    dataset_dir.mkdir(parents=True, exist_ok=True)

    manifest = _copy_samples(samples, dataset_dir, val_ratio=args.val_ratio, seed=args.seed)
    yaml_path = _write_dataset_yaml(dataset_dir)
    (dataset_dir / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    return dataset_dir, yaml_path, manifest


def run_training(args, dataset_yaml: Path) -> dict:
    project_root = Path(__file__).resolve().parents[2]
    _ensure_ultralytics_import_path(project_root)
    from ultralytics import YOLO

    model_path = _resolve_path(project_root, args.model)
    model = YOLO(str(model_path))
    project_dir = _resolve_path(project_root, args.project)
    result = model.train(
        data=str(dataset_yaml),
        epochs=args.epochs,
        batch=args.batch,
        imgsz=args.imgsz,
        device=None if args.device in (None, "", "auto") else args.device,
        workers=args.workers,
        project=str(project_dir),
        name=args.name,
        resume=args.resume,
        exist_ok=True,
        verbose=True,
    )
    save_dir = getattr(result, "save_dir", None)
    return {"save_dir": str(save_dir) if save_dir else "", "model": str(model_path), "dataset_yaml": str(dataset_yaml)}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Build a YOLO dataset from collection sessions and optionally launch training")
    parser.add_argument("inputs", nargs="*", default=["PythonClient/Run/datasets/raw"], help="Session directories or roots that contain multiple sessions")
    parser.add_argument("--dataset-root", default="PythonClient/Run/datasets/yolo")
    parser.add_argument("--dataset-name", default="")
    parser.add_argument("--val-ratio", type=float, default=0.1)
    parser.add_argument("--seed", type=int, default=1234)
    parser.add_argument("--force-rebuild", action="store_true")

    parser.add_argument("--model", default="PythonClient/YOLO/weights/yolo26n.pt")
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--project", default="PythonClient/YOLO/runs/detect")
    parser.add_argument("--name", default="collect_train")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--no-train", dest="train", action="store_false")
    parser.set_defaults(train=True)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    dataset_dir, yaml_path, manifest = build_dataset(args)
    summary = {
        "dataset_dir": str(dataset_dir),
        "dataset_yaml": str(yaml_path),
        "num_samples": len(manifest),
        "train_samples": sum(1 for item in manifest if item["split"] == "train"),
        "val_samples": sum(1 for item in manifest if item["split"] == "val"),
    }
    if args.train:
        summary["train"] = run_training(args, yaml_path)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

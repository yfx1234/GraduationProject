from __future__ import annotations

import json
from pathlib import Path

from params import train as trainConfig, yolo as yoloConfig
from GraduationSIM import (
    ensure_ultralytics_import_path,
    next_run_name,
    pick_training_model_path,
    prepare_dataset,
    resolve_paths,
    scan_dataset,
)

TRAIN_IMAGE_SIZE = 640
TRAIN_WORKERS = 4
TRAIN_DEVICE = None
TRAIN_PATIENCE = 20
TRAIN_COSINE_LR = True
TRAIN_CLOSE_MOSAIC = 10
TRAIN_DEGREES = 0.0
TRAIN_TRANSLATE = 0.06
TRAIN_SCALE = 0.35
TRAIN_HORIZONTAL_FLIP = 0.5
TRAIN_MOSAIC = 0.20
TRAIN_MIXUP = 0.0
TRAIN_COPY_PASTE = 0.0
TRAIN_RESUME = False


def install_progress_callbacks(model) -> None:
    progressState = {"batch": 0, "next": 0.25}

    def on_train_start(trainer):
        epochs = int(getattr(trainer, "epochs", getattr(trainer.args, "epochs", 0)))
        batchCount = len(trainer.train_loader) if getattr(trainer, "train_loader", None) is not None else 0
        print(f"[train] start epochs={epochs} batches_per_epoch={batchCount}", flush=True)

    def on_train_epoch_start(trainer):
        progressState["batch"] = 0
        progressState["next"] = 0.25
        print(f"[train] epoch {int(trainer.epoch) + 1}/{int(trainer.epochs)} started", flush=True)

    def on_train_batch_end(trainer):
        totalBatches = len(trainer.train_loader) if getattr(trainer, "train_loader", None) is not None else 0
        if totalBatches <= 0:
            return
        progressState["batch"] += 1
        progress = float(progressState["batch"]) / float(totalBatches)
        if progressState["batch"] == totalBatches or progress + 1e-9 >= progressState["next"]:
            lossText = ""
            try:
                lossText = f" loss={float(trainer.loss.detach().cpu().item()):.4f}"
            except Exception:
                pass
            print(
                f"[train] epoch {int(trainer.epoch) + 1}/{int(trainer.epochs)} batch {progressState['batch']}/{totalBatches}{lossText}",
                flush=True,
            )
            progressState["next"] += 0.25

    def on_fit_epoch_end(trainer):
        parts = []
        try:
            lossItems = trainer.label_loss_items(trainer.tloss)
            if isinstance(lossItems, dict):
                for key, value in lossItems.items():
                    parts.append(f"{key.split('/')[-1]}={float(value):.4f}")
        except Exception:
            pass
        metrics = getattr(trainer, "metrics", {}) or {}
        if isinstance(metrics, dict):
            for key in sorted(metrics):
                lowered = key.lower()
                if any(token in lowered for token in ("map50-95", "map50", "precision", "recall", "fitness")):
                    value = metrics.get(key)
                    if isinstance(value, (int, float)):
                        parts.append(f"{key.split('/')[-1]}={float(value):.4f}")
        print(f"[train] epoch {int(trainer.epoch) + 1}/{int(trainer.epochs)} finished: {', '.join(parts) if parts else 'no metrics'}", flush=True)

    model.add_callback("on_train_start", on_train_start)
    model.add_callback("on_train_epoch_start", on_train_epoch_start)
    model.add_callback("on_train_batch_end", on_train_batch_end)
    model.add_callback("on_fit_epoch_end", on_fit_epoch_end)


def run():
    paths = resolve_paths(__file__)
    datasetRoot = paths.dataset_root
    resultsRoot = paths.results_root
    projectDir = paths.runs_root
    epochs = int(trainConfig.get("epochs", 100) or 100)
    batch = int(trainConfig.get("batch", 16) or 16)
    runName = next_run_name(paths)

    dataYamlPath, trainCount, valCount = prepare_dataset(datasetRoot)
    datasetStats = scan_dataset(datasetRoot)
    if datasetStats["train"]["boxes"] <= 0:
        raise RuntimeError("dataset has no positive training labels")
    if datasetStats["train"]["invalid_labels"] > 0 or datasetStats["val"]["invalid_labels"] > 0:
        raise RuntimeError(f"dataset has invalid labels: {datasetStats}")

    batch = min(batch, max(1, trainCount))
    modelPath = pick_training_model_path(paths, configured_model=yoloConfig.get("baseModel"))
    projectDir.mkdir(parents=True, exist_ok=True)
    resultsRoot.mkdir(parents=True, exist_ok=True)
    ensure_ultralytics_import_path(paths.project_root)

    from ultralytics import YOLO

    print(f"[train] base weights: {modelPath}", flush=True)
    print(f"[train] dataset train_images={trainCount} val_images={valCount} total_boxes={datasetStats['total_boxes']}", flush=True)

    model = YOLO(str(modelPath))
    install_progress_callbacks(model)
    result = model.train(
        data=str(dataYamlPath),
        epochs=epochs,
        batch=batch,
        imgsz=TRAIN_IMAGE_SIZE,
        device=TRAIN_DEVICE,
        workers=TRAIN_WORKERS,
        project=str(projectDir),
        name=runName,
        exist_ok=False,
        verbose=True,
        patience=TRAIN_PATIENCE,
        cos_lr=TRAIN_COSINE_LR,
        close_mosaic=TRAIN_CLOSE_MOSAIC,
        degrees=TRAIN_DEGREES,
        translate=TRAIN_TRANSLATE,
        scale=TRAIN_SCALE,
        fliplr=TRAIN_HORIZONTAL_FLIP,
        mosaic=TRAIN_MOSAIC,
        mixup=TRAIN_MIXUP,
        copy_paste=TRAIN_COPY_PASTE,
        resume=TRAIN_RESUME,
    )

    saveDir = Path(str(getattr(result, "save_dir", projectDir / runName)))
    weightsDir = saveDir / "weights"
    bestWeights = weightsDir / "best.pt"
    lastWeights = weightsDir / "last.pt"
    summary = {
        "dataset_root": str(datasetRoot),
        "data_yaml": str(dataYamlPath),
        "dataset_stats": datasetStats,
        "train_images": trainCount,
        "val_images": valCount,
        "model": str(modelPath),
        "epochs": epochs,
        "batch": batch,
        "imgsz": TRAIN_IMAGE_SIZE,
        "run_name": runName,
        "save_dir": str(saveDir),
        "best_weights": str(bestWeights) if bestWeights.exists() else "",
        "last_weights": str(lastWeights) if lastWeights.exists() else "",
    }
    (resultsRoot / f"{runName}_summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    return summary


if __name__ == "__main__":
    print(json.dumps(run(), ensure_ascii=False, indent=2))



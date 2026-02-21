"""
YOLO26 推理服务
从 UE 采集图像 → YOLO26 检测 → 返回检测结果

使用 ultralytics-main (YOLO26) 进行推理
模型文件: ../Yolo/yolo26n.pt 或自定义训练的 yolo26n-objv1-150.pt
"""

import sys, os, time, json

# 将 ultralytics-main 加入路径
YOLO_PATH = os.path.join(os.path.dirname(__file__), "..", "Yolo", "ultralytics-main")
if os.path.exists(YOLO_PATH):
    sys.path.insert(0, YOLO_PATH)

from ultralytics import YOLO
import numpy as np
from sim_client import SimClient


def run_yolo_service(model_path: str = None,
                     host: str = "127.0.0.1",
                     port: int = 9000,
                     conf: float = 0.5,
                     interval: float = 0.2):
    """
    YOLO 推理服务主循环
    1. 从 UE 获取图像
    2. YOLO 检测
    3. 将检测结果发送给制导系统
    """

    # 默认模型路径
    if model_path is None:
        model_path = os.path.join(os.path.dirname(__file__), "..", "Yolo", "yolo26n.pt")

    if not os.path.exists(model_path):
        print(f"[ERROR] Model not found: {model_path}")
        print("Available models:")
        yolo_dir = os.path.join(os.path.dirname(__file__), "..", "Yolo")
        for f in os.listdir(yolo_dir):
            if f.endswith(".pt"):
                print(f"  - {f}")
        return

    print(f"[YOLO] Loading model: {model_path}")
    model = YOLO(model_path)

    client = SimClient(host, port)

    print(f"[YOLO] Service started, interval={interval}s, conf={conf}")

    try:
        while True:
            # 采集图像
            img = client.get_image_numpy()
            if img is None:
                time.sleep(interval)
                continue

            # YOLO 推理
            results = model(img, conf=conf, verbose=False)

            if len(results) > 0 and len(results[0].boxes) > 0:
                # 取置信度最高的检测框
                boxes = results[0].boxes
                best_idx = boxes.conf.argmax()
                box = boxes.xyxy[best_idx].cpu().numpy()
                conf_val = float(boxes.conf[best_idx])
                cls = int(boxes.cls[best_idx])

                # 检测框中心
                cx = (box[0] + box[2]) / 2
                cy = (box[1] + box[3]) / 2
                w = box[2] - box[0]
                h = box[3] - box[1]

                print(f"[YOLO] Detected cls={cls} conf={conf_val:.2f} "
                      f"center=({cx:.0f},{cy:.0f}) size=({w:.0f}x{h:.0f})")

                # 将检测结果喂给制导系统（通过 update_target 使用像素坐标估算位置）
                # 注意：这里的坐标需要和实际 3D 位置对应，简单场景可用像素比例估算
                # 实际应用需要 VisionCoordinate 做投影转换

            else:
                pass  # 无检测结果

            time.sleep(interval)

    except KeyboardInterrupt:
        print("\n[YOLO] Service stopped")
    finally:
        client.close()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="YOLO26 推理服务")
    parser.add_argument("--model", default=None, help="模型路径 (.pt)")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9000)
    parser.add_argument("--conf", type=float, default=0.5, help="置信度阈值")
    parser.add_argument("--interval", type=float, default=0.2, help="采集间隔(秒)")
    args = parser.parse_args()

    run_yolo_service(
        model_path=args.model,
        host=args.host, port=args.port,
        conf=args.conf, interval=args.interval,
    )

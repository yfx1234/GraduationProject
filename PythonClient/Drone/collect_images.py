"""
collect_images.py — 无人机摄像头图像采集

控制无人机随机飞行，从无人机机载摄像头拍照并保存到本地。
纯采集，不做标注。

输出:
  drone_images/
    00000.jpg
    00001.jpg
    ...

使用:
  1. 启动 UE PIE
  2. python collect_images.py --num 200
"""

import sys, os, time, math, argparse
import base64
import numpy as np
import cv2

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
# 复用 Turret 目录下的 sim_client
TURRET_DIR = os.path.join(os.path.dirname(SCRIPT_DIR), "Turret")
sys.path.insert(0, TURRET_DIR)
from sim_client import SimClient


def random_position(center, radius):
    """在 center 周围 radius 范围内生成随机位置"""
    angle = np.random.uniform(0, 2 * math.pi)
    dist = np.random.uniform(0.2, 1.0) * radius
    x = center[0] + dist * math.cos(angle)
    y = center[1] + dist * math.sin(angle)
    return x, y


def collect(args):
    client = SimClient(args.host, args.port)

    # 输出目录
    out_dir = args.output
    os.makedirs(out_dir, exist_ok=True)

    # 获取无人机初始位置作为中心
    state = client.drone_state(args.drone_id)
    init_pos = state.get("position", [0, 0, 0])
    center_x, center_y = init_pos[0], init_pos[1]
    print(f"[Drone] 初始位置: ({center_x:.1f}, {center_y:.1f})")

    # 起飞
    print(f"[Drone] 起飞 altitude={args.altitude}m...")
    client.drone_takeoff(args.altitude)
    time.sleep(4)

    # 随机摄像头角度范围
    pitch_range = (-30, 30)
    yaw_range = (-90, 90)
    altitude_range = (max(2.0, args.altitude - 5), args.altitude + 10)

    saved = 0
    step = 0

    while saved < args.num:
        # 随机移动无人机
        rx, ry = random_position([center_x, center_y], args.radius)
        alt = np.random.uniform(*altitude_range)
        speed = np.random.uniform(2.0, 8.0)

        print(f"  [step {step}] 飞到 ({rx:.1f}, {ry:.1f}, {alt:.1f}) speed={speed:.1f}")
        client.drone_move_to(rx, ry, alt, speed)
        time.sleep(np.random.uniform(2.0, 4.0))

        # 随机调整摄像头角度
        cam_pitch = np.random.uniform(*pitch_range)
        cam_yaw = np.random.uniform(*yaw_range)
        client.drone_set_camera_angles(cam_pitch, cam_yaw, args.drone_id)
        time.sleep(0.5)  # 等待云台旋转到位

        # 拍照
        print(f"  [step {step}] 拍照 (cam pitch={cam_pitch:.1f}, yaw={cam_yaw:.1f})...")
        img = client.get_image_numpy(args.drone_id)
        if img is None:
            print(f"  [step {step}] ✗ 获取图像失败")
            step += 1
            continue

        # 保存
        name = f"{saved:05d}.jpg"
        filepath = os.path.join(out_dir, name)
        cv2.imwrite(filepath, img)
        saved += 1
        print(f"  [step {step}] ✓ 保存 #{saved}/{args.num} -> {name}")

        step += 1

    # 清理
    client.drone_hover()
    client.close()
    print(f"\n[完成] {saved} 张图像保存到 {out_dir}/")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="无人机摄像头图像采集")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9000)
    parser.add_argument("--num", type=int, default=200, help="采集数量")
    parser.add_argument("--output", default=os.path.join(SCRIPT_DIR, "drone_images"), help="输出目录")
    parser.add_argument("--altitude", type=float, default=5.0, help="基准飞行高度(m)")
    parser.add_argument("--radius", type=float, default=30.0, help="飞行半径(m)")
    parser.add_argument("--drone-id", default="drone_0", help="无人机 Agent ID")
    args = parser.parse_args()
    collect(args)

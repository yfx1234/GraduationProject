"""
collect_data.py — 自动采集 YOLO 训练数据

从 UE 转台摄像头采集图像，同时获取无人机 3D 坐标，
利用相机参数将 3D 投影到 2D，自动生成 YOLO 标注文件。

**完全不需要手动标注！**

输出:
  dataset/
    images/train/  — JPEG 图像
    labels/train/  — YOLO 格式标签 (class cx cy w h)

使用:
  1. 启动 UE PIE
  2. python collect_data.py --num 500
"""

import sys, os, time, math, argparse
import base64
import numpy as np
import cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sim_client import SimClient


def project_3d_to_2d(point_3d, cam_pos, cam_rot, fov, img_w, img_h):
    """
    将 3D 世界坐标投影到 2D 像素坐标

    cam_rot: [pitch, yaw, roll] degrees
    返回: (u, v) 像素坐标，如果在视野外返回 None
    """
    # 相对坐标
    rel = np.array(point_3d) - np.array(cam_pos)

    # 旋转矩阵（世界→相机）
    p, y, r = [math.radians(x) for x in cam_rot]
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    cr, sr = math.cos(r), math.sin(r)

    fwd = np.array([cp * cy, cp * sy, sp])
    right = np.array([cy * sp * sr - sy * cr, sy * sp * sr + cy * cr, -cp * sr])
    up = np.array([-(cy * sp * cr + sy * sr), -(sy * sp * cr - cy * sr), cp * cr])

    # 投影到相机空间
    x_cam = np.dot(rel, fwd)    # 深度
    y_cam = np.dot(rel, right)  # 右
    z_cam = np.dot(rel, up)     # 上

    if x_cam <= 0:
        return None  # 在相机后方

    # 焦距
    fx = (img_w / 2.0) / math.tan(math.radians(fov / 2.0))

    # 像素坐标
    u = img_w / 2.0 + (y_cam / x_cam) * fx
    v = img_h / 2.0 - (z_cam / x_cam) * fx

    # 检查是否在图像内
    if 0 <= u < img_w and 0 <= v < img_h:
        return (u, v, x_cam)  # 返回深度用于计算 bbox 大小
    return None


def estimate_bbox_size(depth_cm, fov, img_w, img_h, drone_size_cm=120):
    """
    根据深度估算无人机在图像中的 bbox 大小

    drone_size_cm: 无人机大致尺寸 (cm)，默认 120cm
    """
    fx = (img_w / 2.0) / math.tan(math.radians(fov / 2.0))
    pixel_size = (drone_size_cm / depth_cm) * fx
    # bbox 宽高（无人机大致长方形，宽 > 高）
    w = pixel_size * 1.2
    h = pixel_size * 0.7
    return w, h


def save_yolo_label(filepath, class_id, cx, cy, w, h, img_w, img_h):
    """保存 YOLO 格式标签（归一化坐标）"""
    cx_n = cx / img_w
    cy_n = cy / img_h
    w_n = min(w / img_w, 1.0)
    h_n = min(h / img_h, 1.0)
    with open(filepath, "w") as f:
        f.write(f"0 {cx_n:.6f} {cy_n:.6f} {w_n:.6f} {h_n:.6f}\n")


def collect(args):
    client = SimClient(args.host, args.port)

    # 输出目录
    img_dir = os.path.join(args.output, "images", "train")
    lbl_dir = os.path.join(args.output, "labels", "train")
    os.makedirs(img_dir, exist_ok=True)
    os.makedirs(lbl_dir, exist_ok=True)

    # 获取转台位置
    ts = client.turret_state()
    turret_pos = ts.get("position", [0, 0, 0])
    tx, ty = turret_pos[0], turret_pos[1]
    R = args.radius * 100  # m → cm
    print(f"[Turret] pos=({tx:.0f}, {ty:.0f})")

    # 起飞
    print(f"[Drone] 起飞 altitude={args.altitude}m...")
    client.drone_takeoff(args.altitude)
    time.sleep(4)

    # 飞到转台附近
    client.drone_move_to(tx / 100 + args.radius * 0.3, ty / 100, args.altitude, 3.0)
    time.sleep(3)

    # 开启跟踪（让转台跟着无人机转）
    client.turret_start_tracking("drone_0")

    saved = 0
    step = 0
    # 修改距离和高度分布：主要向四周横向展开，高度不再集中在转台正上方
    # R 默认改为 100m
    altitudes = [2.0, 5.0, 8.0, 15.0, 20.0, 30.0] 
    distances = [0.2, 0.4, 0.7, 1.0, 1.5, 2.0, 2.5] # 20m ~ 250m 大范围

    empty_images_count = 0  # 纯负样本计数

    while saved < args.num:
        # 10% 的概率收集没有无人机的纯负样本（抑制假阳性）
        collect_empty = (np.random.random() < 0.1)

        # 变换无人机位置
        alt = np.random.choice(altitudes) + np.random.uniform(-2.0, 5.0)
        dist_ratio = np.random.choice(distances)
        
        # 随机偏航角，360度全方位
        angle = np.random.uniform(0, 360)
        rad = math.radians(angle)

        target_x = tx + R * dist_ratio * math.cos(rad)
        target_y = ty + R * dist_ratio * math.sin(rad)
        
        if collect_empty:
            # 如果收集空图片，就把无人机“藏起”或者飞到极高极远的地方
            client.drone_move_to(tx / 100 + 400.0, ty / 100 + 400.0, 500.0, 20.0)
            time.sleep(1.0)
            # 转台随便转个角度
            client.turret_set_angles(np.random.uniform(-10, 30), np.random.uniform(-180, 180))
        else:
            print(f"  [step {step}] 移动无人机 -> ({target_x/100:.1f}, {target_y/100:.1f}, {alt:.1f})")
            move_speed = np.random.uniform(4.0, 12.0)
            client.drone_move_to(target_x / 100, target_y / 100, alt, move_speed)
            time.sleep(np.random.uniform(1.5, 3.0)) 

        # 随机开火制造干扰 (子弹轨迹、枪口火光)
        if np.random.random() < 0.4:  # 40% 概率开火
            client.turret_fire(400.0)
            time.sleep(0.1) # 让子弹飞一会儿入画

        # 获取图像 + 相机参数
        print(f"  [step {step}] 请求图像...")
        resp = client._send({"get_image": {}})
        if resp.get("status") != "ok":
            print(f"  [step {step}] ✗ get_image 失败: {resp}")
            step += 1
            continue

        # 获取无人机位置
        drone_state = client.drone_state()
        drone_pos = drone_state.get("position", None)
        if not drone_pos:
            print(f"  [step {step}] ✗ 获取无人机位置失败: {drone_state}")
            step += 1
            continue

        # 解码图像
        b64 = resp.get("data", "")
        if not b64:
            print(f"  [step {step}] ✗ 图像 data 为空")
            step += 1
            continue

        img_data = base64.b64decode(b64)
        img = cv2.imdecode(np.frombuffer(img_data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            print(f"  [step {step}] ✗ cv2.imdecode 失败 (数据长度={len(img_data)})")
            step += 1
            continue

        img_w = resp.get("width", 640)
        img_h = resp.get("height", 480)
        cam_pos = resp.get("camera_pos", [0, 0, 0])
        cam_rot = resp.get("camera_rot", [0, 0, 0])
        fov = resp.get("fov", 90.0)
        print(f"  [step {step}] 图像 {img_w}x{img_h}, cam_pos={cam_pos}, cam_rot={cam_rot}, fov={fov}")

        # 处理纯负样本
        if collect_empty:
            name = f"{saved:05d}"
            img_path = os.path.join(img_dir, f"{name}.jpg")
            lbl_path = os.path.join(lbl_dir, f"{name}.txt")
            cv2.imwrite(img_path, img)
            # 生成空标签文件（表示这图里没有目标）
            open(lbl_path, 'w').close()
            saved += 1
            empty_images_count += 1
            print(f"  [step {step}] ✓ 保存 #{saved}/{args.num} [背景/干扰图]")
            step += 1
            
            # 恢复转台跟踪
            client.turret_start_tracking("drone_0")
            time.sleep(0.5)
            continue

        # 正常样本处理：3D → 2D 投影
        drone_pos_cm = [p * 100 for p in drone_pos]
        result = project_3d_to_2d(drone_pos_cm, cam_pos, cam_rot, fov, img_w, img_h)
        if result is None:
            # 有时目标太快飞出视野外，也可以作为纯负样本
            step += 1
            continue

        u, v, depth = result
        if depth < 30:  # 太近跳过
            print(f"  [step {step}] ✗ 太近 depth={depth:.0f}cm")
            step += 1
            continue

        # 估算 bbox
        bw, bh = estimate_bbox_size(depth, fov, img_w, img_h, args.drone_size)

        # 检查 bbox 合理性
        if bw < 5 or bh < 3 or bw > img_w * 0.8 or bh > img_h * 0.8:
            print(f"  [step {step}] ✗ bbox 不合理 ({bw:.0f}x{bh:.0f}) depth={depth:.0f}cm")
            step += 1
            continue

        # 保存图像及正常标签
        name = f"{saved:05d}"
        img_path = os.path.join(img_dir, f"{name}.jpg")
        lbl_path = os.path.join(lbl_dir, f"{name}.txt")

        cv2.imwrite(img_path, img)
        save_yolo_label(lbl_path, 0, u, v, bw, bh, img_w, img_h)

        saved += 1
        print(f"  [step {step}] ✓ 保存 #{saved}/{args.num} depth={depth:.0f}cm bbox=({bw:.0f}x{bh:.0f}) uv=({u:.0f},{v:.0f})")

        step += 1

    # 清理
    client.turret_stop_tracking()
    client.drone_hover()
    client.close()

    print(f"\n[完成] {saved} 张图像保存到 {args.output}/")
    print(f"  其中纯负样本(背景/干扰): {empty_images_count} 张")
    print(f"  images/train/: {saved} 张 JPEG")
    print(f"  labels/train/: {saved} 个 YOLO 标签")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="自动采集 YOLO 训练数据")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9000)
    parser.add_argument("--num", type=int, default=500, help="采集数量")
    parser.add_argument("--output", default="dataset", help="输出目录")
    parser.add_argument("--altitude", type=float, default=5.0)
    parser.add_argument("--radius", type=float, default=100.0, help="基准飞行半径(m)")
    parser.add_argument("--drone-size", type=float, default=120.0, help="无人机尺寸(cm)")
    args = parser.parse_args()
    collect(args)

"""
drone_camera_viewer.py — 无人机随机飞行 + 实时图像显示

控制无人机在场景中随机移动，同时实时显示无人机摄像头画面。
类似 yolo_guidance.py 的界面，但：
  - 从无人机摄像头视角（不是转台）
  - 无人机在场景中随机巡航
  - 支持键盘控制摄像头角度

按键:
  q — 退出
  w/s — 摄像头 Pitch 上/下
  a/d — 摄像头 Yaw 左/右
  r — 重置摄像头角度到中心
  1 — 直线巡航模式
  2 — 圆形巡航模式
  3 — 随机漫游模式
  h — 悬停
  空格 — 保存当前帧到 snapshots/

使用:
  1. 启动 UE PIE
  2. python drone_camera_viewer.py
"""

import sys, os, time, math, argparse, threading
import base64
import numpy as np
import cv2

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
TURRET_DIR = os.path.join(os.path.dirname(SCRIPT_DIR), "Turret")
sys.path.insert(0, TURRET_DIR)
from sim_client import SimClient


class DroneFlightController:
    """无人机自动飞行控制器（后台线程）"""

    def __init__(self, client, center, altitude=5.0, speed=3.0, radius=30.0, drone_id="drone_0"):
        self.client = client
        self.center = np.array(center[:2])
        self.altitude = altitude
        self.speed = speed
        self.radius = radius
        self.drone_id = drone_id
        self.pattern = "circle"
        self.running = False
        self.thread = None

    def set_pattern(self, pattern):
        self.pattern = pattern
        print(f"[Flight] 模式切换: {pattern}")

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=5)

    def _fly_to(self, x, y, z):
        self.client.drone_move_to(x, y, z, self.speed)

    def _run(self):
        cx, cy = self.center
        alt, R = self.altitude, self.radius

        # 起飞
        self.client.drone_takeoff(alt)
        time.sleep(4)

        step = 0
        while self.running:
            try:
                if self.pattern == "straight":
                    # 直线来回
                    pts = [
                        (cx + R * 0.6, cy - R * 0.3),
                        (cx + R * 0.6, cy + R * 0.3),
                        (cx - R * 0.3, cy + R * 0.5),
                        (cx - R * 0.3, cy - R * 0.5),
                    ]
                    p = pts[step % len(pts)]
                    self._fly_to(p[0], p[1], alt)
                    time.sleep(4)

                elif self.pattern == "circle":
                    # 圆形巡航
                    angle = math.radians((step * 20) % 360)
                    x = cx + R * 0.6 * math.cos(angle)
                    y = cy + R * 0.6 * math.sin(angle)
                    z = alt + math.sin(angle) * 2
                    self._fly_to(x, y, z)
                    time.sleep(2)

                elif self.pattern == "random":
                    # 随机漫游
                    angle = np.random.uniform(0, 2 * math.pi)
                    dist = np.random.uniform(0.2, 0.8) * R
                    x = cx + dist * math.cos(angle)
                    y = cy + dist * math.sin(angle)
                    z = alt + np.random.uniform(-3, 5)
                    self._fly_to(x, y, max(z, 2.0))
                    time.sleep(np.random.uniform(2.0, 5.0))

                step += 1
            except Exception as e:
                print(f"[Flight] 飞行异常: {e}")
                time.sleep(1)

        self.client.drone_hover()


class HUD:
    """简单 HUD 叠加"""

    @staticmethod
    def text(img, s, pos, color=(230, 230, 230), scale=0.4, thick=1):
        cv2.putText(img, s, pos, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thick + 2, cv2.LINE_AA)
        cv2.putText(img, s, pos, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick, cv2.LINE_AA)

    @classmethod
    def draw_crosshair(cls, img):
        h, w = img.shape[:2]
        cx, cy = w // 2, h // 2
        c = (200, 200, 200)
        gap, ln = 8, 14
        cv2.line(img, (cx - gap - ln, cy), (cx - gap, cy), c, 1, cv2.LINE_AA)
        cv2.line(img, (cx + gap, cy), (cx + gap + ln, cy), c, 1, cv2.LINE_AA)
        cv2.line(img, (cx, cy - gap - ln), (cx, cy - gap), c, 1, cv2.LINE_AA)
        cv2.line(img, (cx, cy + gap), (cx, cy + gap + ln), c, 1, cv2.LINE_AA)
        cv2.circle(img, (cx, cy), 2, c, -1, cv2.LINE_AA)

    @classmethod
    def draw_info(cls, img, data):
        lines = [
            f"FPS {data.get('fps', 0):.0f}  F#{data.get('frame', 0)}  MODE: {data.get('pattern', '-').upper()}",
            f"CAM P{data.get('cam_pitch', 0):+.1f}  Y{data.get('cam_yaw', 0):+.1f}",
            f"POS ({data.get('pos_x', 0):.1f}, {data.get('pos_y', 0):.1f}, {data.get('pos_z', 0):.1f})",
        ]
        for i, ln in enumerate(lines):
            cls.text(img, ln, (6, 16 + i * 18))

    @classmethod
    def draw_saved(cls, img):
        h, w = img.shape[:2]
        cls.text(img, "SAVED", (w // 2 - 30, h // 2 + 40), (80, 255, 120), 0.6, 2)


def main():
    parser = argparse.ArgumentParser(description="无人机随机飞行 + 实时图像显示")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9000)
    parser.add_argument("--altitude", type=float, default=5.0, help="基准飞行高度(m)")
    parser.add_argument("--speed", type=float, default=3.0, help="飞行速度")
    parser.add_argument("--radius", type=float, default=30.0, help="飞行半径(m)")
    parser.add_argument("--pattern", default="random", choices=["straight", "circle", "random"])
    parser.add_argument("--drone-id", default="drone_0")
    parser.add_argument("--snapshot-dir", default=os.path.join(SCRIPT_DIR, "snapshots"))
    args = parser.parse_args()

    client = SimClient(args.host, args.port)

    # 获取无人机初始位置
    state = client.drone_state(args.drone_id)
    init_pos = state.get("position", [0, 0, 0])
    print(f"[Drone] 初始位置: ({init_pos[0]:.1f}, {init_pos[1]:.1f}, {init_pos[2]:.1f})")

    # 启动飞行控制
    flight = DroneFlightController(client, init_pos, args.altitude, args.speed, args.radius, args.drone_id)
    flight.set_pattern(args.pattern)
    flight.start()
    time.sleep(5)

    # 摄像头状态
    cam_pitch = 0.0
    cam_yaw = 0.0
    cam_step = 5.0  # 每次按键调整角度

    hud = HUD()
    frame_count = 0
    snap_count = 0
    last_t = time.time()
    saved_flash = 0

    cv2.namedWindow("Drone Camera", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Drone Camera", 960, 540)

    try:
        while True:
            # 获取图像
            img = client.get_image_numpy(args.drone_id)
            if img is None:
                time.sleep(0.1)
                continue

            frame_count += 1
            now = time.time()
            dt = max(now - last_t, 0.001)
            last_t = now
            fps = 1.0 / dt

            # 获取当前位置
            try:
                ds = client.drone_state(args.drone_id)
                pos = ds.get("position", [0, 0, 0])
            except:
                pos = [0, 0, 0]

            # HUD
            hud.draw_crosshair(img)
            hud.draw_info(img, {
                "fps": fps, "frame": frame_count,
                "pattern": flight.pattern,
                "cam_pitch": cam_pitch, "cam_yaw": cam_yaw,
                "pos_x": pos[0], "pos_y": pos[1], "pos_z": pos[2],
            })

            if saved_flash > 0:
                hud.draw_saved(img)
                saved_flash -= 1

            cv2.imshow("Drone Camera", img)

            # 键盘控制
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('w'):
                cam_pitch = min(cam_pitch + cam_step, 90)
                client.drone_set_camera_angles(cam_pitch, cam_yaw, args.drone_id)
            elif key == ord('s'):
                cam_pitch = max(cam_pitch - cam_step, -90)
                client.drone_set_camera_angles(cam_pitch, cam_yaw, args.drone_id)
            elif key == ord('a'):
                cam_yaw -= cam_step
                client.drone_set_camera_angles(cam_pitch, cam_yaw, args.drone_id)
            elif key == ord('d'):
                cam_yaw += cam_step
                client.drone_set_camera_angles(cam_pitch, cam_yaw, args.drone_id)
            elif key == ord('r'):
                cam_pitch, cam_yaw = 0.0, 0.0
                client.drone_set_camera_angles(0, 0, args.drone_id)
            elif key == ord('1'):
                flight.set_pattern("straight")
            elif key == ord('2'):
                flight.set_pattern("circle")
            elif key == ord('3'):
                flight.set_pattern("random")
            elif key == ord('h'):
                client.drone_hover()
                print("[Drone] 悬停")
            elif key == ord(' '):
                # 保存快照
                os.makedirs(args.snapshot_dir, exist_ok=True)
                snap_name = f"snap_{snap_count:04d}.jpg"
                snap_path = os.path.join(args.snapshot_dir, snap_name)
                cv2.imwrite(snap_path, img)
                snap_count += 1
                saved_flash = 15  # 闪烁15帧
                print(f"[Snapshot] 保存 {snap_path}")

    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        flight.stop()
        client.close()
        print(f"\n[统计] 总帧数: {frame_count}, 快照: {snap_count}")


if __name__ == "__main__":
    main()

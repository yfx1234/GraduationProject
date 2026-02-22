"""
yolo_guidance.py — YOLO 视觉制导 + 无人机飞行控制 (现代化科幻 HUD 优化版)

管线:
  后台线程: 无人机在转台 50m 范围内飞行
  主线程: 转台摄像头 → YOLO → OpenCV 实时显示 → 制导 → 转台控制

按键: q退出 f开火 m切换制导 1/2/3飞行模式 p预测线 t跟踪
"""

import sys, os, time, math, argparse, threading
import base64
import numpy as np
import cv2

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
for p in [os.path.join(SCRIPT_DIR, "..", "..", "Yolo", "ultralytics-main"),
          os.path.join(SCRIPT_DIR, "..", "Yolo", "ultralytics-main")]:
    if os.path.exists(p):
        sys.path.insert(0, os.path.abspath(p))
        break

from ultralytics import YOLO
from sim_client import SimClient


# ============================================================
#  无人机飞行控制（后台线程）
# ============================================================

class DroneController:
    def __init__(self, client, turret_pos, altitude=5.0, speed=2.0, radius=40.0):
        self.client = client
        self.turret_pos = np.array(turret_pos[:2])
        self.altitude = altitude
        self.speed = speed
        self.radius_cm = radius * 100
        self.pattern = "straight"
        self.running = False
        self.thread = None

    def set_pattern(self, p):
        self.pattern = p

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=5)

    def _fly_to(self, x_cm, y_cm, z_m):
        self.client.drone_move_to(x_cm / 100.0, y_cm / 100.0, z_m, self.speed)

    def _run(self):
        c = self.client
        tx, ty = self.turret_pos
        alt, R = self.altitude, self.radius_cm

        c.drone_takeoff(alt)
        time.sleep(3)
        self._fly_to(tx + R * 0.5, ty, alt)
        time.sleep(4)

        step = 0
        while self.running:
            try:
                if self.pattern == "straight":
                    pts = [(tx+R*0.6, ty-R*0.3), (tx+R*0.6, ty+R*0.3),
                           (tx+R*0.3, ty+R*0.5), (tx+R*0.3, ty-R*0.5)]
                    p = pts[step % len(pts)]
                    self._fly_to(p[0], p[1], alt)
                    time.sleep(4)
                elif self.pattern == "curved":
                    angle = math.radians((step * 30) % 360)
                    self._fly_to(tx + R*0.5*math.cos(angle), ty + R*0.5*math.sin(angle),
                                 alt + math.sin(angle)*2)
                    time.sleep(2)
                elif self.pattern == "evasive":
                    d = 1 if step % 2 == 0 else -1
                    c.drone_move_by_velocity(self.speed*0.8, self.speed*1.5*d, 0)
                    time.sleep(2)
                    pos = c.drone_position()
                    if np.linalg.norm(pos[:2] - self.turret_pos) > R * 0.8:
                        self._fly_to(tx + R*0.3, ty, alt)
                        time.sleep(3)
                step += 1
            except:
                time.sleep(1)
        c.drone_hover()


# ============================================================
#  现代 HUD 绘制 (科幻风格与半透明面板)
# ============================================================

class HUDRenderer:
    """现代科幻风 HUD — 包含半透明背景板、科技感锁定框"""

    # 采用 BGR 色彩空间，使用科幻感更强的青色/荧光绿
    C_CYAN   = (255, 200, 50)  
    C_GREEN  = (120, 255, 80)
    C_RED    = (60, 60, 255)
    C_WHITE  = (240, 240, 240)
    C_BLACK  = (10, 10, 10)

    @staticmethod
    def draw_panel(img, x1, y1, x2, y2, alpha=0.4, color=(10, 10, 10)):
        """绘制半透明背景板，提升文字可读性与高级感"""
        h, w = img.shape[:2]
        x1, y1 = max(0, int(x1)), max(0, int(y1))
        x2, y2 = min(w, int(x2)), min(h, int(y2))
        if x1 >= x2 or y1 >= y2: return
        
        sub_img = img[y1:y2, x1:x2]
        rect = np.full(sub_img.shape, color, dtype=np.uint8)
        cv2.addWeighted(rect, alpha, sub_img, 1 - alpha, 0, sub_img)

    @staticmethod
    def text(img, s, pos, color=(240,240,240), scale=0.4, thick=1):
        """抗锯齿文字（无需再加粗黑边，因为有背景板）"""
        cv2.putText(img, s, pos, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick, cv2.LINE_AA)

    @classmethod
    def draw_crosshair(cls, img):
        """科幻风准星：包含中心点、内环指示和外侧十字"""
        h, w = img.shape[:2]
        cx, cy = w // 2, h // 2
        c = cls.C_CYAN
        
        # 中心点与微型内环
        cv2.circle(img, (cx, cy), 2, c, -1, cv2.LINE_AA)
        cv2.circle(img, (cx, cy), 12, c, 1, cv2.LINE_AA)
        
        # 外部断续十字线
        gap, ln = 20, 15
        cv2.line(img, (cx-gap-ln, cy), (cx-gap, cy), c, 1, cv2.LINE_AA)
        cv2.line(img, (cx+gap, cy), (cx+gap+ln, cy), c, 1, cv2.LINE_AA)
        cv2.line(img, (cx, cy-gap-ln), (cx, cy-gap), c, 1, cv2.LINE_AA)
        cv2.line(img, (cx, cy+gap), (cx, cy+gap+ln), c, 1, cv2.LINE_AA)

    @classmethod
    def draw_detection_box(cls, img, box, conf, cls_id=0):
        """高级锁定框：四角包围 + 内部微透明填充 + 数据跟随"""
        x1, y1, x2, y2 = [int(v) for v in box]
        c = cls.C_GREEN
        
        # 内部半透明锁定高亮 (Alpha 0.15)
        cls.draw_panel(img, x1, y1, x2, y2, alpha=0.15, color=(0, 200, 0))
        
        # 绘制四角 L 型边框
        corner = 15
        thick = 2
        cv2.line(img, (x1, y1), (x1+corner, y1), c, thick, cv2.LINE_AA)
        cv2.line(img, (x1, y1), (x1, y1+corner), c, thick, cv2.LINE_AA)
        cv2.line(img, (x2, y1), (x2-corner, y1), c, thick, cv2.LINE_AA)
        cv2.line(img, (x2, y1), (x2, y1+corner), c, thick, cv2.LINE_AA)
        cv2.line(img, (x1, y2), (x1+corner, y2), c, thick, cv2.LINE_AA)
        cv2.line(img, (x1, y2), (x1, y2-corner), c, thick, cv2.LINE_AA)
        cv2.line(img, (x2, y2), (x2-corner, y2), c, thick, cv2.LINE_AA)
        cv2.line(img, (x2, y2), (x2, y2-corner), c, thick, cv2.LINE_AA)
        
        # 顶部信息标签底板与文字
        cls.draw_panel(img, x1, y1-20, x1+100, y1, alpha=0.6, color=cls.C_BLACK)
        cls.text(img, f"TGT-LOCKED [{conf:.2f}]", (x1+5, y1-6), cls.C_CYAN, 0.4)
        
        # 准心锁定标记
        mx, my = (x1+x2)//2, (y1+y2)//2
        cv2.drawMarker(img, (mx, my), cls.C_RED, cv2.MARKER_CROSS, 10, 1, cv2.LINE_AA)

    @classmethod
    def draw_info(cls, img, data):
        """左上角系统状态面板"""
        lines = [
            f"SYS: ACTIVE | FPS: {data.get('fps',0):.1f}",
            f"FRAME: {data.get('frame',0)} | DET: {data.get('detections', 0)}",
            f"MODE: {data.get('method','-').upper()}",
            f"TRACKING: {'ENGAGED' if data.get('tracking') else 'STANDBY'}",
            f"PITCH: {data.get('pitch',0):+06.1f} | YAW: {data.get('yaw',0):+06.1f}",
        ]
        
        # 绘制统一的半透明底板
        panel_w = 260
        panel_h = 15 + len(lines) * 20
        cls.draw_panel(img, 10, 10, 10 + panel_w, 10 + panel_h, alpha=0.5)
        
        # 边框修饰
        cv2.rectangle(img, (10, 10), (10 + panel_w, 10 + panel_h), cls.C_CYAN, 1, cv2.LINE_AA)
        
        for i, ln in enumerate(lines):
            # 区分重点信息的颜色
            color = cls.C_CYAN if "ENGAGED" in ln or "ACTIVE" in ln else cls.C_WHITE
            cls.text(img, ln, (20, 32 + i * 20), color, 0.45)

    @classmethod
    def draw_target(cls, img, pos):
        """右下角空间坐标面板"""
        h, w = img.shape[:2]
        panel_w = 220
        panel_h = 40
        x1, y1 = w - panel_w - 10, h - panel_h - 10
        
        cls.draw_panel(img, x1, y1, x1 + panel_w, y1 + panel_h, alpha=0.5)
        cv2.rectangle(img, (x1, y1), (x1 + panel_w, y1 + panel_h), cls.C_CYAN, 1, cv2.LINE_AA)
        
        cls.text(img, "ESTIMATED SPATIAL COORD", (x1 + 10, y1 + 15), (150, 150, 150), 0.35)
        cls.text(img, f"X:{pos[0]:.0f} Y:{pos[1]:.0f} Z:{pos[2]:.0f}", (x1 + 10, y1 + 32), cls.C_CYAN, 0.45)

    @classmethod
    def draw_fire(cls, img):
        """开火警告指示"""
        h, w = img.shape[:2]
        cls.draw_panel(img, w//2 - 60, h//2 + 40, w//2 + 60, h//2 + 75, alpha=0.7, color=(0,0,150))
        cv2.putText(img, "FIRING", (w//2 - 45, h//2 + 65), cv2.FONT_HERSHEY_SIMPLEX, 0.8, cls.C_WHITE, 2, cv2.LINE_AA)


# ============================================================
#  视觉制导
# ============================================================

class VisionGuidance:
    def __init__(self, client, model_path, turret_id="turret_0",
                 muzzle_speed=400.0, conf=0.3, method="predictive"):
        self.client = client
        self.turret_id = turret_id
        self.muzzle_speed = muzzle_speed
        self.conf = conf
        self.model = YOLO(model_path)
        client.guidance_reset()
        client.guidance_set_method(method)
        self.method = method
        self.cam_fov = 90.0
        self.cam_w = 640
        self.cam_h = 480
        self.cam_pos = np.zeros(3)
        self.cam_rot = np.zeros(3)
        self.frame_count = 0
        self.detect_count = 0
        self.hud = HUDRenderer()

    def get_image(self):
        try:
            resp = self.client._send({"get_image": {}})
        except Exception:
            return None
        if resp.get("status") != "ok":
            return None
        b64 = resp.get("data", "")
        if not b64:
            return None
        img = cv2.imdecode(np.frombuffer(base64.b64decode(b64), np.uint8), cv2.IMREAD_COLOR)
        self.cam_w = resp.get("width", self.cam_w)
        self.cam_h = resp.get("height", self.cam_h)
        self.cam_fov = resp.get("fov", self.cam_fov)
        self.cam_pos = np.array(resp.get("camera_pos", [0,0,0]))
        self.cam_rot = np.array(resp.get("camera_rot", [0,0,0]))
        return img

    def pixel_to_direction(self, u, v):
        fx = (self.cam_w / 2.0) / math.tan(math.radians(self.cam_fov / 2.0))
        d = np.array([fx, u - self.cam_w/2.0, -(v - self.cam_h/2.0)])
        return d / np.linalg.norm(d)

    def cam_to_world(self, d):
        p, y, r = [math.radians(x) for x in self.cam_rot]
        cp, sp = math.cos(p), math.sin(p)
        cy, sy = math.cos(y), math.sin(y)
        cr, sr = math.cos(r), math.sin(r)
        fwd = np.array([cp*cy, cp*sy, sp])
        right = np.array([cy*sp*sr-sy*cr, sy*sp*sr+cy*cr, -cp*sr])
        up = np.array([-(cy*sp*cr+sy*sr), -(sy*sp*cr-cy*sr), cp*cr])
        w = d[0]*fwd + d[1]*right + d[2]*up
        return w / np.linalg.norm(w)

    def estimate_3d(self, u, v, bbox_area):
        d_cam = self.pixel_to_direction(u, v)
        d_world = self.cam_to_world(d_cam)
        depth = max(100, min(5000/math.sqrt(max(bbox_area,1)), 50000))
        return self.cam_pos + d_world * depth, depth

    def run(self, drone_ctrl, auto_fire=False, fire_interval=80):
        cv2.namedWindow("Turret Camera - YOLO Guidance", cv2.WINDOW_NORMAL)
        # 将窗口初始化为推荐的高清分辨率
        cv2.resizeWindow("Turret Camera - YOLO Guidance", 1280, 720)

        methods = ["predictive", "proportional", "direct"]
        mi = methods.index(self.method) if self.method in methods else 0
        tracking = True
        last_t = time.time()
        fired = False

        try:
            while True:
                img = self.get_image()
                if img is None:
                    time.sleep(0.05)
                    continue

                # ⚠️ 此处已删除原有的 self.hud.enhance_image，保留 UE5 原生高清画质
                self.frame_count += 1

                try:
                    results = self.model(img, conf=self.conf, verbose=False)
                except Exception:
                    results = []

                detected = False
                pos = None
                pitch, yaw = 0, 0
                fired = False

                if len(results) > 0 and len(results[0].boxes) > 0:
                    boxes = results[0].boxes.xyxy.cpu().numpy()
                    confs = results[0].boxes.conf.cpu().numpy()
                    classes = results[0].boxes.cls.cpu().numpy()
                    self.detect_count += 1
                    detected = True

                    # 找到置信度最高的框用于制导
                    bi = confs.argmax()
                    x1,y1,x2,y2 = boxes[bi]
                    cx, cy = (x1+x2)/2, (y1+y2)/2
                    area = (x2-x1)*(y2-y1)
                    pos, _ = self.estimate_3d(cx, cy, area)

                    self.client.guidance_update_target(float(pos[0]), float(pos[1]), float(pos[2]), 0.1)
                    aim = self.client.guidance_compute_aim(self.turret_id, self.muzzle_speed)
                    if aim.get("status") == "ok":
                        pitch = aim["pitch"]
                        yaw = aim["yaw"]

                    # 绘制所有检测到的目标框
                    for i in range(len(boxes)):
                        self.hud.draw_detection_box(img, boxes[i], confs[i])

                if auto_fire and tracking and detected and self.frame_count % fire_interval == 0:
                    self.client.turret_fire(self.muzzle_speed, self.turret_id)
                    fired = True

                now = time.time()
                fps = 1.0 / max(now - last_t, 0.001)
                last_t = now

                # 获取转台当前角度
                try:
                    ts = self.client.turret_state(self.turret_id)
                    pitch = ts.get("pitch", pitch)
                    yaw = ts.get("yaw", yaw)
                except:
                    pass

                # 绘制现代 HUD
                self.hud.draw_crosshair(img)
                self.hud.draw_info(img, {
                    "fps": fps, "frame": self.frame_count,
                    "detections": self.detect_count,
                    "method": self.method, "pattern": drone_ctrl.pattern,
                    "tracking": tracking, "pitch": pitch, "yaw": yaw,
                })
                if detected and pos is not None:
                    self.hud.draw_target(img, pos)
                if fired:
                    self.hud.draw_fire(img)

                cv2.imshow("Turret Camera - YOLO Guidance", img)

                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('f'):
                    self.client.turret_fire(self.muzzle_speed, self.turret_id)
                elif key == ord('m'):
                    mi = (mi + 1) % len(methods)
                    self.method = methods[mi]
                    self.client.guidance_set_method(self.method)
                elif key == ord('1'):
                    drone_ctrl.set_pattern("straight")
                elif key == ord('2'):
                    drone_ctrl.set_pattern("curved")
                elif key == ord('3'):
                    drone_ctrl.set_pattern("evasive")
                elif key == ord('p'):
                    self.client._send({"call_turret": {"function": "show_prediction"}})
                elif key == ord('t'):
                    tracking = not tracking
                    if tracking:
                        self.client.turret_start_tracking("drone_0", self.turret_id)
                    else:
                        self.client.turret_stop_tracking(self.turret_id)
        except KeyboardInterrupt:
            pass
        finally:
            cv2.destroyAllWindows()
            rate = self.detect_count / max(self.frame_count, 1) * 100
            print(f"\n[统计] 帧:{self.frame_count} 检测:{self.detect_count} 率:{rate:.1f}%")


# ============================================================

def find_model():
    # 优先使用自定义训练模型
    for p in [os.path.join(SCRIPT_DIR, "..", "..", "Yolo", "yolo26n-objv1-150.pt"),
              os.path.join(SCRIPT_DIR, "..", "..", "Yolo", "yolo26n.pt")]:
        if os.path.exists(p):
            return os.path.abspath(p)
    return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default=None)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9000)
    parser.add_argument("--conf", type=float, default=0.3)
    parser.add_argument("--method", default="predictive")
    parser.add_argument("--pattern", default="straight")
    parser.add_argument("--speed", type=float, default=2.0)
    parser.add_argument("--altitude", type=float, default=5.0)
    parser.add_argument("--fire", action="store_true")
    parser.add_argument("--fire-interval", type=int, default=80)
    parser.add_argument("--muzzle-speed", type=float, default=400.0)
    parser.add_argument("--radius", type=float, default=40.0)
    args = parser.parse_args()

    if args.model is None:
        args.model = find_model()
    if not args.model or not os.path.exists(args.model):
        print("[ERROR] YOLO 模型未找到，请用 --model 指定")
        return

    client = SimClient(args.host, args.port)

    ts = client.turret_state()
    turret_pos = ts.get("position", [0, 0, 0])
    print(f"[Turret] pos=({turret_pos[0]:.0f}, {turret_pos[1]:.0f}, {turret_pos[2]:.0f})")

    drone = DroneController(client, turret_pos, args.altitude, args.speed, args.radius)
    drone.set_pattern(args.pattern)
    drone.start()
    time.sleep(8)

    client.turret_start_tracking("drone_0")
    client._send({"call_turret": {"function": "show_prediction"}})
    print("[Turret] 跟踪 + 预测线 ON")

    guidance = VisionGuidance(client, args.model, muzzle_speed=args.muzzle_speed,
                              conf=args.conf, method=args.method)
    guidance.run(drone, auto_fire=args.fire, fire_interval=args.fire_interval)

    client.turret_stop_tracking()
    client._send({"call_turret": {"function": "hide_prediction"}})
    drone.stop()
    client.close()

if __name__ == "__main__":
    main()
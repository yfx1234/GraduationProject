"""
control.py — 控制无人机和转台运动
运行时同时运行 yolo_guidance.py 实现视觉制导闭环

功能：
  1. 控制无人机按指定模式飞行（直线/弧线/S型）
  2. 可选手动控制转台角度/开火
  3. 配合 yolo_guidance.py 使用
"""

import time, sys, argparse
sys.path.insert(0, ".")
from sim_client import SimClient


def fly_straight(client, speed=2.0, duration=15):
    """直线匀速飞行"""
    print(f"[飞行模式] 直线匀速 speed={speed} duration={duration}s")
    client.drone_move_by_velocity(speed, 0, 0)
    time.sleep(duration)


def fly_curved(client, speed=2.0, duration=15):
    """弧线飞行"""
    print(f"[飞行模式] 弧线 speed={speed} duration={duration}s")
    client.drone_move_by_velocity(speed, speed * 0.5, 0.3)
    time.sleep(duration)


def fly_evasive(client, speed=2.0, duration=15, period=3.0):
    """S型机动飞行"""
    print(f"[飞行模式] S型机动 speed={speed} period={period}s duration={duration}s")
    t_start = time.time()
    direction = 1
    while time.time() - t_start < duration:
        vy = speed * direction
        client.drone_move_by_velocity(speed, vy, 0)
        time.sleep(period)
        direction *= -1


def main():
    parser = argparse.ArgumentParser(description="控制无人机和转台")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9000)
    parser.add_argument("--pattern", default="straight", choices=["straight", "curved", "evasive"],
                        help="飞行模式")
    parser.add_argument("--speed", type=float, default=2.0, help="飞行速度")
    parser.add_argument("--duration", type=float, default=20.0, help="飞行时长(秒)")
    parser.add_argument("--altitude", type=float, default=5.0, help="起飞高度(m)")
    args = parser.parse_args()

    client = SimClient(args.host, args.port)

    try:
        # 1. ping 测试
        print("[1] 连接测试:", client.ping())
        print("[2] Agent 列表:", client.get_agents())

        # 2. 起飞
        print(f"\n[3] 无人机起飞 (altitude={args.altitude}m)...")
        client.drone_takeoff(args.altitude)
        time.sleep(4)
        print("    起飞完成:", client.drone_state())

        # 3. 提示用户启动 yolo_guidance.py
        print("\n" + "=" * 50)
        print("  请在另一个终端运行: python yolo_guidance.py")
        print("  视觉制导将自动控制转台瞄准和开火")
        print("=" * 50)
        input("按 Enter 开始飞行...")

        # 4. 按模式飞行
        print(f"\n[4] 开始飞行 (模式={args.pattern})...")
        if args.pattern == "straight":
            fly_straight(client, args.speed, args.duration)
        elif args.pattern == "curved":
            fly_curved(client, args.speed, args.duration)
        elif args.pattern == "evasive":
            fly_evasive(client, args.speed, args.duration)

        # 5. 悬停
        print("\n[5] 飞行结束，悬停...")
        client.drone_hover()
        time.sleep(2)

        print("[6] 最终状态:", client.drone_state())

    except KeyboardInterrupt:
        print("\n[中断] 悬停并退出...")
        client.drone_hover()

    finally:
        client.close()
        print("[完成]")


if __name__ == "__main__":
    main()

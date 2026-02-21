"""
GuidanceRunner — 单次制导实验
无人机飞行 → 制导跟踪 → 采集瞄准误差数据

支持 3 种飞行模式：
  - straight: 直线匀速
  - curved:   弧线
  - evasive:  S 型机动

支持 3 种制导方法：
  - direct
  - proportional
  - predictive

使用方法:
  from sim_client import SimClient
  from guidance_runner import GuidanceRunner

  client = SimClient()
  runner = GuidanceRunner(client)
  result = runner.run_trial("predictive", "straight", duration=10, fire_interval=5)
"""

import time, math
import numpy as np
from sim_client import SimClient

class GuidanceRunner:
    def __init__(self, client: SimClient,
                 turret_id: str = "turret_0",
                 drone_id: str = "drone_0",
                 muzzle_speed: float = 400.0):
        self.client = client
        self.turret_id = turret_id
        self.drone_id = drone_id
        self.muzzle_speed = muzzle_speed

    def set_flight_pattern(self, pattern: str, speed: float = 2.0):
        """设置无人机飞行模式"""
        if pattern == "straight":
            self.client.drone_move_by_velocity(speed, 0, 0)
        elif pattern == "curved":
            self.client.drone_move_by_velocity(speed, speed * 0.5, 0)
        elif pattern == "evasive":
            self.client.drone_move_by_velocity(speed, 0, 0)
        else:
            raise ValueError(f"Unknown pattern: {pattern}")

    def run_trial(self, method: str, pattern: str,
                  duration: float = 10.0, dt: float = 0.1,
                  fire_interval: int = 10) -> dict:
        """
        运行单次制导实验

        Returns:
            dict: {
                "method": str,
                "pattern": str,
                "positions": list,        # 无人机真实位置序列
                "predicted_positions": list, # 卡尔曼预测位置序列
                "aim_errors": list,        # 瞄准角度误差序列
                "fire_count": int,         # 开火次数
                "elapsed": float           # 实际耗时
            }
        """
        c = self.client

        # 重置状态
        c.guidance_reset()
        c.turret_reset(self.turret_id)

        # 设置制导方法
        c.guidance_set_method(method)

        # 起飞并等待
        c.drone_takeoff(altitude=5)
        time.sleep(3)

        # 开始飞行
        self.set_flight_pattern(pattern)

        # 采集数据
        positions = []
        predicted_positions = []
        aim_errors = []
        fire_count = 0
        steps = int(duration / dt)

        t_start = time.time()

        for step in range(steps):
            # 1. 获取无人机位置
            pos = c.drone_position(self.drone_id)
            positions.append(pos.tolist())

            # 2. S 型机动：周期性变向
            if pattern == "evasive" and step % 20 == 10:
                vy = 2.0 * (1 if (step // 20) % 2 == 0 else -1)
                c.drone_move_by_velocity(2.0, vy, 0)

            # 3. 制导更新 + 计算瞄准
            c.guidance_update_target(pos[0], pos[1], pos[2], dt)

            aim_result = c.guidance_compute_aim(self.turret_id, self.muzzle_speed)

            if aim_result.get("status") == "ok":
                # 设置转台角度
                c.turret_set_angles(aim_result["pitch"], aim_result["yaw"], self.turret_id)

                # 获取预测位置
                g_state = c.guidance_state()
                pred_pos = g_state.get("est_pos", [0, 0, 0])
                predicted_positions.append(pred_pos)

                # 计算预测误差
                err = np.linalg.norm(np.array(pred_pos) - pos)
                aim_errors.append(err)

            # 4. 定期开火
            if step > 0 and step % fire_interval == 0:
                c.turret_fire(self.muzzle_speed, self.turret_id)
                fire_count += 1

            time.sleep(dt)

        elapsed = time.time() - t_start

        # 清理
        c.drone_hover()

        return {
            "method": method,
            "pattern": pattern,
            "positions": positions,
            "predicted_positions": predicted_positions,
            "aim_errors": aim_errors,
            "fire_count": fire_count,
            "elapsed": elapsed,
            "mean_error": float(np.mean(aim_errors)) if aim_errors else 0.0,
            "final_error": aim_errors[-1] if aim_errors else 0.0,
        }


if __name__ == "__main__":
    client = SimClient()
    runner = GuidanceRunner(client)

    result = runner.run_trial("predictive", "straight", duration=5)
    print(f"Method: {result['method']}, Pattern: {result['pattern']}")
    print(f"Mean error: {result['mean_error']:.2f}, Fire count: {result['fire_count']}")
    print(f"Elapsed: {result['elapsed']:.2f}s")

    client.close()

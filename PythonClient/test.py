import socket, json, time, sys

# ============================================================
#  GraduationProject 综合测试脚本
#  覆盖：基础命令 / Drone / Turret / Guidance / 仿真控制
# ============================================================

HOST = "127.0.0.1"
PORT = 9000

def send(s, cmd):
    """发送 JSON 命令并接收响应"""
    s.send(json.dumps(cmd).encode() + b"\n")
    return json.loads(s.recv(8192))

def section(title):
    print(f"\n{'='*60}")
    print(f"  {title}")
    print(f"{'='*60}")

def step(name, s, cmd, wait=0):
    print(f"\n--- {name} ---")
    result = send(s, cmd)
    print(json.dumps(result, indent=2, ensure_ascii=False))
    if wait > 0:
        time.sleep(wait)
    return result

# ---- 连接 ----
s = socket.socket()
try:
    s.connect((HOST, PORT))
except ConnectionRefusedError:
    print(f"[ERROR] 无法连接 {HOST}:{PORT}，请确认 UE 已启动 PIE 模式")
    sys.exit(1)
print(f"[OK] 已连接 {HOST}:{PORT}")

# ============================================================
# 1. 基础命令
# ============================================================
section("1. 基础命令")

step("1.1 Ping", s, {"ping": {}})
step("1.2 Agent 列表", s, {"get_agent_list": {}})

# ============================================================
# 2. Drone 全功能测试
# ============================================================
section("2. Drone 全功能")

# 起飞
step("2.1 起飞 (altitude=3)", s,
     {"call_drone": {"function": "takeoff", "altitude": 3}}, wait=3)

step("2.2 查询状态", s,
     {"get_drone_state": {"id": "drone_0"}})

# 位置控制 - 多点飞行
step("2.3 移动到 (5, 0, 3)", s,
     {"call_drone": {"function": "move_to_position", "x": 5, "y": 0, "z": 3, "speed": 2}}, wait=4)

step("2.3.1 到达后状态", s,
     {"get_drone_state": {"id": "drone_0"}})

step("2.4 移动到 (5, 5, 5)", s,
     {"call_drone": {"function": "move_to_position", "x": 5, "y": 5, "z": 5, "speed": 3}}, wait=3)

step("2.4.1 到达后状态", s,
     {"get_drone_state": {"id": "drone_0"}})

# 速度控制
step("2.5 速度控制 vx=1, vy=0, vz=0", s,
     {"call_drone": {"function": "move_by_velocity", "vx": 1, "vy": 0, "vz": 0}}, wait=2)

step("2.5.1 飞行中状态", s,
     {"get_drone_state": {"id": "drone_0"}})

# 悬停
step("2.6 悬停", s,
     {"call_drone": {"function": "hover"}}, wait=2)

step("2.6.1 悬停后状态", s,
     {"get_drone_state": {"id": "drone_0"}})

# PID 调参
step("2.7 设置位置控制器 PID", s,
     {"call_drone": {"function": "set_pid", "controller": "position", "kp": 1.0, "kd": 0.5}})

step("2.8 设置速度控制器 PID", s,
     {"call_drone": {"function": "set_pid", "controller": "velocity", "kp": 5.0, "ki": 0.1, "kd": 0.5}})

step("2.9 设置姿态控制器 PID", s,
     {"call_drone": {"function": "set_pid", "controller": "attitude", "kp": 10.0, "kd": 1.0}})

step("2.10 设置角速率控制器 PID", s,
     {"call_drone": {"function": "set_pid", "controller": "angle_rate", "kp": 0.5}})

# 回到原点附近
step("2.11 回到 (0, 0, 3)", s,
     {"call_drone": {"function": "move_to_position", "x": 0, "y": 0, "z": 3, "speed": 3}}, wait=4)

# ============================================================
# 3. Turret 全功能测试
# ============================================================
section("3. Turret 全功能")

step("3.1 查询初始状态", s,
     {"get_turret_state": {"id": "turret_0"}})

# 设置角度 - 多组
step("3.2 设置角度 pitch=15, yaw=45", s,
     {"call_turret": {"function": "set_angles", "pitch": 15, "yaw": 45}}, wait=2)

step("3.2.1 旋转后状态", s,
     {"get_turret_state": {"id": "turret_0"}})

step("3.3 设置角度 pitch=-10, yaw=-90", s,
     {"call_turret": {"function": "set_angles", "pitch": -10, "yaw": -90}}, wait=2)

step("3.3.1 旋转后状态", s,
     {"get_turret_state": {"id": "turret_0"}})

# 开火
step("3.4 开火 speed=400", s,
     {"call_turret": {"function": "fire", "speed": 400}}, wait=1)

step("3.5 调整角度后开火 (默认速度)", s,
     {"call_turret": {"function": "set_angles", "pitch": 20, "yaw": 0}}, wait=2)

step("3.5.1 开火", s,
     {"call_turret": {"function": "fire"}}, wait=1)

# 弹道预测线
step("3.6 显示弹道预测线", s,
     {"call_turret": {"function": "show_prediction"}}, wait=2)

step("3.7 隐藏弹道预测线", s,
     {"call_turret": {"function": "hide_prediction"}})

# 跟踪
step("3.8 无人机移到 (8, 3, 5) 供跟踪", s,
     {"call_drone": {"function": "move_to_position", "x": 8, "y": 3, "z": 5, "speed": 3}}, wait=4)

step("3.9 开始跟踪 drone_0", s,
     {"call_turret": {"function": "start_tracking", "target_id": "drone_0"}}, wait=2)

step("3.9.1 跟踪中状态", s,
     {"get_turret_state": {"id": "turret_0"}})

step("3.10 无人机移到 (-5, 5, 4) 验证跟踪更新", s,
     {"call_drone": {"function": "move_to_position", "x": -5, "y": 5, "z": 4, "speed": 2}}, wait=4)

step("3.10.1 跟踪更新后状态", s,
     {"get_turret_state": {"id": "turret_0"}})

step("3.11 停止跟踪", s,
     {"call_turret": {"function": "stop_tracking"}})

# 复位
step("3.12 转台复位", s,
     {"call_turret": {"function": "reset"}}, wait=1)

step("3.12.1 复位后状态", s,
     {"get_turret_state": {"id": "turret_0"}})

# ============================================================
# 4. Guidance 制导系统测试
# ============================================================
section("4. Guidance 制导系统")

# 4.1 设置卡尔曼滤波参数
step("4.1 设置卡尔曼参数 Q=1.0 R=0.5", s,
     {"call_guidance": {"function": "set_kalman_params", "process_noise": 1.0, "measurement_noise": 0.5}})

# 4.2 设置制导方法 - 预测制导 (默认)
step("4.2 设置制导方法: predictive (iterations=3)", s,
     {"call_guidance": {"function": "set_method", "method": "predictive", "iterations": 3}})

# 4.3 模拟目标观测 (喂卡尔曼数据)
step("4.3 开始喂入目标观测数据 (模拟匀速运动)", s, {"ping": {}})
for i in range(10):
    x = 100 + i * 50
    y = 200 + i * 10
    z = 300
    resp = send(s, {"call_guidance": {"function": "update_target", "x": x, "y": y, "z": z, "dt": 0.1}})
    if i % 3 == 0:
        print(f"  [frame {i}] est_pos={resp.get('est_pos')}, est_vel={resp.get('est_vel')}")

# 4.4 查询制导状态
step("4.4 制导系统状态", s,
     {"get_guidance_state": {}})

# 4.5 计算瞄准角 (predictive)
step("4.5 计算瞄准角 (predictive, muzzle_speed=400)", s,
     {"call_guidance": {"function": "compute_aim", "turret_id": "turret_0", "muzzle_speed": 400}})

# 4.6 切换到直接瞄准
step("4.6 切换制导方法: direct", s,
     {"call_guidance": {"function": "set_method", "method": "direct"}})

step("4.6.1 计算瞄准角 (direct)", s,
     {"call_guidance": {"function": "compute_aim", "turret_id": "turret_0"}})

# 4.7 切换到比例导引
step("4.7 切换制导方法: proportional (N=4)", s,
     {"call_guidance": {"function": "set_method", "method": "proportional", "nav_constant": 4.0}})

step("4.7.1 计算瞄准角 (proportional)", s,
     {"call_guidance": {"function": "compute_aim", "turret_id": "turret_0", "muzzle_speed": 400}})

# 4.8 无人机移到一个位置，然后 auto_engage
step("4.8 无人机移到 (10, 0, 4)", s,
     {"call_drone": {"function": "move_to_position", "x": 10, "y": 0, "z": 4, "speed": 3}}, wait=4)

step("4.9 自动交战 (不开火)", s,
     {"call_guidance": {"function": "auto_engage", "turret_id": "turret_0", "target_id": "drone_0"}})

step("4.9.1 转台状态 (角度已设置)", s,
     {"get_turret_state": {"id": "turret_0"}})

# 4.10 自动交战 + 开火
step("4.10 切换回 predictive 制导", s,
     {"call_guidance": {"function": "set_method", "method": "predictive"}})

step("4.10.1 自动交战 + 开火", s,
     {"call_guidance": {"function": "auto_engage", "turret_id": "turret_0", "target_id": "drone_0",
                        "muzzle_speed": 400, "fire": True}}, wait=1)

# 4.11 制导系统复位
step("4.11 制导系统复位", s,
     {"call_guidance": {"function": "reset"}})

step("4.11.1 复位后状态", s,
     {"get_guidance_state": {}})

# ============================================================
# 5. 仿真控制
# ============================================================
section("5. 仿真控制")

step("5.1 暂停仿真", s,
     {"sim_pause": {}}, wait=2)

step("5.1.1 暂停时查询 drone 状态", s,
     {"get_drone_state": {"id": "drone_0"}})

step("5.2 恢复仿真", s,
     {"sim_resume": {}}, wait=1)

# ============================================================
# 6. 收尾
# ============================================================
section("6. 收尾")

step("6.1 转台复位", s,
     {"call_turret": {"function": "reset"}}, wait=1)

step("6.2 无人机悬停", s,
     {"call_drone": {"function": "hover"}}, wait=1)

step("6.3 无人机降落", s,
     {"call_drone": {"function": "land"}}, wait=3)

step("6.3.1 降落后状态", s,
     {"get_drone_state": {"id": "drone_0"}})

# ============================================================
s.close()
section("全部测试完成 ✓")

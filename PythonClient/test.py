import socket, json, time

def send(s, cmd):
    s.send(json.dumps(cmd).encode() + b"\n")
    return json.loads(s.recv(4096))

s = socket.socket()
s.connect(("127.0.0.1", 9000))

# 1. ping
print("=== PING ===")
print(send(s, {"ping": {}}))

# 2. 查看 agent 列表
print("\n=== AGENT LIST ===")
print(send(s, {"get_agent_list": {}}))

# 3. 起飞到 3m
print("\n=== TAKEOFF ===")
print(send(s, {"call_drone": {"function": "takeoff", "altitude": 3}}))
time.sleep(3)

# 4. 查状态
print("\n=== STATE (after takeoff) ===")
print(send(s, {"get_drone_state": {"id": "drone_0"}}))

# 5. 飞到 (5, 0, 3)
print("\n=== MOVE TO (5, 0, 3) ===")
print(send(s, {"call_drone": {"function": "move_to_position", "x": 5, "y": 0, "z": 3, "speed": 2}}))
time.sleep(5)

# 6. 查状态
print("\n=== STATE (after move) ===")
print(send(s, {"get_drone_state": {"id": "drone_0"}}))

# 7. 悬停
print("\n=== HOVER ===")
print(send(s, {"call_drone": {"function": "hover"}}))

s.close()
print("\n=== DONE ===")

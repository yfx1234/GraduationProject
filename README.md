# 基于虚幻引擎的无人机视觉追踪与协同仿真系统

> 毕业设计项目 — 在 Unreal Engine 5.7 中构建的无人机-转台协同仿真平台，集成了基于 YOLO 的目标检测、卡尔曼滤波预测制导和弹道仿真。

## 📋 项目概述

本项目实现了一个完整的无人机视觉追踪与武器制导仿真系统，主要包含以下功能：

- **无人机飞行仿真**：基于 PID 控制的六自由度无人机飞行模型，支持起降、悬停、航点飞行和速度控制
- **转台武器系统**：可旋转的武器平台，支持俯仰/偏航角控制、目标跟踪和射击，内置弹道计算模块
- **YOLO 目标检测**：基于 YOLOv11 的实时无人机检测，从转台摄像头画面中识别和定位目标
- **多模式制导算法**：支持直接瞄准、比例导引和卡尔曼预测制导三种瞄准方法
- **TCP 通信架构**：Python 客户端通过 JSON-TCP 协议与 UE 引擎实时通信
- **自动化数据管线**：一键采集训练数据并自动标注，支持断点续训

## 🏗️ 系统架构

## 📁 目录结构

```
GraduationProject/
├── Source/GraduationProject/          # C++ 源码
│   ├── Core/                          # 核心框架
│   │   ├── CameraPawn.*               #   自由视角相机
│   │   ├── SimGameMode.*              #   游戏模式 & Agent 管理
│   │   ├── Network/                   #   TCP 服务器 & JSON 命令路由
│   │   ├── Controller/                #   玩家控制器
│   │   └── Manager/                   #   Agent 注册管理
│   ├── Drone/                         # 无人机模块
│   │   ├── DronePawn.*                #   无人机 Actor
│   │   ├── DroneMovementComponent.*   #   飞行物理 (PID 控制)
│   │   ├── DroneParameters.h          #   物理参数配置
│   │   ├── DroneState.h               #   状态数据结构
│   │   ├── DroneApi.*                 #   API 接口层
│   │   └── DroneCommandHandler.*      #   命令处理器
│   ├── Turret/                        # 转台武器模块
│   │   ├── TurretPawn.*               #   转台 Actor (含摄像头)
│   │   ├── TurretAiming.*             #   瞄准计算组件
│   │   ├── TurretCommandHandler.*     #   命令处理器
│   │   ├── BulletActor.*              #   子弹 Actor
│   │   └── BC_*.hpp                   #   弹道计算库
│   ├── Guidance/                      # 制导模块
│   │   ├── IGuidanceMethod.h          #   制导方法接口
│   │   ├── GuidanceMethods.*          #   三种制导实现
│   │   ├── ITargetPredictor.h         #   预测器接口
│   │   ├── KalmanPredictor.*          #   卡尔曼滤波器
│   │   └── GuidanceCommandHandler.*   #   制导命令处理
│   ├── UI/                            # HUD 界面
│   └── Vision/                        # 视觉模块
│
├── PythonClient/                      # Python 客户端
│   ├── sim_client.py                  #   TCP 通信封装
│   ├── yolo_guidance.py               #   YOLO 视觉制导主程序
│   ├── collect_data.py                #   自动数据采集 & 标注
│   ├── train_yolo.py                  #   YOLO 模型训练
│   └── YOLO/                          #   YOLO 相关资源
│       ├── ultralytics/               #     ultralytics 库源码
│       ├── weights/                   #     预训练权重 (yolo26n*.pt)
│       ├── dataset/                   #     训练数据集
│       │   ├── images/train/          #       训练图片
│       │   ├── labels/train/          #       YOLO 标签
│       │   └── data.yaml              #       数据集配置
│       └── runs/                      #     训练输出
│           └── detect/
│               └── drone_detect*/weights/best.pt
```

## 🚀 快速开始

### 环境要求

- **Unreal Engine** 5.7
- **Visual Studio** 2022
- **Python** 3.11（Anaconda 环境）

### Python 环境配置

使用 Anaconda 创建独立环境：

```bash
conda create -n yolo python=3.11
conda activate yolo
```

**核心依赖：**

```
torch              2.5.0+cu118
torchvision        0.20.0+cu118
torchaudio         2.5.0+cu118
ultralytics        8.4.9          # 从本地源码安装
numpy              2.3.5
opencv-python      4.13.0.90
matplotlib         3.10.8
pandas             3.0.1
scipy              1.17.0
PyYAML             6.0.3
requests           2.32.5
pillow             12.0.0
```

**外部资源：**

- ultralytics 源码：https://github.com/ultralytics/ultralytics.git
- YOLO26 模型权重：https://github.com/ultralytics/assets/releases

> **环境摘要：** `PyTorch 2.5.0` + `CUDA 11.8` + `ultralytics 8.4.9`（本地源码）

### 运行步骤

1. 用 UE 5.7 打开 `GraduationProject.uproject`，编译并启动 PIE
2. 在终端中运行 YOLO 视觉制导：`python yolo_guidance.py`

**快捷键：**

| 按键            | 功能                                              |
| --------------- | ------------------------------------------------- |
| `q`             | 退出                                              |
| `f`             | 手动开火                                          |
| `m`             | 切换制导模式 (predictive → proportional → direct) |
| `1` / `2` / `3` | 切换无人机飞行模式 (直线 / 曲线 / 规避)           |
| `p`             | 显示/隐藏预测线                                   |
| `t`             | 开关跟踪                                          |

## 🎯 制导方法

### Direct Aiming（直接瞄准）

直接对准目标当前位置。最简单但因弹丸飞行时间会导致打在目标后方。

### Proportional Navigation（比例导引）

利用视线角速率（LOS Rate）引导转台旋转，使弹丸飞向目标未来位置。对匀速目标效果较好。

### Predictive Guidance（卡尔曼预测制导）⭐

核心制导算法。使用 6 维卡尔曼滤波器估计目标位置和速度，通过迭代收敛计算提前量，预测弹丸飞行时间内目标的未来位置。对高速机动目标效果最佳。

## 📊 数据采集与训练

### 采集训练数据

```bash
python collect_data.py --num 500        # 采集500个数据
```

### 训练 / 微调模型

```bash
python train_yolo.py                    # 自动加载上次的 best.pt 继续训练
python train_yolo.py --model yolo11n.pt # 从官方预训练权重重新训练
python train_yolo.py --epochs 150       # 指定训练轮数
```

---

## 🔧 SimClient API 参考

Python 客户端通过 TCP（默认端口 `9000`）以 JSON 格式与 UE 引擎实时通信。
所有接口封装在 `SimClient` 类中，每个方法对应一条 TCP JSON 消息。

```python
from sim_client import SimClient
client = SimClient(host="127.0.0.1", port=9000, timeout=10.0)
```

---

### 基础命令

#### `ping()`

测试连接是否正常。

```json
{ "ping": {} }
```

#### `get_agents()` → `List[str]`

获取场景中所有 Agent 列表。

```json
{ "get_agent_list": {} }
```

#### `sim_pause()` / `sim_resume()` / `sim_reset()`

暂停 / 恢复 / 重置仿真。

```json
{"sim_pause": {}}
{"sim_resume": {}}
{"sim_reset": {}}
```

---

### 无人机控制

#### `drone_takeoff(altitude=3.0)`

起飞到指定高度（米）。

```json
{ "call_drone": { "function": "takeoff", "altitude": 5.0 } }
```

#### `drone_land()`

降落到地面。

```json
{ "call_drone": { "function": "land" } }
```

#### `drone_hover()`

原地悬停。

```json
{ "call_drone": { "function": "hover" } }
```

#### `drone_move_to(x, y, z, speed=2.0)`

飞往指定世界坐标（米），`speed` 为飞行速度（m/s）。

```json
{
  "call_drone": {
    "function": "move_to_position",
    "x": 10,
    "y": 20,
    "z": 5,
    "speed": 3.0
  }
}
```

#### `drone_move_by_velocity(vx, vy, vz)`

以指定速度向量持续飞行（m/s）。

```json
{
  "call_drone": { "function": "move_by_velocity", "vx": 2.0, "vy": 0, "vz": 0 }
}
```

#### `drone_state(drone_id="drone_0")` → `dict`

获取无人机完整状态（位置、速度、姿态等）。

```json
{ "get_drone_state": { "id": "drone_0" } }
```

#### `drone_position(drone_id="drone_0")` → `np.ndarray`

获取无人机位置 `[x, y, z]`，封装自 `drone_state()`。

---

### 转台控制

#### `turret_set_angles(pitch, yaw, turret_id="turret_0")`

直接设置转台俯仰角和偏航角（度）。

```json
{
  "call_turret": {
    "function": "set_angles",
    "id": "turret_0",
    "pitch": 10.0,
    "yaw": 45.0
  }
}
```

#### `turret_fire(speed=400.0, turret_id="turret_0")`

以指定炮口初速（m/s）开火。

```json
{ "call_turret": { "function": "fire", "id": "turret_0", "speed": 400.0 } }
```

#### `turret_start_tracking(target_id, turret_id="turret_0")`

开始自动跟踪指定目标。

```json
{
  "call_turret": {
    "function": "start_tracking",
    "id": "turret_0",
    "target_id": "drone_0"
  }
}
```

#### `turret_stop_tracking(turret_id="turret_0")`

停止自动跟踪。

```json
{ "call_turret": { "function": "stop_tracking", "id": "turret_0" } }
```

#### `turret_state(turret_id="turret_0")` → `dict`

获取转台状态（当前角度、位置等）。

```json
{ "get_turret_state": { "id": "turret_0" } }
```

#### `turret_reset(turret_id="turret_0")`

重置转台到初始状态。

```json
{ "call_turret": { "function": "reset", "id": "turret_0" } }
```

---

### 制导系统

#### `guidance_set_method(method="predictive")`

切换制导算法。可选值：`"direct"` / `"proportional"` / `"predictive"`。

```json
{ "call_guidance": { "function": "set_method", "method": "predictive" } }
```

#### ⭐ `guidance_auto_engage(turret_id, target_id, muzzle_speed=400.0, fire=False, dt=0.05)`

**核心接口**。一次调用完成完整制导流程：  
读取目标真实坐标 → 更新卡尔曼滤波器 → 计算瞄准角度 → 设置转台。  
`dt` 参数应传入**真实的帧间时间差**，否则卡尔曼测速会出错。

```json
{
  "call_guidance": {
    "function": "auto_engage",
    "turret_id": "turret_0",
    "target_id": "drone_0",
    "muzzle_speed": 400.0,
    "fire": false,
    "dt": 0.033
  }
}
```

#### `guidance_compute_aim(turret_id, muzzle_speed=400.0)` → `dict`

仅计算瞄准角度（不设置转台），返回 `{pitch, yaw}`。

```json
{
  "call_guidance": {
    "function": "compute_aim",
    "turret_id": "turret_0",
    "muzzle_speed": 400.0
  }
}
```

#### `guidance_update_target(x, y, z, dt=0.1)`

手动向卡尔曼滤波器输入目标坐标。

```json
{
  "call_guidance": {
    "function": "update_target",
    "x": 100,
    "y": 200,
    "z": 500,
    "dt": 0.05
  }
}
```

#### `guidance_set_kalman(process_noise=1.0, measurement_noise=0.5)`

动态调整卡尔曼滤波器的过程噪声（Q）和测量噪声（R）。

```json
{
  "call_guidance": {
    "function": "set_kalman_params",
    "process_noise": 100.0,
    "measurement_noise": 0.01
  }
}
```

#### `guidance_state()` → `dict`

获取当前制导系统状态。

```json
{ "get_guidance_state": {} }
```

#### `guidance_reset()`

重置制导系统，清空卡尔曼滤波器状态。

```json
{ "call_guidance": { "function": "reset" } }
```

---

### 图像获取

#### `get_image_numpy()` → `Optional[np.ndarray]`

获取转台摄像头画面，返回 OpenCV BGR 格式的 numpy 数组。

#### `get_image_base64()` → `Optional[str]`

获取 Base64 编码的 JPEG 图像字符串。

#### `get_image_bytes()` → `Optional[bytes]`

获取 JPEG 图像原始字节。

以上三个方法均通过同一 TCP 消息获取：

```json
{ "get_image": {} }
```

返回值中除图像数据外，还包含相机参数元数据（`camera_pos`、`camera_rot`、`fov`、`width`、`height`），可用于 3D→2D 投影计算。

---

## 📄 许可证

本项目为毕业设计作品，仅供学术研究使用。

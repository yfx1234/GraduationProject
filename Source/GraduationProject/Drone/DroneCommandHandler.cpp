/**
 * @file DroneCommandHandler.cpp
 * @brief 无人机 TCP 命令处理器的实现文件
 *
 * 解析 call_drone / get_drone_state JSON 命令，
 * 通过 AgentManager 查找目标无人机的 DroneApi，委托执行操作。
 */

#include "DroneCommandHandler.h"
#include "DronePawn.h"
#include "DroneApi.h"
#include "DroneMovementComponent.h"
#include "GraduationProject/Core/Manager/AgentManager.h"
#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"

// ---- 工具方法 ----

/** @brief 构造错误响应 JSON */
FString UDroneCommandHandler::MakeError(const FString& Msg)
{
    return FString::Printf(TEXT("{\"status\":\"error\",\"message\":\"%s\"}"), *Msg);
}

/** @brief 构造成功响应 JSON */
FString UDroneCommandHandler::MakeOk(const FString& Msg)
{
    return FString::Printf(TEXT("{\"status\":\"ok\",\"message\":\"%s\"}"), *Msg);
}

/**
 * @brief 通过 Agent ID 查找 DroneApi 实例
 * @param DroneId 无人机 ID
 * @param World 当前 World（当前未使用）
 * @return DroneApi 指针，未找到返回 nullptr
 *
 * 查找路径：AgentManager → DronePawn → DroneApi (Api成员)
 */
UDroneApi* UDroneCommandHandler::FindDroneApi(const FString& DroneId, UWorld* World)
{
    UAgentManager* Manager = UAgentManager::GetInstance();
    AActor* Agent = Manager->GetAgent(DroneId);
    ADronePawn* Drone = Cast<ADronePawn>(Agent);
    if (Drone && Drone->Api)
    {
        return Drone->Api;
    }
    return nullptr;
}

// ---- call_drone 命令处理 ----

/**
 * @brief 处理 call_drone 命令
 * @param JsonObject 已解析的完整 JSON
 * @param World 当前 World
 * @return JSON 响应
 *
 * 解析流程：
 * 1. 从 "call_drone" 字段提取子对象
 * 2. 读取 "function" 字段确定具体操作
 * 3. 读取 "id" 字段（默认 "drone_0"）查找 DroneApi
 * 4. 根据 function 值分发处理
 */
FString UDroneCommandHandler::HandleCallDrone(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World)
{
    const TSharedPtr<FJsonObject>* CmdObj = nullptr;
    if (!JsonObject->TryGetObjectField(TEXT("call_drone"), CmdObj))
    {
        return MakeError(TEXT("Missing call_drone field"));
    }

    // 读取操作函数名
    FString Function;
    (*CmdObj)->TryGetStringField(TEXT("function"), Function);

    // 读取无人机 ID（默认 drone_0）
    FString DroneId;
    if (!(*CmdObj)->TryGetStringField(TEXT("id"), DroneId))
        DroneId = TEXT("drone_0");

    // 查找目标无人机的 API
    UDroneApi* Api = FindDroneApi(DroneId, World);
    if (!Api)
    {
        return MakeError(FString::Printf(TEXT("Drone '%s' not found"), *DroneId));
    }

    // ---- 命令分发 ----

    // 位置控制：移动到指定坐标 (SI, 米)
    if (Function == TEXT("move_to_position"))
    {
        float X = (*CmdObj)->GetNumberField(TEXT("x"));
        float Y = (*CmdObj)->GetNumberField(TEXT("y"));
        float Z = (*CmdObj)->GetNumberField(TEXT("z"));
        float Speed = 2.0f;
        if ((*CmdObj)->HasField(TEXT("speed")))
            Speed = (*CmdObj)->GetNumberField(TEXT("speed"));
        Api->MoveToPosition(X, Y, Z, Speed);
        return MakeOk(FString::Printf(TEXT("move_to_position(%.1f, %.1f, %.1f)"), X, Y, Z));
    }

    // 悬停：保持当前位置
    if (Function == TEXT("hover"))
    {
        Api->Hover();
        return MakeOk(TEXT("hovering"));
    }

    // 起飞：上升到指定高度
    if (Function == TEXT("takeoff"))
    {
        float Altitude = 3.0f;
        if ((*CmdObj)->HasField(TEXT("altitude")))
            Altitude = (*CmdObj)->GetNumberField(TEXT("altitude"));
        Api->Takeoff(Altitude);
        return MakeOk(FString::Printf(TEXT("takeoff to %.1f m"), Altitude));
    }

    // 降落：下降到地面
    if (Function == TEXT("land"))
    {
        Api->Land();
        return MakeOk(TEXT("landing"));
    }

    // 速度控制：按给定速度飞行
    if (Function == TEXT("move_by_velocity"))
    {
        float Vx = (*CmdObj)->GetNumberField(TEXT("vx"));
        float Vy = (*CmdObj)->GetNumberField(TEXT("vy"));
        float Vz = (*CmdObj)->GetNumberField(TEXT("vz"));
        Api->MoveByVelocity(Vx, Vy, Vz);
        return MakeOk(FString::Printf(TEXT("move_by_velocity(%.1f, %.1f, %.1f)"), Vx, Vy, Vz));
    }

    // PID 参数调整：位置控制器
    if (Function == TEXT("set_pid_position"))
    {
        float Kp = (*CmdObj)->GetNumberField(TEXT("kp"));
        float Kd = (*CmdObj)->HasField(TEXT("kd")) ? (*CmdObj)->GetNumberField(TEXT("kd")) : 0.0f;
        Api->SetPositionControllerGains(Kp, Kd);
        return MakeOk(FString::Printf(TEXT("position PD: Kp=%.2f Kd=%.2f"), Kp, Kd));
    }

    // PID 参数调整：速度控制器
    if (Function == TEXT("set_pid_velocity"))
    {
        float Kp = (*CmdObj)->GetNumberField(TEXT("kp"));
        float Ki = (*CmdObj)->HasField(TEXT("ki")) ? (*CmdObj)->GetNumberField(TEXT("ki")) : 0.0f;
        float Kd = (*CmdObj)->HasField(TEXT("kd")) ? (*CmdObj)->GetNumberField(TEXT("kd")) : 0.0f;
        Api->SetVelocityControllerGains(Kp, Ki, Kd);
        return MakeOk(FString::Printf(TEXT("velocity PID: Kp=%.2f Ki=%.2f Kd=%.2f"), Kp, Ki, Kd));
    }

    // PID 参数调整：姿态控制器
    if (Function == TEXT("set_pid_attitude"))
    {
        float Kp = (*CmdObj)->GetNumberField(TEXT("kp"));
        float Kd = (*CmdObj)->HasField(TEXT("kd")) ? (*CmdObj)->GetNumberField(TEXT("kd")) : 0.0f;
        Api->SetAttitudeControllerGains(Kp, Kd);
        return MakeOk(FString::Printf(TEXT("attitude PD: Kp=%.2f Kd=%.2f"), Kp, Kd));
    }

    // PID 参数调整：角速率控制器
    if (Function == TEXT("set_pid_angle_rate"))
    {
        float Kp = (*CmdObj)->GetNumberField(TEXT("kp"));
        Api->SetAngleRateControllerGains(Kp);
        return MakeOk(FString::Printf(TEXT("angle_rate P: Kp=%.2f"), Kp));
    }

    // 重置无人机
    if (Function == TEXT("reset"))
    {
        Api->Reset();
        return MakeOk(TEXT("drone reset"));
    }

    return MakeError(FString::Printf(TEXT("Unknown function: %s"), *Function));
}

// ---- get_drone_state 命令处理 ----

/**
 * @brief 处理 get_drone_state 命令
 * @param JsonObject 已解析的完整 JSON
 * @param World 当前 World
 * @return JSON 响应，包含完整的无人机状态信息
 *
 * 返回字段：
 * - id: 无人机 ID
 * - position: [x, y, z] 位置 (m)
 * - velocity: [vx, vy, vz] 速度 (m/s)
 * - orientation: {roll, pitch, yaw} 姿态 (度)
 * - motor_speeds: [ω0, ω1, ω2, ω3] 电机转速 (rad/s)
 * - control_mode: 控制模式名称
 */
FString UDroneCommandHandler::HandleGetDroneState(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World)
{
    const TSharedPtr<FJsonObject>* StateObj = nullptr;
    if (!JsonObject->TryGetObjectField(TEXT("get_drone_state"), StateObj))
    {
        return MakeError(TEXT("Missing get_drone_state field"));
    }

    // 读取无人机 ID
    FString DroneId;
    if (!(*StateObj)->TryGetStringField(TEXT("id"), DroneId))
        DroneId = TEXT("drone_0");

    // 查找无人机
    UAgentManager* Manager = UAgentManager::GetInstance();
    ADronePawn* Drone = Cast<ADronePawn>(Manager->GetAgent(DroneId));
    if (!Drone)
    {
        return MakeError(FString::Printf(TEXT("Drone '%s' not found"), *DroneId));
    }

    // 获取状态数据
    FVector Pos = Drone->GetCurrentPosition();
    FVector Vel = Drone->GetCurrentVelocity();
    FRotator Rot = Drone->CurrentState.GetRotator();
    TArray<float> Motors;
    if (Drone->Api) Motors = Drone->Api->GetMotorSpeeds();

    // 获取控制模式名称
    FString ModeName;
    switch (Drone->ControlMode)
    {
    case EDroneControlMode::Idle: ModeName = TEXT("idle"); break;
    case EDroneControlMode::Position: ModeName = TEXT("position"); break;
    case EDroneControlMode::Velocity: ModeName = TEXT("velocity"); break;
    case EDroneControlMode::AttitudeThrust: ModeName = TEXT("attitude"); break;
    case EDroneControlMode::MotorSpeed: ModeName = TEXT("motor_speed"); break;
    case EDroneControlMode::TorqueThrust: ModeName = TEXT("torque_thrust"); break;
    default: ModeName = TEXT("unknown"); break;
    }

    // 构造 JSON 响应
    return FString::Printf(
        TEXT("{\"status\":\"ok\",\"id\":\"%s\",")
        TEXT("\"position\":[%.4f,%.4f,%.4f],")
        TEXT("\"velocity\":[%.4f,%.4f,%.4f],")
        TEXT("\"orientation\":{\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f},")
        TEXT("\"motor_speeds\":[%.1f,%.1f,%.1f,%.1f],")
        TEXT("\"control_mode\":\"%s\"}"),
        *DroneId,
        Pos.X, Pos.Y, Pos.Z,
        Vel.X, Vel.Y, Vel.Z,
        Rot.Roll, Rot.Pitch, Rot.Yaw,
        Motors.Num() >= 4 ? Motors[0] : 0.0f,
        Motors.Num() >= 4 ? Motors[1] : 0.0f,
        Motors.Num() >= 4 ? Motors[2] : 0.0f,
        Motors.Num() >= 4 ? Motors[3] : 0.0f,
        *ModeName);
}

#include "DroneCommandHandler.h"
#include "DronePawn.h"
#include "DroneApi.h"
#include "DroneMovementComponent.h"
#include "GraduationProject/Core/Manager/AgentManager.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"

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
 * @param World 当前 World
 * @return DroneApi 指针
 */
UDroneApi* UDroneCommandHandler::FindDroneApi(const FString& DroneId, UWorld* World)
{
    UAgentManager* Manager = UAgentManager::GetInstance();
    AActor* Agent = Manager->GetAgent(DroneId);
    ADronePawn* Drone = Cast<ADronePawn>(Agent);
    if (Drone && Drone->Api) return Drone->Api;
    return nullptr;
}

/**
 * @brief 处理 call_drone 命令
 * @param JsonObject 已解析的完整 JSON
 * @param World 当前 World
 * @return JSON 响应
 */
FString UDroneCommandHandler::HandleCallDrone(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World)
{
    const TSharedPtr<FJsonObject>* CmdObj = nullptr;
    if (!JsonObject->TryGetObjectField(TEXT("call_drone"), CmdObj)) return MakeError(TEXT("Missing call_drone field"));
    FString Function;
    (*CmdObj)->TryGetStringField(TEXT("function"), Function);
    FString DroneId;
    if (!(*CmdObj)->TryGetStringField(TEXT("id"), DroneId)) DroneId = TEXT("drone_0");
    UDroneApi* Api = FindDroneApi(DroneId, World);
    if (!Api) return MakeError(FString::Printf(TEXT("Drone '%s' not found"), *DroneId));
    if (Function == TEXT("move_to_position"))
    {
        float X = (*CmdObj)->GetNumberField(TEXT("x"));
        float Y = (*CmdObj)->GetNumberField(TEXT("y"));
        float Z = (*CmdObj)->GetNumberField(TEXT("z"));
        float Speed = 2.0f;
        if ((*CmdObj)->HasField(TEXT("speed"))) Speed = (*CmdObj)->GetNumberField(TEXT("speed"));
        Api->MoveToPosition(X, Y, Z, Speed);
        return MakeOk(FString::Printf(TEXT("move_to_position(%.1f, %.1f, %.1f)"), X, Y, Z));
    }
    if (Function == TEXT("hover"))
    {
        Api->Hover();
        return MakeOk(TEXT("hovering"));
    }
    if (Function == TEXT("takeoff"))
    {
        float Altitude = 3.0f;
        if ((*CmdObj)->HasField(TEXT("altitude"))) Altitude = (*CmdObj)->GetNumberField(TEXT("altitude"));
        Api->Takeoff(Altitude);
        return MakeOk(FString::Printf(TEXT("takeoff to %.1f m"), Altitude));
    }
    if (Function == TEXT("land"))
    {
        Api->Land();
        return MakeOk(TEXT("landing"));
    }
    if (Function == TEXT("move_by_velocity"))
    {
        float Vx = (*CmdObj)->GetNumberField(TEXT("vx"));
        float Vy = (*CmdObj)->GetNumberField(TEXT("vy"));
        float Vz = (*CmdObj)->GetNumberField(TEXT("vz"));
        Api->MoveByVelocity(Vx, Vy, Vz);
        return MakeOk(FString::Printf(TEXT("move_by_velocity(%.1f, %.1f, %.1f)"), Vx, Vy, Vz));
    }
    if (Function == TEXT("set_pid_position"))
    {
        float Kp = (*CmdObj)->GetNumberField(TEXT("kp"));
        float Kd = (*CmdObj)->HasField(TEXT("kd")) ? (*CmdObj)->GetNumberField(TEXT("kd")) : 0.0f;
        Api->SetPositionControllerGains(Kp, Kd);
        return MakeOk(FString::Printf(TEXT("position PD: Kp=%.2f Kd=%.2f"), Kp, Kd));
    }
    if (Function == TEXT("set_pid_velocity"))
    {
        float Kp = (*CmdObj)->GetNumberField(TEXT("kp"));
        float Ki = (*CmdObj)->HasField(TEXT("ki")) ? (*CmdObj)->GetNumberField(TEXT("ki")) : 0.0f;
        float Kd = (*CmdObj)->HasField(TEXT("kd")) ? (*CmdObj)->GetNumberField(TEXT("kd")) : 0.0f;
        Api->SetVelocityControllerGains(Kp, Ki, Kd);
        return MakeOk(FString::Printf(TEXT("velocity PID: Kp=%.2f Ki=%.2f Kd=%.2f"), Kp, Ki, Kd));
    }
    if (Function == TEXT("set_pid_attitude"))
    {
        float Kp = (*CmdObj)->GetNumberField(TEXT("kp"));
        float Kd = (*CmdObj)->HasField(TEXT("kd")) ? (*CmdObj)->GetNumberField(TEXT("kd")) : 0.0f;
        Api->SetAttitudeControllerGains(Kp, Kd);
        return MakeOk(FString::Printf(TEXT("attitude PD: Kp=%.2f Kd=%.2f"), Kp, Kd));
    }
    if (Function == TEXT("set_pid_angle_rate"))
    {
        float Kp = (*CmdObj)->GetNumberField(TEXT("kp"));
        Api->SetAngleRateControllerGains(Kp);
        return MakeOk(FString::Printf(TEXT("angle_rate P: Kp=%.2f"), Kp));
    }
    if (Function == TEXT("set_camera_angles"))
    {
        float Pitch = (*CmdObj)->GetNumberField(TEXT("pitch"));
        float Yaw = (*CmdObj)->GetNumberField(TEXT("yaw"));
        ADronePawn* Drone = Cast<ADronePawn>(UAgentManager::GetInstance()->GetAgent(DroneId));
        if (Drone) Drone->SetCameraAngles(Pitch, Yaw);
        return MakeOk(FString::Printf(TEXT("set_camera_angles pitch=%.1f yaw=%.1f"), Pitch, Yaw));
    }
    if (Function == TEXT("reset"))
    {
        Api->Reset();
        return MakeOk(TEXT("drone reset"));
    }
    return MakeError(FString::Printf(TEXT("Unknown function: %s"), *Function));
}

/**
 * @brief 处理 get_drone_state 命令
 * @param JsonObject 已解析的完整 JSON
 * @param World 当前 World
 * @return JSON 响应
 */
FString UDroneCommandHandler::HandleGetDroneState(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World)
{
    const TSharedPtr<FJsonObject>* StateObj = nullptr;
    if (!JsonObject->TryGetObjectField(TEXT("get_drone_state"), StateObj)) return MakeError(TEXT("Missing get_drone_state field"));
    FString DroneId;
    if (!(*StateObj)->TryGetStringField(TEXT("id"), DroneId)) DroneId = TEXT("drone_0");
    UAgentManager* Manager = UAgentManager::GetInstance();
    ADronePawn* Drone = Cast<ADronePawn>(Manager->GetAgent(DroneId));
    if (!Drone) return MakeError(FString::Printf(TEXT("Drone '%s' not found"), *DroneId));
    FVector Pos = Drone->GetCurrentPosition();
    FVector Vel = Drone->GetCurrentVelocity();
    FRotator Rot = Drone->CurrentState.GetRotator();
    TArray<float> Motors;
    if (Drone->Api) Motors = Drone->Api->GetMotorSpeeds();
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
    return FString::Printf(
        TEXT("{\"status\":\"ok\",\"id\":\"%s\",")
        TEXT("\"position\":[%.4f,%.4f,%.4f],")
        TEXT("\"velocity\":[%.4f,%.4f,%.4f],")
        TEXT("\"orientation\":{\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f},")
        TEXT("\"motor_speeds\":[%.1f,%.1f,%.1f,%.1f],")
        TEXT("\"camera_pitch\":%.2f,\"camera_yaw\":%.2f,")
        TEXT("\"control_mode\":\"%s\"}"),
        *DroneId,
        Pos.X, Pos.Y, Pos.Z,
        Vel.X, Vel.Y, Vel.Z,
        Rot.Roll, Rot.Pitch, Rot.Yaw,
        Motors.Num() >= 4 ? Motors[0] : 0.0f,
        Motors.Num() >= 4 ? Motors[1] : 0.0f,
        Motors.Num() >= 4 ? Motors[2] : 0.0f,
        Motors.Num() >= 4 ? Motors[3] : 0.0f,
        Drone->GetCameraCurrentPitch(), Drone->GetCameraCurrentYaw(),
        *ModeName);
}

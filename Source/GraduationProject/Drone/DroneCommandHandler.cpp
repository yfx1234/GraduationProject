#include "DroneCommandHandler.h"
#include "DroneApi.h"
#include "DronePawn.h"
#include "GraduationProject/Core/Manager/AgentManager.h"
#include "Dom/JsonObject.h"

FString UDroneCommandHandler::HandleCallDrone(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World)
{
    const TSharedPtr<FJsonObject>* CmdObj = nullptr;
    if (!JsonObject->TryGetObjectField(TEXT("call_drone"), CmdObj))
    {
        return MakeError(TEXT("Missing call_drone field"));
    }

    FString Function;
    (*CmdObj)->TryGetStringField(TEXT("function"), Function);

    FString DroneId;
    if (!(*CmdObj)->TryGetStringField(TEXT("id"), DroneId))
        DroneId = TEXT("drone_0"); // 默认

    UDroneApi* Api = FindDroneApi(DroneId, World);
    if (!Api)
    {
        return MakeError(FString::Printf(TEXT("Drone '%s' not found"), *DroneId));
    }

    // ---- 命令分发 ----
    if (Function == TEXT("move_to_position"))
    {
        float X = (*CmdObj)->GetNumberField(TEXT("x"));
        float Y = (*CmdObj)->GetNumberField(TEXT("y"));
        float Z = (*CmdObj)->GetNumberField(TEXT("z"));
        float Speed = (*CmdObj)->HasField(TEXT("speed")) ? (*CmdObj)->GetNumberField(TEXT("speed")) : 2.0f;
        Api->MoveToPosition(X, Y, Z, Speed);
        return MakeOk(FString::Printf(TEXT("move_to_position(%.1f,%.1f,%.1f)"), X, Y, Z));
    }

    if (Function == TEXT("move_by_velocity"))
    {
        float Vx = (*CmdObj)->GetNumberField(TEXT("vx"));
        float Vy = (*CmdObj)->GetNumberField(TEXT("vy"));
        float Vz = (*CmdObj)->GetNumberField(TEXT("vz"));
        Api->MoveByVelocity(Vx, Vy, Vz);
        return MakeOk(TEXT("move_by_velocity"));
    }

    if (Function == TEXT("takeoff"))
    {
        float Alt = (*CmdObj)->HasField(TEXT("altitude")) ? (*CmdObj)->GetNumberField(TEXT("altitude")) : 3.0f;
        Api->Takeoff(Alt);
        return MakeOk(FString::Printf(TEXT("takeoff(%.1f)"), Alt));
    }

    if (Function == TEXT("land"))
    {
        Api->Land();
        return MakeOk(TEXT("land"));
    }

    if (Function == TEXT("hover"))
    {
        Api->Hover();
        return MakeOk(TEXT("hover"));
    }

    if (Function == TEXT("reset"))
    {
        Api->Reset();
        return MakeOk(TEXT("reset"));
    }

    if (Function == TEXT("set_pid"))
    {
        FString Controller;
        (*CmdObj)->TryGetStringField(TEXT("controller"), Controller);
        float Kp = (*CmdObj)->HasField(TEXT("kp")) ? (*CmdObj)->GetNumberField(TEXT("kp")) : 0.0f;
        float Ki = (*CmdObj)->HasField(TEXT("ki")) ? (*CmdObj)->GetNumberField(TEXT("ki")) : 0.0f;
        float Kd = (*CmdObj)->HasField(TEXT("kd")) ? (*CmdObj)->GetNumberField(TEXT("kd")) : 0.0f;

        if (Controller == TEXT("position"))
            Api->SetPositionControllerGains(Kp, Kd);
        else if (Controller == TEXT("velocity"))
            Api->SetVelocityControllerGains(Kp, Ki, Kd);
        else if (Controller == TEXT("attitude"))
            Api->SetAttitudeControllerGains(Kp, Kd);
        else if (Controller == TEXT("angle_rate"))
            Api->SetAngleRateControllerGains(Kp);
        else
            return MakeError(FString::Printf(TEXT("Unknown controller: %s"), *Controller));

        return MakeOk(FString::Printf(TEXT("set_pid(%s)"), *Controller));
    }

    return MakeError(FString::Printf(TEXT("Unknown function: %s"), *Function));
}

FString UDroneCommandHandler::HandleGetDroneState(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World)
{
    const TSharedPtr<FJsonObject>* StateObj = nullptr;
    if (!JsonObject->TryGetObjectField(TEXT("get_drone_state"), StateObj))
    {
        return MakeError(TEXT("Missing get_drone_state field"));
    }

    FString DroneId;
    if (!(*StateObj)->TryGetStringField(TEXT("id"), DroneId))
        DroneId = TEXT("drone_0");

    UDroneApi* Api = FindDroneApi(DroneId, World);
    if (!Api)
    {
        return MakeError(FString::Printf(TEXT("Drone '%s' not found"), *DroneId));
    }

    FVector Pos = Api->GetPosition();
    FVector Vel = Api->GetVelocity();
    FRotator Rot = Api->GetOrientation();
    TArray<float> Motors = Api->GetMotorSpeeds();
    EDroneControlMode Mode = Api->GetControlMode();

    // 转模式字符串
    FString ModeStr;
    switch (Mode)
    {
    case EDroneControlMode::Idle: ModeStr = TEXT("idle"); break;
    case EDroneControlMode::Position: ModeStr = TEXT("position"); break;
    case EDroneControlMode::Velocity: ModeStr = TEXT("velocity"); break;
    case EDroneControlMode::AttitudeThrust: ModeStr = TEXT("attitude"); break;
    case EDroneControlMode::MotorSpeed: ModeStr = TEXT("motor_speed"); break;
    default: ModeStr = TEXT("unknown"); break;
    }

    // 构造 JSON 响应
    FString MotorsStr = TEXT("[");
    for (int32 i = 0; i < Motors.Num(); i++)
    {
        MotorsStr += FString::Printf(TEXT("%.1f"), Motors[i]);
        if (i < Motors.Num() - 1) MotorsStr += TEXT(",");
    }
    MotorsStr += TEXT("]");

    return FString::Printf(
        TEXT("{\"status\":\"ok\",\"id\":\"%s\",\"position\":[%.3f,%.3f,%.3f],\"velocity\":[%.3f,%.3f,%.3f],")
        TEXT("\"orientation\":{\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f},\"motors\":%s,\"mode\":\"%s\"}"),
        *DroneId,
        Pos.X, Pos.Y, Pos.Z,
        Vel.X, Vel.Y, Vel.Z,
        Rot.Roll, Rot.Pitch, Rot.Yaw,
        *MotorsStr, *ModeStr);
}

UDroneApi* UDroneCommandHandler::FindDroneApi(const FString& DroneId, UWorld* World)
{
    AActor* Agent = UAgentManager::GetInstance()->GetAgent(DroneId);
    ADronePawn* Drone = Cast<ADronePawn>(Agent);
    return Drone ? Drone->Api : nullptr;
}

FString UDroneCommandHandler::MakeOk(const FString& Msg)
{
    return FString::Printf(TEXT("{\"status\":\"ok\",\"message\":\"%s\"}"), *Msg);
}

FString UDroneCommandHandler::MakeError(const FString& Msg)
{
    return FString::Printf(TEXT("{\"status\":\"error\",\"message\":\"%s\"}"), *Msg);
}

/**
 * @file TurretCommandHandler.cpp
 * @brief 转台 TCP 命令处理器的实现文件
 *
 * 解析 call_turret / get_turret_state JSON 命令，
 * 通过 AgentManager 查找 TurretPawn 并委托执行。
 */

#include "TurretCommandHandler.h"
#include "TurretPawn.h"
#include "GraduationProject/Core/Manager/AgentManager.h"
#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"

// ---- 工具方法 ----

FString UTurretCommandHandler::MakeError(const FString& Msg)
{
    return FString::Printf(TEXT("{\"status\":\"error\",\"message\":\"%s\"}"), *Msg);
}

FString UTurretCommandHandler::MakeOk(const FString& Msg)
{
    return FString::Printf(TEXT("{\"status\":\"ok\",\"message\":\"%s\"}"), *Msg);
}

ATurretPawn* UTurretCommandHandler::FindTurretPawn(const FString& TurretId, UWorld* World)
{
    UAgentManager* Manager = UAgentManager::GetInstance();
    AActor* Agent = Manager->GetAgent(TurretId);
    return Cast<ATurretPawn>(Agent);
}

// ---- call_turret ----

FString UTurretCommandHandler::HandleCallTurret(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World)
{
    const TSharedPtr<FJsonObject>* CmdObj = nullptr;
    if (!JsonObject->TryGetObjectField(TEXT("call_turret"), CmdObj))
    {
        return MakeError(TEXT("Missing call_turret field"));
    }

    FString Function;
    (*CmdObj)->TryGetStringField(TEXT("function"), Function);

    FString TurretId;
    if (!(*CmdObj)->TryGetStringField(TEXT("id"), TurretId))
        TurretId = TEXT("turret_0");

    ATurretPawn* Turret = FindTurretPawn(TurretId, World);
    if (!Turret)
    {
        return MakeError(FString::Printf(TEXT("Turret '%s' not found"), *TurretId));
    }

    // ---- 命令分发 ----

    if (Function == TEXT("set_angles"))
    {
        float Pitch = (*CmdObj)->GetNumberField(TEXT("pitch"));
        float Yaw = (*CmdObj)->GetNumberField(TEXT("yaw"));
        Turret->SetTargetAngles(Pitch, Yaw);
        return MakeOk(FString::Printf(TEXT("set_angles pitch=%.1f yaw=%.1f"), Pitch, Yaw));
    }

    if (Function == TEXT("fire"))
    {
        float Speed = Turret->DefaultMuzzleSpeed;
        if ((*CmdObj)->HasField(TEXT("speed")))
        {
            Speed = (*CmdObj)->GetNumberField(TEXT("speed"));
        }
        Turret->FireX(Speed);
        return MakeOk(FString::Printf(TEXT("fired speed=%.1f"), Speed));
    }

    if (Function == TEXT("start_tracking"))
    {
        FString TargetId;
        if (!(*CmdObj)->TryGetStringField(TEXT("target_id"), TargetId))
        {
            return MakeError(TEXT("Missing target_id"));
        }
        Turret->StartTracking(TargetId);
        return MakeOk(FString::Printf(TEXT("tracking %s"), *TargetId));
    }

    if (Function == TEXT("stop_tracking"))
    {
        Turret->StopTracking();
        return MakeOk(TEXT("tracking stopped"));
    }

    if (Function == TEXT("show_prediction"))
    {
        Turret->ShowPredictionLine();
        return MakeOk(TEXT("prediction line shown"));
    }

    if (Function == TEXT("hide_prediction"))
    {
        Turret->HidePredictionLine();
        return MakeOk(TEXT("prediction line hidden"));
    }

    if (Function == TEXT("reset"))
    {
        Turret->SetTargetAngles(0.0f, 0.0f);
        Turret->StopTracking();
        Turret->HidePredictionLine();
        return MakeOk(TEXT("turret reset"));
    }

    return MakeError(FString::Printf(TEXT("Unknown function: %s"), *Function));
}

// ---- get_turret_state ----

FString UTurretCommandHandler::HandleGetTurretState(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World)
{
    const TSharedPtr<FJsonObject>* StateObj = nullptr;
    if (!JsonObject->TryGetObjectField(TEXT("get_turret_state"), StateObj))
    {
        return MakeError(TEXT("Missing get_turret_state field"));
    }

    FString TurretId;
    if (!(*StateObj)->TryGetStringField(TEXT("id"), TurretId))
        TurretId = TEXT("turret_0");

    ATurretPawn* Turret = FindTurretPawn(TurretId, World);
    if (!Turret)
    {
        return MakeError(FString::Printf(TEXT("Turret '%s' not found"), *TurretId));
    }

    FVector Pos = Turret->GetActorLocation();

    return FString::Printf(
        TEXT("{\"status\":\"ok\",\"id\":\"%s\",\"pitch\":%.2f,\"yaw\":%.2f,")
        TEXT("\"target_pitch\":%.2f,\"target_yaw\":%.2f,\"is_tracking\":%s,")
        TEXT("\"position\":[%.1f,%.1f,%.1f]}"),
        *TurretId,
        Turret->GetCurrentPitch(), Turret->GetCurrentYaw(),
        Turret->GetTargetPitchAngle(), Turret->GetTargetYawAngle(),
        Turret->IsTracking() ? TEXT("true") : TEXT("false"),
        Pos.X, Pos.Y, Pos.Z);
}

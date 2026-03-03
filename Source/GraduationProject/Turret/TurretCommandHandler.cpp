#include "TurretCommandHandler.h"
#include "TurretPawn.h"
#include "GraduationProject/Core/Manager/AgentManager.h"
#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"

/**
 * @brief 生成错误响应 JSON
 * @param Msg 错误描述信息
 * @return {"status":"error","message":"<Msg>"}
 */
FString UTurretCommandHandler::MakeError(const FString& Msg)
{
    return FString::Printf(TEXT("{\"status\":\"error\",\"message\":\"%s\"}"), *Msg);
}

/**
 * @brief 生成成功响应 JSON
 * @param Msg 成功描述信息
 * @return {"status":"ok","message":"<Msg>"}
 */
FString UTurretCommandHandler::MakeOk(const FString& Msg)
{
    return FString::Printf(TEXT("{\"status\":\"ok\",\"message\":\"%s\"}"), *Msg);
}

/**
 * @brief 通过 Agent ID 在 AgentManager 中查找转台 Pawn
 * @param TurretId 转台 Agent ID
 * @param World 当前 UWorld 指针
 * @return ATurretPawn 指针
 */
ATurretPawn* UTurretCommandHandler::FindTurretPawn(const FString& TurretId, UWorld* World)
{
    UAgentManager* Manager = UAgentManager::GetInstance();
    AActor* Agent = Manager->GetAgent(TurretId);
    return Cast<ATurretPawn>(Agent);
}

/**
 * @brief 处理 call_turret 命令
 * @param JsonObject 完整的 JSON 请求对象
 * @param World 当前 UWorld 指针
 * @return JSON 格式的响应字符串
 * 提取 call_turret 子对象
 * 读取 function 和 id 字段
 * 通过 FindTurretPawn() 查找转台实例
 * 根据 function 分发到对应操作
 */
FString UTurretCommandHandler::HandleCallTurret(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World)
{
    const TSharedPtr<FJsonObject>* CmdObj = nullptr;
    if (!JsonObject->TryGetObjectField(TEXT("call_turret"), CmdObj)) return MakeError(TEXT("Missing call_turret field"));
    FString Function;
    (*CmdObj)->TryGetStringField(TEXT("function"), Function);
    FString TurretId;
    if (!(*CmdObj)->TryGetStringField(TEXT("id"), TurretId)) TurretId = TEXT("turret_0");
    ATurretPawn* Turret = FindTurretPawn(TurretId, World);
    if (!Turret) return MakeError(FString::Printf(TEXT("Turret '%s' not found"), *TurretId));
    if (Function == TEXT("set_angles"))         // 设置目标角度
    {
        float Pitch = (*CmdObj)->GetNumberField(TEXT("pitch"));
        float Yaw = (*CmdObj)->GetNumberField(TEXT("yaw"));
        Turret->SetTargetAngles(Pitch, Yaw);
        return MakeOk(FString::Printf(TEXT("set_angles pitch=%.1f yaw=%.1f"), Pitch, Yaw));
    }
    if (Function == TEXT("fire"))               // 发射
    {
        float Speed = Turret->DefaultMuzzleSpeed;
        if ((*CmdObj)->HasField(TEXT("speed"))) Speed = (*CmdObj)->GetNumberField(TEXT("speed"));
        Turret->FireX(Speed);
        return MakeOk(FString::Printf(TEXT("fired speed=%.1f"), Speed));
    }
    if (Function == TEXT("start_tracking"))     // 开始跟踪目标
    {
        FString TargetId;
        if (!(*CmdObj)->TryGetStringField(TEXT("target_id"), TargetId)) return MakeError(TEXT("Missing target_id"));
        Turret->StartTracking(TargetId);
        return MakeOk(FString::Printf(TEXT("tracking %s"), *TargetId));
    }
    if (Function == TEXT("stop_tracking"))      // 停止跟踪
    {
        Turret->StopTracking();
        return MakeOk(TEXT("tracking stopped"));
    }
    if (Function == TEXT("show_prediction"))    // 显示预测弹道线
    {
        Turret->ShowPredictionLine();
        return MakeOk(TEXT("prediction line shown"));
    }
    if (Function == TEXT("hide_prediction"))    // 隐藏预测弹道线
    {
        Turret->HidePredictionLine();
        return MakeOk(TEXT("prediction line hidden"));
    }
    if (Function == TEXT("reset"))              // 重置转台
    {
        Turret->SetTargetAngles(0.0f, 0.0f);
        Turret->StopTracking();
        Turret->HidePredictionLine();
        return MakeOk(TEXT("turret reset"));
    }
    return MakeError(FString::Printf(TEXT("Unknown function: %s"), *Function));
}

/**
 * @brief 处理 get_turret_state 命令
 * @param JsonObject 完整的 JSON 请求对象
 * @param World 当前 UWorld 指针
 * @return JSON 格式的转台状态数据
 * 返回字段：
 * - id: 转台 ID
 * - pitch/yaw: 当前实际角度
 * - target_pitch/target_yaw: 目标角度
 * - is_tracking: 是否正在跟踪
 * - position: [X, Y, Z] 转台世界位置
 */
FString UTurretCommandHandler::HandleGetTurretState(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World)
{
    const TSharedPtr<FJsonObject>* StateObj = nullptr;
    if (!JsonObject->TryGetObjectField(TEXT("get_turret_state"), StateObj)) return MakeError(TEXT("Missing get_turret_state field"));
    FString TurretId;
    if (!(*StateObj)->TryGetStringField(TEXT("id"), TurretId)) TurretId = TEXT("turret_0");
    ATurretPawn* Turret = FindTurretPawn(TurretId, World);
    if (!Turret) return MakeError(FString::Printf(TEXT("Turret '%s' not found"), *TurretId));
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

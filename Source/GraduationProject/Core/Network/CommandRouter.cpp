#include "CommandRouter.h"

#include "Dom/JsonObject.h"
#include "GraduationProject/Core/Manager/AgentManager.h"
#include "GraduationProject/Core/Manager/CommandExecutionManager.h"
#include "GraduationProject/Core/Network/FCommandHandle.h"
#include "GraduationProject/Core/SimGameMode.h"
#include "GraduationProject/Core/Simulation/SensorManager.h"
#include "GraduationProject/Core/Simulation/SimClockService.h"
#include "GraduationProject/Core/Simulation/SimulationRecorder.h"
#include "GraduationProject/Drone/DronePawn.h"
#include "GraduationProject/Guidance/GuidanceActor.h"
#include "GraduationProject/Turret/TurretPawn.h"
#include "Engine/GameInstance.h"
#include "Kismet/GameplayStatics.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

// ──── 匿名命名空间：工具函数 ────
namespace
{
    /** @brief 无人机任务角色枚举转 JSON 字符串 */
    FString DroneRoleToString(EDroneMissionRole Role)
    {
        switch (Role)
        {
        case EDroneMissionRole::Target:
            return TEXT("target");
        case EDroneMissionRole::Interceptor:
            return TEXT("interceptor");
        case EDroneMissionRole::Unknown:
        default:
            return TEXT("unknown");
        }
    }

    /** @brief JSON 字符串转义（反斜杠 + 双引号） */
    FString JsonEscape(const FString& In)
    {
        FString Out = In;
        Out.ReplaceInline(TEXT("\\"), TEXT("\\\\"));
        Out.ReplaceInline(TEXT("\""), TEXT("\\\""));
        return Out;
    }

    /** @brief 检查 JSON 是否包含通用 Actor 命令字段 */
    bool HasGenericActorCommand(const TSharedPtr<FJsonObject>& JsonObject)
    {
        return JsonObject.IsValid() &&
            (JsonObject->HasField(TEXT("add_actor")) ||
             JsonObject->HasField(TEXT("remove_actor")) ||
             JsonObject->HasField(TEXT("call_actor")));
    }
}

// ──── 命令总入口 ────

/**
 * @brief 解析 JSON 命令并分派到对应处理函数
 *
 * 工作流程：
 * 1. 反序列化 JSON
 * 2. 优先检查通用 Actor 命令（add_actor / remove_actor / call_actor）
 * 3. 按字段名分派到专用处理函数
 */
FString UCommandRouter::HandleCommand(const FString& JsonString, UWorld* World)
{
    const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(JsonString);  // 创建 JSON 读取器
    TSharedPtr<FJsonObject> JsonObject;
    if (!FJsonSerializer::Deserialize(Reader, JsonObject) || !JsonObject.IsValid())  // 反序列化失败
    {
        return MakeErrorResponse(TEXT("Invalid JSON"));
    }

    // ── 优先处理通用 Actor 命令 ──
    if (HasGenericActorCommand(JsonObject))
    {
        if (!CommandHandle.IsValid())  // 首次使用时惰性创建
        {
            if (!World)
            {
                return MakeErrorResponse(TEXT("No World"));
            }

            UGameInstance* GameInstance = World->GetGameInstance();
            if (!GameInstance)
            {
                return MakeErrorResponse(TEXT("GameInstance not found"));
            }

            CommandHandle = MakeUnique<FCommandHandle>(GameInstance);  // 创建 FCommandHandle
        }

        const FString GenericActorResponse = CommandHandle->HandleCommand(JsonObject);  // 委托处理
        if (!GenericActorResponse.IsEmpty())
        {
            return GenericActorResponse;
        }
    }

    // ── 按字段名分派到各处理函数 ──

    if (JsonObject->HasField(TEXT("ping")))
    {
        return HandlePing();
    }
    if (JsonObject->HasField(TEXT("sim_pause")))
    {
        return HandleSimPause(World);
    }
    if (JsonObject->HasField(TEXT("sim_resume")))
    {
        return HandleSimResume(World);
    }
    if (JsonObject->HasField(TEXT("sim_reset")))
    {
        return HandleSimReset(World);
    }
    if (JsonObject->HasField(TEXT("sim_get_time")))
    {
        return HandleSimGetTime(World);
    }
    if (JsonObject->HasField(TEXT("sim_set_time_scale")))
    {
        return HandleSimSetTimeScale(JsonObject, World);
    }
    if (JsonObject->HasField(TEXT("sim_step")))
    {
        return HandleSimStep(JsonObject, World);
    }
    if (JsonObject->HasField(TEXT("get_agent_list")))
    {
        return HandleGetAgentList();
    }
    if (JsonObject->HasField(TEXT("get_command_status")))
    {
        return HandleGetCommandStatus(JsonObject);
    }
    if (JsonObject->HasField(TEXT("cancel_command")))
    {
        return HandleCancelCommand(JsonObject);
    }
    if (JsonObject->HasField(TEXT("get_sensor_data")))
    {
        return HandleGetSensorData(JsonObject, World);
    }
    if (JsonObject->HasField(TEXT("recorder_start")))
    {
        return HandleRecorderStart(JsonObject);
    }
    if (JsonObject->HasField(TEXT("recorder_stop")))
    {
        return HandleRecorderStop();
    }
    if (JsonObject->HasField(TEXT("recorder_status")))
    {
        return HandleRecorderStatus();
    }
    if (JsonObject->HasField(TEXT("recorder_record_state")))
    {
        return HandleRecorderRecordState(World);
    }
return MakeErrorResponse(TEXT("Unknown command"));  // 未匹配任何命令
}

// ──── 仿真控制处理 ────

/** @brief 心跳检测，返回 "pong" */
FString UCommandRouter::HandlePing()
{
    return TEXT("{\"status\":\"ok\",\"message\":\"pong\"}");
}

/** @brief 暂停仿真——委托 GameMode */
FString UCommandRouter::HandleSimPause(UWorld* World)
{
    if (!World)
    {
        return MakeErrorResponse(TEXT("No World"));
    }

    ASimGameMode* GameMode = Cast<ASimGameMode>(World->GetAuthGameMode());  // 获取 GameMode
    if (!GameMode)
    {
        return MakeErrorResponse(TEXT("GameMode not found"));
    }

    GameMode->PauseSimulation();
    return MakeOkResponse(TEXT("simulation paused"));
}

/** @brief 恢复仿真 */
FString UCommandRouter::HandleSimResume(UWorld* World)
{
    if (!World)
    {
        return MakeErrorResponse(TEXT("No World"));
    }

    ASimGameMode* GameMode = Cast<ASimGameMode>(World->GetAuthGameMode());
    if (!GameMode)
    {
        return MakeErrorResponse(TEXT("GameMode not found"));
    }

    GameMode->ResumeSimulation();
    return MakeOkResponse(TEXT("simulation resumed"));
}

/** @brief 重置仿真 */
FString UCommandRouter::HandleSimReset(UWorld* World)
{
    if (!World)
    {
        return MakeErrorResponse(TEXT("No World"));
    }

    ASimGameMode* GameMode = Cast<ASimGameMode>(World->GetAuthGameMode());
    if (!GameMode)
    {
        return MakeErrorResponse(TEXT("GameMode not found"));
    }

    GameMode->ResetSimulation();
    return MakeOkResponse(TEXT("simulation reset"));
}

/** @brief 获取仿真时间、实时时间、时间缩放倍率 */
FString UCommandRouter::HandleSimGetTime(UWorld* World)
{
    if (!World)
    {
        return MakeErrorResponse(TEXT("No World"));
    }

    USimClockService* Clock = USimClockService::GetInstance();
    if (!Clock->IsInitialized())
    {
        Clock->Initialize(World);
    }

    return FString::Printf(
        TEXT("{\"status\":\"ok\",\"sim_time\":%.6f,\"wall_time\":%.6f,\"time_scale\":%.3f}"),
        Clock->GetSimTimeSec(World),
        Clock->GetWallTimeSec(),
        Clock->GetTimeScale());
}

/** @brief 设置时间缩放倍率，读取 sim_set_time_scale.time_scale 字段 */
FString UCommandRouter::HandleSimSetTimeScale(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World)
{
    if (!World)
    {
        return MakeErrorResponse(TEXT("No World"));
    }

    const TSharedPtr<FJsonObject>* Obj = nullptr;
    if (!JsonObject->TryGetObjectField(TEXT("sim_set_time_scale"), Obj) || !Obj || !Obj->IsValid())
    {
        return MakeErrorResponse(TEXT("Missing sim_set_time_scale field"));
    }

    const float TimeScale = (*Obj)->HasField(TEXT("time_scale"))
        ? static_cast<float>((*Obj)->GetNumberField(TEXT("time_scale")))
        : 1.0f;

    if (!USimClockService::GetInstance()->IsInitialized())
    {
        USimClockService::GetInstance()->Initialize(World);
    }
    USimClockService::GetInstance()->SetTimeScale(TimeScale, World);
    return MakeOkResponse(FString::Printf(TEXT("time_scale=%.3f"), USimClockService::GetInstance()->GetTimeScale()));
}

/**
 * @brief 单步推进仿真：暂时解除暂停 → Tick 指定次数 → 恢复暂停状态
 * @note 读取 sim_step.steps 和 sim_step.dt 字段
 */
FString UCommandRouter::HandleSimStep(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World)
{
    if (!World)
    {
        return MakeErrorResponse(TEXT("No World"));
    }

    const TSharedPtr<FJsonObject>* Obj = nullptr;
    if (!JsonObject->TryGetObjectField(TEXT("sim_step"), Obj) || !Obj || !Obj->IsValid())
    {
        return MakeErrorResponse(TEXT("Missing sim_step field"));
    }

    const int32 Steps = (*Obj)->HasField(TEXT("steps"))  // 读取步数，默认 1
        ? FMath::Max(1, static_cast<int32>((*Obj)->GetNumberField(TEXT("steps"))))
        : 1;
    const float Dt = (*Obj)->HasField(TEXT("dt"))  // 读取 dt，默认 1/60s
        ? FMath::Max(0.0005f, static_cast<float>((*Obj)->GetNumberField(TEXT("dt"))))
        : (1.0f / 60.0f);

    const bool bWasPaused = UGameplayStatics::IsGamePaused(World);  // 记录原始暂停状态
    UGameplayStatics::SetGamePaused(World, false);                  // 解除暂停
    for (int32 Index = 0; Index < Steps; ++Index)                   // 逻辑步进
    {
        World->Tick(ELevelTick::LEVELTICK_All, Dt);
    }
    UGameplayStatics::SetGamePaused(World, bWasPaused);             // 恢复原状态

    USimClockService* Clock = USimClockService::GetInstance();
    if (!Clock->IsInitialized())
    {
        Clock->Initialize(World);
    }

    return FString::Printf(
        TEXT("{\"status\":\"ok\",\"steps\":%d,\"dt\":%.6f,\"sim_time\":%.6f,\"wall_time\":%.6f,\"time_scale\":%.3f}"),
        Steps,
        Dt,
        Clock->GetSimTimeSec(World),
        Clock->GetWallTimeSec(),
        Clock->GetTimeScale());
}

// ──── 智能体查询 ────

/** @brief 获取全部已注册智能体列表（ID + 类型 + 角色） */
FString UCommandRouter::HandleGetAgentList()
{
    UAgentManager* Manager = UAgentManager::GetInstance();
    const TArray<FString> Ids = Manager->GetAllAgentIds();  // 获取所有 ID

    FString AgentIdsJson = TEXT("[");      // ID 数组
    FString AgentDetailJson = TEXT("[");   // 详细信息数组
    for (int32 Index = 0; Index < Ids.Num(); ++Index)
    {
        const FString& Id = Ids[Index];
        AActor* Agent = Manager->GetAgent(Id);  // 获取 Actor 对象

        // ​── 根据类型确定 type 和 role ──
        FString Type = TEXT("actor");
        FString Role = TEXT("unknown");

        if (const ADronePawn* Drone = Cast<ADronePawn>(Agent))
        {
            Type = TEXT("drone");
            Role = DroneRoleToString(Drone->MissionRole);
        }
        else if (Cast<ATurretPawn>(Agent))
        {
            Type = TEXT("turret");
        }
        else if (Cast<AGuidanceActor>(Agent))
        {
            Type = TEXT("guidance");
        }

        AgentIdsJson += FString::Printf(TEXT("\"%s\""), *JsonEscape(Id));
        AgentDetailJson += FString::Printf(TEXT("{\"id\":\"%s\",\"type\":\"%s\",\"role\":\"%s\"}"), *JsonEscape(Id), *Type, *Role);
        if (Index + 1 < Ids.Num())
        {
            AgentIdsJson += TEXT(",");
            AgentDetailJson += TEXT(",");
        }
    }
    AgentIdsJson += TEXT("]");
    AgentDetailJson += TEXT("]");

    return FString::Printf(TEXT("{\"status\":\"ok\",\"agents\":%s,\"agents_detail\":%s,\"count\":%d}"), *AgentIdsJson, *AgentDetailJson, Ids.Num());
}

// ──── 命令执行管理 ────

/** @brief 查询异步命令的执行状态（可指定超时等待） */
FString UCommandRouter::HandleGetCommandStatus(const TSharedPtr<FJsonObject>& JsonObject)
{
    const TSharedPtr<FJsonObject>* Obj = nullptr;
    if (!JsonObject->TryGetObjectField(TEXT("get_command_status"), Obj) || !Obj || !Obj->IsValid())
    {
        return MakeErrorResponse(TEXT("Missing get_command_status field"));
    }

    FString CommandId;
    if (!(*Obj)->TryGetStringField(TEXT("command_id"), CommandId))
    {
        return MakeErrorResponse(TEXT("Missing command_id"));
    }

    const double TimeoutSec = (*Obj)->HasField(TEXT("timeout_sec"))
        ? FMath::Max(0.0, (*Obj)->GetNumberField(TEXT("timeout_sec")))
        : 0.0;

    FCommandExecutionRecord Record;
    if (!UCommandExecutionManager::GetInstance()->GetCommandWithTimeout(CommandId, TimeoutSec, Record))
    {
        return MakeErrorResponse(FString::Printf(TEXT("Command '%s' not found"), *CommandId));
    }

    const double Duration = (Record.EndTimeSec > Record.StartTimeSec)
        ? (Record.EndTimeSec - Record.StartTimeSec)
        : 0.0;

    return FString::Printf(
        TEXT("{\"status\":\"ok\",\"command_id\":\"%s\",\"agent_id\":\"%s\",\"function\":\"%s\",\"state\":\"%s\",\"message\":\"%s\",\"start_time\":%.6f,\"end_time\":%.6f,\"duration\":%.6f}"),
        *Record.CommandId,
        *Record.AgentId,
        *Record.FunctionName,
        *Record.Status,
        *Record.Message.ReplaceCharWithEscapedChar(),
        Record.StartTimeSec,
        Record.EndTimeSec,
        Duration);
}

/** @brief 取消正在执行的异步命令 */
FString UCommandRouter::HandleCancelCommand(const TSharedPtr<FJsonObject>& JsonObject)
{
    const TSharedPtr<FJsonObject>* Obj = nullptr;
    if (!JsonObject->TryGetObjectField(TEXT("cancel_command"), Obj) || !Obj || !Obj->IsValid())
    {
        return MakeErrorResponse(TEXT("Missing cancel_command field"));
    }

    FString CommandId;
    if (!(*Obj)->TryGetStringField(TEXT("command_id"), CommandId))
    {
        return MakeErrorResponse(TEXT("Missing command_id"));
    }

    FString Reason = TEXT("canceled");
    (*Obj)->TryGetStringField(TEXT("reason"), Reason);

    if (!UCommandExecutionManager::GetInstance()->CancelCommand(CommandId, Reason))
    {
        return MakeErrorResponse(FString::Printf(TEXT("Command '%s' cannot be canceled"), *CommandId));
    }

    return MakeOkResponse(FString::Printf(TEXT("command '%s' canceled"), *CommandId));
}

// ──── 传感器数据 ────

/** @brief 获取指定无人机的传感器状态，帧坐标系可选 "ue" / "ned" 等 */
FString UCommandRouter::HandleGetSensorData(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World)
{
    const TSharedPtr<FJsonObject>* Obj = nullptr;
    if (!JsonObject->TryGetObjectField(TEXT("get_sensor_data"), Obj) || !Obj || !Obj->IsValid())
    {
        return MakeErrorResponse(TEXT("Missing get_sensor_data field"));
    }

    FString DroneId = TEXT("drone_0");
    (*Obj)->TryGetStringField(TEXT("id"), DroneId);

    FString Frame = TEXT("ue");
    (*Obj)->TryGetStringField(TEXT("frame"), Frame);

    return USensorManager::GetInstance()->BuildDroneSensorJson(DroneId, World, Frame);
}

// ──── 录制 ────

/** @brief 开始录制，可指定输出路径 */
FString UCommandRouter::HandleRecorderStart(const TSharedPtr<FJsonObject>& JsonObject)
{
    const TSharedPtr<FJsonObject>* Obj = nullptr;
    if (!JsonObject->TryGetObjectField(TEXT("recorder_start"), Obj) || !Obj || !Obj->IsValid())
    {
        return MakeErrorResponse(TEXT("Missing recorder_start field"));
    }

    FString Path;
    (*Obj)->TryGetStringField(TEXT("path"), Path);

    if (!USimulationRecorder::GetInstance()->Start(Path))
    {
        return MakeErrorResponse(TEXT("recorder start failed"));
    }

    return FString::Printf(TEXT("{\"status\":\"ok\",\"path\":\"%s\"}"), *USimulationRecorder::GetInstance()->GetPath().ReplaceCharWithEscapedChar());
}

/** @brief 停止录制 */
FString UCommandRouter::HandleRecorderStop()
{
    USimulationRecorder::GetInstance()->Stop();
    return MakeOkResponse(TEXT("recorder stopped"));
}

/** @brief 查询录制状态（是否正在录制、路径、已记录条数） */
FString UCommandRouter::HandleRecorderStatus()
{
    USimulationRecorder* Recorder = USimulationRecorder::GetInstance();
    return FString::Printf(
        TEXT("{\"status\":\"ok\",\"recording\":%s,\"path\":\"%s\",\"count\":%lld}"),
        Recorder->IsRecording() ? TEXT("true") : TEXT("false"),
        *Recorder->GetPath().ReplaceCharWithEscapedChar(),
        Recorder->GetRecordCount());
}

/** @brief 立即记录一帧全部智能体状态到录制文件 */
FString UCommandRouter::HandleRecorderRecordState(UWorld* World)
{
    if (!World)
    {
        return MakeErrorResponse(TEXT("No World"));
    }

    USimulationRecorder* Recorder = USimulationRecorder::GetInstance();
    if (!Recorder->IsRecording())
    {
        return MakeErrorResponse(TEXT("recorder is not running"));
    }

    UAgentManager* Manager = UAgentManager::GetInstance();
    FString Payload = TEXT("{\"agents\":[");

    const TArray<FString> Ids = Manager->GetAllAgentIds();
    for (int32 Index = 0; Index < Ids.Num(); ++Index)
    {
        const FString& Id = Ids[Index];
        if (ADronePawn* Drone = Cast<ADronePawn>(Manager->GetAgent(Id)))
        {
            const FVector Pos = Drone->GetCurrentPosition();
            const FVector Vel = Drone->GetCurrentVelocity();
            Payload += FString::Printf(TEXT("{\"id\":\"%s\",\"type\":\"drone\",\"position\":[%.4f,%.4f,%.4f],\"velocity\":[%.4f,%.4f,%.4f]}"), *Id, Pos.X, Pos.Y, Pos.Z, Vel.X, Vel.Y, Vel.Z);
        }
        else if (ATurretPawn* Turret = Cast<ATurretPawn>(Manager->GetAgent(Id)))
        {
            Payload += FString::Printf(TEXT("{\"id\":\"%s\",\"type\":\"turret\",\"pitch\":%.3f,\"yaw\":%.3f}"), *Id, Turret->GetCurrentPitch(), Turret->GetCurrentYaw());
        }
        else if (Cast<AGuidanceActor>(Manager->GetAgent(Id)))
        {
            Payload += FString::Printf(TEXT("{\"id\":\"%s\",\"type\":\"guidance\"}"), *Id);
        }
        else
        {
            Payload += FString::Printf(TEXT("{\"id\":\"%s\",\"type\":\"actor\"}"), *Id);
        }

        if (Index + 1 < Ids.Num())
        {
            Payload += TEXT(",");
        }
    }
    Payload += TEXT("]}");

    Recorder->RecordJsonLine(TEXT("state"), Payload);
    return MakeOkResponse(TEXT("state recorded"));
}

// ──── 响应工具 ────

/** @brief 生成错误 JSON 响应 */
FString UCommandRouter::MakeErrorResponse(const FString& Error)
{
    return FString::Printf(TEXT("{\"status\":\"error\",\"message\":\"%s\"}"), *JsonEscape(Error));
}

/** @brief 生成成功 JSON 响应 */
FString UCommandRouter::MakeOkResponse(const FString& Message)
{
    return FString::Printf(TEXT("{\"status\":\"ok\",\"message\":\"%s\"}"), *JsonEscape(Message));
}




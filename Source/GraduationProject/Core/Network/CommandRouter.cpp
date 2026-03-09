#include "CommandRouter.h"
#include "GraduationProject/Core/Manager/AgentManager.h"
#include "GraduationProject/Core/SimGameMode.h"
#include "GraduationProject/Drone/DroneCommandHandler.h"
#include "GraduationProject/Drone/DronePawn.h"
#include "GraduationProject/Turret/TurretCommandHandler.h"
#include "GraduationProject/Guidance/GuidanceCommandHandler.h"
#include "GraduationProject/Turret/TurretPawn.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"

/**
 * @brief 处理 TCP 命令
 * @param JsonString 收到的 JSON 字符串
 * @param World 当前 UWorld 指针
 * @return 响应 JSON 字符串
 */
FString UCommandRouter::HandleCommand(const FString& JsonString, UWorld* World)
{
    TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(JsonString);    
    TSharedPtr<FJsonObject> JsonObject;
    if (!FJsonSerializer::Deserialize(Reader, JsonObject) || !JsonObject.IsValid())
    {
        return MakeErrorResponse(TEXT("Invalid JSON"));
    }
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
    if (JsonObject->HasField(TEXT("get_agent_list")))
    {
        return HandleGetAgentList();
    }
    if (JsonObject->HasField(TEXT("get_image")))
    {
        return HandleGetImage(JsonObject, World);
    }
    if (JsonObject->HasField(TEXT("call_drone")))
    {
        if (!DroneHandler) DroneHandler = NewObject<UDroneCommandHandler>(this);
        return DroneHandler->HandleCallDrone(JsonObject, World);
    }
    if (JsonObject->HasField(TEXT("get_drone_state")))
    {
        if (!DroneHandler) DroneHandler = NewObject<UDroneCommandHandler>(this);
        return DroneHandler->HandleGetDroneState(JsonObject, World);
    }
    if (JsonObject->HasField(TEXT("call_turret")))
    {
        if (!TurretHandler) TurretHandler = NewObject<UTurretCommandHandler>(this);
        return TurretHandler->HandleCallTurret(JsonObject, World);
    }
    if (JsonObject->HasField(TEXT("get_turret_state")))
    {
        if (!TurretHandler) TurretHandler = NewObject<UTurretCommandHandler>(this);
        return TurretHandler->HandleGetTurretState(JsonObject, World);
    }
    if (JsonObject->HasField(TEXT("call_guidance")))
    {
        if (!GuidanceHandler) GuidanceHandler = NewObject<UGuidanceCommandHandler>(this);
        return GuidanceHandler->HandleCallGuidance(JsonObject, World);
    }
    if (JsonObject->HasField(TEXT("get_guidance_state")))
    {
        if (!GuidanceHandler) GuidanceHandler = NewObject<UGuidanceCommandHandler>(this);
        return GuidanceHandler->HandleGetGuidanceState(JsonObject, World);
    }
    return MakeErrorResponse(TEXT("Unknown command"));
}

/**
 * @brief 心跳检测响应
 * @return 返回 pong 消息，客户端用于确认连接正常
 */
FString UCommandRouter::HandlePing()
{
    return TEXT("{\"status\": \"ok\", \"message\": \"pong\"}");
}

/**
 * @brief 暂停仿真
 * @param World 当前 World
 * @return JSON 响应
 */
FString UCommandRouter::HandleSimPause(UWorld* World)
{
    if (!World) return MakeErrorResponse(TEXT("No World"));
    ASimGameMode* GameMode = Cast<ASimGameMode>(World->GetAuthGameMode());
    if (GameMode)
    {
        GameMode->PauseSimulation();
        return MakeOkResponse(TEXT("simulation paused"));
    }
    return MakeErrorResponse(TEXT("GameMode not found"));
}

/**
 * @brief 恢复仿真
 * @param World 当前 World
 * @return JSON 响应
 */
FString UCommandRouter::HandleSimResume(UWorld* World)
{
    if (!World) return MakeErrorResponse(TEXT("No World"));
    ASimGameMode* GameMode = Cast<ASimGameMode>(World->GetAuthGameMode());
    if (GameMode)
    {
        GameMode->ResumeSimulation();
        return MakeOkResponse(TEXT("simulation resumed"));
    }
    return MakeErrorResponse(TEXT("GameMode not found"));
}

/**
 * @brief 重置仿真
 * @param World 当前 World
 * @return JSON 响应
 */
FString UCommandRouter::HandleSimReset(UWorld* World)
{
    if (!World) return MakeErrorResponse(TEXT("No World"));
    ASimGameMode* GameMode = Cast<ASimGameMode>(World->GetAuthGameMode());
    if (GameMode)
    {
        GameMode->ResetSimulation();
        return MakeOkResponse(TEXT("simulation reset"));
    }
    return MakeErrorResponse(TEXT("GameMode not found"));
}

/**
 * @brief 获取已注册智能体列表
 * @return JSON 响应，格式 {"status":"ok","agents":["drone_0","turret_0"],"count":2}
 */
FString UCommandRouter::HandleGetAgentList()
{
    UAgentManager* Manager = UAgentManager::GetInstance();
    TArray<FString> Ids = Manager->GetAllAgentIds();
    FString IdList = TEXT("[");
    for (int32 i = 0; i < Ids.Num(); i++)
    {
        IdList += FString::Printf(TEXT("\"%s\""), *Ids[i]);
        if (i < Ids.Num() - 1) IdList += TEXT(", ");
    }
    IdList += TEXT("]");
    return FString::Printf(TEXT("{\"status\": \"ok\", \"agents\": %s, \"count\": %d}"),
        *IdList, Ids.Num());
}

/**
 * @brief 构造错误响应 JSON
 * @param Error 错误信息文本
 * @return 格式化的 JSON 错误响应
 */
FString UCommandRouter::MakeErrorResponse(const FString& Error)
{
    return FString::Printf(TEXT("{\"status\": \"error\", \"message\": \"%s\"}"), *Error);
}

/**
 * @brief 构造成功响应 JSON
 * @param Message 成功信息文本
 * @return 格式化的 JSON 成功响应
 */
FString UCommandRouter::MakeOkResponse(const FString& Message)
{
    return FString::Printf(TEXT("{\"status\": \"ok\", \"message\": \"%s\"}"), *Message);
}

/**
 * @brief 获取摄像头图像（支持 Turret 和 Drone）
 * @param JsonObject 已解析的 JSON 对象，可包含 get_image.id 字段指定 Agent
 * @param World 当前 World
 * @return JSON 响应，包含 Base64 编码的 JPEG、摄像头位置、姿态、FOV
 * 优先通过 id 查找指定 Agent（DronePawn 或 TurretPawn），
 * 若无 id 则遍历所有 Agent 查找第一个带摄像头的
 */
FString UCommandRouter::HandleGetImage(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World)
{
    if (!World) return MakeErrorResponse(TEXT("No World"));
    UAgentManager* Manager = UAgentManager::GetInstance();

    // 尝试从 get_image 对象中读取 id
    FString AgentId;
    const TSharedPtr<FJsonObject>* ImgObj = nullptr;
    if (JsonObject->TryGetObjectField(TEXT("get_image"), ImgObj))
    {
        (*ImgObj)->TryGetStringField(TEXT("id"), AgentId);
    }

    // 如果指定了 id，直接查找该 Agent
    if (!AgentId.IsEmpty())
    {
        AActor* Agent = Manager->GetAgent(AgentId);
        if (!Agent) return MakeErrorResponse(FString::Printf(TEXT("Agent '%s' not found"), *AgentId));

        // 尝试作为 DronePawn
        ADronePawn* Drone = Cast<ADronePawn>(Agent);
        if (Drone && Drone->DroneSceneCapture)
        {
            FString Base64 = Drone->CaptureImageBase64(85);
            if (Base64.IsEmpty()) return MakeErrorResponse(TEXT("Capture failed"));
            FVector CamPos = Drone->DroneSceneCapture->GetComponentLocation();
            FRotator CamRot = Drone->DroneSceneCapture->GetComponentRotation();
            return FString::Printf(
                TEXT("{\"status\":\"ok\",\"source\":\"%s\",\"width\":%d,\"height\":%d,\"format\":\"jpeg\",")
                TEXT("\"camera_pos\":[%.2f,%.2f,%.2f],")
                TEXT("\"camera_rot\":[%.2f,%.2f,%.2f],")
                TEXT("\"fov\":%.2f,")
                TEXT("\"data\":\"%s\"}"),
                *AgentId, Drone->CameraWidth, Drone->CameraHeight,
                CamPos.X, CamPos.Y, CamPos.Z,
                CamRot.Pitch, CamRot.Yaw, CamRot.Roll,
                Drone->CameraFOV,
                *Base64);
        }

        // 尝试作为 TurretPawn
        ATurretPawn* Turret = Cast<ATurretPawn>(Agent);
        if (Turret && Turret->TurretSceneCapture)
        {
            FString Base64 = Turret->CaptureImageBase64(85);
            if (Base64.IsEmpty()) return MakeErrorResponse(TEXT("Capture failed"));
            FVector CamPos = Turret->TurretSceneCapture->GetComponentLocation();
            FRotator CamRot = Turret->TurretSceneCapture->GetComponentRotation();
            return FString::Printf(
                TEXT("{\"status\":\"ok\",\"source\":\"%s\",\"width\":%d,\"height\":%d,\"format\":\"jpeg\",")
                TEXT("\"camera_pos\":[%.2f,%.2f,%.2f],")
                TEXT("\"camera_rot\":[%.2f,%.2f,%.2f],")
                TEXT("\"fov\":%.2f,")
                TEXT("\"data\":\"%s\"}"),
                *AgentId, Turret->CameraWidth, Turret->CameraHeight,
                CamPos.X, CamPos.Y, CamPos.Z,
                CamRot.Pitch, CamRot.Yaw, CamRot.Roll,
                Turret->CameraFOV,
                *Base64);
        }

        return MakeErrorResponse(FString::Printf(TEXT("Agent '%s' has no camera"), *AgentId));
    }

    // 未指定 id，遍历所有 Agent 查找第一个带摄像头的（向后兼容）
    TArray<FString> Ids = Manager->GetAllAgentIds();
    for (const FString& Id : Ids)
    {
        AActor* Agent = Manager->GetAgent(Id);

        // 优先检查 DronePawn
        ADronePawn* Drone = Cast<ADronePawn>(Agent);
        if (Drone && Drone->DroneSceneCapture)
        {
            FString Base64 = Drone->CaptureImageBase64(85);
            if (Base64.IsEmpty()) continue;
            FVector CamPos = Drone->DroneSceneCapture->GetComponentLocation();
            FRotator CamRot = Drone->DroneSceneCapture->GetComponentRotation();
            return FString::Printf(
                TEXT("{\"status\":\"ok\",\"source\":\"%s\",\"width\":%d,\"height\":%d,\"format\":\"jpeg\",")
                TEXT("\"camera_pos\":[%.2f,%.2f,%.2f],")
                TEXT("\"camera_rot\":[%.2f,%.2f,%.2f],")
                TEXT("\"fov\":%.2f,")
                TEXT("\"data\":\"%s\"}"),
                *Id, Drone->CameraWidth, Drone->CameraHeight,
                CamPos.X, CamPos.Y, CamPos.Z,
                CamRot.Pitch, CamRot.Yaw, CamRot.Roll,
                Drone->CameraFOV,
                *Base64);
        }

        // 检查 TurretPawn
        ATurretPawn* Turret = Cast<ATurretPawn>(Agent);
        if (Turret && Turret->TurretSceneCapture)
        {
            FString Base64 = Turret->CaptureImageBase64(85);
            if (Base64.IsEmpty()) continue;
            FVector CamPos = Turret->TurretSceneCapture->GetComponentLocation();
            FRotator CamRot = Turret->TurretSceneCapture->GetComponentRotation();
            return FString::Printf(
                TEXT("{\"status\":\"ok\",\"source\":\"%s\",\"width\":%d,\"height\":%d,\"format\":\"jpeg\",")
                TEXT("\"camera_pos\":[%.2f,%.2f,%.2f],")
                TEXT("\"camera_rot\":[%.2f,%.2f,%.2f],")
                TEXT("\"fov\":%.2f,")
                TEXT("\"data\":\"%s\"}"),
                *Id, Turret->CameraWidth, Turret->CameraHeight,
                CamPos.X, CamPos.Y, CamPos.Z,
                CamRot.Pitch, CamRot.Yaw, CamRot.Roll,
                Turret->CameraFOV,
                *Base64);
        }
    }
    return MakeErrorResponse(TEXT("No agent with camera found"));
}

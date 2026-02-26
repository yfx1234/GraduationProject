/**
 * @file CommandRouter.cpp
 * @brief TCP 命令路由器的实现文件
 *
 * 解析 JSON 命令并路由到对应的 Handler。
 * 内置命令（ping、仿真控制、智能体列表、图像获取）在此文件直接实现。
 */

#include "CommandRouter.h"
#include "GraduationProject/Core/Manager/AgentManager.h"
#include "GraduationProject/Core/SimGameMode.h"
#include "GraduationProject/Drone/DroneCommandHandler.h"
#include "GraduationProject/Turret/TurretCommandHandler.h"
#include "GraduationProject/Guidance/GuidanceCommandHandler.h"
#include "GraduationProject/Turret/TurretPawn.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"

/**
 * @brief 处理一条 TCP 命令
 * @param JsonString 收到的 JSON 字符串
 * @param World 当前 UWorld 指针
 * @return 响应 JSON 字符串
 *
 * 处理流程：
 * 1. 解析 JSON 字符串为 FJsonObject
 * 2. 按优先级检查已知字段名，路由到对应处理函数
 * 3. Handler 对象采用懒加载（首次使用时创建）
 * 4. 无法识别的命令返回错误响应
 */
FString UCommandRouter::HandleCommand(const FString& JsonString, UWorld* World)
{
    // 解析 JSON 字符串
    TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(JsonString);
    TSharedPtr<FJsonObject> JsonObject;

    if (!FJsonSerializer::Deserialize(Reader, JsonObject) || !JsonObject.IsValid())
    {
        return MakeErrorResponse(TEXT("Invalid JSON"));
    }

    // ---- 路由逻辑：按字段名匹配命令类型 ----

    // 基础命令：心跳检测
    if (JsonObject->HasField(TEXT("ping")))
    {
        return HandlePing();
    }

    // 仿真控制命令
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

    // 智能体列表查询
    if (JsonObject->HasField(TEXT("get_agent_list")))
    {
        return HandleGetAgentList();
    }

    // 图像获取命令
    if (JsonObject->HasField(TEXT("get_image")))
    {
        return HandleGetImage(World);
    }

    // Drone 命令 — 委托给 DroneCommandHandler（懒加载创建）
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

    // Turret 命令 — 委托给 TurretCommandHandler（懒加载创建）
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

    // Guidance 命令 — 委托给 GuidanceCommandHandler（懒加载创建）
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

    // 未知命令
    return MakeErrorResponse(TEXT("Unknown command"));
}

// ---- 内置命令实现 ----

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
 *
 * 通过 SimGameMode::PauseSimulation() 实现。
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
 *
 * 通过 SimGameMode::ResumeSimulation() 实现。
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
 *
 * 通过 SimGameMode::ResetSimulation() 实现。
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

    // 构造 JSON 数组字符串
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

// ---- 工具方法 ----

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
 * @brief 获取转台摄像头图像
 * @param World 当前 World
 * @return JSON 响应，包含 Base64 编码的 JPEG 和摄像头参数
 *
 * 遍历所有已注册智能体，找到第一个带摄像头的转台 (ATurretPawn)，
 * 采集图像并以 Base64 编码返回，同时附带摄像头位姿和 FOV 信息。
 */
FString UCommandRouter::HandleGetImage(UWorld* World)
{
    if (!World) return MakeErrorResponse(TEXT("No World"));

    UAgentManager* Manager = UAgentManager::GetInstance();
    TArray<FString> Ids = Manager->GetAllAgentIds();

    // 遍历所有智能体，查找带摄像头的转台
    for (const FString& Id : Ids)
    {
        ATurretPawn* Turret = Cast<ATurretPawn>(Manager->GetAgent(Id));
        if (!Turret || !Turret->TurretSceneCapture) continue;

        // 采集一帧 JPEG 图像并 Base64 编码
        FString Base64 = Turret->CaptureImageBase64(85);
        if (Base64.IsEmpty())
        {
            return MakeErrorResponse(TEXT("Capture failed"));
        }

        // 获取摄像头世界坐标变换（位置和旋转）
        FVector CamPos = Turret->TurretSceneCapture->GetComponentLocation();
        FRotator CamRot = Turret->TurretSceneCapture->GetComponentRotation();

        // 构造包含图像数据和摄像头信息的 JSON 响应
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

    return MakeErrorResponse(TEXT("No turret with camera found"));
}

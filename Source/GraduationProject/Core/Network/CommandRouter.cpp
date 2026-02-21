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

FString UCommandRouter::HandleCommand(const FString& JsonString, UWorld* World)
{
    // 解析 JSON
    TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(JsonString);
    TSharedPtr<FJsonObject> JsonObject;

    if (!FJsonSerializer::Deserialize(Reader, JsonObject) || !JsonObject.IsValid())
    {
        return MakeErrorResponse(TEXT("Invalid JSON"));
    }

    // ---- 路由逻辑 ----

    // 基础命令
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
        return HandleGetImage(World);
    }

    // Drone 命令 — 委托给 DroneCommandHandler
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

    // Turret 命令 — 委托给 TurretCommandHandler
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

    // Guidance 命令 — 委托给 GuidanceCommandHandler
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

// ---- 内置命令实现 ----

FString UCommandRouter::HandlePing()
{
    return TEXT("{\"status\": \"ok\", \"message\": \"pong\"}");
}

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

FString UCommandRouter::HandleGetAgentList()
{
    UAgentManager* Manager = UAgentManager::GetInstance();
    TArray<FString> Ids = Manager->GetAllAgentIds();

    // 构造 JSON 数组
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

FString UCommandRouter::MakeErrorResponse(const FString& Error)
{
    return FString::Printf(TEXT("{\"status\": \"error\", \"message\": \"%s\"}"), *Error);
}

FString UCommandRouter::MakeOkResponse(const FString& Message)
{
    return FString::Printf(TEXT("{\"status\": \"ok\", \"message\": \"%s\"}"), *Message);
}

FString UCommandRouter::HandleGetImage(UWorld* World)
{
    if (!World) return MakeErrorResponse(TEXT("No World"));

    UAgentManager* Manager = UAgentManager::GetInstance();
    TArray<FString> Ids = Manager->GetAllAgentIds();

    for (const FString& Id : Ids)
    {
        ATurretPawn* Turret = Cast<ATurretPawn>(Manager->GetAgent(Id));
        if (!Turret || !Turret->TurretSceneCapture) continue;

        FString Base64 = Turret->CaptureImageBase64(85);
        if (Base64.IsEmpty())
        {
            return MakeErrorResponse(TEXT("Capture failed"));
        }

        // 获取摄像头世界变换
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

    return MakeErrorResponse(TEXT("No turret with camera found"));
}


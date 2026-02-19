#include "CommandRouter.h"
#include "GraduationProject/Core/Manager/AgentManager.h"
#include "GraduationProject/Core/SimGameMode.h"
#include "GraduationProject/Drone/DroneCommandHandler.h"
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

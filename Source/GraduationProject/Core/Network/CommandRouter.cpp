
#include "CommandRouter.h"
#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"
#include "GraduationProject/Core/Manager/AgentManager.h"
#include "GraduationProject/Core/Manager/CommandExecutionManager.h"
#include "GraduationProject/Core/Network/FCommandHandle.h"
#include "GraduationProject/Core/SimGameMode.h"
#include "GraduationProject/Core/Simulation/SensorManager.h"
#include "GraduationProject/Core/Simulation/SimClockService.h"
#include "GraduationProject/Core/Simulation/SimulationRecorder.h"
#include "GraduationProject/Drone/DronePawn.h"
#include "GraduationProject/Guidance/GuidanceActor.h"
#include "Engine/GameInstance.h"
#include "Kismet/GameplayStatics.h"
#include "Policies/CondensedJsonPrintPolicy.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
namespace
{
    FString SerializeJsonObject(const TSharedPtr<FJsonObject>& JsonObject)
    {
        if (!JsonObject.IsValid())
        {
            return TEXT("{}");
        }

        FString Output;
        const TSharedRef<TJsonWriter<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>> Writer =
            TJsonWriterFactory<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>::Create(&Output);
        FJsonSerializer::Serialize(JsonObject.ToSharedRef(), Writer);
        return Output;
    }

    TSharedPtr<FJsonObject> MakeStatusResponseObject(const FString& Status, const FString& Message)
    {
        const TSharedPtr<FJsonObject> Response = MakeShared<FJsonObject>();
        Response->SetStringField(TEXT("status"), Status);
        Response->SetStringField(TEXT("message"), Message);
        return Response;
    }

    ASimGameMode* ResolveSimGameMode(UWorld* World, FString& OutError)
    {
        if (!World)
        {
            OutError = TEXT("No World");
            return nullptr;
        }

        if (ASimGameMode* GameMode = Cast<ASimGameMode>(World->GetAuthGameMode()))
        {
            return GameMode;
        }

        OutError = TEXT("GameMode not found");
        return nullptr;
    }

    USimClockService* ResolveClock(UWorld* World, FString& OutError)
    {
        if (!World)
        {
            OutError = TEXT("No World");
            return nullptr;
        }

        USimClockService* Clock = USimClockService::GetInstance();
        if (!Clock)
        {
            OutError = TEXT("Clock service unavailable");
            return nullptr;
        }

        if (!Clock->IsInitialized())
        {
            Clock->Initialize(World);
        }

        return Clock;
    }

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
    FString JsonEscape(const FString& In)
    {
        FString Out = In;
        Out.ReplaceInline(TEXT("\\"), TEXT("\\\\"));
        Out.ReplaceInline(TEXT("\""), TEXT("\\\""));
        return Out;
    }
    bool HasGenericActorCommand(const TSharedPtr<FJsonObject>& JsonObject)
    {
        return JsonObject.IsValid() &&
            (JsonObject->HasField(TEXT("add_actor")) ||
             JsonObject->HasField(TEXT("remove_actor")) ||
             JsonObject->HasField(TEXT("call_actor")));
    }
}
FString UCommandRouter::HandleCommand(const FString& JsonString, UWorld* World)
{
    const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(JsonString);
    TSharedPtr<FJsonObject> JsonObject;
    if (!FJsonSerializer::Deserialize(Reader, JsonObject) || !JsonObject.IsValid())
    {
        return MakeErrorResponse(TEXT("Invalid JSON"));
    }
    if (HasGenericActorCommand(JsonObject))
    {
        if (!CommandHandle.IsValid())
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
            CommandHandle = MakeUnique<FCommandHandle>(GameInstance);
        }
        const FString GenericActorResponse = CommandHandle->HandleCommand(JsonObject);
        if (!GenericActorResponse.IsEmpty())
        {
            return GenericActorResponse;
        }
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
    return MakeErrorResponse(TEXT("Unknown command"));
}
FString UCommandRouter::HandlePing()
{
    return MakeOkResponse(TEXT("pong"));
}
FString UCommandRouter::HandleSimPause(UWorld* World)
{
    FString Error;
    ASimGameMode* GameMode = ResolveSimGameMode(World, Error);
    if (!GameMode)
    {
        return MakeErrorResponse(Error);
    }
    GameMode->PauseSimulation();
    return MakeOkResponse(TEXT("simulation paused"));
}
FString UCommandRouter::HandleSimResume(UWorld* World)
{
    FString Error;
    ASimGameMode* GameMode = ResolveSimGameMode(World, Error);
    if (!GameMode)
    {
        return MakeErrorResponse(Error);
    }
    GameMode->ResumeSimulation();
    return MakeOkResponse(TEXT("simulation resumed"));
}
FString UCommandRouter::HandleSimReset(UWorld* World)
{
    FString Error;
    ASimGameMode* GameMode = ResolveSimGameMode(World, Error);
    if (!GameMode)
    {
        return MakeErrorResponse(Error);
    }
    GameMode->ResetSimulation();
    return MakeOkResponse(TEXT("simulation reset"));
}
FString UCommandRouter::HandleSimGetTime(UWorld* World)
{
    FString Error;
    USimClockService* Clock = ResolveClock(World, Error);
    if (!Clock)
    {
        return MakeErrorResponse(Error);
    }

    const TSharedPtr<FJsonObject> Response = MakeShared<FJsonObject>();
    Response->SetStringField(TEXT("status"), TEXT("ok"));
    Response->SetNumberField(TEXT("sim_time"), Clock->GetSimTimeSec(World));
    Response->SetNumberField(TEXT("wall_time"), Clock->GetWallTimeSec());
    Response->SetNumberField(TEXT("time_scale"), Clock->GetTimeScale());
    return SerializeJsonObject(Response);
}
FString UCommandRouter::HandleSimSetTimeScale(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World)
{
    const TSharedPtr<FJsonObject>* Obj = nullptr;
    if (!JsonObject->TryGetObjectField(TEXT("sim_set_time_scale"), Obj) || !Obj || !Obj->IsValid())
    {
        return MakeErrorResponse(TEXT("Missing sim_set_time_scale field"));
    }
    const float TimeScale = (*Obj)->HasField(TEXT("time_scale"))
        ? static_cast<float>((*Obj)->GetNumberField(TEXT("time_scale")))
        : 1.0f;

    FString Error;
    USimClockService* Clock = ResolveClock(World, Error);
    if (!Clock)
    {
        return MakeErrorResponse(Error);
    }

    Clock->SetTimeScale(TimeScale, World);
    return MakeOkResponse(FString::Printf(TEXT("time_scale=%.3f"), Clock->GetTimeScale()));
}
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
    const int32 Steps = (*Obj)->HasField(TEXT("steps"))
        ? FMath::Max(1, static_cast<int32>((*Obj)->GetNumberField(TEXT("steps"))))
        : 1;
    const float Dt = (*Obj)->HasField(TEXT("dt"))
        ? FMath::Max(0.0005f, static_cast<float>((*Obj)->GetNumberField(TEXT("dt"))))
        : (1.0f / 60.0f);
    const bool bWasPaused = UGameplayStatics::IsGamePaused(World);
    UGameplayStatics::SetGamePaused(World, false);
    for (int32 Index = 0; Index < Steps; ++Index)
    {
        World->Tick(ELevelTick::LEVELTICK_All, Dt);
    }
    UGameplayStatics::SetGamePaused(World, bWasPaused);

    FString Error;
    USimClockService* Clock = ResolveClock(World, Error);
    if (!Clock)
    {
        return MakeErrorResponse(Error);
    }

    const TSharedPtr<FJsonObject> Response = MakeShared<FJsonObject>();
    Response->SetStringField(TEXT("status"), TEXT("ok"));
    Response->SetNumberField(TEXT("steps"), Steps);
    Response->SetNumberField(TEXT("dt"), Dt);
    Response->SetNumberField(TEXT("sim_time"), Clock->GetSimTimeSec(World));
    Response->SetNumberField(TEXT("wall_time"), Clock->GetWallTimeSec());
    Response->SetNumberField(TEXT("time_scale"), Clock->GetTimeScale());
    return SerializeJsonObject(Response);
}
FString UCommandRouter::HandleGetAgentList()
{
    UAgentManager* Manager = UAgentManager::GetInstance();
    const TArray<FString> Ids = Manager->GetAllAgentIds();

    TArray<TSharedPtr<FJsonValue>> AgentIdsJson;
    TArray<TSharedPtr<FJsonValue>> AgentDetailJson;
    AgentIdsJson.Reserve(Ids.Num());
    AgentDetailJson.Reserve(Ids.Num());

    for (const FString& Id : Ids)
    {
        AActor* Agent = Manager->GetAgent(Id);
        FString Type = TEXT("actor");
        FString Role = TEXT("unknown");
        if (const ADronePawn* Drone = Cast<ADronePawn>(Agent))
        {
            Type = TEXT("drone");
            Role = DroneRoleToString(Drone->MissionRole);
        }
        else if (Cast<AGuidanceActor>(Agent))
        {
            Type = TEXT("guidance");
        }

        AgentIdsJson.Add(MakeShared<FJsonValueString>(Id));

        const TSharedPtr<FJsonObject> AgentJson = MakeShared<FJsonObject>();
        AgentJson->SetStringField(TEXT("id"), Id);
        AgentJson->SetStringField(TEXT("type"), Type);
        AgentJson->SetStringField(TEXT("role"), Role);
        AgentDetailJson.Add(MakeShared<FJsonValueObject>(AgentJson));
    }

    const TSharedPtr<FJsonObject> Response = MakeShared<FJsonObject>();
    Response->SetStringField(TEXT("status"), TEXT("ok"));
    Response->SetArrayField(TEXT("agents"), AgentIdsJson);
    Response->SetArrayField(TEXT("agents_detail"), AgentDetailJson);
    Response->SetNumberField(TEXT("count"), Ids.Num());
    return SerializeJsonObject(Response);
}
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

    const TSharedPtr<FJsonObject> Response = MakeShared<FJsonObject>();
    Response->SetStringField(TEXT("status"), TEXT("ok"));
    Response->SetStringField(TEXT("command_id"), Record.CommandId);
    Response->SetStringField(TEXT("agent_id"), Record.AgentId);
    Response->SetStringField(TEXT("function"), Record.FunctionName);
    Response->SetStringField(TEXT("state"), Record.Status);
    Response->SetStringField(TEXT("message"), Record.Message);
    Response->SetNumberField(TEXT("start_time"), Record.StartTimeSec);
    Response->SetNumberField(TEXT("end_time"), Record.EndTimeSec);
    Response->SetNumberField(TEXT("duration"), Duration);
    return SerializeJsonObject(Response);
}
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

    const TSharedPtr<FJsonObject> Response = MakeShared<FJsonObject>();
    Response->SetStringField(TEXT("status"), TEXT("ok"));
    Response->SetStringField(TEXT("path"), USimulationRecorder::GetInstance()->GetPath());
    return SerializeJsonObject(Response);
}
FString UCommandRouter::HandleRecorderStop()
{
    USimulationRecorder::GetInstance()->Stop();
    return MakeOkResponse(TEXT("recorder stopped"));
}
FString UCommandRouter::HandleRecorderStatus()
{
    USimulationRecorder* Recorder = USimulationRecorder::GetInstance();
    const TSharedPtr<FJsonObject> Response = MakeShared<FJsonObject>();
    Response->SetStringField(TEXT("status"), TEXT("ok"));
    Response->SetBoolField(TEXT("recording"), Recorder->IsRecording());
    Response->SetStringField(TEXT("path"), Recorder->GetPath());
    Response->SetNumberField(TEXT("count"), static_cast<double>(Recorder->GetRecordCount()));
    return SerializeJsonObject(Response);
}
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
    const TArray<FString> Ids = Manager->GetAllAgentIds();

    TArray<TSharedPtr<FJsonValue>> AgentArray;
    AgentArray.Reserve(Ids.Num());
    for (const FString& Id : Ids)
    {
        const TSharedPtr<FJsonObject> AgentJson = MakeShared<FJsonObject>();
        AgentJson->SetStringField(TEXT("id"), Id);
        if (ADronePawn* Drone = Cast<ADronePawn>(Manager->GetAgent(Id)))
        {
            const FVector Pos = Drone->GetCurrentPosition();
            const FVector Vel = Drone->GetCurrentVelocity();
            AgentJson->SetStringField(TEXT("type"), TEXT("drone"));
            TArray<TSharedPtr<FJsonValue>> PositionArray;
            PositionArray.Add(MakeShared<FJsonValueNumber>(Pos.X));
            PositionArray.Add(MakeShared<FJsonValueNumber>(Pos.Y));
            PositionArray.Add(MakeShared<FJsonValueNumber>(Pos.Z));
            AgentJson->SetArrayField(TEXT("position"), PositionArray);

            TArray<TSharedPtr<FJsonValue>> VelocityArray;
            VelocityArray.Add(MakeShared<FJsonValueNumber>(Vel.X));
            VelocityArray.Add(MakeShared<FJsonValueNumber>(Vel.Y));
            VelocityArray.Add(MakeShared<FJsonValueNumber>(Vel.Z));
            AgentJson->SetArrayField(TEXT("velocity"), VelocityArray);
        }
        else if (Cast<AGuidanceActor>(Manager->GetAgent(Id)))
        {
            AgentJson->SetStringField(TEXT("type"), TEXT("guidance"));
        }
        else
        {
            AgentJson->SetStringField(TEXT("type"), TEXT("actor"));
        }
        AgentArray.Add(MakeShared<FJsonValueObject>(AgentJson));
    }

    const TSharedPtr<FJsonObject> PayloadJson = MakeShared<FJsonObject>();
    PayloadJson->SetArrayField(TEXT("agents"), AgentArray);
    Recorder->RecordJsonLine(TEXT("state"), SerializeJsonObject(PayloadJson));
    return MakeOkResponse(TEXT("state recorded"));
}
FString UCommandRouter::MakeErrorResponse(const FString& Error)
{
    return SerializeJsonObject(MakeStatusResponseObject(TEXT("error"), Error));
}
FString UCommandRouter::MakeOkResponse(const FString& Message)
{
    return SerializeJsonObject(MakeStatusResponseObject(TEXT("ok"), Message));
}
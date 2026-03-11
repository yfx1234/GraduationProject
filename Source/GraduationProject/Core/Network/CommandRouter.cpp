#include "CommandRouter.h"

#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"
#include "GraduationProject/Core/Manager/AgentManager.h"
#include "GraduationProject/Core/Manager/CommandExecutionManager.h"
#include "GraduationProject/Core/SimGameMode.h"
#include "GraduationProject/Core/Simulation/SimClockService.h"
#include "GraduationProject/Core/Simulation/SensorManager.h"
#include "GraduationProject/Core/Simulation/SimulationRecorder.h"
#include "GraduationProject/Drone/DroneCommandHandler.h"
#include "GraduationProject/Drone/DronePawn.h"
#include "GraduationProject/Guidance/GuidanceCommandHandler.h"
#include "GraduationProject/Turret/TurretCommandHandler.h"
#include "GraduationProject/Turret/TurretPawn.h"
#include "GraduationProject/Vision/AirSimImageUtils.h"
#include "Engine/TextureRenderTarget2D.h"
#include "IImageWrapper.h"
#include "IImageWrapperModule.h"
#include "Misc/Base64.h"
#include "Modules/ModuleManager.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "GraduationProject/Core/Network/FCommandHandle.h"
#include "Kismet/GameplayStatics.h"

namespace
{
    FString DroneRoleToString(EDroneMissionRole Role)
    {
        switch (Role)
        {
        case EDroneMissionRole::Target: return TEXT("target");
        case EDroneMissionRole::Interceptor: return TEXT("interceptor");
        case EDroneMissionRole::Unknown:
        default: return TEXT("unknown");
        }
    }

    FString JsonEscape(const FString& In)
    {
        FString Out = In;
        Out.ReplaceInline(TEXT("\\"), TEXT("\\\\"));
        Out.ReplaceInline(TEXT("\""), TEXT("\\\""));
        return Out;
    }

    FString EncodeDepthGrayJpegBase64(const TArray<FLinearColor>& DepthPixels, int32 Width, int32 Height, int32 Quality, float MaxDepthMeters)
    {
        if (DepthPixels.Num() != Width * Height)
        {
            return TEXT("");
        }

        TArray<uint8> RawData;
        RawData.SetNumUninitialized(Width * Height);

        const float InvMaxDepth = (MaxDepthMeters > KINDA_SMALL_NUMBER) ? (1.0f / MaxDepthMeters) : 1.0f;
        for (int32 Index = 0; Index < DepthPixels.Num(); ++Index)
        {
            // SCS_SceneDepth stores linear depth in centimeters.
            const float DepthMeters = FMath::Max(0.0f, DepthPixels[Index].R / 100.0f);
            const float Normalized = FMath::Clamp(DepthMeters * InvMaxDepth, 0.0f, 1.0f);
            const uint8 Gray = static_cast<uint8>(FMath::RoundToInt(Normalized * 255.0f));
            RawData[Index] = Gray;
        }

        IImageWrapperModule& ImageWrapperModule = FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));
        TSharedPtr<IImageWrapper> Wrapper = ImageWrapperModule.CreateImageWrapper(EImageFormat::JPEG);
        if (!Wrapper.IsValid())
        {
            return TEXT("");
        }

        if (!Wrapper->SetRaw(RawData.GetData(), RawData.Num(), Width, Height, ERGBFormat::Gray, 8))
        {
            return TEXT("");
        }

        const TArray64<uint8>& JpegData = Wrapper->GetCompressed(FMath::Clamp(Quality, 1, 100));
        return FBase64::Encode(JpegData.GetData(), JpegData.Num());
    }

    FString CaptureDepthBase64(USceneCaptureComponent2D* Capture, int32 Width, int32 Height, int32 Quality, float MaxDepthMeters)
    {
        if (!Capture || Width <= 0 || Height <= 0)
        {
            return TEXT("");
        }

        UTextureRenderTarget2D* OriginalTarget = Capture->TextureTarget;
        const ESceneCaptureSource OriginalSource = Capture->CaptureSource;

        UTextureRenderTarget2D* DepthRT = NewObject<UTextureRenderTarget2D>(Capture);
        if (!DepthRT)
        {
            return TEXT("");
        }

        DepthRT->InitCustomFormat(Width, Height, PF_FloatRGBA, false);
        DepthRT->ClearColor = FLinearColor::Black;

        Capture->TextureTarget = DepthRT;
        Capture->CaptureSource = ESceneCaptureSource::SCS_SceneDepth;
        Capture->CaptureScene();

        FString Base64;
        if (FTextureRenderTargetResource* Resource = DepthRT->GameThread_GetRenderTargetResource())
        {
            TArray<FLinearColor> DepthPixels;
            if (Resource->ReadLinearColorPixels(DepthPixels))
            {
                Base64 = EncodeDepthGrayJpegBase64(DepthPixels, Width, Height, Quality, MaxDepthMeters);
            }
        }

        Capture->TextureTarget = OriginalTarget;
        Capture->CaptureSource = OriginalSource;
        return Base64;
    }
}

FString UCommandRouter::HandleCommand(const FString& JsonString, UWorld* World)
{
    TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(JsonString);
    TSharedPtr<FJsonObject> JsonObject;
    if (!FJsonSerializer::Deserialize(Reader, JsonObject) || !JsonObject.IsValid())
    {
        return MakeErrorResponse(TEXT("Invalid JSON"));
    }

    if (!CommandHandle.IsValid())
    {
        CommandHandle = MakeUnique<FCommandHandle>(World->GetGameInstance());
    }

    if (JsonObject->HasField(TEXT("add_actor")))
    {
        return CommandHandle->HandleAddActor(JsonObject->GetObjectField(TEXT("add_actor")));
    }
    if (JsonObject->HasField(TEXT("remove_actor")))
    {
        return CommandHandle->HandleRemoveActor(JsonObject->GetObjectField(TEXT("remove_actor")));
    }
    if (JsonObject->HasField(TEXT("call_actor")))
    {
        return CommandHandle->HandleCallActor(JsonObject->GetObjectField(TEXT("call_actor")));
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

FString UCommandRouter::HandlePing()
{
    return TEXT("{\"status\":\"ok\",\"message\":\"pong\"}");
}

FString UCommandRouter::HandleSimPause(UWorld* World)
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

    GameMode->PauseSimulation();
    return MakeOkResponse(TEXT("simulation paused"));
}

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
FString UCommandRouter::HandleGetAgentList()
{
    UAgentManager* Manager = UAgentManager::GetInstance();
    const TArray<FString> Ids = Manager->GetAllAgentIds();

    FString AgentIdsJson = TEXT("[");
    FString AgentDetailJson = TEXT("[");
    for (int32 Index = 0; Index < Ids.Num(); ++Index)
    {
        const FString& Id = Ids[Index];
        AActor* Agent = Manager->GetAgent(Id);

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

    return FString::Printf(TEXT("{\"status\":\"ok\",\"path\":\"%s\"}"), *USimulationRecorder::GetInstance()->GetPath().ReplaceCharWithEscapedChar());
}

FString UCommandRouter::HandleRecorderStop()
{
    USimulationRecorder::GetInstance()->Stop();
    return MakeOkResponse(TEXT("recorder stopped"));
}

FString UCommandRouter::HandleRecorderStatus()
{
    USimulationRecorder* Recorder = USimulationRecorder::GetInstance();
    return FString::Printf(
        TEXT("{\"status\":\"ok\",\"recording\":%s,\"path\":\"%s\",\"count\":%lld}"),
        Recorder->IsRecording() ? TEXT("true") : TEXT("false"),
        *Recorder->GetPath().ReplaceCharWithEscapedChar(),
        Recorder->GetRecordCount());
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
    FString Payload = TEXT("{\"agents\":[");

    const TArray<FString> Ids = Manager->GetAllAgentIds();
    for (int32 Index = 0; Index < Ids.Num(); ++Index)
    {
        const FString& Id = Ids[Index];
        if (ADronePawn* Drone = Cast<ADronePawn>(Manager->GetAgent(Id)))
        {
            const FVector Pos = Drone->GetCurrentPosition();
            const FVector Vel = Drone->GetCurrentVelocity();
            Payload += FString::Printf(TEXT("{\"id\":\"%s\",\"type\":\"drone\",\"position\":[%.4f,%.4f,%.4f],\"velocity\":[%.4f,%.4f,%.4f]}"),
                *Id, Pos.X, Pos.Y, Pos.Z, Vel.X, Vel.Y, Vel.Z);
        }
        else if (ATurretPawn* Turret = Cast<ATurretPawn>(Manager->GetAgent(Id)))
        {
            Payload += FString::Printf(TEXT("{\"id\":\"%s\",\"type\":\"turret\",\"pitch\":%.3f,\"yaw\":%.3f}"),
                *Id, Turret->GetCurrentPitch(), Turret->GetCurrentYaw());
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
FString UCommandRouter::MakeErrorResponse(const FString& Error)
{
    return FString::Printf(TEXT("{\"status\":\"error\",\"message\":\"%s\"}"), *JsonEscape(Error));
}

FString UCommandRouter::MakeOkResponse(const FString& Message)
{
    return FString::Printf(TEXT("{\"status\":\"ok\",\"message\":\"%s\"}"), *JsonEscape(Message));
}

FString UCommandRouter::HandleGetImage(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World)
{
    if (!World)
    {
        return MakeErrorResponse(TEXT("No World"));
    }

    UAgentManager* Manager = UAgentManager::GetInstance();
    if (!Manager)
    {
        return MakeErrorResponse(TEXT("AgentManager unavailable"));
    }

    FString AgentId;
    FString ImageTypeRaw = TEXT("scene");
    int32 Quality = 85;
    float MaxDepthMeters = 200.0f;

    const TSharedPtr<FJsonObject>* ImgObj = nullptr;
    if (JsonObject->TryGetObjectField(TEXT("get_image"), ImgObj) && ImgObj && ImgObj->IsValid())
    {
        (*ImgObj)->TryGetStringField(TEXT("id"), AgentId);

        double ImageTypeNumber = 0.0;
        if (!(*ImgObj)->TryGetStringField(TEXT("image_type"), ImageTypeRaw) && (*ImgObj)->TryGetNumberField(TEXT("image_type"), ImageTypeNumber))
        {
            ImageTypeRaw = FString::FromInt(static_cast<int32>(ImageTypeNumber));
        }

        if ((*ImgObj)->HasField(TEXT("quality")))
        {
            Quality = FMath::Clamp(static_cast<int32>((*ImgObj)->GetNumberField(TEXT("quality"))), 1, 100);
        }
        if ((*ImgObj)->HasField(TEXT("max_depth_m")))
        {
            MaxDepthMeters = FMath::Max(1.0f, static_cast<float>((*ImgObj)->GetNumberField(TEXT("max_depth_m"))));
        }
    }

    AirSimImageUtils::EImageType ImageType = AirSimImageUtils::EImageType::Scene;
    if (!AirSimImageUtils::TryParseImageType(ImageTypeRaw, ImageType))
    {
        return MakeErrorResponse(TEXT("Unsupported image_type, use 0/1/3/5/7 or scene/depth_planar/depth_vis/segmentation/infrared"));
    }

    const FString ImageTypeToken = AirSimImageUtils::ToCanonicalString(ImageType);
    const int32 ImageTypeId = static_cast<int32>(ImageType);

    auto BuildImageJson = [&](const FString& SourceId, int32 Width, int32 Height, const FVector& CamPos, const FRotator& CamRot, float Fov, const FString& Data)->FString
    {
        return FString::Printf(
            TEXT("{\"status\":\"ok\",\"source\":\"%s\",\"image_type\":\"%s\",\"image_type_id\":%d,\"width\":%d,\"height\":%d,\"format\":\"jpeg\",\"camera_pos\":[%.2f,%.2f,%.2f],\"camera_rot\":[%.2f,%.2f,%.2f],\"fov\":%.2f,\"data\":\"%s\"}"),
            *JsonEscape(SourceId),
            *ImageTypeToken,
            ImageTypeId,
            Width,
            Height,
            CamPos.X, CamPos.Y, CamPos.Z,
            CamRot.Pitch, CamRot.Yaw, CamRot.Roll,
            Fov,
            *Data);
    };

    auto TryBuildForAgent = [&](AActor* Agent, const FString& SourceId, FString& OutJson)->bool
    {
        if (ADronePawn* Drone = Cast<ADronePawn>(Agent))
        {
            if (!Drone->DroneSceneCapture)
            {
                return false;
            }

            const FString Base64 = AirSimImageUtils::CaptureJpegBase64(
                Drone->DroneSceneCapture,
                Drone->CameraWidth,
                Drone->CameraHeight,
                ImageType,
                Quality,
                MaxDepthMeters);

            if (Base64.IsEmpty())
            {
                return false;
            }

            const FVector CamPos = Drone->DroneSceneCapture->GetComponentLocation();
            const FRotator CamRot = Drone->DroneSceneCapture->GetComponentRotation();
            OutJson = BuildImageJson(SourceId, Drone->CameraWidth, Drone->CameraHeight, CamPos, CamRot, Drone->CameraFOV, Base64);
            return true;
        }

        if (ATurretPawn* Turret = Cast<ATurretPawn>(Agent))
        {
            if (!Turret->TurretSceneCapture)
            {
                return false;
            }

            const FString Base64 = AirSimImageUtils::CaptureJpegBase64(
                Turret->TurretSceneCapture,
                Turret->CameraWidth,
                Turret->CameraHeight,
                ImageType,
                Quality,
                MaxDepthMeters);

            if (Base64.IsEmpty())
            {
                return false;
            }

            const FVector CamPos = Turret->TurretSceneCapture->GetComponentLocation();
            const FRotator CamRot = Turret->TurretSceneCapture->GetComponentRotation();
            OutJson = BuildImageJson(SourceId, Turret->CameraWidth, Turret->CameraHeight, CamPos, CamRot, Turret->CameraFOV, Base64);
            return true;
        }

        return false;
    };

    if (!AgentId.IsEmpty())
    {
        AActor* Agent = Manager->GetAgent(AgentId);
        if (!Agent)
        {
            return MakeErrorResponse(FString::Printf(TEXT("Agent '%s' not found"), *AgentId));
        }

        FString Result;
        if (TryBuildForAgent(Agent, AgentId, Result))
        {
            return Result;
        }

        return MakeErrorResponse(FString::Printf(TEXT("Agent '%s' has no camera or capture failed"), *AgentId));
    }

    const TArray<FString> Ids = Manager->GetAllAgentIds();
    for (const FString& Id : Ids)
    {
        FString Result;
        if (TryBuildForAgent(Manager->GetAgent(Id), Id, Result))
        {
            return Result;
        }
    }

    return MakeErrorResponse(TEXT("No agent with camera found"));
}




























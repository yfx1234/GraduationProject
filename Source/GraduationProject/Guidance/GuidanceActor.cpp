#include "GuidanceActor.h"

#include "Dom/JsonObject.h"
#include "GraduationProject/Core/Manager/AgentManager.h"
#include "VisualInterceptController.h"

namespace
{
    FString JsonEscape(const FString& Value)
    {
        return Value.ReplaceCharWithEscapedChar();
    }

    void SetStringIfNotEmpty(const TSharedPtr<FJsonObject>& Json, const TCHAR* FieldName, const FString& Value)
    {
        if (Json.IsValid() && !Value.IsEmpty())
        {
            Json->SetStringField(FieldName, Value);
        }
    }

    void SetNumberIfValid(const TSharedPtr<FJsonObject>& Json, const TCHAR* FieldName, float Value)
    {
        if (Json.IsValid() && FMath::IsFinite(Value) && Value >= 0.0f)
        {
            Json->SetNumberField(FieldName, Value);
        }
    }

    void SetIntIfValid(const TSharedPtr<FJsonObject>& Json, const TCHAR* FieldName, int32 Value)
    {
        if (Json.IsValid() && Value >= 0)
        {
            Json->SetNumberField(FieldName, Value);
        }
    }
}

AGuidanceActor::AGuidanceActor()
{
    PrimaryActorTick.bCanEverTick = false;
}

void AGuidanceActor::BeginPlay()
{
    Super::BeginPlay();

    if (UAgentManager* Manager = UAgentManager::GetInstance())
    {
        FString ExistingId;
        for (const FString& Id : Manager->GetAllAgentIds())
        {
            if (Manager->GetAgent(Id) == this)
            {
                ExistingId = Id;
                break;
            }
        }

        if (!ExistingId.IsEmpty() && ExistingId != GuidanceId)
        {
            GuidanceId = ExistingId;
        }

        if (Manager->GetAgent(GuidanceId) != this)
        {
            Manager->RegisterAgent(GuidanceId, this);
        }
    }
}

void AGuidanceActor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    if (UAgentManager* Manager = UAgentManager::GetInstance())
    {
        if (!GuidanceId.IsEmpty() && Manager->GetAgent(GuidanceId) == this)
        {
            Manager->UnregisterAgent(GuidanceId);
        }
    }

    Super::EndPlay(EndPlayReason);
}

void AGuidanceActor::EnsureInitialized()
{
    if (!VisualInterceptController)
    {
        VisualInterceptController = NewObject<UVisualInterceptController>(this);
        VisualInterceptController->EnsureInitialized();
    }
}

FString AGuidanceActor::MakeError(const FString& Msg) const
{
    return FString::Printf(TEXT("{\"status\":\"error\",\"message\":\"%s\"}"), *JsonEscape(Msg));
}

FString AGuidanceActor::MakeOk(const FString& Msg) const
{
    return FString::Printf(TEXT("{\"status\":\"ok\",\"message\":\"%s\"}"), *JsonEscape(Msg));
}

FString AGuidanceActor::ResetGuidance()
{
    EnsureInitialized();
    VisualInterceptController->Reset();
    return MakeOk(TEXT("guidance reset"));
}

FString AGuidanceActor::GetState()
{
    EnsureInitialized();
    const FString VisualState = VisualInterceptController ? VisualInterceptController->HandleState() : TEXT("{}");
    return FString::Printf(
        TEXT("{\"status\":\"ok\",\"id\":\"%s\",\"visual_state\":%s}"),
        *JsonEscape(GuidanceId),
        *VisualState);
}

FString AGuidanceActor::VisualInterceptStart(
    FString InterceptorId,
    FString TargetId,
    FString Method,
    float DesiredArea,
    float CaptureArea,
    float CenterTolX,
    float CenterTolY,
    int32 CaptureHoldFrames,
    int32 LostToSearchFrames,
    float MaxForwardSpeed,
    float MaxReverseSpeed,
    float MaxVerticalSpeed,
    float MaxYawRateDeg,
    float RamAreaTarget,
    float MinRamSpeed,
    float InterceptDistance,
    float TrackLeadTime,
    float RamLeadTime,
    float SearchCamYawLimitDeg,
    float SearchCamRateDeg,
    float SearchBodyYawRateDeg,
    float SearchCamPitchDeg,
    float SearchVzAmp,
    int32 StopOnCaptureFlag,
    int32 UseKalmanFlag)
{
    EnsureInitialized();

    const TSharedPtr<FJsonObject> Cmd = MakeShareable(new FJsonObject);
    SetStringIfNotEmpty(Cmd, TEXT("interceptor_id"), InterceptorId);
    SetStringIfNotEmpty(Cmd, TEXT("target_id"), TargetId);
    SetStringIfNotEmpty(Cmd, TEXT("method"), Method);
    SetNumberIfValid(Cmd, TEXT("desired_area"), DesiredArea);
    SetNumberIfValid(Cmd, TEXT("capture_area"), CaptureArea);
    SetNumberIfValid(Cmd, TEXT("center_tol_x"), CenterTolX);
    SetNumberIfValid(Cmd, TEXT("center_tol_y"), CenterTolY);
    SetIntIfValid(Cmd, TEXT("capture_hold_frames"), CaptureHoldFrames);
    SetIntIfValid(Cmd, TEXT("lost_to_search_frames"), LostToSearchFrames);
    SetNumberIfValid(Cmd, TEXT("max_forward_speed"), MaxForwardSpeed);
    SetNumberIfValid(Cmd, TEXT("max_reverse_speed"), MaxReverseSpeed);
    SetNumberIfValid(Cmd, TEXT("max_vertical_speed"), MaxVerticalSpeed);
    SetNumberIfValid(Cmd, TEXT("max_yaw_rate_deg"), MaxYawRateDeg);
    SetNumberIfValid(Cmd, TEXT("ram_area_target"), RamAreaTarget);
    SetNumberIfValid(Cmd, TEXT("min_ram_speed"), MinRamSpeed);
    SetNumberIfValid(Cmd, TEXT("intercept_distance"), InterceptDistance);
    SetNumberIfValid(Cmd, TEXT("track_lead_time"), TrackLeadTime);
    SetNumberIfValid(Cmd, TEXT("ram_lead_time"), RamLeadTime);
    SetNumberIfValid(Cmd, TEXT("search_cam_yaw_limit_deg"), SearchCamYawLimitDeg);
    SetNumberIfValid(Cmd, TEXT("search_cam_rate_deg"), SearchCamRateDeg);
    SetNumberIfValid(Cmd, TEXT("search_body_yaw_rate_deg"), SearchBodyYawRateDeg);
    SetNumberIfValid(Cmd, TEXT("search_vz_amp"), SearchVzAmp);
    if (!FMath::IsNearlyEqual(SearchCamPitchDeg, -1000.0f))
    {
        Cmd->SetNumberField(TEXT("search_cam_pitch_deg"), SearchCamPitchDeg);
    }
    if (StopOnCaptureFlag >= 0)
    {
        Cmd->SetBoolField(TEXT("stop_on_capture"), StopOnCaptureFlag != 0);
    }
    if (UseKalmanFlag >= 0)
    {
        Cmd->SetBoolField(TEXT("use_kalman"), UseKalmanFlag != 0);
    }

    return VisualInterceptController->HandleStart(Cmd, GetWorld());
}

FString AGuidanceActor::VisualInterceptUpdate(
    int32 HasDetection,
    float Cx,
    float Cy,
    float Area,
    float AreaRatio,
    float Conf,
    float Dt,
    float ProcessingLatency,
    float ImageW,
    float ImageH,
    FString InterceptorId,
    FString TargetId)
{
    EnsureInitialized();

    const TSharedPtr<FJsonObject> Cmd = MakeShareable(new FJsonObject);
    Cmd->SetBoolField(TEXT("has_detection"), HasDetection != 0);
    Cmd->SetNumberField(TEXT("cx"), Cx);
    Cmd->SetNumberField(TEXT("cy"), Cy);
    Cmd->SetNumberField(TEXT("area"), Area);
    Cmd->SetNumberField(TEXT("area_ratio"), AreaRatio);
    Cmd->SetNumberField(TEXT("conf"), Conf);
    Cmd->SetNumberField(TEXT("dt"), Dt);
    Cmd->SetNumberField(TEXT("processing_latency"), ProcessingLatency);
    Cmd->SetNumberField(TEXT("image_w"), ImageW);
    Cmd->SetNumberField(TEXT("image_h"), ImageH);
    SetStringIfNotEmpty(Cmd, TEXT("interceptor_id"), InterceptorId);
    SetStringIfNotEmpty(Cmd, TEXT("target_id"), TargetId);

    return VisualInterceptController->HandleUpdate(Cmd, GetWorld());
}

FString AGuidanceActor::VisualInterceptStop(FString InterceptorId, FString TargetId)
{
    EnsureInitialized();

    const TSharedPtr<FJsonObject> Cmd = MakeShareable(new FJsonObject);
    SetStringIfNotEmpty(Cmd, TEXT("interceptor_id"), InterceptorId);
    SetStringIfNotEmpty(Cmd, TEXT("target_id"), TargetId);
    return VisualInterceptController->HandleStop(Cmd, GetWorld());
}

FString AGuidanceActor::VisualInterceptState()
{
    EnsureInitialized();
    return VisualInterceptController->HandleState();
}

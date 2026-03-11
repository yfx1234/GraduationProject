#include "VisualInterceptController.h"

#include "Dom/JsonObject.h"
#include "GraduationProject/Core/Controller/PIDController.h"
#include "GraduationProject/Core/Manager/AgentManager.h"
#include "GraduationProject/Drone/DronePawn.h"
#include "KalmanPredictor.h"

namespace
{
    constexpr float kDefaultDt = 0.08f;

    template <typename T>
    bool TryGetNumberAs(const TSharedPtr<FJsonObject>& Obj, const TCHAR* Field, T& OutValue)
    {
        if (!Obj.IsValid() || !Obj->HasField(Field))
        {
            return false;
        }
        const double Value = Obj->GetNumberField(Field);
        OutValue = static_cast<T>(Value);
        return true;
    }

    ADronePawn* FindDroneByRole(UAgentManager* Manager, EDroneMissionRole Role, const FString& ExcludeId = TEXT(""))
    {
        if (!Manager)
        {
            return nullptr;
        }

        const TArray<FString> AgentIds = Manager->GetAllAgentIds();
        for (const FString& AgentId : AgentIds)
        {
            if (!ExcludeId.IsEmpty() && AgentId == ExcludeId)
            {
                continue;
            }

            ADronePawn* Drone = Cast<ADronePawn>(Manager->GetAgent(AgentId));
            if (Drone && Drone->MissionRole == Role)
            {
                return Drone;
            }
        }

        return nullptr;
    }
}

void UVisualInterceptController::EnsureInitialized()
{
    if (!FeaturePredictor)
    {
        FeaturePredictor = NewObject<UKalmanPredictor>(this);
        FeaturePredictor->Initialize(25.0f, 40.0f);
    }

    if (!YawPid)
    {
        YawPid = NewObject<UPIDController>(this);
        YawPid->Initialize(42.0, 0.0, 12.0, 0.03, Params.MaxYawRateDeg, 0.02, -200.0, 200.0);
    }

    if (!VerticalPid)
    {
        VerticalPid = NewObject<UPIDController>(this);
        VerticalPid->Initialize(2.2, 0.0, 0.7, 0.05, Params.MaxVerticalSpeed, 0.02, -5.0, 5.0);
    }

    if (!ForwardPid)
    {
        ForwardPid = NewObject<UPIDController>(this);
        ForwardPid->Initialize(80.0, 0.0, 22.0, 0.05, Params.MaxForwardSpeed, 0.02, -5.0, 5.0);
    }
}

void UVisualInterceptController::ResetRuntimeOnly()
{
    Runtime = FVisualRuntime();
}

void UVisualInterceptController::Reset()
{
    EnsureInitialized();

    if (FeaturePredictor)
    {
        FeaturePredictor->Reset();
        FeaturePredictor->Initialize(25.0f, 40.0f);
    }

    if (YawPid)
    {
        YawPid->Reset();
    }

    if (VerticalPid)
    {
        VerticalPid->Reset();
    }

    if (ForwardPid)
    {
        ForwardPid->Reset();
    }

    ResetRuntimeOnly();
}

FString UVisualInterceptController::BoolLiteral(bool bValue)
{
    return bValue ? TEXT("true") : TEXT("false");
}

float UVisualInterceptController::NormalizeYawDeg(float YawDeg)
{
    return FRotator::NormalizeAxis(YawDeg);
}

FString UVisualInterceptController::StateToString(EVisualState State)
{
    switch (State)
    {
    case EVisualState::Search:
        return TEXT("SEARCH");
    case EVisualState::Track:
        return TEXT("TRACK");
    case EVisualState::Approach:
        return TEXT("APPROACH");
    case EVisualState::Captured:
        return TEXT("CAPTURED");
    case EVisualState::Idle:
    default:
        return TEXT("IDLE");
    }
}

bool UVisualInterceptController::NormalizeMethod(const FString& InMethod, FString& OutCanonical)
{
    FString Method = InMethod;
    Method.TrimStartAndEndInline();
    Method.ToLowerInline();

    if (Method.IsEmpty() || Method == TEXT("auto"))
    {
        OutCanonical = TEXT("vision_pid_kalman");
        return true;
    }

    if (Method == TEXT("vision_pid_kalman") || Method == TEXT("visual_pid_kalman") || Method == TEXT("pid_kalman"))
    {
        OutCanonical = TEXT("vision_pid_kalman");
        return true;
    }

    if (Method == TEXT("vision_pid") || Method == TEXT("visual_pid") || Method == TEXT("pid"))
    {
        OutCanonical = TEXT("vision_pid");
        return true;
    }

    if (Method == TEXT("vision_pd_kalman") || Method == TEXT("visual_pd_kalman") || Method == TEXT("pd_kalman"))
    {
        OutCanonical = TEXT("vision_pd_kalman");
        return true;
    }

    if (Method == TEXT("vision_pd") || Method == TEXT("visual_pd") || Method == TEXT("pd"))
    {
        OutCanonical = TEXT("vision_pd");
        return true;
    }

    return false;
}
FString UVisualInterceptController::MakeError(const FString& Msg) const
{
    return FString::Printf(TEXT("{\"status\":\"error\",\"message\":\"%s\"}"), *Msg.ReplaceCharWithEscapedChar());
}

FString UVisualInterceptController::MakeOk(const FString& Msg) const
{
    return FString::Printf(TEXT("{\"status\":\"ok\",\"message\":\"%s\"}"), *Msg.ReplaceCharWithEscapedChar());
}

float UVisualInterceptController::SafeDt(float Dt) const
{
    if (!FMath::IsFinite(Dt) || Dt <= 0.0f)
    {
        return kDefaultDt;
    }
    return FMath::Clamp(Dt, 0.005f, 0.5f);
}

void UVisualInterceptController::SyncPidTimeStep(float Dt)
{
    if (YawPid)
    {
        YawPid->SetTimeStep(Dt);
    }
    if (VerticalPid)
    {
        VerticalPid->SetTimeStep(Dt);
    }
    if (ForwardPid)
    {
        ForwardPid->SetTimeStep(Dt);
    }
}

void UVisualInterceptController::ApplyParamsFromJson(const TSharedPtr<FJsonObject>& CmdObj)
{
    if (!CmdObj.IsValid())
    {
        return;
    }

    TryGetNumberAs(CmdObj, TEXT("desired_area"), Params.DesiredArea);
    TryGetNumberAs(CmdObj, TEXT("capture_area"), Params.CaptureArea);
    TryGetNumberAs(CmdObj, TEXT("center_tol_x"), Params.CenterTolX);
    TryGetNumberAs(CmdObj, TEXT("center_tol_y"), Params.CenterTolY);
    TryGetNumberAs(CmdObj, TEXT("capture_hold_frames"), Params.CaptureHoldFrames);
    TryGetNumberAs(CmdObj, TEXT("lost_to_search_frames"), Params.LostToSearchFrames);

    TryGetNumberAs(CmdObj, TEXT("max_forward_speed"), Params.MaxForwardSpeed);
    TryGetNumberAs(CmdObj, TEXT("max_reverse_speed"), Params.MaxReverseSpeed);
    TryGetNumberAs(CmdObj, TEXT("max_vertical_speed"), Params.MaxVerticalSpeed);
    TryGetNumberAs(CmdObj, TEXT("max_yaw_rate_deg"), Params.MaxYawRateDeg);

    TryGetNumberAs(CmdObj, TEXT("search_cam_yaw_limit_deg"), Params.SearchCamYawLimitDeg);
    TryGetNumberAs(CmdObj, TEXT("search_cam_rate_deg"), Params.SearchCamRateDeg);
    TryGetNumberAs(CmdObj, TEXT("search_body_yaw_rate_deg"), Params.SearchBodyYawRateDeg);
    TryGetNumberAs(CmdObj, TEXT("search_cam_pitch_deg"), Params.SearchCamPitchDeg);
    TryGetNumberAs(CmdObj, TEXT("search_vz_amp"), Params.SearchVzAmp);

    if (CmdObj->HasField(TEXT("stop_on_capture")))
    {
        Params.bStopOnCapture = CmdObj->GetBoolField(TEXT("stop_on_capture"));
    }

    if (CmdObj->HasField(TEXT("use_kalman")))
    {
        Params.bUseKalman = CmdObj->GetBoolField(TEXT("use_kalman"));
    }

    Params.DesiredArea = FMath::Clamp(Params.DesiredArea, 0.0001f, 0.95f);
    Params.CaptureArea = FMath::Clamp(Params.CaptureArea, Params.DesiredArea, 0.98f);
    Params.CenterTolX = FMath::Clamp(Params.CenterTolX, 0.001f, 1.0f);
    Params.CenterTolY = FMath::Clamp(Params.CenterTolY, 0.001f, 1.0f);
    Params.CaptureHoldFrames = FMath::Max(1, Params.CaptureHoldFrames);
    Params.LostToSearchFrames = FMath::Max(1, Params.LostToSearchFrames);

    Params.MaxForwardSpeed = FMath::Max(0.1f, Params.MaxForwardSpeed);
    Params.MaxReverseSpeed = FMath::Max(0.0f, Params.MaxReverseSpeed);
    Params.MaxVerticalSpeed = FMath::Max(0.1f, Params.MaxVerticalSpeed);
    Params.MaxYawRateDeg = FMath::Max(1.0f, Params.MaxYawRateDeg);

    Params.SearchCamYawLimitDeg = FMath::Clamp(Params.SearchCamYawLimitDeg, 10.0f, 179.0f);
    Params.SearchCamRateDeg = FMath::Clamp(Params.SearchCamRateDeg, 1.0f, 180.0f);
    Params.SearchBodyYawRateDeg = FMath::Clamp(Params.SearchBodyYawRateDeg, 0.0f, 120.0f);
    Params.SearchVzAmp = FMath::Clamp(Params.SearchVzAmp, 0.0f, 5.0f);

    if (YawPid)
    {
        YawPid->SetParameters(YawPid->Kp, YawPid->Ki, YawPid->Kd, YawPid->DiffFilterTau, Params.MaxYawRateDeg);
    }
    if (VerticalPid)
    {
        VerticalPid->SetParameters(VerticalPid->Kp, VerticalPid->Ki, VerticalPid->Kd, VerticalPid->DiffFilterTau, Params.MaxVerticalSpeed);
    }
    if (ForwardPid)
    {
        ForwardPid->SetParameters(ForwardPid->Kp, ForwardPid->Ki, ForwardPid->Kd, ForwardPid->DiffFilterTau, Params.MaxForwardSpeed);
    }
}

ADronePawn* UVisualInterceptController::ResolveInterceptor(UWorld* World, const TSharedPtr<FJsonObject>& CmdObj)
{
    UAgentManager* Manager = UAgentManager::GetInstance();
    if (!Manager)
    {
        return nullptr;
    }

    FString DesiredId = Runtime.InterceptorId;
    if (CmdObj.IsValid() && CmdObj->HasField(TEXT("interceptor_id")))
    {
        DesiredId = CmdObj->GetStringField(TEXT("interceptor_id"));
    }

    ADronePawn* Drone = nullptr;
    if (!DesiredId.IsEmpty())
    {
        Drone = Cast<ADronePawn>(Manager->GetAgent(DesiredId));
    }

    if (!Drone)
    {
        Drone = FindDroneByRole(Manager, EDroneMissionRole::Interceptor);
    }

    if (!Drone)
    {
        Drone = Cast<ADronePawn>(Manager->GetAgent(TEXT("drone_1")));
    }

    if (!Drone)
    {
        Drone = Cast<ADronePawn>(Manager->GetAgent(TEXT("drone_0")));
    }

    if (Drone)
    {
        Runtime.InterceptorId = Drone->DroneId;
    }

    return Drone;
}

ADronePawn* UVisualInterceptController::ResolveTarget(UWorld* World, const TSharedPtr<FJsonObject>& CmdObj)
{
    UAgentManager* Manager = UAgentManager::GetInstance();
    if (!Manager)
    {
        return nullptr;
    }

    FString DesiredId = Runtime.TargetId;
    if (CmdObj.IsValid() && CmdObj->HasField(TEXT("target_id")))
    {
        DesiredId = CmdObj->GetStringField(TEXT("target_id"));
    }

    ADronePawn* Drone = nullptr;
    if (!DesiredId.IsEmpty())
    {
        Drone = Cast<ADronePawn>(Manager->GetAgent(DesiredId));
    }

    if (!Drone)
    {
        Drone = FindDroneByRole(Manager, EDroneMissionRole::Target, Runtime.InterceptorId);
    }

    if (!Drone)
    {
        Drone = Cast<ADronePawn>(Manager->GetAgent(TEXT("drone_0")));
    }

    if (Drone)
    {
        Runtime.TargetId = Drone->DroneId;
    }

    return Drone;
}

FString UVisualInterceptController::BuildStartJson() const
{
    return FString::Printf(
        TEXT("{\"status\":\"ok\",\"mode\":\"visual_intercept\",\"enabled\":%s,\"state\":\"%s\",\"interceptor_id\":\"%s\",\"target_id\":\"%s\",\"method\":\"%s\",\"params\":{\"desired_area\":%.5f,\"capture_area\":%.5f,\"center_tol_x\":%.4f,\"center_tol_y\":%.4f,\"capture_hold_frames\":%d,\"lost_to_search_frames\":%d,\"max_forward_speed\":%.3f,\"max_reverse_speed\":%.3f,\"max_vertical_speed\":%.3f,\"max_yaw_rate_deg\":%.3f,\"stop_on_capture\":%s,\"use_kalman\":%s}}"),
        *BoolLiteral(Runtime.bEnabled),
        *StateToString(Runtime.State),
        *Runtime.InterceptorId,
        *Runtime.TargetId,
        *Runtime.Method,
        Params.DesiredArea,
        Params.CaptureArea,
        Params.CenterTolX,
        Params.CenterTolY,
        Params.CaptureHoldFrames,
        Params.LostToSearchFrames,
        Params.MaxForwardSpeed,
        Params.MaxReverseSpeed,
        Params.MaxVerticalSpeed,
        Params.MaxYawRateDeg,
        *BoolLiteral(Params.bStopOnCapture),
        *BoolLiteral(Params.bUseKalman));
}

FString UVisualInterceptController::BuildUpdateJson(bool bValidControl) const
{
    return FString::Printf(
        TEXT("{\"status\":\"ok\",\"mode\":\"visual_intercept\",\"enabled\":%s,\"state\":\"%s\",\"captured\":%s,\"valid\":%s,\"interceptor_id\":\"%s\",\"target_id\":\"%s\",\"method\":\"%s\",\"frame\":%d,\"detections\":%d,\"lost_count\":%d,\"capture_count\":%d,\"cmd_velocity\":[%.4f,%.4f,%.4f],\"control\":{\"ex\":%.5f,\"ey\":%.5f,\"area_norm\":%.6f,\"conf\":%.4f,\"yaw_cmd_deg\":%.4f,\"yaw_rate_deg\":%.4f}}"),
        *BoolLiteral(Runtime.bEnabled),
        *StateToString(Runtime.State),
        *BoolLiteral(Runtime.bCaptured),
        *BoolLiteral(bValidControl),
        *Runtime.InterceptorId,
        *Runtime.TargetId,
        *Runtime.Method,
        Runtime.FrameCount,
        Runtime.DetectionCount,
        Runtime.LostCount,
        Runtime.CaptureCount,
        Runtime.LastCmdVel.X,
        Runtime.LastCmdVel.Y,
        Runtime.LastCmdVel.Z,
        Runtime.LastEx,
        Runtime.LastEy,
        Runtime.LastAreaNorm,
        Runtime.LastConf,
        Runtime.LastYawCmdDeg,
        Runtime.LastYawRateDeg);
}

bool UVisualInterceptController::ComputeTrackControl(
    ADronePawn* Interceptor,
    const FVector& Feature,
    float FrameW,
    float FrameH,
    float Confidence,
    float Dt,
    bool bDetectionReal,
    bool& bOutCaptured)
{
    bOutCaptured = false;

    if (!Interceptor || FrameW <= 1.0f || FrameH <= 1.0f)
    {
        return false;
    }

    const float Cx = Feature.X;
    const float Cy = Feature.Y;
    const float Area = FMath::Max(1.0f, Feature.Z);

    const float Ex = (Cx - 0.5f * FrameW) / FMath::Max(1.0f, 0.5f * FrameW);
    const float Ey = (Cy - 0.5f * FrameH) / FMath::Max(1.0f, 0.5f * FrameH);
    const float AreaNorm = Area / FMath::Max(1.0f, FrameW * FrameH);

    float YawRateCmd = static_cast<float>(YawPid ? YawPid->Update(Ex, 0.0) : 0.0);
    YawRateCmd = FMath::Clamp(YawRateCmd, -Params.MaxYawRateDeg, Params.MaxYawRateDeg);

    const float CurrentYawDeg = Interceptor->CurrentState.GetRotator().Yaw;
    const float YawCmdDeg = NormalizeYawDeg(CurrentYawDeg + YawRateCmd * Dt);

    float VzCmd = static_cast<float>(VerticalPid ? -VerticalPid->Update(Ey, 0.0) : 0.0);
    VzCmd = FMath::Clamp(VzCmd, -Params.MaxVerticalSpeed, Params.MaxVerticalSpeed);

    const float AreaError = Params.DesiredArea - AreaNorm;
    float ForwardCmd = static_cast<float>(ForwardPid ? ForwardPid->Update(AreaError, 0.0) : 0.0);
    ForwardCmd = FMath::Clamp(ForwardCmd, -Params.MaxReverseSpeed, Params.MaxForwardSpeed);
    if (AreaNorm >= Params.CaptureArea)
    {
        ForwardCmd = FMath::Min(0.0f, ForwardCmd);
    }

    const float YawRad = FMath::DegreesToRadians(YawCmdDeg);
    const FVector CmdVel(
        ForwardCmd * FMath::Cos(YawRad),
        ForwardCmd * FMath::Sin(YawRad),
        VzCmd);

    Interceptor->SetCameraAngles(0.0f, 0.0f);
    Interceptor->SetHeadingControl(EDroneYawMode::Angle, EDroneDrivetrainMode::ForwardOnly, YawCmdDeg);
    Interceptor->SetTargetVelocity(CmdVel);

    Runtime.LastEx = Ex;
    Runtime.LastEy = Ey;
    Runtime.LastAreaNorm = AreaNorm;
    Runtime.LastConf = Confidence;
    Runtime.LastYawCmdDeg = YawCmdDeg;
    Runtime.LastYawRateDeg = YawRateCmd;
    Runtime.LastCmdVel = CmdVel;

    Runtime.State = (AreaNorm >= Params.DesiredArea * 0.7f) ? EVisualState::Approach : EVisualState::Track;

    const bool bCentered = FMath::Abs(Ex) <= Params.CenterTolX && FMath::Abs(Ey) <= Params.CenterTolY;
    const bool bCloseEnough = AreaNorm >= Params.CaptureArea;

    if (bDetectionReal && bCentered && bCloseEnough)
    {
        Runtime.CaptureCount++;
    }
    else
    {
        Runtime.CaptureCount = 0;
    }

    if (Runtime.CaptureCount >= Params.CaptureHoldFrames)
    {
        Runtime.State = EVisualState::Captured;
        Runtime.bCaptured = true;
        bOutCaptured = true;
    }
    else
    {
        Runtime.bCaptured = false;
    }

    return true;
}

void UVisualInterceptController::ComputeSearchControl(ADronePawn* Interceptor, float Dt)
{
    if (!Interceptor)
    {
        return;
    }

    Runtime.SearchTime += Dt;

    Runtime.SearchCamYawDeg += Runtime.SearchDir * Params.SearchCamRateDeg * Dt;
    if (FMath::Abs(Runtime.SearchCamYawDeg) >= Params.SearchCamYawLimitDeg)
    {
        Runtime.SearchCamYawDeg = FMath::Clamp(Runtime.SearchCamYawDeg, -Params.SearchCamYawLimitDeg, Params.SearchCamYawLimitDeg);
        Runtime.SearchDir *= -1.0f;
    }

    Interceptor->SetCameraAngles(Params.SearchCamPitchDeg, Runtime.SearchCamYawDeg);

    const float CurrentYawDeg = Interceptor->CurrentState.GetRotator().Yaw;
    const float YawCmdDeg = NormalizeYawDeg(CurrentYawDeg + Runtime.SearchDir * Params.SearchBodyYawRateDeg * Dt);
    const float VzCmd = Params.SearchVzAmp * FMath::Sin(Runtime.SearchTime * 0.8f);

    const FVector CmdVel(0.0f, 0.0f, VzCmd);
    Interceptor->SetHeadingControl(EDroneYawMode::Angle, EDroneDrivetrainMode::ForwardOnly, YawCmdDeg);
    Interceptor->SetTargetVelocity(CmdVel);

    Runtime.State = EVisualState::Search;
    Runtime.LastEx = 0.0f;
    Runtime.LastEy = 0.0f;
    Runtime.LastAreaNorm = 0.0f;
    Runtime.LastConf = 0.0f;
    Runtime.LastYawCmdDeg = YawCmdDeg;
    Runtime.LastYawRateDeg = Runtime.SearchDir * Params.SearchBodyYawRateDeg;
    Runtime.LastCmdVel = CmdVel;
}

FString UVisualInterceptController::HandleStart(const TSharedPtr<FJsonObject>& CmdObj, UWorld* World)
{
    EnsureInitialized();

    if (!World)
    {
        return MakeError(TEXT("No World"));
    }

    ApplyParamsFromJson(CmdObj);
    FString RequestedMethod = Runtime.Method;
    if (CmdObj.IsValid() && CmdObj->HasField(TEXT("method")))
    {
        RequestedMethod = CmdObj->GetStringField(TEXT("method"));
    }

    FString CanonicalMethod;
    if (!NormalizeMethod(RequestedMethod, CanonicalMethod))
    {
        return MakeError(FString::Printf(TEXT("unsupported visual intercept method: %s"), *RequestedMethod));
    }
    Runtime.Method = CanonicalMethod;

    if (CmdObj.IsValid() && !CmdObj->HasField(TEXT("use_kalman")))
    {
        Params.bUseKalman = Runtime.Method.Contains(TEXT("kalman"));
    }

    ADronePawn* Interceptor = ResolveInterceptor(World, CmdObj);
    if (!Interceptor)
    {
        return MakeError(TEXT("interceptor drone not found"));
    }

    ResolveTarget(World, CmdObj);

    if (FeaturePredictor)
    {
        FeaturePredictor->Reset();
    }
    if (YawPid)
    {
        YawPid->Reset();
    }
    if (VerticalPid)
    {
        VerticalPid->Reset();
    }
    if (ForwardPid)
    {
        ForwardPid->Reset();
    }

    Runtime.bEnabled = true;
    Runtime.bCaptured = false;
    Runtime.State = EVisualState::Search;
    Runtime.FrameCount = 0;
    Runtime.DetectionCount = 0;
    Runtime.LostCount = 0;
    Runtime.CaptureCount = 0;
    Runtime.LastCmdVel = FVector::ZeroVector;
    Runtime.LastYawCmdDeg = Interceptor->CurrentState.GetRotator().Yaw;
    Runtime.SearchCamYawDeg = 0.0f;
    Runtime.SearchDir = 1.0f;
    Runtime.SearchTime = 0.0f;

    Interceptor->SetCameraAngles(Params.SearchCamPitchDeg, 0.0f);

    return BuildStartJson();
}

FString UVisualInterceptController::HandleUpdate(const TSharedPtr<FJsonObject>& CmdObj, UWorld* World)
{
    EnsureInitialized();

    if (!Runtime.bEnabled)
    {
        return MakeError(TEXT("visual intercept is not started"));
    }

    ADronePawn* Interceptor = ResolveInterceptor(World, CmdObj);
    if (!Interceptor)
    {
        return MakeError(TEXT("interceptor drone not found"));
    }

    ResolveTarget(World, CmdObj);

    float Dt = kDefaultDt;
    TryGetNumberAs(CmdObj, TEXT("dt"), Dt);
    Dt = SafeDt(Dt);
    SyncPidTimeStep(Dt);

    float FrameW = Runtime.LastFrameW;
    float FrameH = Runtime.LastFrameH;
    TryGetNumberAs(CmdObj, TEXT("image_w"), FrameW);
    TryGetNumberAs(CmdObj, TEXT("image_h"), FrameH);
    Runtime.LastFrameW = FMath::Max(1.0f, FrameW);
    Runtime.LastFrameH = FMath::Max(1.0f, FrameH);

    Runtime.FrameCount++;

    bool bHasDetection = false;
    if (CmdObj->HasField(TEXT("has_detection")))
    {
        bHasDetection = CmdObj->GetBoolField(TEXT("has_detection"));
    }
    else
    {
        bHasDetection = CmdObj->HasField(TEXT("cx")) && CmdObj->HasField(TEXT("cy"));
    }

    bool bValidControl = true;
    bool bCapturedThisFrame = false;

    if (bHasDetection)
    {
        float Cx = 0.0f;
        float Cy = 0.0f;
        float Area = 0.0f;
        float AreaRatio = -1.0f;
        float Confidence = 1.0f;

        TryGetNumberAs(CmdObj, TEXT("cx"), Cx);
        TryGetNumberAs(CmdObj, TEXT("cy"), Cy);
        TryGetNumberAs(CmdObj, TEXT("area"), Area);
        TryGetNumberAs(CmdObj, TEXT("area_ratio"), AreaRatio);
        TryGetNumberAs(CmdObj, TEXT("conf"), Confidence);

        if (Area <= 0.0f && AreaRatio > 0.0f)
        {
            Area = AreaRatio * Runtime.LastFrameW * Runtime.LastFrameH;
        }

        Area = FMath::Max(1.0f, Area);

        Runtime.DetectionCount++;
        Runtime.LostCount = 0;

        FVector Feature(Cx, Cy, Area);
        if (Params.bUseKalman && FeaturePredictor)
        {
            FeaturePredictor->Update(Feature, Dt);
            Feature = FeaturePredictor->GetEstimatedPosition();
            Feature.Z = FMath::Max(1.0f, Feature.Z);
        }

        bValidControl = ComputeTrackControl(
            Interceptor,
            Feature,
            Runtime.LastFrameW,
            Runtime.LastFrameH,
            Confidence,
            Dt,
            true,
            bCapturedThisFrame);
    }
    else
    {
        Runtime.LostCount++;
        Runtime.CaptureCount = 0;

        bool bUsedPrediction = false;
        if (Params.bUseKalman && FeaturePredictor && FeaturePredictor->IsInitialized() && Runtime.LostCount < Params.LostToSearchFrames)
        {
            FVector Pred = FeaturePredictor->PredictPosition(Dt);
            Pred.Z = FMath::Max(1.0f, Pred.Z);
            bUsedPrediction = true;
            bValidControl = ComputeTrackControl(
                Interceptor,
                Pred,
                Runtime.LastFrameW,
                Runtime.LastFrameH,
                0.0f,
                Dt,
                false,
                bCapturedThisFrame);
        }

        if (!bUsedPrediction)
        {
            ComputeSearchControl(Interceptor, Dt);
        }
    }

    if (bCapturedThisFrame && Params.bStopOnCapture)
    {
        Interceptor->Hover();
        Runtime.bEnabled = false;
    }

    return BuildUpdateJson(bValidControl);
}

FString UVisualInterceptController::HandleStop(const TSharedPtr<FJsonObject>& CmdObj, UWorld* World)
{
    EnsureInitialized();

    ADronePawn* Interceptor = ResolveInterceptor(World, CmdObj);
    if (Interceptor)
    {
        Interceptor->Hover();
    }

    Runtime.bEnabled = false;
    Runtime.State = Runtime.bCaptured ? EVisualState::Captured : EVisualState::Idle;

    return MakeOk(TEXT("visual intercept stopped"));
}

FString UVisualInterceptController::HandleState() const
{
    return FString::Printf(
        TEXT("{\"status\":\"ok\",\"mode\":\"visual_intercept\",\"enabled\":%s,\"state\":\"%s\",\"captured\":%s,\"interceptor_id\":\"%s\",\"target_id\":\"%s\",\"method\":\"%s\",\"frame\":%d,\"detections\":%d,\"lost_count\":%d,\"capture_count\":%d,\"cmd_velocity\":[%.4f,%.4f,%.4f],\"control\":{\"ex\":%.5f,\"ey\":%.5f,\"area_norm\":%.6f,\"conf\":%.4f,\"yaw_cmd_deg\":%.4f,\"yaw_rate_deg\":%.4f}}"),
        *BoolLiteral(Runtime.bEnabled),
        *StateToString(Runtime.State),
        *BoolLiteral(Runtime.bCaptured),
        *Runtime.InterceptorId,
        *Runtime.TargetId,
        *Runtime.Method,
        Runtime.FrameCount,
        Runtime.DetectionCount,
        Runtime.LostCount,
        Runtime.CaptureCount,
        Runtime.LastCmdVel.X,
        Runtime.LastCmdVel.Y,
        Runtime.LastCmdVel.Z,
        Runtime.LastEx,
        Runtime.LastEy,
        Runtime.LastAreaNorm,
        Runtime.LastConf,
        Runtime.LastYawCmdDeg,
        Runtime.LastYawRateDeg);
}





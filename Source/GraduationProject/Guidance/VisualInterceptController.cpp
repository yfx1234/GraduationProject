#include "VisualInterceptController.h"

#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"
#include "GraduationProject/Core/Controller/PIDController.h"
#include "GraduationProject/Core/Manager/AgentManager.h"
#include "GameFramework/Actor.h"
#include "GraduationProject/Drone/DronePawn.h"
#include "KalmanPredictor.h"
#include "Math/RotationMatrix.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"

namespace
{
constexpr float kDefaultDt = 0.08f;

template <typename T>
bool TryGetNumberAs(const TSharedPtr<FJsonObject>& Obj, const TCHAR* Field, T& OutValue)
{
    double Temp = 0.0;
    if (!Obj.IsValid() || !Obj->TryGetNumberField(Field, Temp))
    {
        return false;
    }


    OutValue = static_cast<T>(Temp);
    return true;
}

bool TryGetBool(const TSharedPtr<FJsonObject>& Obj, const TCHAR* Field, bool& OutValue)
{
    return Obj.IsValid() && Obj->TryGetBoolField(Field, OutValue);
}

bool TryGetString(const TSharedPtr<FJsonObject>& Obj, const TCHAR* Field, FString& OutValue)
{
    return Obj.IsValid() && Obj->TryGetStringField(Field, OutValue);
}

FString ToJsonString(const TSharedPtr<FJsonObject>& Obj)
{
    FString Output;
    const TSharedRef<TJsonWriter<>> Writer = TJsonWriterFactory<>::Create(&Output);
    FJsonSerializer::Serialize(Obj.ToSharedRef(), Writer);
    return Output;
}

TArray<TSharedPtr<FJsonValue>> MakeVectorArray(const FVector& Value)
{
    TArray<TSharedPtr<FJsonValue>> Array;
    Array.Add(MakeShareable(new FJsonValueNumber(Value.X)));
    Array.Add(MakeShareable(new FJsonValueNumber(Value.Y)));
    Array.Add(MakeShareable(new FJsonValueNumber(Value.Z)));
    return Array;
}

TSharedPtr<FJsonObject> MakeFeatureObject(bool bValid, bool bUsedPrediction, const FVector& Feature, float LeadTime)
{
    const TSharedPtr<FJsonObject> Obj = MakeShareable(new FJsonObject);
    Obj->SetBoolField(TEXT("valid"), bValid);
    Obj->SetBoolField(TEXT("used_prediction"), bUsedPrediction);
    Obj->SetNumberField(TEXT("cx"), bValid ? Feature.X : 0.0f);
    Obj->SetNumberField(TEXT("cy"), bValid ? Feature.Y : 0.0f);
    Obj->SetNumberField(TEXT("area"), bValid ? Feature.Z : 0.0f);
    Obj->SetNumberField(TEXT("lead_time"), bValid ? LeadTime : 0.0f);
    return Obj;
}

bool IsFiniteFeature(const FVector& Feature)
{
    return FMath::IsFinite(Feature.X) && FMath::IsFinite(Feature.Y) && FMath::IsFinite(Feature.Z);
}

FVector ClampFeatureToFrame(const FVector& Feature, float FrameW, float FrameH)
{
    const float SafeW = FMath::Max(FrameW, 2.0f);
    const float SafeH = FMath::Max(FrameH, 2.0f);
    return FVector(
        FMath::Clamp(Feature.X, 0.0f, SafeW - 1.0f),
        FMath::Clamp(Feature.Y, 0.0f, SafeH - 1.0f),
        FMath::Clamp(Feature.Z, 0.0f, 1.0f));
}

FVector PixelsToNormalized(const FVector& Feature, float FrameW, float FrameH)
{
    const float SafeW = FMath::Max(FrameW, 2.0f);
    const float SafeH = FMath::Max(FrameH, 2.0f);
    return FVector(
        FMath::Clamp(Feature.X / SafeW, 0.0f, 1.0f),
        FMath::Clamp(Feature.Y / SafeH, 0.0f, 1.0f),
        FMath::Clamp(Feature.Z, 0.0f, 1.0f));
}

FVector NormalizedToPixels(const FVector& Feature, float FrameW, float FrameH)
{
    const float SafeW = FMath::Max(FrameW, 2.0f);
    const float SafeH = FMath::Max(FrameH, 2.0f);
    return FVector(
        FMath::Clamp(Feature.X, 0.0f, 1.0f) * SafeW,
        FMath::Clamp(Feature.Y, 0.0f, 1.0f) * SafeH,
        FMath::Clamp(Feature.Z, 0.0f, 1.0f));
}

ADronePawn* GetDrone(AActor* Actor)
{
    return Cast<ADronePawn>(Actor);
}

const ADronePawn* GetDrone(const AActor* Actor)
{
    return Cast<ADronePawn>(Actor);
}

AActor* FindDroneById(UAgentManager* Manager, const FString& AgentId)
{
    if (!Manager || AgentId.IsEmpty())
    {
        return nullptr;
    }

    AActor* Actor = Manager->GetAgent(AgentId);
    return GetDrone(Actor) ? Actor : nullptr;
}

AActor* FindDroneByRole(UAgentManager* Manager, EDroneMissionRole Role, const FString& ExcludeId = TEXT(""))
{
    if (!Manager)
    {
        return nullptr;
    }

    for (const FString& AgentId : Manager->GetAllAgentIds())
    {
        AActor* Actor = Manager->GetAgent(AgentId);
        const ADronePawn* Drone = GetDrone(Actor);
        if (!Drone)
        {
            continue;
        }

        if (!ExcludeId.IsEmpty() && Drone->DroneId == ExcludeId)
        {
            continue;
        }

        if (Drone->MissionRole == Role)
        {
            return Actor;
        }
    }

    return nullptr;
}

FString GetAgentId(const AActor* Actor)
{
    if (const ADronePawn* Drone = GetDrone(Actor))
    {
        return Drone->DroneId;
    }

    return Actor ? Actor->GetName() : FString();
}

FVector GetAgentPosition(const AActor* Actor)
{
    if (const ADronePawn* Drone = GetDrone(Actor))
    {
        return Drone->GetCurrentPosition();
    }

    return Actor ? Actor->GetActorLocation() / 100.0f : FVector::ZeroVector;
}
FVector GetAgentVelocity(const AActor* Actor)
{
    if (const ADronePawn* Drone = GetDrone(Actor))
    {
        return Drone->GetCurrentVelocity();
    }
    return Actor ? Actor->GetVelocity() / 100.0f : FVector::ZeroVector;
}
void SetDroneCameraAngles(AActor* Actor, float TargetPitch, float TargetYaw)
{
    if (ADronePawn* Drone = GetDrone(Actor))
    {
        Drone->SetCameraAngles(TargetPitch, TargetYaw);
    }
}

void SetDroneHeadingControl(AActor* Actor, EDroneYawMode NewYawMode, EDroneDrivetrainMode NewDrivetrain, float YawDeg)
{
    if (ADronePawn* Drone = GetDrone(Actor))
    {
        Drone->SetHeadingControl(NewYawMode, NewDrivetrain, YawDeg);
    }
}

void SetDroneTargetVelocity(AActor* Actor, const FVector& NewTargetVelocity)
{
    if (ADronePawn* Drone = GetDrone(Actor))
    {
        Drone->SetTargetVelocity(NewTargetVelocity);
    }
}

void HoverDrone(AActor* Actor)
{
    if (ADronePawn* Drone = GetDrone(Actor))
    {
        Drone->Hover();
    }
}
} // namespace

void UVisualInterceptController::EnsureInitialized()
{
    if (!FeaturePredictor)
    {
        FeaturePredictor = NewObject<UKalmanPredictor>(this);
        FeaturePredictor->Initialize(6.0f, 0.003f);
    }

    if (!YawPid)
    {
        YawPid = NewObject<UPIDController>(this);
        // 娑擃厽鏋冨▔銊╁櫞閿涙艾浜搁懜顏嗗箚閸忓牊鏁规稉鈧悙鐧哥礉閸氬酣娼伴崘宥囨暏閸涙垝鎶ら獮铏拨閿涘矂浼╅崗宥嗘簚婢舵潙涔忛崣铏降閸ョ偟鏁ラ妴?
        YawPid->Initialize(112.0, 0.0, 16.0, 0.04, Params.MaxYawRateDeg, kDefaultDt, -20.0, 20.0);
    }

    if (!VerticalPid)
    {
        VerticalPid = NewObject<UPIDController>(this);
        VerticalPid->Initialize(3.0, 0.0, 0.45, 0.04, Params.MaxVerticalSpeed, kDefaultDt, -10.0, 10.0);
    }

    if (!ForwardPid)
    {
        ForwardPid = NewObject<UPIDController>(this);
        ForwardPid->Initialize(40.0, 0.0, 0.0, 0.05, Params.MaxForwardSpeed, kDefaultDt, -10.0, 10.0);
    }

    SyncPidTimeStep(kDefaultDt);
}

void UVisualInterceptController::ResetPidControllers()
{
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
    }

    ResetPidControllers();
    ResetRuntimeOnly();
    Runtime.Method = Params.bUseKalman ? TEXT("vision_pid_kalman") : TEXT("vision_pid");
}

void UVisualInterceptController::BeginSession(const AActor* Interceptor)
{
    if (FeaturePredictor)
    {
        FeaturePredictor->Reset();
    }

    ResetPidControllers();
    ResetRuntimeOnly();

    Runtime.bEnabled = true;
    Runtime.bCaptured = false;
    Runtime.State = EVisualState::Search;
    Runtime.LastYawCmdDeg = Interceptor ? Interceptor->GetActorRotation().Yaw : 0.0f;
}
AActor* UVisualInterceptController::ResolveInterceptor(const TSharedPtr<FJsonObject>& CmdObj)
{
    UAgentManager* Manager = UAgentManager::GetInstance();
    if (!Manager)
    {
        return nullptr;
    }

    FString RequestedId;
    TryGetString(CmdObj, TEXT("interceptor_id"), RequestedId);

    if (AActor* Drone = FindDroneById(Manager, RequestedId))
    {
        return Drone;
    }

    if (AActor* Drone = FindDroneById(Manager, Runtime.InterceptorId))
    {
        return Drone;
    }

    if (AActor* Drone = FindDroneByRole(Manager, EDroneMissionRole::Interceptor))
    {
        return Drone;
    }

    const TArray<FString> FallbackIds = {TEXT("uav_1"), TEXT("uav_0"), TEXT("drone_1"), TEXT("drone_0")};
    for (const FString& FallbackId : FallbackIds)
    {
        if (AActor* Drone = FindDroneById(Manager, FallbackId))
        {
            return Drone;
        }
    }

    for (const FString& AgentId : Manager->GetAllAgentIds())
    {
        if (AActor* Drone = FindDroneById(Manager, AgentId))
        {
            return Drone;
        }
    }

    return nullptr;
}

AActor* UVisualInterceptController::ResolveTarget(const TSharedPtr<FJsonObject>& CmdObj)
{
    UAgentManager* Manager = UAgentManager::GetInstance();
    if (!Manager)
    {
        return nullptr;
    }

    FString RequestedId;
    TryGetString(CmdObj, TEXT("target_id"), RequestedId);

    FString ExcludeId = Runtime.InterceptorId;
    FString RequestedInterceptorId;
    if (TryGetString(CmdObj, TEXT("interceptor_id"), RequestedInterceptorId) && !RequestedInterceptorId.IsEmpty())
    {
        ExcludeId = RequestedInterceptorId;
    }

    if (AActor* Interceptor = FindDroneById(Manager, ExcludeId))
    {
        ExcludeId = GetAgentId(Interceptor);
    }

    if (AActor* Drone = FindDroneById(Manager, RequestedId))
    {
        if (GetAgentId(Drone) != ExcludeId)
        {
            return Drone;
        }
    }

    if (AActor* Drone = FindDroneById(Manager, Runtime.TargetId))
    {
        if (GetAgentId(Drone) != ExcludeId)
        {
            return Drone;
        }
    }

    if (AActor* Drone = FindDroneByRole(Manager, EDroneMissionRole::Target, ExcludeId))
    {
        return Drone;
    }

    const TArray<FString> FallbackIds = {TEXT("uav_0"), TEXT("uav_1"), TEXT("drone_0"), TEXT("drone_1")};
    for (const FString& FallbackId : FallbackIds)
    {
        if (FallbackId == ExcludeId)
        {
            continue;
        }

        if (AActor* Drone = FindDroneById(Manager, FallbackId))
        {
            if (GetAgentId(Drone) != ExcludeId)
            {
                return Drone;
            }
        }
    }

    for (const FString& AgentId : Manager->GetAllAgentIds())
    {
        if (AActor* Drone = FindDroneById(Manager, AgentId))
        {
            if (GetAgentId(Drone) != ExcludeId)
            {
                return Drone;
            }
        }
    }

    return nullptr;
}

FString UVisualInterceptController::BoolLiteral(bool bValue)
{
    return bValue ? TEXT("true") : TEXT("false");
}

float UVisualInterceptController::NormalizeYawDeg(float YawDeg)
{
    float Value = FMath::Fmod(YawDeg + 180.0f, 360.0f);
    if (Value < 0.0f)
    {
        Value += 360.0f;
    }
    return Value - 180.0f;
}

FString UVisualInterceptController::StateToString(EVisualState State)
{
    switch (State)
    {
    case EVisualState::Idle:
        return TEXT("IDLE");
    case EVisualState::Search:
        return TEXT("SEARCH");
    case EVisualState::Track:
        return TEXT("TRACK");
    case EVisualState::Approach:
        return TEXT("APPROACH");
    case EVisualState::Captured:
        return TEXT("CAPTURED");
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
    const TSharedPtr<FJsonObject> Root = MakeShareable(new FJsonObject);
    Root->SetStringField(TEXT("status"), TEXT("error"));
    Root->SetStringField(TEXT("mode"), TEXT("visual_intercept"));
    Root->SetStringField(TEXT("message"), Msg);
    return ToJsonString(Root);
}

FString UVisualInterceptController::MakeOk(const FString& Msg) const
{
    const TSharedPtr<FJsonObject> Root = MakeShareable(new FJsonObject);
    Root->SetStringField(TEXT("status"), TEXT("ok"));
    Root->SetStringField(TEXT("mode"), TEXT("visual_intercept"));
    Root->SetStringField(TEXT("message"), Msg);
    return ToJsonString(Root);
}

float UVisualInterceptController::SafeDt(float Dt) const
{
    if (!FMath::IsFinite(Dt) || Dt <= KINDA_SMALL_NUMBER)
    {
        return kDefaultDt;
    }

    return FMath::Clamp(Dt, 0.01f, 0.25f);
}

void UVisualInterceptController::SyncPidTimeStep(float Dt)
{
    const double SafeStep = static_cast<double>(SafeDt(Dt));

    if (YawPid)
    {
        YawPid->SetTimeStep(SafeStep);
    }

    if (VerticalPid)
    {
        VerticalPid->SetTimeStep(SafeStep);
    }

    if (ForwardPid)
    {
        ForwardPid->SetTimeStep(SafeStep);
    }
}
void UVisualInterceptController::ApplyParamsFromJson(const TSharedPtr<FJsonObject>& CmdObj)
{
    FString CanonicalMethod = Runtime.Method.IsEmpty() ? TEXT("vision_pid_kalman") : Runtime.Method;
    FString MethodText;
    if (TryGetString(CmdObj, TEXT("method"), MethodText))
    {
        NormalizeMethod(MethodText, CanonicalMethod);
    }

    bool bUsePd = CanonicalMethod.StartsWith(TEXT("vision_pd"));
    Params.bUseKalman = CanonicalMethod.Contains(TEXT("kalman"));

    bool BoolValue = false;
    float FloatValue = 0.0f;
    int32 IntValue = 0;

    if (TryGetBool(CmdObj, TEXT("use_kalman"), BoolValue))
    {
        Params.bUseKalman = BoolValue;
    }

    if (TryGetBool(CmdObj, TEXT("stop_on_capture"), BoolValue))
    {
        Params.bStopOnCapture = BoolValue;
    }

    if (TryGetNumberAs(CmdObj, TEXT("desired_area"), FloatValue))
    {
        Params.DesiredArea = FMath::Clamp(FloatValue, 0.002f, 0.95f);
    }

    if (TryGetNumberAs(CmdObj, TEXT("capture_area"), FloatValue))
    {
        Params.CaptureArea = FMath::Clamp(FloatValue, Params.DesiredArea, 0.98f);
    }

    if (TryGetNumberAs(CmdObj, TEXT("center_tol_x"), FloatValue))
    {
        Params.CenterTolX = FMath::Clamp(FloatValue, 0.005f, 0.5f);
    }

    if (TryGetNumberAs(CmdObj, TEXT("center_tol_y"), FloatValue))
    {
        Params.CenterTolY = FMath::Clamp(FloatValue, 0.005f, 0.5f);
    }

    if (TryGetNumberAs(CmdObj, TEXT("capture_hold_frames"), IntValue))
    {
        Params.CaptureHoldFrames = FMath::Clamp(IntValue, 1, 120);
    }

    if (TryGetNumberAs(CmdObj, TEXT("lost_to_search_frames"), IntValue))
    {
        Params.LostToSearchFrames = FMath::Clamp(IntValue, 1, 240);
    }

    if (TryGetNumberAs(CmdObj, TEXT("max_forward_speed"), FloatValue))
    {
        Params.MaxForwardSpeed = FMath::Clamp(FloatValue, 0.5f, 60.0f);
    }

    if (TryGetNumberAs(CmdObj, TEXT("max_reverse_speed"), FloatValue))
    {
        Params.MaxReverseSpeed = FMath::Clamp(FloatValue, 0.0f, 20.0f);
    }

    if (TryGetNumberAs(CmdObj, TEXT("max_vertical_speed"), FloatValue))
    {
        Params.MaxVerticalSpeed = FMath::Clamp(FloatValue, 0.2f, 20.0f);
    }

    if (TryGetNumberAs(CmdObj, TEXT("max_yaw_rate_deg"), FloatValue))
    {
        Params.MaxYawRateDeg = FMath::Clamp(FloatValue, 1.0f, 360.0f);
    }

    if (TryGetNumberAs(CmdObj, TEXT("ram_area_target"), FloatValue))
    {
        Params.RamAreaTarget = FMath::Clamp(FloatValue, Params.CaptureArea, 0.99f);
    }

    if (TryGetNumberAs(CmdObj, TEXT("min_ram_speed"), FloatValue))
    {
        Params.MinRamSpeed = FMath::Clamp(FloatValue, 0.0f, Params.MaxForwardSpeed);
    }

    if (TryGetNumberAs(CmdObj, TEXT("intercept_distance"), FloatValue))
    {
        Params.InterceptDistance = FMath::Clamp(FloatValue, 0.1f, 20.0f);
    }

    if (TryGetNumberAs(CmdObj, TEXT("track_lead_time"), FloatValue))
    {
        Params.TrackLeadTime = FMath::Clamp(FloatValue, 0.0f, 2.0f);
    }

    if (TryGetNumberAs(CmdObj, TEXT("ram_lead_time"), FloatValue))
    {
        Params.RamLeadTime = FMath::Clamp(FloatValue, 0.0f, 3.0f);
    }

    if (TryGetNumberAs(CmdObj, TEXT("search_cam_yaw_limit_deg"), FloatValue))
    {
        Params.SearchCamYawLimitDeg = FMath::Max(0.0f, FloatValue);
    }

    if (TryGetNumberAs(CmdObj, TEXT("search_cam_rate_deg"), FloatValue))
    {
        Params.SearchCamRateDeg = FMath::Max(0.0f, FloatValue);
    }

    if (TryGetNumberAs(CmdObj, TEXT("search_body_yaw_rate_deg"), FloatValue))
    {
        Params.SearchBodyYawRateDeg = FMath::Max(0.0f, FloatValue);
    }

    if (TryGetNumberAs(CmdObj, TEXT("search_cam_pitch_deg"), FloatValue))
    {
        Params.SearchCamPitchDeg = FMath::Clamp(FloatValue, -89.0f, 89.0f);
    }

    if (TryGetNumberAs(CmdObj, TEXT("search_vz_amp"), FloatValue))
    {
        Params.SearchVzAmp = FMath::Abs(FloatValue);
    }

    Runtime.Method = bUsePd
        ? (Params.bUseKalman ? TEXT("vision_pd_kalman") : TEXT("vision_pd"))
        : (Params.bUseKalman ? TEXT("vision_pid_kalman") : TEXT("vision_pid"));

    if (YawPid)
    {
        // 娑擃厽鏋冨▔銊╁櫞閿涙俺绻栭柌灞芥嫲閸掓繂顫愰崠鏍︾箽閹镐礁鎮撴稉鈧總妤佹纯缁嬪磭娈戦崑蹇氬焻閸欏倹鏆熼敍灞藉櫤鐏忔垿顣╁ù瀣嚖瀹割喖鐢弶銉ф畱閹藉棗銇旈妴?
        YawPid->SetParameters(112.0, 0.0, bUsePd ? 20.0 : 16.0, 0.04, Params.MaxYawRateDeg);
    }

    if (VerticalPid)
    {
        VerticalPid->SetParameters(3.0, 0.0, 0.45, 0.04, Params.MaxVerticalSpeed);
    }

    if (ForwardPid)
    {
        ForwardPid->SetParameters(40.0, 0.0, 0.0, 0.05, Params.MaxForwardSpeed);
    }

    if (!Params.bUseKalman && FeaturePredictor)
    {
        FeaturePredictor->Reset();
    }
}

FString UVisualInterceptController::BuildStartJson() const
{
    const TSharedPtr<FJsonObject> Root = MakeShareable(new FJsonObject);
    const TSharedPtr<FJsonObject> ParamsObj = MakeShareable(new FJsonObject);

    Root->SetStringField(TEXT("status"), TEXT("ok"));
    Root->SetStringField(TEXT("mode"), TEXT("visual_intercept"));
    Root->SetStringField(TEXT("action"), TEXT("start"));
    Root->SetBoolField(TEXT("enabled"), Runtime.bEnabled);
    Root->SetBoolField(TEXT("captured"), Runtime.bCaptured);
    Root->SetStringField(TEXT("state"), StateToString(Runtime.State));
    Root->SetStringField(TEXT("interceptor_id"), Runtime.InterceptorId);
    Root->SetStringField(TEXT("target_id"), Runtime.TargetId);
    Root->SetStringField(TEXT("method"), Runtime.Method);
    Root->SetNumberField(TEXT("target_distance"), Runtime.LastTargetDistance);

    ParamsObj->SetNumberField(TEXT("desired_area"), Params.DesiredArea);
    ParamsObj->SetNumberField(TEXT("capture_area"), Params.CaptureArea);
    ParamsObj->SetNumberField(TEXT("center_tol_x"), Params.CenterTolX);
    ParamsObj->SetNumberField(TEXT("center_tol_y"), Params.CenterTolY);
    ParamsObj->SetNumberField(TEXT("capture_hold_frames"), Params.CaptureHoldFrames);
    ParamsObj->SetNumberField(TEXT("lost_to_search_frames"), Params.LostToSearchFrames);
    ParamsObj->SetNumberField(TEXT("max_forward_speed"), Params.MaxForwardSpeed);
    ParamsObj->SetNumberField(TEXT("max_reverse_speed"), Params.MaxReverseSpeed);
    ParamsObj->SetNumberField(TEXT("max_vertical_speed"), Params.MaxVerticalSpeed);
    ParamsObj->SetNumberField(TEXT("max_yaw_rate_deg"), Params.MaxYawRateDeg);
    ParamsObj->SetNumberField(TEXT("ram_area_target"), Params.RamAreaTarget);
    ParamsObj->SetNumberField(TEXT("min_ram_speed"), Params.MinRamSpeed);
    ParamsObj->SetNumberField(TEXT("intercept_distance"), Params.InterceptDistance);
    ParamsObj->SetNumberField(TEXT("track_lead_time"), Params.TrackLeadTime);
    ParamsObj->SetNumberField(TEXT("ram_lead_time"), Params.RamLeadTime);
    ParamsObj->SetNumberField(TEXT("search_cam_yaw_limit_deg"), Params.SearchCamYawLimitDeg);
    ParamsObj->SetNumberField(TEXT("search_cam_rate_deg"), Params.SearchCamRateDeg);
    ParamsObj->SetNumberField(TEXT("search_body_yaw_rate_deg"), Params.SearchBodyYawRateDeg);
    ParamsObj->SetNumberField(TEXT("search_cam_pitch_deg"), Params.SearchCamPitchDeg);
    ParamsObj->SetNumberField(TEXT("search_vz_amp"), Params.SearchVzAmp);
    ParamsObj->SetBoolField(TEXT("stop_on_capture"), Params.bStopOnCapture);
    ParamsObj->SetBoolField(TEXT("use_kalman"), Params.bUseKalman);
    Root->SetObjectField(TEXT("params"), ParamsObj);

    return ToJsonString(Root);
}

FString UVisualInterceptController::BuildUpdateJson(bool bValidControl) const
{
    const TSharedPtr<FJsonObject> Root = MakeShareable(new FJsonObject);
    const TSharedPtr<FJsonObject> ControlObj = MakeShareable(new FJsonObject);
    const TSharedPtr<FJsonObject> KalmanObj = MakeShareable(new FJsonObject);

    Root->SetStringField(TEXT("status"), TEXT("ok"));
    Root->SetStringField(TEXT("mode"), TEXT("visual_intercept"));
    Root->SetBoolField(TEXT("enabled"), Runtime.bEnabled);
    Root->SetBoolField(TEXT("valid"), bValidControl);
    Root->SetStringField(TEXT("state"), StateToString(Runtime.State));
    Root->SetBoolField(TEXT("captured"), Runtime.bCaptured);
    Root->SetStringField(TEXT("interceptor_id"), Runtime.InterceptorId);
    Root->SetStringField(TEXT("target_id"), Runtime.TargetId);
    Root->SetStringField(TEXT("method"), Runtime.Method);
    Root->SetNumberField(TEXT("frame"), Runtime.FrameCount);
    Root->SetNumberField(TEXT("detections"), Runtime.DetectionCount);
    Root->SetNumberField(TEXT("lost_count"), Runtime.LostCount);
    Root->SetNumberField(TEXT("capture_count"), Runtime.CaptureCount);
    Root->SetNumberField(TEXT("target_distance"), Runtime.LastTargetDistance);
    Root->SetArrayField(TEXT("cmd_velocity"), MakeVectorArray(Runtime.LastCmdVel));
    Root->SetNumberField(TEXT("processing_latency"), Runtime.LastProcessingLatency);

    ControlObj->SetNumberField(TEXT("ex"), Runtime.LastEx);
    ControlObj->SetNumberField(TEXT("ey"), Runtime.LastEy);
    ControlObj->SetNumberField(TEXT("area_norm"), Runtime.LastAreaNorm);
    ControlObj->SetNumberField(TEXT("conf"), Runtime.LastConf);
    ControlObj->SetNumberField(TEXT("yaw_cmd_deg"), Runtime.LastYawCmdDeg);
    ControlObj->SetNumberField(TEXT("yaw_rate_deg"), Runtime.LastYawRateDeg);
    ControlObj->SetNumberField(TEXT("processing_latency_ms"), Runtime.LastProcessingLatency * 1000.0f);
    Root->SetObjectField(TEXT("control"), ControlObj);

    Root->SetObjectField(TEXT("measurement"), MakeFeatureObject(Runtime.bLastHasMeasurement, false, Runtime.LastMeasurementFeature, 0.0f));
    Root->SetObjectField(TEXT("track"), MakeFeatureObject(Runtime.bLastHasTrackFeature, Runtime.bLastUsedPrediction, Runtime.LastTrackFeature, Runtime.LastPredictionLeadTime));
    Root->SetObjectField(TEXT("prediction"), MakeFeatureObject(Runtime.bLastHasPrediction, false, Runtime.LastPredictionFeature, Runtime.LastPredictionLeadTime));

    KalmanObj->SetBoolField(TEXT("valid"), Runtime.bLastKalmanValid);
    KalmanObj->SetArrayField(TEXT("pos"), MakeVectorArray(Runtime.LastKalmanPosition));
    KalmanObj->SetArrayField(TEXT("vel"), MakeVectorArray(Runtime.LastKalmanVelocity));
    KalmanObj->SetArrayField(TEXT("acc"), MakeVectorArray(Runtime.LastKalmanAcceleration));
    KalmanObj->SetNumberField(TEXT("uncertainty"), Runtime.LastKalmanUncertainty);
    KalmanObj->SetNumberField(TEXT("q"), Runtime.LastKalmanProcessNoise);
    Root->SetObjectField(TEXT("kalman"), KalmanObj);

    return ToJsonString(Root);
}
bool UVisualInterceptController::ComputeTrackControl(
    AActor* Interceptor,
    const FVector& Feature,
    float FrameW,
    float FrameH,
    float Confidence,
    float Dt,
    bool bDetectionReal,
    float TargetDistance,
    bool& bOutCaptured)
{
    bOutCaptured = false;
    if (!Interceptor || FrameW < 2.0f || FrameH < 2.0f || !IsFiniteFeature(Feature))
    {
        return false;
    }

    const FVector ClampedFeature = ClampFeatureToFrame(Feature, FrameW, FrameH);
    const float HalfW = 0.5f * FrameW;
    const float HalfH = 0.5f * FrameH;
    const float Ex = FMath::Clamp((ClampedFeature.X - HalfW) / FMath::Max(HalfW, 1.0f), -1.5f, 1.5f);
    const float Ey = FMath::Clamp((ClampedFeature.Y - HalfH) / FMath::Max(HalfH, 1.0f), -1.5f, 1.5f);
    const float AreaNorm = FMath::Clamp(ClampedFeature.Z, 0.0f, 1.0f);

    const bool bCentered = FMath::Abs(Ex) <= Params.CenterTolX && FMath::Abs(Ey) <= Params.CenterTolY;
    // 娑擃厽鏋冨▔銊╁櫞閿涙碍娲块弮鈺勭箻閸忋儴绻庣捄婵囧閹搭亷绱濋柆鍨帳闂€鎸庢闂傛潙浠犻悾娆忔躬 TRACK 鐠佲晜瀚ら幋顏呮▔瀵版瀚嬪▽鎾扁偓?
    const float NearDistance = FMath::Max(Params.InterceptDistance * 5.6f, 7.0f);
    const bool bNearTarget = TargetDistance > 0.0f && TargetDistance <= NearDistance;
    const bool bRamPhase = AreaNorm >= Params.RamAreaTarget * 0.48f || bNearTarget;

    // Close-range interception should get more aggressive, not softer.
    const float AreaSpan = FMath::Max(Params.RamAreaTarget - Params.DesiredArea, 0.02f);
    const float CloseByArea = FMath::Clamp((AreaNorm - Params.DesiredArea) / AreaSpan, 0.0f, 1.0f);
    const float CloseByDistance = (TargetDistance > 0.0f)
        ? FMath::Clamp((NearDistance - TargetDistance) / FMath::Max(NearDistance - Params.InterceptDistance, 0.5f), 0.0f, 1.0f)
        : 0.0f;
    const float CloseFactor = FMath::Max(CloseByArea, CloseByDistance);

    Runtime.State = bRamPhase ? EVisualState::Approach : EVisualState::Track;
    SetDroneCameraAngles(Interceptor, 0.0f, 0.0f);
    const FVector CurrentVelocity = GetAgentVelocity(Interceptor);
    const FVector CurrentPlanarVelocity(CurrentVelocity.X, CurrentVelocity.Y, 0.0f);
    const float RawYawRate = static_cast<float>(YawPid ? YawPid->Update(0.0, -static_cast<double>(Ex), false) : Ex * Params.MaxYawRateDeg);
    const float YawBoost = bRamPhase ? FMath::Lerp(1.00f, 1.08f, CloseFactor) : 0.98f;
    float YawRateCmd = FMath::Clamp(RawYawRate * YawBoost, -Params.MaxYawRateDeg, Params.MaxYawRateDeg);
    const float MinYawRate = bRamPhase ? FMath::Lerp(3.0f, FMath::Min(Params.MaxYawRateDeg * 0.12f, 20.0f), CloseFactor) : 0.0f;
    const float MinYawActivation = bRamPhase ? FMath::Max(Params.CenterTolX * 1.05f, 0.022f) : FMath::Max(Params.CenterTolX * 1.55f, 0.032f);
    if (FMath::Abs(Ex) > MinYawActivation)
    {
        const float YawSign = Ex >= 0.0f ? 1.0f : -1.0f;
        const float MinYawCmd = MinYawRate * FMath::Clamp(FMath::Abs(Ex) / 0.20f, 0.25f, 1.0f);
        if (FMath::Abs(YawRateCmd) < MinYawCmd)
        {
            YawRateCmd = YawSign * MinYawCmd;
        }
    }
    const float YawDeadband = bRamPhase ? FMath::Max(Params.CenterTolX * 0.95f, 0.020f) : FMath::Max(Params.CenterTolX * 1.10f, 0.026f);
    if (FMath::Abs(Ex) < YawDeadband)
    {
        const float Ratio = FMath::Clamp(FMath::Abs(Ex) / FMath::Max(YawDeadband, 0.001f), 0.0f, 1.0f);
        YawRateCmd *= Ratio;
    }
    const float PrevYawRateCmd = Runtime.LastYawRateDeg;
    const bool bYawReversing = PrevYawRateCmd * YawRateCmd < 0.0f;
    const float ReverseWindow = bRamPhase ? 0.24f : 0.30f;
    if (bYawReversing && FMath::Abs(Ex) < ReverseWindow)
    {
        const float ReverseRatio = FMath::Clamp(FMath::Abs(Ex) / FMath::Max(ReverseWindow, 0.001f), 0.0f, 1.0f);
        const float ReverseScale = bRamPhase
            ? FMath::Lerp(0.18f, 0.34f, ReverseRatio)
            : FMath::Lerp(0.26f, 0.42f, ReverseRatio);
        YawRateCmd *= ReverseScale;
    }
    const float SmoothedYawRate = FMath::FInterpTo(PrevYawRateCmd, YawRateCmd, Dt, bRamPhase ? 5.4f : 4.4f);
    const float MaxYawStep = (bRamPhase ? 140.0f : 115.0f) * Dt;
    YawRateCmd = FMath::Clamp(SmoothedYawRate, PrevYawRateCmd - MaxYawStep, PrevYawRateCmd + MaxYawStep);
    const float DesiredEy = bRamPhase ? -FMath::Lerp(0.02f, 0.06f, CloseFactor) : 0.0f;
    float VerticalCmd = FMath::Clamp(
        static_cast<float>(VerticalPid ? VerticalPid->Update(static_cast<double>(DesiredEy), static_cast<double>(Ey), false) : (DesiredEy - Ey) * Params.MaxVerticalSpeed),
        -Params.MaxVerticalSpeed,
        Params.MaxVerticalSpeed);
    if (bRamPhase)
    {
        VerticalCmd *= 0.82f;
        if (VerticalCmd > 0.0f)
        {
            VerticalCmd *= FMath::Lerp(0.65f, 0.35f, CloseFactor);
        }
    }
    const float TargetArea = bRamPhase ? FMath::Max(Params.RamAreaTarget, Params.DesiredArea) : Params.DesiredArea;
    float ForwardCmd = static_cast<float>(ForwardPid ? ForwardPid->Update(TargetArea, AreaNorm, false) : (TargetArea - AreaNorm) * Params.MaxForwardSpeed);
    ForwardCmd = FMath::Max(0.0f, ForwardCmd);
    const float BaseCruise = bRamPhase
        ? FMath::Lerp(FMath::Max(Params.MinRamSpeed, Params.MaxForwardSpeed * 0.80f), Params.MaxForwardSpeed * 0.98f, CloseFactor)
        : FMath::Clamp(3.4f + Params.MaxForwardSpeed * 0.38f, 3.2f, Params.MaxForwardSpeed);
    const float CenterNorm = FMath::Clamp(
        FMath::Max(
            FMath::Abs(Ex) / FMath::Max(Params.CenterTolX, 0.01f),
            FMath::Abs(Ey) / FMath::Max(Params.CenterTolY, 0.01f)),
        0.0f,
        4.0f);
    const float AimScale = bRamPhase
        ? FMath::Clamp(1.14f - 0.02f * CenterNorm, 1.04f, 1.14f)
        : FMath::Clamp(1.08f - 0.03f * CenterNorm, 0.96f, 1.08f);
    const float ChargeBoost = bRamPhase ? FMath::Lerp(1.35f, 4.2f, CloseFactor) : 0.82f;
    ForwardCmd = FMath::Max(ForwardCmd + ChargeBoost, BaseCruise) * AimScale;
    ForwardCmd = FMath::Clamp(ForwardCmd, -Params.MaxReverseSpeed, Params.MaxForwardSpeed);
    const float CurrentYawDeg = Interceptor->GetActorRotation().Yaw;
    const float YawLeadTime = bRamPhase ? FMath::Lerp(0.12f, 0.08f, CloseFactor) : 0.18f;
    const float YawLeadDeltaDeg = FMath::Clamp(YawRateCmd * FMath::Max(YawLeadTime, Dt), -48.0f, 48.0f);
    const float DesiredYawDeg = NormalizeYawDeg(CurrentYawDeg + YawLeadDeltaDeg);
    const FRotator DesiredBodyYawOnly(0.0f, DesiredYawDeg, 0.0f);
    const FVector DesiredForwardDir = DesiredBodyYawOnly.Vector();
    const FVector DesiredRightDir = FRotationMatrix(DesiredBodyYawOnly).GetUnitAxis(EAxis::Y);
    const float ForwardSlowdownStart = bRamPhase ? FMath::Max(Params.CenterTolX * 1.25f, 0.05f) : FMath::Max(Params.CenterTolX * 1.10f, 0.04f);
    const float ForwardSlowdownFull = bRamPhase ? 0.55f : 0.50f;
    const float MinForwardAlignScale = bRamPhase ? 0.18f : 0.28f;
    const float ForwardAlignScale = FMath::GetMappedRangeValueClamped(
        FVector2D(ForwardSlowdownStart, ForwardSlowdownFull),
        FVector2D(1.0f, MinForwardAlignScale),
        FMath::Abs(Ex));
    const float TurnDemandNorm = FMath::Clamp(FMath::Abs(YawRateCmd) / FMath::Max(Params.MaxYawRateDeg, 1.0f), 0.0f, 1.0f);
    const float TurnAlignScale = FMath::GetMappedRangeValueClamped(
        FVector2D(bRamPhase ? 0.28f : 0.36f, 1.0f),
        FVector2D(1.0f, bRamPhase ? 0.65f : 0.78f),
        TurnDemandNorm);
    float DesiredForwardSpeed = FMath::Clamp(ForwardCmd * ForwardAlignScale * TurnAlignScale, 0.0f, Params.MaxForwardSpeed);
    const float MinPursuitSpeed = bRamPhase
        ? FMath::Clamp(Params.MaxForwardSpeed * 0.18f, Params.MinRamSpeed, Params.MaxForwardSpeed)
        : FMath::Clamp(Params.MaxForwardSpeed * 0.26f, 3.0f, Params.MaxForwardSpeed);
    const float PursuitFloorScale = FMath::GetMappedRangeValueClamped(
        FVector2D(bRamPhase ? 0.18f : 0.16f, bRamPhase ? 0.72f : 0.68f),
        FVector2D(1.0f, bRamPhase ? 0.58f : 0.66f),
        FMath::Abs(Ex));
    DesiredForwardSpeed = FMath::Max(DesiredForwardSpeed, MinPursuitSpeed * PursuitFloorScale);
    if (!bDetectionReal)
    {
        DesiredForwardSpeed *= 0.96f;
        VerticalCmd *= 0.92f;
    }
    const bool bNearlyCentered = FMath::Abs(Ex) <= FMath::Max(Params.CenterTolX * 3.0f, 0.10f)
        && FMath::Abs(Ey - DesiredEy) <= FMath::Max(Params.CenterTolY * 2.4f, 0.10f);
    if (bRamPhase && bNearlyCentered)
    {
        const float MinCloseForward = FMath::Lerp(Params.MinRamSpeed * 0.78f, Params.MinRamSpeed, CloseFactor);
        DesiredForwardSpeed = FMath::Max(DesiredForwardSpeed, MinCloseForward);
    }
    if (bRamPhase && TargetDistance > 0.0f && TargetDistance <= FMath::Max(Params.InterceptDistance * 1.8f, 2.6f) && bNearlyCentered)
    {
        DesiredForwardSpeed = FMath::Max(DesiredForwardSpeed, FMath::Min(Params.MaxForwardSpeed, Params.MinRamSpeed + 1.8f));
    }
    const float CurrentLateralSpeed = FVector::DotProduct(CurrentPlanarVelocity, DesiredRightDir);
    const float LateralDampingGain = bRamPhase ? FMath::Lerp(1.15f, 1.55f, CloseFactor) : 1.00f;
    const float MaxLateralCorrection = FMath::Max(Params.MaxForwardSpeed * (bRamPhase ? 0.88f : 0.62f), 2.0f);
    const float DesiredLateralSpeed = FMath::Clamp(-CurrentLateralSpeed * LateralDampingGain, -MaxLateralCorrection, MaxLateralCorrection);
    FVector PlanarCmdVel = DesiredForwardDir * DesiredForwardSpeed + DesiredRightDir * DesiredLateralSpeed;
    const float MaxPlanarCmdSpeed = FMath::Max(Params.MaxForwardSpeed, Params.MinRamSpeed);
    if (PlanarCmdVel.SizeSquared() > FMath::Square(MaxPlanarCmdSpeed))
    {
        PlanarCmdVel = PlanarCmdVel.GetClampedToMaxSize(MaxPlanarCmdSpeed);
    }
    FVector CmdVel = PlanarCmdVel;
    CmdVel.Z = VerticalCmd;
    if (!IsFiniteFeature(CmdVel))
    {
        return false;
    }

    SetDroneHeadingControl(Interceptor, EDroneYawMode::Angle, EDroneDrivetrainMode::ForwardOnly, DesiredYawDeg);
    SetDroneTargetVelocity(Interceptor, CmdVel);

    Runtime.LastEx = Ex;
    Runtime.LastEy = Ey;
    Runtime.LastAreaNorm = AreaNorm;
    Runtime.LastConf = Confidence;
    Runtime.LastYawRateDeg = YawRateCmd;
    Runtime.LastYawCmdDeg = DesiredYawDeg;
    Runtime.LastCmdVel = CmdVel;

    const bool bCaptureByDistance = TargetDistance > 0.0f && TargetDistance <= Params.InterceptDistance;
    const bool bCaptureByImage = AreaNorm >= Params.CaptureArea && bCentered;
    if (bCaptureByDistance || bCaptureByImage)
    {
        ++Runtime.CaptureCount;
    }
    else
    {
        Runtime.CaptureCount = 0;
    }

    bOutCaptured = bCaptureByDistance || Runtime.CaptureCount >= Params.CaptureHoldFrames;
    Runtime.bCaptured = bOutCaptured;
    if (bOutCaptured)
    {
        Runtime.State = EVisualState::Captured;
        if (Params.bStopOnCapture)
        {
            HoverDrone(Interceptor);
        }
    }

    return true;
}

void UVisualInterceptController::ComputeSearchControl(AActor* Interceptor, float Dt)
{
    if (!Interceptor)
    {
        return;
    }

    Runtime.State = EVisualState::Search;
    Runtime.CaptureCount = 0;
    Runtime.LastEx = 0.0f;
    Runtime.LastEy = 0.0f;
    Runtime.LastAreaNorm = 0.0f;
    Runtime.LastConf = 0.0f;
    Runtime.LastPredictionLeadTime = 0.0f;
    Runtime.LastProcessingLatency = 0.0f;

    Runtime.SearchTime += Dt;
    if (Runtime.SearchTime >= 2.5f)
    {
        Runtime.SearchTime = 0.0f;
        Runtime.SearchDir *= -1.0f;
    }

    const bool bMoveCamera = Params.SearchCamYawLimitDeg > KINDA_SMALL_NUMBER && Params.SearchCamRateDeg > KINDA_SMALL_NUMBER;
    if (bMoveCamera)
    {
        Runtime.SearchCamYawDeg += Runtime.SearchDir * Params.SearchCamRateDeg * Dt;
        if (Runtime.SearchCamYawDeg >= Params.SearchCamYawLimitDeg)
        {
            Runtime.SearchCamYawDeg = Params.SearchCamYawLimitDeg;
            Runtime.SearchDir = -1.0f;
        }
        else if (Runtime.SearchCamYawDeg <= -Params.SearchCamYawLimitDeg)
        {
            Runtime.SearchCamYawDeg = -Params.SearchCamYawLimitDeg;
            Runtime.SearchDir = 1.0f;
        }

        SetDroneCameraAngles(Interceptor, Params.SearchCamPitchDeg, Runtime.SearchCamYawDeg);
    }
    else
    {
        Runtime.SearchCamYawDeg = 0.0f;
        SetDroneCameraAngles(Interceptor, 0.0f, 0.0f);
    }

    const float BodyYawRate = Params.SearchBodyYawRateDeg > KINDA_SMALL_NUMBER ? Runtime.SearchDir * Params.SearchBodyYawRateDeg : 0.0f;
    const EDroneYawMode YawMode = FMath::Abs(BodyYawRate) > KINDA_SMALL_NUMBER ? EDroneYawMode::Rate : EDroneYawMode::Hold;
    SetDroneHeadingControl(Interceptor, YawMode, EDroneDrivetrainMode::MaxDegreeOfFreedom, BodyYawRate);

    const float VerticalCmd = (bMoveCamera && Params.SearchVzAmp > KINDA_SMALL_NUMBER)
        ? Params.SearchVzAmp * FMath::Sin(Runtime.SearchTime * 1.5f)
        : 0.0f;

    SetDroneTargetVelocity(Interceptor, FVector(0.0f, 0.0f, VerticalCmd));

    Runtime.LastYawRateDeg = BodyYawRate;
    Runtime.LastYawCmdDeg = NormalizeYawDeg(Interceptor->GetActorRotation().Yaw + BodyYawRate * Dt);
    Runtime.LastCmdVel = FVector(0.0f, 0.0f, VerticalCmd);
}
FString UVisualInterceptController::HandleStart(const TSharedPtr<FJsonObject>& CmdObj, UWorld* World)
{
    (void)World;
    EnsureInitialized();

    AActor* Interceptor = ResolveInterceptor(CmdObj);
    if (!Interceptor)
    {
        return MakeError(TEXT("interceptor drone not found"));
    }

    AActor* Target = ResolveTarget(CmdObj);
    if (!Target)
    {
        return MakeError(TEXT("target drone not found"));
    }

    if (Interceptor == Target)
    {
        return MakeError(TEXT("interceptor and target can not be the same actor"));
    }

    BeginSession(Interceptor);
    ApplyParamsFromJson(CmdObj);

    Runtime.InterceptorId = GetAgentId(Interceptor);
    Runtime.TargetId = GetAgentId(Target);
    Runtime.LastTargetDistance = FVector::Dist(GetAgentPosition(Interceptor), GetAgentPosition(Target));

    if (Params.SearchCamYawLimitDeg > KINDA_SMALL_NUMBER && Params.SearchCamRateDeg > KINDA_SMALL_NUMBER)
    {
        SetDroneCameraAngles(Interceptor, Params.SearchCamPitchDeg, 0.0f);
    }
    else
    {
        SetDroneCameraAngles(Interceptor, 0.0f, 0.0f);
    }

    return BuildStartJson();
}

FString UVisualInterceptController::HandleUpdate(const TSharedPtr<FJsonObject>& CmdObj, UWorld* World)
{
    (void)World;
    EnsureInitialized();

    if (!Runtime.bEnabled)
    {
        return MakeError(TEXT("visual intercept is not started"));
    }

    AActor* Interceptor = ResolveInterceptor(CmdObj);
    if (!Interceptor)
    {
        return MakeError(TEXT("interceptor drone not found"));
    }

    AActor* Target = ResolveTarget(CmdObj);
    if (!Target)
    {
        return MakeError(TEXT("target drone not found"));
    }

    if (Interceptor == Target)
    {
        return MakeError(TEXT("interceptor and target can not be the same actor"));
    }

    float Dt = kDefaultDt;
    TryGetNumberAs(CmdObj, TEXT("dt"), Dt);
    Dt = SafeDt(Dt);
    SyncPidTimeStep(Dt);

    float ProcessingLatency = 0.0f;
    TryGetNumberAs(CmdObj, TEXT("processing_latency"), ProcessingLatency);
    ProcessingLatency = FMath::Clamp(ProcessingLatency, 0.0f, 0.35f);
    Runtime.LastProcessingLatency = ProcessingLatency;

    float FrameW = Runtime.LastFrameW;
    float FrameH = Runtime.LastFrameH;
    TryGetNumberAs(CmdObj, TEXT("image_w"), FrameW);
    TryGetNumberAs(CmdObj, TEXT("image_h"), FrameH);
    FrameW = FMath::Max(FrameW, 2.0f);
    FrameH = FMath::Max(FrameH, 2.0f);
    Runtime.LastFrameW = FrameW;
    Runtime.LastFrameH = FrameH;

    const EVisualState PreviousState = Runtime.State;
    const float RealTargetDistance = FVector::Dist(GetAgentPosition(Interceptor), GetAgentPosition(Target));

    Runtime.FrameCount++;
    Runtime.InterceptorId = GetAgentId(Interceptor);
    Runtime.TargetId = GetAgentId(Target);
    Runtime.LastTargetDistance = RealTargetDistance;
    Runtime.bLastHasMeasurement = false;
    Runtime.LastMeasurementFeature = FVector::ZeroVector;
    Runtime.bLastHasTrackFeature = false;
    Runtime.bLastUsedPrediction = false;
    Runtime.LastTrackFeature = FVector::ZeroVector;
    Runtime.bLastHasPrediction = false;
    Runtime.LastPredictionFeature = FVector::ZeroVector;
    Runtime.LastPredictionLeadTime = 0.0f;

    auto UpdateKalmanDebug = [this, FrameW, FrameH]()
    {
        Runtime.bLastKalmanValid = Params.bUseKalman && FeaturePredictor && FeaturePredictor->IsInitialized();
        if (!Runtime.bLastKalmanValid)
        {
            Runtime.LastKalmanPosition = FVector::ZeroVector;
            Runtime.LastKalmanVelocity = FVector::ZeroVector;
            Runtime.LastKalmanAcceleration = FVector::ZeroVector;
            Runtime.LastKalmanUncertainty = 0.0f;
            Runtime.LastKalmanProcessNoise = 0.0f;
            return;
        }

        const FVector KalmanPos = FeaturePredictor->GetEstimatedPosition();
        const FVector KalmanVel = FeaturePredictor->GetEstimatedVelocity();
        const FVector KalmanAcc = FeaturePredictor->GetEstimatedAcceleration();
        Runtime.LastKalmanPosition = NormalizedToPixels(KalmanPos, FrameW, FrameH);
        Runtime.LastKalmanVelocity = FVector(KalmanVel.X * FrameW, KalmanVel.Y * FrameH, KalmanVel.Z);
        Runtime.LastKalmanAcceleration = FVector(KalmanAcc.X * FrameW, KalmanAcc.Y * FrameH, KalmanAcc.Z);
        Runtime.LastKalmanUncertainty = FeaturePredictor->GetPositionUncertainty();
        Runtime.LastKalmanProcessNoise = FeaturePredictor->GetAdaptiveProcessNoise();
    };

    auto ComputeLeadTime = [this, RealTargetDistance, ProcessingLatency, Dt](float AreaNormValue)
    {
        if (!Params.bUseKalman)
        {
            return 0.0f;
        }

        const bool bApproachPhase = AreaNormValue >= Params.RamAreaTarget * 0.54f || RealTargetDistance <= FMath::Max(Params.InterceptDistance * 5.0f, 6.4f);
        const float SpeedRef = FMath::Max(Params.MaxForwardSpeed, 1.0f);
        // 娑擃厽鏋冨▔銊╁櫞閿涙岸顣╁ù瀣涧鐞涖儱浼╅張澶愭閻ㄥ嫰鈧矮锟?婢跺嫮鎮婂鎯扮箿閿涘奔绗夐崘宥囩舶婢额亪鏆遍惃鍕鐟欏棙妞傞梻娣偓?
        const float EffectiveLatency = FMath::Clamp(ProcessingLatency + 0.25f * Dt, 0.0f, 0.16f);
        if (bApproachPhase)
        {
            const float DistLead = RealTargetDistance > 0.0f ? RealTargetDistance / SpeedRef * 0.14f + 0.02f : Params.RamLeadTime;
            return FMath::Clamp(FMath::Min(Params.RamLeadTime, DistLead + EffectiveLatency), 0.02f, 0.22f);
        }

        const float DistLead = RealTargetDistance > 0.0f ? RealTargetDistance / SpeedRef * 0.04f + 0.01f : Params.TrackLeadTime;
        return FMath::Clamp(FMath::Min(Params.TrackLeadTime, DistLead + EffectiveLatency), 0.01f, 0.16f);
    };

    bool bHasDetection = false;
    TryGetBool(CmdObj, TEXT("has_detection"), bHasDetection);

    if (bHasDetection)
    {
        float Cx = 0.0f;
        float Cy = 0.0f;
        float RawArea = 0.0f;
        float AreaRatio = 0.0f;
        float Conf = 0.0f;
        TryGetNumberAs(CmdObj, TEXT("cx"), Cx);
        TryGetNumberAs(CmdObj, TEXT("cy"), Cy);
        TryGetNumberAs(CmdObj, TEXT("area"), RawArea);
        TryGetNumberAs(CmdObj, TEXT("area_ratio"), AreaRatio);
        TryGetNumberAs(CmdObj, TEXT("conf"), Conf);

        float AreaNorm = AreaRatio;
        if (AreaNorm <= KINDA_SMALL_NUMBER && RawArea > 0.0f)
        {
            AreaNorm = RawArea / FMath::Max(FrameW * FrameH, 1.0f);
        }

        FVector Measurement = ClampFeatureToFrame(FVector(Cx, Cy, AreaNorm), FrameW, FrameH);
        Runtime.bLastHasMeasurement = true;
        Runtime.LastMeasurementFeature = Measurement;
        Runtime.DetectionCount++;

        if (PreviousState == EVisualState::Search || Runtime.LostCount >= Params.LostToSearchFrames)
        {
            ResetPidControllers();
            if (FeaturePredictor)
            {
                FeaturePredictor->Reset();
            }
        }

        Runtime.LostCount = 0;

        const float LeadTime = ComputeLeadTime(Measurement.Z);

        Runtime.LastPredictionLeadTime = LeadTime;

        if (Params.bUseKalman && FeaturePredictor)
        {
            FeaturePredictor->Update(PixelsToNormalized(Measurement, FrameW, FrameH), Dt);
        }

        FVector TrackFeature = Measurement;
        bool bUsingPrediction = false;

        if (Params.bUseKalman && FeaturePredictor && FeaturePredictor->IsInitialized())
        {
            const FVector PredNorm = FeaturePredictor->PredictPosition(LeadTime);
            FVector Prediction = NormalizedToPixels(PredNorm, FrameW, FrameH);
            if (IsFiniteFeature(Prediction))
            {
                Prediction = ClampFeatureToFrame(Prediction, FrameW, FrameH);
                Runtime.bLastHasPrediction = true;
                Runtime.LastPredictionFeature = Prediction;
                const bool bCloseTrack = Measurement.Z >= Params.RamAreaTarget * 0.60f || (RealTargetDistance > 0.0f && RealTargetDistance <= FMath::Max(Params.InterceptDistance * 3.5f, 4.0f));
                // 娑擃厽鏋冨▔銊╁櫞閿涙碍顥呭ù瀣╃矝閻掓湹缍旀稉杞板瘜鏉堟挸鍙嗛敍瀛燼lman 閸欘亜浠涚亸蹇撶畽閸撳秷顫嬫穱顔筋劀閿涘矂浼╅崗宥夘暕濞村顢嬮幎濠冨付閸掕泛鐣崗銊ョ敨閸嬪繈锟?
                const float OffsetPx = FVector2D(Prediction.X - Measurement.X, Prediction.Y - Measurement.Y).Size();
                const float OffsetNorm = OffsetPx / FMath::Max(FMath::Min(FrameW, FrameH), 1.0f);
                const float BaseBlend = bCloseTrack ? 0.05f : 0.12f;
                const float OffsetGate = bCloseTrack ? 1.0f - FMath::Clamp((OffsetNorm - 0.010f) / 0.14f, 0.0f, 1.0f) : 1.0f - FMath::Clamp((OffsetNorm - 0.015f) / 0.10f, 0.0f, 1.0f);
                const float LeadGate = bCloseTrack ? 1.0f - 0.30f * FMath::Clamp((LeadTime - 0.04f) / 0.08f, 0.0f, 1.0f) : 1.0f - 0.65f * FMath::Clamp((LeadTime - 0.05f) / 0.10f, 0.0f, 1.0f);
                const float PredictionBlend = BaseBlend * OffsetGate * LeadGate;
                if (PredictionBlend >= (bCloseTrack ? 0.02f : 0.03f))
                {
                    TrackFeature.X = FMath::Lerp(Measurement.X, Prediction.X, PredictionBlend);
                    TrackFeature.Y = FMath::Lerp(Measurement.Y, Prediction.Y, PredictionBlend);
                    TrackFeature.Z = Measurement.Z;
                    bUsingPrediction = true;
                }
            }
        }

        Runtime.bLastHasTrackFeature = true;
        Runtime.bLastUsedPrediction = bUsingPrediction;
        Runtime.LastTrackFeature = TrackFeature;

        bool bCapturedNow = false;
        const bool bValid = ComputeTrackControl(Interceptor, TrackFeature, FrameW, FrameH, Conf, Dt, true, RealTargetDistance, bCapturedNow);
        UpdateKalmanDebug();
        return BuildUpdateJson(bValid);
    }

    Runtime.LostCount++;
    Runtime.CaptureCount = 0;
    Runtime.LastConf = 0.0f;
    Runtime.LastPredictionLeadTime = ComputeLeadTime(Runtime.LastAreaNorm);

    if (Params.bUseKalman && FeaturePredictor && FeaturePredictor->IsInitialized() && Runtime.LostCount <= Params.LostToSearchFrames)
    {
        FVector Prediction = NormalizedToPixels(FeaturePredictor->PredictPosition(Runtime.LastPredictionLeadTime), FrameW, FrameH);
        if (IsFiniteFeature(Prediction))
        {
            Prediction = ClampFeatureToFrame(Prediction, FrameW, FrameH);
            Runtime.bLastHasPrediction = true;
            Runtime.LastPredictionFeature = Prediction;
            Runtime.bLastHasTrackFeature = true;
            Runtime.bLastUsedPrediction = true;
            Runtime.LastTrackFeature = Prediction;

            bool bCapturedNow = false;
            const bool bValid = ComputeTrackControl(Interceptor, Prediction, FrameW, FrameH, 0.0f, Dt, false, RealTargetDistance, bCapturedNow);
            UpdateKalmanDebug();
            return BuildUpdateJson(bValid);
        }
    }

    ResetPidControllers();
    ComputeSearchControl(Interceptor, Dt);
    UpdateKalmanDebug();
    return BuildUpdateJson(true);
}

FString UVisualInterceptController::HandleStop(const TSharedPtr<FJsonObject>& CmdObj, UWorld* World)
{
    (void)World;
    EnsureInitialized();

    if (AActor* Interceptor = ResolveInterceptor(CmdObj))
    {
        HoverDrone(Interceptor);
        SetDroneCameraAngles(Interceptor, 0.0f, 0.0f);
    }

    ResetPidControllers();

    Runtime.bEnabled = false;
    Runtime.State = Runtime.bCaptured ? EVisualState::Captured : EVisualState::Idle;
    Runtime.LastCmdVel = FVector::ZeroVector;
    Runtime.LastYawRateDeg = 0.0f;
    Runtime.LastPredictionLeadTime = 0.0f;
    Runtime.LastProcessingLatency = 0.0f;

    return MakeOk(TEXT("visual intercept stopped"));
}

FString UVisualInterceptController::HandleState() const
{
    return BuildUpdateJson(Runtime.bEnabled);
}









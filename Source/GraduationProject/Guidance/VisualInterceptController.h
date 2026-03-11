#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "VisualInterceptController.generated.h"

class FJsonObject;
class ADronePawn;
class UKalmanPredictor;
class UPIDController;

UCLASS()
class GRADUATIONPROJECT_API UVisualInterceptController : public UObject
{
    GENERATED_BODY()

public:
    void EnsureInitialized();
    void Reset();

    FString HandleStart(const TSharedPtr<FJsonObject>& CmdObj, UWorld* World);
    FString HandleUpdate(const TSharedPtr<FJsonObject>& CmdObj, UWorld* World);
    FString HandleStop(const TSharedPtr<FJsonObject>& CmdObj, UWorld* World);
    FString HandleState() const;

private:
    enum class EVisualState : uint8
    {
        Idle,
        Search,
        Track,
        Approach,
        Captured,
    };

    struct FVisualParams
    {
        float DesiredArea = 0.05f;
        float CaptureArea = 0.09f;
        float CenterTolX = 0.08f;
        float CenterTolY = 0.10f;
        int32 CaptureHoldFrames = 12;
        int32 LostToSearchFrames = 8;

        float MaxForwardSpeed = 7.5f;
        float MaxReverseSpeed = 1.5f;
        float MaxVerticalSpeed = 2.5f;
        float MaxYawRateDeg = 60.0f;

        float SearchCamYawLimitDeg = 165.0f;
        float SearchCamRateDeg = 70.0f;
        float SearchBodyYawRateDeg = 22.0f;
        float SearchCamPitchDeg = -5.0f;
        float SearchVzAmp = 0.4f;

        bool bStopOnCapture = true;
        bool bUseKalman = true;
    };

    struct FVisualRuntime
    {
        bool bEnabled = false;
        bool bCaptured = false;

        FString InterceptorId;
        FString TargetId;
        FString Method = TEXT("vision_pid_kalman");

        EVisualState State = EVisualState::Idle;

        int32 FrameCount = 0;
        int32 DetectionCount = 0;
        int32 LostCount = 0;
        int32 CaptureCount = 0;

        float LastEx = 0.0f;
        float LastEy = 0.0f;
        float LastAreaNorm = 0.0f;
        float LastConf = 0.0f;
        float LastYawCmdDeg = 0.0f;
        float LastYawRateDeg = 0.0f;
        FVector LastCmdVel = FVector::ZeroVector;

        float SearchCamYawDeg = 0.0f;
        float SearchDir = 1.0f;
        float SearchTime = 0.0f;

        float LastFrameW = 640.0f;
        float LastFrameH = 480.0f;
    };

    void ApplyParamsFromJson(const TSharedPtr<FJsonObject>& CmdObj);
    void ResetRuntimeOnly();

    ADronePawn* ResolveInterceptor(UWorld* World, const TSharedPtr<FJsonObject>& CmdObj);
    ADronePawn* ResolveTarget(UWorld* World, const TSharedPtr<FJsonObject>& CmdObj);

    float SafeDt(float Dt) const;
    void SyncPidTimeStep(float Dt);

    FString BuildUpdateJson(bool bValidControl) const;
    FString BuildStartJson() const;

    FString MakeError(const FString& Msg) const;
    FString MakeOk(const FString& Msg) const;

    static FString BoolLiteral(bool bValue);
    static float NormalizeYawDeg(float YawDeg);
    static FString StateToString(EVisualState State);
    static bool NormalizeMethod(const FString& InMethod, FString& OutCanonical);

    bool ComputeTrackControl(
        ADronePawn* Interceptor,
        const FVector& Feature,
        float FrameW,
        float FrameH,
        float Confidence,
        float Dt,
        bool bDetectionReal,
        bool& bOutCaptured);

    void ComputeSearchControl(ADronePawn* Interceptor, float Dt);

private:
    UPROPERTY()
    UKalmanPredictor* FeaturePredictor = nullptr;

    UPROPERTY()
    UPIDController* YawPid = nullptr;

    UPROPERTY()
    UPIDController* VerticalPid = nullptr;

    UPROPERTY()
    UPIDController* ForwardPid = nullptr;

    FVisualParams Params;
    FVisualRuntime Runtime;
};


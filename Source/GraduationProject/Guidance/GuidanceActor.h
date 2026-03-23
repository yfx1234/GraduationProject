#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "GuidanceActor.generated.h"

class UVisualInterceptController;

UCLASS()
class GRADUATIONPROJECT_API AGuidanceActor : public AActor
{
    GENERATED_BODY()

public:
    AGuidanceActor();

    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

    UFUNCTION(BlueprintCallable, Category = "Guidance")
    FString ResetGuidance();

    UFUNCTION(BlueprintCallable, Category = "Guidance")
    FString GetState();

    UFUNCTION(BlueprintCallable, Category = "Guidance|Visual")
    FString VisualInterceptStart(
        FString InterceptorId = TEXT(""),
        FString TargetId = TEXT(""),
        FString Method = TEXT("vision_pid_kalman"),
        float DesiredArea = -1.0f,
        float CaptureArea = -1.0f,
        float CenterTolX = -1.0f,
        float CenterTolY = -1.0f,
        int32 CaptureHoldFrames = -1,
        int32 LostToSearchFrames = -1,
        float MaxForwardSpeed = -1.0f,
        float MaxReverseSpeed = -1.0f,
        float MaxVerticalSpeed = -1.0f,
        float MaxYawRateDeg = -1.0f,
        float RamAreaTarget = -1.0f,
        float MinRamSpeed = -1.0f,
        float InterceptDistance = -1.0f,
        float TrackLeadTime = -1.0f,
        float RamLeadTime = -1.0f,
        float SearchCamYawLimitDeg = -1.0f,
        float SearchCamRateDeg = -1.0f,
        float SearchBodyYawRateDeg = -1.0f,
        float SearchCamPitchDeg = -1000.0f,
        float SearchVzAmp = -1.0f,
        int32 StopOnCaptureFlag = -1,
        int32 UseKalmanFlag = -1);

    UFUNCTION(BlueprintCallable, Category = "Guidance|Visual")
    FString VisualInterceptUpdate(
        int32 HasDetection = 0,
        float Cx = 0.0f,
        float Cy = 0.0f,
        float Area = 0.0f,
        float AreaRatio = -1.0f,
        float Conf = 1.0f,
        float Dt = 0.08f,
        float ProcessingLatency = 0.0f,
        float ImageW = 640.0f,
        float ImageH = 480.0f,
        FString InterceptorId = TEXT(""),
        FString TargetId = TEXT(""));

    UFUNCTION(BlueprintCallable, Category = "Guidance|Visual")
    FString VisualInterceptStop(FString InterceptorId = TEXT(""), FString TargetId = TEXT(""));

    UFUNCTION(BlueprintCallable, Category = "Guidance|Visual")
    FString VisualInterceptState();

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Guidance")
    FString GuidanceId = TEXT("guidance_0");

private:
    void EnsureInitialized();
    FString MakeError(const FString& Msg) const;
    FString MakeOk(const FString& Msg = TEXT("ok")) const;

    UPROPERTY()
    UVisualInterceptController* VisualInterceptController = nullptr;
};

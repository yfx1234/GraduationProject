#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "../Drone/DronePawn.h"
#include "GuidanceCommandHandler.generated.h"

class UKalmanPredictor;
class IGuidanceMethod;
class UAgentManager;
class ADronePawn;
class UVisualInterceptController;

/**
 * Guidance TCP command handler.
 * Keeps existing turret guidance API and extends drone interception loop.
 */
UCLASS()
class GRADUATIONPROJECT_API UGuidanceCommandHandler : public UObject
{
    GENERATED_BODY()

public:
    UGuidanceCommandHandler();
    ~UGuidanceCommandHandler();

    FString HandleCallGuidance(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World);
    FString HandleGetGuidanceState(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World);

private:
    void EnsureInitialized();
    ADronePawn* FindDroneByRole(UAgentManager* Manager, EDroneMissionRole Role, const FString& ExcludeId = TEXT("")) const;

    UPROPERTY()
    UKalmanPredictor* Predictor = nullptr;

    UPROPERTY()
    UVisualInterceptController* VisualInterceptController = nullptr;

    IGuidanceMethod* CurrentMethod = nullptr;
    FString CurrentMethodName;

    float LastPitch = 0.0f;
    float LastYaw = 0.0f;
    FVector LastAimPoint = FVector::ZeroVector;
    float LastFlightTime = 0.0f;
    float DefaultVisionLatency = 0.08f;
    float LastLatencyCompensation = 0.08f;


    // Drone interception config/state.
    FString CurrentInterceptMethod = TEXT("pure_pursuit");
    float InterceptorSpeed = 8.0f;
    float InterceptNavGain = 3.0f;
    float InterceptLeadTime = 0.6f;
    float CaptureRadius = 1.5f;
    FString LastInterceptorId;
    FString LastTargetId;
    FVector LastInterceptorCmdVel = FVector::ZeroVector;
    float LastDistanceToTarget = 0.0f;
    float LastClosingSpeed = 0.0f;
    bool bLastInterceptValid = false;
    bool bLastCaptured = false;

    FString MakeError(const FString& Msg);
    FString MakeOk(const FString& Msg = TEXT("ok"));
};

#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "SensorManager.generated.h"

USTRUCT()
struct FSensorKinematicCache
{
    GENERATED_BODY()

    FVector LastVelocity = FVector::ZeroVector;
    double LastTimeSec = 0.0;
    bool bValid = false;
};

UCLASS()
class GRADUATIONPROJECT_API USensorManager : public UObject
{
    GENERATED_BODY()

public:
    static USensorManager* GetInstance();
    static void Cleanup();

    FString BuildDroneSensorJson(const FString& DroneId, UWorld* World, const FString& Frame = TEXT("ue"));

private:
    static USensorManager* Instance;

    UPROPERTY()
    TMap<FString, FSensorKinematicCache> DroneCache;
};

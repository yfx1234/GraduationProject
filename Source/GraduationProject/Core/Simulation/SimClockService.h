#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "SimClockService.generated.h"

UCLASS()
class GRADUATIONPROJECT_API USimClockService : public UObject
{
    GENERATED_BODY()

public:
    static USimClockService* GetInstance();
    static void Cleanup();

    void Initialize(UWorld* World);
    bool IsInitialized() const { return bInitialized; }
    void SetTimeScale(float InTimeScale, UWorld* World);

    float GetTimeScale() const { return TimeScale; }
    double GetSimTimeSec(UWorld* World) const;
    double GetWallTimeSec() const;

private:
    static USimClockService* Instance;

    float TimeScale = 1.0f;
    double StartWallTimeSec = 0.0;
    double StartWorldTimeSec = 0.0;
    bool bInitialized = false;
};



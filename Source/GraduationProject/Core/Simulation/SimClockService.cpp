#include "SimClockService.h"

#include "Engine/World.h"
#include "GameFramework/WorldSettings.h"

USimClockService* USimClockService::Instance = nullptr;

USimClockService* USimClockService::GetInstance()
{
    if (!Instance)
    {
        Instance = NewObject<USimClockService>();
        Instance->AddToRoot();
    }
    return Instance;
}

void USimClockService::Cleanup()
{
    if (Instance)
    {
        Instance->RemoveFromRoot();
        Instance = nullptr;
    }
}

void USimClockService::Initialize(UWorld* World)
{
    StartWallTimeSec = FPlatformTime::Seconds();
    StartWorldTimeSec = World ? World->GetTimeSeconds() : 0.0;
    bInitialized = true;
}

void USimClockService::SetTimeScale(float InTimeScale, UWorld* World)
{
    TimeScale = FMath::Clamp(InTimeScale, 0.05f, 20.0f);
    if (World && World->GetWorldSettings())
    {
        World->GetWorldSettings()->SetTimeDilation(TimeScale);
    }
}

double USimClockService::GetSimTimeSec(UWorld* World) const
{
    if (!World)
    {
        return 0.0;
    }
    return World->GetTimeSeconds() - StartWorldTimeSec;
}

double USimClockService::GetWallTimeSec() const
{
    return FPlatformTime::Seconds() - StartWallTimeSec;
}





#include "SimGameMode.h"
#include "CameraPawn.h"
#include "Manager/AgentManager.h"
#include "Kismet/GameplayStatics.h"

/** @brief 设置默认 Pawn 为 ACameraPawn */
ASimGameMode::ASimGameMode()
{
    DefaultPawnClass = ACameraPawn::StaticClass();
}

/** @brief 根据配置自动生成无人机 */
void ASimGameMode::BeginPlay()
{
    Super::BeginPlay();
    UE_LOG(LogTemp, Log, TEXT("[SimGameMode] BeginPlay"));
    if (bAutoSpawnDrone && DefaultDroneClass) SpawnDefaultDrone();
    else if (bAutoSpawnDrone && !DefaultDroneClass) UE_LOG(LogTemp, Warning, TEXT("[SimGameMode] bAutoSpawnDrone=true but DefaultDroneClass is not set! Please set it in BP_SimGameMode."));
}

/** @brief 生成默认无人机 */
void ASimGameMode::SpawnDefaultDrone()
{
    UWorld* World = GetWorld();
    if (!World) return;
    FVector SpawnLocationUE = DroneSpawnLocation * 100.0f;
    FRotator SpawnRotation = FRotator::ZeroRotator;
    FActorSpawnParameters SpawnParams;
    SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    SpawnedDrone = World->SpawnActor<APawn>(DefaultDroneClass, SpawnLocationUE, SpawnRotation, SpawnParams);
    if (SpawnedDrone)
    {
        UAgentManager::GetInstance()->RegisterAgent(DefaultDroneId, SpawnedDrone);
        UE_LOG(LogTemp, Log, TEXT("[SimGameMode] Spawned drone '%s' at (%s) m"), *DefaultDroneId, *DroneSpawnLocation.ToString());
        if (bAutoTakeoff)
        {
            FTimerHandle TakeoffTimer;
            World->GetTimerManager().SetTimer(TakeoffTimer, this, &ASimGameMode::DelayedTakeoff, 1.0f, false);
        }
    }
    else UE_LOG(LogTemp, Error, TEXT("[SimGameMode] Failed to spawn drone!"));
}

/** @brief 延迟起飞回调 */
void ASimGameMode::DelayedTakeoff()
{
    if (SpawnedDrone)
    {
        UFunction* TakeoffFunc = SpawnedDrone->FindFunction(FName("Takeoff"));
        if (TakeoffFunc)
        {
            struct { float Altitude; } Params;
            Params.Altitude = TakeoffAltitude;
            SpawnedDrone->ProcessEvent(TakeoffFunc, &Params);
            UE_LOG(LogTemp, Log, TEXT("[SimGameMode] Auto takeoff to %.1f m"), TakeoffAltitude);
        }
        else
        {
            UE_LOG(LogTemp, Warning, TEXT("[SimGameMode] Drone does not have Takeoff function yet (Phase 2)"));
        }
    }
}

/** @brief 暂停仿真 */
void ASimGameMode::PauseSimulation()
{
    UGameplayStatics::SetGamePaused(GetWorld(), true);
    UE_LOG(LogTemp, Log, TEXT("[SimGameMode] Simulation PAUSED"));
}

/** @brief 恢复仿真 */
void ASimGameMode::ResumeSimulation()
{
    UGameplayStatics::SetGamePaused(GetWorld(), false);
    UE_LOG(LogTemp, Log, TEXT("[SimGameMode] Simulation RESUMED"));
}

/** @brief 重置仿真 */
void ASimGameMode::ResetSimulation()
{
    UE_LOG(LogTemp, Log, TEXT("[SimGameMode] Simulation RESET"));
}


#include "SimGameMode.h"
#include "CameraPawn.h"
#include "Manager/AgentManager.h"
#include "Manager/CommandExecutionManager.h"
#include "Kismet/GameplayStatics.h"
#include "EngineUtils.h"
#include "../Drone/DronePawn.h"
ASimGameMode::ASimGameMode()
{
    DefaultPawnClass = ACameraPawn::StaticClass();
}
void ASimGameMode::BeginPlay()
{
    Super::BeginPlay();
    UAgentManager::Cleanup();
    UCommandExecutionManager::Cleanup();
    UAgentManager::GetInstance();
    UCommandExecutionManager::GetInstance();
    UE_LOG(LogTemp, Log, TEXT("[SimGameMode] BeginPlay (AutoSpawn=%s, Bootstrap=%s)"),
        bAutoSpawnDrone ? TEXT("true") : TEXT("false"),
        bEnableBootstrapSpawn ? TEXT("true") : TEXT("false"));
    const bool bShouldBootstrapSpawn = bEnableBootstrapSpawn && bAutoSpawnDrone;
    if (bShouldBootstrapSpawn)
    {
        if (!DefaultDroneClass)
        {
            UE_LOG(LogTemp, Warning, TEXT("[SimGameMode] bootstrap spawn enabled but DefaultDroneClass is not set"));
        }
        else if (bUseDroneSpawnList && DroneSpawnList.Num() > 0)
        {
            SpawnConfiguredDrones();
        }
        else
        {
            SpawnDefaultDrone();
        }
    }
    else
    {
        UE_LOG(LogTemp, Log, TEXT("[SimGameMode] Bootstrap auto-spawn is disabled"));
    }
    GetWorldTimerManager().SetTimerForNextTick(this, &ASimGameMode::CaptureInitialAgentStates);
}
ADronePawn* ASimGameMode::SpawnDrone(const FString& DroneId, const FVector& SpawnLocationMeters, EDroneMissionRole InMissionRole)
{
    UWorld* World = GetWorld();
    if (!World) return nullptr;
    const TSubclassOf<ADronePawn> DroneClass = TSubclassOf<ADronePawn>(DefaultDroneClass);
    if (!DroneClass)
    {
        UE_LOG(LogTemp, Error, TEXT("[SimGameMode] DefaultDroneClass must derive from ADronePawn"));
        return nullptr;
    }
    const FVector SpawnLocationUE = SpawnLocationMeters * 100.0f;
    const FRotator SpawnRotation = FRotator::ZeroRotator;
    const FTransform SpawnTransform(SpawnRotation, SpawnLocationUE);
    ADronePawn* Drone = World->SpawnActorDeferred<ADronePawn>(
        DroneClass,
        SpawnTransform,
        nullptr,
        nullptr,
        ESpawnActorCollisionHandlingMethod::AlwaysSpawn);
    if (!Drone) return nullptr;
    Drone->DroneId = DroneId;
    Drone->MissionRole = InMissionRole;
    UGameplayStatics::FinishSpawningActor(Drone, SpawnTransform);
    return Drone;
}
void ASimGameMode::SpawnDefaultDrone()
{
    SpawnedDrone = SpawnDrone(DefaultDroneId, DroneSpawnLocation, DefaultDroneRole);
    if (SpawnedDrone)
    {
        UE_LOG(LogTemp, Log, TEXT("[SimGameMode] Spawned drone '%s' at (%s) m"), *DefaultDroneId, *DroneSpawnLocation.ToString());
        if (bAutoTakeoff)
        {
            FTimerHandle TakeoffTimer;
            GetWorldTimerManager().SetTimer(TakeoffTimer, this, &ASimGameMode::DelayedTakeoff, 1.0f, false);
        }
    }
    else
    {
        UE_LOG(LogTemp, Error, TEXT("[SimGameMode] Failed to spawn drone!"));
    }
}
void ASimGameMode::SpawnConfiguredDrones()
{
    for (const FDroneSpawnConfig& Config : DroneSpawnList)
    {
        ADronePawn* Drone = SpawnDrone(Config.DroneId, Config.SpawnLocation, Config.MissionRole);
        if (!Drone)
        {
            UE_LOG(LogTemp, Error, TEXT("[SimGameMode] Failed to spawn configured drone '%s'"), *Config.DroneId);
            continue;
        }
        UE_LOG(LogTemp, Log, TEXT("[SimGameMode] Spawned configured drone '%s' at (%s) m"), *Config.DroneId, *Config.SpawnLocation.ToString());
        if (Config.bAutoTakeoff)
        {
            TWeakObjectPtr<ADronePawn> WeakDrone = Drone;
            const float Alt = Config.TakeoffAltitude;
            FTimerHandle Timer;
            GetWorldTimerManager().SetTimer(
                Timer,
                FTimerDelegate::CreateLambda([WeakDrone, Alt]()
                {
                    if (WeakDrone.IsValid())
                    {
                        WeakDrone->Takeoff(Alt);
                    }
                }),
                1.0f,
                false);
        }
    }
}
void ASimGameMode::CaptureInitialAgentStates()
{
    InitialAgentTransforms.Empty();
    UAgentManager* Manager = UAgentManager::GetInstance();
    if (!Manager) return;
    const TArray<FString> Ids = Manager->GetAllAgentIds();
    for (const FString& Id : Ids)
    {
        if (AActor* Agent = Manager->GetAgent(Id))
        {
            InitialAgentTransforms.Add(Id, Agent->GetActorTransform());
        }
    }
    UE_LOG(LogTemp, Log, TEXT("[SimGameMode] Captured initial states for %d agents"), InitialAgentTransforms.Num());
}
void ASimGameMode::DelayedTakeoff()
{
    if (ADronePawn* Drone = Cast<ADronePawn>(SpawnedDrone))
    {
        Drone->Takeoff(TakeoffAltitude);
        UE_LOG(LogTemp, Log, TEXT("[SimGameMode] Auto takeoff to %.1f m"), TakeoffAltitude);
    }
}
void ASimGameMode::PauseSimulation()
{
    UGameplayStatics::SetGamePaused(GetWorld(), true);
    UE_LOG(LogTemp, Log, TEXT("[SimGameMode] Simulation PAUSED"));
}
void ASimGameMode::ResumeSimulation()
{
    UGameplayStatics::SetGamePaused(GetWorld(), false);
    UE_LOG(LogTemp, Log, TEXT("[SimGameMode] Simulation RESUMED"));
}
void ASimGameMode::ResetSimulation()
{
    UWorld* World = GetWorld();
    if (!World) return;
    UGameplayStatics::SetGamePaused(World, false);
    if (InitialAgentTransforms.Num() == 0)
    {
        CaptureInitialAgentStates();
    }
    UAgentManager* Manager = UAgentManager::GetInstance();
    if (Manager)
    {
        const TArray<FString> Ids = Manager->GetAllAgentIds();
        for (const FString& Id : Ids)
        {
            AActor* Agent = Manager->GetAgent(Id);
            if (!Agent) continue;
            const FTransform* Initial = InitialAgentTransforms.Find(Id);
            if (!Initial) continue;
            if (ADronePawn* Drone = Cast<ADronePawn>(Agent))
            {
                const FVector PosMeters = Initial->GetLocation() / 100.0f;
                Drone->ResetDrone(PosMeters, Initial->GetRotation().Rotator());
                continue;
            }
            Agent->SetActorTransform(*Initial);
        }
    }
    UE_LOG(LogTemp, Log, TEXT("[SimGameMode] Simulation RESET completed"));
}

#include "SimGameMode.h"
#include "CameraPawn.h"
#include "Manager/AgentManager.h"
#include "Kismet/GameplayStatics.h"

ASimGameMode::ASimGameMode()
{
    // 默认 Pawn 为自由相机
    DefaultPawnClass = ACameraPawn::StaticClass();
    // HUD 在 Phase 5 中设置
}

void ASimGameMode::BeginPlay()
{
    Super::BeginPlay();

    UE_LOG(LogTemp, Log, TEXT("[SimGameMode] BeginPlay"));

    // 自动生成无人机
    if (bAutoSpawnDrone && DefaultDroneClass)
    {
        SpawnDefaultDrone();
    }
    else if (bAutoSpawnDrone && !DefaultDroneClass)
    {
        UE_LOG(LogTemp, Warning, TEXT("[SimGameMode] bAutoSpawnDrone=true but DefaultDroneClass is not set! Please set it in BP_SimGameMode."));
    }
}

void ASimGameMode::SpawnDefaultDrone()
{
    UWorld* World = GetWorld();
    if (!World) return;

    // SI -> UE 单位转换（米 -> 厘米）
    FVector SpawnLocationUE = DroneSpawnLocation * 100.0f;
    FRotator SpawnRotation = FRotator::ZeroRotator;

    FActorSpawnParameters SpawnParams;
    SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

    SpawnedDrone = World->SpawnActor<APawn>(DefaultDroneClass, SpawnLocationUE, SpawnRotation, SpawnParams);

    if (SpawnedDrone)
    {
        // 注册到 AgentManager
        UAgentManager::GetInstance()->RegisterAgent(DefaultDroneId, SpawnedDrone);

        UE_LOG(LogTemp, Log, TEXT("[SimGameMode] Spawned drone '%s' at (%s) m"),
            *DefaultDroneId, *DroneSpawnLocation.ToString());

        // 延迟起飞
        if (bAutoTakeoff)
        {
            FTimerHandle TakeoffTimer;
            World->GetTimerManager().SetTimer(TakeoffTimer, this, &ASimGameMode::DelayedTakeoff, 1.0f, false);
        }
    }
    else
    {
        UE_LOG(LogTemp, Error, TEXT("[SimGameMode] Failed to spawn drone!"));
    }
}

void ASimGameMode::DelayedTakeoff()
{
    if (SpawnedDrone)
    {
        // 通过反射调用 Takeoff（Phase 2 中 DronePawn 会实现此函数）
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
    UE_LOG(LogTemp, Log, TEXT("[SimGameMode] Simulation RESET"));
    // Phase 2 中完善：重置所有 Agent 状态
}

#include "SimGameMode.h"
#include "CameraPawn.h"
#include "Manager/AgentManager.h"
#include "Manager/CommandExecutionManager.h"
#include "Kismet/GameplayStatics.h"
#include "EngineUtils.h"
#include "../Drone/DronePawn.h"
#include "../Turret/TurretPawn.h"
#include "../Turret/BulletActor.h"
#include "../UI/SimHUD.h"

/** @brief 设置默认 Pawn 为 ACameraPawn */
ASimGameMode::ASimGameMode()
{
    DefaultPawnClass = ACameraPawn::StaticClass();
    HUDClass = ASimHUD::StaticClass();
}

/** @brief 根据配置自动生成无人机 */
void ASimGameMode::BeginPlay()
{
    Super::BeginPlay();
    // 每次 PIE 启动前重置单例状态
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
            SpawnConfiguredDrones();                 // 使用配置列表批量生成
        }
        else
        {
            SpawnDefaultDrone();                      // 生成单架默认无人机
        }
    }
    else
    {
        UE_LOG(LogTemp, Log, TEXT("[SimGameMode] Bootstrap auto-spawn is disabled"));
    }

    // 下一帧捕获所有已注册 Agent 的初始状态（供重置使用）
    GetWorldTimerManager().SetTimerForNextTick(this, &ASimGameMode::CaptureInitialAgentStates);
}

/**
 * @brief 在世界中生成一架无人机
 * @param DroneId       唯一标识
 * @param SpawnLocationMeters 生成位置（米）
 * @param InMissionRole 任务角色
 * @return 生成的 DronePawn 指针，失败返回 nullptr
 */
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

    // 米  厘米
    const FVector SpawnLocationUE = SpawnLocationMeters * 100.0f;
    const FRotator SpawnRotation = FRotator::ZeroRotator;
    const FTransform SpawnTransform(SpawnRotation, SpawnLocationUE);

    // 延迟生成以便在 FinishSpawning 前设置属性
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

/** @brief 生成默认无人机并可选自动起飞 */
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

/** @brief 按 DroneSpawnList 配置批量生成无人机 */
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

/** @brief 记录当前所有已注册 Agent 的 Transform，用于重置还原 */
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

/** @brief 延迟起飞回调 */
void ASimGameMode::DelayedTakeoff()
{
    if (ADronePawn* Drone = Cast<ADronePawn>(SpawnedDrone))
    {
        Drone->Takeoff(TakeoffAltitude);
        UE_LOG(LogTemp, Log, TEXT("[SimGameMode] Auto takeoff to %.1f m"), TakeoffAltitude);
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

/** @brief 重置仿真：清除子弹、恢复所有 Agent 初始状态 */
void ASimGameMode::ResetSimulation()
{
    UWorld* World = GetWorld();
    if (!World) return;

    // 先取消暂停
    UGameplayStatics::SetGamePaused(World, false);

    // 销毁所有残留子弹
    for (TActorIterator<ABulletActor> It(World); It; ++It)
    {
        if (IsValid(*It))
        {
            It->Destroy();
        }
    }

    // 若尚未捕获过初始状态，先捕获
    if (InitialAgentTransforms.Num() == 0)
    {
        CaptureInitialAgentStates();
    }

    // 逐 Agent 恢复初始 Transform
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

            // 无人机：使用 ResetDrone（米制坐标 + 旋转）
            if (ADronePawn* Drone = Cast<ADronePawn>(Agent))
            {
                const FVector PosMeters = Initial->GetLocation() / 100.0f;
                Drone->ResetDrone(PosMeters, Initial->GetRotation().Rotator());
                continue;
            }

            // 炮塔：恢复位置并清除跟踪/预测线
            if (ATurretPawn* Turret = Cast<ATurretPawn>(Agent))
            {
                Turret->SetActorLocationAndRotation(Initial->GetLocation(), Initial->GetRotation().Rotator());
                Turret->SetTargetAngles(0.0f, 0.0f);
                Turret->StopTracking();
                Turret->HidePredictionLine();
                continue;
            }

            // 其他 Actor：直接恢复 Transform
            Agent->SetActorTransform(*Initial);
        }
    }

    UE_LOG(LogTemp, Log, TEXT("[SimGameMode] Simulation RESET completed"));
}
/**
 * @file SimGameMode.cpp
 * @brief 仿真 GameMode 的实现文件
 *
 * 实现无人机自动生成、延迟起飞（反射调用）和仿真暂停/恢复/重置功能。
 */

#include "SimGameMode.h"
#include "CameraPawn.h"
#include "Manager/AgentManager.h"
#include "Kismet/GameplayStatics.h"

/**
 * @brief 构造函数
 *
 * 设置默认 Pawn 为 ACameraPawn（自由相机），
 * HUD 在后续阶段中设置。
 */
ASimGameMode::ASimGameMode()
{
    // 默认玩家 Pawn 为自由相机
    DefaultPawnClass = ACameraPawn::StaticClass();
    // HUD 在 Phase 5 中设置
}

/**
 * @brief 游戏开始时调用
 *
 * 根据配置自动生成无人机：
 * - bAutoSpawnDrone 为 true 且 DefaultDroneClass 已设置 → 生成无人机
 * - bAutoSpawnDrone 为 true 但未设置蓝图类 → 打印警告
 */
void ASimGameMode::BeginPlay()
{
    Super::BeginPlay();

    UE_LOG(LogTemp, Log, TEXT("[SimGameMode] BeginPlay"));

    // 根据配置决定是否自动生成无人机
    if (bAutoSpawnDrone && DefaultDroneClass)
    {
        SpawnDefaultDrone();
    }
    else if (bAutoSpawnDrone && !DefaultDroneClass)
    {
        UE_LOG(LogTemp, Warning, TEXT("[SimGameMode] bAutoSpawnDrone=true but DefaultDroneClass is not set! Please set it in BP_SimGameMode."));
    }
}

/**
 * @brief 生成默认无人机
 *
 * 流程：
 * 1. 将生成位置从 SI 单位（米）转换为 UE 单位（厘米）
 * 2. 使用 SpawnActor 生成无人机 Pawn
 * 3. 注册到 AgentManager
 * 4. 如果启用自动起飞，设置 1 秒延迟后调用 DelayedTakeoff()
 */
void ASimGameMode::SpawnDefaultDrone()
{
    UWorld* World = GetWorld();
    if (!World) return;

    // SI → UE 单位转换（米 → 厘米）
    FVector SpawnLocationUE = DroneSpawnLocation * 100.0f;
    FRotator SpawnRotation = FRotator::ZeroRotator;

    // 配置生成参数：总是生成，忽略碰撞
    FActorSpawnParameters SpawnParams;
    SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

    // 生成无人机 Pawn
    SpawnedDrone = World->SpawnActor<APawn>(DefaultDroneClass, SpawnLocationUE, SpawnRotation, SpawnParams);

    if (SpawnedDrone)
    {
        // 注册到全局智能体管理器
        UAgentManager::GetInstance()->RegisterAgent(DefaultDroneId, SpawnedDrone);

        UE_LOG(LogTemp, Log, TEXT("[SimGameMode] Spawned drone '%s' at (%s) m"),
            *DefaultDroneId, *DroneSpawnLocation.ToString());

        // 设置延迟起飞（1 秒后触发，让所有组件完成初始化）
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

/**
 * @brief 延迟起飞回调
 *
 * 通过 UE 反射系统（FindFunction + ProcessEvent）调用无人机的 Takeoff 函数，
 * 避免 SimGameMode 对 DronePawn 类型的硬编码依赖。
 * 如果无人机尚未实现 Takeoff 函数，会打印警告。
 */
void ASimGameMode::DelayedTakeoff()
{
    if (SpawnedDrone)
    {
        // 通过反射查找 Takeoff 函数
        UFunction* TakeoffFunc = SpawnedDrone->FindFunction(FName("Takeoff"));
        if (TakeoffFunc)
        {
            // 构造函数参数结构体
            struct { float Altitude; } Params;
            Params.Altitude = TakeoffAltitude;
            // 通过 ProcessEvent 调用函数
            SpawnedDrone->ProcessEvent(TakeoffFunc, &Params);
            UE_LOG(LogTemp, Log, TEXT("[SimGameMode] Auto takeoff to %.1f m"), TakeoffAltitude);
        }
        else
        {
            UE_LOG(LogTemp, Warning, TEXT("[SimGameMode] Drone does not have Takeoff function yet (Phase 2)"));
        }
    }
}

/**
 * @brief 暂停仿真
 *
 * 调用 UGameplayStatics::SetGamePaused() 暂停所有 Actor 的 Tick 和物理仿真。
 */
void ASimGameMode::PauseSimulation()
{
    UGameplayStatics::SetGamePaused(GetWorld(), true);
    UE_LOG(LogTemp, Log, TEXT("[SimGameMode] Simulation PAUSED"));
}

/**
 * @brief 恢复仿真
 *
 * 取消暂停状态，恢复所有 Actor 的 Tick 和物理仿真。
 */
void ASimGameMode::ResumeSimulation()
{
    UGameplayStatics::SetGamePaused(GetWorld(), false);
    UE_LOG(LogTemp, Log, TEXT("[SimGameMode] Simulation RESUMED"));
}

/**
 * @brief 重置仿真
 *
 * 当前仅打印日志。后续版本中将遍历所有 Agent 并重置其状态。
 */
void ASimGameMode::ResetSimulation()
{
    UE_LOG(LogTemp, Log, TEXT("[SimGameMode] Simulation RESET"));
    // Phase 2 中完善：重置所有 Agent 状态
}

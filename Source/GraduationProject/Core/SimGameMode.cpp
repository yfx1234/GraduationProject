// 解释：引入当前实现文件对应的头文件 `SimGameMode.h`，使实现部分能够看到类和函数声明。
#include "SimGameMode.h"
// 解释：引入 `CameraPawn.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "CameraPawn.h"
// 解释：引入 `AgentManager.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "Manager/AgentManager.h"
// 解释：引入 `CommandExecutionManager.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "Manager/CommandExecutionManager.h"
// 解释：引入 `GameplayStatics.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "Kismet/GameplayStatics.h"
// 解释：引入 `EngineUtils.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "EngineUtils.h"
// 解释：引入 `DronePawn.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "../Drone/DronePawn.h"
// 解释：引入 `TurretPawn.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "../Turret/TurretPawn.h"
// 解释：引入 `BulletActor.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "../Turret/BulletActor.h"
// 解释：引入 `SimHUD.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "../UI/SimHUD.h"

/** @brief 设置默认 Pawn 为 ACameraPawn */
// 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
ASimGameMode::ASimGameMode()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `DefaultPawnClass`，完成 defaultPawnclass 的更新。
    DefaultPawnClass = ACameraPawn::StaticClass();
    // 解释：这一行把右侧表达式的结果写入 `HUDClass`，完成 hudclass 的更新。
    HUDClass = ASimHUD::StaticClass();
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 根据配置自动生成无人机 */
// 解释：这一行定义函数 `BeginPlay`，开始实现beginplay的具体逻辑。
void ASimGameMode::BeginPlay()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：调用 `BeginPlay` 执行当前步骤需要的功能逻辑。
    Super::BeginPlay();
    // 每次 PIE 启动前重置单例状态
    // 解释：调用 `Cleanup` 执行当前步骤需要的功能逻辑。
    UAgentManager::Cleanup();
    // 解释：调用 `Cleanup` 执行当前步骤需要的功能逻辑。
    UCommandExecutionManager::Cleanup();
    // 解释：调用 `GetInstance` 执行当前步骤需要的功能逻辑。
    UAgentManager::GetInstance();
    // 解释：调用 `GetInstance` 执行当前步骤需要的功能逻辑。
    UCommandExecutionManager::GetInstance();

    // 解释：这一行位于构造函数初始化列表中，把 `UE_LOG` 直接初始化为 `LogTemp, Log, TEXT("[SimGameMode] BeginPlay (AutoSpawn=%s, Bootstrap=%s)"`，减少进入函数体后的额外赋值开销。
    UE_LOG(LogTemp, Log, TEXT("[SimGameMode] BeginPlay (AutoSpawn=%s, Bootstrap=%s)"),
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        bAutoSpawnDrone ? TEXT("true") : TEXT("false"),
        // 解释：调用 `TEXT` 执行当前步骤需要的功能逻辑。
        bEnableBootstrapSpawn ? TEXT("true") : TEXT("false"));

    // 解释：这一行声明成员或局部变量 `bShouldBootstrapSpawn`，用于保存布尔标志 shouldbootstrapspawn。
    const bool bShouldBootstrapSpawn = bEnableBootstrapSpawn && bAutoSpawnDrone;
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (bShouldBootstrapSpawn)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (!DefaultDroneClass)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：调用 `UE_LOG` 输出调试日志，便于运行时观察状态和排查问题。
            UE_LOG(LogTemp, Warning, TEXT("[SimGameMode] bootstrap spawn enabled but DefaultDroneClass is not set"));
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
        // 解释：这一行声明补充分支，当上一个条件不成立且当前条件成立时执行。
        else if (bUseDroneSpawnList && DroneSpawnList.Num() > 0)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：调用 `SpawnConfiguredDrones` 执行当前步骤需要的功能逻辑。
            SpawnConfiguredDrones();                 // 使用配置列表批量生成
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
        // 解释：这一行给出兜底分支，当上面的条件都不满足时执行。
        else
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：调用 `SpawnDefaultDrone` 执行当前步骤需要的功能逻辑。
            SpawnDefaultDrone();                      // 生成单架默认无人机
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行给出兜底分支，当上面的条件都不满足时执行。
    else
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `UE_LOG` 输出调试日志，便于运行时观察状态和排查问题。
        UE_LOG(LogTemp, Log, TEXT("[SimGameMode] Bootstrap auto-spawn is disabled"));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 下一帧捕获所有已注册 Agent 的初始状态（供重置使用）
    // 解释：调用 `GetWorldTimerManager` 执行当前步骤需要的功能逻辑。
    GetWorldTimerManager().SetTimerForNextTick(this, &ASimGameMode::CaptureInitialAgentStates);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 在世界中生成一架无人机
 * @param DroneId       唯一标识
 * @param SpawnLocationMeters 生成位置（米）
 * @param InMissionRole 任务角色
 * @return 生成的 DronePawn 指针，失败返回 nullptr
 */
// 解释：这一行定义函数 `SpawnDrone`，开始实现spawn无人机的具体逻辑。
ADronePawn* ASimGameMode::SpawnDrone(const FString& DroneId, const FVector& SpawnLocationMeters, EDroneMissionRole InMissionRole)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `UWorld* World`，完成 uworldworld 的更新。
    UWorld* World = GetWorld();
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!World) return nullptr;

    // 解释：调用 `ADronePawn>` 执行当前步骤需要的功能逻辑。
    const TSubclassOf<ADronePawn> DroneClass = TSubclassOf<ADronePawn>(DefaultDroneClass);
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!DroneClass)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `UE_LOG` 输出调试日志，便于运行时观察状态和排查问题。
        UE_LOG(LogTemp, Error, TEXT("[SimGameMode] DefaultDroneClass must derive from ADronePawn"));
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return nullptr;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 米  厘米
    // 解释：这一行声明成员或局部变量 `SpawnLocationUE`，用于保存spawnlocationue。
    const FVector SpawnLocationUE = SpawnLocationMeters * 100.0f;
    // 解释：这一行声明成员或局部变量 `SpawnRotation`，用于保存spawnrotation。
    const FRotator SpawnRotation = FRotator::ZeroRotator;
    // 解释：调用 `SpawnTransform` 执行当前步骤需要的功能逻辑。
    const FTransform SpawnTransform(SpawnRotation, SpawnLocationUE);

    // 延迟生成以便在 FinishSpawning 前设置属性
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    ADronePawn* Drone = World->SpawnActorDeferred<ADronePawn>(
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        DroneClass,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        SpawnTransform,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        nullptr,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        nullptr,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        ESpawnActorCollisionHandlingMethod::AlwaysSpawn);

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!Drone) return nullptr;

    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    Drone->DroneId = DroneId;
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    Drone->MissionRole = InMissionRole;
    // 解释：调用 `FinishSpawningActor` 执行当前步骤需要的功能逻辑。
    UGameplayStatics::FinishSpawningActor(Drone, SpawnTransform);
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return Drone;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 生成默认无人机并可选自动起飞 */
// 解释：这一行定义函数 `SpawnDefaultDrone`，开始实现spawndefault无人机的具体逻辑。
void ASimGameMode::SpawnDefaultDrone()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `SpawnedDrone`，完成 spawned无人机 的更新。
    SpawnedDrone = SpawnDrone(DefaultDroneId, DroneSpawnLocation, DefaultDroneRole);

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (SpawnedDrone)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `UE_LOG` 输出调试日志，便于运行时观察状态和排查问题。
        UE_LOG(LogTemp, Log, TEXT("[SimGameMode] Spawned drone '%s' at (%s) m"), *DefaultDroneId, *DroneSpawnLocation.ToString());
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (bAutoTakeoff)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行声明成员或局部变量 `TakeoffTimer`，用于保存takeofftimer。
            FTimerHandle TakeoffTimer;
            // 解释：调用 `GetWorldTimerManager` 执行当前步骤需要的功能逻辑。
            GetWorldTimerManager().SetTimer(TakeoffTimer, this, &ASimGameMode::DelayedTakeoff, 1.0f, false);
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行给出兜底分支，当上面的条件都不满足时执行。
    else
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `UE_LOG` 输出调试日志，便于运行时观察状态和排查问题。
        UE_LOG(LogTemp, Error, TEXT("[SimGameMode] Failed to spawn drone!"));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 按 DroneSpawnList 配置批量生成无人机 */
// 解释：这一行定义函数 `SpawnConfiguredDrones`，开始实现spawnconfigureddrones的具体逻辑。
void ASimGameMode::SpawnConfiguredDrones()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
    for (const FDroneSpawnConfig& Config : DroneSpawnList)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `ADronePawn* Drone`，完成 adronePawn无人机 的更新。
        ADronePawn* Drone = SpawnDrone(Config.DroneId, Config.SpawnLocation, Config.MissionRole);
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (!Drone)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：调用 `UE_LOG` 输出调试日志，便于运行时观察状态和排查问题。
            UE_LOG(LogTemp, Error, TEXT("[SimGameMode] Failed to spawn configured drone '%s'"), *Config.DroneId);
            // 解释：这一行跳过本轮循环剩余语句，直接进入下一轮迭代。
            continue;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：调用 `UE_LOG` 输出调试日志，便于运行时观察状态和排查问题。
        UE_LOG(LogTemp, Log, TEXT("[SimGameMode] Spawned configured drone '%s' at (%s) m"), *Config.DroneId, *Config.SpawnLocation.ToString());

        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (Config.bAutoTakeoff)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行声明成员或局部变量 `WeakDrone`，用于保存weak无人机。
            TWeakObjectPtr<ADronePawn> WeakDrone = Drone;
            // 解释：这一行声明成员或局部变量 `Alt`，用于保存alt。
            const float Alt = Config.TakeoffAltitude;
            // 解释：这一行声明成员或局部变量 `Timer`，用于保存timer。
            FTimerHandle Timer;
            // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
            GetWorldTimerManager().SetTimer(
                // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                Timer,
                // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                FTimerDelegate::CreateLambda([WeakDrone, Alt]()
                // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
                {
                    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
                    if (WeakDrone.IsValid())
                    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
                    {
                        // 解释：调用 `Takeoff` 执行当前步骤需要的功能逻辑。
                        WeakDrone->Takeoff(Alt);
                    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
                    }
                // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                }),
                // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                1.0f,
                // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                false);
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 记录当前所有已注册 Agent 的 Transform，用于重置还原 */
// 解释：这一行定义函数 `CaptureInitialAgentStates`，开始实现采集initialagentstates的具体逻辑。
void ASimGameMode::CaptureInitialAgentStates()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：调用 `Empty` 执行当前步骤需要的功能逻辑。
    InitialAgentTransforms.Empty();

    // 解释：这一行把右侧表达式的结果写入 `UAgentManager* Manager`，完成 uagent管理器管理器 的更新。
    UAgentManager* Manager = UAgentManager::GetInstance();
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!Manager) return;

    // 解释：调用 `GetAllAgentIds` 执行当前步骤需要的功能逻辑。
    const TArray<FString> Ids = Manager->GetAllAgentIds();
    // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
    for (const FString& Id : Ids)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (AActor* Agent = Manager->GetAgent(Id))
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：调用 `Add` 执行当前步骤需要的功能逻辑。
            InitialAgentTransforms.Add(Id, Agent->GetActorTransform());
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：调用 `UE_LOG` 输出调试日志，便于运行时观察状态和排查问题。
    UE_LOG(LogTemp, Log, TEXT("[SimGameMode] Captured initial states for %d agents"), InitialAgentTransforms.Num());
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 延迟起飞回调 */
// 解释：这一行定义函数 `DelayedTakeoff`，开始实现delayedtakeoff的具体逻辑。
void ASimGameMode::DelayedTakeoff()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (ADronePawn* Drone = Cast<ADronePawn>(SpawnedDrone))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `Takeoff` 执行当前步骤需要的功能逻辑。
        Drone->Takeoff(TakeoffAltitude);
        // 解释：调用 `UE_LOG` 输出调试日志，便于运行时观察状态和排查问题。
        UE_LOG(LogTemp, Log, TEXT("[SimGameMode] Auto takeoff to %.1f m"), TakeoffAltitude);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 暂停仿真 */
// 解释：这一行定义函数 `PauseSimulation`，开始实现pausesimulation的具体逻辑。
void ASimGameMode::PauseSimulation()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：调用 `SetGamePaused` 执行当前步骤需要的功能逻辑。
    UGameplayStatics::SetGamePaused(GetWorld(), true);
    // 解释：调用 `UE_LOG` 输出调试日志，便于运行时观察状态和排查问题。
    UE_LOG(LogTemp, Log, TEXT("[SimGameMode] Simulation PAUSED"));
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 恢复仿真 */
// 解释：这一行定义函数 `ResumeSimulation`，开始实现resumesimulation的具体逻辑。
void ASimGameMode::ResumeSimulation()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：调用 `SetGamePaused` 执行当前步骤需要的功能逻辑。
    UGameplayStatics::SetGamePaused(GetWorld(), false);
    // 解释：调用 `UE_LOG` 输出调试日志，便于运行时观察状态和排查问题。
    UE_LOG(LogTemp, Log, TEXT("[SimGameMode] Simulation RESUMED"));
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 重置仿真：清除子弹、恢复所有 Agent 初始状态 */
// 解释：这一行定义函数 `ResetSimulation`，开始实现resetsimulation的具体逻辑。
void ASimGameMode::ResetSimulation()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `UWorld* World`，完成 uworldworld 的更新。
    UWorld* World = GetWorld();
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!World) return;

    // 先取消暂停
    // 解释：调用 `SetGamePaused` 执行当前步骤需要的功能逻辑。
    UGameplayStatics::SetGamePaused(World, false);

    // 销毁所有残留子弹
    // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
    for (TActorIterator<ABulletActor> It(World); It; ++It)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (IsValid(*It))
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：调用 `Destroy` 执行当前步骤需要的功能逻辑。
            It->Destroy();
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 若尚未捕获过初始状态，先捕获
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (InitialAgentTransforms.Num() == 0)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `CaptureInitialAgentStates` 执行当前步骤需要的功能逻辑。
        CaptureInitialAgentStates();
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 逐 Agent 恢复初始 Transform
    // 解释：这一行把右侧表达式的结果写入 `UAgentManager* Manager`，完成 uagent管理器管理器 的更新。
    UAgentManager* Manager = UAgentManager::GetInstance();
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (Manager)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `GetAllAgentIds` 执行当前步骤需要的功能逻辑。
        const TArray<FString> Ids = Manager->GetAllAgentIds();
        // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
        for (const FString& Id : Ids)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行把右侧表达式的结果写入 `AActor* Agent`，完成 aactoragent 的更新。
            AActor* Agent = Manager->GetAgent(Id);
            // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
            if (!Agent) continue;

            // 解释：这一行把右侧表达式的结果写入 `const FTransform* Initial`，完成 constftransforminitial 的更新。
            const FTransform* Initial = InitialAgentTransforms.Find(Id);
            // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
            if (!Initial) continue;

            // 无人机：使用 ResetDrone（米制坐标 + 旋转）
            // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
            if (ADronePawn* Drone = Cast<ADronePawn>(Agent))
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            {
                // 解释：这一行把右侧表达式的结果写入 `const FVector PosMeters`，完成 constfvectorposmeters 的更新。
                const FVector PosMeters = Initial->GetLocation() / 100.0f;
                // 解释：调用 `ResetDrone` 执行当前步骤需要的功能逻辑。
                Drone->ResetDrone(PosMeters, Initial->GetRotation().Rotator());
                // 解释：这一行跳过本轮循环剩余语句，直接进入下一轮迭代。
                continue;
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            }

            // 炮塔：恢复位置并清除跟踪/预测线
            // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
            if (ATurretPawn* Turret = Cast<ATurretPawn>(Agent))
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            {
                // 解释：调用 `SetActorLocationAndRotation` 执行当前步骤需要的功能逻辑。
                Turret->SetActorLocationAndRotation(Initial->GetLocation(), Initial->GetRotation().Rotator());
                // 解释：调用 `SetTargetAngles` 执行当前步骤需要的功能逻辑。
                Turret->SetTargetAngles(0.0f, 0.0f);
                // 解释：调用 `StopTracking` 执行当前步骤需要的功能逻辑。
                Turret->StopTracking();
                // 解释：调用 `HidePredictionLine` 执行当前步骤需要的功能逻辑。
                Turret->HidePredictionLine();
                // 解释：这一行跳过本轮循环剩余语句，直接进入下一轮迭代。
                continue;
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            }

            // 其他 Actor：直接恢复 Transform
            // 解释：调用 `SetActorTransform` 执行当前步骤需要的功能逻辑。
            Agent->SetActorTransform(*Initial);
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：调用 `UE_LOG` 输出调试日志，便于运行时观察状态和排查问题。
    UE_LOG(LogTemp, Log, TEXT("[SimGameMode] Simulation RESET completed"));
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

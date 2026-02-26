/**
 * @file SimGameMode.h
 * @brief 仿真 GameMode 的头文件
 *
 * 本文件定义了 ASimGameMode 类，负责仿真的全局管理：
 * - 配置默认 Pawn（自由相机）和 HUD
 * - 自动生成无人机 Agent
 * - 仿真控制（暂停/恢复/重置）
 * 参考 AirSim 的 SimModeBase 职责划分。
 */

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "SimGameMode.generated.h"

/**
 * 仿真 GameMode
 *
 * 职责（参考 AirSim SimModeBase）：
 * 1. 智能体（Agent）管理 — 负责生成/销毁无人机和转台
 * 2. 仿真控制 — 暂停/恢复/重置仿真
 * 3. 默认 Pawn 和 HUD 配置 — 指定 ACameraPawn 为默认玩家 Pawn
 *
 * 在 Blueprint 中（BP_SimGameMode）可配置无人机蓝图类、生成位置等参数。
 */
UCLASS()
class GRADUATIONPROJECT_API ASimGameMode : public AGameModeBase
{
    GENERATED_BODY()

public:
    /** @brief 构造函数，设置默认 Pawn 为 ACameraPawn */
    ASimGameMode();

    /**
     * @brief 游戏开始时调用
     *
     * 如果 bAutoSpawnDrone 为 true 且 DefaultDroneClass 已配置，
     * 则自动生成一架无人机。
     */
    virtual void BeginPlay() override;

    // ---- 仿真控制（参考 AirSim pause/continueForTime） ----

    /**
     * @brief 暂停仿真
     *
     * 通过 UGameplayStatics::SetGamePaused() 暂停所有 Actor 的 Tick。
     */
    UFUNCTION(BlueprintCallable, Category = "Simulation")
    void PauseSimulation();

    /**
     * @brief 恢复仿真
     *
     * 取消暂停状态。
     */
    UFUNCTION(BlueprintCallable, Category = "Simulation")
    void ResumeSimulation();

    /**
     * @brief 重置仿真
     *
     * 目前仅打印日志，后续版本中将重置所有 Agent 状态到初始值。
     */
    UFUNCTION(BlueprintCallable, Category = "Simulation")
    void ResetSimulation();

protected:
    /**
     * @brief 生成默认无人机
     *
     * 从 DefaultDroneClass 蓝图类生成无人机 Pawn，
     * 将生成位置从 SI 单位（米）转换为 UE 单位（厘米），
     * 注册到 AgentManager，并根据配置延迟自动起飞。
     */
    void SpawnDefaultDrone();

    /**
     * @brief 延迟起飞回调（由定时器 1 秒后触发）
     *
     * 通过反射调用（ProcessEvent）无人机的 Takeoff 函数，
     * 以解除对具体 DronePawn 类型的编译依赖。
     */
    void DelayedTakeoff();

public:
    // ---- Blueprint 可配置参数 ----

    /** @brief 是否在游戏开始时自动生成无人机 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Drone")
    bool bAutoSpawnDrone = true;

    /** @brief 无人机蓝图类（在 Blueprint 中选择 BP_Drone） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Drone")
    TSubclassOf<APawn> DefaultDroneClass;

    /** @brief 无人机生成位置（SI 单位，米），会自动转换为 UE 厘米单位 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Drone")
    FVector DroneSpawnLocation = FVector(0.0f, 0.0f, 0.5f);

    /** @brief 无人机生成时分配的 Agent ID */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Drone")
    FString DefaultDroneId = TEXT("drone_0");

    /** @brief 是否在生成后自动起飞 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Drone")
    bool bAutoTakeoff = true;

    /** @brief 自动起飞目标高度（米） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Drone")
    float TakeoffAltitude = 3.0f;

private:
    /** @brief 已生成的无人机 Pawn 引用 */
    UPROPERTY()
    APawn* SpawnedDrone = nullptr;
};

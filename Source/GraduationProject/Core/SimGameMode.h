#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "SimGameMode.generated.h"

/**
 * 仿真 GameMode（参考 AirSim SimModeBase 职责划分）
 * 职责：
 *   1. Agent 管理（生成/销毁无人机/转台）
 *   2. 仿真控制（暂停/恢复/重置）
 *   3. 默认 Pawn 和 HUD 配置
 */
UCLASS()
class GRADUATIONPROJECT_API ASimGameMode : public AGameModeBase
{
    GENERATED_BODY()

public:
    ASimGameMode();

    virtual void BeginPlay() override;

    // ---- 仿真控制（参考 AirSim pause/continueForTime） ----

    UFUNCTION(BlueprintCallable, Category = "Simulation")
    void PauseSimulation();

    UFUNCTION(BlueprintCallable, Category = "Simulation")
    void ResumeSimulation();

    UFUNCTION(BlueprintCallable, Category = "Simulation")
    void ResetSimulation();

protected:
    /** 生成默认无人机 */
    void SpawnDefaultDrone();

    /** 延迟起飞 */
    void DelayedTakeoff();

public:
    // ---- Blueprint 可配置参数 ----

    /** 是否自动生成无人机 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Drone")
    bool bAutoSpawnDrone = true;

    /** 无人机蓝图类（在 Blueprint 中选择 BP_Drone） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Drone")
    TSubclassOf<APawn> DefaultDroneClass;

    /** 无人机生成位置（SI 单位，米） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Drone")
    FVector DroneSpawnLocation = FVector(0.0f, 0.0f, 0.5f);

    /** 无人机生成时的 ID */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Drone")
    FString DefaultDroneId = TEXT("drone_0");

    /** 是否自动起飞 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Drone")
    bool bAutoTakeoff = true;

    /** 起飞高度（米） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Drone")
    float TakeoffAltitude = 3.0f;

private:
    UPROPERTY()
    APawn* SpawnedDrone = nullptr;
};

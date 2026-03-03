#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "SimGameMode.generated.h"

/**
 * 智能体管理 — 负责生成/销毁无人机和转台
 * 仿真控制 — 暂停/恢复/重置仿真
 * 默认 Pawn 和 HUD 配置
 */
UCLASS()
class GRADUATIONPROJECT_API ASimGameMode : public AGameModeBase
{
    GENERATED_BODY()

public:
    ASimGameMode();
    virtual void BeginPlay() override;

    /** @brief 暂停仿真 */
    UFUNCTION(BlueprintCallable, Category = "Simulation")
    void PauseSimulation();

    /** @brief 恢复仿真 */
    UFUNCTION(BlueprintCallable, Category = "Simulation")
    void ResumeSimulation();

    /** @brief 重置仿真 */
    UFUNCTION(BlueprintCallable, Category = "Simulation")
    void ResetSimulation();

protected:
    /** @brief 生成默认无人机 */
    void SpawnDefaultDrone();

    /** @brief 延迟起飞回调 */
    void DelayedTakeoff();

public:
    /** @brief 是否在游戏开始时自动生成无人机 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Drone")
    bool bAutoSpawnDrone = true;

    /** @brief 无人机蓝图类 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Drone")
    TSubclassOf<APawn> DefaultDroneClass;

    /** @brief 无人机生成位置 */
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

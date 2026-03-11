#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "../Drone/DronePawn.h"
#include "SimGameMode.generated.h"

USTRUCT(BlueprintType)
struct FDroneSpawnConfig
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Drone")
    FString DroneId = TEXT("drone_0");

    // meters in simulation coordinates
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Drone")
    FVector SpawnLocation = FVector(0.0f, 0.0f, 0.5f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Drone")
    bool bAutoTakeoff = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Drone")
    float TakeoffAltitude = 3.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Drone")
    EDroneMissionRole MissionRole = EDroneMissionRole::Unknown;
};

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

    /** @brief 按配置生成多无人机 */
    void SpawnConfiguredDrones();

    /** @brief 生成一架无人机（位置单位：米） */
    ADronePawn* SpawnDrone(const FString& DroneId, const FVector& SpawnLocationMeters, EDroneMissionRole InMissionRole = EDroneMissionRole::Unknown);

    /** @brief 捕获当前 Agent 初始状态快照，用于 reset */
    void CaptureInitialAgentStates();

    /** @brief 延迟起飞回调 */
    void DelayedTakeoff();

public:
    /** @brief 是否在游戏开始时自动生成无人机 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Drone")
    bool bAutoSpawnDrone = false;

    /** @brief 启动阶段是否允许批量自动生成（双开关保护，默认关闭） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Drone")
    bool bEnableBootstrapSpawn = false;

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

    /** @brief 默认单机角色（目标机/拦截机） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Drone")
    EDroneMissionRole DefaultDroneRole = EDroneMissionRole::Target;

    /** @brief 是否启用多无人机配置生成 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Drone")
    bool bUseDroneSpawnList = false;

    /** @brief 多无人机生成配置列表（启用后优先于单机配置） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Drone")
    TArray<FDroneSpawnConfig> DroneSpawnList;

private:
    /** @brief 已生成的无人机 Pawn 引用 */
    UPROPERTY()
    APawn* SpawnedDrone = nullptr;

    UPROPERTY()
    TMap<FString, FTransform> InitialAgentTransforms;
};





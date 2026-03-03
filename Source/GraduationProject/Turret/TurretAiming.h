#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "TurretAiming.generated.h"

class ATurretPawn;

/**
 * @brief 转台跟踪组件
 * 作为 ActorComponent 挂载在 ATurretPawn 上，
 * 自动计算目标方位角并设置转台旋转。
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class GRADUATIONPROJECT_API UTurretAiming : public UActorComponent
{
    GENERATED_BODY()

public:
    /** @brief 构造函数，启用 TickComponent */
    UTurretAiming();

protected:
    /**
     * @brief 游戏开始时调用
     * 获取 Owner 并转型为 ATurretPawn
     */
    virtual void BeginPlay() override;

public:
    /**
     * @brief 每帧组件更新
     * @param DeltaTime 帧间隔时间（秒）
     * @param TickType Tick 类型
     * @param ThisTickFunction Tick 函数指针
     * 当跟踪状态激活时更新瞄准角度。
     */
    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

    /**
     * @brief 通过 Agent ID 开始跟踪目标
     * @param TargetActorID 目标 Agent ID
     * 从 AgentManager 查找目标 Actor 并设置为跟踪目标。
     */
    UFUNCTION(BlueprintCallable, Category = "Turret Aiming")
    void StartTracking(const FString& TargetActorID);

    /** @brief 停止跟踪 */
    UFUNCTION(BlueprintCallable, Category = "Turret Aiming")
    void StopTracking();

    /**
     * @brief 传入 Actor 指针设置跟踪目标
     * @param Target 目标 Actor，传入 nullptr 等于停止跟踪
     */
    UFUNCTION(BlueprintCallable, Category = "Turret Aiming")
    void SetTrackingTarget(AActor* Target);

    /**
     * @brief 查询当前是否正在跟踪目标
     * @return true 表示正在跟踪
     */
    UFUNCTION(BlueprintCallable, Category = "Turret Aiming")
    bool IsTracking() const { return bIsTracking; }

private:
    /**
     * @brief 计算瞄准角度（Pitch/Yaw）
     * @param TargetPos 目标世界位置
     * @param MuzzleSpeed 弹丸出膛速度
     * @param OutPitch 计算得到的俯仰角
     * @param OutYaw 计算得到的偏航角
     * @return true 表示计算成功
     * 两步计算：
     * 先从转台中心到目标的方向估算初始角度
     * 用初始角度估算枪口位置，再从枪口到目标重新计算最终角度
     * 补偿枪口相对转台中心的偏移量。
     */
    bool CalculateAimAngles(const FVector& TargetPos, float MuzzleSpeed, float& OutPitch, float& OutYaw);

    /**
     * @brief 根据给定角度计算枪口世界位置
     * @param Pitch 俯仰角
     * @param Yaw 偏航角
     * @return 枪口世界位置
     */
    FVector GetMuzzleWorldPosition(float Pitch, float Yaw) const;

    /**
     * @brief 每帧更新跟踪逻辑
     * @param DeltaTime 帧间隔时间（秒）
     */
    void UpdateTracking(float DeltaTime);

    /**
     * @brief 获取跟踪目标的当前世界位置
     * @return 目标 Actor 的世界位置，无目标时返回 ZeroVector
     */
    FVector GetTargetPosition() const;

    /** @brief 是否正在跟踪目标 */
    UPROPERTY()
    bool bIsTracking = false;

    /** @brief 当前跟踪的目标 Actor */
    UPROPERTY()
    AActor* TrackingTarget = nullptr;

    /** @brief 所属的转台 Pawn */
    UPROPERTY()
    ATurretPawn* OwnerTurret = nullptr;

    /** @brief 瞄准迭代最大次数 */
    static constexpr int32 MaxIterations = 10;

    /** @brief 瞄准收敛阈值 */
    static constexpr float ConvergenceThreshold = 0.1f;
};

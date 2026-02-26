/**
 * @file TurretAiming.h
 * @brief 转台瞄准计算组件的头文件
 *
 * 定义 UTurretAiming 组件，负责计算转台的瞄准角度。
 * 支持直接瞄准和使用制导算法（IGuidanceMethod）计算射击诸元。
 */

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "TurretAiming.generated.h"

class ATurretPawn;

/**
 * 转台瞄准/跟踪组件
 * 自动计算目标方位角并设置转台旋转
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class GRADUATIONPROJECT_API UTurretAiming : public UActorComponent
{
    GENERATED_BODY()

public:
    UTurretAiming();

protected:
    virtual void BeginPlay() override;

public:
    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

    /** 通过 ActorID 开始跟踪 */
    UFUNCTION(BlueprintCallable, Category = "Turret Aiming")
    void StartTracking(const FString& TargetActorID);

    /** 停止跟踪 */
    UFUNCTION(BlueprintCallable, Category = "Turret Aiming")
    void StopTracking();

    /** 直接传入 Actor 指针跟踪 */
    UFUNCTION(BlueprintCallable, Category = "Turret Aiming")
    void SetTrackingTarget(AActor* Target);

    /** 查询跟踪状态 */
    UFUNCTION(BlueprintCallable, Category = "Turret Aiming")
    bool IsTracking() const { return bIsTracking; }

private:
    bool CalculateAimAngles(const FVector& TargetPos, float MuzzleSpeed, float& OutPitch, float& OutYaw);
    FVector GetMuzzleWorldPosition(float Pitch, float Yaw) const;
    void UpdateTracking(float DeltaTime);
    FVector GetTargetPosition() const;

    UPROPERTY()
    bool bIsTracking = false;

    UPROPERTY()
    AActor* TrackingTarget = nullptr;

    UPROPERTY()
    ATurretPawn* OwnerTurret = nullptr;

    static constexpr int32 MaxIterations = 10;
    static constexpr float ConvergenceThreshold = 0.1f;
};

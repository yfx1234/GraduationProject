#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "Components/StaticMeshComponent.h"
#include "Camera/CameraComponent.h"
#include "DroneState.h"
#include "DroneParameters.h"
#include "DronePawn.generated.h"

class UDroneMovementComponent;
class UDroneApi;

/**
 * 四旋翼无人机 Pawn
 * 组件层级：
 *   RootComp (USceneComponent)
 *     └─ BodyMesh 
 *          ├─ Fan0..Fan3 — 四个风扇网格
 *          ├─ FPVCamera — 第一人称视角相机
 *          └─ MovementComp (UDroneMovementComponent)
 */
UCLASS()
class GRADUATIONPROJECT_API ADronePawn : public APawn
{
    GENERATED_BODY()

public:
    ADronePawn();
    virtual void BeginPlay() override;
    virtual void Tick(float DeltaTime) override;

    /**
     * @brief 设置目标位置
     * @param TargetPos 目标位置
     */
    void SetTargetPosition(const FVector& TargetPos);

    /**
     * @brief 设置目标速度
     * @param TargetVel 目标速度
     */
    void SetTargetVelocity(const FVector& TargetVel);

    /** @brief 在当前位置悬停 */
    void Hover();

    /**
     * @brief 起飞到指定高度
     * @param Altitude 目标高度
     */
    UFUNCTION(BlueprintCallable, Category = "Drone")
    void Takeoff(float Altitude = 3.0f);

    /** @brief 降落到地面 */
    UFUNCTION(BlueprintCallable, Category = "Drone")
    void Land();

    /**
     * @brief 重置无人机到指定位置和姿态
     * @param NewLocation 重置位置
     * @param NewRotation 重置姿态
     */
    void ResetDrone(const FVector& NewLocation, const FRotator& NewRotation);

    /** @brief 获取当前位置 */
    FVector GetCurrentPosition() const;

    /** @brief 获取当前速度 */
    FVector GetCurrentVelocity() const;

    /** @brief 当前无人机状态 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|State")
    FDroneState CurrentState;

    /** @brief 当前控制模式 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|State")
    EDroneControlMode ControlMode = EDroneControlMode::Idle;

    /** @brief 无人机物理参数 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters")
    FDroneParameters Parameters;

    /** @brief 运动仿真组件指针 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    UDroneMovementComponent* MovementComp = nullptr;

    /** @brief DroneApi 接口指针 */
    UPROPERTY()
    UDroneApi* Api = nullptr;

    /** @brief 无人机 Agent ID */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Config")
    FString DroneId = TEXT("drone_0");

    /** @brief 根场景组件 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    USceneComponent* RootComp;

    /** @brief 机身网格组件 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    UStaticMeshComponent* BodyMesh;

    /** @brief 风扇0 网格 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    UStaticMeshComponent* Fan0;

    /** @brief 风扇1 网格 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    UStaticMeshComponent* Fan1;

    /** @brief 风扇2 网格 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    UStaticMeshComponent* Fan2;

    /** @brief 风扇3 网格 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    UStaticMeshComponent* Fan3;

    /** @brief 第一人称视角 (FPV) 相机 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    UCameraComponent* FPVCamera;

private:
    /**
     * @brief 将 CurrentState 同步到 UE Actor 的 Transform
     * @param State 要同步的状态
     */
    void ApplyStateToActor(const FDroneState& State);

    /**
     * @brief 更新螺旋桨旋转动画
     * @param DeltaTime 帧间隔（秒）
     * 根据电机转速转换为每帧旋转角度
     */
    void UpdatePropellerAnimation(float DeltaTime);

    /**
     * @brief 根据索引获取对应的风扇网格
     * @param Index 风扇索引
     * @return 对应的 UStaticMeshComponent 指针
     */
    UStaticMeshComponent* GetFanMesh(int32 Index) const;
};

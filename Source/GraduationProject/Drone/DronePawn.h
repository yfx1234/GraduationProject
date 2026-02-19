#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "DroneState.h"
#include "DroneParameters.h"
#include "DronePawn.generated.h"

class UDroneMovementComponent;
class UDroneApi;

/**
 * 无人机 Pawn — 静态网格体组装版
 * 机身 + 4螺旋桨 均为 UStaticMeshComponent
 * 参考 AirSim MultirotorPawnSimApi 的职责划分
 */
UCLASS()
class GRADUATIONPROJECT_API ADronePawn : public APawn
{
    GENERATED_BODY()

public:
    ADronePawn();

    virtual void Tick(float DeltaTime) override;

protected:
    virtual void BeginPlay() override;

public:
    // ---- 组件 ----

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    USceneComponent* RootComp;

    /** 机身静态网格 — Blueprint 中指定 Drone_Body */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    UStaticMeshComponent* BodyMesh;

    /** 4个螺旋桨 — Blueprint 中指定位置和网格 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    UStaticMeshComponent* Fan0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    UStaticMeshComponent* Fan1;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    UStaticMeshComponent* Fan2;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    UStaticMeshComponent* Fan3;

    /** 辅助：按索引获取螺旋桨 */
    UStaticMeshComponent* GetFanMesh(int32 Index) const;

    /** 前视 FPV 相机 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    class UCameraComponent* FPVCamera;

    /** 运动仿真组件 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    UDroneMovementComponent* MovementComp;

    // ---- 参数/状态 ----

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters")
    FDroneParameters Parameters;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|State")
    FDroneState CurrentState;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
    EDroneControlMode ControlMode = EDroneControlMode::Idle;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Identity")
    FString DroneId = TEXT("drone_0");

    // ---- 控制接口（供 DroneApi 调用） ----

    UFUNCTION(BlueprintCallable, Category = "Drone|Control")
    void SetTargetPosition(const FVector& TargetPos);

    UFUNCTION(BlueprintCallable, Category = "Drone|Control")
    void SetTargetVelocity(const FVector& TargetVel);

    UFUNCTION(BlueprintCallable, Category = "Drone|Control")
    void Takeoff(float Altitude = 3.0f);

    UFUNCTION(BlueprintCallable, Category = "Drone|Control")
    void Land();

    UFUNCTION(BlueprintCallable, Category = "Drone|Control")
    void Hover();

    UFUNCTION(BlueprintPure, Category = "Drone|State")
    FVector GetCurrentPosition() const;

    UFUNCTION(BlueprintPure, Category = "Drone|State")
    FVector GetCurrentVelocity() const;

    UFUNCTION(BlueprintCallable, Category = "Drone|Control")
    void ResetDrone(const FVector& NewLocation, const FRotator& NewRotation);

    /** DroneApi 实例 */
    UPROPERTY()
    UDroneApi* Api;

protected:
    /** 将物理状态同步到 Actor Transform */
    void ApplyStateToActor(const FDroneState& State);

    /** 螺旋桨旋转动画 */
    void UpdatePropellerAnimation(float DeltaTime);
};

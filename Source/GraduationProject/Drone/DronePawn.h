/**
 * @file DronePawn.h
 * @brief 无人机 Pawn 的头文件
 *
 * 本文件定义了 ADronePawn 类，表示仿真环境中的四旋翼无人机。
 * ADronePawn 是 UE Actor 层面的无人机表示形式，负责：
 * 1. 管理无人机的 3D 网格组件（机身 + 4个风扇）
 * 2. 管理 DroneMovementComponent（物理仿真和控制）
 * 3. 管理 DroneApi（TCP 控制接口）
 * 4. 管理 FPV 摄像头
 * 5. 将仿真状态同步到 UE Actor 的 Transform
 *
 * 参考 AirSim 的 MultirotorPawnSimApi + SimpleFlightQuadRotorPhysicsBody 的职责分配。
 */

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
 *
 * 组件层级：
 *   RootComp (USceneComponent)
 *     └─ BodyMesh (UStaticMeshComponent, Blueprint 中配置)
 *          ├─ Fan0..Fan3 — 四个风扇网格（挂载到骨骼 Socket）
 *          ├─ FPVCamera — 第一人称视角相机（挂载到 Camera_Yaw_002 Socket）
 *          └─ MovementComp (UDroneMovementComponent) — 物理/控制组件
 *
 * 数据流：
 *   TCP Command → DroneApi → DronePawn → MovementComp (控制+物理)
 *   MovementComp → CurrentState → ApplyStateToActor() → UE Transform
 */
UCLASS()
class GRADUATIONPROJECT_API ADronePawn : public APawn
{
    GENERATED_BODY()

public:
    /** @brief 构造函数，创建机身网格、风扇、相机和运动组件 */
    ADronePawn();

    /**
     * @brief 游戏开始时调用
     *
     * 初始化 MovementComp 参数和初始状态（从 Actor 位置读取），
     * 创建 DroneApi 实例，注册到 AgentManager。
     */
    virtual void BeginPlay() override;

    /**
     * @brief 每帧更新
     * @param DeltaTime 帧间隔时间（秒）
     *
     * 从 MovementComp 读取最新状态 → ApplyStateToActor() 同步到 UE Transform
     * → UpdatePropellerAnimation() 旋转风扇。
     */
    virtual void Tick(float DeltaTime) override;

    // ---- 控制接口（由 DroneApi 调用） ----

    /**
     * @brief 设置目标位置（SI 坐标，米）
     * @param TargetPos 目标位置 (m)
     *
     * 将控制模式切换为 Position（仅在模式变化时切换，
     * 避免重复切换导致 PID 积分器清零），交给 MovementComp 处理。
     */
    void SetTargetPosition(const FVector& TargetPos);

    /**
     * @brief 设置目标速度（SI，m/s）
     * @param TargetVel 目标速度 (m/s)
     *
     * 将控制模式切换为 Velocity，交给 MovementComp 处理。
     */
    void SetTargetVelocity(const FVector& TargetVel);

    /** @brief 在当前位置悬停（设置当前位置为目标位置） */
    void Hover();

    /**
     * @brief 起飞到指定高度
     * @param Altitude 目标高度 (m)，默认 3.0
     *
     * 以当前 XY 位置为基础，设置目标位置为 (X, Y, Altitude)。
     */
    UFUNCTION(BlueprintCallable, Category = "Drone")
    void Takeoff(float Altitude = 3.0f);

    /** @brief 降落到地面（Z=0） */
    UFUNCTION(BlueprintCallable, Category = "Drone")
    void Land();

    /**
     * @brief 重置无人机到指定位置和姿态
     * @param NewLocation 重置位置 (m)
     * @param NewRotation 重置姿态
     *
     * 清空物理状态，切换到 Idle 模式。
     */
    void ResetDrone(const FVector& NewLocation, const FRotator& NewRotation);

    // ---- 状态查询 ----

    /** @brief 获取当前位置 (SI, 米) */
    FVector GetCurrentPosition() const;

    /** @brief 获取当前速度 (SI, m/s) */
    FVector GetCurrentVelocity() const;

    // ---- 公开成员 ----

    /** @brief 当前无人机状态（位置/速度/姿态/电机，全部 SI 单位） */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|State")
    FDroneState CurrentState;

    /** @brief 当前控制模式 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|State")
    EDroneControlMode ControlMode = EDroneControlMode::Idle;

    /** @brief 无人机物理参数（可在 Blueprint 中配置） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters")
    FDroneParameters Parameters;

    /** @brief 运动仿真组件指针 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    UDroneMovementComponent* MovementComp = nullptr;

    /** @brief DroneApi 接口指针 */
    UPROPERTY()
    UDroneApi* Api = nullptr;

    /** @brief 无人机 Agent ID（用于 AgentManager 注册和 TCP 命令路由） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Config")
    FString DroneId = TEXT("drone_0");

    // ---- 组件 ----

    /** @brief 根场景组件 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    USceneComponent* RootComp;

    /** @brief 机身网格组件（Blueprint 中配置 StaticMesh） */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    UStaticMeshComponent* BodyMesh;

    /** @brief 风扇0 网格（挂载到 Drone_Fan_002 Socket） */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    UStaticMeshComponent* Fan0;

    /** @brief 风扇1 网格（挂载到 Drone_Fan1_002 Socket） */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    UStaticMeshComponent* Fan1;

    /** @brief 风扇2 网格（挂载到 Drone_Fan2_002 Socket） */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    UStaticMeshComponent* Fan2;

    /** @brief 风扇3 网格（挂载到 Drone_Fan3_002 Socket） */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    UStaticMeshComponent* Fan3;

    /** @brief 第一人称视角 (FPV) 相机 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    UCameraComponent* FPVCamera;

private:
    /**
     * @brief 将 CurrentState 同步到 UE Actor 的 Transform
     * @param State 要同步的状态
     *
     * SI 坐标（米）× 100 → UE 坐标（厘米），
     * 四元数 → FRotator 并应用到 Actor。
     */
    void ApplyStateToActor(const FDroneState& State);

    /**
     * @brief 更新螺旋桨旋转动画
     * @param DeltaTime 帧间隔（秒）
     *
     * 根据电机转速 (rad/s) 转换为每帧旋转角度，
     * 交替旋向（0/2 顺时针，1/3 逆时针）。
     */
    void UpdatePropellerAnimation(float DeltaTime);

    /**
     * @brief 根据索引获取对应的风扇网格
     * @param Index 风扇索引 (0-3)
     * @return 对应的 UStaticMeshComponent 指针，越界返回 nullptr
     */
    UStaticMeshComponent* GetFanMesh(int32 Index) const;
};

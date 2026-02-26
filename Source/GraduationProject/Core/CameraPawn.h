/**
 * @file CameraPawn.h
 * @brief 自由相机 Pawn 的头文件，提供自由漫游和轨道跟踪两种相机模式
 *
 * 本文件定义了 ACameraPawn 类，它作为默认玩家控制的相机 Pawn，
 * 支持 WASD 自由移动和鼠标旋转漫游，以及锁定目标 Actor 的轨道跟踪模式。
 * 参考旧项目 CameraPawn（已从 Master 移植）。
 */

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/SpringArmComponent.h"
#include "Camera/CameraComponent.h"
#include "CameraPawn.generated.h"

/**
 * 自由相机 Pawn（默认玩家控制）
 *
 * 支持两种操控模式：
 *   1. 自由漫游模式 — WASD 控制移动，鼠标控制旋转视角
 *   2. 轨道跟踪模式 — 锁定目标 Actor，鼠标控制轨道旋转，WS 控制与目标的距离
 *
 * 模式切换通过 StartTracking() / StopTracking() 实现。
 * 参考旧项目 CameraPawn（已从 Master 移植）。
 */
UCLASS()
class GRADUATIONPROJECT_API ACameraPawn : public APawn
{
    GENERATED_BODY()

public:
    /** @brief 构造函数，初始化根组件、弹簧臂和相机组件 */
    ACameraPawn();

    /**
     * @brief 游戏开始时调用，初始化鼠标输入设置
     */
    virtual void BeginPlay() override;

    /**
     * @brief 每帧更新，根据当前模式处理相机移动和旋转
     * @param DeltaTime 帧间隔时间（秒）
     */
    virtual void Tick(float DeltaTime) override;

    /**
     * @brief 绑定玩家输入到相机操控函数
     * @param PlayerInputComponent 玩家输入组件
     */
    virtual void SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent) override;

    /**
     * @brief 开始跟踪指定 Actor，切换到轨道跟踪模式
     * @param Target 要跟踪的目标 Actor 指针，为 nullptr 时不做任何操作
     */
    UFUNCTION(BlueprintCallable, Category = "Camera")
    void StartTracking(AActor* Target);

    /**
     * @brief 停止跟踪，回到自由漫游模式
     */
    UFUNCTION(BlueprintCallable, Category = "Camera")
    void StopTracking();

    /**
     * @brief UI 列表点击回调，根据是否有 Actor 决定开始/停止跟踪
     * @param AgentId 被点击的智能体 ID
     * @param Actor 对应的 Actor 指针，为 nullptr 时停止跟踪
     */
    void OnItemClicked(const FString& AgentId, AActor* Actor);

protected:
    // ---- 输入处理函数 ----

    /**
     * @brief 前后移动输入处理
     * @param Value 输入轴值，正值为前进，负值为后退
     */
    void MoveForward(float Value);

    /**
     * @brief 左右移动输入处理
     * @param Value 输入轴值，正值为右移，负值为左移
     */
    void MoveRight(float Value);

    /**
     * @brief 上下移动输入处理
     * @param Value 输入轴值，正值为上升，负值为下降
     */
    void MoveUp(float Value);

    /**
     * @brief 相机俯仰（Pitch）输入处理
     * @param Value 鼠标Y轴偏移值
     *
     * 自由模式下旋转 Pawn 的 Pitch；轨道模式下调整轨道 Pitch 角度。
     * Pitch 范围限制在 [-89°, 89°] 以防止万向锁。
     */
    void CameraPitch(float Value);

    /**
     * @brief 相机偏航（Yaw）输入处理
     * @param Value 鼠标X轴偏移值
     *
     * 自由模式下旋转 Pawn 的 Yaw；轨道模式下调整轨道 Yaw 角度。
     */
    void CameraYaw(float Value);

    /**
     * @brief 缩放拉近回调（轨道模式下减小与目标的距离）
     */
    void OnZoomIn();

    /**
     * @brief 缩放拉远回调（轨道模式下增大与目标的距离）
     */
    void OnZoomOut();

    /**
     * @brief 更新轨道跟踪模式下的相机位置和朝向
     * @param DeltaTime 帧间隔时间（秒）
     *
     * 根据 OrbitPitch/OrbitYaw/TrackingDistance 计算相机的轨道位置，
     * 然后朝向目标 Actor。
     */
    void UpdateTrackingCamera(float DeltaTime);

protected:
    /** @brief 根场景组件 */
    UPROPERTY(VisibleAnywhere, Category = "Camera")
    USceneComponent* RootComp;

    /** @brief 弹簧臂组件（自由模式下长度为0，轨道模式下用于调整距离） */
    UPROPERTY(VisibleAnywhere, Category = "Camera")
    USpringArmComponent* SpringArm;

    /** @brief 相机组件，挂载在弹簧臂末端 */
    UPROPERTY(VisibleAnywhere, Category = "Camera")
    UCameraComponent* Camera;

    /** @brief 自由模式移动速度，单位 cm/s */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Settings")
    float MoveSpeed = 600.0f;

    /** @brief 鼠标灵敏度系数，影响旋转速度 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Settings")
    float MouseSensitivity = 1.0f;

    /** @brief 缩放速度，每次缩放操作改变的距离值 (cm) */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Settings")
    float ZoomSpeed = 50.0f;

    /** @brief 轨道跟踪模式下相机与目标的距离 (cm) */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Settings")
    float TrackingDistance = 500.0f;

    /** @brief 轨道最小跟踪距离 (cm) */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Settings")
    float MinTrackingDistance = 100.0f;

    /** @brief 轨道最大跟踪距离 (cm) */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Settings")
    float MaxTrackingDistance = 3000.0f;

private:
    /** @brief 是否处于轨道跟踪模式 */
    bool bIsTracking = false;

    /** @brief 当前跟踪的目标 Actor */
    AActor* TrackingTarget = nullptr;

    /** @brief 轨道偏航角（度） */
    float OrbitYaw = 0.0f;

    /** @brief 轨道俯仰角（度），负值表示从上方俯视 */
    float OrbitPitch = -30.0f;

    /** @brief 当前帧的移动输入方向缓冲（X=前后, Y=左右, Z=上下） */
    FVector InputMoveDirection = FVector::ZeroVector;

    /** @brief 当前帧的俯仰输入值缓冲 */
    float InputPitchValue = 0.0f;

    /** @brief 当前帧的偏航输入值缓冲 */
    float InputYawValue = 0.0f;
};

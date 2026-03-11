#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/SpringArmComponent.h"
#include "Camera/CameraComponent.h"
#include "CameraPawn.generated.h"

class ADronePawn;
class ASimHUD;

/** @brief 无人机观察模式循环状态 */
UENUM()
enum class EDroneViewCycleMode : uint8
{
    Chase = 0,
    TopDown,
    FPV
};

/**
 * @brief 场景自由相机 Pawn
 * 支持自由移动、目标跟随、无人机追踪视角切换，以及与 HUD 中 PIP/Agent List 联动。
 */
UCLASS()
class GRADUATIONPROJECT_API ACameraPawn : public APawn
{
    GENERATED_BODY()

public:
    /** @brief 构造相机 Pawn，创建根节点、弹簧臂和相机组件 */
    ACameraPawn();

    /** @brief 初始化输入模式和鼠标状态 */
    virtual void BeginPlay() override;

    /** @brief 每帧更新自由相机或跟踪相机状态 */
    virtual void Tick(float DeltaTime) override;

    /** @brief 绑定移动、视角和快捷键输入 */
    virtual void SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent) override;

    /**
     * @brief 开始跟踪指定目标
     * @param Target 需要被观察的 Actor
     */
    UFUNCTION(BlueprintCallable, Category = "Camera")
    void StartTracking(AActor* Target);

    /** @brief 停止跟踪并回到自由视角 */
    UFUNCTION(BlueprintCallable, Category = "Camera")
    void StopTracking();

    /**
     * @brief 响应 Agent 列表点击事件
     * @param AgentId 被点击的智能体 ID
     * @param Actor 对应的 Actor 实例
     */
    UFUNCTION(BlueprintCallable, Category = "Camera")
    void OnItemClicked(const FString& AgentId, AActor* Actor);

protected:
    /** @brief 处理前后移动输入 */
    void MoveForward(float Value);

    /** @brief 处理左右平移输入 */
    void MoveRight(float Value);

    /** @brief 处理上下移动输入 */
    void MoveUp(float Value);

    /** @brief 处理俯仰输入 */
    void CameraPitch(float Value);

    /** @brief 处理偏航输入 */
    void CameraYaw(float Value);

    /** @brief 缩小跟踪距离或俯视距离 */
    void OnZoomIn();

    /** @brief 放大跟踪距离或俯视距离 */
    void OnZoomOut();

    /** @brief 切换 PIP 1 显示状态 */
    void OnTogglePip1();

    /** @brief 切换 PIP 2 显示状态 */
    void OnTogglePip2();

    /** @brief 切换 PIP 3 显示状态 */
    void OnTogglePip3();

    /** @brief 在追踪/俯视/FPV 视角之间循环切换 */
    void OnCycleDroneView();

    /** @brief 强制切回自由视角 */
    void OnSwitchFreeView();

    /** @brief 更新绕目标的追踪相机轨道位置 */
    void UpdateTrackingCamera(float DeltaTime);

    /** @brief 更新无人机俯视观察视角 */
    void UpdateTopDownCamera();

    /** @brief 更新无人机第一人称视角 */
    void UpdateFPVCamera();

    /**
     * @brief 解析当前激活的无人机
     * @param bAutoSelect 为真时允许自动选择场景中可用无人机
     * @return 当前用于视角切换的无人机实例
     */
    ADronePawn* ResolveActiveDrone(bool bAutoSelect = true);

    /** @brief 获取当前 HUD 实例，用于联动 PIP 状态 */
    ASimHUD* ResolveSimHUD() const;

protected:
    /** @brief 根场景组件 */
    UPROPERTY(VisibleAnywhere, Category = "Camera")
    USceneComponent* RootComp;

    /** @brief 弹簧臂组件，负责相机偏移与轨道控制 */
    UPROPERTY(VisibleAnywhere, Category = "Camera")
    USpringArmComponent* SpringArm;

    /** @brief 主观察相机 */
    UPROPERTY(VisibleAnywhere, Category = "Camera")
    UCameraComponent* Camera;

    /** @brief 自由移动速度 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Settings")
    float MoveSpeed = 600.0f;

    /** @brief 鼠标灵敏度 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Settings")
    float MouseSensitivity = 1.0f;

    /** @brief 缩放步长 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Settings")
    float ZoomSpeed = 50.0f;

    /** @brief 默认跟踪距离 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Settings")
    float TrackingDistance = 500.0f;

    /** @brief 跟踪距离下限 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Settings")
    float MinTrackingDistance = 100.0f;

    /** @brief 跟踪距离上限 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Settings")
    float MaxTrackingDistance = 3000.0f;

private:
    /** @brief 当前是否处于目标跟踪模式 */
    bool bIsTracking = false;

    /** @brief 当前跟踪目标 */
    AActor* TrackingTarget = nullptr;

    /** @brief 绕目标的水平轨道角 */
    float OrbitYaw = 0.0f;

    /** @brief 绕目标的俯仰轨道角 */
    float OrbitPitch = -30.0f;

    /** @brief 累计的平移输入方向 */
    FVector InputMoveDirection = FVector::ZeroVector;

    /** @brief 累计的俯仰输入 */
    float InputPitchValue = 0.0f;

    /** @brief 累计的偏航输入 */
    float InputYawValue = 0.0f;

    /** @brief 当前激活的无人机 ID */
    FString ActiveDroneId;

    /** @brief 当前无人机观察模式 */
    EDroneViewCycleMode DroneViewMode = EDroneViewCycleMode::Chase;
};

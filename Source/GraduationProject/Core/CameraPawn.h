#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/SpringArmComponent.h"
#include "Camera/CameraComponent.h"
#include "CameraPawn.generated.h"

UCLASS()
class GRADUATIONPROJECT_API ACameraPawn : public APawn
{
    GENERATED_BODY()

public:
    /** @brief 构造函数，初始化根组件、弹簧臂和相机组件 */
    ACameraPawn();

    /** @brief 游戏开始时调用，初始化鼠标输入设置 */
    virtual void BeginPlay() override;

    /** @brief 每帧更新，根据当前模式处理相机移动和旋转 */
    virtual void Tick(float DeltaTime) override;

    /**
     * @brief 绑定玩家输入到相机操控函数
     * @param PlayerInputComponent 玩家输入组件
     */
    virtual void SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent) override;

    /**
     * @brief 开始跟踪指定 Actor，切换到轨道跟踪模式
     * @param Target 要跟踪的目标 Actor 指针
     */
    UFUNCTION(BlueprintCallable, Category = "Camera")
    void StartTracking(AActor* Target);

    /** @brief 停止跟踪，回到自由模式 */
    UFUNCTION(BlueprintCallable, Category = "Camera")
    void StopTracking();

    /**
     * @brief UI 列表点击回调，根据是否有 Actor 决定开始/停止跟踪
     * @param AgentId 被点击的智能体 ID
     * @param Actor 对应的 Actor 指针，为 nullptr 时停止跟踪
     */
    void OnItemClicked(const FString& AgentId, AActor* Actor);

protected:
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
     * @brief 相机俯仰输入处理
     * @param Value 鼠标Y轴偏移值
     */
    void CameraPitch(float Value);

    /**
     * @brief 相机偏航输入处理
     * @param Value 鼠标X轴偏移值
     */
    void CameraYaw(float Value);

    /** @brief 缩放拉近回调 */
    void OnZoomIn();

    /** @brief 缩放拉远回调 */
    void OnZoomOut();

    /**
     * @brief 更新轨道跟踪模式下的相机位置和朝向
     * @param DeltaTime 帧间隔时间（秒）
     */
    void UpdateTrackingCamera(float DeltaTime);

protected:
    /** @brief 根场景组件 */
    UPROPERTY(VisibleAnywhere, Category = "Camera")
    USceneComponent* RootComp;

    /** @brief 弹簧臂组件 */
    UPROPERTY(VisibleAnywhere, Category = "Camera")
    USpringArmComponent* SpringArm;

    /** @brief 相机组件 */
    UPROPERTY(VisibleAnywhere, Category = "Camera")
    UCameraComponent* Camera;

    /** @brief 自由模式移动速度 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Settings")
    float MoveSpeed = 600.0f;

    /** @brief 鼠标灵敏度系数 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Settings")
    float MouseSensitivity = 1.0f;

    /** @brief 缩放速度 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Settings")
    float ZoomSpeed = 50.0f;

    /** @brief 轨道跟踪模式下相机与目标的距离 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Settings")
    float TrackingDistance = 500.0f;

    /** @brief 轨道最小跟踪距离 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Settings")
    float MinTrackingDistance = 100.0f;

    /** @brief 轨道最大跟踪距离 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Settings")
    float MaxTrackingDistance = 3000.0f;

private:
    /** @brief 是否处于轨道跟踪模式 */
    bool bIsTracking = false;

    /** @brief 当前跟踪的目标 Actor */
    AActor* TrackingTarget = nullptr;

    /** @brief 轨道偏航角（度） */
    float OrbitYaw = 0.0f;

    /** @brief 轨道俯仰角（度） */
    float OrbitPitch = -30.0f;

    /** @brief 当前帧的移动输入方向缓冲 */
    FVector InputMoveDirection = FVector::ZeroVector;

    /** @brief 当前帧的俯仰输入值缓冲 */
    float InputPitchValue = 0.0f;

    /** @brief 当前帧的偏航输入值缓冲 */
    float InputYawValue = 0.0f;
};

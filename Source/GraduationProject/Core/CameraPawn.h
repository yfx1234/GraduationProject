#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/SpringArmComponent.h"
#include "Camera/CameraComponent.h"
#include "CameraPawn.generated.h"

/**
 * 自由相机 Pawn（默认玩家控制）
 * 支持两种模式：
 *   1. 自由漫游 — WASD移动，鼠标旋转
 *   2. 轨道跟踪 — 锁定目标Actor，鼠标轨道旋转，WS控制距离
 * 参考旧项目CameraPawn（已从Master移植）
 */
UCLASS()
class GRADUATIONPROJECT_API ACameraPawn : public APawn
{
    GENERATED_BODY()

public:
    ACameraPawn();

    virtual void BeginPlay() override;
    virtual void Tick(float DeltaTime) override;
    virtual void SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent) override;

    /** 开始跟踪指定Actor */
    UFUNCTION(BlueprintCallable, Category = "Camera")
    void StartTracking(AActor* Target);

    /** 停止跟踪，回到自由模式 */
    UFUNCTION(BlueprintCallable, Category = "Camera")
    void StopTracking();

    /** UI列表点击回调 */
    void OnItemClicked(const FString& AgentId, AActor* Actor);

protected:
    // 输入处理
    void MoveForward(float Value);
    void MoveRight(float Value);
    void MoveUp(float Value);
    void CameraPitch(float Value);
    void CameraYaw(float Value);
    void OnZoomIn();
    void OnZoomOut();

    // 更新跟踪模式下的相机位置
    void UpdateTrackingCamera(float DeltaTime);

protected:
    UPROPERTY(VisibleAnywhere, Category = "Camera")
    USceneComponent* RootComp;

    UPROPERTY(VisibleAnywhere, Category = "Camera")
    USpringArmComponent* SpringArm;

    UPROPERTY(VisibleAnywhere, Category = "Camera")
    UCameraComponent* Camera;

    /** 自由模式移动速度 (cm/s) */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Settings")
    float MoveSpeed = 600.0f;

    /** 鼠标灵敏度 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Settings")
    float MouseSensitivity = 1.0f;

    /** 缩放速度 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Settings")
    float ZoomSpeed = 50.0f;

    /** 跟踪模式下的轨道距离 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Settings")
    float TrackingDistance = 500.0f;

    /** 跟踪最小距离 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Settings")
    float MinTrackingDistance = 100.0f;

    /** 跟踪最大距离 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Settings")
    float MaxTrackingDistance = 3000.0f;

private:
    bool bIsTracking = false;
    AActor* TrackingTarget = nullptr;

    // 轨道旋转角度
    float OrbitYaw = 0.0f;
    float OrbitPitch = -30.0f;

    // 输入缓冲
    FVector InputMoveDirection = FVector::ZeroVector;
    float InputPitchValue = 0.0f;
    float InputYawValue = 0.0f;
};

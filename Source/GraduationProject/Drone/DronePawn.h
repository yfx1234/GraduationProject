#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "Components/StaticMeshComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "CineCameraComponent.h"
#include "DroneState.h"
#include "DroneParameters.h"
#include "DronePawn.generated.h"

class UDroneMovementComponent;
class UDroneApi;

UENUM(BlueprintType)
enum class EDroneMissionRole : uint8
{
    Unknown     UMETA(DisplayName = "Unknown"),
    Target      UMETA(DisplayName = "Target"),
    Interceptor UMETA(DisplayName = "Interceptor")
};

/**
 * @brief 无人机 Pawn
 * 集成飞行控制、运动学状态、相机云台和图像采集能力。
 * 主要组件层级如下：
 * `RootComp -> BodyMesh -> Fan0..Fan3`
 * `BodyMesh -> CameraYawMesh -> CameraPitchMesh -> DroneSceneCapture / DroneCineCamera`
 * `MovementComp` 负责飞行控制与状态积分。
 */
UCLASS()
class GRADUATIONPROJECT_API ADronePawn : public APawn
{
    GENERATED_BODY()

public:
    /** @brief 构造无人机 Pawn 并创建各组件 */
    ADronePawn();

    /** @brief 初始化飞控、相机和 Agent 注册 */
    virtual void BeginPlay() override;

    /** @brief 每帧同步状态、旋翼动画和相机姿态 */
    virtual void Tick(float DeltaTime) override;

    /**
     * @brief 设置位置控制目标
     * @param TargetPos 目标位置（米）
     * @param Speed 期望飞行速度；为 0 时使用默认逻辑
     */
    UFUNCTION(BlueprintCallable, Category = "DroneControl")
    void SetTargetPosition(const FVector& TargetPos, float Speed = 0.0f);

    /**
     * @brief 设置速度控制目标
     * @param TargetVel 目标速度（米/秒）
     */
    UFUNCTION(BlueprintCallable, Category = "DroneControl")
    void SetTargetVelocity(const FVector& TargetVel);

    /**
     * @brief 设置航向控制模式
     * @param YawMode 偏航控制模式
     * @param Drivetrain 运动学约束模式
     * @param YawDeg 目标偏航角；仅在角度模式下生效
     */
    UFUNCTION(BlueprintCallable, Category = "DroneControl")
    void SetHeadingControl(EDroneYawMode YawMode, EDroneDrivetrainMode Drivetrain, float YawDeg = 0.0f);

    /** @brief 在当前位置悬停 */
    UFUNCTION(BlueprintCallable, Category = "DroneControl")
    void Hover();

    /**
     * @brief 以速度模式移动
     * @param Vx X 方向速度（米/秒）
     * @param Vy Y 方向速度（米/秒）
     * @param Vz Z 方向速度（米/秒）
     */
    UFUNCTION(BlueprintCallable, Category = "DroneControl")
    void MoveByVelocity(float Vx, float Vy, float Vz);

    /**
     * @brief 起飞到指定高度
     * @param Altitude 目标高度（米）
     */
    UFUNCTION(BlueprintCallable, Category = "Drone")
    void Takeoff(float Altitude = 3.0f);

    /** @brief 执行降落命令 */
    UFUNCTION(BlueprintCallable, Category = "Drone")
    void Land();

    /**
     * @brief 重置无人机状态
     * @param NewLocation 新位置（米）
     * @param NewRotation 新姿态
     */
    UFUNCTION(BlueprintCallable, Category = "DroneControl")
    void ResetDrone(const FVector& NewLocation, const FRotator& NewRotation);

    /** @brief 获取当前位置（米） */
    FVector GetCurrentPosition() const;

    /** @brief 获取当前速度（米/秒） */
    FVector GetCurrentVelocity() const;

    /**
     * @brief 设置云台目标角度
     * @param TargetPitch 目标俯仰角（度）
     * @param TargetYaw 目标偏航角（度）
     */
    UFUNCTION(BlueprintCallable, Category = "Drone|Camera")
    void SetCameraAngles(float TargetPitch, float TargetYaw);

    /**
     * @brief 捕获当前相机画面并返回 Base64 JPEG
     * @param Quality JPEG 压缩质量；小于等于 0 时使用 `JpegQuality`
     * @return Base64 编码后的 JPEG 字符串
     */
    FString CaptureImageBase64(int32 Quality = -1);

    /** @brief 获取当前云台俯仰角 */
    float GetCameraCurrentPitch() const { return CameraCurrentPitch; }

    /** @brief 获取当前云台偏航角 */
    float GetCameraCurrentYaw() const { return CameraCurrentYaw; }

    /** @brief 当前无人机状态 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|State")
    FDroneState CurrentState;

    /** @brief 当前控制模式 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|State")
    EDroneControlMode ControlMode = EDroneControlMode::Idle;

    /** @brief 飞行参数配置 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters")
    FDroneParameters Parameters;

    /** @brief 飞行运动组件 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    UDroneMovementComponent* MovementComp = nullptr;

    /** @brief 对外控制 API 包装对象 */
    UPROPERTY()
    UDroneApi* Api = nullptr;

    /** @brief 无人机 Agent ID */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Config")
    FString DroneId = TEXT("drone_0");

    /** @brief 无人机任务角色 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Config")
    EDroneMissionRole MissionRole = EDroneMissionRole::Unknown;

    /** @brief 根场景组件 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    USceneComponent* RootComp;

    /** @brief 机体网格组件 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    UStaticMeshComponent* BodyMesh;

    /** @brief 旋翼 0 网格组件 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    UStaticMeshComponent* Fan0;

    /** @brief 旋翼 1 网格组件 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    UStaticMeshComponent* Fan1;

    /** @brief 旋翼 2 网格组件 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    UStaticMeshComponent* Fan2;

    /** @brief 旋翼 3 网格组件 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    UStaticMeshComponent* Fan3;

    /** @brief 云台偏航轴网格组件 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    UStaticMeshComponent* CameraYawMesh;

    /** @brief 云台俯仰轴网格组件 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    UStaticMeshComponent* CameraPitchMesh;

    /** @brief 场景采集组件，用于导出图像 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Camera")
    USceneCaptureComponent2D* DroneSceneCapture;

    /** @brief CineCamera，用于继承 PostProcess 设置 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Camera")
    UCineCameraComponent* DroneCineCamera;

    /** @brief 相机视场角 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera")
    float CameraFOV = 90.0f;

    /** @brief 相机输出宽度 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera")
    int32 CameraWidth = 1280;

    /** @brief 相机输出高度 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera")
    int32 CameraHeight = 720;

    /** @brief 分割图像使用的 Stencil ID */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera", meta = (ClampMin = "0", ClampMax = "255"))
    int32 SegmentationId = 20;

    /** @brief 自动曝光补偿 EV */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera", meta = (ClampMin = "-10.0", ClampMax = "10.0"))
    float ExposureBias = 0.0f;

    /** @brief 默认 JPEG 压缩质量 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera", meta = (ClampMin = "1", ClampMax = "100"))
    int32 JpegQuality = 90;

    /** @brief 云台角度插值速度 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera")
    float CameraRotationSpeed = 8.0f;

private:
    /**
     * @brief 将飞控状态同步到 Unreal Actor
     * @param State 需要应用的无人机状态
     */
    void ApplyStateToActor(const FDroneState& State);

    /**
     * @brief 更新旋翼动画
     * @param DeltaTime 帧间隔（秒）
     * 根据电机角速度换算为每帧旋转角度。
     */
    void UpdatePropellerAnimation(float DeltaTime);

    /**
     * @brief 获取指定索引的旋翼网格组件
     * @param Index 旋翼索引（0-3）
     * @return 对应的旋翼网格组件
     */
    UStaticMeshComponent* GetFanMesh(int32 Index) const;

    /** @brief 更新云台俯仰和偏航插值 */
    void UpdateCameraRotation(float DeltaTime);

    /** @brief 将 `DroneCineCamera` 的后处理设置同步到 `SceneCapture` */
    void SyncPostProcessToCapture();

    /** @brief 将无人机可见网格写入 CustomDepth/Stencil，供分割图像使用 */
    void ApplySegmentationStencil();

    /** @brief 目标云台俯仰角 */
    float CameraTargetPitch = 0.0f;

    /** @brief 目标云台偏航角 */
    float CameraTargetYaw = 0.0f;

    /** @brief 当前云台俯仰角 */
    float CameraCurrentPitch = 0.0f;

    /** @brief 当前云台偏航角 */
    float CameraCurrentYaw = 0.0f;
};

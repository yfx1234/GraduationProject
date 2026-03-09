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

/**
 * 四旋翼无人机 Pawn
 * 组件层级：
 *   RootComp (USceneComponent)
 *     └─ BodyMesh
 *          ├─ Fan0..Fan3 — 四个风扇网格（通过插槽附着）
 *          ├─ CameraYawMesh — 摄像头 Yaw 云台（Camera_Yaw_002 插槽）
 *          │    └─ CameraPitchMesh — 摄像头 Pitch 云台（Camera_Pitch_002 插槽）
 *          │         ├─ DroneSceneCapture — 场景采集（图像传输用）
 *          │         └─ DroneCineCamera — CineCamera（PostProcess 同步用）
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

    // ── 摄像头控制 ──────────────────────────────────

    /**
     * @brief 设置摄像头云台目标角度
     * @param TargetPitch 目标俯仰角
     * @param TargetYaw 目标偏航角
     */
    UFUNCTION(BlueprintCallable, Category = "Drone|Camera")
    void SetCameraAngles(float TargetPitch, float TargetYaw);

    /**
     * @brief 采集一帧图像并返回 Base64 编码的 JPEG 字符串
     * @param Quality JPEG 压缩质量（<=0 则使用 JpegQuality 默认值）
     * @return Base64 编码的 JPEG 图像数据
     */
    FString CaptureImageBase64(int32 Quality = -1);

    /** @brief 获取摄像头当前俯仰角 */
    float GetCameraCurrentPitch() const { return CameraCurrentPitch; }

    /** @brief 获取摄像头当前偏航角 */
    float GetCameraCurrentYaw() const { return CameraCurrentYaw; }

    // ── 状态与参数 ──────────────────────────────────

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

    // ── 组件 ────────────────────────────────────────

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

    /** @brief 摄像头 Yaw 云台网格（附着到 BodyMesh 的 Camera_Yaw_002 插槽） */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    UStaticMeshComponent* CameraYawMesh;

    /** @brief 摄像头 Pitch 云台网格（附着到 CameraYawMesh 的 Camera_Pitch_002 插槽） */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    UStaticMeshComponent* CameraPitchMesh;

    /** @brief 场景采集组件，采集摄像头视角画面用于图像传输 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Camera")
    USceneCaptureComponent2D* DroneSceneCapture;

    /** @brief CineCamera — 用于继承场景 PostProcessVolume 设置，再同步到 SceneCapture */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Camera")
    UCineCameraComponent* DroneCineCamera;

    // ── 摄像头参数 ──────────────────────────────────

    /** @brief 摄像头视场角 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera")
    float CameraFOV = 90.0f;

    /** @brief 摄像头采集分辨率宽度 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera")
    int32 CameraWidth = 1280;

    /** @brief 摄像头采集分辨率高度 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera")
    int32 CameraHeight = 720;

    /** @brief 曝光补偿 EV */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera", meta = (ClampMin = "-10.0", ClampMax = "10.0"))
    float ExposureBias = 0.0f;

    /** @brief JPEG 压缩质量 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera", meta = (ClampMin = "1", ClampMax = "100"))
    int32 JpegQuality = 90;

    /** @brief 摄像头旋转插值速度 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera")
    float CameraRotationSpeed = 8.0f;

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

    /** @brief 更新摄像头云台旋转插值 */
    void UpdateCameraRotation(float DeltaTime);

    /** @brief 将 CineCamera 的 PostProcess 设置同步到 SceneCapture */
    void SyncPostProcessToCapture();

    /** @brief 摄像头目标俯仰角 */
    float CameraTargetPitch = 0.0f;

    /** @brief 摄像头目标偏航角 */
    float CameraTargetYaw = 0.0f;

    /** @brief 摄像头当前实际俯仰角 */
    float CameraCurrentPitch = 0.0f;

    /** @brief 摄像头当前实际偏航角 */
    float CameraCurrentYaw = 0.0f;
};

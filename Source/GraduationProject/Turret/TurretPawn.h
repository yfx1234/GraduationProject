/**
 * 转台旋转、跟踪、弹道计算、开火、预测线、图像采集
 * 组件层级：
 *   CollisionComponent 
 *     └─ BaseMesh
 *          └─ GimbalMesh 
 *               └─ GunMesh 
 *                    ├─ TurretSceneCapture 
 *                    └─ TurretCineCamera
 * 数据流：
 *   TCP Command → TurretCommandHandler → TurretPawn
 *   TurretAiming → CalculateAimAngles() → SetTargetAngles() → Tick()
 *   FireX() → Ballistic() → BulletActor
 */

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "Components/StaticMeshComponent.h"
#include "Components/SphereComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "CineCameraComponent.h"
#include "TurretPawn.generated.h"

class ABulletActor;
class UTurretAiming;

UCLASS()
class GRADUATIONPROJECT_API ATurretPawn : public APawn
{
    GENERATED_BODY()

public:
    /** @brief 构造函数，创建各组件 */
    ATurretPawn();

protected:
    /**
     * @brief 初始化当前角度为零，注册到 AgentManager
     * 创建 RenderTarget 并配置 SceneCaptureComponent2D
     */
    virtual void BeginPlay() override;

public:
    /**
     * @brief 每帧更新
     * @param DeltaTime 帧间隔时间（秒）
     */
    virtual void Tick(float DeltaTime) override;

    /**
     * @brief 设置转台目标角度
     * @param TargetPitch 目标俯仰角
     * @param TargetYaw 目标偏航角
     */
    UFUNCTION(BlueprintCallable, Category = "Turret Control")
    void SetTargetAngles(float TargetPitch, float TargetYaw);

    /**
     * @brief 发射弹丸
     * @param InitialSpeed 弹丸初始速度
     */
    UFUNCTION(BlueprintCallable, Category = "Turret Control")
    void FireX(float InitialSpeed = 400.0f);

    /**
     * @brief 开始自动跟踪目标
     * @param TargetActorID 目标 Agent ID
     */
    UFUNCTION(BlueprintCallable, Category = "Turret Control")
    void StartTracking(const FString& TargetActorID);

    /** @brief 停止自动跟踪 */
    UFUNCTION(BlueprintCallable, Category = "Turret Control")
    void StopTracking();

    /** @brief 显示预测弹道线 */
    UFUNCTION(BlueprintCallable, Category = "Turret Control")
    void ShowPredictionLine();

    /** @brief 隐藏预测弹道线 */
    UFUNCTION(BlueprintCallable, Category = "Turret Control")
    void HidePredictionLine();

    /** @brief 获取当前俯仰角 */
    float GetCurrentPitch() const { return CurrentPitch; }

    /** @brief 获取当前偏航角 */
    float GetCurrentYaw() const { return CurrentYaw; }

    /** @brief 获取目标俯仰角 */
    float GetTargetPitchAngle() const { return TargetPitch; }

    /** @brief 获取目标偏航角 */
    float GetTargetYawAngle() const { return TargetYaw; }

    /** @brief 查询是否正在跟踪目标 */
    bool IsTracking() const;

    /**
     * @brief 使用 BC 弹道库计算弹道轨迹
     * @param StartPos 发射起点
     * @param ShootDir 发射方向旋转值
     * @param InitialSpeed 弹丸初始速度
     * @param OutTotalTime 弹道总飞行时间
     * @return 弹道轨迹点数组
     */
    TArray<FVector> Ballistic(FVector StartPos, FRotator ShootDir, float InitialSpeed, float& OutTotalTime);

    /** @brief 根碰撞球组件 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components")
    USphereComponent* CollisionComponent;

    /** @brief 底座网格组件 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components")
    UStaticMeshComponent* BaseMesh;

    /** @brief 云台网格组件 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components")
    UStaticMeshComponent* GimbalMesh;

    /** @brief 枪管网格组件 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components")
    UStaticMeshComponent* GunMesh;

    /** @brief 瞄准/跟踪组件 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components")
    UTurretAiming* AimingComponent;

    /** @brief 场景采集组件，向枪口方向采集画面 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Camera")
    USceneCaptureComponent2D* TurretSceneCapture;

    /** @brief CineCamera — 用于继承场景 PostProcessVolume 设置，再同步到 SceneCapture */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Camera")
    UCineCameraComponent* TurretCineCamera;

    /**
     * @brief 采集一帧图像并返回 Base64 编码的 JPEG 字符串
     * @param Quality JPEG 压缩质量
     * @return Base64 编码的 JPEG 图像数据
     */
    FString CaptureImageBase64(int32 Quality = -1);

    /** @brief 转台 Agent ID */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config")
    FString TurretId = TEXT("turret_0");

    /** @brief 枪口相对于 GunMesh 原点的偏移量 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config")
    FVector MuzzleOffset = FVector(-249.0f, -84.0f, 2.0f);
    

    /** @brief 弹丸 Actor 类 */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Config")
    TSubclassOf<ABulletActor> BulletClass;

    /** @brief 默认弹丸出膛速度 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config")
    float DefaultMuzzleSpeed = 400.0f;

    /** @brief 转台旋转插值速度 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config")
    float RotationSpeed = 8.0f;

    /** @brief 摄像头视场角 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera")
    float CameraFOV = 90.0f;

    /** @brief 摄像头采集分辨率宽度 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera")
    int32 CameraWidth = 1280;

    /** @brief 摄像头采集分辨率高度 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera")
    int32 CameraHeight = 720;

    /** @brief 语义分割 ID（0-255，对齐 AirSim segmentation 逻辑） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera", meta = (ClampMin = "0", ClampMax = "255"))
    int32 SegmentationId = 60;

    /** @brief 曝光补偿 EV */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera", meta = (ClampMin = "-10.0", ClampMax = "10.0"))
    float ExposureBias = 0.0f;

    /** @brief JPEG 压缩质量 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera", meta = (ClampMin = "1", ClampMax = "100"))
    int32 JpegQuality = 90;

    /** @brief 是否正在显示预测弹道线 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Prediction")
    bool bShowPredictionLine = false;

    /** @brief 预测弹道线颜色 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Prediction")
    FColor PredictionLineColor = FColor::Cyan;

    /** @brief 预测弹道线宽度 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Prediction")
    float PredictionLineThickness = 2.0f;

    /** @brief 预测命中点标记球半径 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Prediction")
    float PredictionHitRadius = 15.0f;

    /** @brief 预测命中点标记球颜色 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Prediction")
    FColor PredictionHitColor = FColor::Yellow;

private:
    /** @brief 创建底座/云台/枪管网格组件并设置层级关系 */
    void SetupTurretMesh();

    /** @brief 绘制预测弹道线和命中点 */
    void DrawPredictionLine();

    /** @brief 将 CineCamera 的 PostProcess 设置同步到 SceneCapture */
    void SyncPostProcessToCapture();

    /** @brief 将该转台可见网格写入 CustomDepth/Stencil，用于 Segmentation 图像 */
    void ApplySegmentationStencil();

    /** @brief 目标俯仰角 */
    float TargetPitch = 0.0f;

    /** @brief 目标偏航角 */
    float TargetYaw = 0.0f;

    /** @brief 当前实际俯仰角 */
    float CurrentPitch = 0.0f;

    /** @brief 当前实际偏航角 */
    float CurrentYaw = 0.0f;
};

/**
 * @brief 炮台 Pawn
 * 集成炮台旋转、目标跟踪、弹道计算、开火、预测轨迹和图像采集能力。
 * 主要组件层级如下：
 * `CollisionComponent -> BaseMesh -> GimbalMesh -> GunMesh`
 * `GunMesh -> TurretSceneCapture / TurretCineCamera`
 * 典型调用链为：`TurretCommandHandler -> TurretPawn -> TurretAiming/Ballistic/BulletActor`。
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
    /** @brief 构造炮台 Pawn 并创建主要组件 */
    ATurretPawn();

protected:
    /**
     * @brief 初始化炮台角度、注册 Agent，并创建相机 RenderTarget
     */
    virtual void BeginPlay() override;

public:
    /**
     * @brief 每帧更新炮台姿态和预测轨迹显示
     * @param DeltaTime 帧间隔（秒）
     */
    virtual void Tick(float DeltaTime) override;

    /**
     * @brief 设置炮台目标角度
     * @param TargetPitch 目标俯仰角（度）
     * @param TargetYaw 目标偏航角（度）
     */
    UFUNCTION(BlueprintCallable, Category = "Turret Control")
    void SetTargetAngles(float TargetPitch, float TargetYaw);

    /**
     * @brief 发射弹丸
     * @param InitialSpeed 弹丸初速度（m/s）
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

    /** @brief 查询当前是否正在跟踪目标 */
    bool IsTracking() const;

    /**
     * @brief 使用 BC 弹道库计算弹道轨迹
     * @param StartPos 发射起点
     * @param ShootDir 发射方向
     * @param InitialSpeed 初速度（m/s）
     * @param OutTotalTime 输出总飞行时间
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

    /** @brief 瞄准与跟踪组件 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components")
    UTurretAiming* AimingComponent;

    /** @brief 场景采集组件，朝枪口方向输出图像 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Camera")
    USceneCaptureComponent2D* TurretSceneCapture;

    /** @brief CineCamera，用于继承场景 PostProcess 设置 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Camera")
    UCineCameraComponent* TurretCineCamera;

    /**
     * @brief 捕获当前炮台相机画面并返回 Base64 JPEG
     * @param Quality JPEG 压缩质量
     * @return Base64 编码后的 JPEG 图像数据
     */
    FString CaptureImageBase64(int32 Quality = -1);

    /** @brief 炮台 Agent ID */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config")
    FString TurretId = TEXT("turret_0");

    /** @brief 枪口相对 `GunMesh` 原点的偏移量 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config")
    FVector MuzzleOffset = FVector(-249.0f, -84.0f, 2.0f);

    /** @brief 弹丸 Actor 类型 */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Config")
    TSubclassOf<ABulletActor> BulletClass;

    /** @brief 默认弹丸初速度 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config")
    float DefaultMuzzleSpeed = 400.0f;

    /** @brief 炮台角度插值速度 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config")
    float RotationSpeed = 8.0f;

    /** @brief 相机视场角 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera")
    float CameraFOV = 90.0f;

    /** @brief 相机输出宽度 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera")
    int32 CameraWidth = 1280;

    /** @brief 相机输出高度 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera")
    int32 CameraHeight = 720;

    /** @brief 分割图像使用的 Stencil ID */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera", meta = (ClampMin = "0", ClampMax = "255"))
    int32 SegmentationId = 60;

    /** @brief 自动曝光补偿 EV */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera", meta = (ClampMin = "-10.0", ClampMax = "10.0"))
    float ExposureBias = 0.0f;

    /** @brief 默认 JPEG 压缩质量 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera", meta = (ClampMin = "1", ClampMax = "100"))
    int32 JpegQuality = 90;

    /** @brief 当前是否显示预测弹道线 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Prediction")
    bool bShowPredictionLine = false;

    /** @brief 预测弹道线颜色 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Prediction")
    FColor PredictionLineColor = FColor::Cyan;

    /** @brief 预测弹道线宽度 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Prediction")
    float PredictionLineThickness = 2.0f;

    /** @brief 预测命中点调试球半径 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Prediction")
    float PredictionHitRadius = 15.0f;

    /** @brief 预测命中点调试球颜色 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Prediction")
    FColor PredictionHitColor = FColor::Yellow;

private:
    /** @brief 创建底座、云台和枪管网格组件并建立层级关系 */
    void SetupTurretMesh();

    /** @brief 绘制预测弹道线和命中点 */
    void DrawPredictionLine();

    /** @brief 将 `TurretCineCamera` 的后处理设置同步到 `SceneCapture` */
    void SyncPostProcessToCapture();

    /** @brief 将炮台可见网格写入 CustomDepth/Stencil，供分割图像使用 */
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

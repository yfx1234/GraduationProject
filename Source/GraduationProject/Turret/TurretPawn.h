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

/**
 * 转台 Pawn
 * 底座(Base) → 云台(Gimbal,Yaw旋转) → 枪管(Gun,Pitch旋转)
 * 从 Master 项目迁移，移除 AgentActorBase / MavlinkSender 依赖
 */
UCLASS()
class GRADUATIONPROJECT_API ATurretPawn : public APawn
{
    GENERATED_BODY()

public:
    ATurretPawn();

protected:
    virtual void BeginPlay() override;

public:
    virtual void Tick(float DeltaTime) override;

    // ---- 控制接口 ----
    UFUNCTION(BlueprintCallable, Category = "Turret Control")
    void SetTargetAngles(float TargetPitch, float TargetYaw);

    UFUNCTION(BlueprintCallable, Category = "Turret Control")
    void FireX(float InitialSpeed = 400.0f);

    UFUNCTION(BlueprintCallable, Category = "Turret Control")
    void StartTracking(const FString& TargetActorID);

    UFUNCTION(BlueprintCallable, Category = "Turret Control")
    void StopTracking();

    UFUNCTION(BlueprintCallable, Category = "Turret Control")
    void ShowPredictionLine();

    UFUNCTION(BlueprintCallable, Category = "Turret Control")
    void HidePredictionLine();

    // ---- 状态查询 ----
    float GetCurrentPitch() const { return CurrentPitch; }
    float GetCurrentYaw() const { return CurrentYaw; }
    float GetTargetPitchAngle() const { return TargetPitch; }
    float GetTargetYawAngle() const { return TargetYaw; }
    bool IsTracking() const;

    // ---- 弹道计算 ----
    TArray<FVector> Ballistic(FVector StartPos, FRotator ShootDir, float InitialSpeed, float& OutTotalTime);

    // ---- 组件 ----
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components")
    USphereComponent* CollisionComponent;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components")
    UStaticMeshComponent* BaseMesh;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components")
    UStaticMeshComponent* GimbalMesh;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components")
    UStaticMeshComponent* GunMesh;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components")
    UTurretAiming* AimingComponent;

    // ---- 枪管摄像头 ----
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Camera")
    USceneCaptureComponent2D* TurretSceneCapture;

    /** CineCamera — 用于继承场景 PostProcessVolume 设置，再同步到 SceneCapture (仿 AirSim) */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Camera")
    UCineCameraComponent* TurretCineCamera;

    /** 采集一帧，返回 Base64 JPEG (Quality<=0 时使用 JpegQuality 属性) */
    FString CaptureImageBase64(int32 Quality = -1);

    // ---- 配置 ----
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config")
    FString TurretId = TEXT("turret_0");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config")
    FVector MuzzleOffset = FVector(-249.0f, -84.0f, 2.0f);
    

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Config")
    TSubclassOf<ABulletActor> BulletClass;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config")
    float DefaultMuzzleSpeed = 400.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config")
    float RotationSpeed = 8.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera")
    float CameraFOV = 90.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera")
    int32 CameraWidth = 1280;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera")
    int32 CameraHeight = 720;

    /** 曝光补偿 EV — 负值降低亮度, 正值提高亮度 (在 CineCamera PP 同步后叠加) */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera", meta = (ClampMin = "-10.0", ClampMax = "10.0"))
    float ExposureBias = 0.0f;

    /** JPEG 压缩质量 (1~100, 越高画质越好, 传输越慢) */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera", meta = (ClampMin = "1", ClampMax = "100"))
    int32 JpegQuality = 90;

    // ---- 预测线配置 ----
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Prediction")
    bool bShowPredictionLine = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Prediction")
    FColor PredictionLineColor = FColor::Cyan;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Prediction")
    float PredictionLineThickness = 2.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Prediction")
    float PredictionHitRadius = 15.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Prediction")
    FColor PredictionHitColor = FColor::Yellow;

private:
    void SetupTurretMesh();
    void DrawPredictionLine();
    /** 仿 AirSim copyCameraSettingsToSceneCapture — 将 CineCamera 的 PostProcess 同步到 SceneCapture */
    void SyncPostProcessToCapture();

    float TargetPitch = 0.0f;
    float TargetYaw = 0.0f;
    float CurrentPitch = 0.0f;
    float CurrentYaw = 0.0f;
};

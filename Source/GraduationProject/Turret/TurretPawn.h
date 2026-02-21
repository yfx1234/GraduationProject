#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "Components/StaticMeshComponent.h"
#include "Components/SphereComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/TextureRenderTarget2D.h"
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

    /** 采集一帧，返回 Base64 JPEG */
    FString CaptureImageBase64(int32 Quality = 85);

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
    int32 CameraWidth = 640;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera")
    int32 CameraHeight = 480;

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

    float TargetPitch = 0.0f;
    float TargetYaw = 0.0f;
    float CurrentPitch = 0.0f;
    float CurrentYaw = 0.0f;
};

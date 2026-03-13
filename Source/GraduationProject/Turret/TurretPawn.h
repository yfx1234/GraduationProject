#pragma once

#include "CineCameraComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Components/SphereComponent.h"
#include "Components/StaticMeshComponent.h"
#include "CoreMinimal.h"
#include "Engine/TextureRenderTarget2D.h"
#include "GameFramework/Pawn.h"
#include "TurretPawn.generated.h"

class ABulletActor;
class UTurretAiming;

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

    UFUNCTION(BlueprintCallable, Category = "Turret Control")
    void EnableApiControl(bool bEnable = true);

    UFUNCTION(BlueprintCallable, Category = "Turret Control")
    void ResetTurret();

    UFUNCTION(BlueprintCallable, Category = "Turret")
    FString GetState();

    UFUNCTION(BlueprintCallable, Category = "Turret|Camera")
    FString GetImage(FString ImageType = TEXT("scene"), int32 Quality = -1, float MaxDepthMeters = 200.0f);

    float GetCurrentPitch() const { return CurrentPitch; }
    float GetCurrentYaw() const { return CurrentYaw; }
    float GetTargetPitchAngle() const { return TargetPitch; }
    float GetTargetYawAngle() const { return TargetYaw; }
    bool IsTracking() const;

    TArray<FVector> Ballistic(FVector StartPos, FRotator ShootDir, float InitialSpeed, float& OutTotalTime);

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

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Camera")
    USceneCaptureComponent2D* TurretSceneCapture;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Camera")
    UCineCameraComponent* TurretCineCamera;

    FString CaptureImageBase64(int32 Quality = -1);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Config")
    FString TurretId = TEXT("turret_0");

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Config")
    bool bApiControlEnabled = true;

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

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera", meta = (ClampMin = "0", ClampMax = "255"))
    int32 SegmentationId = 60;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera", meta = (ClampMin = "-10.0", ClampMax = "10.0"))
    float ExposureBias = 0.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera", meta = (ClampMin = "1", ClampMax = "100"))
    int32 JpegQuality = 90;

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

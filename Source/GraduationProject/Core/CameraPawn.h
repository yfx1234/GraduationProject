#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/SpringArmComponent.h"
#include "Camera/CameraComponent.h"
#include "CameraPawn.generated.h"

class ADronePawn;
class ASimHUD;

UENUM()
enum class EDroneViewCycleMode : uint8
{
    Chase = 0,
    TopDown,
    FPV
};

UCLASS()
class GRADUATIONPROJECT_API ACameraPawn : public APawn
{
    GENERATED_BODY()

public:
    ACameraPawn();
    virtual void BeginPlay() override;
    virtual void Tick(float DeltaTime) override;
    virtual void SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent) override;

    UFUNCTION(BlueprintCallable, Category = "Camera")
    void StartTracking(AActor* Target);

    UFUNCTION(BlueprintCallable, Category = "Camera")
    void StopTracking();

    UFUNCTION(BlueprintCallable, Category = "Camera")
    void OnItemClicked(const FString& AgentId, AActor* Actor);

protected:
    void MoveForward(float Value);
    void MoveRight(float Value);
    void MoveUp(float Value);
    void CameraPitch(float Value);
    void CameraYaw(float Value);
    void OnZoomIn();
    void OnZoomOut();

    void OnTogglePip1();
    void OnTogglePip2();
    void OnTogglePip3();
    void OnCycleDroneView();
    void OnSwitchFreeView();

    void UpdateTrackingCamera(float DeltaTime);
    void UpdateTopDownCamera();
    void UpdateFPVCamera();

    ADronePawn* ResolveActiveDrone(bool bAutoSelect = true);
    ASimHUD* ResolveSimHUD() const;

protected:
    UPROPERTY(VisibleAnywhere, Category = "Camera")
    USceneComponent* RootComp;

    UPROPERTY(VisibleAnywhere, Category = "Camera")
    USpringArmComponent* SpringArm;

    UPROPERTY(VisibleAnywhere, Category = "Camera")
    UCameraComponent* Camera;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Settings")
    float MoveSpeed = 600.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Settings")
    float MouseSensitivity = 1.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Settings")
    float ZoomSpeed = 50.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Settings")
    float TrackingDistance = 500.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Settings")
    float MinTrackingDistance = 100.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Settings")
    float MaxTrackingDistance = 3000.0f;

private:
    bool bIsTracking = false;
    AActor* TrackingTarget = nullptr;
    float OrbitYaw = 0.0f;
    float OrbitPitch = -30.0f;

    FVector InputMoveDirection = FVector::ZeroVector;
    float InputPitchValue = 0.0f;
    float InputYawValue = 0.0f;

    FString ActiveDroneId;
    EDroneViewCycleMode DroneViewMode = EDroneViewCycleMode::Chase;
};

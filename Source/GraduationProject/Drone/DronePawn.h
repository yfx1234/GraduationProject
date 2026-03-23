
#pragma once
#include "CineCameraComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Components/StaticMeshComponent.h"
#include "CoreMinimal.h"
#include "DroneParameters.h"
#include "DroneState.h"
#include "Engine/TextureRenderTarget2D.h"
#include "GameFramework/Pawn.h"
#include "DronePawn.generated.h"
class UDroneApi;
class UDroneMovementComponent;
UCLASS()
class GRADUATIONPROJECT_API ADronePawn : public APawn
{
    GENERATED_BODY()
public:
    ADronePawn();
    virtual void BeginPlay() override;
    virtual void Tick(float DeltaTime) override;
    UFUNCTION(BlueprintCallable, Category = "DroneControl")
    void SetTargetPosition(const FVector& NewTargetPosition, float Speed = 0.0f, FString Frame = TEXT("ue"));
    UFUNCTION(BlueprintCallable, Category = "DroneControl")
    void SetTargetVelocity(const FVector& NewTargetVelocity, FString Frame = TEXT("ue"));
    UFUNCTION(BlueprintCallable, Category = "DroneControl")
    void SetHeadingControl(EDroneYawMode NewYawMode, EDroneDrivetrainMode NewDrivetrain, float YawDeg = 0.0f);
    UFUNCTION(BlueprintCallable, Category = "DroneControl")
    void Hover();
    UFUNCTION(BlueprintCallable, Category = "DroneControl")
    void MoveByVelocity(float Vx, float Vy, float Vz, FString Frame = TEXT("ue"));
    UFUNCTION(BlueprintCallable, Category = "Drone")
    void Takeoff(float Altitude = 3.0f);
    UFUNCTION(BlueprintCallable, Category = "Drone")
    void Land();
    UFUNCTION(BlueprintCallable, Category = "Drone")
    void EnableApiControl(bool bEnable = true);
    UFUNCTION(BlueprintCallable, Category = "DroneControl")
    void ResetDrone(const FVector& NewLocation, const FRotator& NewRotation, FString Frame = TEXT("ue"));
    UFUNCTION(BlueprintCallable, Category = "DroneControl")
    void ResetActorState();
    UFUNCTION(BlueprintCallable, Category = "DroneControl")
    void SetTargetAttitude(float RollDeg, float PitchDeg, float YawDeg, float Thrust = 9.81f, FString Frame = TEXT("ned"));
    UFUNCTION(BlueprintCallable, Category = "DroneControl")
    void SetMotorSpeeds(float M0, float M1, float M2, float M3);
    UFUNCTION(BlueprintCallable, Category = "DroneControl")
    void SetPositionControllerGains(float Kp, float Kd = 0.0f);
    UFUNCTION(BlueprintCallable, Category = "DroneControl")
    void SetVelocityControllerGains(float Kp, float Ki = 0.0f, float Kd = 0.0f);
    UFUNCTION(BlueprintCallable, Category = "DroneControl")
    void SetAttitudeControllerGains(float Kp, float Kd = 0.0f);
    UFUNCTION(BlueprintCallable, Category = "DroneControl")
    void SetAngleRateControllerGains(float Kp);
    FVector GetCurrentPosition() const;
    FVector GetCurrentVelocity() const;
    UFUNCTION(BlueprintCallable, Category = "Drone|Camera")
    void SetCameraAngles(float TargetPitch, float TargetYaw);
    UFUNCTION(BlueprintCallable, Category = "Drone|Camera")
    void SetCameraFOV(float NewFOV);
    UFUNCTION(BlueprintCallable, Category = "Drone|Camera")
    void SetSegmentationId(int32 NewSegmentationId);
    UFUNCTION(BlueprintCallable, Category = "Drone")
    FString GetState(FString Frame = TEXT("ue"));
    UFUNCTION(BlueprintCallable, Category = "Drone|Camera")
    FString GetImage(FString ImageType = TEXT("scene"), int32 Quality = -1, float MaxDepthMeters = 200.0f);
    FString CaptureImageBase64(int32 Quality = -1);
    float GetCameraCurrentPitch() const { return CameraCurrentPitch; }
    float GetCameraCurrentYaw() const { return CameraCurrentYaw; }
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|State")
    FDroneState CurrentState;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|State")
    EDroneControlMode ControlMode = EDroneControlMode::Idle;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters")
    FDroneParameters Parameters;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    UDroneMovementComponent* MovementComp = nullptr;
    UPROPERTY()
    UDroneApi* Api = nullptr;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Config")
    FString DroneId = TEXT("drone_0");
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Config")
    EDroneMissionRole MissionRole = EDroneMissionRole::Unknown;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Config")
    bool bApiControlEnabled = true;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    USceneComponent* RootComp;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    UStaticMeshComponent* BodyMesh;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    UStaticMeshComponent* Fan0;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    UStaticMeshComponent* Fan1;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    UStaticMeshComponent* Fan2;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    UStaticMeshComponent* Fan3;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    UStaticMeshComponent* CameraYawMesh;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    UStaticMeshComponent* CameraPitchMesh;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Camera")
    USceneCaptureComponent2D* DroneSceneCapture;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Camera")
    UCineCameraComponent* DroneCineCamera;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera")
    float CameraFOV = 90.0f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera")
    int32 CameraWidth = 1280;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera")
    int32 CameraHeight = 720;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera", meta = (ClampMin = "0", ClampMax = "255"))
    int32 SegmentationId = 20;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera", meta = (ClampMin = "-10.0", ClampMax = "10.0"))
    float ExposureBias = 0.0f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera", meta = (ClampMin = "1", ClampMax = "100"))
    int32 JpegQuality = 90;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera")
    float CameraRotationSpeed = 8.0f;
private:
    void ApplyStateToActor(const FDroneState& State);
    void UpdatePropellerAnimation(float DeltaTime);
    UStaticMeshComponent* GetFanMesh(int32 Index) const;
    void UpdateCameraRotation(float DeltaTime);
    float CameraTargetPitch = 0.0f;
    float CameraTargetYaw = 0.0f;
    float CameraCurrentPitch = 0.0f;
    float CameraCurrentYaw = 0.0f;
};

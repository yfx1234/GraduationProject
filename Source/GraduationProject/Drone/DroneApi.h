#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "DroneParameters.h"
#include "DroneApi.generated.h"

class ADronePawn;

/**
 * DroneApi — AirSim 风格统一接口
 * 参考 AirSim MultirotorApiBase
 * TCP Handler → DroneApi → DronePawn → MovementComp
 */
UCLASS()
class GRADUATIONPROJECT_API UDroneApi : public UObject
{
    GENERATED_BODY()

public:
    void Initialize(ADronePawn* Owner);

    // ---- 位置控制 (SI 坐标, 米) ----
    void MoveToPosition(float X, float Y, float Z, float Speed = 2.0f);
    void Hover();
    void Takeoff(float Altitude);
    void Land();

    // ---- 速度控制 (m/s) ----
    void MoveByVelocity(float Vx, float Vy, float Vz);

    // ---- 状态查询 ----
    FVector GetPosition() const;
    FVector GetVelocity() const;
    FRotator GetOrientation() const;
    TArray<float> GetMotorSpeeds() const;
    EDroneControlMode GetControlMode() const;

    // ---- PID 调参 ----
    void SetPositionControllerGains(float Kp, float Kd);
    void SetVelocityControllerGains(float Kp, float Ki, float Kd);
    void SetAttitudeControllerGains(float Kp, float Kd);
    void SetAngleRateControllerGains(float Kp);

    // ---- 重置 ----
    void Reset(FVector Position = FVector::ZeroVector, FRotator Rotation = FRotator::ZeroRotator);

private:
    UPROPERTY()
    ADronePawn* OwnerPawn = nullptr;
};

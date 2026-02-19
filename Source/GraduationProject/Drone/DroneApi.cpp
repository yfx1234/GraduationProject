#include "DroneApi.h"
#include "DronePawn.h"
#include "DroneMovementComponent.h"

void UDroneApi::Initialize(ADronePawn* Owner)
{
    OwnerPawn = Owner;
}

void UDroneApi::MoveToPosition(float X, float Y, float Z, float Speed)
{
    if (OwnerPawn)
    {
        OwnerPawn->SetTargetPosition(FVector(X, Y, Z));
        UE_LOG(LogTemp, Log, TEXT("[DroneApi] MoveToPosition(%.1f, %.1f, %.1f)"), X, Y, Z);
    }
}

void UDroneApi::Hover()
{
    if (OwnerPawn) OwnerPawn->Hover();
}

void UDroneApi::Takeoff(float Altitude)
{
    if (OwnerPawn) OwnerPawn->Takeoff(Altitude);
}

void UDroneApi::Land()
{
    if (OwnerPawn) OwnerPawn->Land();
}

void UDroneApi::MoveByVelocity(float Vx, float Vy, float Vz)
{
    if (OwnerPawn)
    {
        OwnerPawn->SetTargetVelocity(FVector(Vx, Vy, Vz));
        UE_LOG(LogTemp, Log, TEXT("[DroneApi] MoveByVelocity(%.1f, %.1f, %.1f)"), Vx, Vy, Vz);
    }
}

FVector UDroneApi::GetPosition() const
{
    return OwnerPawn ? OwnerPawn->GetCurrentPosition() : FVector::ZeroVector;
}

FVector UDroneApi::GetVelocity() const
{
    return OwnerPawn ? OwnerPawn->GetCurrentVelocity() : FVector::ZeroVector;
}

FRotator UDroneApi::GetOrientation() const
{
    return OwnerPawn ? OwnerPawn->CurrentState.GetRotator() : FRotator::ZeroRotator;
}

TArray<float> UDroneApi::GetMotorSpeeds() const
{
    TArray<float> Result;
    if (OwnerPawn && OwnerPawn->CurrentState.MotorSpeeds.Num() == 4)
    {
        for (double Speed : OwnerPawn->CurrentState.MotorSpeeds)
            Result.Add(static_cast<float>(Speed));
    }
    return Result;
}

EDroneControlMode UDroneApi::GetControlMode() const
{
    return OwnerPawn ? OwnerPawn->ControlMode : EDroneControlMode::Idle;
}

void UDroneApi::SetPositionControllerGains(float Kp, float Kd)
{
    if (OwnerPawn && OwnerPawn->MovementComp)
        OwnerPawn->MovementComp->SetPositionGains(Kp, Kd);
}

void UDroneApi::SetVelocityControllerGains(float Kp, float Ki, float Kd)
{
    if (OwnerPawn && OwnerPawn->MovementComp)
        OwnerPawn->MovementComp->SetVelocityGains(Kp, Ki, Kd);
}

void UDroneApi::SetAttitudeControllerGains(float Kp, float Kd)
{
    if (OwnerPawn && OwnerPawn->MovementComp)
        OwnerPawn->MovementComp->SetAttitudeGains(Kp, Kd);
}

void UDroneApi::SetAngleRateControllerGains(float Kp)
{
    if (OwnerPawn && OwnerPawn->MovementComp)
        OwnerPawn->MovementComp->SetAngleRateGains(Kp);
}

void UDroneApi::Reset(FVector Position, FRotator Rotation)
{
    if (OwnerPawn) OwnerPawn->ResetDrone(Position, Rotation);
}


#pragma once
#include "CoreMinimal.h"
#include "DroneParameters.generated.h"
UENUM(BlueprintType)
enum class EDroneMissionRole : uint8
{
    Unknown     UMETA(DisplayName = "Unknown"),
    Target      UMETA(DisplayName = "Target"),
    Interceptor UMETA(DisplayName = "Interceptor")
};
UENUM(BlueprintType)
enum class EDroneControlMode : uint8
{
    Idle            UMETA(DisplayName = "Idle"),
    MotorSpeed      UMETA(DisplayName = "Motor Speed"),
    TorqueThrust    UMETA(DisplayName = "Torque & Thrust"),
    AttitudeThrust  UMETA(DisplayName = "Attitude & Thrust"),
    Velocity        UMETA(DisplayName = "Velocity"),
    Position        UMETA(DisplayName = "Position")
};
UENUM(BlueprintType)
enum class EDroneYawMode : uint8
{
    Auto    UMETA(DisplayName = "Auto"),
    Hold    UMETA(DisplayName = "Hold"),
    Angle   UMETA(DisplayName = "Angle"),
    Rate    UMETA(DisplayName = "Rate")
};
UENUM(BlueprintType)
enum class EDroneDrivetrainMode : uint8
{
    ForwardOnly        UMETA(DisplayName = "ForwardOnly"),
    MaxDegreeOfFreedom UMETA(DisplayName = "MaxDegreeOfFreedom")
};
USTRUCT(BlueprintType)
struct GRADUATIONPROJECT_API FDroneParameters
{
    GENERATED_BODY()
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Mass")
    double Mass = 1.0230;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Mass")
    double MotorAssemblyWeight = 0.055;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Inertia")
    double Jx = 0.0095;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Inertia")
    double Jy = 0.0095;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Inertia")
    double Jz = 0.0186;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Geometry")
    double ArmLength = 0.2223;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Geometry")
    double MotorAngle = 45.0;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Geometry")
    double BodyBoxX = 0.180;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Geometry")
    double BodyBoxY = 0.11;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Geometry")
    double BodyBoxZ = 0.04;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Geometry")
    double RotorZ = 0.025;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Propulsion")
    double C_T = 0.1667725;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Propulsion")
    double C_P = 0.0901964;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Propulsion")
    double MaxRPM = 12000.0;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Propulsion")
    double PropellerDiameter = 0.2286;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Propulsion")
    double PropellerHeight = 0.01;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Motor")
    double MotorFilterTC = 0.076;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Motor")
    double MinMotorSpeed = 0.0;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Environment")
    double Gravity = 9.81;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Environment")
    double AirDensity = 1.175;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Environment")
    double DragCoefficient = 0.325;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Collision")
    double Restitution = 0.55;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Collision")
    double Friction = 0.5;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Simulation")
    double TimeStep = 0.003;
    double ThrustCoefficient = 1.355525e-5;
    double TorqueCoefficient = 2.66728e-7;
    double MaxMotorSpeed = 1256.637;
    double MaxThrust = 21.405352;
    double MaxTorque = 0.421195;
    void CalculateMaxThrust()
    {
        const double RevPerSec = MaxRPM / 60.0;
        MaxMotorSpeed = RevPerSec * 2.0 * PI;
        const double nSquared = RevPerSec * RevPerSec;
        const double D4 = FMath::Pow(PropellerDiameter, 4.0);
        const double D5 = FMath::Pow(PropellerDiameter, 5.0);
        MaxThrust = C_T * AirDensity * nSquared * D4;
        MaxTorque = C_P * AirDensity * nSquared * D5 / (2.0 * PI);
        const double MaxSpeedSquared = MaxMotorSpeed * MaxMotorSpeed;
        if (MaxSpeedSquared > KINDA_SMALL_NUMBER)
        {
            ThrustCoefficient = MaxThrust / MaxSpeedSquared;
            TorqueCoefficient = MaxTorque / MaxSpeedSquared;
        }
    }
    void ComputeInertiaMatrix()
    {
        double BoxMass = Mass - 4.0 * MotorAssemblyWeight;
        BoxMass = FMath::Max(BoxMass, 0.1);
        Jx = BoxMass / 12.0 * (BodyBoxY * BodyBoxY + BodyBoxZ * BodyBoxZ);
        Jy = BoxMass / 12.0 * (BodyBoxX * BodyBoxX + BodyBoxZ * BodyBoxZ);
        Jz = BoxMass / 12.0 * (BodyBoxX * BodyBoxX + BodyBoxY * BodyBoxY);
        const double SinA = FMath::Sin(FMath::DegreesToRadians(MotorAngle));
        const double CosA = FMath::Cos(FMath::DegreesToRadians(MotorAngle));
        for (int32 i = 0; i < 4; ++i)
        {
            const double Sign1 = (i == 0 || i == 3) ? 1.0 : -1.0;
            const double Sign2 = (i < 2) ? 1.0 : -1.0;
            const double mx = ArmLength * CosA * Sign2;
            const double my = ArmLength * SinA * Sign1;
            const double mz = RotorZ;
            Jx += (my * my + mz * mz) * MotorAssemblyWeight;
            Jy += (mx * mx + mz * mz) * MotorAssemblyWeight;
            Jz += (mx * mx + my * my) * MotorAssemblyWeight;
        }
    }
    double GetHoverMotorSpeed() const
    {
        if (ThrustCoefficient > SMALL_NUMBER)
        {
            return FMath::Sqrt(Mass * Gravity / (4.0 * ThrustCoefficient));
        }
        return 0.0;
    }
    void InitializeComputed()
    {
        CalculateMaxThrust();
        if (Jx <= SMALL_NUMBER || Jy <= SMALL_NUMBER || Jz <= SMALL_NUMBER)
        {
            ComputeInertiaMatrix();
        }
    }
};

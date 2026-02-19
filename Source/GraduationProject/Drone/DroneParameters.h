#pragma once
#include "CoreMinimal.h"

#include "DroneParameters.generated.h"

UENUM(BlueprintType)

/**
 * 枚举控制模式
 */
enum class EDroneControlMode : uint8
{
    // 空闲
    Idle            UMETA(DisplayName = "Idle"),

    // 电机转速控制
    MotorSpeed      UMETA(DisplayName = "Motor Speed"),

    // 力矩推力控制
    TorqueThrust    UMETA(DisplayName = "Torque & Thrust"),

    // 姿态推力控制，控制无人机的倾斜角度和油门大小
    AttitudeThrust  UMETA(DisplayName = "Attitude & Thrust"),

    // 速度控制，只控制速度
    Velocity        UMETA(DisplayName = "Velocity"),

    // 位置控制
    Position        UMETA(DisplayName = "Position")
};

/**
 * 无人机物理参数结构体
 */
USTRUCT(BlueprintType)
struct GRADUATIONPROJECT_API FDroneParameters
{
    GENERATED_BODY()
    /**
     * 质量参数
     * @param Mass 质量 kg
     * @param Jx 绕X轴转动惯量 kg·m²
     * @param Jy 绕Y轴转动惯量 kg·m²
     * @param Jz 绕Z轴转动惯量 kg·m²
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Mass")
    double Mass = 1.5;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Inertia")
    double Jx = 0.025;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Inertia")
    double Jy = 0.025;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Inertia")
    double Jz = 0.04;

    /**
     * 几何参数
     * @param ArmLength 臂长 m
     * @param MotorAngle 电机与X轴夹角 度
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Geometry")
    double ArmLength = 0.23;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Geometry")
    double MotorAngle = 45.0;

    /**
     * 推进系统参数
     * @param ThrustCoefficient 推力系数 N/(rad/s)²，升力F = kT * ω^2
     * @param TorqueCoefficient 力矩系数 N·m/(rad/s)²，反向力矩τ = kQ * ω^2
     * @param MaxMotorSpeed 最大电机转速 rad/s
     * @param MinMotorSpeed 最小电机转速 rad/s
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Propulsion")
    double ThrustCoefficient = 1.0e-5;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Propulsion")
    double TorqueCoefficient = 1.0e-7;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Propulsion")
    double MaxMotorSpeed = 1500.0;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Propulsion")
    double MinMotorSpeed = 100.0;

    /**
     * 环境参数
     * @param Gravity 重力加速度 m/s²
     * @param DragCoefficient 空气阻力系数
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Environment")
    double Gravity = 9.81;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Environment")
    double DragCoefficient = 0.1;

    /**
     * 仿真参数
     * @param TimeStep 仿真时间步长 s
     * @param SubSteps 每帧子步数
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Simulation")
    double TimeStep = 0.002;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Simulation")
    int32 SubSteps = 10;

    /**
     * 辅助函数
     * @return 悬停时的电机转速 rad/s
     */
    double GetHoverMotorSpeed() const
    {
        // 4 * kT * w^2 = m * g  =>  w = sqrt(m*g / (4*kT))
        return FMath::Sqrt(Mass * Gravity / (4.0 * ThrustCoefficient));
    }
};

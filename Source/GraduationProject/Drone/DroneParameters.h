#pragma once
#include "CoreMinimal.h"

#include "DroneParameters.generated.h"

/**
 * @brief 无人机控制模式枚举
 * - Idle: 空闲，不产生任何控制输出
 * - MotorSpeed: 直接控制四个电机转速
 * - TorqueThrust: 控制力矩和推力
 * - AttitudeThrust: 控制倾斜角度和油门
 * - Velocity: 速度控制
 * - Position: 位置控制
 */
UENUM(BlueprintType)

enum class EDroneControlMode : uint8
{
    /** @brief 空闲模式，不产生控制输出 */
    Idle            UMETA(DisplayName = "Idle"),

    /** @brief 电机转速直接控制模式 */
    MotorSpeed      UMETA(DisplayName = "Motor Speed"),

    /** @brief 力矩+推力直接控制模式 */
    TorqueThrust    UMETA(DisplayName = "Torque & Thrust"),

    /** @brief 姿态+推力控制模式 */
    AttitudeThrust  UMETA(DisplayName = "Attitude & Thrust"),

    /** @brief 速度控制模式 */
    Velocity        UMETA(DisplayName = "Velocity"),

    /** @brief 位置控制模式 */
    Position        UMETA(DisplayName = "Position")
};

/**
 * @brief 无人机物理参数结构体
 * - 质量和惯量参数
 * - 几何参数（臂长、电机角度）
 * - 推进系统参数（推力/力矩系数、电机转速限制）
 * - 环境参数（重力、空气阻力）
 * - 仿真参数（时间步长、子步数）
 */
USTRUCT(BlueprintType)
struct GRADUATIONPROJECT_API FDroneParameters
{
    GENERATED_BODY()

    /** @brief 无人机总质量 kg */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Mass")
    double Mass = 1.5;

    /** @brief 绕 X 轴的转动惯量 kg·m² */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Inertia")
    double Jx = 0.025;

    /** @brief 绕 Y 轴的转动惯量 kg·m² */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Inertia")
    double Jy = 0.025;

    /** @brief 绕 Z 轴的转动惯量 kg·m² */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Inertia")
    double Jz = 0.04;

    /** @brief 电机臂长 m */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Geometry")
    double ArmLength = 0.23;

    /** @brief 电机臂与机体 X 轴的夹角 度 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Geometry")
    double MotorAngle = 45.0;

    /** @brief 推力系数 N/(rad/s)² */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Propulsion")
    double ThrustCoefficient = 1.0e-5;

    /** @brief 力矩系数 N·m/(rad/s)² */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Propulsion")
    double TorqueCoefficient = 1.0e-7;

    /** @brief 最大电机转速 rad/s */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Propulsion")
    double MaxMotorSpeed = 1500.0;

    /** @brief 最小电机转速 rad/s */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Propulsion")
    double MinMotorSpeed = 100.0;

    /** @brief 重力加速度 m/s² */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Environment")
    double Gravity = 9.81;

    /** @brief 空气阻力系数 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Environment")
    double DragCoefficient = 0.1;

    /** @brief 仿真基础时间步长 秒 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Simulation")
    double TimeStep = 0.002;

    /** @brief 每帧的计划子步数 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Simulation")
    int32 SubSteps = 10;

    /**
     * @brief 计算悬停时的电机转速
     * @return 悬停转速 rad/s
     * 4 * kT * ω² = m * g
     * ω = sqrt(m*g / (4*kT))
     */
    double GetHoverMotorSpeed() const
    {
        return FMath::Sqrt(Mass * Gravity / (4.0 * ThrustCoefficient));
    }
};

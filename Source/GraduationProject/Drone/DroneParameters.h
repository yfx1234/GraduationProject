/**
 * @file DroneParameters.h
 * @brief 无人机物理参数和控制模式枚举的头文件
 *
 * 本文件定义了：
 * 1. EDroneControlMode 枚举 — 无人机的控制模式（空闲、电机、力矩、姿态、速度、位置）
 * 2. FDroneParameters 结构体 — 四旋翼无人机的全部物理参数（质量、惯量、几何、推进、环境、仿真）
 *
 * 所有物理参数使用 SI 国际单位制。
 */

#pragma once
#include "CoreMinimal.h"

#include "DroneParameters.generated.h"

/**
 * 无人机控制模式枚举
 *
 * 按控制复杂度从低到高排列：
 * - Idle: 空闲，不产生任何控制输出
 * - MotorSpeed: 直接控制四个电机转速
 * - TorqueThrust: 控制力矩和推力
 * - AttitudeThrust: 控制倾斜角度和油门
 * - Velocity: 速度控制（经过位置→速度→姿态→角速率级联控制环）
 * - Position: 位置控制（完整级联控制链）
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

    /** @brief 姿态+推力控制模式（控制倾斜角度和油门大小） */
    AttitudeThrust  UMETA(DisplayName = "Attitude & Thrust"),

    /** @brief 速度控制模式（控制三轴速度，内部级联到姿态环） */
    Velocity        UMETA(DisplayName = "Velocity"),

    /** @brief 位置控制模式（控制三维位置，内部级联到速度→姿态环） */
    Position        UMETA(DisplayName = "Position")
};

/**
 * 无人机物理参数结构体
 *
 * 封装四旋翼无人机仿真所需的全部物理和仿真参数，分为：
 * - 质量和惯量参数
 * - 几何参数（臂长、电机角度）
 * - 推进系统参数（推力/力矩系数、电机转速限制）
 * - 环境参数（重力、空气阻力）
 * - 仿真参数（时间步长、子步数）
 *
 * 所有参数使用 SI 国际单位制。
 */
USTRUCT(BlueprintType)
struct GRADUATIONPROJECT_API FDroneParameters
{
    GENERATED_BODY()

    // ========== 质量参数 ==========

    /** @brief 无人机总质量，单位：kg */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Mass")
    double Mass = 1.5;

    /** @brief 绕 X 轴（滚转轴）的转动惯量，单位：kg·m² */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Inertia")
    double Jx = 0.025;

    /** @brief 绕 Y 轴（俯仰轴）的转动惯量，单位：kg·m² */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Inertia")
    double Jy = 0.025;

    /** @brief 绕 Z 轴（偏航轴）的转动惯量，单位：kg·m² */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Inertia")
    double Jz = 0.04;

    // ========== 几何参数 ==========

    /** @brief 电机臂长（中心到电机的距离），单位：m */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Geometry")
    double ArmLength = 0.23;

    /** @brief 电机臂与机体 X 轴的夹角，单位：度（X型布局为45°） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Geometry")
    double MotorAngle = 45.0;

    // ========== 推进系统参数 ==========

    /**
     * @brief 推力系数 kT，单位：N/(rad/s)²
     * 
     * 单个电机产生的升力：F = kT * ω²
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Propulsion")
    double ThrustCoefficient = 1.0e-5;

    /**
     * @brief 力矩系数 kQ，单位：N·m/(rad/s)²
     * 
     * 单个电机产生的反扭矩：τ = kQ * ω²
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Propulsion")
    double TorqueCoefficient = 1.0e-7;

    /** @brief 最大电机转速，单位：rad/s */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Propulsion")
    double MaxMotorSpeed = 1500.0;

    /** @brief 最小电机转速，单位：rad/s */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Propulsion")
    double MinMotorSpeed = 100.0;

    // ========== 环境参数 ==========

    /** @brief 重力加速度，单位：m/s² */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Environment")
    double Gravity = 9.81;

    /** @brief 空气阻力系数（线性阻力模型：F_drag = -Cd * |v| * v） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Environment")
    double DragCoefficient = 0.1;

    // ========== 仿真参数 ==========

    /** @brief 仿真基础时间步长，单位：秒（RK4 子步的目标时长） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Simulation")
    double TimeStep = 0.002;

    /** @brief 每帧的计划子步数（实际子步数由帧时间动态调整） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Simulation")
    int32 SubSteps = 10;

    // ========== 辅助函数 ==========

    /**
     * @brief 计算悬停时的电机转速
     * @return 悬停转速，单位：rad/s
     *
     * 悬停条件：4 * kT * ω² = m * g
     * 因此：ω = sqrt(m*g / (4*kT))
     */
    double GetHoverMotorSpeed() const
    {
        return FMath::Sqrt(Mass * Gravity / (4.0 * ThrustCoefficient));
    }
};

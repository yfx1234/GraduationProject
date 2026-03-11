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
    /** @brief 空闲模式，不产生任何控制输出 */
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
 * @brief 机体航向模式（对齐 AirSim 常见语义）
 * - Auto: AirSim 默认 yaw_mode (is_rate=true, yaw_or_rate=0)
 * - Hold: 保持当前航向
 * - Angle: 指定绝对偏航角
 * - Rate: 指定偏航角速度
 */
UENUM(BlueprintType)
enum class EDroneYawMode : uint8
{
    Auto    UMETA(DisplayName = "Auto"),
    Hold    UMETA(DisplayName = "Hold"),
    Angle   UMETA(DisplayName = "Angle"),
    Rate    UMETA(DisplayName = "Rate")
};


/**
 * @brief 驱动模式（对齐 AirSim drivetrain 语义）
 * - ForwardOnly: 机头优先跟随运动方向
 * - MaxDegreeOfFreedom: 航向和运动解耦
 */
UENUM(BlueprintType)
enum class EDroneDrivetrainMode : uint8
{
    ForwardOnly        UMETA(DisplayName = "ForwardOnly"),
    MaxDegreeOfFreedom UMETA(DisplayName = "MaxDegreeOfFreedom")
};
/**
 * @brief 无人机物理参数结构体（对标 AirSim）
 *
 * 参数分组：
 * - 质量和惯量参数
 * - 几何参数（臂长、电机角度、机体包围盒）
 * - 推进系统参数（推力/力矩系数 - 基于空气动力学公式）
 * - 电机参数（转速限制、滤波时间常数）
 * - 环境参数（重力、空气密度、阻力系数）
 * - 碰撞参数（恢复系数、摩擦系数）
 * - 仿真参数（时间步长）
 */
USTRUCT(BlueprintType)
struct GRADUATIONPROJECT_API FDroneParameters
{
    GENERATED_BODY()

    // ═══════════════════ 质量 ═══════════════════

    /** @brief 无人机总质量 kg（AirSim 默认 1.0） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Mass")
    double Mass = 1.0;

    /** @brief 单个电机组件重量 kg（AirSim F450: 0.055） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Mass")
    double MotorAssemblyWeight = 0.055;

    // ═══════════════════ 惯量（可手动设置或自动计算） ═══════════════════

    /** @brief 绕 X 轴的转动惯量 kg·m2（由 ComputeInertiaMatrix 自动计算） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Inertia")
    double Jx = 0.0023;

    /** @brief 绕 Y 轴的转动惯量 kg·m2（由 ComputeInertiaMatrix 自动计算） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Inertia")
    double Jy = 0.0023;

    /** @brief 绕 Z 轴的转动惯量 kg·m2（由 ComputeInertiaMatrix 自动计算） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Inertia")
    double Jz = 0.004;

    // ═══════════════════ 几何参数 ═══════════════════

    /** @brief 电机臂长 m（AirSim F450: 0.2275） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Geometry")
    double ArmLength = 0.2275;

    /** @brief 电机臂与机体 X 轴的夹角 度（标准 QuadX: 45） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Geometry")
    double MotorAngle = 45.0;

    /** @brief 机体包围盒尺寸 X（前后）m（AirSim: 0.18） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Geometry")
    double BodyBoxX = 0.180;

    /** @brief 机体包围盒尺寸 Y（左右）m（AirSim: 0.11） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Geometry")
    double BodyBoxY = 0.11;

    /** @brief 机体包围盒尺寸 Z（上下）m（AirSim: 0.04） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Geometry")
    double BodyBoxZ = 0.04;

    /** @brief 电机相对重心的 Z 偏移量 m（AirSim: 0.025） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Geometry")
    double RotorZ = 0.025;

    // ═══════════════════ 推进系统（空气动力学公式） ═══════════════════

    /**
     * @brief 推力系数 C_T（无量纲）
     * AirSim 默认 GWS 9X5 螺旋桨: 0.109919 @ 6396.667 RPM
     * Force = C_T * ρ * n2 * D?
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Propulsion")
    double C_T = 0.109919;

    /**
     * @brief 功率/力矩系数 C_P（无量纲）
     * AirSim 默认: 0.040164
     * Torque = C_P * ρ * n2 * D? / (2π)
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Propulsion")
    double C_P = 0.040164;

    /** @brief 最大电机转速 RPM（AirSim: 6396.667） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Propulsion")
    double MaxRPM = 6396.667;

    /** @brief 螺旋桨直径 m（AirSim DJI Phantom 2: 0.2286） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Propulsion")
    double PropellerDiameter = 0.2286;

    /** @brief 螺旋桨厚度/高度 m（AirSim: 0.01） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Propulsion")
    double PropellerHeight = 0.01;

    // ═══════════════════ 电机动态 ═══════════════════

    /** @brief 电机控制信号低通滤波时间常数 秒（AirSim: 0.005） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Motor")
    double MotorFilterTC = 0.005;

    /** @brief 最小电机转速 rad/s */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Motor")
    double MinMotorSpeed = 0.0;

    // ═══════════════════ 环境参数 ═══════════════════

    /** @brief 重力加速度 m/s2 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Environment")
    double Gravity = 9.81;

    /** @brief 空气密度 kg/m3（海平面标准值） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Environment")
    double AirDensity = 1.225;

    /**
     * @brief 线性阻力系数（AirSim: 1.3/4 = 0.325）
     * AirSim 使用 6 面体阻力模型，此处为等效标量
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Environment")
    double DragCoefficient = 0.325;

    // ═══════════════════ 碰撞参数 ═══════════════════

    /** @brief 碰撞恢复系数（0=完全非弹性, 1=完全弹性, AirSim: 0.55） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Collision")
    double Restitution = 0.55;

    /** @brief 碰撞摩擦系数（AirSim: 0.5） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Collision")
    double Friction = 0.5;

    // ═══════════════════ 仿真参数 ═══════════════════

    /** @brief 仿真基础时间步长 秒 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Simulation")
    double TimeStep = 0.003;

    // ═══════════════════ 计算属性 ═══════════════════

    /** @brief 计算得到的推力系数 kT = C_T * ρ * D?（N/(rev/s)2） */
    double ThrustCoefficient = 1.0e-5;

    /** @brief 计算得到的力矩系数 kQ = C_P * ρ * D? / (2π)（N·m/(rev/s)2） */
    double TorqueCoefficient = 1.0e-7;

    /** @brief 计算得到的最大电机转速 rad/s */
    double MaxMotorSpeed = 669.85;

    /** @brief 计算得到的单电机最大推力 N */
    double MaxThrust = 4.179;

    /** @brief 计算得到的单电机最大力矩 N·m */
    double MaxTorque = 0.0556;

    /**
     * @brief 根据空气动力学参数计算推力/力矩系数和最大值
     * 基于 AirSim 公式：
     *   MaxThrust = C_T * ρ * n2 * D?
     *   MaxTorque = C_P * ρ * n2 * D? / (2π)
     *   kT = MaxThrust / MaxSpeed2（转换为 rad/s 制式）
     *   kQ = MaxTorque / MaxSpeed2
     */
    void CalculateMaxThrust()
    {
        double RevPerSec = MaxRPM / 60.0;
        MaxMotorSpeed = RevPerSec * 2.0 * PI;
        double nSquared = RevPerSec * RevPerSec;
        double D4 = FMath::Pow(PropellerDiameter, 4.0);
        double D5 = FMath::Pow(PropellerDiameter, 5.0);
        MaxThrust = C_T * AirDensity * nSquared * D4;
        MaxTorque = C_P * AirDensity * nSquared * D5 / (2.0 * PI);

        // 转换为 kT·ω2 形式的系数（ω 为 rad/s）
        double MaxSpeedSquared = MaxMotorSpeed * MaxMotorSpeed;
        if (MaxSpeedSquared > KINDA_SMALL_NUMBER)
        {
            ThrustCoefficient = MaxThrust / MaxSpeedSquared;
            TorqueCoefficient = MaxTorque / MaxSpeedSquared;
        }
    }

    /**
     * @brief 基于机体尺寸和电机位置自动计算惯性矩阵（对角元素）
     * 参考 AirSim MultiRotorParams::computeInertiaMatrix
     *
     * 将机体近似为均匀长方体，加上 4 个电机的质点贡献：
     *   Jx = m_box/12 * (By2 + Bz2) + Σ(yi2 + zi2) * m_motor
     *   Jy = m_box/12 * (Bx2 + Bz2) + Σ(xi2 + zi2) * m_motor
     *   Jz = m_box/12 * (Bx2 + By2) + Σ(xi2 + yi2) * m_motor
     */
    void ComputeInertiaMatrix()
    {
        double BoxMass = Mass - 4.0 * MotorAssemblyWeight;
        BoxMass = FMath::Max(BoxMass, 0.1);

        // 长方体惯量
        Jx = BoxMass / 12.0 * (BodyBoxY * BodyBoxY + BodyBoxZ * BodyBoxZ);
        Jy = BoxMass / 12.0 * (BodyBoxX * BodyBoxX + BodyBoxZ * BodyBoxZ);
        Jz = BoxMass / 12.0 * (BodyBoxX * BodyBoxX + BodyBoxY * BodyBoxY);

        // 4 个电机（QuadX 布局，45° 旋转）的贡献
        double SinA = FMath::Sin(FMath::DegreesToRadians(MotorAngle));
        double CosA = FMath::Cos(FMath::DegreesToRadians(MotorAngle));
        for (int32 i = 0; i < 4; ++i)
        {
            // 电机 i 的位置
            double Sign1 = (i == 0 || i == 3) ? 1.0 : -1.0;
            double Sign2 = (i < 2) ? 1.0 : -1.0;
            double mx = ArmLength * CosA * Sign2;
            double my = ArmLength * SinA * Sign1;
            double mz = RotorZ;

            Jx += (my * my + mz * mz) * MotorAssemblyWeight;
            Jy += (mx * mx + mz * mz) * MotorAssemblyWeight;
            Jz += (mx * mx + my * my) * MotorAssemblyWeight;
        }
    }

    /**
     * @brief 计算悬停时的电机转速
     * @return 悬停转速 rad/s
     * 4 * kT * ω2 = m * g  →  ω = sqrt(m*g / (4*kT))
     */
    double GetHoverMotorSpeed() const
    {
        if (ThrustCoefficient > KINDA_SMALL_NUMBER)
            return FMath::Sqrt(Mass * Gravity / (4.0 * ThrustCoefficient));
        return 0.0;
    }

    /**
     * @brief 初始化所有计算属性
     * 调用此方法来根据基础参数计算推力系数、惯性矩阵等
     */
    void InitializeComputed()
    {
        CalculateMaxThrust();
        ComputeInertiaMatrix();
    }
};






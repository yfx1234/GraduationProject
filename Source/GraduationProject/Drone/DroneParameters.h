#pragma once
#include "CoreMinimal.h"

#include "DroneParameters.generated.h"

/**
 * @brief 无人机控制模式枚举
 *
 * 该枚举描述飞控当前接受哪一层控制指令：
 * - `Idle`：空闲，不输出任何控制命令；
 * - `MotorSpeed`：直接给四个电机下发角速度；
 * - `TorqueThrust`：直接控制总推力与机体系力矩；
 * - `AttitudeThrust`：控制姿态角与总推力；
 * - `Velocity`：控制线速度；
 * - `Position`：控制目标位置。
 */
UENUM(BlueprintType)
enum class EDroneControlMode : uint8
{
    /** @brief 空闲模式，不发送任何控制命令 */
    Idle            UMETA(DisplayName = "Idle"),

    /** @brief 电机转速直接控制模式 */
    MotorSpeed      UMETA(DisplayName = "Motor Speed"),

    /** @brief 力矩与推力直接控制模式 */
    TorqueThrust    UMETA(DisplayName = "Torque & Thrust"),

    /** @brief 姿态与推力控制模式 */
    AttitudeThrust  UMETA(DisplayName = "Attitude & Thrust"),

    /** @brief 速度控制模式 */
    Velocity        UMETA(DisplayName = "Velocity"),

    /** @brief 位置控制模式 */
    Position        UMETA(DisplayName = "Position")
};

/**
 * @brief 偏航控制模式
 *
 * 与 AirSim `yaw_mode` 语义保持一致：
 * - `Auto` ：由系统自动决定偏航；
 * - `Hold` ：保持当前偏航角；
 * - `Angle`：指定绝对偏航角；
 * - `Rate` ：指定偏航角速度。
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
 * @brief 驱动模式
 *
 * 与 AirSim `drivetrain` 语义对齐：
 * - `ForwardOnly`：机头优先朝速度方向；
 * - `MaxDegreeOfFreedom`：允许完全自由机动。
 */
UENUM(BlueprintType)
enum class EDroneDrivetrainMode : uint8
{
    ForwardOnly        UMETA(DisplayName = "ForwardOnly"),
    MaxDegreeOfFreedom UMETA(DisplayName = "MaxDegreeOfFreedom")
};

/**
 * @brief 无人机物理参数结构体
 *
 * 该结构体对标 AirSim 多旋翼参数，覆盖：
 * 1. 质量与惯性；
 * 2. 机体几何尺寸；
 * 3. 推进系统空气动力学参数；
 * 4. 电机动态；
 * 5. 环境与碰撞参数；
 * 6. 由基础参数推导出的最大推力、最大扭矩和悬停转速等量。
 */
USTRUCT(BlueprintType)
struct GRADUATIONPROJECT_API FDroneParameters
{
    GENERATED_BODY()

    // ──── 质量 ────

    /** @brief 无人机总质量（kg，AirSim 默认约 1.0） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Mass")
    double Mass = 1.0;

    /** @brief 单个电机组件质量（kg，F450 近似值 0.055） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Mass")
    double MotorAssemblyWeight = 0.055;

    // ──── 惯性矩阵 ────

    /** @brief 绕 X 轴的转动惯量 Jx（kg·m²） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Inertia")
    double Jx = 0.0023;

    /** @brief 绕 Y 轴的转动惯量 Jy（kg·m²） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Inertia")
    double Jy = 0.0023;

    /** @brief 绕 Z 轴的转动惯量 Jz（kg·m²） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Inertia")
    double Jz = 0.004;

    // ──── 几何参数 ────

    /** @brief 机臂长度（m，F450 近似值 0.2275） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Geometry")
    double ArmLength = 0.2275;

    /** @brief QuadX 布局中电机相对机体系 X 轴夹角（度） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Geometry")
    double MotorAngle = 45.0;

    /** @brief 机体包围盒 X 尺寸（m，前后方向） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Geometry")
    double BodyBoxX = 0.180;

    /** @brief 机体包围盒 Y 尺寸（m，左右方向） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Geometry")
    double BodyBoxY = 0.11;

    /** @brief 机体包围盒 Z 尺寸（m，上下方向） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Geometry")
    double BodyBoxZ = 0.04;

    /** @brief 旋翼平面相对质心的 Z 偏移（m） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Geometry")
    double RotorZ = 0.025;

    // ──── 推进系统空气动力学参数 ────

    /**
     * @brief 推力系数 $C_T$
     *
     * 在经典螺旋桨模型中，最大推力满足：
     * $T = C_T \rho n^2 D^4$
     * 其中：
     * - $\rho$ 为空气密度；
     * - $n$ 为转速（rev/s）；
     * - $D$ 为螺旋桨直径。
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Propulsion")
    double C_T = 0.109919;

    /**
     * @brief 扭矩系数 $C_P$
     *
     * 旋翼反扭矩近似满足：
     * $Q = C_P \rho n^2 D^5 / (2\pi)$
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Propulsion")
    double C_P = 0.040164;

    /** @brief 最大转速（RPM） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Propulsion")
    double MaxRPM = 6396.667;

    /** @brief 螺旋桨直径（m） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Propulsion")
    double PropellerDiameter = 0.2286;

    /** @brief 螺旋桨等效厚度/高度（m） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Propulsion")
    double PropellerHeight = 0.01;

    // ──── 电机动态 ────

    /** @brief 电机一阶低通滤波时间常数（s） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Motor")
    double MotorFilterTC = 0.005;

    /** @brief 最小电机角速度（rad/s） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Motor")
    double MinMotorSpeed = 0.0;

    // ──── 环境参数 ────

    /** @brief 重力加速度（m/s²） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Environment")
    double Gravity = 9.81;

    /** @brief 空气密度（kg/m³） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Environment")
    double AirDensity = 1.225;

    /**
     * @brief 等效阻力系数
     *
     * 该值用于把多面体阻力模型折算为单一近似阻力系数，便于飞行动力学中使用。
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Environment")
    double DragCoefficient = 0.325;

    // ──── 碰撞参数 ────

    /** @brief 碰撞恢复系数（0 为完全非弹性，1 为完全弹性） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Collision")
    double Restitution = 0.55;

    /** @brief 碰撞摩擦系数 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Collision")
    double Friction = 0.5;

    // ──── 仿真参数 ────

    /** @brief 物理仿真时间步长（s） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Simulation")
    double TimeStep = 0.003;

    // ──── 派生参数 ────

    /** @brief 推力系数 $k_T$，满足 $T = k_T \omega^2$ */
    double ThrustCoefficient = 1.0e-5;

    /** @brief 反扭矩系数 $k_Q$，满足 $Q = k_Q \omega^2$ */
    double TorqueCoefficient = 1.0e-7;

    /** @brief 最大电机角速度（rad/s） */
    double MaxMotorSpeed = 669.85;

    /** @brief 单电机最大推力（N） */
    double MaxThrust = 4.179;

    /** @brief 单电机最大反扭矩（N·m） */
    double MaxTorque = 0.0556;

    /**
     * @brief 根据空气动力学参数计算派生推进量
     *
     * 计算公式：
     * - $T_{max} = C_T \rho n^2 D^4$
     * - $Q_{max} = C_P \rho n^2 D^5 / (2\pi)$
     * - $k_T = T_{max} / \omega_{max}^2$
     * - $k_Q = Q_{max} / \omega_{max}^2$
     */
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

    /**
     * @brief 根据机体尺寸和电机布局自动估算惯性矩阵对角项
     *
     * 近似把机体视为均质长方体、把四个电机视为质点，则：
     * - $J_x = \frac{m_{box}}{12}(B_y^2 + B_z^2) + \sum (y_i^2 + z_i^2)m_{motor}$
     * - $J_y = \frac{m_{box}}{12}(B_x^2 + B_z^2) + \sum (x_i^2 + z_i^2)m_{motor}$
     * - $J_z = \frac{m_{box}}{12}(B_x^2 + B_y^2) + \sum (x_i^2 + y_i^2)m_{motor}$
     */
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

    /**
     * @brief 计算悬停时的理论电机角速度
     * @return 悬停转速（rad/s）
     *
     * 由四旋翼悬停平衡条件：
     * $4 k_T \omega^2 = m g$
     * 故：
     * $\omega_{hover} = \sqrt{m g / (4 k_T)}$
     */
    double GetHoverMotorSpeed() const
    {
        if (ThrustCoefficient > KINDA_SMALL_NUMBER)
        {
            return FMath::Sqrt(Mass * Gravity / (4.0 * ThrustCoefficient));
        }
        return 0.0;
    }

    /**
     * @brief 初始化所有派生参数
     *
     * 在基础空气动力学参数发生变化后调用，用于重新计算推力、扭矩和惯性估计。
     */
    void InitializeComputed()
    {
        CalculateMaxThrust();
        ComputeInertiaMatrix();
    }
};
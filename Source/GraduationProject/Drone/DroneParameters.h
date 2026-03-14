// 解释：使用 `#pragma once` 防止该头文件在编译过程中被重复包含。
#pragma once
// 解释：引入 Unreal 的核心基础头文件，提供常用容器、数学类型和日志宏。
#include "CoreMinimal.h"

// 解释：引入 `DroneParameters.generated.h`，让 Unreal Header Tool 生成的反射代码在本文件中可见。
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
// 解释：使用 `UENUM` 宏把枚举注册给 Unreal 反射系统和蓝图。
UENUM(BlueprintType)
// 解释：这一行声明枚举 `EDroneControlMode`，用于约束一组有限的状态或模式取值。
enum class EDroneControlMode : uint8
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    /** @brief 空闲模式，不发送任何控制命令 */
    // 解释：这一行定义函数 `UMETA`，开始实现umeta的具体逻辑。
    Idle            UMETA(DisplayName = "Idle"),

    /** @brief 电机转速直接控制模式 */
    // 解释：这一行继续补充函数 `UMETA` 的参数列表、限定符或返回类型说明。
    MotorSpeed      UMETA(DisplayName = "Motor Speed"),

    /** @brief 力矩与推力直接控制模式 */
    // 解释：这一行继续补充函数 `UMETA` 的参数列表、限定符或返回类型说明。
    TorqueThrust    UMETA(DisplayName = "Torque & Thrust"),

    /** @brief 姿态与推力控制模式 */
    // 解释：这一行继续补充函数 `UMETA` 的参数列表、限定符或返回类型说明。
    AttitudeThrust  UMETA(DisplayName = "Attitude & Thrust"),

    /** @brief 速度控制模式 */
    // 解释：这一行继续补充函数 `UMETA` 的参数列表、限定符或返回类型说明。
    Velocity        UMETA(DisplayName = "Velocity"),

    /** @brief 位置控制模式 */
    // 解释：这一行收束函数 `UMETA` 的签名，后面会进入实现体或以分号结束声明。
    Position        UMETA(DisplayName = "Position")
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
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
// 解释：使用 `UENUM` 宏把枚举注册给 Unreal 反射系统和蓝图。
UENUM(BlueprintType)
// 解释：这一行声明枚举 `EDroneYawMode`，用于约束一组有限的状态或模式取值。
enum class EDroneYawMode : uint8
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行定义函数 `UMETA`，开始实现umeta的具体逻辑。
    Auto    UMETA(DisplayName = "Auto"),
    // 解释：这一行继续补充函数 `UMETA` 的参数列表、限定符或返回类型说明。
    Hold    UMETA(DisplayName = "Hold"),
    // 解释：这一行继续补充函数 `UMETA` 的参数列表、限定符或返回类型说明。
    Angle   UMETA(DisplayName = "Angle"),
    // 解释：这一行收束函数 `UMETA` 的签名，后面会进入实现体或以分号结束声明。
    Rate    UMETA(DisplayName = "Rate")
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
};

/**
 * @brief 驱动模式
 *
 * 与 AirSim `drivetrain` 语义对齐：
 * - `ForwardOnly`：机头优先朝速度方向；
 * - `MaxDegreeOfFreedom`：允许完全自由机动。
 */
// 解释：使用 `UENUM` 宏把枚举注册给 Unreal 反射系统和蓝图。
UENUM(BlueprintType)
// 解释：这一行声明枚举 `EDroneDrivetrainMode`，用于约束一组有限的状态或模式取值。
enum class EDroneDrivetrainMode : uint8
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行定义函数 `UMETA`，开始实现umeta的具体逻辑。
    ForwardOnly        UMETA(DisplayName = "ForwardOnly"),
    // 解释：这一行收束函数 `UMETA` 的签名，后面会进入实现体或以分号结束声明。
    MaxDegreeOfFreedom UMETA(DisplayName = "MaxDegreeOfFreedom")
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
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
// 解释：使用 `USTRUCT` 宏声明可被 Unreal 反射系统识别的结构体类型。
USTRUCT(BlueprintType)
// 解释：这一行声明 结构体 `FDroneParameters`，用于封装fdrone参数相关的数据与行为。
struct GRADUATIONPROJECT_API FDroneParameters
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：展开 Unreal Header Tool 生成的样板代码，这是 UObject、Actor、组件等类型正常工作的基础。
    GENERATED_BODY()

    // ──── 质量 ────

    /** @brief 无人机总质量（kg，AirSim 默认约 1.0） */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Mass")
    // 解释：这一行声明成员或局部变量 `Mass`，用于保存mass。
    double Mass = 1.0;

    /** @brief 单个电机组件质量（kg，F450 近似值 0.055） */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Mass")
    // 解释：这一行声明成员或局部变量 `MotorAssemblyWeight`，用于保存motorassemblyweight。
    double MotorAssemblyWeight = 0.055;

    // ──── 惯性矩阵 ────

    /** @brief 绕 X 轴的转动惯量 Jx（kg·m²） */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Inertia")
    // 解释：这一行声明成员或局部变量 `Jx`，用于保存jx。
    double Jx = 0.0023;

    /** @brief 绕 Y 轴的转动惯量 Jy（kg·m²） */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Inertia")
    // 解释：这一行声明成员或局部变量 `Jy`，用于保存jy。
    double Jy = 0.0023;

    /** @brief 绕 Z 轴的转动惯量 Jz（kg·m²） */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Inertia")
    // 解释：这一行声明成员或局部变量 `Jz`，用于保存jz。
    double Jz = 0.004;

    // ──── 几何参数 ────

    /** @brief 机臂长度（m，F450 近似值 0.2275） */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Geometry")
    // 解释：这一行声明成员或局部变量 `ArmLength`，用于保存armlength。
    double ArmLength = 0.2275;

    /** @brief QuadX 布局中电机相对机体系 X 轴夹角（度） */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Geometry")
    // 解释：这一行声明成员或局部变量 `MotorAngle`，用于保存motorangle。
    double MotorAngle = 45.0;

    /** @brief 机体包围盒 X 尺寸（m，前后方向） */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Geometry")
    // 解释：这一行声明成员或局部变量 `BodyBoxX`，用于保存bodyboxX。
    double BodyBoxX = 0.180;

    /** @brief 机体包围盒 Y 尺寸（m，左右方向） */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Geometry")
    // 解释：这一行声明成员或局部变量 `BodyBoxY`，用于保存bodyboxY。
    double BodyBoxY = 0.11;

    /** @brief 机体包围盒 Z 尺寸（m，上下方向） */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Geometry")
    // 解释：这一行声明成员或局部变量 `BodyBoxZ`，用于保存bodyboxZ。
    double BodyBoxZ = 0.04;

    /** @brief 旋翼平面相对质心的 Z 偏移（m） */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Geometry")
    // 解释：这一行声明成员或局部变量 `RotorZ`，用于保存rotorZ。
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
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Propulsion")
    // 解释：这一行声明成员或局部变量 `C_T`，用于保存CT。
    double C_T = 0.109919;

    /**
     * @brief 扭矩系数 $C_P$
     *
     * 旋翼反扭矩近似满足：
     * $Q = C_P \rho n^2 D^5 / (2\pi)$
     */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Propulsion")
    // 解释：这一行声明成员或局部变量 `C_P`，用于保存CP。
    double C_P = 0.040164;

    /** @brief 最大转速（RPM） */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Propulsion")
    // 解释：这一行声明成员或局部变量 `MaxRPM`，用于保存maxrpm。
    double MaxRPM = 6396.667;

    /** @brief 螺旋桨直径（m） */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Propulsion")
    // 解释：这一行声明成员或局部变量 `PropellerDiameter`，用于保存propellerdiameter。
    double PropellerDiameter = 0.2286;

    /** @brief 螺旋桨等效厚度/高度（m） */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Propulsion")
    // 解释：这一行声明成员或局部变量 `PropellerHeight`，用于保存propellerheight。
    double PropellerHeight = 0.01;

    // ──── 电机动态 ────

    /** @brief 电机一阶低通滤波时间常数（s） */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Motor")
    // 解释：这一行声明成员或局部变量 `MotorFilterTC`，用于保存motorfiltertc。
    double MotorFilterTC = 0.005;

    /** @brief 最小电机角速度（rad/s） */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Motor")
    // 解释：这一行声明成员或局部变量 `MinMotorSpeed`，用于保存minmotorspeed。
    double MinMotorSpeed = 0.0;

    // ──── 环境参数 ────

    /** @brief 重力加速度（m/s²） */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Environment")
    // 解释：这一行声明成员或局部变量 `Gravity`，用于保存gravity。
    double Gravity = 9.81;

    /** @brief 空气密度（kg/m³） */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Environment")
    // 解释：这一行声明成员或局部变量 `AirDensity`，用于保存空中density。
    double AirDensity = 1.225;

    /**
     * @brief 等效阻力系数
     *
     * 该值用于把多面体阻力模型折算为单一近似阻力系数，便于飞行动力学中使用。
     */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Environment")
    // 解释：这一行声明成员或局部变量 `DragCoefficient`，用于保存dragcoefficient。
    double DragCoefficient = 0.325;

    // ──── 碰撞参数 ────

    /** @brief 碰撞恢复系数（0 为完全非弹性，1 为完全弹性） */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Collision")
    // 解释：这一行声明成员或局部变量 `Restitution`，用于保存restitution。
    double Restitution = 0.55;

    /** @brief 碰撞摩擦系数 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Collision")
    // 解释：这一行声明成员或局部变量 `Friction`，用于保存friction。
    double Friction = 0.5;

    // ──── 仿真参数 ────

    /** @brief 物理仿真时间步长（s） */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters|Simulation")
    // 解释：这一行声明成员或局部变量 `TimeStep`，用于保存采样时间步长。
    double TimeStep = 0.003;

    // ──── 派生参数 ────

    /** @brief 推力系数 $k_T$，满足 $T = k_T \omega^2$ */
    // 解释：这一行声明成员或局部变量 `ThrustCoefficient`，用于保存thrustcoefficient。
    double ThrustCoefficient = 1.0e-5;

    /** @brief 反扭矩系数 $k_Q$，满足 $Q = k_Q \omega^2$ */
    // 解释：这一行声明成员或局部变量 `TorqueCoefficient`，用于保存torquecoefficient。
    double TorqueCoefficient = 1.0e-7;

    /** @brief 最大电机角速度（rad/s） */
    // 解释：这一行声明成员或局部变量 `MaxMotorSpeed`，用于保存maxmotorspeed。
    double MaxMotorSpeed = 669.85;

    /** @brief 单电机最大推力（N） */
    // 解释：这一行声明成员或局部变量 `MaxThrust`，用于保存maxthrust。
    double MaxThrust = 4.179;

    /** @brief 单电机最大反扭矩（N·m） */
    // 解释：这一行声明成员或局部变量 `MaxTorque`，用于保存maxtorque。
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
    // 解释：这一行定义函数 `CalculateMaxThrust`，开始实现calculatemaxthrust的具体逻辑。
    void CalculateMaxThrust()
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行声明成员或局部变量 `RevPerSec`，用于保存revpersec。
        const double RevPerSec = MaxRPM / 60.0;
        // 解释：这一行把右侧表达式的结果写入 `MaxMotorSpeed`，完成 maxmotorspeed 的更新。
        MaxMotorSpeed = RevPerSec * 2.0 * PI;
        // 解释：这一行声明成员或局部变量 `nSquared`，用于保存Nsquared。
        const double nSquared = RevPerSec * RevPerSec;
        // 解释：这一行把右侧表达式的结果写入 `const double D4`，完成 constdoubled4 的更新。
        const double D4 = FMath::Pow(PropellerDiameter, 4.0);
        // 解释：这一行把右侧表达式的结果写入 `const double D5`，完成 constdoubled5 的更新。
        const double D5 = FMath::Pow(PropellerDiameter, 5.0);
        // 解释：这一行把右侧表达式的结果写入 `MaxThrust`，完成 maxthrust 的更新。
        MaxThrust = C_T * AirDensity * nSquared * D4;
        // 解释：这一行把右侧表达式的结果写入 `MaxTorque`，完成 maxtorque 的更新。
        MaxTorque = C_P * AirDensity * nSquared * D5 / (2.0 * PI);

        // 解释：这一行声明成员或局部变量 `MaxSpeedSquared`，用于保存maxspeedsquared。
        const double MaxSpeedSquared = MaxMotorSpeed * MaxMotorSpeed;
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (MaxSpeedSquared > KINDA_SMALL_NUMBER)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行把右侧表达式的结果写入 `ThrustCoefficient`，完成 thrustcoefficient 的更新。
            ThrustCoefficient = MaxThrust / MaxSpeedSquared;
            // 解释：这一行把右侧表达式的结果写入 `TorqueCoefficient`，完成 torquecoefficient 的更新。
            TorqueCoefficient = MaxTorque / MaxSpeedSquared;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    /**
     * @brief 根据机体尺寸和电机布局自动估算惯性矩阵对角项
     *
     * 近似把机体视为均质长方体、把四个电机视为质点，则：
     * - $J_x = \frac{m_{box}}{12}(B_y^2 + B_z^2) + \sum (y_i^2 + z_i^2)m_{motor}$
     * - $J_y = \frac{m_{box}}{12}(B_x^2 + B_z^2) + \sum (x_i^2 + z_i^2)m_{motor}$
     * - $J_z = \frac{m_{box}}{12}(B_x^2 + B_y^2) + \sum (x_i^2 + y_i^2)m_{motor}$
     */
    // 解释：这一行定义函数 `ComputeInertiaMatrix`，开始实现computeinertiamatrix的具体逻辑。
    void ComputeInertiaMatrix()
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行声明成员或局部变量 `BoxMass`，用于保存boxmass。
        double BoxMass = Mass - 4.0 * MotorAssemblyWeight;
        // 解释：这一行通过 `FMath::Max` 给 `BoxMass` 施加下界约束，避免 boxmass 过小。
        BoxMass = FMath::Max(BoxMass, 0.1);

        // 解释：这一行把右侧表达式的结果写入 `Jx`，完成 jx 的更新。
        Jx = BoxMass / 12.0 * (BodyBoxY * BodyBoxY + BodyBoxZ * BodyBoxZ);
        // 解释：这一行把右侧表达式的结果写入 `Jy`，完成 jy 的更新。
        Jy = BoxMass / 12.0 * (BodyBoxX * BodyBoxX + BodyBoxZ * BodyBoxZ);
        // 解释：这一行把右侧表达式的结果写入 `Jz`，完成 jz 的更新。
        Jz = BoxMass / 12.0 * (BodyBoxX * BodyBoxX + BodyBoxY * BodyBoxY);

        // 解释：这一行把右侧表达式的结果写入 `const double SinA`，完成 constdoublesinA 的更新。
        const double SinA = FMath::Sin(FMath::DegreesToRadians(MotorAngle));
        // 解释：这一行把右侧表达式的结果写入 `const double CosA`，完成 constdoublecosA 的更新。
        const double CosA = FMath::Cos(FMath::DegreesToRadians(MotorAngle));
        // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
        for (int32 i = 0; i < 4; ++i)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行把右侧表达式的结果写入 `const double Sign1`，完成 constdoublesign1 的更新。
            const double Sign1 = (i == 0 || i == 3) ? 1.0 : -1.0;
            // 解释：这一行把右侧表达式的结果写入 `const double Sign2`，完成 constdoublesign2 的更新。
            const double Sign2 = (i < 2) ? 1.0 : -1.0;
            // 解释：这一行声明成员或局部变量 `mx`，用于保存mx。
            const double mx = ArmLength * CosA * Sign2;
            // 解释：这一行声明成员或局部变量 `my`，用于保存my。
            const double my = ArmLength * SinA * Sign1;
            // 解释：这一行声明成员或局部变量 `mz`，用于保存mz。
            const double mz = RotorZ;

            // 解释：这一行在 `Jx` 的原有基础上继续累加新量，用于持续更新 jx。
            Jx += (my * my + mz * mz) * MotorAssemblyWeight;
            // 解释：这一行在 `Jy` 的原有基础上继续累加新量，用于持续更新 jy。
            Jy += (mx * mx + mz * mz) * MotorAssemblyWeight;
            // 解释：这一行在 `Jz` 的原有基础上继续累加新量，用于持续更新 jz。
            Jz += (mx * mx + my * my) * MotorAssemblyWeight;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
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
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    double GetHoverMotorSpeed() const
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (ThrustCoefficient > KINDA_SMALL_NUMBER)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return FMath::Sqrt(Mass * Gravity / (4.0 * ThrustCoefficient));
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return 0.0;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    /**
     * @brief 初始化所有派生参数
     *
     * 在基础空气动力学参数发生变化后调用，用于重新计算推力、扭矩和惯性估计。
     */
    // 解释：这一行定义函数 `InitializeComputed`，开始实现initializecomputed的具体逻辑。
    void InitializeComputed()
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `CalculateMaxThrust` 执行当前步骤需要的功能逻辑。
        CalculateMaxThrust();
        // 解释：调用 `ComputeInertiaMatrix` 执行当前步骤需要的功能逻辑。
        ComputeInertiaMatrix();
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
};

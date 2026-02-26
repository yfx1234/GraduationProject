/**
 * @file DroneMovementComponent.h
 * @brief 无人机运动仿真组件的头文件
 *
 * 本文件定义了 UDroneMovementComponent 类，它是整个无人机仿真的核心组件，
 * 负责两大功能：
 *
 * 1. **级联 PID 控制器** — 将高层命令（位置/速度）逐级转换为电机转速：
 *    位置(PD) → 速度(PID) → 姿态(PD) → 角速率(PID) → 力矩 → 电机转速
 *
 * 2. **RK4 动力学仿真** — 基于刚体动力学方程（牛顿-欧拉方程）
 *    使用四阶 Runge-Kutta 法进行数值积分，更新 13 维状态向量。
 *
 * 参考 AirSim 的 SimpleFlightQuadRotorPhysicsBody 和 CascadedPID 设计。
 */

#pragma once
#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "DroneState.h"
#include "DroneParameters.h"
#include "DroneMovementComponent.generated.h"

class UPDController;
class UPIDController;

/**
 * 无人机运动仿真组件
 *
 * 控制架构（4层级联）：
 * @code
 *   [外环]                      [内环]
 *   Position(PD) → Velocity(PID) → Attitude(PD) → AngularRate(PID)
 *         ↓              ↓               ↓               ↓
 *      速度命令       加速度命令      角速率命令        力矩命令
 *                                                        ↓
 *                                               CalculateMotorSpeeds()
 *                                                        ↓
 *                                                   电机转速 ω[4]
 * @endcode
 *
 * 动力学模型：
 * - 推力模型：F = kT * ω²，力矩模型：τ = kQ * ω²
 * - 控制分配：G 矩阵（4×4）将 [推力, τx, τy, τz] → [ω₁², ω₂², ω₃², ω₄²]
 * - 状态积分：RK4 四阶积分，支持动态子步细分
 */
UCLASS(ClassGroup=(CustomDrone), meta=(BlueprintSpawnableComponent))
class GRADUATIONPROJECT_API UDroneMovementComponent : public UActorComponent
{
    GENERATED_BODY()

public:
    /** @brief 构造函数，初始化默认控制模式和命令 */
    UDroneMovementComponent();

    /**
     * @brief 组件初始化
     *
     * 创建所有 PD/PID 控制器实例，计算控制分配矩阵。
     */
    virtual void BeginPlay() override;

    /**
     * @brief 每帧 Tick — 执行控制更新和物理仿真
     * @param DeltaTime 帧间隔时间（秒）
     * @param TickType Tick 类型
     * @param ThisTickFunction Tick 函数引用
     *
     * 流程：
     * 1. ControlUpdate() — 根据控制模式运行级联控制器
     * 2. RK4 子步仿真 — 将帧时间细分为多个子步进行积分
     */
    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

    // ---- 设置接口 ----

    /**
     * @brief 设置初始状态
     * @param InitialState 初始无人机状态
     *
     * 如果电机转速未设置，自动填充为悬停转速。
     */
    UFUNCTION(BlueprintCallable, Category = "DroneMovement|Setup")
    void SetInitialState(const FDroneState& InitialState);

    /**
     * @brief 设置物理参数并重新计算控制分配矩阵
     * @param NewParameters 新的物理参数
     */
    UFUNCTION(BlueprintCallable, Category = "DroneMovement|Setup")
    void SetParameters(const FDroneParameters& NewParameters);

    /**
     * @brief 切换控制模式，同时重置所有控制器状态
     * @param NewMode 新的控制模式
     */
    UFUNCTION(BlueprintCallable, Category = "DroneMovement|Setup")
    void SetControlMode(EDroneControlMode NewMode);

    // ---- 控制命令接口 ----

    /**
     * @brief 设置原始控制命令（MotorSpeed/TorqueThrust 模式使用）
     * @param Command 命令数组（长度 ≥ 4）
     */
    UFUNCTION(BlueprintCallable, Category = "DroneMovement|Control")
    void SetControlCommand(const TArray<double>& Command);

    /**
     * @brief 设置目标位置（Position 模式使用）
     * @param TargetPos 目标位置 (SI, 米)
     */
    UFUNCTION(BlueprintCallable, Category = "DroneMovement|Control")
    void SetTargetPosition(const FVector& TargetPos);

    /**
     * @brief 设置目标速度（Velocity 模式使用）
     * @param TargetVel 目标速度 (SI, m/s)
     */
    UFUNCTION(BlueprintCallable, Category = "DroneMovement|Control")
    void SetTargetVelocity(const FVector& TargetVel);

    /**
     * @brief 设置目标姿态和推力（AttitudeThrust 模式使用）
     * @param Attitude 目标姿态 (Roll, Pitch, Yaw) 度
     * @param Thrust 推力系数（0~1，1 = 悬停推力）
     */
    UFUNCTION(BlueprintCallable, Category = "DroneMovement|Control")
    void SetTargetAttitude(const FRotator& Attitude, float Thrust);

    // ---- 状态查询 ----

    /**
     * @brief 获取当前无人机状态
     * @return 当前状态的拷贝
     */
    UFUNCTION(BlueprintPure, Category = "DroneMovement|State")
    FDroneState GetCurrentState() const { return CurrentState; }

    /**
     * @brief 重置状态并清零所有控制器
     * @param NewState 新的初始状态
     */
    UFUNCTION(BlueprintCallable, Category = "DroneMovement|State")
    void ResetState(const FDroneState& NewState);

    // ---- PID 增益运行时调参（供 DroneApi 使用） ----

    /**
     * @brief 设置位置控制器(PD)增益
     * @param Kp 比例增益
     * @param Kd 微分增益
     */
    void SetPositionGains(float Kp, float Kd);

    /**
     * @brief 设置速度控制器(PID)增益
     * @param Kp 比例增益
     * @param Ki 积分增益
     * @param Kd 微分增益
     */
    void SetVelocityGains(float Kp, float Ki, float Kd);

    /**
     * @brief 设置姿态控制器(PD)增益
     * @param Kp 比例增益（Yaw 轴自动使用 Kp*0.5）
     * @param Kd 微分增益
     */
    void SetAttitudeGains(float Kp, float Kd);

    /**
     * @brief 设置角速率控制器增益
     * @param Kp 比例增益（Yaw 轴自动使用 Kp*0.5）
     */
    void SetAngleRateGains(float Kp);

public:
    /** @brief 无人机物理参数 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DroneMovement|Parameters")
    FDroneParameters Parameters;

    /** @brief 当前控制模式 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DroneMovement|Control")
    EDroneControlMode CurrentControlMode;

    /** @brief 当前无人机状态（13 维状态向量 + 电机转速） */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "DroneMovement|State")
    FDroneState CurrentState;

    /** @brief 组件是否已完成初始化 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "DroneMovement|State")
    bool bInitialized = false;

protected:
    // ---- 控制更新 ----

    /**
     * @brief 执行一帧的控制更新
     * @param DeltaTime 帧间隔
     *
     * 根据 CurrentControlMode 运行对应的控制环路链，
     * 最终计算出电机转速写入 CurrentState.MotorSpeeds。
     */
    void ControlUpdate(double DeltaTime);
    
    // ---- 级联控制环路 ----

    /**
     * @brief 位置控制环（外环，PD控制器）
     * @param PositionCommand 目标位置 (m)
     * @return 速度命令 (m/s)，传给 VelocityLoop()
     */
    FVector PositionLoop(const FVector& PositionCommand);

    /**
     * @brief 速度控制环（PID控制器）
     * @param VelocityCommand 目标速度 (m/s)
     * @param OutThrust 输出：计算得到的总推力 (N)，包含重力补偿和姿态补偿
     * @return 加速度命令 (m/s²)，传给 AttitudeLoop()
     */
    FVector VelocityLoop(const FVector& VelocityCommand, double& OutThrust);

    /**
     * @brief 姿态控制环（PD控制器）
     * @param AccelerationCommand 期望加速度 (m/s²)，用于计算期望倾斜角
     * @return 角速率命令 (rad/s)，传给 AngularVelocityLoop()
     *
     * 从期望加速度计算期望 Roll/Pitch 角度：
     *   RollDes = atan2(-Ay, g)
     *   PitchDes = atan2(Ax, g)
     * 限制在 ±17.2°（0.3 rad）避免过度倾斜。
     */
    FVector AttitudeLoop(const FVector& AccelerationCommand);

    /**
     * @brief 角速率控制环（内环，PID控制器）
     * @param AngularVelocityCommand 目标角速率 (rad/s)
     * @return 力矩命令 (N·m)，传给 CalculateMotorSpeeds()
     */
    FVector AngularVelocityLoop(const FVector& AngularVelocityCommand);

    /**
     * @brief 从力矩和推力命令计算电机转速
     * @param TorqueCommand 三轴力矩命令 (N·m)
     * @param Thrust 总推力命令 (N)
     * @return 四个电机的转速 (rad/s)
     *
     * 使用控制分配逆矩阵 GInv：
     *   ω²[4] = GInv × [Thrust, τx, τy, τz]
     */
    TArray<double> CalculateMotorSpeeds(const FVector& TorqueCommand, double Thrust);

    // ---- 动力学仿真 ----

    /**
     * @brief RK4 四阶 Runge-Kutta 积分一步
     * @param State 当前状态
     * @param TimeStep 积分步长（秒）
     * @return 积分后的新状态
     *
     * 经典 RK4 算法：
     *   k1 = f(t, y)
     *   k2 = f(t+h/2, y+h/2*k1)
     *   k3 = f(t+h/2, y+h/2*k2)
     *   k4 = f(t+h, y+h*k3)
     *   y(t+h) = y(t) + h/6*(k1 + 2*k2 + 2*k3 + k4)
     */
    FDroneState RK4Update(const FDroneState& State, double TimeStep);

    /**
     * @brief 计算给定状态下的合外力和合力矩
     * @param State 当前状态
     * @param OutTotalForce 输出：世界坐标系下的合外力 [Fx, Fy, Fz] (N)
     * @param OutTorque 输出：机体坐标系下的合力矩 [τx, τy, τz] (N·m)
     *
     * 合外力 = 推力（机体→世界变换） + 重力 + 空气阻力
     * 合力矩 = 通过控制分配矩阵 G 从电机转速计算
     */
    void CalculateTotalForcesAndTorques(const FDroneState& State, TArray<double>& OutTotalForce, TArray<double>& OutTorque);

    /**
     * @brief 计算状态导数向量（13维）
     * @param State 当前状态
     * @param TotalForce 世界坐标系合外力
     * @param Torque 机体坐标系合力矩
     * @return 13 维导数向量 [ẋ, ẏ, ż, v̇x, v̇y, v̇z, q̇w, q̇x, q̇y, q̇z, ṗ, q̇, ṙ]
     *
     * 动力学方程：
     * - 位置导数 = 速度
     * - 速度导数 = 合力 / 质量
     * - 四元数导数 = ½ * Ω(ω) * q （四元数运动学方程）
     * - 角速度导数 = (τ - ω × Jω) / J （欧拉旋转方程）
     */
    TArray<double> Derivatives(const FDroneState& State, const TArray<double>& TotalForce, const TArray<double>& Torque);

    /**
     * @brief 状态向量加法：State + Derivative * dt
     * @param State 基准状态
     * @param Derivative 导数向量（13维）
     * @param dt 时间步长
     * @return 更新后的状态
     */
    FDroneState StateAdd(const FDroneState& State, const TArray<double>& Derivative, double dt);

    // ---- 辅助函数 ----

    /**
     * @brief 将机体坐标系向量旋转到世界坐标系
     * @param BodyVector 机体坐标系向量
     * @param Orientation 姿态四元数
     * @return 世界坐标系向量
     */
    FVector RotateBodyToWorld(const FVector& BodyVector, const FQuat& Orientation);

    /** @brief 创建并初始化所有 PD/PID 控制器实例 */
    void InitializeControllers();

    /** @brief 重置所有控制器的内部状态（误差历史、积分器） */
    void ResetAllControllers();

    /**
     * @brief 角度归一化到 [-π, π] 范围
     * @param AngleRad 输入角度（弧度）
     * @return 归一化后的角度
     */
    static double NormalizeAngle(double AngleRad);

    /**
     * @brief 计算控制分配矩阵 G 及其逆矩阵 GInv
     *
     * G 矩阵将电机转速的平方映射到推力和力矩：
     *   [T, τx, τy, τz]ᵀ = G × [ω₁², ω₂², ω₃², ω₄²]ᵀ
     *
     * GInv 是其逆（X 型四旋翼有解析解），用于从期望推力/力矩
     * 反算电机转速。
     */
    void ComputeControlAllocation();

private:
    /** @brief 原始控制命令（MotorSpeed/TorqueThrust 模式使用） */
    TArray<double> ControlCommands;

    /** @brief 目标位置（Position 模式使用，SI 米） */
    FVector TargetPosition;

    /** @brief 目标速度（Velocity 模式使用，SI m/s） */
    FVector TargetVelocity;

    /** @brief 目标姿态（AttitudeThrust 模式使用） */
    FRotator TargetAttitude;

    /** @brief 目标推力（AttitudeThrust 模式使用，N） */
    double TargetThrust;

    // ---- 位置控制器 (PD, 3轴) ----
    UPROPERTY() UPDController* PxController;    ///< X 轴位置控制器
    UPROPERTY() UPDController* PyController;    ///< Y 轴位置控制器
    UPROPERTY() UPDController* PzController;    ///< Z 轴位置控制器

    // ---- 速度控制器 (PID, 3轴) ----
    UPROPERTY() UPIDController* VxController;   ///< X 轴速度控制器
    UPROPERTY() UPIDController* VyController;   ///< Y 轴速度控制器
    UPROPERTY() UPIDController* VzController;   ///< Z 轴速度控制器

    // ---- 姿态控制器 (PD, 3轴) ----
    UPROPERTY() UPDController* RollController;  ///< 滚转角控制器
    UPROPERTY() UPDController* PitchController; ///< 俯仰角控制器
    UPROPERTY() UPDController* YawController;   ///< 偏航角控制器

    // ---- 角速率控制器 (PID, 3轴) ----
    UPROPERTY() UPIDController* RollRateController;   ///< 滚转角速率控制器
    UPROPERTY() UPIDController* PitchRateController;  ///< 俯仰角速率控制器
    UPROPERTY() UPIDController* YawRateController;    ///< 偏航角速率控制器

    /**
     * @brief 控制分配矩阵 G (4×4)
     *
     * 将电机转速平方向量映射为推力和力矩向量：
     * G[0][i] = kT (推力行)
     * G[1][i] = 滚转力矩贡献
     * G[2][i] = 俯仰力矩贡献
     * G[3][i] = 偏航力矩贡献
     */
    double G[4][4];

    /**
     * @brief 控制分配逆矩阵 GInv (4×4)
     *
     * X 型四旋翼的解析逆，用于从 [T, τx, τy, τz] 反算 ω²。
     */
    double GInv[4][4];
};

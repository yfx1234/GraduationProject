/**
 * @file DroneMovementComponent.cpp
 * @brief 无人机运动仿真组件的实现文件
 *
 * 实现四旋翼无人机的完整仿真：
 * 1. 级联 PID 控制器：Position(PD) → Velocity(PID) → Attitude(PD) → AngRate(PID)
 * 2. 控制分配：力矩+推力 → 电机转速（X型四旋翼解析逆）
 * 3. RK4 动力学积分：推力/力矩/重力/阻力 → 13维状态更新
 */

#include "DroneMovementComponent.h"
#include "GraduationProject/Core/Controller/PDController.h"
#include "GraduationProject/Core/Controller/PIDController.h"

/**
 * @brief 构造函数
 *
 * 设置 Tick 在 PrePhysics 阶段执行（在 UE 物理引擎之前），
 * 初始化控制模式为 Idle，命令为零。
 */
UDroneMovementComponent::UDroneMovementComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
    PrimaryComponentTick.TickGroup = TG_PrePhysics; // 在物理引擎之前执行
    CurrentControlMode = EDroneControlMode::Idle;
    ControlCommands = {0.0, 0.0, 0.0, 0.0};
    TargetPosition = FVector::ZeroVector;
    TargetVelocity = FVector::ZeroVector;
    TargetAttitude = FRotator::ZeroRotator;
    TargetThrust = 0.0;
}

/**
 * @brief 组件初始化
 *
 * 创建 12 个 PD/PID 控制器实例，计算控制分配矩阵及其逆矩阵。
 */
void UDroneMovementComponent::BeginPlay()
{
    Super::BeginPlay();
    InitializeControllers();
    ComputeControlAllocation();
    bInitialized = true;
}

/**
 * @brief 每帧 Tick — 控制更新 + RK4 仿真
 * @param DeltaTime 帧间隔时间
 *
 * 流程：
 * 1. 跳过未初始化或 Idle 模式
 * 2. ControlUpdate() — 运行级联控制器计算电机转速
 * 3. 将帧时间按 Parameters.TimeStep 细分为多个子步
 * 4. 每个子步执行一次 RK4 积分
 */
void UDroneMovementComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
    if (!bInitialized) return;
    if (CurrentControlMode == EDroneControlMode::Idle) return;

    // 第一步：运行级联控制器，计算电机转速
    ControlUpdate(DeltaTime);

    // 第二步：RK4 子步仿真
    // 根据帧时间和目标步长动态计算子步数，保证数值稳定性
    double SubStepTime = Parameters.TimeStep;
    int32 NumSubSteps = FMath::Max(1, FMath::CeilToInt(DeltaTime / SubStepTime));
    SubStepTime = DeltaTime / NumSubSteps; // 均分帧时间

    for (int32 i = 0; i < NumSubSteps; ++i)
    {
        CurrentState = RK4Update(CurrentState, SubStepTime);
    }
}

// ====================================================================
// 初始化
// ====================================================================

/**
 * @brief 设置物理参数并重新初始化
 * @param NewParameters 新的物理参数
 *
 * 更新参数后重新计算控制分配矩阵和重新初始化控制器。
 */
void UDroneMovementComponent::SetParameters(const FDroneParameters& NewParameters)
{
    Parameters = NewParameters;
    ComputeControlAllocation();
    InitializeControllers();
}

/**
 * @brief 创建并初始化所有 PD/PID 控制器
 *
 * 控制器参数设计（参考 AirSim CascadedPID）：
 *
 * 位置环(PD)：Kp=1.0, 输出限幅 2.0 m/s（限制最大速度命令避免激进机动）
 * 速度环(PID)：Kp=2.0, Ki=0.2, 输出限幅 3.0 m/s²，积分器 ±5.0
 * 姿态环(PD)：Kp=10.0（Yaw 减半为 5.0），输出限幅 3.0 rad/s
 * 角速率环(PID)：Kp=0.5（Yaw 减半为 0.25），输出限幅 1.0 N·m（Yaw 0.5）
 */
void UDroneMovementComponent::InitializeControllers()
{
    double Ts = Parameters.TimeStep;

    // ---- 位置控制器 (PD) ----
    float PosPGain = 1.0f;
    float VelMaxLimit = 2.0f;  // 最大速度命令 m/s（降低以减少激进倾斜）
    
    PxController = NewObject<UPDController>(this);
    PxController->Initialize(PosPGain, 0.0, 0.0, VelMaxLimit, Ts);
    
    PyController = NewObject<UPDController>(this);
    PyController->Initialize(PosPGain, 0.0, 0.0, VelMaxLimit, Ts);
    
    PzController = NewObject<UPDController>(this);
    PzController->Initialize(PosPGain, 0.0, 0.0, VelMaxLimit, Ts);

    // ---- 速度控制器 (PID) ----
    float VelPGain = 2.0f;
    float VelDGain = 0.0f;
    
    VxController = NewObject<UPIDController>(this);
    VxController->Initialize(VelPGain, 0.2, VelDGain, 0.05, 3.0, Ts, -5.0, 5.0);
    
    VyController = NewObject<UPIDController>(this);
    VyController->Initialize(VelPGain, 0.2, VelDGain, 0.05, 3.0, Ts, -5.0, 5.0);
    
    VzController = NewObject<UPIDController>(this);
    VzController->Initialize(VelPGain, 0.2, VelDGain, 0.05, 3.0, Ts, -5.0, 5.0);

    // ---- 姿态控制器 (PD) ----
    float AnglePGain = 10.0f;
    float AngVelMaxLimit = 3.0f;
    
    RollController = NewObject<UPDController>(this);
    RollController->Initialize(AnglePGain, 0.0, 0.0, AngVelMaxLimit, Ts);
    
    PitchController = NewObject<UPDController>(this);
    PitchController->Initialize(AnglePGain, 0.0, 0.0, AngVelMaxLimit, Ts);
    
    // Yaw 控制器减半增益（偏航响应不需要太快）
    YawController = NewObject<UPDController>(this);
    YawController->Initialize(AnglePGain * 0.5f, 0.0, 0.0, AngVelMaxLimit, Ts);

    // ---- 角速率控制器 (PID) ----
    float RatePGain = 0.5f;
    float TorqueMaxLimit = 1.0f;
    
    RollRateController = NewObject<UPIDController>(this);
    RollRateController->Initialize(RatePGain, 0.0, 0.0, 0.05, TorqueMaxLimit, Ts, -2.0, 2.0);
    
    PitchRateController = NewObject<UPIDController>(this);
    PitchRateController->Initialize(RatePGain, 0.0, 0.0, 0.05, TorqueMaxLimit, Ts, -2.0, 2.0);
    
    // Yaw 角速率控制器减半增益和限幅
    YawRateController = NewObject<UPIDController>(this);
    YawRateController->Initialize(RatePGain * 0.5f, 0.0, 0.0, 0.05, TorqueMaxLimit * 0.5f, Ts, -1.0, 1.0);
}

/**
 * @brief 计算控制分配矩阵 G 及其逆矩阵 GInv
 *
 * X 型四旋翼布局（俯视）：
 * @code
 *       前方(+X)
 *    M0 (+)    M1 (-)
 *        \    /
 *         \/
 *         /\
 *        /  \
 *    M3 (-)    M2 (+)
 *       后方
 * @endcode
 *
 * (+/-) 表示旋转方向：M0/M2 逆时针，M1/M3 顺时针
 *
 * G 矩阵将 [ω₁², ..., ω₄²] 映射为 [T, τ_roll, τ_pitch, τ_yaw]
 * GInv 是其解析逆（X型布局有解析解）
 */
void UDroneMovementComponent::ComputeControlAllocation()
{
    double kT = Parameters.ThrustCoefficient;
    double kQ = Parameters.TorqueCoefficient;
    double L = Parameters.ArmLength;
    double Beta = FMath::DegreesToRadians(Parameters.MotorAngle); // 电机臂角度
    double SinB = FMath::Sin(Beta);
    double CosB = FMath::Cos(Beta);

    // ---- G 矩阵（正向映射）----
    // 推力行：所有电机贡献相同的 kT
    G[0][0] = kT; G[0][1] = kT; G[0][2] = kT; G[0][3] = kT;
    // 滚转力矩行：电机0/3 正贡献，电机1/2 负贡献
    G[1][0] = kT*L*SinB; G[1][1] = -kT*L*SinB; G[1][2] = -kT*L*SinB; G[1][3] = kT*L*SinB;
    // 俯仰力矩行：电机0/1 在前方（正贡献），电机2/3 在后方（负贡献）
    G[2][0] = kT*L*CosB; G[2][1] = kT*L*CosB; G[2][2] = -kT*L*CosB; G[2][3] = -kT*L*CosB;
    // 偏航力矩行：反扭矩，电机0/2 正，电机1/3 负
    G[3][0] = kQ; G[3][1] = -kQ; G[3][2] = kQ; G[3][3] = -kQ;

    // ---- GInv 矩阵（解析逆）----
    double a = 1.0 / (4.0 * kT);               // 推力分量
    double b = 1.0 / (4.0 * kT * L * SinB);    // 滚转分量
    double c = 1.0 / (4.0 * kT * L * CosB);    // 俯仰分量
    double d = 1.0 / (4.0 * kQ);               // 偏航分量

    GInv[0][0] = a; GInv[0][1] = b;  GInv[0][2] = c;  GInv[0][3] = d;
    GInv[1][0] = a; GInv[1][1] = -b; GInv[1][2] = c;  GInv[1][3] = -d;
    GInv[2][0] = a; GInv[2][1] = -b; GInv[2][2] = -c; GInv[2][3] = d;
    GInv[3][0] = a; GInv[3][1] = b;  GInv[3][2] = -c; GInv[3][3] = -d;
}

// ====================================================================
// 控制更新
// ====================================================================

/**
 * @brief 执行一帧的级联控制更新
 * @param DeltaTime 帧间隔
 *
 * 1. 将所有控制器的 TimeStep 同步为当前帧间隔
 * 2. 根据控制模式运行对应的控制链：
 *    - Position: Pos→Vel→Att→Rate
 *    - Velocity: Vel→Att→Rate
 *    - AttitudeThrust: Att→Rate
 *    - TorqueThrust: 直接使用命令
 *    - MotorSpeed: 直接使用电机转速命令
 * 3. 调用 CalculateMotorSpeeds() 将力矩/推力转换为电机转速
 */
void UDroneMovementComponent::ControlUpdate(double DeltaTime)
{
    // 同步所有控制器的时间步长为当前帧间隔
    double ControlTimeStep = FMath::Max(0.001, static_cast<double>(DeltaTime));
    if (PxController) PxController->SetTimeStep(ControlTimeStep);
    if (PyController) PyController->SetTimeStep(ControlTimeStep);
    if (PzController) PzController->SetTimeStep(ControlTimeStep);
    if (VxController) VxController->SetTimeStep(ControlTimeStep);
    if (VyController) VyController->SetTimeStep(ControlTimeStep);
    if (VzController) VzController->SetTimeStep(ControlTimeStep);
    if (RollController) RollController->SetTimeStep(ControlTimeStep);
    if (PitchController) PitchController->SetTimeStep(ControlTimeStep);
    if (YawController) YawController->SetTimeStep(ControlTimeStep);
    if (RollRateController) RollRateController->SetTimeStep(ControlTimeStep);
    if (PitchRateController) PitchRateController->SetTimeStep(ControlTimeStep);
    if (YawRateController) YawRateController->SetTimeStep(ControlTimeStep);

    // 默认推力为悬停推力 mg
    FVector TorqueCommand = FVector::ZeroVector;
    double ThrustCommand = Parameters.Mass * Parameters.Gravity;

    // 根据控制模式运行对应的控制链
    switch (CurrentControlMode)
    {
    case EDroneControlMode::MotorSpeed:
        // 直接设置电机转速（最底层控制）
        if (ControlCommands.Num() >= 4)
        {
            CurrentState.MotorSpeeds = ControlCommands;
        }
        return; // 不需要 CalculateMotorSpeeds

    case EDroneControlMode::Position:
        {
            // 完整级联：位置→速度→姿态→角速率
            FVector VelCmd = PositionLoop(TargetPosition);
            FVector AccCmd = VelocityLoop(VelCmd, ThrustCommand);
            FVector AngVelCmd = AttitudeLoop(AccCmd);
            TorqueCommand = AngularVelocityLoop(AngVelCmd);
        }
        break;

    case EDroneControlMode::Velocity:
        {
            // 从速度环开始：速度→姿态→角速率
            FVector AccCmd = VelocityLoop(TargetVelocity, ThrustCommand);
            FVector AngVelCmd = AttitudeLoop(AccCmd);
            TorqueCommand = AngularVelocityLoop(AngVelCmd);
        }
        break;

    case EDroneControlMode::AttitudeThrust:
        {
            // 从姿态环开始：将期望姿态转为角速率命令
            double RollDes = FMath::DegreesToRadians(TargetAttitude.Roll);
            double PitchDes = FMath::DegreesToRadians(TargetAttitude.Pitch);
            double YawDes = FMath::DegreesToRadians(TargetAttitude.Yaw);

            FRotator CurrentRot = CurrentState.GetRotator();
            // UE FRotator 符号与动力学四元数相反，需取反
            double Roll = -FMath::DegreesToRadians(CurrentRot.Roll);
            double Pitch = -FMath::DegreesToRadians(CurrentRot.Pitch);
            double Yaw = FMath::DegreesToRadians(CurrentRot.Yaw);

            // 计算角度误差（处理 ±180° 边界）
            double RollTarget = Roll + NormalizeAngle(RollDes - Roll);
            double PitchTarget = Pitch + NormalizeAngle(PitchDes - Pitch);
            double YawTarget = Yaw + NormalizeAngle(YawDes - Yaw);

            // 姿态PD控制器输出角速率命令
            double RollRateCmd = RollController ? RollController->Update(RollTarget, Roll) : 0.0;
            double PitchRateCmd = PitchController ? PitchController->Update(PitchTarget, Pitch) : 0.0;
            double YawRateCmd = YawController ? YawController->Update(YawTarget, Yaw) : 0.0;

            TorqueCommand = AngularVelocityLoop(FVector(RollRateCmd, PitchRateCmd, YawRateCmd));
            ThrustCommand = TargetThrust;
        }
        break;

    case EDroneControlMode::TorqueThrust:
        // 直接使用力矩和推力命令
        if (ControlCommands.Num() >= 4)
        {
            TorqueCommand = FVector(ControlCommands[0], ControlCommands[1], ControlCommands[2]);
            ThrustCommand = ControlCommands[3];
        }
        break;

    case EDroneControlMode::Idle:
    default:
        return;
    }

    // 从力矩+推力命令计算电机转速
    TArray<double> MotorSpeeds = CalculateMotorSpeeds(TorqueCommand, ThrustCommand);
    CurrentState.MotorSpeeds = MotorSpeeds;
}

/**
 * @brief 从力矩和推力命令计算四个电机的转速
 * @param TorqueCommand 三轴力矩命令 (τx, τy, τz) N·m
 * @param Thrust 总推力命令 (N)
 * @return 四个电机转速 (rad/s)
 *
 * 使用控制分配逆矩阵 GInv：
 *   ω²[i] = ΣGInv[i][j] * [T, τx, τy, τz][j]
 * 然后取平方根并限幅到 [MinMotorSpeed, MaxMotorSpeed]。
 */
TArray<double> UDroneMovementComponent::CalculateMotorSpeeds(const FVector& TorqueCommand, double Thrust)
{
    // 构造输入向量 [推力, 滚转力矩, 俯仰力矩, 偏航力矩]
    TArray<double> Input = {Thrust, TorqueCommand.X, TorqueCommand.Y, TorqueCommand.Z};
    TArray<double> OmegaSquared = {0.0, 0.0, 0.0, 0.0};

    // 矩阵乘法：ω² = GInv × Input
    for (int32 i = 0; i < 4; ++i)
        for (int32 j = 0; j < 4; ++j)
            OmegaSquared[i] += GInv[i][j] * Input[j];

    // 取平方根并限幅
    TArray<double> MotorSpeeds;
    for (int32 i = 0; i < 4; ++i)
    {
        double Omega = FMath::Sqrt(FMath::Max(0.0, OmegaSquared[i])); // 防止负数开方
        Omega = FMath::Clamp(Omega, Parameters.MinMotorSpeed, Parameters.MaxMotorSpeed);
        MotorSpeeds.Add(Omega);
    }
    return MotorSpeeds;
}

// ====================================================================
// RK4 动力学仿真
// ====================================================================

/**
 * @brief RK4 四阶积分一步
 * @param State 当前状态
 * @param TimeStep 积分步长
 * @return 积分后的新状态
 *
 * 经典四阶 Runge-Kutta 法：
 *   k1 = f(t, y)
 *   k2 = f(t+h/2, y + h/2·k1)
 *   k3 = f(t+h/2, y + h/2·k2)
 *   k4 = f(t+h, y + h·k3)
 *   y(t+h) = y(t) + h/6·(k1 + 2k2 + 2k3 + k4)
 *
 * 积分后归一化四元数，防止数值漂移。
 */
FDroneState UDroneMovementComponent::RK4Update(const FDroneState& State, double TimeStep)
{
    TArray<double> TotalForce, Torque;
    FDroneState TempState = State;

    // k1 = f(y_n)
    CalculateTotalForcesAndTorques(TempState, TotalForce, Torque);
    TArray<double> k1 = Derivatives(TempState, TotalForce, Torque);
    
    // k2 = f(y_n + h/2 * k1)
    TempState = StateAdd(State, k1, TimeStep * 0.5);
    CalculateTotalForcesAndTorques(TempState, TotalForce, Torque);
    TArray<double> k2 = Derivatives(TempState, TotalForce, Torque);
    
    // k3 = f(y_n + h/2 * k2)
    TempState = StateAdd(State, k2, TimeStep * 0.5);
    CalculateTotalForcesAndTorques(TempState, TotalForce, Torque);
    TArray<double> k3 = Derivatives(TempState, TotalForce, Torque);
    
    // k4 = f(y_n + h * k3)
    TempState = StateAdd(State, k3, TimeStep);
    CalculateTotalForcesAndTorques(TempState, TotalForce, Torque);
    TArray<double> k4 = Derivatives(TempState, TotalForce, Torque);
    
    // 加权平均: y_{n+1} = y_n + h/6 * (k1 + 2*k2 + 2*k3 + k4)
    TArray<double> FinalDerivative;
    for (int32 i = 0; i < k1.Num(); ++i)
        FinalDerivative.Add((k1[i] + 2.0*k2[i] + 2.0*k3[i] + k4[i]) / 6.0);
    
    FDroneState NewState = StateAdd(State, FinalDerivative, TimeStep);
    NewState.NormalizeQuaternion(); // 归一化四元数，防止积分漂移
    return NewState;
}

/**
 * @brief 计算给定状态下的合外力和合力矩
 * @param State 当前状态
 * @param OutTotalForce 输出：世界坐标系合外力 [Fx, Fy, Fz] (N)
 * @param OutTorque 输出：机体坐标系合力矩 [τx, τy, τz] (N·m)
 *
 * 力的组成：
 * 1. 推力（机体 +Z 方向，大小 = ΣkT*ω²，旋转到世界坐标系）
 * 2. 重力（世界 -Z 方向，大小 = mg）
 * 3. 空气阻力（与速度方向相反，大小 = Cd*|v|*v）
 *
 * 力矩通过控制分配矩阵 G 从电机转速计算。
 */
void UDroneMovementComponent::CalculateTotalForcesAndTorques(const FDroneState& State, TArray<double>& OutTotalForce, TArray<double>& OutTorque)
{
    OutTotalForce.SetNum(3);
    OutTorque.SetNum(3);

    double TotalThrust = 0.0;
    TArray<double> TotalTorque = {0.0, 0.0, 0.0};

    // 从电机转速计算推力和力矩（使用 G 矩阵）
    if (State.MotorSpeeds.Num() >= 4)
    {
        // 计算 ω² 向量
        TArray<double> OmegaSq = {
            State.MotorSpeeds[0] * State.MotorSpeeds[0],
            State.MotorSpeeds[1] * State.MotorSpeeds[1],
            State.MotorSpeeds[2] * State.MotorSpeeds[2],
            State.MotorSpeeds[3] * State.MotorSpeeds[3]
        };

        // G × ω²
        for (int32 i = 0; i < 4; ++i)
        {
            TotalThrust += G[0][i] * OmegaSq[i];        // 推力
            TotalTorque[0] += G[1][i] * OmegaSq[i];     // 滚转力矩
            TotalTorque[1] += G[2][i] * OmegaSq[i];     // 俯仰力矩
            TotalTorque[2] += G[3][i] * OmegaSq[i];     // 偏航力矩
        }
    }

    // 推力向量：机体 +Z 方向，旋转到世界坐标系
    FVector ThrustVectorBody(0.0, 0.0, TotalThrust);
    FQuat Orientation = State.GetQuaternion();
    FVector ThrustVectorWorld = RotateBodyToWorld(ThrustVectorBody, Orientation);

    // 重力向量：世界 -Z 方向
    FVector Gravity(0.0, 0.0, -Parameters.Mass * Parameters.Gravity);

    // 空气阻力：F_drag = -Cd * |v| * v（二次阻力模型）
    FVector Velocity = State.GetVelocity();
    FVector Drag = -Parameters.DragCoefficient * Velocity.Size() * Velocity;

    // 合外力 = 推力 + 重力 + 阻力
    FVector TotalForceVec = ThrustVectorWorld + Gravity + Drag;

    OutTotalForce[0] = TotalForceVec.X;
    OutTotalForce[1] = TotalForceVec.Y;
    OutTotalForce[2] = TotalForceVec.Z;
    OutTorque[0] = TotalTorque[0];
    OutTorque[1] = TotalTorque[1];
    OutTorque[2] = TotalTorque[2];
}

/**
 * @brief 计算 13 维状态导数向量
 * @param State 当前状态
 * @param TotalForce 世界坐标系合外力 [3]
 * @param Torque 机体坐标系合力矩 [3]
 * @return 导数向量 [13]: [ẋ, ẏ, ż, v̇x, v̇y, v̇z, q̇w, q̇x, q̇y, q̇z, ṗ, q̇, ṙ]
 *
 * 物理方程：
 * - 位置导数 = 速度 (运动学关系)
 * - 速度导数 = F/m (牛顿第二定律)
 * - 四元数导数 = ½·Ω(ω)·q (四元数运动学方程)
 *   其中 Ω(ω) 是由角速度构成的反对称矩阵
 * - 角速度导数 = J⁻¹·(τ - ω×Jω) (欧拉旋转方程)
 *   陀螺效应项 ω×Jω 描述了刚体旋转时的进动力矩
 */
TArray<double> UDroneMovementComponent::Derivatives(const FDroneState& State, const TArray<double>& TotalForce, const TArray<double>& Torque)
{
    TArray<double> Deriv;
    
    // ---- 位置导数 = 速度（运动学） ----
    Deriv.Add(State.Vx);  // dx/dt
    Deriv.Add(State.Vy);  // dy/dt
    Deriv.Add(State.Vz);  // dz/dt

    // ---- 速度导数 = 加速度 = F/m（牛顿第二定律） ----
    double m = Parameters.Mass;
    Deriv.Add(TotalForce[0] / m);  // dvx/dt
    Deriv.Add(TotalForce[1] / m);  // dvy/dt
    Deriv.Add(TotalForce[2] / m);  // dvz/dt

    // ---- 四元数导数（四元数运动学方程） ----
    // dq/dt = ½ · Ω(ω) · q
    // 其中 p=滚转角速率, q=俯仰角速率, r=偏航角速率
    double p = State.AngRollRate, q = State.AngPitchRate, r = State.AngYawRate;
    double qw = State.Qw, qx = State.Qx, qy = State.Qy, qz = State.Qz;
    
    Deriv.Add(0.5 * (-p*qx - q*qy - r*qz));  // dQw/dt
    Deriv.Add(0.5 * (p*qw + r*qy - q*qz));    // dQx/dt
    Deriv.Add(0.5 * (q*qw - r*qx + p*qz));    // dQy/dt
    Deriv.Add(0.5 * (r*qw + q*qx - p*qy));    // dQz/dt

    // ---- 角速度导数（欧拉旋转方程） ----
    // dω/dt = J⁻¹ · (τ - ω × Jω)
    // 陀螺效应项：ω × Jω = [(Jz-Jy)·q·r, (Jx-Jz)·p·r, (Jy-Jx)·p·q]
    double Jx = Parameters.Jx, Jy = Parameters.Jy, Jz = Parameters.Jz;
    Deriv.Add((Torque[0] - (Jz - Jy) * q * r) / Jx);  // dp/dt (滚转角加速度)
    Deriv.Add((Torque[1] - (Jx - Jz) * p * r) / Jy);  // dq/dt (俯仰角加速度)
    Deriv.Add((Torque[2] - (Jy - Jx) * p * q) / Jz);  // dr/dt (偏航角加速度)

    return Deriv;
}

/**
 * @brief 状态向量加法：NewState = State + Derivative × dt
 * @param State 基准状态
 * @param Derivative 13 维导数向量
 * @param dt 时间步长
 * @return 更新后的新状态
 *
 * 导数顺序对应：[X,Y,Z, Vx,Vy,Vz, Qw,Qx,Qy,Qz, p,q,r]
 */
FDroneState UDroneMovementComponent::StateAdd(const FDroneState& State, const TArray<double>& Derivative, double dt)
{
    FDroneState NewState = State;
    if (Derivative.Num() < 13) return NewState;

    // 位置
    NewState.X += Derivative[0] * dt;
    NewState.Y += Derivative[1] * dt;
    NewState.Z += Derivative[2] * dt;
    // 速度
    NewState.Vx += Derivative[3] * dt;
    NewState.Vy += Derivative[4] * dt;
    NewState.Vz += Derivative[5] * dt;
    // 四元数
    NewState.Qw += Derivative[6] * dt;
    NewState.Qx += Derivative[7] * dt;
    NewState.Qy += Derivative[8] * dt;
    NewState.Qz += Derivative[9] * dt;
    // 角速度
    NewState.AngRollRate += Derivative[10] * dt;
    NewState.AngPitchRate += Derivative[11] * dt;
    NewState.AngYawRate += Derivative[12] * dt;

    return NewState;
}

// ====================================================================
// 级联控制环路
// ====================================================================

/**
 * @brief 位置控制环（PD控制器，最外环）
 * @param PositionCommand 目标位置 (m)
 * @return 速度命令 (m/s)
 *
 * 每个轴独立的 PD 控制器，输出限幅为最大速度命令。
 */
FVector UDroneMovementComponent::PositionLoop(const FVector& PositionCommand)
{
    FVector Pos = CurrentState.GetPosition();
    double VxCmd = PxController ? PxController->Update(PositionCommand.X, Pos.X) : 0.0;
    double VyCmd = PyController ? PyController->Update(PositionCommand.Y, Pos.Y) : 0.0;
    double VzCmd = PzController ? PzController->Update(PositionCommand.Z, Pos.Z) : 0.0;
    return FVector(VxCmd, VyCmd, VzCmd);
}

/**
 * @brief 速度控制环（PID控制器）
 * @param VelocityCommand 目标速度 (m/s)
 * @param OutThrust 输出推力 (N)
 * @return 加速度命令 (m/s²)
 *
 * Z 轴加速度命令经过重力补偿：Thrust = m * (Az + g)
 * 倾斜补偿：实际推力 = Thrust / (cosRoll * cosPitch)
 * 安全限制：倾斜超过 60° 不再补偿，避免推力发散。
 */
FVector UDroneMovementComponent::VelocityLoop(const FVector& VelocityCommand, double& OutThrust)
{
    FVector Vel = CurrentState.GetVelocity();
    double AxCmd = VxController ? VxController->Update(VelocityCommand.X, Vel.X) : 0.0;
    double AyCmd = VyController ? VyController->Update(VelocityCommand.Y, Vel.Y) : 0.0;
    double AzCmd = VzController ? VzController->Update(VelocityCommand.Z, Vel.Z) : 0.0;

    // 重力补偿：推力 = m * (期望垂直加速度 + g)
    OutThrust = Parameters.Mass * (AzCmd + Parameters.Gravity);
    OutThrust = FMath::Max(0.0, OutThrust); // 推力不能为负

    // 推力-姿态补偿：倾斜时垂直推力分量减小，需要增大总推力
    FRotator CurrentRot = CurrentState.GetRotator();
    double CosRoll = FMath::Cos(FMath::DegreesToRadians(CurrentRot.Roll));
    double CosPitch = FMath::Cos(FMath::DegreesToRadians(CurrentRot.Pitch));
    double CosAttitude = CosRoll * CosPitch;
    if (CosAttitude > 0.5)  // 安全限制：倾斜超过60°不再补偿
    {
        OutThrust /= CosAttitude;
    }

    return FVector(AxCmd, AyCmd, AzCmd);
}

/**
 * @brief 姿态控制环（PD控制器）
 * @param AccelerationCommand 期望加速度 (m/s²)
 * @return 角速率命令 (rad/s)
 *
 * 从期望水平加速度计算期望倾斜角度：
 *   RollDes = atan2(-Ay, g)  —— 右移需要右倾
 *   PitchDes = atan2(Ax, g)  —— 前进需要低头
 *
 * 注意 UE FRotator 符号与动力学四元数方向相反：
 * - 负 Qy → 正 FRotator.Pitch
 * - 负 Qx → 正 FRotator.Roll
 * 因此测量值取反以匹配动力学约定。
 *
 * 倾斜角限制 ±0.3 rad (约 ±17.2°)，避免过度倾斜导致推力损失。
 */
FVector UDroneMovementComponent::AttitudeLoop(const FVector& AccelerationCommand)
{
    double AxDes = AccelerationCommand.X;
    double AyDes = AccelerationCommand.Y;
    double g = Parameters.Gravity;

    // 从期望加速度计算期望倾斜角
    double RollDes = FMath::Atan2(-AyDes, g);
    double PitchDes = FMath::Atan2(AxDes, g);
    double YawDes = FMath::DegreesToRadians(TargetAttitude.Yaw);

    // 限制最大倾斜角
    RollDes = FMath::Clamp(RollDes, -0.3, 0.3);
    PitchDes = FMath::Clamp(PitchDes, -0.3, 0.3);

    // 读取当前姿态（UE→动力学符号转换）
    FRotator CurrentRot = CurrentState.GetRotator();
    double Roll = -FMath::DegreesToRadians(CurrentRot.Roll);   // 符号取反
    double Pitch = -FMath::DegreesToRadians(CurrentRot.Pitch); // 符号取反
    double Yaw = FMath::DegreesToRadians(CurrentRot.Yaw);

    // 角度误差处理（归一化到 [-π, π]，处理 ±180° 边界）
    double RollTarget = Roll + NormalizeAngle(RollDes - Roll);
    double PitchTarget = Pitch + NormalizeAngle(PitchDes - Pitch);
    double YawTarget = Yaw + NormalizeAngle(YawDes - Yaw);

    // PD 控制器输出角速率命令
    double RollRateCmd = RollController ? RollController->Update(RollTarget, Roll) : 0.0;
    double PitchRateCmd = PitchController ? PitchController->Update(PitchTarget, Pitch) : 0.0;
    double YawRateCmd = YawController ? YawController->Update(YawTarget, Yaw) : 0.0;
    return FVector(RollRateCmd, PitchRateCmd, YawRateCmd);
}

/**
 * @brief 角速率控制环（PID控制器，最内环）
 * @param AngularVelocityCommand 目标角速率 (rad/s)
 * @return 力矩命令 (N·m)
 *
 * 每个轴独立的 PID 控制器，输出力矩命令。
 */
FVector UDroneMovementComponent::AngularVelocityLoop(const FVector& AngularVelocityCommand)
{
    double TorqueX = RollRateController ? RollRateController->Update(AngularVelocityCommand.X, CurrentState.AngRollRate) : 0.0;
    double TorqueY = PitchRateController ? PitchRateController->Update(AngularVelocityCommand.Y, CurrentState.AngPitchRate) : 0.0;
    double TorqueZ = YawRateController ? YawRateController->Update(AngularVelocityCommand.Z, CurrentState.AngYawRate) : 0.0;
    return FVector(TorqueX, TorqueY, TorqueZ);
}

// ====================================================================
// 设置接口
// ====================================================================

/**
 * @brief 设置初始状态
 * @param InitialState 初始状态
 *
 * 如果电机转速未满 4 个，自动填充为悬停转速。
 */
void UDroneMovementComponent::SetInitialState(const FDroneState& InitialState)
{
    CurrentState = InitialState;
    double HoverSpeed = Parameters.GetHoverMotorSpeed();
    if (CurrentState.MotorSpeeds.Num() < 4)
        CurrentState.MotorSpeeds = {HoverSpeed, HoverSpeed, HoverSpeed, HoverSpeed};
}

/**
 * @brief 切换控制模式并重置所有控制器
 * @param NewMode 新的控制模式
 *
 * 模式切换时重置控制器状态（清零误差历史和积分器），
 * 避免残留控制量导致突变。
 */
void UDroneMovementComponent::SetControlMode(EDroneControlMode NewMode)
{
    CurrentControlMode = NewMode;
    ResetAllControllers();
}

/** @brief 设置原始控制命令 */
void UDroneMovementComponent::SetControlCommand(const TArray<double>& Command) { ControlCommands = Command; }

/** @brief 设置目标位置 (SI, 米) */
void UDroneMovementComponent::SetTargetPosition(const FVector& TargetPos) { TargetPosition = TargetPos; }

/** @brief 设置目标速度 (SI, m/s) */
void UDroneMovementComponent::SetTargetVelocity(const FVector& TargetVel) { TargetVelocity = TargetVel; }

/**
 * @brief 设置目标姿态和推力
 * @param Attitude 目标姿态 (Roll, Pitch, Yaw 度)
 * @param Thrust 推力系数 (0~1)，1.0 对应悬停推力 mg
 */
void UDroneMovementComponent::SetTargetAttitude(const FRotator& Attitude, float Thrust)
{
    TargetAttitude = Attitude;
    TargetThrust = Thrust * Parameters.Mass * Parameters.Gravity;
}

/**
 * @brief 重置状态并清零所有控制器
 * @param NewState 新的初始状态
 */
void UDroneMovementComponent::ResetState(const FDroneState& NewState)
{
    CurrentState = NewState;
    ResetAllControllers();
}

/**
 * @brief 重置所有 12 个控制器的内部状态
 *
 * 清零误差历史、积分器和滤波状态。
 */
void UDroneMovementComponent::ResetAllControllers()
{
    if (PxController) PxController->Reset();
    if (PyController) PyController->Reset();
    if (PzController) PzController->Reset();
    if (VxController) VxController->Reset();
    if (VyController) VyController->Reset();
    if (VzController) VzController->Reset();
    if (RollController) RollController->Reset();
    if (PitchController) PitchController->Reset();
    if (YawController) YawController->Reset();
    if (RollRateController) RollRateController->Reset();
    if (PitchRateController) PitchRateController->Reset();
    if (YawRateController) YawRateController->Reset();
}

// ====================================================================
// PID 增益运行时调参
// ====================================================================

/**
 * @brief 设置位置控制器 PD 增益
 * @param Kp 比例增益
 * @param Kd 微分增益
 */
void UDroneMovementComponent::SetPositionGains(float Kp, float Kd)
{
    if (PxController) PxController->SetParameters(Kp, Kd, 0.0, 2.0);
    if (PyController) PyController->SetParameters(Kp, Kd, 0.0, 2.0);
    if (PzController) PzController->SetParameters(Kp, Kd, 0.0, 2.0);
}

/**
 * @brief 设置速度控制器 PID 增益
 * @param Kp 比例增益
 * @param Ki 积分增益
 * @param Kd 微分增益
 */
void UDroneMovementComponent::SetVelocityGains(float Kp, float Ki, float Kd)
{
    if (VxController) VxController->SetParameters(Kp, Ki, Kd, 0.05, 3.0);
    if (VyController) VyController->SetParameters(Kp, Ki, Kd, 0.05, 3.0);
    if (VzController) VzController->SetParameters(Kp, Ki, Kd, 0.05, 3.0);
}

/**
 * @brief 设置姿态控制器 PD 增益
 * @param Kp 比例增益（Yaw 轴自动使用 Kp*0.5）
 * @param Kd 微分增益
 */
void UDroneMovementComponent::SetAttitudeGains(float Kp, float Kd)
{
    if (RollController) RollController->SetParameters(Kp, Kd, 0.0, 3.0);
    if (PitchController) PitchController->SetParameters(Kp, Kd, 0.0, 3.0);
    if (YawController) YawController->SetParameters(Kp * 0.5f, Kd, 0.0, 3.0);
}

/**
 * @brief 设置角速率控制器增益
 * @param Kp 比例增益（Yaw 轴自动使用 Kp*0.5）
 */
void UDroneMovementComponent::SetAngleRateGains(float Kp)
{
    if (RollRateController) RollRateController->SetParameters(Kp, 0, 0, 0.05, 1.0);
    if (PitchRateController) PitchRateController->SetParameters(Kp, 0, 0, 0.05, 1.0);
    if (YawRateController) YawRateController->SetParameters(Kp * 0.5f, 0, 0, 0.05, 0.5);
}

// ====================================================================
// 辅助函数
// ====================================================================

/**
 * @brief 将机体坐标系向量旋转到世界坐标系
 * @param BodyVector 机体坐标系向量
 * @param Orientation 姿态四元数
 * @return 世界坐标系向量
 */
FVector UDroneMovementComponent::RotateBodyToWorld(const FVector& BodyVector, const FQuat& Orientation)
{
    return Orientation.RotateVector(BodyVector);
}

/**
 * @brief 角度归一化到 [-π, π] 范围
 * @param AngleRad 输入角度（弧度）
 * @return 归一化后的角度
 *
 * 用于处理 ±180° 边界的角度误差计算。
 */
double UDroneMovementComponent::NormalizeAngle(double AngleRad)
{
    while (AngleRad > PI) AngleRad -= 2.0 * PI;
    while (AngleRad < -PI) AngleRad += 2.0 * PI;
    return AngleRad;
}

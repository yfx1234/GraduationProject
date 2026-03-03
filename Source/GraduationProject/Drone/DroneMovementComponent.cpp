#include "DroneMovementComponent.h"
#include "GraduationProject/Core/Controller/PDController.h"
#include "GraduationProject/Core/Controller/PIDController.h"

/** @brief 构造函数 */
UDroneMovementComponent::UDroneMovementComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
    PrimaryComponentTick.TickGroup = TG_PrePhysics; 
    CurrentControlMode = EDroneControlMode::Idle;
    ControlCommands = {0.0, 0.0, 0.0, 0.0};
    TargetPosition = FVector::ZeroVector;
    TargetVelocity = FVector::ZeroVector;
    TargetAttitude = FRotator::ZeroRotator;
    TargetThrust = 0.0;
}

/**
 * @brief 组件初始化
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
 * @brief 每帧 Tick 控制更新
 * @param DeltaTime 帧间隔时间
 */
void UDroneMovementComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
    if (!bInitialized) return;
    if (CurrentControlMode == EDroneControlMode::Idle) return;
    ControlUpdate(DeltaTime);
    double SubStepTime = Parameters.TimeStep;
    int32 NumSubSteps = FMath::Max(1, FMath::CeilToInt(DeltaTime / SubStepTime));
    SubStepTime = DeltaTime / NumSubSteps; 
    for (int32 i = 0; i < NumSubSteps; ++i) CurrentState = RK4Update(CurrentState, SubStepTime);
}

/**
 * @brief 设置物理参数并重新初始化
 * @param NewParameters 新的物理参数
 * 更新参数后重新计算控制分配矩阵和重新初始化控制器
 */
void UDroneMovementComponent::SetParameters(const FDroneParameters& NewParameters)
{
    Parameters = NewParameters;
    ComputeControlAllocation();
    InitializeControllers();
}

/**
 * @brief 创建并初始化所有 PD/PID 控制器
 * 位置环(PD)：Kp=1.0, 输出限幅 2.0 m/s
 * 速度环(PID)：Kp=2.0, Ki=0.2, 输出限幅 3.0 m/s²，积分器 ±5.0
 * 姿态环(PD)：Kp=10.0（Yaw 减半为 5.0），输出限幅 3.0 rad/s
 * 角速率环(PID)：Kp=0.5（Yaw 减半为 0.25），输出限幅 1.0 N·m（Yaw 减半为 0.5）
 */
void UDroneMovementComponent::InitializeControllers()
{
    double Ts = Parameters.TimeStep;
    float PosPGain = 1.0f;
    float VelMaxLimit = 2.0f; 
    PxController = NewObject<UPDController>(this);
    PxController->Initialize(PosPGain, 0.0, 0.0, VelMaxLimit, Ts);
    PyController = NewObject<UPDController>(this);
    PyController->Initialize(PosPGain, 0.0, 0.0, VelMaxLimit, Ts);
    PzController = NewObject<UPDController>(this);
    PzController->Initialize(PosPGain, 0.0, 0.0, VelMaxLimit, Ts);
    float VelPGain = 2.0f;
    float VelDGain = 0.0f;
    VxController = NewObject<UPIDController>(this);
    VxController->Initialize(VelPGain, 0.2, VelDGain, 0.05, 3.0, Ts, -5.0, 5.0);
    VyController = NewObject<UPIDController>(this);
    VyController->Initialize(VelPGain, 0.2, VelDGain, 0.05, 3.0, Ts, -5.0, 5.0);
    VzController = NewObject<UPIDController>(this);
    VzController->Initialize(VelPGain, 0.2, VelDGain, 0.05, 3.0, Ts, -5.0, 5.0);
    float AnglePGain = 10.0f;
    float AngVelMaxLimit = 3.0f;
    RollController = NewObject<UPDController>(this);
    RollController->Initialize(AnglePGain, 0.0, 0.0, AngVelMaxLimit, Ts);
    PitchController = NewObject<UPDController>(this);
    PitchController->Initialize(AnglePGain, 0.0, 0.0, AngVelMaxLimit, Ts);
    YawController = NewObject<UPDController>(this);
    YawController->Initialize(AnglePGain * 0.5f, 0.0, 0.0, AngVelMaxLimit, Ts);
    float RatePGain = 0.5f;
    float TorqueMaxLimit = 1.0f;
    RollRateController = NewObject<UPIDController>(this);
    RollRateController->Initialize(RatePGain, 0.0, 0.0, 0.05, TorqueMaxLimit, Ts, -2.0, 2.0);
    PitchRateController = NewObject<UPIDController>(this);
    PitchRateController->Initialize(RatePGain, 0.0, 0.0, 0.05, TorqueMaxLimit, Ts, -2.0, 2.0);
    YawRateController = NewObject<UPIDController>(this);
    YawRateController->Initialize(RatePGain * 0.5f, 0.0, 0.0, 0.05, TorqueMaxLimit * 0.5f, Ts, -1.0, 1.0);
}

/**
 * @brief 计算控制分配矩阵 G 及其逆矩阵 GInv
 *  总推力 T = F1 + F2 + F3 + F4 = kT * ω1² + kT * ω2² + kT * ω3² + kT * ω4²
 *  滚转力矩 τx = F1 * L * sinβ - F2 * L * sinβ - F3 * L * sinβ + F4 * L * sinβ 
 *  俯仰力矩 τy = -F1 * L * cosβ - F2 * L * cosβ + F3 * L * cosβ + F4 * L * cosβ
 *  偏航力矩 τz = ω1² * kQ - ω2² * kQ + ω3² * kQ - ω4² * kQ
 *  控制分配矩阵 [T, τx, τy, τz]^T = G * [ω1², ω2², ω3², ω4²]^T
 */
void UDroneMovementComponent::ComputeControlAllocation()
{
    double kT = Parameters.ThrustCoefficient;
    double kQ = Parameters.TorqueCoefficient;
    double L = Parameters.ArmLength;
    double Beta = FMath::DegreesToRadians(Parameters.MotorAngle); // 电机臂角度
    double SinB = FMath::Sin(Beta);
    double CosB = FMath::Cos(Beta);
    G[0][0] = kT; 
    G[0][1] = kT; 
    G[0][2] = kT; 
    G[0][3] = kT;
    G[1][0] = kT*L*SinB; 
    G[1][1] = -kT*L*SinB; 
    G[1][2] = -kT*L*SinB; 
    G[1][3] = kT*L*SinB;
    G[2][0] = kT*L*CosB; 
    G[2][1] = kT*L*CosB; 
    G[2][2] = -kT*L*CosB; 
    G[2][3] = -kT*L*CosB;
    G[3][0] = kQ; 
    G[3][1] = -kQ; 
    G[3][2] = kQ; 
    G[3][3] = -kQ;
    double a = 1.0 / (4.0 * kT);             
    double b = 1.0 / (4.0 * kT * L * SinB);  
    double c = 1.0 / (4.0 * kT * L * CosB);  
    double d = 1.0 / (4.0 * kQ);             
    GInv[0][0] = a; GInv[0][1] = b;  GInv[0][2] = c;  GInv[0][3] = d;
    GInv[1][0] = a; GInv[1][1] = -b; GInv[1][2] = c;  GInv[1][3] = -d;
    GInv[2][0] = a; GInv[2][1] = -b; GInv[2][2] = -c; GInv[2][3] = d;
    GInv[3][0] = a; GInv[3][1] = b;  GInv[3][2] = -c; GInv[3][3] = -d;
}

/**
 * @brief 执行一帧的级联控制更新
 * @param DeltaTime 帧间隔
 * 根据控制模式更新推力力矩，再得到电机转速
 */
void UDroneMovementComponent::ControlUpdate(double DeltaTime)
{
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
    FVector TorqueCommand = FVector::ZeroVector;
    double ThrustCommand = Parameters.Mass * Parameters.Gravity;
    switch (CurrentControlMode)
    {
    case EDroneControlMode::MotorSpeed:
        if (ControlCommands.Num() >= 4) CurrentState.MotorSpeeds = ControlCommands;
        return;
    case EDroneControlMode::Position:
        {
            FVector VelCmd = PositionLoop(TargetPosition);
            FVector AccCmd = VelocityLoop(VelCmd, ThrustCommand);
            FVector AngVelCmd = AttitudeLoop(AccCmd);
            TorqueCommand = AngularVelocityLoop(AngVelCmd);
        }
        break;
    case EDroneControlMode::Velocity:
        {
            FVector AccCmd = VelocityLoop(TargetVelocity, ThrustCommand);
            FVector AngVelCmd = AttitudeLoop(AccCmd);
            TorqueCommand = AngularVelocityLoop(AngVelCmd);
        }
        break;
    case EDroneControlMode::AttitudeThrust:
        {
            double RollDes = FMath::DegreesToRadians(TargetAttitude.Roll);
            double PitchDes = FMath::DegreesToRadians(TargetAttitude.Pitch);
            double YawDes = FMath::DegreesToRadians(TargetAttitude.Yaw);
            FRotator CurrentRot = CurrentState.GetRotator();
            double Roll = -FMath::DegreesToRadians(CurrentRot.Roll);
            double Pitch = -FMath::DegreesToRadians(CurrentRot.Pitch);
            double Yaw = FMath::DegreesToRadians(CurrentRot.Yaw);
            double RollTarget = Roll + NormalizeAngle(RollDes - Roll);
            double PitchTarget = Pitch + NormalizeAngle(PitchDes - Pitch);
            double YawTarget = Yaw + NormalizeAngle(YawDes - Yaw);
            double RollRateCmd = RollController ? RollController->Update(RollTarget, Roll) : 0.0;
            double PitchRateCmd = PitchController ? PitchController->Update(PitchTarget, Pitch) : 0.0;
            double YawRateCmd = YawController ? YawController->Update(YawTarget, Yaw) : 0.0;
            TorqueCommand = AngularVelocityLoop(FVector(RollRateCmd, PitchRateCmd, YawRateCmd));
            ThrustCommand = TargetThrust;
        }
        break;
    case EDroneControlMode::TorqueThrust:
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
    TArray<double> MotorSpeeds = CalculateMotorSpeeds(TorqueCommand, ThrustCommand);
    CurrentState.MotorSpeeds = MotorSpeeds;
}

/**
 * @brief 从力矩和推力命令计算四个电机的转速
 * @param TorqueCommand 三轴力矩命令
 * @param Thrust 总推力命令
 * @return 四个电机转速 
 * [T, τx, τy, τz]^T = G * [ω1², ω2², ω3², ω4²]^T
 */
TArray<double> UDroneMovementComponent::CalculateMotorSpeeds(const FVector& TorqueCommand, double Thrust)
{
    TArray<double> Input = {Thrust, TorqueCommand.X, TorqueCommand.Y, TorqueCommand.Z};
    TArray<double> OmegaSquared = {0.0, 0.0, 0.0, 0.0};
    for (int32 i = 0; i < 4; ++i)
        for (int32 j = 0; j < 4; ++j)
            OmegaSquared[i] += GInv[i][j] * Input[j];
    TArray<double> MotorSpeeds;
    for (int32 i = 0; i < 4; ++i)
    {
        double Omega = FMath::Sqrt(FMath::Max(0.0, OmegaSquared[i])); // 防止负数开方
        Omega = FMath::Clamp(Omega, Parameters.MinMotorSpeed, Parameters.MaxMotorSpeed);
        MotorSpeeds.Add(Omega);
    }
    return MotorSpeeds;
}

/**
 * @brief RK4 四阶积分一步
 * @param State 当前状态
 * @param TimeStep 积分步长
 * @return 积分后的新状态
 * k1 = f(t, y)
 * k2 = f(t+h/2, y + h/2·k1)
 * k3 = f(t+h/2, y + h/2·k2)
 * k4 = f(t+h, y + h·k3)
 * y(t+h) = y(t) + h/6·(k1 + 2k2 + 2k3 + k4)
 */
FDroneState UDroneMovementComponent::RK4Update(const FDroneState& State, double TimeStep)
{
    TArray<double> TotalForce, Torque;
    FDroneState TempState = State;
    CalculateTotalForcesAndTorques(TempState, TotalForce, Torque);
    TArray<double> k1 = Derivatives(TempState, TotalForce, Torque);
    TempState = StateAdd(State, k1, TimeStep * 0.5);
    CalculateTotalForcesAndTorques(TempState, TotalForce, Torque);
    TArray<double> k2 = Derivatives(TempState, TotalForce, Torque);
    TempState = StateAdd(State, k2, TimeStep * 0.5);
    CalculateTotalForcesAndTorques(TempState, TotalForce, Torque);
    TArray<double> k3 = Derivatives(TempState, TotalForce, Torque);
    TempState = StateAdd(State, k3, TimeStep);
    CalculateTotalForcesAndTorques(TempState, TotalForce, Torque);
    TArray<double> k4 = Derivatives(TempState, TotalForce, Torque);
    TArray<double> FinalDerivative;
    for (int32 i = 0; i < k1.Num(); ++i)
        FinalDerivative.Add((k1[i] + 2.0*k2[i] + 2.0*k3[i] + k4[i]) / 6.0);
    FDroneState NewState = StateAdd(State, FinalDerivative, TimeStep);
    NewState.NormalizeQuaternion();
    return NewState;
}

/**
 * @brief 计算给定状态下的合外力和合力矩
 * @param State 当前状态
 * @param OutTotalForce 世界坐标系合外力 [Fx, Fy, Fz]
 * @param OutTorque 机体坐标系合力矩 [τx, τy, τz]
 */
void UDroneMovementComponent::CalculateTotalForcesAndTorques(const FDroneState& State, TArray<double>& OutTotalForce, TArray<double>& OutTorque)
{
    OutTotalForce.SetNum(3);
    OutTorque.SetNum(3);
    double TotalThrust = 0.0;
    TArray<double> TotalTorque = {0.0, 0.0, 0.0};
    if (State.MotorSpeeds.Num() >= 4)
    {
        TArray<double> OmegaSq = {
            State.MotorSpeeds[0] * State.MotorSpeeds[0],
            State.MotorSpeeds[1] * State.MotorSpeeds[1],
            State.MotorSpeeds[2] * State.MotorSpeeds[2],
            State.MotorSpeeds[3] * State.MotorSpeeds[3]
        };
        for (int32 i = 0; i < 4; ++i)
        {
            TotalThrust += G[0][i] * OmegaSq[i];
            TotalTorque[0] += G[1][i] * OmegaSq[i];
            TotalTorque[1] += G[2][i] * OmegaSq[i];
            TotalTorque[2] += G[3][i] * OmegaSq[i];
        }
    }
    FVector ThrustVectorBody(0.0, 0.0, TotalThrust);
    FQuat Orientation = State.GetQuaternion();
    FVector ThrustVectorWorld = RotateBodyToWorld(ThrustVectorBody, Orientation);
    FVector Gravity(0.0, 0.0, -Parameters.Mass * Parameters.Gravity);
    FVector Velocity = State.GetVelocity();
    FVector Drag = -Parameters.DragCoefficient * Velocity.Size() * Velocity;
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
 * @param TotalForce 世界坐标系合外力
 * @param Torque 机体坐标系合力矩
 * @return 导数向量 : [ẋ, ẏ, ż, v̇x, v̇y, v̇z, q̇w, q̇x, q̇y, q̇z, ṗ, q̇, ṙ]
 * 位置导数 = 速度 
 * 速度导数 = F/m 
 * 四元数导数 = 1/2 * q ⊗ ω
 *  [q̇w]         [0 -p -q -r] [qw]
 *  [q̇x] = 1/2 * [p  0  r -q] [qx]
 *  [q̇y]         [q -r  0  p] [qy]
 *  [q̇z]         [r  q -p  0] [qz]
 * 角速度导数 = J⁻¹·(τ - ω×Jω) 
 *                                     [p]   [Jx*p]   [q*r*(Jz-Jy)]
 *  J * ω_dot  = τ - ω × (J * ω) = τ - [q] × [Jy*q] = [p*r*(Jx-Jz)]
 *                                     [r]   [Jz*r]   [p*q*(Jy-Jx)]
 */
TArray<double> UDroneMovementComponent::Derivatives(const FDroneState& State, const TArray<double>& TotalForce, const TArray<double>& Torque)
{
    TArray<double> Deriv;
    Deriv.Add(State.Vx); 
    Deriv.Add(State.Vy); 
    Deriv.Add(State.Vz); 
    double m = Parameters.Mass;
    Deriv.Add(TotalForce[0] / m);
    Deriv.Add(TotalForce[1] / m);
    Deriv.Add(TotalForce[2] / m);
    double p = State.AngRollRate, q = State.AngPitchRate, r = State.AngYawRate;
    double qw = State.Qw, qx = State.Qx, qy = State.Qy, qz = State.Qz;
    Deriv.Add(0.5 * (-p*qx - q*qy - r*qz)); 
    Deriv.Add(0.5 * (p*qw + r*qy - q*qz));  
    Deriv.Add(0.5 * (q*qw - r*qx + p*qz));   
    Deriv.Add(0.5 * (r*qw + q*qx - p*qy));  
    double Jx = Parameters.Jx, Jy = Parameters.Jy, Jz = Parameters.Jz;
    Deriv.Add((Torque[0] - (Jz - Jy) * q * r) / Jx);  
    Deriv.Add((Torque[1] - (Jx - Jz) * p * r) / Jy);  
    Deriv.Add((Torque[2] - (Jy - Jx) * p * q) / Jz);  
    return Deriv;
}

/**
 * @brief NewState = State + Derivative × dt
 * @param State 基准状态
 * @param Derivative 13 维导数向量
 * @param dt 时间步长
 * @return 更新后的新状态
 * [X,Y,Z, Vx,Vy,Vz, Qw,Qx,Qy,Qz, p,q,r]
 */
FDroneState UDroneMovementComponent::StateAdd(const FDroneState& State, const TArray<double>& Derivative, double dt)
{
    FDroneState NewState = State;
    if (Derivative.Num() < 13) return NewState;
    NewState.X += Derivative[0] * dt;
    NewState.Y += Derivative[1] * dt;
    NewState.Z += Derivative[2] * dt;
    NewState.Vx += Derivative[3] * dt;
    NewState.Vy += Derivative[4] * dt;
    NewState.Vz += Derivative[5] * dt;
    NewState.Qw += Derivative[6] * dt;
    NewState.Qx += Derivative[7] * dt;
    NewState.Qy += Derivative[8] * dt;
    NewState.Qz += Derivative[9] * dt;
    NewState.AngRollRate += Derivative[10] * dt;
    NewState.AngPitchRate += Derivative[11] * dt;
    NewState.AngYawRate += Derivative[12] * dt;
    return NewState;
}

/**
 * @brief 位置控制环 PD控制器
 * @param PositionCommand 目标位置
 * @return 速度命令
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
 * @brief 速度控制环 PID控制器
 * @param VelocityCommand 目标速度
 * @param OutThrust 输出推力
 * @return 加速度命令
 * Z 轴加速度命令经过重力补偿：Thrust = m * (Az + g)
 * 倾斜补偿：实际推力 = Thrust / (cosRoll * cosPitch)
 */
FVector UDroneMovementComponent::VelocityLoop(const FVector& VelocityCommand, double& OutThrust)
{
    FVector Vel = CurrentState.GetVelocity();
    double AxCmd = VxController ? VxController->Update(VelocityCommand.X, Vel.X) : 0.0;
    double AyCmd = VyController ? VyController->Update(VelocityCommand.Y, Vel.Y) : 0.0;
    double AzCmd = VzController ? VzController->Update(VelocityCommand.Z, Vel.Z) : 0.0;
    OutThrust = Parameters.Mass * (AzCmd + Parameters.Gravity);
    OutThrust = FMath::Max(0.0, OutThrust);
    FRotator CurrentRot = CurrentState.GetRotator();
    double CosRoll = FMath::Cos(FMath::DegreesToRadians(CurrentRot.Roll));
    double CosPitch = FMath::Cos(FMath::DegreesToRadians(CurrentRot.Pitch));
    double CosAttitude = CosRoll * CosPitch;
    if (CosAttitude > 0.5) OutThrust /= CosAttitude;
    return FVector(AxCmd, AyCmd, AzCmd);
}

/**
 * @brief 姿态控制环 PD控制器
 * @param AccelerationCommand 期望加速度
 * @return 角速率命令
 * 从期望水平加速度计算期望倾斜角度：
 *   RollDes = atan2(-Ay, g)  —— 右移需要右倾
 *   PitchDes = atan2(Ax, g)  —— 前进需要低头
 */
FVector UDroneMovementComponent::AttitudeLoop(const FVector& AccelerationCommand)
{
    double AxDes = AccelerationCommand.X;
    double AyDes = AccelerationCommand.Y;
    double g = Parameters.Gravity;
    double RollDes = FMath::Atan2(-AyDes, g);
    double PitchDes = FMath::Atan2(AxDes, g);
    double YawDes = FMath::DegreesToRadians(TargetAttitude.Yaw);
    RollDes = FMath::Clamp(RollDes, -0.3, 0.3);
    PitchDes = FMath::Clamp(PitchDes, -0.3, 0.3);
    FRotator CurrentRot = CurrentState.GetRotator();
    double Roll = -FMath::DegreesToRadians(CurrentRot.Roll);
    double Pitch = -FMath::DegreesToRadians(CurrentRot.Pitch);
    double Yaw = FMath::DegreesToRadians(CurrentRot.Yaw);
    double RollTarget = Roll + NormalizeAngle(RollDes - Roll);
    double PitchTarget = Pitch + NormalizeAngle(PitchDes - Pitch);
    double YawTarget = Yaw + NormalizeAngle(YawDes - Yaw);
    double RollRateCmd = RollController ? RollController->Update(RollTarget, Roll) : 0.0;
    double PitchRateCmd = PitchController ? PitchController->Update(PitchTarget, Pitch) : 0.0;
    double YawRateCmd = YawController ? YawController->Update(YawTarget, Yaw) : 0.0;
    return FVector(RollRateCmd, PitchRateCmd, YawRateCmd);
}

/**
 * @brief 角速率控制环 PID控制器
 * @param AngularVelocityCommand 目标角速率
 * @return 力矩命令
 */
FVector UDroneMovementComponent::AngularVelocityLoop(const FVector& AngularVelocityCommand)
{
    double TorqueX = RollRateController ? RollRateController->Update(AngularVelocityCommand.X, CurrentState.AngRollRate) : 0.0;
    double TorqueY = PitchRateController ? PitchRateController->Update(AngularVelocityCommand.Y, CurrentState.AngPitchRate) : 0.0;
    double TorqueZ = YawRateController ? YawRateController->Update(AngularVelocityCommand.Z, CurrentState.AngYawRate) : 0.0;
    return FVector(TorqueX, TorqueY, TorqueZ);
}

/**
 * @brief 设置初始状态
 * @param InitialState 初始状态
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
 */
void UDroneMovementComponent::SetControlMode(EDroneControlMode NewMode)
{
    CurrentControlMode = NewMode;
    ResetAllControllers();
}

/** @brief 设置原始控制命令 */
void UDroneMovementComponent::SetControlCommand(const TArray<double>& Command) { ControlCommands = Command; }

/** @brief 设置目标位置 */
void UDroneMovementComponent::SetTargetPosition(const FVector& TargetPos) { TargetPosition = TargetPos; }

/** @brief 设置目标速度 */
void UDroneMovementComponent::SetTargetVelocity(const FVector& TargetVel) { TargetVelocity = TargetVel; }

/**
 * @brief 设置目标姿态和推力
 * @param Attitude 目标姿态
 * @param Thrust 推力系数
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

/** @brief 重置所有 12 个控制器的内部状态 */
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
 * @param Kp 比例增益
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
 * @param Kp 比例增益
 */
void UDroneMovementComponent::SetAngleRateGains(float Kp)
{
    if (RollRateController) RollRateController->SetParameters(Kp, 0, 0, 0.05, 1.0);
    if (PitchRateController) PitchRateController->SetParameters(Kp, 0, 0, 0.05, 1.0);
    if (YawRateController) YawRateController->SetParameters(Kp * 0.5f, 0, 0, 0.05, 0.5);
}

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
 * @param AngleRad 输入角度
 * @return 归一化后的角度
 */
double UDroneMovementComponent::NormalizeAngle(double AngleRad)
{
    while (AngleRad > PI) AngleRad -= 2.0 * PI;
    while (AngleRad < -PI) AngleRad += 2.0 * PI;
    return AngleRad;
}

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
    Parameters.InitializeComputed();
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

    // --- AirSim 风格：控制器和物理在同一循环中以相同频率更新 ---
    // AirSim: PhysicsBody::update() → updateSensorsAndController() → updatePhysics()
    // 控制器每个物理子步都更新，产生平滑的电机命令
    double SubStepTime = Parameters.TimeStep;
    int32 NumSubSteps = FMath::Max(1, FMath::CeilToInt(DeltaTime / SubStepTime));
    SubStepTime = DeltaTime / NumSubSteps;

    FDroneState BackupState = CurrentState;
    for (int32 i = 0; i < NumSubSteps; ++i)
    {
        // 控制更新（每个子步都运行，与物理同频）
        ControlUpdate(SubStepTime);

        // 物理更新
        VerletUpdate(SubStepTime);
        CheckGroundCollision(InitialGroundZ);

        // NaN 保护
        if (CurrentState.HasNaN())
        {
            UE_LOG(LogTemp, Warning, TEXT("[DroneMovement] NaN detected, reverting to backup state"));
            CurrentState = BackupState;
            CurrentState.AngRollRate = 0.0;
            CurrentState.AngPitchRate = 0.0;
            CurrentState.AngYawRate = 0.0;
            PrevLinearAcceleration = FVector::ZeroVector;
            PrevAngularAcceleration = FVector::ZeroVector;
            break;
        }
    }

    // 速度裁剪防发散
    CurrentState.ClampVelocities(50.0, 50.0);
}

/**
 * @brief 设置物理参数并重新初始化
 * @param NewParameters 新的物理参数
 * 更新参数后重新计算控制分配矩阵和重新初始化控制器
 */
void UDroneMovementComponent::SetParameters(const FDroneParameters& NewParameters)
{
    Parameters = NewParameters;
    Parameters.InitializeComputed();
    ComputeControlAllocation();
    InitializeControllers();
}

/**
 * @brief 创建并初始化所有 PD/PID 控制器
 * 位置环(PD)：Kp=1.0, Kd=0.3, 输出限幅 2.0 m/s
 * 速度环(PID)：Kp=2.0, Ki=0.1, Kd=0.1, 输出限幅 3.0 m/s²
 * 姿态环(PD)：Kp=6.0, Kd=0.3（Yaw 减半），输出限幅 3.0 rad/s
 * 角速率环(PID)：Kp=0.5, Ki=0.0, Kd=0.0，输出限幅 1.0 N·m
 */
void UDroneMovementComponent::InitializeControllers()
{
    double Ts = Parameters.TimeStep;

    // ── 位置环 PD ── Kp=1.0, Kd=0.3
    float PosPGain = 1.0f;
    float PosDGain = 0.3f;
    float VelMaxLimit = 2.0f;
    PxController = NewObject<UPDController>(this);
    PxController->Initialize(PosPGain, PosDGain, 0.1, VelMaxLimit, Ts);
    PyController = NewObject<UPDController>(this);
    PyController->Initialize(PosPGain, PosDGain, 0.1, VelMaxLimit, Ts);
    PzController = NewObject<UPDController>(this);
    PzController->Initialize(PosPGain, PosDGain, 0.1, VelMaxLimit, Ts);

    // ── 速度环 PID ── Kp=2.0, Ki=0.1, Kd=0.1
    float VelPGain = 2.0f;
    float VelIGain = 0.1f;
    float VelDGain = 0.1f;
    VxController = NewObject<UPIDController>(this);
    VxController->Initialize(VelPGain, VelIGain, VelDGain, 0.1, 2.0, Ts, -5.0, 5.0);
    VyController = NewObject<UPIDController>(this);
    VyController->Initialize(VelPGain, VelIGain, VelDGain, 0.1, 2.0, Ts, -5.0, 5.0);
    VzController = NewObject<UPIDController>(this);
    VzController->Initialize(VelPGain, VelIGain, VelDGain, 0.1, 2.0, Ts, -5.0, 5.0);

    // ── 姿态环 PD ── Kp=6.0, Kd=0.3
    float AnglePGain = 6.0f;
    float AngleDGain = 0.3f;
    float AngVelMaxLimit = 3.0f;
    RollController = NewObject<UPDController>(this);
    RollController->Initialize(AnglePGain, AngleDGain, 0.1, AngVelMaxLimit, Ts);
    PitchController = NewObject<UPDController>(this);
    PitchController->Initialize(AnglePGain, AngleDGain, 0.1, AngVelMaxLimit, Ts);
    YawController = NewObject<UPDController>(this);
    YawController->Initialize(AnglePGain * 0.5f, AngleDGain * 0.5f, 0.1, AngVelMaxLimit, Ts);

    // ── 角速率环 PID ── Kp=0.5, Ki=0, Kd=0（内环宜快，纯P足够）
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
    LastControlDeltaTime = DeltaTime;  // 保存帧时间供电机滤波器使用
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
            // 计算自动偏航：从当前位置到目标位置的方向（稳定，不产生反馈环）
            FVector Pos = CurrentState.GetPosition();
            double DirX = TargetPosition.X - Pos.X;
            double DirY = TargetPosition.Y - Pos.Y;
            double DirMag = FMath::Sqrt(DirX * DirX + DirY * DirY);
            if (DirMag > YawSpeedThreshold)
            {
                DesiredYaw = FMath::Atan2(DirY, DirX) + FMath::DegreesToRadians(YawOffset);
                LockedYaw = DesiredYaw;
                bYawInitialized = true;
            }
            else if (bYawInitialized)
            {
                DesiredYaw = LockedYaw;
            }
            else
            {
                DesiredYaw = FMath::DegreesToRadians(CurrentState.GetRotator().Yaw);
            }

            FVector VelCmd = PositionLoop(TargetPosition);
            FVector AccCmd = VelocityLoop(VelCmd, ThrustCommand);
            FVector AngVelCmd = AttitudeLoop(AccCmd);
            TorqueCommand = AngularVelocityLoop(AngVelCmd);
        }
        break;
    case EDroneControlMode::Velocity:
        {
            // 计算自动偏航：从目标速度方向（稳定，不产生反馈环）
            double VelMag = FMath::Sqrt(TargetVelocity.X * TargetVelocity.X + TargetVelocity.Y * TargetVelocity.Y);
            if (VelMag > YawSpeedThreshold)
            {
                DesiredYaw = FMath::Atan2(TargetVelocity.Y, TargetVelocity.X) + FMath::DegreesToRadians(YawOffset);
                LockedYaw = DesiredYaw;
                bYawInitialized = true;
            }
            else if (bYawInitialized)
            {
                DesiredYaw = LockedYaw;
            }
            else
            {
                DesiredYaw = FMath::DegreesToRadians(CurrentState.GetRotator().Yaw);
            }

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
            // UE FRotator 的 Roll/Pitch 与控制分配矩阵符号相反，需要取反
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
 * 1. 逆分配矩阵求解 ω²
 * 2. Mixer 饱和补偿（下溢偏移 + 上溢缩放）
 * 3. 一阶低通滤波模拟电机惯性
 */
TArray<double> UDroneMovementComponent::CalculateMotorSpeeds(const FVector& TorqueCommand, double Thrust)
{
    TArray<double> Input = {Thrust, TorqueCommand.X, TorqueCommand.Y, TorqueCommand.Z};
    TArray<double> OmegaSquared = {0.0, 0.0, 0.0, 0.0};
    for (int32 i = 0; i < 4; ++i)
        for (int32 j = 0; j < 4; ++j)
            OmegaSquared[i] += GInv[i][j] * Input[j];

    TArray<double> MotorSpeeds;
    MotorSpeeds.SetNum(4);
    for (int32 i = 0; i < 4; ++i)
        MotorSpeeds[i] = FMath::Sqrt(FMath::Max(0.0, OmegaSquared[i]));

    // --- Mixer 饱和补偿 ---
    // 下溢补偿：若任一电机低于最小值，所有电机加偏移
    double MinSpeed = MotorSpeeds[0];
    for (int32 i = 1; i < 4; ++i)
        MinSpeed = FMath::Min(MinSpeed, MotorSpeeds[i]);
    if (MinSpeed < Parameters.MinMotorSpeed)
    {
        double Undershoot = Parameters.MinMotorSpeed - MinSpeed;
        for (int32 i = 0; i < 4; ++i)
            MotorSpeeds[i] += Undershoot;
    }

    // 上溢补偿：若任一电机超过最大值，等比缩放
    double MaxSpeed = MotorSpeeds[0];
    for (int32 i = 1; i < 4; ++i)
        MaxSpeed = FMath::Max(MaxSpeed, MotorSpeeds[i]);
    if (MaxSpeed > Parameters.MaxMotorSpeed)
    {
        double Scale = Parameters.MaxMotorSpeed / MaxSpeed;
        for (int32 i = 0; i < 4; ++i)
            MotorSpeeds[i] *= Scale;
    }

    // 最终 Clamp
    for (int32 i = 0; i < 4; ++i)
        MotorSpeeds[i] = FMath::Clamp(MotorSpeeds[i], Parameters.MinMotorSpeed, Parameters.MaxMotorSpeed);

    // --- 一阶低通滤波模拟电机惯性 ---
    // 使用控制更新的实际 DeltaTime，而非物理子步步长
    if (MotorSpeedsFiltered.Num() < 4)
        MotorSpeedsFiltered = {0.0, 0.0, 0.0, 0.0};
    double FilterDt = FMath::Max(0.001, LastControlDeltaTime);
    double Alpha = FMath::Exp(-FilterDt / FMath::Max(0.001, Parameters.MotorFilterTC));
    for (int32 i = 0; i < 4; ++i)
        MotorSpeedsFiltered[i] = MotorSpeedsFiltered[i] * Alpha + MotorSpeeds[i] * (1.0 - Alpha);

    return MotorSpeedsFiltered;
}

/**
 * @brief Verlet 积分一步
 * @param DeltaTime 积分步长
 * 使用 Velocity Verlet 方法：
 *   x(t+dt) = x(t) + v(t)*dt + 0.5*a(t)*dt²
 *   a(t+dt) = F(t+dt) / m
 *   v(t+dt) = v(t) + 0.5*(a(t) + a(t+dt))*dt
 * 角度更新使用角速度和 AngleAxis 方法
 */
void UDroneMovementComponent::VerletUpdate(double DeltaTime)
{
    const double dt = DeltaTime;
    const double halfDt = 0.5 * dt;
    const double halfDtSq = 0.5 * dt * dt;

    // --- Ground Lock：地面上时检查推力的垂直分量是否超过重力 ---
    if (bGrounded)
    {
        FVector Force, Torque;
        CalculateTotalForcesAndTorques(CurrentState, Force, Torque);
        // 检查推力在世界 Z 轴的分量是否超过重力
        double WeightForce = Parameters.Mass * Parameters.Gravity;
        if (Force.Z > WeightForce * 1.05)  // 5% 裕量防止抢起又落下
        {
            bGrounded = false;
            UE_LOG(LogTemp, Log, TEXT("[DroneMovement] Lifting off ground, Force.Z=%.2f > Weight=%.2f"),
                Force.Z, WeightForce);
        }
        else
        {
            return; // 留在地面，不更新状态
        }
    }

    // --- 位置更新：x(t+dt) = x(t) + v(t)*dt + 0.5*a(t)*dt² ---
    FVector Pos = CurrentState.GetPosition();
    FVector Vel = CurrentState.GetVelocity();
    FVector AngVel = CurrentState.GetAngularVelocity();

    Pos += Vel * dt + PrevLinearAcceleration * halfDtSq;
    CurrentState.SetPosition(Pos);

    // --- 姿态更新：用角速度计算角度增量 ---
    FVector AvgAngVel = AngVel + PrevAngularAcceleration * halfDt;
    double AnglePerUnit = AvgAngVel.Size();
    if (AnglePerUnit > KINDA_SMALL_NUMBER)
    {
        FQuat DeltaQ(AvgAngVel / AnglePerUnit, AnglePerUnit * dt);
        FQuat CurrentQ = CurrentState.GetQuaternion();
        FQuat NewQ = CurrentQ * DeltaQ;
        NewQ.Normalize();
        CurrentState.SetQuaternion(NewQ);
    }

    // --- 计算新位置处的力和力矩 ---
    FVector Force, Torque;
    CalculateTotalForcesAndTorques(CurrentState, Force, Torque);

    // --- 线加速度：a(t+dt) = F/m ---
    FVector NewLinearAcc = Force / Parameters.Mass + FVector(0.0, 0.0, -Parameters.Gravity);

    // --- 角加速度：欧拉旋转方程 ---
    double Jx = Parameters.Jx, Jy = Parameters.Jy, Jz = Parameters.Jz;
    double p = CurrentState.AngRollRate;
    double q = CurrentState.AngPitchRate;
    double r = CurrentState.AngYawRate;
    FVector AngMomentumRate;
    AngMomentumRate.X = (Torque.X - (Jz - Jy) * q * r) / Jx;
    AngMomentumRate.Y = (Torque.Y - (Jx - Jz) * p * r) / Jy;
    AngMomentumRate.Z = (Torque.Z - (Jy - Jx) * p * q) / Jz;

    // --- 速度更新：v(t+dt) = v(t) + 0.5*(a(t) + a(t+dt))*dt ---
    FVector NewVel = Vel + (PrevLinearAcceleration + NewLinearAcc) * halfDt;
    CurrentState.SetVelocity(NewVel);

    FVector NewAngVel = AngVel + (PrevAngularAcceleration + AngMomentumRate) * halfDt;
    CurrentState.AngRollRate = NewAngVel.X;
    CurrentState.AngPitchRate = NewAngVel.Y;
    CurrentState.AngYawRate = NewAngVel.Z;

    // --- 保存加速度供下一帧使用 ---
    PrevLinearAcceleration = NewLinearAcc;
    PrevAngularAcceleration = AngMomentumRate;

    // --- 四元数归一化 ---
    CurrentState.NormalizeQuaternion();
}

/**
 * @brief 计算给定状态下的合外力和合力矩
 * @param State 当前状态
 * @param OutForce 世界坐标系合外力（不含重力，重力在 Verlet 中单独处理）
 * @param OutTorque 机体坐标系合力矩
 */
void UDroneMovementComponent::CalculateTotalForcesAndTorques(const FDroneState& State, FVector& OutForce, FVector& OutTorque)
{
    OutForce = FVector::ZeroVector;
    OutTorque = FVector::ZeroVector;

    double TotalThrust = 0.0;
    FVector TotalTorque = FVector::ZeroVector;

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
            TotalTorque.X += G[1][i] * OmegaSq[i];
            TotalTorque.Y += G[2][i] * OmegaSq[i];
            TotalTorque.Z += G[3][i] * OmegaSq[i];
        }
    }

    // 推力向量从机体 Z 轴转到世界坐标系
    FVector ThrustVectorBody(0.0, 0.0, TotalThrust);
    FQuat Orientation = State.GetQuaternion();
    FVector ThrustVectorWorld = RotateBodyToWorld(ThrustVectorBody, Orientation);

    // 阻力（简化模型，Phase 4 将升级为各向异性）
    FVector Velocity = State.GetVelocity();
    FVector Drag = -Parameters.DragCoefficient * Velocity.Size() * Velocity;

    OutForce = ThrustVectorWorld + Drag;
    OutTorque = TotalTorque;
}

/**
 * @brief 检查并处理地面碰撞
 * @param GroundZ 地面 Z 坐标（仿真坐标系）
 * 当无人机位置低于地面且向下运动时：
 * - 将位置钦定到地面
 * - 清除向下速度
 * - 使用恢复系数处理弹跳
 * - Ground Lock：低速着陆时锁定姿态
 */
void UDroneMovementComponent::CheckGroundCollision(double GroundZ)
{
    if (CurrentState.Z <= GroundZ && CurrentState.Vz < 0.0)
    {
        CurrentState.Z = GroundZ;

        // 判断是否为低速着陆（垂直速度占主导）
        double SpeedXY = FMath::Sqrt(CurrentState.Vx * CurrentState.Vx + CurrentState.Vy * CurrentState.Vy);
        bool bLanding = FMath::Abs(CurrentState.Vz) > SpeedXY;

        if (bLanding && FMath::Abs(CurrentState.Vz) < 1.0)
        {
            // Ground Lock：低速着陆，锁定在地面
            bGrounded = true;
            CurrentState.Vx = 0.0;
            CurrentState.Vy = 0.0;
            CurrentState.Vz = 0.0;
            CurrentState.AngRollRate = 0.0;
            CurrentState.AngPitchRate = 0.0;
            CurrentState.AngYawRate = 0.0;

            // 矫正姿态：Roll/Pitch 归零，保留 Yaw
            FRotator Rot = CurrentState.GetRotator();
            Rot.Roll = 0.0f;
            Rot.Pitch = 0.0f;
            CurrentState.SetQuaternion(Rot.Quaternion());

            PrevLinearAcceleration = FVector::ZeroVector;
            PrevAngularAcceleration = FVector::ZeroVector;

            UE_LOG(LogTemp, Log, TEXT("[DroneMovement] Ground locked"));
        }
        else
        {
            // 弹跳响应：反弹垂直速度
            double Restitution = Parameters.Restitution;
            CurrentState.Vz = -CurrentState.Vz * Restitution;

            // 摩擦衰减水平速度
            double FrictionFactor = 1.0 - Parameters.Friction * 0.5;
            CurrentState.Vx *= FrictionFactor;
            CurrentState.Vy *= FrictionFactor;

            // 角速度衰减
            CurrentState.AngRollRate *= 0.9;
            CurrentState.AngPitchRate *= 0.9;
            CurrentState.AngYawRate *= 0.9;
        }
    }
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

    // 从期望水平加速度计算期望倾斜角
    double RollDes = FMath::Atan2(-AyDes, g);
    double PitchDes = FMath::Atan2(AxDes, g);

    // 自动偏航角已在 ControlUpdate 中从目标方向计算好（存在 DesiredYaw）
    double YawDes = DesiredYaw;

    // 限制最大倾斜角（约12°，比 AirSim 默认的 ~15° 稍保守，确保平滑）
    RollDes = FMath::Clamp(RollDes, -0.21, 0.21);
    PitchDes = FMath::Clamp(PitchDes, -0.21, 0.21);

    // UE FRotator 的 Roll/Pitch 正方向与控制分配矩阵 G 的力矩符号相反
    // 取反使 PD 控制器的误差方向与实际力矩方向一致
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
 * 记录起始高度用于地面碰撞检测，初始化电机滤波器
 */
void UDroneMovementComponent::SetInitialState(const FDroneState& InitialState)
{
    CurrentState = InitialState;
    double HoverSpeed = Parameters.GetHoverMotorSpeed();
    if (CurrentState.MotorSpeeds.Num() < 4)
        CurrentState.MotorSpeeds = {HoverSpeed, HoverSpeed, HoverSpeed, HoverSpeed};

    // 记录起始地面高度：用初始 Z 位置作为地面参考
    // 无人机初始生成在地面上，起始为 Ground Lock 状态
    InitialGroundZ = CurrentState.Z;
    bGrounded = true;

    // 初始化电机滤波器到悬停转速
    MotorSpeedsFiltered = CurrentState.MotorSpeeds;

    // 清零加速度历史
    PrevLinearAcceleration = FVector::ZeroVector;
    PrevAngularAcceleration = FVector::ZeroVector;

    // 重置自动偏航状态
    LockedYaw = FMath::DegreesToRadians(CurrentState.GetRotator().Yaw);
    bYawInitialized = false;

    UE_LOG(LogTemp, Log, TEXT("[DroneMovement] InitialState set, HoverSpeed=%.1f rad/s, GroundZ=%.2f, Grounded=true"),
        HoverSpeed, InitialGroundZ);
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

    // 重置自动偏航状态
    LockedYaw = FMath::DegreesToRadians(NewState.GetRotator().Yaw);
    bYawInitialized = false;
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

#include "DroneMovementComponent.h"
#include "GraduationProject/Core/Controller/PDController.h"
#include "GraduationProject/Core/Controller/PIDController.h"

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

void UDroneMovementComponent::BeginPlay()
{
    Super::BeginPlay();
    InitializeControllers();
    ComputeControlAllocation();
    bInitialized = true;
}

void UDroneMovementComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
    if (!bInitialized) return;
    if (CurrentControlMode == EDroneControlMode::Idle) return;

    ControlUpdate(DeltaTime);

    // RK4 子步仿真
    double SubStepTime = Parameters.TimeStep;
    int32 NumSubSteps = FMath::Max(1, FMath::CeilToInt(DeltaTime / SubStepTime));
    SubStepTime = DeltaTime / NumSubSteps;

    for (int32 i = 0; i < NumSubSteps; ++i)
    {
        CurrentState = RK4Update(CurrentState, SubStepTime);
    }
}

// ========== 初始化 ==========

void UDroneMovementComponent::SetParameters(const FDroneParameters& NewParameters)
{
    Parameters = NewParameters;
    ComputeControlAllocation();
    InitializeControllers();
}

void UDroneMovementComponent::InitializeControllers()
{
    double Ts = Parameters.TimeStep;

    float PosPGain = 1.0f;
    float VelMaxLimit = 5.0f;
    
    PxController = NewObject<UPDController>(this);
    PxController->Initialize(PosPGain, 0.0, 0.0, VelMaxLimit, Ts);
    
    PyController = NewObject<UPDController>(this);
    PyController->Initialize(PosPGain, 0.0, 0.0, VelMaxLimit, Ts);
    
    PzController = NewObject<UPDController>(this);
    PzController->Initialize(PosPGain, 0.0, 0.0, VelMaxLimit, Ts);

    float VelPGain = 6.0f;
    float VelDGain = 0.0f;
    
    VxController = NewObject<UPIDController>(this);
    VxController->Initialize(VelPGain, 0.5, VelDGain, 0.05, 10.0, Ts, -20.0, 20.0);
    
    VyController = NewObject<UPIDController>(this);
    VyController->Initialize(VelPGain, 0.5, VelDGain, 0.05, 10.0, Ts, -20.0, 20.0);
    
    VzController = NewObject<UPIDController>(this);
    VzController->Initialize(VelPGain, 0.5, VelDGain, 0.05, 10.0, Ts, -20.0, 20.0);

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

void UDroneMovementComponent::ComputeControlAllocation()
{
    double kT = Parameters.ThrustCoefficient;
    double kQ = Parameters.TorqueCoefficient;
    double L = Parameters.ArmLength;
    double Beta = FMath::DegreesToRadians(Parameters.MotorAngle);
    double SinB = FMath::Sin(Beta);
    double CosB = FMath::Cos(Beta);

    // 推力行
    G[0][0] = kT; G[0][1] = kT; G[0][2] = kT; G[0][3] = kT;
    // 滚转力矩行
    G[1][0] = kT*L*SinB; G[1][1] = -kT*L*SinB; G[1][2] = -kT*L*SinB; G[1][3] = kT*L*SinB;
    // 俯仰力矩行
    G[2][0] = -kT*L*CosB; G[2][1] = -kT*L*CosB; G[2][2] = kT*L*CosB; G[2][3] = kT*L*CosB;
    // 偏航力矩行
    G[3][0] = kQ; G[3][1] = -kQ; G[3][2] = kQ; G[3][3] = -kQ;

    // X 型四旋翼的解析解
    double a = 1.0 / (4.0 * kT);
    double b = 1.0 / (4.0 * kT * L * SinB);
    double c = 1.0 / (4.0 * kT * L * CosB);
    double d = 1.0 / (4.0 * kQ);

    GInv[0][0] = a; GInv[0][1] = b;  GInv[0][2] = -c; GInv[0][3] = d;
    GInv[1][0] = a; GInv[1][1] = -b; GInv[1][2] = -c; GInv[1][3] = -d;
    GInv[2][0] = a; GInv[2][1] = -b; GInv[2][2] = c;  GInv[2][3] = d;
    GInv[3][0] = a; GInv[3][1] = b;  GInv[3][2] = c;  GInv[3][3] = -d;
}

// ========== 控制更新 ==========

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
        if (ControlCommands.Num() >= 4)
        {
            CurrentState.MotorSpeeds = ControlCommands;
        }
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
            double Roll = FMath::DegreesToRadians(CurrentRot.Roll);
            double Pitch = FMath::DegreesToRadians(CurrentRot.Pitch);
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
        double Omega = FMath::Sqrt(FMath::Max(0.0, OmegaSquared[i]));
        Omega = FMath::Clamp(Omega, Parameters.MinMotorSpeed, Parameters.MaxMotorSpeed);
        MotorSpeeds.Add(Omega);
    }
    return MotorSpeeds;
}

// ========== RK4 动力学仿真 ==========

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

TArray<double> UDroneMovementComponent::Derivatives(const FDroneState& State, const TArray<double>& TotalForce, const TArray<double>& Torque)
{
    TArray<double> Deriv;
    
    // 位置导数 = 速度
    Deriv.Add(State.Vx);
    Deriv.Add(State.Vy);
    Deriv.Add(State.Vz);

    // 速度导数 = 加速度
    double m = Parameters.Mass;
    Deriv.Add(TotalForce[0] / m);
    Deriv.Add(TotalForce[1] / m);
    Deriv.Add(TotalForce[2] / m);

    // 四元数导数
    double p = State.AngRollRate, q = State.AngPitchRate, r = State.AngYawRate;
    double qw = State.Qw, qx = State.Qx, qy = State.Qy, qz = State.Qz;
    
    Deriv.Add(0.5 * (-p*qx - q*qy - r*qz));  // dQw
    Deriv.Add(0.5 * (p*qw + r*qy - q*qz));    // dQx
    Deriv.Add(0.5 * (q*qw - r*qx + p*qz));    // dQy
    Deriv.Add(0.5 * (r*qw + q*qx - p*qy));    // dQz

    // 角速度导数 = (力矩 - 陀螺力矩) / 转动惯量
    double Jx = Parameters.Jx, Jy = Parameters.Jy, Jz = Parameters.Jz;
    Deriv.Add((Torque[0] - (Jz - Jy) * q * r) / Jx);
    Deriv.Add((Torque[1] - (Jx - Jz) * p * r) / Jy);
    Deriv.Add((Torque[2] - (Jy - Jx) * p * q) / Jz);

    return Deriv;
}

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

// ========== 级联控制环路 ==========

FVector UDroneMovementComponent::PositionLoop(const FVector& PositionCommand)
{
    FVector Pos = CurrentState.GetPosition();
    double VxCmd = PxController ? PxController->Update(PositionCommand.X, Pos.X) : 0.0;
    double VyCmd = PyController ? PyController->Update(PositionCommand.Y, Pos.Y) : 0.0;
    double VzCmd = PzController ? PzController->Update(PositionCommand.Z, Pos.Z) : 0.0;
    return FVector(VxCmd, VyCmd, VzCmd);
}

FVector UDroneMovementComponent::VelocityLoop(const FVector& VelocityCommand, double& OutThrust)
{
    FVector Vel = CurrentState.GetVelocity();
    double AxCmd = VxController ? VxController->Update(VelocityCommand.X, Vel.X) : 0.0;
    double AyCmd = VyController ? VyController->Update(VelocityCommand.Y, Vel.Y) : 0.0;
    double AzCmd = VzController ? VzController->Update(VelocityCommand.Z, Vel.Z) : 0.0;
    OutThrust = Parameters.Mass * (AzCmd + Parameters.Gravity);
    OutThrust = FMath::Max(0.0, OutThrust);
    return FVector(AxCmd, AyCmd, AzCmd);
}

FVector UDroneMovementComponent::AttitudeLoop(const FVector& AccelerationCommand)
{
    double AxDes = AccelerationCommand.X;
    double AyDes = AccelerationCommand.Y;
    double g = Parameters.Gravity;
    double RollDes = FMath::Atan2(AyDes, g);
    double PitchDes = FMath::Atan2(-AxDes, g);
    double YawDes = FMath::DegreesToRadians(TargetAttitude.Yaw);
    RollDes = FMath::Clamp(RollDes, -0.5, 0.5);   // ±28.6°
    PitchDes = FMath::Clamp(PitchDes, -0.5, 0.5);

    FRotator CurrentRot = CurrentState.GetRotator();
    double Roll = FMath::DegreesToRadians(CurrentRot.Roll);
    double Pitch = FMath::DegreesToRadians(CurrentRot.Pitch);
    double Yaw = FMath::DegreesToRadians(CurrentRot.Yaw);

    double RollTarget = Roll + NormalizeAngle(RollDes - Roll);
    double PitchTarget = Pitch + NormalizeAngle(PitchDes - Pitch);
    double YawTarget = Yaw + NormalizeAngle(YawDes - Yaw);

    double RollRateCmd = RollController ? RollController->Update(RollTarget, Roll) : 0.0;
    double PitchRateCmd = PitchController ? PitchController->Update(PitchTarget, Pitch) : 0.0;
    double YawRateCmd = YawController ? YawController->Update(YawTarget, Yaw) : 0.0;
    return FVector(RollRateCmd, PitchRateCmd, YawRateCmd);
}

FVector UDroneMovementComponent::AngularVelocityLoop(const FVector& AngularVelocityCommand)
{
    double TorqueX = RollRateController ? RollRateController->Update(AngularVelocityCommand.X, CurrentState.AngRollRate) : 0.0;
    double TorqueY = PitchRateController ? PitchRateController->Update(AngularVelocityCommand.Y, CurrentState.AngPitchRate) : 0.0;
    double TorqueZ = YawRateController ? YawRateController->Update(AngularVelocityCommand.Z, CurrentState.AngYawRate) : 0.0;
    return FVector(TorqueX, TorqueY, TorqueZ);
}

// ========== 设置接口 ==========

void UDroneMovementComponent::SetInitialState(const FDroneState& InitialState)
{
    CurrentState = InitialState;
    double HoverSpeed = Parameters.GetHoverMotorSpeed();
    if (CurrentState.MotorSpeeds.Num() < 4)
        CurrentState.MotorSpeeds = {HoverSpeed, HoverSpeed, HoverSpeed, HoverSpeed};
}

void UDroneMovementComponent::SetControlMode(EDroneControlMode NewMode)
{
    CurrentControlMode = NewMode;
    ResetAllControllers();
}

void UDroneMovementComponent::SetControlCommand(const TArray<double>& Command) { ControlCommands = Command; }
void UDroneMovementComponent::SetTargetPosition(const FVector& TargetPos) { TargetPosition = TargetPos; }
void UDroneMovementComponent::SetTargetVelocity(const FVector& TargetVel) { TargetVelocity = TargetVel; }

void UDroneMovementComponent::SetTargetAttitude(const FRotator& Attitude, float Thrust)
{
    TargetAttitude = Attitude;
    TargetThrust = Thrust * Parameters.Mass * Parameters.Gravity;
}

void UDroneMovementComponent::ResetState(const FDroneState& NewState)
{
    CurrentState = NewState;
    ResetAllControllers();
}

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

// ========== PID 调参接口（新增） ==========

void UDroneMovementComponent::SetPositionGains(float Kp, float Kd)
{
    if (PxController) PxController->SetParameters(Kp, Kd, 0.0, 5.0);
    if (PyController) PyController->SetParameters(Kp, Kd, 0.0, 5.0);
    if (PzController) PzController->SetParameters(Kp, Kd, 0.0, 5.0);
}

void UDroneMovementComponent::SetVelocityGains(float Kp, float Ki, float Kd)
{
    if (VxController) VxController->SetParameters(Kp, Ki, Kd, 0.05, 10.0);
    if (VyController) VyController->SetParameters(Kp, Ki, Kd, 0.05, 10.0);
    if (VzController) VzController->SetParameters(Kp, Ki, Kd, 0.05, 10.0);
}

void UDroneMovementComponent::SetAttitudeGains(float Kp, float Kd)
{
    if (RollController) RollController->SetParameters(Kp, Kd, 0.0, 3.0);
    if (PitchController) PitchController->SetParameters(Kp, Kd, 0.0, 3.0);
    if (YawController) YawController->SetParameters(Kp * 0.5f, Kd, 0.0, 3.0);
}

void UDroneMovementComponent::SetAngleRateGains(float Kp)
{
    if (RollRateController) RollRateController->SetParameters(Kp, 0, 0, 0.05, 1.0);
    if (PitchRateController) PitchRateController->SetParameters(Kp, 0, 0, 0.05, 1.0);
    if (YawRateController) YawRateController->SetParameters(Kp * 0.5f, 0, 0, 0.05, 0.5);
}

// ========== 辅助 ==========

FVector UDroneMovementComponent::RotateBodyToWorld(const FVector& BodyVector, const FQuat& Orientation)
{
    return Orientation.RotateVector(BodyVector);
}

double UDroneMovementComponent::NormalizeAngle(double AngleRad)
{
    while (AngleRad > PI) AngleRad -= 2.0 * PI;
    while (AngleRad < -PI) AngleRad += 2.0 * PI;
    return AngleRad;
}

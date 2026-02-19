#include "PIDController.h"

UPIDController::UPIDController(): 
    Kp(1.0),                    // 比例增益
    Ki(0.1),                    // 积分增益
    Kd(0.05),                   // 微分增益
    DiffFilterTau(0.05),        // 微分滤波器时间常数
    OutputLimit(0.0),           // 输出限幅
    IntegratorMin(-100.0),      // 积分器下限
    IntegratorMax(100.0),       // 积分器上限
    TimeStep(0.02),             // 采样时间步长
    bInitialized(false),        // 控制器是否已初始化
    Integrator(0.0),            // 积分器累积值
    DiffFilterAlpha(0.0),       // 微分滤波器系数 α
    DiffFilterBeta(0.0),        // 微分滤波器系数 β
    PreviousError(0.0),         // 上一次的误差值
    PreviousErrorRate(0.0)       // 上一次的误差变化率
{
}

void UPIDController::Initialize(double InputKp, double InputKi, double InputKd, 
                                double InputDiffFilterTau, double InputOutputLimit, double InputTimeStep,
                                double InputIntegratorMin, double InputIntegratorMax)
{
    Kp = InputKp;
    Ki = InputKi;
    Kd = InputKd;
    DiffFilterTau = FMath::Clamp(InputDiffFilterTau, 0.001, 1.0);
    OutputLimit = FMath::Max(0.0, InputOutputLimit);
    IntegratorMin = InputIntegratorMin;
    IntegratorMax = InputIntegratorMax;
    SetTimeStep(InputTimeStep);
    Reset();
    bInitialized = true;
}

double UPIDController::Update(double TargetValue, double CurrentValue, bool bReset)
{
    if (bReset)
    {
        Reset();
    }

    double Error = TargetValue - CurrentValue;
    Integrator += (TimeStep / 2.0) * (Error + PreviousError);
    Integrator = FMath::Clamp(Integrator, IntegratorMin, IntegratorMax);

    double ErrorRate = DiffFilterAlpha * (Error - PreviousError) + DiffFilterBeta * PreviousErrorRate;

    double Output = Kp * Error + Ki * Integrator + Kd * ErrorRate;

    if (OutputLimit > 0.0)
    {
        Output = FMath::Clamp(Output, -OutputLimit, OutputLimit);
    }

    PreviousError = Error;
    PreviousErrorRate = ErrorRate;

    return Output;
}

void UPIDController::SetParameters(double NewKp, double NewKi, double NewKd, double NewDiffFilterTau, double NewOutputLimit)
{
    Kp = NewKp;
    Ki = NewKi;
    Kd = NewKd;
    DiffFilterTau = FMath::Clamp(NewDiffFilterTau, 0.001, 1.0);
    OutputLimit = FMath::Max(0.0, NewOutputLimit);
    SetTimeStep(TimeStep);
}

void UPIDController::SetTimeStep(double NewTimeStep)
{
    TimeStep = FMath::Max(0.001, NewTimeStep);
    double Tau = DiffFilterTau;
    DiffFilterAlpha = 2.0 / (2.0 * Tau + TimeStep);
    DiffFilterBeta = (2.0 * Tau - TimeStep) / (2.0 * Tau + TimeStep);
}

void UPIDController::SetIntegratorLimits(double NewMin, double NewMax)
{
    IntegratorMin = NewMin;
    IntegratorMax = NewMax;
}

void UPIDController::Reset()
{
    Integrator = 0.0;
    PreviousError = 0.0;
    PreviousErrorRate = 0.0;
}

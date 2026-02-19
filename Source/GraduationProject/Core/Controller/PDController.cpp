#include "PDController.h"

UPDController::UPDController(): 
    Kp(1.0),                // 比例增益
    Kd(0.1),                // 微分增益
    DiffFilterTau(0.05),    // 微分滤波器时间常数
    OutputLimit(0.0),       // 输出限幅
    TimeStep(0.02),         // 采样时间步长
    bInitialized(false),    // 是否已初始化
    DiffFilterAlpha(0.0),   // 微分滤波器系数 α
    DiffFilterBeta(0.0),    // 微分滤波器系数 β
    PreviousError(0.0),     // 上一次的误差值
    PreviousErrorRate(0.0)  // 上一次的误差变化率
{
}

void UPDController::Initialize(double InputKp, double InputKd, double InputDiffFilterTau, double InputOutputLimit, double InputTimeStep)
{
    Kp = InputKp;
    Kd = InputKd;
    DiffFilterTau = FMath::Clamp(InputDiffFilterTau, 0.001, 1.0);
    OutputLimit = FMath::Max(0.0, InputOutputLimit);
    SetTimeStep(InputTimeStep);
    Reset();
    bInitialized = true;
}

double UPDController::Update(double TargetValue, double CurrentValue, bool bReset)
{
    if (bReset)
    {
        Reset();
    }

    double Error = TargetValue - CurrentValue;

    double ErrorRate = DiffFilterAlpha * (Error - PreviousError) + DiffFilterBeta * PreviousErrorRate;

    double Output = Kp * Error + Kd * ErrorRate;

    if (OutputLimit > 0.0)
    {
        Output = FMath::Clamp(Output, -OutputLimit, OutputLimit);
    }

    PreviousError = Error;
    PreviousErrorRate = ErrorRate;

    return Output;
}

void UPDController::SetParameters(double NewKp, double NewKd, double NewDiffFilterTau, double NewOutputLimit)
{
    Kp = NewKp;
    Kd = NewKd;
    DiffFilterTau = FMath::Clamp(NewDiffFilterTau, 0.001, 1.0);
    OutputLimit = FMath::Max(0.0, NewOutputLimit);
    SetTimeStep(TimeStep);
}

void UPDController::SetTimeStep(double NewTimeStep)
{
    TimeStep = FMath::Max(0.001, NewTimeStep);
    double Tau = DiffFilterTau;
    DiffFilterAlpha = 2.0 / (2.0 * Tau + TimeStep);
    DiffFilterBeta = (2.0 * Tau - TimeStep) / (2.0 * Tau + TimeStep);
}

void UPDController::Reset()
{
    PreviousError = 0.0;
    PreviousErrorRate = 0.0;
}

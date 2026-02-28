#include "PIDController.h"

UPIDController::UPIDController(): 
    Kp(1.0),                    // 比例增益
    Ki(0.1),                    // 积分增益
    Kd(0.05),                   // 微分增益
    DiffFilterTau(0.05),        // 微分滤波器时间常数（秒）
    OutputLimit(0.0),           // 输出限幅
    IntegratorMin(-100.0),      // 积分器下限
    IntegratorMax(100.0),       // 积分器上限
    TimeStep(0.02),             // 采样时间步长 50Hz
    bInitialized(false),        // 控制器是否已初始化
    Integrator(0.0),            // 积分器累积值
    DiffFilterAlpha(0.0),       // 微分滤波器系数 α
    DiffFilterBeta(0.0),        // 微分滤波器系数 β
    PreviousError(0.0),         // 上一次的误差值
    PreviousErrorRate(0.0)      // 上一次的误差变化率
{
}

/**
 * 初始化控制器参数并重置内部状态
 * @param InputKp 比例增益
 * @param InputKi 积分增益
 * @param InputKd 微分增益
 * @param InputDiffFilterTau 微分滤波时间常数
 * @param InputOutputLimit 输出限幅值
 * @param InputTimeStep 采样时间步长（秒）
 * @param InputIntegratorMin 积分器下限
 * @param InputIntegratorMax 积分器上限
 */
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

/**
 *  PID 控制更新
 * @param TargetValue 目标值
 * @param CurrentValue 当前测量值
 * @param bReset 是否在计算前重置状态
 * @return 控制输出量
 *  e = target - current
 *  I += (T/2) * (e + e_prev)，并限幅
 *  α = 2 / (2τ + T), β = (2τ - T) / (2τ + T)
 *  de = α * (e[k] - e[k-1]) + β * de[k-1]
 *  u = Kp*e + Ki*I + Kd*de
 */
double UPIDController::Update(double TargetValue, double CurrentValue, bool bReset)
{
    if (bReset) Reset();
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

/**
 * 运行时修改 PID 参数
 * @param NewKp 新的比例增益
 * @param NewKi 新的积分增益
 * @param NewKd 新的微分增益
 * @param NewDiffFilterTau 新的微分滤波时间常数
 * @param NewOutputLimit 新的输出限幅值
 */
void UPIDController::SetParameters(double NewKp, double NewKi, double NewKd, double NewDiffFilterTau, double NewOutputLimit)
{
    Kp = NewKp;
    Ki = NewKi;
    Kd = NewKd;
    DiffFilterTau = FMath::Clamp(NewDiffFilterTau, 0.001, 1.0);
    OutputLimit = FMath::Max(0.0, NewOutputLimit);
    SetTimeStep(TimeStep); // 重新计算滤波器系数
}

/**
 * 设置采样时间步长，并重新计算微分滤波器系数
 * @param NewTimeStep 新的时间步长（秒）
 * α = 2 / (2τ + T)
 * β = (2τ - T) / (2τ + T)
 */
void UPIDController::SetTimeStep(double NewTimeStep)
{
    TimeStep = FMath::Max(0.001, NewTimeStep);
    double Tau = DiffFilterTau;
    DiffFilterAlpha = 2.0 / (2.0 * Tau + TimeStep);
    DiffFilterBeta = (2.0 * Tau - TimeStep) / (2.0 * Tau + TimeStep);
}

/**
 * 设置积分器限幅范围
 * @param NewMin 积分器下限
 * @param NewMax 积分器上限
 */
void UPIDController::SetIntegratorLimits(double NewMin, double NewMax)
{
    IntegratorMin = NewMin;
    IntegratorMax = NewMax;
}

/** 重置控制器内部状态 */
void UPIDController::Reset()
{
    Integrator = 0.0;
    PreviousError = 0.0;
    PreviousErrorRate = 0.0;
}

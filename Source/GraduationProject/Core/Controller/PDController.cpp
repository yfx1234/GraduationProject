#include "PDController.h"

UPDController::UPDController(): 
    Kp(1.0),                // 比例增益
    Kd(0.1),                // 微分增益
    DiffFilterTau(0.05),    // 微分滤波器时间常数（秒）
    OutputLimit(0.0),       // 输出限幅
    TimeStep(0.02),         // 采样时间步长 50Hz
    bInitialized(false),    // 是否已初始化
    DiffFilterAlpha(0.0),   // 微分滤波器系数 α
    DiffFilterBeta(0.0),    // 微分滤波器系数 β
    PreviousError(0.0),     // 上一次的误差值
    PreviousErrorRate(0.0)  // 上一次的误差变化率
{
}

/**
 * 初始化控制器参数并重置内部状态
 * @param InputKp 比例增益
 * @param InputKd 微分增益
 * @param InputDiffFilterTau 微分滤波时间常数
 * @param InputOutputLimit 输出限幅值
 * @param InputTimeStep 采样时间步长（秒）
 * 调用 SetTimeStep() 计算滤波器系数，调用 Reset() 清零历史状态。
 */
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

/**
 * PD 控制更新
 * @param TargetValue 目标值
 * @param CurrentValue 当前测量值
 * @param bReset 是否在计算前重置状态
 * @return 控制输出量
 * e = target - current
 * α = 2 / (2τ + T), β = (2τ - T) / (2τ + T)
 * de = α * (e[k] - e[k-1]) + β * de[k-1]
 * u = Kp*e + Kd*de
 */
double UPDController::Update(double TargetValue, double CurrentValue, bool bReset)
{
    if (bReset) Reset();
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

/**
 * 运行时修改 PD 参数
 * @param NewKp 新的比例增益
 * @param NewKd 新的微分增益
 * @param NewDiffFilterTau 新的微分滤波时间常数
 * @param NewOutputLimit 新的输出限幅值
 */
void UPDController::SetParameters(double NewKp, double NewKd, double NewDiffFilterTau, double NewOutputLimit)
{
    Kp = NewKp;
    Kd = NewKd;
    DiffFilterTau = FMath::Clamp(NewDiffFilterTau, 0.001, 1.0);
    OutputLimit = FMath::Max(0.0, NewOutputLimit);
    SetTimeStep(TimeStep); // 重新计算滤波器系数
}

/**
 * 设置采样时间步长，并重新计算微分滤波器系数
 * @param NewTimeStep 新的时间步长（秒）
 * 滤波器系数公式（双线性变换/Tustin法）：
 *  α = 2 / (2τ + T)
 *  β = (2τ - T) / (2τ + T)
 */
void UPDController::SetTimeStep(double NewTimeStep)
{
    TimeStep = FMath::Max(0.001, NewTimeStep);
    double Tau = DiffFilterTau;
    DiffFilterAlpha = 2.0 / (2.0 * Tau + TimeStep);
    DiffFilterBeta = (2.0 * Tau - TimeStep) / (2.0 * Tau + TimeStep);
}

/**
 * 重置控制器内部状态
 * 将上一步误差和误差变化率清零。
 */
void UPDController::Reset()
{
    PreviousError = 0.0;
    PreviousErrorRate = 0.0;
}

/**
 * @file PIDController.cpp
 * @brief PID（比例-积分-微分）控制器的实现文件
 *
 * 实现带梯形积分和一阶低通滤波微分项的 PID 控制器。
 */

#include "PIDController.h"

/**
 * @brief 默认构造函数，设置所有参数为默认值
 */
UPIDController::UPIDController(): 
    Kp(1.0),                    // 比例增益
    Ki(0.1),                    // 积分增益
    Kd(0.05),                   // 微分增益
    DiffFilterTau(0.05),        // 微分滤波器时间常数（秒）
    OutputLimit(0.0),           // 输出限幅（0=不限幅）
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
 * @brief 初始化控制器参数并重置内部状态
 * @param InputKp 比例增益
 * @param InputKi 积分增益
 * @param InputKd 微分增益
 * @param InputDiffFilterTau 微分滤波时间常数，范围 [0.001, 1.0]
 * @param InputOutputLimit 输出限幅值（0=不限幅）
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
 * @brief 执行一步 PID 控制更新
 * @param TargetValue 目标值
 * @param CurrentValue 当前测量值
 * @param bReset 是否在计算前重置状态
 * @return 控制输出量
 *
 * 计算流程：
 * 1. 计算误差 e = target - current
 * 2. 梯形积分法更新积分器：I += (T/2) * (e + e_prev)，并限幅
 * 3. 一阶滤波器计算微分项：de = α*(e - e_prev) + β*de_prev
 * 4. 输出 u = Kp*e + Ki*I + Kd*de
 * 5. 如果 OutputLimit > 0，则限幅
 * 6. 保存当前误差和误差率供下一步使用
 */
double UPIDController::Update(double TargetValue, double CurrentValue, bool bReset)
{
    if (bReset)
    {
        Reset();
    }

    // 步骤1：计算误差
    double Error = TargetValue - CurrentValue;

    // 步骤2：梯形积分法更新积分器（比矩形积分更精确）
    Integrator += (TimeStep / 2.0) * (Error + PreviousError);
    // 积分器限幅，防止积分饱和（Anti-Windup）
    Integrator = FMath::Clamp(Integrator, IntegratorMin, IntegratorMax);

    // 步骤3：一阶低通滤波的微分项
    double ErrorRate = DiffFilterAlpha * (Error - PreviousError) + DiffFilterBeta * PreviousErrorRate;

    // 步骤4：PID 控制律
    double Output = Kp * Error + Ki * Integrator + Kd * ErrorRate;

    // 步骤5：输出限幅
    if (OutputLimit > 0.0)
    {
        Output = FMath::Clamp(Output, -OutputLimit, OutputLimit);
    }

    // 步骤6：保存状态
    PreviousError = Error;
    PreviousErrorRate = ErrorRate;

    return Output;
}

/**
 * @brief 运行时修改 PID 参数
 * @param NewKp 新的比例增益
 * @param NewKi 新的积分增益
 * @param NewKd 新的微分增益
 * @param NewDiffFilterTau 新的微分滤波时间常数
 * @param NewOutputLimit 新的输出限幅值
 *
 * 修改参数后重新计算微分滤波器系数。
 * 不重置积分器和历史状态，保持控制的连续性。
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
 * @brief 设置采样时间步长，并重新计算微分滤波器系数
 * @param NewTimeStep 新的时间步长（秒），下限 0.001
 *
 * 滤波器系数公式（双线性变换/Tustin法）：
 *   α = 2 / (2τ + T)
 *   β = (2τ - T) / (2τ + T)
 */
void UPIDController::SetTimeStep(double NewTimeStep)
{
    TimeStep = FMath::Max(0.001, NewTimeStep);
    double Tau = DiffFilterTau;
    DiffFilterAlpha = 2.0 / (2.0 * Tau + TimeStep);
    DiffFilterBeta = (2.0 * Tau - TimeStep) / (2.0 * Tau + TimeStep);
}

/**
 * @brief 设置积分器限幅范围
 * @param NewMin 积分器下限
 * @param NewMax 积分器上限
 *
 * 用于在运行时调整 Anti-Windup 参数。
 */
void UPIDController::SetIntegratorLimits(double NewMin, double NewMax)
{
    IntegratorMin = NewMin;
    IntegratorMax = NewMax;
}

/**
 * @brief 重置控制器内部状态
 *
 * 清零积分器、误差历史和微分滤波状态。
 * 用于控制模式切换或目标突变时。
 */
void UPIDController::Reset()
{
    Integrator = 0.0;
    PreviousError = 0.0;
    PreviousErrorRate = 0.0;
}

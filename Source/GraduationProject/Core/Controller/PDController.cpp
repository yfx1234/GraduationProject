/**
 * @file PDController.cpp
 * @brief PD（比例-微分）控制器的实现文件
 *
 * 实现带一阶低通滤波微分项的 PD 控制器。
 * 滤波器使用双线性变换（Tustin法），保证数值稳定性。
 */

#include "PDController.h"

/**
 * @brief 默认构造函数，设置所有参数为默认值
 */
UPDController::UPDController(): 
    Kp(1.0),                // 比例增益
    Kd(0.1),                // 微分增益
    DiffFilterTau(0.05),    // 微分滤波器时间常数（秒）
    OutputLimit(0.0),       // 输出限幅（0=不限幅）
    TimeStep(0.02),         // 采样时间步长 50Hz
    bInitialized(false),    // 是否已初始化
    DiffFilterAlpha(0.0),   // 微分滤波器系数 α
    DiffFilterBeta(0.0),    // 微分滤波器系数 β
    PreviousError(0.0),     // 上一次的误差值
    PreviousErrorRate(0.0)  // 上一次的误差变化率
{
}

/**
 * @brief 初始化控制器参数并重置内部状态
 * @param InputKp 比例增益
 * @param InputKd 微分增益
 * @param InputDiffFilterTau 微分滤波时间常数，范围 [0.001, 1.0]
 * @param InputOutputLimit 输出限幅值（0=不限幅）
 * @param InputTimeStep 采样时间步长（秒）
 *
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
 * @brief 执行一步 PD 控制更新
 * @param TargetValue 目标值
 * @param CurrentValue 当前测量值
 * @param bReset 是否在计算前重置状态
 * @return 控制输出量
 *
 * 计算流程：
 * 1. 计算误差 e = target - current
 * 2. 通过一阶滤波器计算微分项：de = α*(e - e_prev) + β*de_prev
 * 3. 输出 u = Kp*e + Kd*de
 * 4. 如果 OutputLimit > 0，则限幅到 [-OutputLimit, OutputLimit]
 * 5. 保存当前误差和误差率供下一步使用
 */
double UPDController::Update(double TargetValue, double CurrentValue, bool bReset)
{
    if (bReset)
    {
        Reset();
    }

    // 步骤1：计算误差
    double Error = TargetValue - CurrentValue;

    // 步骤2：一阶低通滤波的微分项
    // 公式来源于双线性变换（Tustin法）: s ≈ 2/T * (z-1)/(z+1)
    double ErrorRate = DiffFilterAlpha * (Error - PreviousError) + DiffFilterBeta * PreviousErrorRate;

    // 步骤3：PD 控制律
    double Output = Kp * Error + Kd * ErrorRate;

    // 步骤4：输出限幅
    if (OutputLimit > 0.0)
    {
        Output = FMath::Clamp(Output, -OutputLimit, OutputLimit);
    }

    // 步骤5：保存状态
    PreviousError = Error;
    PreviousErrorRate = ErrorRate;

    return Output;
}

/**
 * @brief 运行时修改 PD 参数
 * @param NewKp 新的比例增益
 * @param NewKd 新的微分增益
 * @param NewDiffFilterTau 新的微分滤波时间常数
 * @param NewOutputLimit 新的输出限幅值
 *
 * 修改参数后重新计算微分滤波器系数（调用 SetTimeStep）。
 * 注意：不会重置内部状态，以保持控制的连续性。
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
 * @brief 设置采样时间步长，并重新计算微分滤波器系数
 * @param NewTimeStep 新的时间步长（秒），下限 0.001
 *
 * 滤波器系数公式（双线性变换/Tustin法）：
 *   α = 2 / (2τ + T)
 *   β = (2τ - T) / (2τ + T)
 * 其中 τ = DiffFilterTau，T = TimeStep
 */
void UPDController::SetTimeStep(double NewTimeStep)
{
    TimeStep = FMath::Max(0.001, NewTimeStep);
    double Tau = DiffFilterTau;
    DiffFilterAlpha = 2.0 / (2.0 * Tau + TimeStep);
    DiffFilterBeta = (2.0 * Tau - TimeStep) / (2.0 * Tau + TimeStep);
}

/**
 * @brief 重置控制器内部状态
 *
 * 将上一步误差和误差变化率清零。
 * 用于控制模式切换或目标突变时避免微分项产生尖峰输出。
 */
void UPDController::Reset()
{
    PreviousError = 0.0;
    PreviousErrorRate = 0.0;
}

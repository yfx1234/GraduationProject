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
 * @brief 初始化控制器参数并重置内部状态
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
 * @brief PD 控制更新
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
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行按照控制律合成当前控制输出。
        Output = FMath::Clamp(Output, -OutputLimit, OutputLimit);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行把右侧表达式的结果写入 `PreviousError`，完成 上一时刻误差 的更新。
    PreviousError = Error;
    // 解释：这一行把右侧表达式的结果写入 `PreviousErrorRate`，完成 上一时刻误差变化率 的更新。
    PreviousErrorRate = ErrorRate;
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return Output;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 运行时修改 PD 参数
 * @param NewKp 新的比例增益
 * @param NewKd 新的微分增益
 * @param NewDiffFilterTau 新的微分滤波时间常数
 * @param NewOutputLimit 新的输出限幅值
 */
// 解释：这一行定义函数 `SetParameters`，开始实现set参数的具体逻辑。
void UPDController::SetParameters(double NewKp, double NewKd, double NewDiffFilterTau, double NewOutputLimit)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `Kp`，完成 比例增益 Kp 的更新。
    Kp = NewKp;
    // 解释：这一行把右侧表达式的结果写入 `Kd`，完成 微分增益 Kd 的更新。
    Kd = NewKd;
    // 解释：这一行先对计算结果做限幅，再写入 `DiffFilterTau`，防止 微分滤波时间常数 τ 超出允许范围。
    DiffFilterTau = FMath::Clamp(NewDiffFilterTau, 0.001, 1.0);
    // 解释：这一行通过 `FMath::Max` 给 `OutputLimit` 施加下界约束，避免 输出限幅值 过小。
    OutputLimit = FMath::Max(0.0, NewOutputLimit);
    // 解释：调用 `SetTimeStep` 执行当前步骤需要的功能逻辑。
    SetTimeStep(TimeStep); 
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 设置采样时间步长，并重新计算微分滤波器系数
 * @param NewTimeStep 新的时间步长（秒）
 * 滤波器系数公式（双线性变换/Tustin法）：
 *  α = 2 / (2τ + T)
 *  β = (2τ - T) / (2τ + T)
 */
// 解释：这一行定义函数 `SetTimeStep`，开始实现settimestep的具体逻辑。
void UPDController::SetTimeStep(double NewTimeStep)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行通过 `FMath::Max` 给 `TimeStep` 施加下界约束，避免 采样时间步长 过小。
    TimeStep = FMath::Max(0.001, NewTimeStep);
    // 解释：这一行声明成员或局部变量 `Tau`，用于保存tau。
    double Tau = DiffFilterTau;
    // 解释：这一行根据 α = 2 / (2τ + T) 计算微分滤波器系数 α。
    DiffFilterAlpha = 2.0 / (2.0 * Tau + TimeStep);
    // 解释：这一行根据 β = (2τ - T) / (2τ + T) 计算微分滤波器系数 β。
    DiffFilterBeta = (2.0 * Tau - TimeStep) / (2.0 * Tau + TimeStep);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 重置控制器内部状态
 * 将上一步误差和误差变化率清零。
 */
// 解释：这一行定义函数 `Reset`，开始实现reset的具体逻辑。
void UPDController::Reset()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `PreviousError`，完成 上一时刻误差 的更新。
    PreviousError = 0.0;
    // 解释：这一行把右侧表达式的结果写入 `PreviousErrorRate`，完成 上一时刻误差变化率 的更新。
    PreviousErrorRate = 0.0;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

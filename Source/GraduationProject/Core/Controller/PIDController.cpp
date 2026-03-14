// 解释：引入当前实现文件对应的头文件 `PIDController.h`，使实现部分能够看到类和函数声明。
#include "PIDController.h"

// 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
UPIDController::UPIDController(): 
    // 解释：这一行位于构造函数初始化列表中，把 `Kp` 直接初始化为 `1.0`，减少进入函数体后的额外赋值开销。
    Kp(1.0),                    // 比例增益
    // 解释：这一行位于构造函数初始化列表中，把 `Ki` 直接初始化为 `0.1`，减少进入函数体后的额外赋值开销。
    Ki(0.1),                    // 积分增益
    // 解释：这一行位于构造函数初始化列表中，把 `Kd` 直接初始化为 `0.05`，减少进入函数体后的额外赋值开销。
    Kd(0.05),                   // 微分增益
    // 解释：这一行位于构造函数初始化列表中，把 `DiffFilterTau` 直接初始化为 `0.05`，减少进入函数体后的额外赋值开销。
    DiffFilterTau(0.05),        // 微分滤波器时间常数（秒）
    // 解释：这一行位于构造函数初始化列表中，把 `OutputLimit` 直接初始化为 `0.0`，减少进入函数体后的额外赋值开销。
    OutputLimit(0.0),           // 输出限幅
    // 解释：这一行位于构造函数初始化列表中，把 `IntegratorMin` 直接初始化为 `-100.0`，减少进入函数体后的额外赋值开销。
    IntegratorMin(-100.0),      // 积分器下限
    // 解释：这一行位于构造函数初始化列表中，把 `IntegratorMax` 直接初始化为 `100.0`，减少进入函数体后的额外赋值开销。
    IntegratorMax(100.0),       // 积分器上限
    // 解释：这一行位于构造函数初始化列表中，把 `TimeStep` 直接初始化为 `0.02`，减少进入函数体后的额外赋值开销。
    TimeStep(0.02),             // 采样时间步长 50Hz
    // 解释：这一行位于构造函数初始化列表中，把 `bInitialized` 直接初始化为 `false`，减少进入函数体后的额外赋值开销。
    bInitialized(false),        // 控制器是否已初始化
    // 解释：这一行位于构造函数初始化列表中，把 `Integrator` 直接初始化为 `0.0`，减少进入函数体后的额外赋值开销。
    Integrator(0.0),            // 积分器累积值
    // 解释：这一行位于构造函数初始化列表中，把 `DiffFilterAlpha` 直接初始化为 `0.0`，减少进入函数体后的额外赋值开销。
    DiffFilterAlpha(0.0),       // 微分滤波器系数 α
    // 解释：这一行位于构造函数初始化列表中，把 `DiffFilterBeta` 直接初始化为 `0.0`，减少进入函数体后的额外赋值开销。
    DiffFilterBeta(0.0),        // 微分滤波器系数 β
    // 解释：这一行位于构造函数初始化列表中，把 `PreviousError` 直接初始化为 `0.0`，减少进入函数体后的额外赋值开销。
    PreviousError(0.0),         // 上一次的误差值
    // 解释：这一行位于构造函数初始化列表中，把 `PreviousErrorRate` 直接初始化为 `0.0`，减少进入函数体后的额外赋值开销。
    PreviousErrorRate(0.0)      // 上一次的误差变化率
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 初始化控制器参数并重置内部状态
 * @param InputKp 比例增益
 * @param InputKi 积分增益
 * @param InputKd 微分增益
 * @param InputDiffFilterTau 微分滤波时间常数
 * @param InputOutputLimit 输出限幅值
 * @param InputTimeStep 采样时间步长（秒）
 * @param InputIntegratorMin 积分器下限
 * @param InputIntegratorMax 积分器上限
 */
// 解释：这一行定义函数 `Initialize`，开始实现initialize的具体逻辑。
void UPIDController::Initialize(double InputKp, double InputKi, double InputKd, 
                                // 解释：这一行继续补充函数 `Initialize` 的参数列表、限定符或返回类型说明。
                                double InputDiffFilterTau, double InputOutputLimit, double InputTimeStep,
                                // 解释：这一行收束函数 `Initialize` 的签名，后面会进入实现体或以分号结束声明。
                                double InputIntegratorMin, double InputIntegratorMax)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `Kp`，完成 比例增益 Kp 的更新。
    Kp = InputKp;
    // 解释：这一行把右侧表达式的结果写入 `Ki`，完成 积分增益 Ki 的更新。
    Ki = InputKi;
    // 解释：这一行把右侧表达式的结果写入 `Kd`，完成 微分增益 Kd 的更新。
    Kd = InputKd;
    // 解释：这一行先对计算结果做限幅，再写入 `DiffFilterTau`，防止 微分滤波时间常数 τ 超出允许范围。
    DiffFilterTau = FMath::Clamp(InputDiffFilterTau, 0.001, 1.0);
    // 解释：这一行通过 `FMath::Max` 给 `OutputLimit` 施加下界约束，避免 输出限幅值 过小。
    OutputLimit = FMath::Max(0.0, InputOutputLimit);
    // 解释：这一行把右侧表达式的结果写入 `IntegratorMin`，完成 积分器下限 的更新。
    IntegratorMin = InputIntegratorMin;
    // 解释：这一行把右侧表达式的结果写入 `IntegratorMax`，完成 积分器上限 的更新。
    IntegratorMax = InputIntegratorMax;
    // 解释：调用 `SetTimeStep` 执行当前步骤需要的功能逻辑。
    SetTimeStep(InputTimeStep);
    // 解释：调用 `Reset` 执行当前步骤需要的功能逻辑。
    Reset();
    // 解释：这一行把右侧表达式的结果写入 `bInitialized`，完成 布尔标志 initialized 的更新。
    bInitialized = true;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief PID 控制更新
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
// 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
double UPIDController::Update(double TargetValue, double CurrentValue, bool bReset)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (bReset) Reset();
    // 解释：这一行声明成员或局部变量 `Error`，用于保存控制误差。
    double Error = TargetValue - CurrentValue;
    // 解释：这一行更新积分项，按照离散积分公式累加历史误差。
    Integrator += (TimeStep / 2.0) * (Error + PreviousError);
    // 解释：这一行更新积分项，按照离散积分公式累加历史误差。
    Integrator = FMath::Clamp(Integrator, IntegratorMin, IntegratorMax);
    // 解释：这一行把右侧表达式的结果写入 `double ErrorRate`，完成 doubleerrorrate 的更新。
    double ErrorRate = DiffFilterAlpha * (Error - PreviousError) + DiffFilterBeta * PreviousErrorRate;
    // 解释：这一行声明成员或局部变量 `Output`，用于保存控制输出。
    double Output = Kp * Error + Ki * Integrator + Kd * ErrorRate;
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
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
 * @brief 运行时修改 PID 参数
 * @param NewKp 新的比例增益
 * @param NewKi 新的积分增益
 * @param NewKd 新的微分增益
 * @param NewDiffFilterTau 新的微分滤波时间常数
 * @param NewOutputLimit 新的输出限幅值
 */
// 解释：这一行定义函数 `SetParameters`，开始实现set参数的具体逻辑。
void UPIDController::SetParameters(double NewKp, double NewKi, double NewKd, double NewDiffFilterTau, double NewOutputLimit)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `Kp`，完成 比例增益 Kp 的更新。
    Kp = NewKp;
    // 解释：这一行把右侧表达式的结果写入 `Ki`，完成 积分增益 Ki 的更新。
    Ki = NewKi;
    // 解释：这一行把右侧表达式的结果写入 `Kd`，完成 微分增益 Kd 的更新。
    Kd = NewKd;
    // 解释：这一行先对计算结果做限幅，再写入 `DiffFilterTau`，防止 微分滤波时间常数 τ 超出允许范围。
    DiffFilterTau = FMath::Clamp(NewDiffFilterTau, 0.001, 1.0);
    // 解释：这一行通过 `FMath::Max` 给 `OutputLimit` 施加下界约束，避免 输出限幅值 过小。
    OutputLimit = FMath::Max(0.0, NewOutputLimit);
    // 解释：调用 `SetTimeStep` 执行当前步骤需要的功能逻辑。
    SetTimeStep(TimeStep); // 重新计算滤波器系数
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 设置采样时间步长，并重新计算微分滤波器系数
 * @param NewTimeStep 新的时间步长（秒）
 * α = 2 / (2τ + T)
 * β = (2τ - T) / (2τ + T)
 */
// 解释：这一行定义函数 `SetTimeStep`，开始实现settimestep的具体逻辑。
void UPIDController::SetTimeStep(double NewTimeStep)
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
 * @brief 设置积分器限幅范围
 * @param NewMin 积分器下限
 * @param NewMax 积分器上限
 */
// 解释：这一行定义函数 `SetIntegratorLimits`，开始实现setintegratorlimits的具体逻辑。
void UPIDController::SetIntegratorLimits(double NewMin, double NewMax)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `IntegratorMin`，完成 积分器下限 的更新。
    IntegratorMin = NewMin;
    // 解释：这一行把右侧表达式的结果写入 `IntegratorMax`，完成 积分器上限 的更新。
    IntegratorMax = NewMax;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 重置控制器内部状态 */
// 解释：这一行定义函数 `Reset`，开始实现reset的具体逻辑。
void UPIDController::Reset()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行更新积分项，按照离散积分公式累加历史误差。
    Integrator = 0.0;
    // 解释：这一行把右侧表达式的结果写入 `PreviousError`，完成 上一时刻误差 的更新。
    PreviousError = 0.0;
    // 解释：这一行把右侧表达式的结果写入 `PreviousErrorRate`，完成 上一时刻误差变化率 的更新。
    PreviousErrorRate = 0.0;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

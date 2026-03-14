// 解释：引入当前实现文件对应的头文件 `GuidanceMethods.h`，使实现部分能够看到类和函数声明。
#include "GuidanceMethods.h"
// 解释：引入 `ITargetPredictor.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "ITargetPredictor.h"

/**
 * @brief 从方向向量计算 Pitch/Yaw
 * @param AimDir 瞄准方向
 * @param AimPoint 瞄准点
 * @param FlightTime 预估飞行时间
 * @return FGuidanceOutput 瞄准输出
 */
// 解释：这一行定义函数 `DirToAngles`，开始实现dirtoangles的具体逻辑。
static FGuidanceOutput DirToAngles(const FVector& AimDir, const FVector& AimPoint, float FlightTime)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行声明成员或局部变量 `Out`，用于保存out。
    FGuidanceOutput Out;
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (AimDir.IsNearlyZero())
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `Out.bValid`，完成 布尔标志 valid 的更新。
        Out.bValid = false;
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return Out;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行把右侧表达式的结果写入 `const FRotator Rot`，完成 constfrotatorrot 的更新。
    const FRotator Rot = AimDir.GetSafeNormal().Rotation();
    // 解释：这一行把右侧表达式的结果写入 `Out.Pitch`，完成 pitch 的更新。
    Out.Pitch = Rot.Pitch;
    // 解释：这一行把右侧表达式的结果写入 `Out.Yaw`，完成 yaw 的更新。
    Out.Yaw = Rot.Yaw;
    // 解释：这一行把右侧表达式的结果写入 `Out.AimPoint`，完成 aimpoint 的更新。
    Out.AimPoint = AimPoint;
    // 解释：这一行把右侧表达式的结果写入 `Out.EstFlightTime`，完成 estflighttime 的更新。
    Out.EstFlightTime = FlightTime;
    // 解释：这一行把右侧表达式的结果写入 `Out.bValid`，完成 布尔标志 valid 的更新。
    Out.bValid = true;
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return Out;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 直接瞄准
 * @param Input 制导输入数据
 * @return FGuidanceOutput 瞄准输出
 */
// 解释：这一行定义函数 `ComputeAim`，开始实现computeaim的具体逻辑。
FGuidanceOutput FDirectAiming::ComputeAim(const FGuidanceInput& Input)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行声明成员或局部变量 `AimDir`，用于保存aimdir。
    const FVector AimDir = Input.TargetPos - Input.MuzzlePos;
    // 解释：这一行把向量模长写入 `const float Dist`，用于表示距离、速度大小或不确定度。
    const float Dist = AimDir.Size();
    // 解释：这一行把右侧表达式的结果写入 `const float FlightTime`，完成 constfloatflighttime 的更新。
    const float FlightTime = (Input.MuzzleSpeed > 0.0f) ? (Dist / (Input.MuzzleSpeed * 100.0f)) : 0.0f;
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return DirToAngles(AimDir, Input.TargetPos, FlightTime);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 比例导引构造函数 */
// 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
FProportionalNavigation::FProportionalNavigation(float NavConstant)
    // 解释：这一行位于构造函数初始化列表中，把 `N` 直接初始化为 `NavConstant`，减少进入函数体后的额外赋值开销。
    : N(NavConstant)
    // 解释：这一行位于构造函数初始化列表中，把 `LastLOS` 直接初始化为 `FVector::ZeroVector`，减少进入函数体后的额外赋值开销。
    , LastLOS(FVector::ZeroVector)
    // 解释：这一行位于构造函数初始化列表中，把 `bHasLastLOS` 直接初始化为 `false`，减少进入函数体后的额外赋值开销。
    , bHasLastLOS(false)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 重置内部状态 */
// 解释：这一行定义函数 `Reset`，开始实现reset的具体逻辑。
void FProportionalNavigation::Reset()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `LastLOS`，完成 lastlos 的更新。
    LastLOS = FVector::ZeroVector;
    // 解释：这一行把右侧表达式的结果写入 `bHasLastLOS`，完成 布尔标志 haslastlos 的更新。
    bHasLastLOS = false;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 比例导引
 * @param Input 制导输入数据
 * @return FGuidanceOutput 瞄准输出
 * AimDir = LOS + N * LOSRate * FlightTime
 */
// 解释：这一行定义函数 `ComputeAim`，开始实现computeaim的具体逻辑。
FGuidanceOutput FProportionalNavigation::ComputeAim(const FGuidanceInput& Input)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行声明成员或局部变量 `LOS`，用于保存视线方向单位向量。
    const FVector LOS = Input.TargetPos - Input.MuzzlePos;
    // 解释：这一行把向量模长写入 `const float Dist`，用于表示距离、速度大小或不确定度。
    const float Dist = LOS.Size();
    // 解释：这一行把右侧表达式的结果写入 `const float FlightTime`，完成 constfloatflighttime 的更新。
    const float FlightTime = (Input.MuzzleSpeed > 0.0f) ? (Dist / (Input.MuzzleSpeed * 100.0f)) : 0.0f;

    // 首帧还没有 LOS 变化率可用，直接退化为当前视线瞄准。
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!bHasLastLOS)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `LastLOS`，完成 lastlos 的更新。
        LastLOS = LOS;
        // 解释：这一行把右侧表达式的结果写入 `bHasLastLOS`，完成 布尔标志 haslastlos 的更新。
        bHasLastLOS = true;
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return DirToAngles(LOS, Input.TargetPos, FlightTime);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行声明成员或局部变量 `LOSRate`，用于保存losrate。
    FVector LOSRate = FVector::ZeroVector;
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (Input.DeltaTime > KINDA_SMALL_NUMBER)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `LOSRate`，完成 losrate 的更新。
        LOSRate = (LOS - LastLOS) / Input.DeltaTime;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行声明成员或局部变量 `AimDir`，用于保存aimdir。
    const FVector AimDir = LOS + N * LOSRate * FlightTime;
    // 解释：这一行声明成员或局部变量 `AimPoint`，用于保存aimpoint。
    const FVector AimPoint = Input.MuzzlePos + AimDir;
    // 解释：这一行把右侧表达式的结果写入 `LastLOS`，完成 lastlos 的更新。
    LastLOS = LOS;
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return DirToAngles(AimDir, AimPoint, FlightTime);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 预测制导构造函数 */
// 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
FPredictiveGuidance::FPredictiveGuidance(ITargetPredictor* Predictor, int32 Iterations)
    // 解释：这一行位于构造函数初始化列表中，把 `KalmanPredictor` 直接初始化为 `Predictor`，减少进入函数体后的额外赋值开销。
    : KalmanPredictor(Predictor)
    // 解释：这一行位于构造函数初始化列表中，把 `MaxIterations` 直接初始化为 `Iterations`，减少进入函数体后的额外赋值开销。
    , MaxIterations(Iterations)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 预测制导
 * @param Input 制导输入数据
 * @return FGuidanceOutput 瞄准输出
 */
// 解释：这一行定义函数 `ComputeAim`，开始实现computeaim的具体逻辑。
FGuidanceOutput FPredictiveGuidance::ComputeAim(const FGuidanceInput& Input)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!KalmanPredictor || !KalmanPredictor->IsInitialized())
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行声明成员或局部变量 `AimDir`，用于保存aimdir。
        const FVector AimDir = Input.TargetPos - Input.MuzzlePos;
        // 解释：这一行把向量模长写入 `const float Dist`，用于表示距离、速度大小或不确定度。
        const float Dist = AimDir.Size();
        // 解释：这一行把右侧表达式的结果写入 `const float FlightTime`，完成 constfloatflighttime 的更新。
        const float FlightTime = (Input.MuzzleSpeed > 0.0f) ? (Dist / (Input.MuzzleSpeed * 100.0f)) : 0.0f;
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return DirToAngles(AimDir, Input.TargetPos, FlightTime);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行声明成员或局部变量 `MuzzleSpeedCm`，用于保存muzzlespeedcm。
    const float MuzzleSpeedCm = Input.MuzzleSpeed * 100.0f;
    // 解释：这一行声明成员或局部变量 `PredPos`，用于保存predpos。
    FVector PredPos = Input.TargetPos;
    // 解释：这一行声明成员或局部变量 `FlightTime`，用于保存flighttime。
    float FlightTime = 0.0f;

    // 交替迭代“飞行时间 -> 预测目标位置”，逐步逼近拦截时刻的目标位置。
    // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
    for (int32 Iter = 0; Iter < MaxIterations; ++Iter)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `const float Dist`，完成 constfloatdist 的更新。
        const float Dist = FVector::Dist(Input.MuzzlePos, PredPos);
        // 解释：这一行把右侧表达式的结果写入 `FlightTime`，完成 flighttime 的更新。
        FlightTime = (MuzzleSpeedCm > 0.0f) ? (Dist / MuzzleSpeedCm) : 0.0f;
        // 解释：这一行把右侧表达式的结果写入 `PredPos`，完成 predpos 的更新。
        PredPos = KalmanPredictor->PredictPosition(FlightTime);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行声明成员或局部变量 `AimDir`，用于保存aimdir。
    const FVector AimDir = PredPos - Input.MuzzlePos;
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return DirToAngles(AimDir, PredPos, FlightTime);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

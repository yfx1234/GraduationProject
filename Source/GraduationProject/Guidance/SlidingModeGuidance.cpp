// 解释：引入当前实现文件对应的头文件 `SlidingModeGuidance.h`，使实现部分能够看到类和函数声明。
#include "SlidingModeGuidance.h"

/**
 * @brief 计算 SMC 制导的拦截器速度指令（不更新内部状态）
 *
 * 算法流程：
 * 1. 计算相对运动学量：相对位置 r, 相对速度 v_rel
 * 2. 计算视线方向 e_LOS = r / |r|
 * 3. 计算视线角速率 ω_LOS = (e_LOS × v_rel) / |r|
 * 4. 计算 SMC 法向加速度指令:
 *    a_cmd = a_T + (|v_rel|/|r|) * [ N * v_rel_⊥ + K * tanh(ω_LOS / Φ) ]
 * 5. 将加速度指令积分为速度指令
 */
// 解释：这一行定义函数 `ComputeInterceptVelocity`，开始实现compute拦截velocity的具体逻辑。
FVector FSlidingModeGuidance::ComputeInterceptVelocity(
    // 解释：这一行继续展开 `ComputeInterceptVelocity` 的参数列表，声明参数 `InterceptorPos` 用于传入拦截机位置。
    const FVector& InterceptorPos,
    // 解释：这一行继续展开 `ComputeInterceptVelocity` 的参数列表，声明参数 `InterceptorVel` 用于传入拦截机速度。
    const FVector& InterceptorVel,
    // 解释：这一行继续展开 `ComputeInterceptVelocity` 的参数列表，声明参数 `TargetPos` 用于传入目标位置。
    const FVector& TargetPos,
    // 解释：这一行继续展开 `ComputeInterceptVelocity` 的参数列表，声明参数 `TargetVel` 用于传入目标速度。
    const FVector& TargetVel,
    // 解释：这一行继续展开 `ComputeInterceptVelocity` 的参数列表，声明参数 `TargetAcc` 用于传入目标加速度。
    const FVector& TargetAcc,
    // 解释：这一行继续展开 `ComputeInterceptVelocity` 的参数列表，声明参数 `DeltaTime` 用于传入时间步长。
    float DeltaTime,
    // 解释：这一行收束函数 `ComputeInterceptVelocity` 的签名，后面会进入实现体或以分号结束声明。
    float MaxSpeed) const
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // ---------- 相对运动学 ----------
    // 解释：这一行声明成员或局部变量 `RelPos`，用于保存相对位置向量。
    const FVector RelPos = TargetPos - InterceptorPos;     // 相对位置 (拦截器指向目标)
    // 解释：这一行把向量模长写入 `const float Range`，用于表示距离、速度大小或不确定度。
    const float Range = RelPos.Size();
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (Range < KINDA_SMALL_NUMBER)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return FVector::ZeroVector;  // 已命中
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行声明成员或局部变量 `LOS`，用于保存视线方向单位向量。
    const FVector LOS = RelPos / Range;                    // 视线单位向量
    // 解释：这一行声明成员或局部变量 `RelVel`，用于保存相对速度向量。
    const FVector RelVel = TargetVel - InterceptorVel;     // 相对速度
    // 解释：这一行利用向量点乘结果更新 `const float ClosingSpeed`，提取某一方向上的投影量。
    const float ClosingSpeed = -FVector::DotProduct(RelVel, LOS);  // 接近速率（正值=接近中）

    // ---------- 视线角速率 ω_LOS ----------
    // ω_LOS = (LOS × v_rel) / |r|
    // 物理含义：视线在惯性空间中的旋转角速度
    // 解释：这一行利用向量叉乘结果更新 `FVector OmegaLOS`，构造垂直方向的几何量。
    FVector OmegaLOS = FVector::CrossProduct(LOS, RelVel) / Range;

    // ---------- 视线角速率的 tanh 平滑切换项 ----------
    // 使用 tanh 代替 sign 函数，消除高频抖振
    // 解释：这一行通过 `FMath::Max` 给 `const float Phi` 施加下界约束，避免 constfloatphi 过小。
    const float Phi = FMath::Max(BoundaryLayer, 0.001f);
    // 解释：这一行定义函数 `SatOmega`，开始实现平滑切换后的视线角速度项的具体逻辑。
    FVector SatOmega(
        // 解释：这一行继续补充函数 `SatOmega` 的参数列表、限定符或返回类型说明。
        FMath::Tanh(OmegaLOS.X / Phi),
        // 解释：这一行继续补充函数 `SatOmega` 的参数列表、限定符或返回类型说明。
        FMath::Tanh(OmegaLOS.Y / Phi),
        // 解释：这一行收束函数 `SatOmega` 的签名，后面会进入实现体或以分号结束声明。
        FMath::Tanh(OmegaLOS.Z / Phi)
    // 解释：这一行收束函数 `SatOmega` 的签名，后面会进入实现体或以分号结束声明。
    );

    // ---------- 垂直于视线的相对速度分量 ----------
    // v_rel_⊥ = v_rel - (v_rel · LOS) * LOS
    // 解释：这一行利用向量点乘结果更新 `const FVector RelVelNormal`，提取某一方向上的投影量。
    const FVector RelVelNormal = RelVel - FVector::DotProduct(RelVel, LOS) * LOS;

    // ---------- SMC 加速度指令 ----------
    // a_cmd = a_T + (|v_rel| / |r|) * [ N * v_rel_⊥ + K * tanh(ω/Φ) ]
    // 解释：这一行把向量模长写入 `const float RelSpeed`，用于表示距离、速度大小或不确定度。
    const float RelSpeed = RelVel.Size();
    // 解释：这一行把右侧表达式的结果写入 `const float Scale`，完成 constfloatscale 的更新。
    const float Scale = (Range > 0.1f) ? (RelSpeed / Range) : 0.0f;

    // 解释：这一行声明成员或局部变量 `AccCommand`，用于保存加速度指令。
    FVector AccCommand = TargetAcc
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        + Scale * (NavConstant * RelVelNormal + SwitchingGain * SatOmega);

    // ---------- 速度指令 = 当前速度 + 加速度 * dt ----------
    // 同时保持沿视线方向的接近速度
    // 解释：这一行声明成员或局部变量 `DesiredVel`，用于保存期望速度指令。
    FVector DesiredVel = InterceptorVel + AccCommand * DeltaTime;

    // 确保始终朝目标方向飞行：叠加一个沿视线的基础速度分量
    // 解释：这一行利用向量点乘结果更新 `const float AlongLOS`，提取某一方向上的投影量。
    const float AlongLOS = FVector::DotProduct(DesiredVel, LOS);
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (AlongLOS < MaxSpeed * 0.3f)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 保证至少 30% 的最大速度沿视线方向
        // 解释：这一行用欧拉积分把加速度指令转换成速度指令。
        DesiredVel += LOS * (MaxSpeed * 0.3f - AlongLOS);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 速度裁剪
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!DesiredVel.IsNearlyZero())
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行用欧拉积分把加速度指令转换成速度指令。
        DesiredVel = DesiredVel.GetClampedToMaxSize(MaxSpeed);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return DesiredVel;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 计算并更新内部状态（更新 LastLOS 用于后续帧）
 */
// 解释：这一行定义函数 `Update`，开始实现update的具体逻辑。
FVector FSlidingModeGuidance::Update(
    // 解释：这一行继续展开 `Update` 的参数列表，声明参数 `InterceptorPos` 用于传入拦截机位置。
    const FVector& InterceptorPos,
    // 解释：这一行继续展开 `Update` 的参数列表，声明参数 `InterceptorVel` 用于传入拦截机速度。
    const FVector& InterceptorVel,
    // 解释：这一行继续展开 `Update` 的参数列表，声明参数 `TargetPos` 用于传入目标位置。
    const FVector& TargetPos,
    // 解释：这一行继续展开 `Update` 的参数列表，声明参数 `TargetVel` 用于传入目标速度。
    const FVector& TargetVel,
    // 解释：这一行继续展开 `Update` 的参数列表，声明参数 `TargetAcc` 用于传入目标加速度。
    const FVector& TargetAcc,
    // 解释：这一行继续展开 `Update` 的参数列表，声明参数 `DeltaTime` 用于传入时间步长。
    float DeltaTime,
    // 解释：这一行收束函数 `Update` 的签名，后面会进入实现体或以分号结束声明。
    float MaxSpeed)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    FVector Result = ComputeInterceptVelocity(
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        InterceptorPos, InterceptorVel,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        TargetPos, TargetVel, TargetAcc,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        DeltaTime, MaxSpeed);

    // 更新 LOS 历史
    // 解释：这一行声明成员或局部变量 `RelPos`，用于保存相对位置向量。
    const FVector RelPos = TargetPos - InterceptorPos;
    // 解释：这一行把向量模长写入 `const float Range`，用于表示距离、速度大小或不确定度。
    const float Range = RelPos.Size();
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (Range > KINDA_SMALL_NUMBER)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `LastLOS`，完成 lastlos 的更新。
        LastLOS = RelPos / Range;
        // 解释：这一行把右侧表达式的结果写入 `bHasLastLOS`，完成 布尔标志 haslastlos 的更新。
        bHasLastLOS = true;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return Result;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

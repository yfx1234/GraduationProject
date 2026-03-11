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
FVector FSlidingModeGuidance::ComputeInterceptVelocity(
    const FVector& InterceptorPos,
    const FVector& InterceptorVel,
    const FVector& TargetPos,
    const FVector& TargetVel,
    const FVector& TargetAcc,
    float DeltaTime,
    float MaxSpeed) const
{
    // ---------- 相对运动学 ----------
    const FVector RelPos = TargetPos - InterceptorPos;     // 相对位置 (拦截器指向目标)
    const float Range = RelPos.Size();
    if (Range < KINDA_SMALL_NUMBER)
    {
        return FVector::ZeroVector;  // 已命中
    }

    const FVector LOS = RelPos / Range;                    // 视线单位向量
    const FVector RelVel = TargetVel - InterceptorVel;     // 相对速度
    const float ClosingSpeed = -FVector::DotProduct(RelVel, LOS);  // 接近速率（正值=接近中）

    // ---------- 视线角速率 ω_LOS ----------
    // ω_LOS = (LOS × v_rel) / |r|
    // 物理含义：视线在惯性空间中的旋转角速度
    FVector OmegaLOS = FVector::CrossProduct(LOS, RelVel) / Range;

    // ---------- 视线角速率的 tanh 平滑切换项 ----------
    // 使用 tanh 代替 sign 函数，消除高频抖振
    const float Phi = FMath::Max(BoundaryLayer, 0.001f);
    FVector SatOmega(
        FMath::Tanh(OmegaLOS.X / Phi),
        FMath::Tanh(OmegaLOS.Y / Phi),
        FMath::Tanh(OmegaLOS.Z / Phi)
    );

    // ---------- 垂直于视线的相对速度分量 ----------
    // v_rel_⊥ = v_rel - (v_rel · LOS) * LOS
    const FVector RelVelNormal = RelVel - FVector::DotProduct(RelVel, LOS) * LOS;

    // ---------- SMC 加速度指令 ----------
    // a_cmd = a_T + (|v_rel| / |r|) * [ N * v_rel_⊥ + K * tanh(ω/Φ) ]
    const float RelSpeed = RelVel.Size();
    const float Scale = (Range > 0.1f) ? (RelSpeed / Range) : 0.0f;

    FVector AccCommand = TargetAcc
        + Scale * (NavConstant * RelVelNormal + SwitchingGain * SatOmega);

    // ---------- 速度指令 = 当前速度 + 加速度 * dt ----------
    // 同时保持沿视线方向的接近速度
    FVector DesiredVel = InterceptorVel + AccCommand * DeltaTime;

    // 确保始终朝目标方向飞行：叠加一个沿视线的基础速度分量
    const float AlongLOS = FVector::DotProduct(DesiredVel, LOS);
    if (AlongLOS < MaxSpeed * 0.3f)
    {
        // 保证至少 30% 的最大速度沿视线方向
        DesiredVel += LOS * (MaxSpeed * 0.3f - AlongLOS);
    }

    // 速度裁剪
    if (!DesiredVel.IsNearlyZero())
    {
        DesiredVel = DesiredVel.GetClampedToMaxSize(MaxSpeed);
    }

    return DesiredVel;
}

/**
 * @brief 计算并更新内部状态（更新 LastLOS 用于后续帧）
 */
FVector FSlidingModeGuidance::Update(
    const FVector& InterceptorPos,
    const FVector& InterceptorVel,
    const FVector& TargetPos,
    const FVector& TargetVel,
    const FVector& TargetAcc,
    float DeltaTime,
    float MaxSpeed)
{
    FVector Result = ComputeInterceptVelocity(
        InterceptorPos, InterceptorVel,
        TargetPos, TargetVel, TargetAcc,
        DeltaTime, MaxSpeed);

    // 更新 LOS 历史
    const FVector RelPos = TargetPos - InterceptorPos;
    const float Range = RelPos.Size();
    if (Range > KINDA_SMALL_NUMBER)
    {
        LastLOS = RelPos / Range;
        bHasLastLOS = true;
    }

    return Result;
}

#include "GuidanceMethods.h"
#include "ITargetPredictor.h"

/**
 * @brief 从方向向量计算 Pitch/Yaw
 * @param AimDir 瞄准方向
 * @param AimPoint 瞄准点
 * @param FlightTime 预估飞行时间
 * @return FGuidanceOutput 瞄准输出
 */
static FGuidanceOutput DirToAngles(const FVector& AimDir, const FVector& AimPoint, float FlightTime)
{
    FGuidanceOutput Out;
    if (AimDir.IsNearlyZero())
    {
        Out.bValid = false;
        return Out;
    }

    const FRotator Rot = AimDir.GetSafeNormal().Rotation();
    Out.Pitch = Rot.Pitch;
    Out.Yaw = Rot.Yaw;
    Out.AimPoint = AimPoint;
    Out.EstFlightTime = FlightTime;
    Out.bValid = true;
    return Out;
}

/**
 * @brief 直接瞄准
 * @param Input 制导输入数据
 * @return FGuidanceOutput 瞄准输出
 */
FGuidanceOutput FDirectAiming::ComputeAim(const FGuidanceInput& Input)
{
    const FVector AimDir = Input.TargetPos - Input.MuzzlePos;
    const float Dist = AimDir.Size();
    const float FlightTime = (Input.MuzzleSpeed > 0.0f) ? (Dist / (Input.MuzzleSpeed * 100.0f)) : 0.0f;
    return DirToAngles(AimDir, Input.TargetPos, FlightTime);
}

/** @brief 比例导引构造函数 */
FProportionalNavigation::FProportionalNavigation(float NavConstant)
    : N(NavConstant)
    , LastLOS(FVector::ZeroVector)
    , bHasLastLOS(false)
{
}

/** @brief 重置内部状态 */
void FProportionalNavigation::Reset()
{
    LastLOS = FVector::ZeroVector;
    bHasLastLOS = false;
}

/**
 * @brief 比例导引
 * @param Input 制导输入数据
 * @return FGuidanceOutput 瞄准输出
 * AimDir = LOS + N * LOSRate * FlightTime
 */
FGuidanceOutput FProportionalNavigation::ComputeAim(const FGuidanceInput& Input)
{
    const FVector LOS = Input.TargetPos - Input.MuzzlePos;
    const float Dist = LOS.Size();
    const float FlightTime = (Input.MuzzleSpeed > 0.0f) ? (Dist / (Input.MuzzleSpeed * 100.0f)) : 0.0f;

    // 首帧还没有 LOS 变化率可用，直接退化为当前视线瞄准。
    if (!bHasLastLOS)
    {
        LastLOS = LOS;
        bHasLastLOS = true;
        return DirToAngles(LOS, Input.TargetPos, FlightTime);
    }

    FVector LOSRate = FVector::ZeroVector;
    if (Input.DeltaTime > KINDA_SMALL_NUMBER)
    {
        LOSRate = (LOS - LastLOS) / Input.DeltaTime;
    }

    const FVector AimDir = LOS + N * LOSRate * FlightTime;
    const FVector AimPoint = Input.MuzzlePos + AimDir;
    LastLOS = LOS;
    return DirToAngles(AimDir, AimPoint, FlightTime);
}

/** @brief 预测制导构造函数 */
FPredictiveGuidance::FPredictiveGuidance(ITargetPredictor* Predictor, int32 Iterations)
    : KalmanPredictor(Predictor)
    , MaxIterations(Iterations)
{
}

/**
 * @brief 预测制导
 * @param Input 制导输入数据
 * @return FGuidanceOutput 瞄准输出
 */
FGuidanceOutput FPredictiveGuidance::ComputeAim(const FGuidanceInput& Input)
{
    if (!KalmanPredictor || !KalmanPredictor->IsInitialized())
    {
        const FVector AimDir = Input.TargetPos - Input.MuzzlePos;
        const float Dist = AimDir.Size();
        const float FlightTime = (Input.MuzzleSpeed > 0.0f) ? (Dist / (Input.MuzzleSpeed * 100.0f)) : 0.0f;
        return DirToAngles(AimDir, Input.TargetPos, FlightTime);
    }

    const float MuzzleSpeedCm = Input.MuzzleSpeed * 100.0f;
    FVector PredPos = Input.TargetPos;
    float FlightTime = 0.0f;

    // 交替迭代“飞行时间 -> 预测目标位置”，逐步逼近拦截时刻的目标位置。
    for (int32 Iter = 0; Iter < MaxIterations; ++Iter)
    {
        const float Dist = FVector::Dist(Input.MuzzlePos, PredPos);
        FlightTime = (MuzzleSpeedCm > 0.0f) ? (Dist / MuzzleSpeedCm) : 0.0f;
        PredPos = KalmanPredictor->PredictPosition(FlightTime);
    }

    const FVector AimDir = PredPos - Input.MuzzlePos;
    return DirToAngles(AimDir, PredPos, FlightTime);
}
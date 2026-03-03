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
    FRotator Rot = AimDir.GetSafeNormal().Rotation();
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
    FVector AimDir = Input.TargetPos - Input.MuzzlePos;
    float Dist = AimDir.Size();
    float FlightTime = (Input.MuzzleSpeed > 0.0f) ? (Dist / (Input.MuzzleSpeed * 100.0f)) : 0.0f;
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
    FVector LOS = Input.TargetPos - Input.MuzzlePos;
    float Dist = LOS.Size();
    float FlightTime = (Input.MuzzleSpeed > 0.0f) ? (Dist / (Input.MuzzleSpeed * 100.0f)) : 0.0f;
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
    FVector AimDir = LOS + N * LOSRate * FlightTime;
    FVector AimPoint = Input.MuzzlePos + AimDir;
    LastLOS = LOS;
    return DirToAngles(AimDir, AimPoint, FlightTime);
}

/** @brief 卡尔曼预测制导构造函数 */
FPredictiveGuidance::FPredictiveGuidance(ITargetPredictor* Predictor, int32 Iterations)
    : KalmanPredictor(Predictor)
    , MaxIterations(Iterations)
{
}

/**
 * @brief 卡尔曼预测制导
 * @param Input 制导输入数据
 * @return FGuidanceOutput 瞄准输出
 */
FGuidanceOutput FPredictiveGuidance::ComputeAim(const FGuidanceInput& Input)
{
    if (!KalmanPredictor || !KalmanPredictor->IsInitialized())
    {
        FVector AimDir = Input.TargetPos - Input.MuzzlePos;
        float Dist = AimDir.Size();
        float FlightTime = (Input.MuzzleSpeed > 0.0f) ? (Dist / (Input.MuzzleSpeed * 100.0f)) : 0.0f;
        return DirToAngles(AimDir, Input.TargetPos, FlightTime);
    }
    float MuzzleSpeedCm = Input.MuzzleSpeed * 100.0f;
    FVector PredPos = Input.TargetPos;
    float FlightTime = 0.0f;
    for (int32 Iter = 0; Iter < MaxIterations; Iter++)
    {
        float Dist = FVector::Dist(Input.MuzzlePos, PredPos);
        FlightTime = (MuzzleSpeedCm > 0.0f) ? (Dist / MuzzleSpeedCm) : 0.0f;
        PredPos = KalmanPredictor->PredictPosition(FlightTime);
    }
    FVector AimDir = PredPos - Input.MuzzlePos;
    return DirToAngles(AimDir, PredPos, FlightTime);
}

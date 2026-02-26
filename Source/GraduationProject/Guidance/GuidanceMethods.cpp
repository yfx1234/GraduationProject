/**
 * @file GuidanceMethods.cpp
 * @brief 三种制导算法的实现文件
 *
 * 实现直接瞄准、比例导引和卡尔曼预测制导三种瞄准算法。
 * 辅助函数 DirToAngles() 将方向向量转换为 Pitch/Yaw 角度。
 */

#include "GuidanceMethods.h"
#include "ITargetPredictor.h"

// ========================================================================
// 辅助：从方向向量计算 Pitch/Yaw
// ========================================================================
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

// ========================================================================
// 方法一：直接瞄准
// ========================================================================
FGuidanceOutput FDirectAiming::ComputeAim(const FGuidanceInput& Input)
{
    FVector AimDir = Input.TargetPos - Input.MuzzlePos;
    float Dist = AimDir.Size();
    float FlightTime = (Input.MuzzleSpeed > 0.0f) ? (Dist / (Input.MuzzleSpeed * 100.0f)) : 0.0f;

    return DirToAngles(AimDir, Input.TargetPos, FlightTime);
}

// ========================================================================
// 方法二：比例导引
// ========================================================================
FProportionalNavigation::FProportionalNavigation(float NavConstant)
    : N(NavConstant)
    , LastLOS(FVector::ZeroVector)
    , bHasLastLOS(false)
{
}

void FProportionalNavigation::Reset()
{
    LastLOS = FVector::ZeroVector;
    bHasLastLOS = false;
}

FGuidanceOutput FProportionalNavigation::ComputeAim(const FGuidanceInput& Input)
{
    FVector LOS = Input.TargetPos - Input.MuzzlePos;
    float Dist = LOS.Size();
    float FlightTime = (Input.MuzzleSpeed > 0.0f) ? (Dist / (Input.MuzzleSpeed * 100.0f)) : 0.0f;

    if (!bHasLastLOS)
    {
        // 第一帧没有 LOS 变化率，退化为直接瞄准
        LastLOS = LOS;
        bHasLastLOS = true;
        return DirToAngles(LOS, Input.TargetPos, FlightTime);
    }

    // LOS 变化率
    FVector LOSRate = FVector::ZeroVector;
    if (Input.DeltaTime > KINDA_SMALL_NUMBER)
    {
        LOSRate = (LOS - LastLOS) / Input.DeltaTime;
    }

    // 比例导引: AimDir = LOS + N * LOSRate * FlightTime
    FVector AimDir = LOS + N * LOSRate * FlightTime;
    FVector AimPoint = Input.MuzzlePos + AimDir;

    LastLOS = LOS;

    return DirToAngles(AimDir, AimPoint, FlightTime);
}

// ========================================================================
// 方法三：卡尔曼预测制导（论文核心）
// ========================================================================
FPredictiveGuidance::FPredictiveGuidance(ITargetPredictor* Predictor, int32 Iterations)
    : KalmanPredictor(Predictor)
    , MaxIterations(Iterations)
{
}

FGuidanceOutput FPredictiveGuidance::ComputeAim(const FGuidanceInput& Input)
{
    if (!KalmanPredictor || !KalmanPredictor->IsInitialized())
    {
        // 卡尔曼未初始化，退化为直接瞄准
        FVector AimDir = Input.TargetPos - Input.MuzzlePos;
        float Dist = AimDir.Size();
        float FlightTime = (Input.MuzzleSpeed > 0.0f) ? (Dist / (Input.MuzzleSpeed * 100.0f)) : 0.0f;
        return DirToAngles(AimDir, Input.TargetPos, FlightTime);
    }

    float MuzzleSpeedCm = Input.MuzzleSpeed * 100.0f; // m/s → cm/s (UE 单位)
    FVector PredPos = Input.TargetPos;
    float FlightTime = 0.0f;

    // 迭代收敛：预测位置 ↔ 飞行时间
    for (int32 Iter = 0; Iter < MaxIterations; Iter++)
    {
        // 估算飞行时间
        float Dist = FVector::Dist(Input.MuzzlePos, PredPos);
        FlightTime = (MuzzleSpeedCm > 0.0f) ? (Dist / MuzzleSpeedCm) : 0.0f;

        // 用卡尔曼预测 FlightTime 后的目标位置
        // 注意：PredictPosition 使用的是滤波后的状态，单位与输入一致
        PredPos = KalmanPredictor->PredictPosition(FlightTime);
    }

    FVector AimDir = PredPos - Input.MuzzlePos;
    return DirToAngles(AimDir, PredPos, FlightTime);
}

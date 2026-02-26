/**
 * @file GuidanceMethods.h
 * @brief 三种制导算法实现的头文件
 *
 * 定义了三种制导方法类：
 * 1. FDirectAiming — 直接瞄准（无预测补偿）
 * 2. FProportionalNavigation — 比例导引（LOS变化率补偿）
 * 3. FPredictiveGuidance — 卡尔曼预测制导（论文核心，迭代收敛）
 */

#pragma once

#include "CoreMinimal.h"
#include "IGuidanceMethod.h"

class ITargetPredictor;

/**
 * 方法一：直接瞄准 (DirectAiming)
 * 直接瞄准目标当前位置，不做任何预测补偿
 */
class FDirectAiming : public IGuidanceMethod
{
public:
    virtual FString GetName() const override { return TEXT("direct"); }
    virtual FGuidanceOutput ComputeAim(const FGuidanceInput& Input) override;
};

/**
 * 方法二：比例导引 (Proportional Navigation)
 * AimDir = LOS + N * LOS_rate * FlightTime
 * N = 导航常数 (3~5)
 */
class FProportionalNavigation : public IGuidanceMethod
{
public:
    FProportionalNavigation(float NavConstant = 4.0f);

    virtual FString GetName() const override { return TEXT("proportional"); }
    virtual FGuidanceOutput ComputeAim(const FGuidanceInput& Input) override;
    virtual void Reset() override;

private:
    float N; // 导航常数
    FVector LastLOS;
    bool bHasLastLOS;
};

/**
 * 方法三：卡尔曼预测制导 (Predictive Guidance) — 论文核心
 * 1. 估算飞行时间 T_est = |Target - Muzzle| / MuzzleSpeed
 * 2. 从卡尔曼预测器获取 T_est 后的预测位置
 * 3. 用预测位置重新估算飞行时间 → 迭代收敛
 * 4. 瞄准最终预测位置
 */
class FPredictiveGuidance : public IGuidanceMethod
{
public:
    FPredictiveGuidance(ITargetPredictor* Predictor, int32 Iterations = 3);

    virtual FString GetName() const override { return TEXT("predictive"); }
    virtual FGuidanceOutput ComputeAim(const FGuidanceInput& Input) override;

    void SetPredictor(ITargetPredictor* Predictor) { KalmanPredictor = Predictor; }

private:
    ITargetPredictor* KalmanPredictor;
    int32 MaxIterations;
};

#pragma once

#include "CoreMinimal.h"
#include "IGuidanceMethod.h"

class ITargetPredictor;

/**
 * @brief 直接瞄准
 * 直接瞄准目标当前位置，不做任何预测补偿
 */
class FDirectAiming : public IGuidanceMethod
{
public:
    virtual FString GetName() const override { return TEXT("direct"); }
    virtual FGuidanceOutput ComputeAim(const FGuidanceInput& Input) override;
};

/**
 * @brief 比例导引
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
    float N; 
    FVector LastLOS;
    bool bHasLastLOS;
};

/** @brief 卡尔曼预测制导 */
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

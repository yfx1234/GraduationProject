#pragma once

#include "CoreMinimal.h"
#include "IGuidanceMethod.h"

class ITargetPredictor;

/**
 * @brief 直接瞄准算法
 * 始终将炮口/拦截器朝向目标当前位置，不做运动预测或视线变化补偿。
 */
class FDirectAiming : public IGuidanceMethod
{
public:
    virtual FString GetName() const override { return TEXT("direct"); }
    virtual FGuidanceOutput ComputeAim(const FGuidanceInput& Input) override;
};

/**
 * @brief 比例导引算法
 * 根据相邻两帧视线向量变化估计 LOS 角速度，并按导航常数放大补偿量。
 */
class FProportionalNavigation : public IGuidanceMethod
{
public:
    /**
     * @brief 构造比例导引算法
     * @param NavConstant 导航常数 N，常见取值约为 3~5
     */
    FProportionalNavigation(float NavConstant = 4.0f);

    virtual FString GetName() const override { return TEXT("proportional"); }
    virtual FGuidanceOutput ComputeAim(const FGuidanceInput& Input) override;
    virtual void Reset() override;

private:
    /** @brief 比例导引导航常数 */
    float N;

    /** @brief 上一帧视线向量，用于估算 LOSRate */
    FVector LastLOS;

    /** @brief 是否已经缓存过上一帧视线向量 */
    bool bHasLastLOS;
};

/**
 * @brief 预测制导算法
 * 借助目标预测器迭代估计拦截时刻的目标位置，再输出对应瞄准角。
 */
class FPredictiveGuidance : public IGuidanceMethod
{
public:
    /**
     * @brief 构造预测制导算法
     * @param Predictor 目标位置预测器
     * @param Iterations 飞行时间/目标位置的迭代次数
     */
    FPredictiveGuidance(ITargetPredictor* Predictor, int32 Iterations = 3);

    virtual FString GetName() const override { return TEXT("predictive"); }
    virtual FGuidanceOutput ComputeAim(const FGuidanceInput& Input) override;

    /**
     * @brief 替换内部目标预测器
     * @param Predictor 新的目标预测器实例
     */
    void SetPredictor(ITargetPredictor* Predictor) { KalmanPredictor = Predictor; }

private:
    /** @brief 目标预测器，不拥有其生命周期 */
    ITargetPredictor* KalmanPredictor;

    /** @brief 预测迭代次数，数值越大越接近收敛但开销也更高 */
    int32 MaxIterations;
};
#pragma once
#include "CoreMinimal.h"

/**
 * @brief 目标预测器接口
 * 输入观测位置序列，输出预测的未来位置和估计速度
 */
class ITargetPredictor
{
public:
    virtual ~ITargetPredictor() = default;

    /** @brief 喂入一帧观测数据 */
    virtual void Update(const FVector& ObservedPos, float DeltaTime) = 0;

    /** @brief 预测 dt 秒后的目标位置 */
    virtual FVector PredictPosition(float dt) const = 0;

    /** @brief 获取估计速度 */
    virtual FVector GetEstimatedVelocity() const = 0;

    /** @brief 获取估计位置（滤波后） */
    virtual FVector GetEstimatedPosition() const = 0;

    /** @brief 重置状态 */
    virtual void Reset() = 0;

    /** @brief 是否已初始化 */
    virtual bool IsInitialized() const = 0;
};

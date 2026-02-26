/**
 * @file ITargetPredictor.h
 * @brief 目标预测器接口的头文件
 *
 * 定义 ITargetPredictor 纯虚接口，为制导系统提供统一的
 * 目标状态估计和未来位置预测接口。
 * 具体实现类：UKalmanPredictor（卡尔曼滤波预测器）。
 */

#pragma once

#include "CoreMinimal.h"

/**
 * 目标预测器接口
 * 输入观测位置序列，输出预测的未来位置和估计速度
 */
class ITargetPredictor
{
public:
    virtual ~ITargetPredictor() = default;

    /** 喂入一帧观测数据 */
    virtual void Update(const FVector& ObservedPos, float DeltaTime) = 0;

    /** 预测 dt 秒后的目标位置 */
    virtual FVector PredictPosition(float dt) const = 0;

    /** 获取估计速度 */
    virtual FVector GetEstimatedVelocity() const = 0;

    /** 获取估计位置（滤波后） */
    virtual FVector GetEstimatedPosition() const = 0;

    /** 重置状态 */
    virtual void Reset() = 0;

    /** 是否已初始化（至少收到过一帧数据） */
    virtual bool IsInitialized() const = 0;
};

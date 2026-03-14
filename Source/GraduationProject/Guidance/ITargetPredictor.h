// 解释：使用 `#pragma once` 防止该头文件在编译过程中被重复包含。
#pragma once
// 解释：引入 Unreal 的核心基础头文件，提供常用容器、数学类型和日志宏。
#include "CoreMinimal.h"

/**
 * @brief 目标预测器接口
 * 输入观测位置序列，输出预测的未来位置和估计速度
 */
// 解释：这一行声明 类 `ITargetPredictor`，用于封装itarget预测器相关的数据与行为。
class ITargetPredictor
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
// 解释：从这一行开始进入 `public` 访问区，下面的成员会对外部模块公开。
public:
    // 解释：这一行把右侧表达式的结果写入 `virtual ~ITargetPredictor()`，完成 virtualitarget预测器 的更新。
    virtual ~ITargetPredictor() = default;

    /** @brief 喂入一帧观测数据 */
    // 解释：这一行把右侧表达式的结果写入 `virtual void Update(const FVector& ObservedPos, float DeltaTime)`，完成 virtualvoidupdateconstfvectorobservedposfloatdeltatime 的更新。
    virtual void Update(const FVector& ObservedPos, float DeltaTime) = 0;

    /** @brief 预测 dt 秒后的目标位置 */
    // 解释：这一行把右侧表达式的结果写入 `virtual FVector PredictPosition(float dt) const`，完成 virtualfvectorpredictpositionfloatdtconst 的更新。
    virtual FVector PredictPosition(float dt) const = 0;

    /** @brief 获取估计速度 */
    // 解释：这一行把右侧表达式的结果写入 `virtual FVector GetEstimatedVelocity() const`，完成 virtualfvectorgetestimatedvelocityconst 的更新。
    virtual FVector GetEstimatedVelocity() const = 0;

    /** @brief 获取估计位置（滤波后） */
    // 解释：这一行把右侧表达式的结果写入 `virtual FVector GetEstimatedPosition() const`，完成 virtualfvectorgetestimatedpositionconst 的更新。
    virtual FVector GetEstimatedPosition() const = 0;

    /** @brief 重置状态 */
    // 解释：这一行把右侧表达式的结果写入 `virtual void Reset()`，完成 virtualvoidreset 的更新。
    virtual void Reset() = 0;

    /** @brief 是否已初始化 */
    // 解释：这一行把右侧表达式的结果写入 `virtual bool IsInitialized() const`，完成 virtualboolisinitializedconst 的更新。
    virtual bool IsInitialized() const = 0;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
};

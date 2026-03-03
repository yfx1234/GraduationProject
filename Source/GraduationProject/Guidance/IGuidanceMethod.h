#pragma once

#include "CoreMinimal.h"

/**
 * @brief 制导输入数据
 * 包含制导算法计算瞄准方向所需的全部信息
 */
struct FGuidanceInput
{
    /** @brief 转台底座位置 */
    FVector TurretPos;
    /** @brief 枪口位置 */
    FVector MuzzlePos;
    /** @brief 目标当前位置 */
    FVector TargetPos;
    /** @brief 卡尔曼估计的目标速度 */
    FVector TargetVel;
    /** @brief 卡尔曼预测的未来位置 */
    FVector PredictedPos;
    /** @brief 弹丸初速 */
    float MuzzleSpeed;
    /** @brief 帧间隔 */
    float DeltaTime;
};

/**
 * @brief 制导输出数据
 * 制导算法的计算结果。
 */
struct FGuidanceOutput
{
    /** @brief 俯仰瞄准角 */
    float Pitch = 0.0f;
    /** @brief 偏航瞄准角 */
    float Yaw = 0.0f;
    /** @brief 瞄准点世界坐标 */
    FVector AimPoint;
    /** @brief 预估弹丸飞行时间 */
    float EstFlightTime = 0.0f;
    /** @brief 计算结果是否有效 */
    bool bValid = false;
};

/**
 * @brief 制导方法接口
 * 输入目标信息，输出瞄准角度。
 */
class IGuidanceMethod
{
public:
    virtual ~IGuidanceMethod() = default;

    /** @brief 获取制导方法名称 */
    virtual FString GetName() const = 0;

    /** @brief 计算瞄准角度 */
    virtual FGuidanceOutput ComputeAim(const FGuidanceInput& Input) = 0;

    /** @brief 重置内部状态 */
    virtual void Reset() {}
};

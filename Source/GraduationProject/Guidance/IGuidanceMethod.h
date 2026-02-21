#pragma once

#include "CoreMinimal.h"

/**
 * 制导输入数据
 */
struct FGuidanceInput
{
    FVector TurretPos;      // 转台位置
    FVector MuzzlePos;      // 枪口位置
    FVector TargetPos;      // 目标当前位置
    FVector TargetVel;      // 目标估计速度
    FVector PredictedPos;   // 卡尔曼预测位置
    float MuzzleSpeed;      // 弹丸初速 (m/s)
    float DeltaTime;        // 帧间隔
};

/**
 * 制导输出数据
 */
struct FGuidanceOutput
{
    float Pitch = 0.0f;     // 俯仰角 (度)
    float Yaw = 0.0f;       // 偏航角 (度)
    FVector AimPoint;       // 瞄准点
    float EstFlightTime = 0.0f; // 预估飞行时间
    bool bValid = false;    // 是否有效
};

/**
 * 制导方法接口
 * 输入目标信息 → 输出瞄准角度
 */
class IGuidanceMethod
{
public:
    virtual ~IGuidanceMethod() = default;

    /** 获取方法名称 */
    virtual FString GetName() const = 0;

    /** 计算瞄准角度 */
    virtual FGuidanceOutput ComputeAim(const FGuidanceInput& Input) = 0;

    /** 重置内部状态（如比例导引的上一帧 LOS） */
    virtual void Reset() {}
};

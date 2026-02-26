/**
 * @file IGuidanceMethod.h
 * @brief 制导方法接口和数据结构的头文件
 *
 * 定义制导系统的核心接口 IGuidanceMethod 和输入输出数据结构：
 * - FGuidanceInput — 制导计算所需的全部输入（转台位置、目标状态、弹丸初速等）
 * - FGuidanceOutput — 制导计算输出（瞄准角度、预估飞行时间等）
 * - IGuidanceMethod — 制导方法纯虚接口，所有具体制导算法继承此接口
 */

#pragma once

#include "CoreMinimal.h"

/**
 * 制导输入数据
 *
 * 包含制导算法计算瞄准方向所需的全部信息。
 */
struct FGuidanceInput
{
    /** @brief 转台底座位置 (UE 坐标, cm) */
    FVector TurretPos;
    /** @brief 枪口位置 (UE 坐标, cm) */
    FVector MuzzlePos;
    /** @brief 目标当前位置 (UE 坐标, cm) */
    FVector TargetPos;
    /** @brief 卡尔曼估计的目标速度 (UE 坐标, cm/s) */
    FVector TargetVel;
    /** @brief 卡尔曼预测的未来位置 (UE 坐标, cm) */
    FVector PredictedPos;
    /** @brief 弹丸初速 (m/s) */
    float MuzzleSpeed;
    /** @brief 帧间隔 (秒) */
    float DeltaTime;
};

/**
 * 制导输出数据
 *
 * 制导算法的计算结果。
 */
struct FGuidanceOutput
{
    /** @brief 俯仰瞄准角 (度) */
    float Pitch = 0.0f;
    /** @brief 偏航瞄准角 (度) */
    float Yaw = 0.0f;
    /** @brief 瞄准点世界坐标 (UE 坐标, cm) */
    FVector AimPoint;
    /** @brief 预估弹丸飞行时间 (秒) */
    float EstFlightTime = 0.0f;
    /** @brief 计算结果是否有效 */
    bool bValid = false;
};

/**
 * 制导方法接口（纯虚基类）
 *
 * 所有具体制导算法（直接瞄准、比例导引、卡尔曼预测）
 * 都实现此接口。输入目标信息，输出瞄准角度。
 */
class IGuidanceMethod
{
public:
    virtual ~IGuidanceMethod() = default;

    /**
     * @brief 获取制导方法名称
     * @return 方法名称字符串（如 "direct", "proportional", "predictive"）
     */
    virtual FString GetName() const = 0;

    /**
     * @brief 计算瞄准角度
     * @param Input 制导输入数据（转台位置、目标状态、弹速等）
     * @return 制导输出（瞄准角度、预估飞行时间等）
     */
    virtual FGuidanceOutput ComputeAim(const FGuidanceInput& Input) = 0;

    /**
     * @brief 重置内部状态
     *
     * 部分制导方法（如比例导引）需要保存上一帧的 LOS（视线方向），
     * 在重置或切换目标时需要清除历史数据。
     */
    virtual void Reset() {}
};

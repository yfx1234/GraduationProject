#pragma once

#include "CoreMinimal.h"

/**
 * @brief 滑模控制三维制导律 (3D Sliding Mode Control Guidance)
 *
 * 用于无人机对高机动目标的拦截制导。
 * 核心思想：定义与视线角速率(LOS rate)相关的滑模面 S，
 * 通过高增益切换项迫使 S → 0，实现对目标的精确拦截。
 *
 * 拦截器法向加速度指令方程：
 *   a_I = a_T + (|v_rel|/|r|) * ( N * v_rel_normal + K * tanh(ω_LOS / Φ) )
 *
 * 其中：
 *   N  = 比例导引常数（等效导引比）
 *   K  = 滑模切换增益（应对未知目标机动）
 *   Φ  = 边界层厚度（用 tanh 替代 sign 防止抖振）
 *   a_T = 目标加速度估计值（来自9维卡尔曼滤波器）
 */
struct FSlidingModeGuidance
{
    /** @brief 比例导引常数 (典型值: 3~5) */
    float NavConstant = 4.0f;

    /** @brief 滑模切换增益 (典型值: 5~20) */
    float SwitchingGain = 10.0f;

    /** @brief 边界层厚度 (典型值: 0.01~0.1 rad/s，越小越激进) */
    float BoundaryLayer = 0.05f;

    /** @brief 上一帧的视线方向，用于计算视线角速率 */
    FVector LastLOS = FVector::ZeroVector;

    /** @brief 是否已有上一帧 LOS 数据 */
    bool bHasLastLOS = false;

    /** @brief 重置内部状态 */
    void Reset()
    {
        LastLOS = FVector::ZeroVector;
        bHasLastLOS = false;
    }

    /**
     * @brief 计算 SMC 制导的拦截器速度指令
     *
     * @param InterceptorPos   拦截器当前位置
     * @param InterceptorVel   拦截器当前速度
     * @param TargetPos        目标当前位置
     * @param TargetVel        目标当前速度
     * @param TargetAcc        目标当前加速度估计（来自9维卡尔曼滤波器）
     * @param DeltaTime        帧间隔 (秒)
     * @param MaxSpeed         拦截器最大速度 (m/s)
     * @return                 拦截器速度指令向量
     */
    FVector ComputeInterceptVelocity(
        const FVector& InterceptorPos,
        const FVector& InterceptorVel,
        const FVector& TargetPos,
        const FVector& TargetVel,
        const FVector& TargetAcc,
        float DeltaTime,
        float MaxSpeed) const;

    /**
     * @brief 计算并更新内部状态的版本（会更新 LastLOS）
     */
    FVector Update(
        const FVector& InterceptorPos,
        const FVector& InterceptorVel,
        const FVector& TargetPos,
        const FVector& TargetVel,
        const FVector& TargetAcc,
        float DeltaTime,
        float MaxSpeed);
};

// 解释：使用 `#pragma once` 防止该头文件在编译过程中被重复包含。
#pragma once

// 解释：引入 Unreal 的核心基础头文件，提供常用容器、数学类型和日志宏。
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
// 解释：这一行声明 结构体 `FSlidingModeGuidance`，用于封装fsliding模式制导相关的数据与行为。
struct FSlidingModeGuidance
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    /** @brief 比例导引常数 (典型值: 3~5) */
    // 解释：这一行声明成员或局部变量 `NavConstant`，用于保存navconstant。
    float NavConstant = 4.0f;

    /** @brief 滑模切换增益 (典型值: 5~20) */
    // 解释：这一行声明成员或局部变量 `SwitchingGain`，用于保存switchinggain。
    float SwitchingGain = 10.0f;

    /** @brief 边界层厚度 (典型值: 0.01~0.1 rad/s，越小越激进) */
    // 解释：这一行声明成员或局部变量 `BoundaryLayer`，用于保存boundarylayer。
    float BoundaryLayer = 0.05f;

    /** @brief 上一帧的视线方向，用于计算视线角速率 */
    // 解释：这一行声明成员或局部变量 `LastLOS`，用于保存lastlos。
    FVector LastLOS = FVector::ZeroVector;

    /** @brief 是否已有上一帧 LOS 数据 */
    // 解释：这一行声明成员或局部变量 `bHasLastLOS`，用于保存布尔标志 haslastlos。
    bool bHasLastLOS = false;

    /** @brief 重置内部状态 */
    // 解释：这一行定义函数 `Reset`，开始实现reset的具体逻辑。
    void Reset()
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `LastLOS`，完成 lastlos 的更新。
        LastLOS = FVector::ZeroVector;
        // 解释：这一行把右侧表达式的结果写入 `bHasLastLOS`，完成 布尔标志 haslastlos 的更新。
        bHasLastLOS = false;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
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
    // 解释：这一行定义函数 `ComputeInterceptVelocity`，开始实现compute拦截velocity的具体逻辑。
    FVector ComputeInterceptVelocity(
        // 解释：这一行继续展开 `ComputeInterceptVelocity` 的参数列表，声明参数 `InterceptorPos` 用于传入拦截机位置。
        const FVector& InterceptorPos,
        // 解释：这一行继续展开 `ComputeInterceptVelocity` 的参数列表，声明参数 `InterceptorVel` 用于传入拦截机速度。
        const FVector& InterceptorVel,
        // 解释：这一行继续展开 `ComputeInterceptVelocity` 的参数列表，声明参数 `TargetPos` 用于传入目标位置。
        const FVector& TargetPos,
        // 解释：这一行继续展开 `ComputeInterceptVelocity` 的参数列表，声明参数 `TargetVel` 用于传入目标速度。
        const FVector& TargetVel,
        // 解释：这一行继续展开 `ComputeInterceptVelocity` 的参数列表，声明参数 `TargetAcc` 用于传入目标加速度。
        const FVector& TargetAcc,
        // 解释：这一行继续展开 `ComputeInterceptVelocity` 的参数列表，声明参数 `DeltaTime` 用于传入时间步长。
        float DeltaTime,
        // 解释：这一行收束函数 `ComputeInterceptVelocity` 的签名，后面会进入实现体或以分号结束声明。
        float MaxSpeed) const;

    /**
     * @brief 计算并更新内部状态的版本（会更新 LastLOS）
     */
    // 解释：这一行定义函数 `Update`，开始实现update的具体逻辑。
    FVector Update(
        // 解释：这一行继续展开 `Update` 的参数列表，声明参数 `InterceptorPos` 用于传入拦截机位置。
        const FVector& InterceptorPos,
        // 解释：这一行继续展开 `Update` 的参数列表，声明参数 `InterceptorVel` 用于传入拦截机速度。
        const FVector& InterceptorVel,
        // 解释：这一行继续展开 `Update` 的参数列表，声明参数 `TargetPos` 用于传入目标位置。
        const FVector& TargetPos,
        // 解释：这一行继续展开 `Update` 的参数列表，声明参数 `TargetVel` 用于传入目标速度。
        const FVector& TargetVel,
        // 解释：这一行继续展开 `Update` 的参数列表，声明参数 `TargetAcc` 用于传入目标加速度。
        const FVector& TargetAcc,
        // 解释：这一行继续展开 `Update` 的参数列表，声明参数 `DeltaTime` 用于传入时间步长。
        float DeltaTime,
        // 解释：这一行继续补充函数 `Update` 的参数列表、限定符或返回类型说明。
        float MaxSpeed);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
};

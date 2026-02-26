/**
 * @file PIDController.h
 * @brief PID（比例-积分-微分）控制器的头文件
 *
 * 本文件定义了 UPIDController 类，实现了带积分项和微分滤波的 PID 控制器。
 * 用于无人机运动仿真中的速度控制环和角速率控制环。
 * 积分项使用梯形积分法，微分项使用一阶低通滤波器平滑。
 */

#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"

#include "PIDController.generated.h"

/**
 * PID（比例-积分-微分）控制器
 *
 * 控制律: u = Kp * e + Ki * ∫e·dt + Kd * de/dt
 *
 * 积分项使用梯形积分法（Trapezoidal rule）：
 *   I += (T/2) * (e + e_prev)
 *   积分器限幅范围 [IntegratorMin, IntegratorMax]，防止积分饱和（Windup）
 *
 * 微分项使用双线性变换（Tustin法）的一阶低通滤波器（同 PDController）。
 * 支持输出限幅（OutputLimit > 0 时生效）。
 */
UCLASS(Blueprintable, BlueprintType)
class GRADUATIONPROJECT_API UPIDController : public UObject
{
    GENERATED_BODY()

public:
    /** @brief 默认构造函数，初始化所有参数为默认值 */
    UPIDController();

    /**
     * @brief 初始化控制器参数
     * @param InputKp 比例增益 Kp
     * @param InputKi 积分增益 Ki
     * @param InputKd 微分增益 Kd
     * @param InputDiffFilterTau 微分滤波器时间常数 τ（秒），范围 [0.001, 1.0]
     * @param InputOutputLimit 输出限幅值（0 表示不限幅）
     * @param InputTimeStep 采样时间步长（秒）
     * @param InputIntegratorMin 积分器下限（防止负向饱和）
     * @param InputIntegratorMax 积分器上限（防止正向饱和）
     */
    UFUNCTION(BlueprintCallable, Category = "PID Controller")
    void Initialize(double InputKp = 1.0, double InputKi = 0.1, double InputKd = 0.05, 
                   double InputDiffFilterTau = 0.05, double InputOutputLimit = 0.0, double InputTimeStep = 0.02,
                   double InputIntegratorMin = -100.0, double InputIntegratorMax = 100.0);

    /**
     * @brief 执行一步 PID 控制更新
     * @param TargetValue 目标值（期望值）
     * @param CurrentValue 当前测量值
     * @param bReset 是否在本次计算前重置控制器内部状态
     * @return 控制输出 u = Kp*e + Ki*∫e + Kd*(de/dt_filtered)
     */
    UFUNCTION(BlueprintCallable, Category = "PID Controller")
    double Update(double TargetValue, double CurrentValue, bool bReset = false);

    /**
     * @brief 运行时动态修改 PID 参数
     * @param NewKp 新的比例增益
     * @param NewKi 新的积分增益
     * @param NewKd 新的微分增益
     * @param NewDiffFilterTau 新的微分滤波时间常数 τ（秒）
     * @param NewOutputLimit 新的输出限幅值（0 表示不限幅）
     */
    UFUNCTION(BlueprintCallable, Category = "PID Controller")
    void SetParameters(double NewKp, double NewKi, double NewKd, double NewDiffFilterTau = 0.05, double NewOutputLimit = 0.0);

    /**
     * @brief 设置采样时间步长，并重新计算微分滤波器系数
     * @param NewTimeStep 新的时间步长（秒），下限 0.001
     */
    UFUNCTION(BlueprintCallable, Category = "PID Controller")
    void SetTimeStep(double NewTimeStep);

    /**
     * @brief 设置积分器限幅范围，防止积分饱和
     * @param NewMin 积分器下限
     * @param NewMax 积分器上限
     */
    UFUNCTION(BlueprintCallable, Category = "PID Controller")
    void SetIntegratorLimits(double NewMin, double NewMax);

    /**
     * @brief 重置控制器内部状态（积分器、误差历史、滤波状态）
     */
    UFUNCTION(BlueprintCallable, Category = "PID Controller")
    void Reset();

public:
    /** @brief 比例增益 Kp */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID Controller|Parameters")
    double Kp;

    /** @brief 积分增益 Ki */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID Controller|Parameters")
    double Ki;

    /** @brief 微分增益 Kd */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID Controller|Parameters")
    double Kd;

    /** @brief 微分滤波器时间常数 τ（秒），值越大滤波越强，范围 [0.001, 1.0] */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID Controller|Parameters", meta = (ClampMin = "0.001", ClampMax = "1.0"))
    double DiffFilterTau;

    /** @brief 输出限幅值，为 0 表示不限幅；非零时输出范围为 [-OutputLimit, OutputLimit] */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID Controller|Parameters", meta = (ClampMin = "0.0"))
    double OutputLimit;

    /** @brief 积分器下限，防止负向积分饱和 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID Controller|Integrator")
    double IntegratorMin;

    /** @brief 积分器上限，防止正向积分饱和 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID Controller|Integrator")
    double IntegratorMax;

    /** @brief 当前采样时间步长（秒） */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "PID Controller|State")
    double TimeStep;

    /** @brief 控制器是否已被初始化 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "PID Controller|State")
    bool bInitialized;

private:
    /** @brief 积分器累积值 ∫e·dt */
    double Integrator;

    /** @brief 微分滤波器系数 α = 2 / (2τ + T) */
    double DiffFilterAlpha;

    /** @brief 微分滤波器系数 β = (2τ - T) / (2τ + T) */
    double DiffFilterBeta;

    /** @brief 上一步的误差值 e(k-1) */
    double PreviousError;

    /** @brief 上一步的滤波后误差变化率 de(k-1) */
    double PreviousErrorRate;
};

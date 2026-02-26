/**
 * @file PDController.h
 * @brief PD（比例-微分）控制器的头文件
 *
 * 本文件定义了 UPDController 类，实现了带微分滤波的 PD 控制器。
 * 用于无人机运动仿真中的位置控制环和姿态控制环。
 * 微分项使用一阶低通滤波器平滑，避免高频噪声放大。
 */

#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"

#include "PDController.generated.h"

/**
 * PD（比例-微分）控制器
 *
 * 控制律: u = Kp * e + Kd * de/dt
 *
 * 微分项使用双线性变换（Tustin法）的一阶低通滤波器：
 *   de_filtered = α * (e - e_prev) + β * de_prev
 *   其中 α = 2/(2τ+T), β = (2τ-T)/(2τ+T)
 *   τ = DiffFilterTau（滤波时间常数），T = TimeStep（采样周期）
 *
 * 支持输出限幅（OutputLimit > 0 时生效）。
 */
UCLASS(Blueprintable, BlueprintType)
class GRADUATIONPROJECT_API UPDController : public UObject
{
    GENERATED_BODY()

public:
    /** @brief 默认构造函数，初始化所有参数为默认值 */
    UPDController();

    /**
     * @brief 初始化控制器参数
     * @param InputKp 比例增益 Kp
     * @param InputKd 微分增益 Kd
     * @param InputDiffFilterTau 微分滤波器时间常数 τ（秒），范围 [0.001, 1.0]
     * @param InputOutputLimit 输出限幅值（0 表示不限幅）
     * @param InputTimeStep 采样时间步长（秒）
     */
    UFUNCTION(BlueprintCallable, Category = "PD Controller")
    void Initialize(double InputKp = 1.0, double InputKd = 0.1, 
        double InputDiffFilterTau = 0.05, double InputOutputLimit = 0.0, double InputTimeStep = 0.02);

    /**
     * @brief 执行一步 PD 控制更新
     * @param TargetValue 目标值（期望值）
     * @param CurrentValue 当前测量值
     * @param bReset 是否在本次计算前重置控制器内部状态
     * @return 控制输出 u = Kp * error + Kd * filtered_error_rate
     */
    UFUNCTION(BlueprintCallable, Category = "PD Controller")
    double Update(double TargetValue, double CurrentValue, bool bReset = false);

    /**
     * @brief 运行时动态修改 PD 参数
     * @param NewKp 新的比例增益
     * @param NewKd 新的微分增益
     * @param NewDiffFilterTau 新的微分滤波时间常数 τ（秒）
     * @param NewOutputLimit 新的输出限幅值（0 表示不限幅）
     */
    UFUNCTION(BlueprintCallable, Category = "PD Controller")
    void SetParameters(double NewKp, double NewKd, double NewDiffFilterTau = 0.05, double NewOutputLimit = 0.0);

    /**
     * @brief 设置采样时间步长，并重新计算微分滤波器系数
     * @param NewTimeStep 新的时间步长（秒），下限 0.001
     */
    UFUNCTION(BlueprintCallable, Category = "PD Controller")
    void SetTimeStep(double NewTimeStep);

    /**
     * @brief 重置控制器内部状态（误差历史和滤波状态）
     */
    UFUNCTION(BlueprintCallable, Category = "PD Controller")
    void Reset();

public:
    /** @brief 比例增益 Kp */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PD Controller|Parameters")
    double Kp;

    /** @brief 微分增益 Kd */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PD Controller|Parameters")
    double Kd;

    /** @brief 微分滤波器时间常数 τ（秒），用于平滑微分项，值越大滤波越强 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PD Controller|Parameters")
    double DiffFilterTau;

    /** @brief 输出限幅值，为 0 表示不限幅；非零时输出范围为 [-OutputLimit, OutputLimit] */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PD Controller|Parameters")
    double OutputLimit;

    /** @brief 当前采样时间步长（秒） */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "PD Controller|State")
    double TimeStep;

    /** @brief 控制器是否已被初始化 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "PD Controller|State")
    bool bInitialized;

private:
    /** @brief 微分滤波器系数 α = 2 / (2τ + T) */
    double DiffFilterAlpha;

    /** @brief 微分滤波器系数 β = (2τ - T) / (2τ + T) */
    double DiffFilterBeta;

    /** @brief 上一步的误差值 e(k-1) */
    double PreviousError;

    /** @brief 上一步的滤波后误差变化率 de(k-1) */
    double PreviousErrorRate;
};

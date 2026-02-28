#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"

#include "PIDController.generated.h"

/**
 *  e = target - current
 *  I += (T/2) * (e + e_prev)
 *  α = 2 / (2τ + T), β = (2τ - T) / (2τ + T)
 *  de = α * (e[k] - e[k-1]) + β * de[k-1]
 *  u = Kp*e + Ki*I + Kd*de
 */
UCLASS(Blueprintable, BlueprintType)
class GRADUATIONPROJECT_API UPIDController : public UObject
{
    GENERATED_BODY()

public:
    UPIDController();

    /**
     * 初始化控制器参数
     * @param InputKp 比例增益 Kp
     * @param InputKi 积分增益 Ki
     * @param InputKd 微分增益 Kd
     * @param InputDiffFilterTau 微分滤波器时间常数 τ（秒）
     * @param InputOutputLimit 输出限幅值
     * @param InputTimeStep 采样时间步长（秒）
     * @param InputIntegratorMin 积分器下限
     * @param InputIntegratorMax 积分器上限
     */
    UFUNCTION(BlueprintCallable, Category = "PID Controller")
    void Initialize(double InputKp = 1.0, double InputKi = 0.1, double InputKd = 0.05, 
                   double InputDiffFilterTau = 0.05, double InputOutputLimit = 0.0, double InputTimeStep = 0.02,
                   double InputIntegratorMin = -100.0, double InputIntegratorMax = 100.0);

    /**
     * 执行一步 PID 控制更新
     * @param TargetValue 目标值
     * @param CurrentValue 当前测量值
     * @param bReset 是否在本次计算前重置控制器内部状态
     * @return 控制输出 u = Kp*e + Ki*∫e + Kd*(de/dt)
     */
    UFUNCTION(BlueprintCallable, Category = "PID Controller")
    double Update(double TargetValue, double CurrentValue, bool bReset = false);

    /**
     * 运行时动态修改 PID 参数
     * @param NewKp 新的比例增益
     * @param NewKi 新的积分增益
     * @param NewKd 新的微分增益
     * @param NewDiffFilterTau 新的微分滤波时间常数 τ（秒）
     * @param NewOutputLimit 新的输出限幅值
     */
    UFUNCTION(BlueprintCallable, Category = "PID Controller")
    void SetParameters(double NewKp, double NewKi, double NewKd, double NewDiffFilterTau = 0.05, double NewOutputLimit = 0.0);

    /**
     * 设置采样时间步长，并重新计算微分滤波器系数
     * @param NewTimeStep 新的时间步长（秒）
     */
    UFUNCTION(BlueprintCallable, Category = "PID Controller")
    void SetTimeStep(double NewTimeStep);

    /**
     * 设置积分器限幅范围，防止积分饱和
     * @param NewMin 积分器下限
     * @param NewMax 积分器上限
     */
    UFUNCTION(BlueprintCallable, Category = "PID Controller")
    void SetIntegratorLimits(double NewMin, double NewMax);

    /**
     * 重置控制器内部状态
     */
    UFUNCTION(BlueprintCallable, Category = "PID Controller")
    void Reset();

public:
    /** 比例增益 Kp */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID Controller|Parameters")
    double Kp;

    /** 积分增益 Ki */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID Controller|Parameters")
    double Ki;

    /** 微分增益 Kd */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID Controller|Parameters")
    double Kd;

    /** 微分滤波器时间常数 τ（秒），值越大滤波越强，范围 [0.001, 1.0] */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID Controller|Parameters", meta = (ClampMin = "0.001", ClampMax = "1.0"))
    double DiffFilterTau;

    /** 输出限幅值 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID Controller|Parameters", meta = (ClampMin = "0.0"))
    double OutputLimit;

    /** 积分器下限，防止负向积分饱和 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID Controller|Integrator")
    double IntegratorMin;

    /** 积分器上限，防止正向积分饱和 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID Controller|Integrator")
    double IntegratorMax;

    /** 当前采样时间步长（秒） */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "PID Controller|State")
    double TimeStep;

    /** 控制器是否已被初始化 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "PID Controller|State")
    bool bInitialized;

private:
    /** 积分器累积值 ∫e·dt */
    double Integrator;

    /** 微分滤波器系数 α = 2 / (2τ + T) */
    double DiffFilterAlpha;

    /** 微分滤波器系数 β = (2τ - T) / (2τ + T) */
    double DiffFilterBeta;

    /** 上一步的误差值 e(k-1) */
    double PreviousError;

    /** 上一步的滤波后误差变化率 de(k-1) */
    double PreviousErrorRate;
};

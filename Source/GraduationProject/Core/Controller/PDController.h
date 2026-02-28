#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"

#include "PDController.generated.h"

/**
 * e = target - current
 * α = 2 / (2τ + T), β = (2τ - T) / (2τ + T)
 * de = α * (e[k] - e[k-1]) + β * de[k-1]
 * u = Kp*e + Kd*de
 */
UCLASS(Blueprintable, BlueprintType)
class GRADUATIONPROJECT_API UPDController : public UObject
{
    GENERATED_BODY()

public:
    UPDController();

    /**
     * 初始化 PD 控制器
     * @param InputKp 比例增益 Kp
     * @param InputKd 微分增益 Kd
     * @param InputDiffFilterTau 微分滤波器时间常数 τ（秒）
     * @param InputOutputLimit 输出限幅值
     * @param InputTimeStep 采样时间步长（秒）
     */
    UFUNCTION(BlueprintCallable, Category = "PD Controller")
    void Initialize(double InputKp = 1.0, double InputKd = 0.1, 
        double InputDiffFilterTau = 0.05, double InputOutputLimit = 0.0, double InputTimeStep = 0.02);

    /**
     * PD 控制更新
     * @param TargetValue 目标值
     * @param CurrentValue 当前测量值
     * @param bReset 是否在本次计算前重置控制器内部状态
     * @return 控制输出 u = Kp*e + Kd*(de/dt)
     */
    UFUNCTION(BlueprintCallable, Category = "PD Controller")
    double Update(double TargetValue, double CurrentValue, bool bReset = false);

    /**
     * 运行时动态修改 PD 参数
     * @param NewKp 新的比例增益
     * @param NewKd 新的微分增益
     * @param NewDiffFilterTau 新的微分滤波时间常数 τ（秒）
     * @param NewOutputLimit 新的输出限幅值
     */
    UFUNCTION(BlueprintCallable, Category = "PD Controller")
    void SetParameters(double NewKp, double NewKd, double NewDiffFilterTau = 0.05, double NewOutputLimit = 0.0);

    /**
     * 设置采样时间步长，并重新计算微分滤波器系数
     * @param NewTimeStep 新的时间步长（秒）
     */
    UFUNCTION(BlueprintCallable, Category = "PD Controller")
    void SetTimeStep(double NewTimeStep);

    /**
     * 重置控制器内部状态
     */
    UFUNCTION(BlueprintCallable, Category = "PD Controller")
    void Reset();

public:
    /** 比例增益 Kp */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PD Controller|Parameters")
    double Kp;

    /** 微分增益 Kd */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PD Controller|Parameters")
    double Kd;

    /** 微分滤波器时间常数 τ（秒），用于平滑微分项，值越大滤波越强 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PD Controller|Parameters")
    double DiffFilterTau;

    /** 输出限幅值；非零时输出范围为 [-OutputLimit, OutputLimit] */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PD Controller|Parameters")
    double OutputLimit;

    /** 当前采样时间步长（秒） */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "PD Controller|State")
    double TimeStep;

    /** 控制器是否已被初始化 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "PD Controller|State")
    bool bInitialized;

private:
    /** 微分滤波器系数 α = 2 / (2τ + T) */
    double DiffFilterAlpha;

    /** 微分滤波器系数 β = (2τ - T) / (2τ + T) */
    double DiffFilterBeta;

    /** 上一步的误差值 e(k-1) */
    double PreviousError;

    /** 上一步的滤波后误差变化率 de(k-1) */
    double PreviousErrorRate;
};

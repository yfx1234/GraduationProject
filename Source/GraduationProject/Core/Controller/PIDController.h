#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"

#include "PIDController.generated.h"

UCLASS(Blueprintable, BlueprintType)
class GRADUATIONPROJECT_API UPIDController : public UObject
{
    GENERATED_BODY()

public:
    UPIDController();

    UFUNCTION(BlueprintCallable, Category = "PID Controller")
    void Initialize(double InputKp = 1.0, double InputKi = 0.1, double InputKd = 0.05, 
                   double InputDiffFilterTau = 0.05, double InputOutputLimit = 0.0, double InputTimeStep = 0.02,
                   double InputIntegratorMin = -100.0, double InputIntegratorMax = 100.0);

    /**
     * 更新控制器，计算一步控制输出
     * @param TargetValue   目标值
     * @param CurrentValue  当前值
     * @param bReset        是否在本次更新前重置控制器状态
     * @return 控制输出
     */
    UFUNCTION(BlueprintCallable, Category = "PID Controller")
    double Update(double TargetValue, double CurrentValue, bool bReset = false);

    // 运行时修改 PID 参数
    UFUNCTION(BlueprintCallable, Category = "PID Controller")
    void SetParameters(double NewKp, double NewKi, double NewKd, double NewDiffFilterTau = 0.05, double NewOutputLimit = 0.0);

    // 设置采样时间步长
    UFUNCTION(BlueprintCallable, Category = "PID Controller")
    void SetTimeStep(double NewTimeStep);

    // 设置积分器限幅范围 
    UFUNCTION(BlueprintCallable, Category = "PID Controller")
    void SetIntegratorLimits(double NewMin, double NewMax);

    // 重置控制器内部状态
    UFUNCTION(BlueprintCallable, Category = "PID Controller")
    void Reset();

public:
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID Controller|Parameters")
    double Kp;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID Controller|Parameters")
    double Ki;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID Controller|Parameters")
    double Kd;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID Controller|Parameters", meta = (ClampMin = "0.001", ClampMax = "1.0"))
    double DiffFilterTau;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID Controller|Parameters", meta = (ClampMin = "0.0"))
    double OutputLimit;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID Controller|Integrator")
    double IntegratorMin;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID Controller|Integrator")
    double IntegratorMax;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "PID Controller|State")
    double TimeStep;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "PID Controller|State")
    bool bInitialized;

private:
    double Integrator;
    double DiffFilterAlpha;
    double DiffFilterBeta;
    double PreviousError;
    double PreviousErrorRate;
};

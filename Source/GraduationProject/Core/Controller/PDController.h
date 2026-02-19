#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"

#include "PDController.generated.h"

UCLASS(Blueprintable, BlueprintType)
class GRADUATIONPROJECT_API UPDController : public UObject
{
    GENERATED_BODY()

public:
    UPDController();

    // 初始化控制器
    UFUNCTION(BlueprintCallable, Category = "PD Controller")
    void Initialize(double InputKp = 1.0, double InputKd = 0.1, 
        double InputDiffFilterTau = 0.05, double InputOutputLimit = 0.0, double InputTimeStep = 0.02);

    /**
     * 更新控制器，计算一步控制输出
     * @param TargetValue   目标值
     * @param CurrentValue  当前值
     * @param bReset        是否在本次更新前重置控制器状态
     * @return 控制输出
     */
    UFUNCTION(BlueprintCallable, Category = "PD Controller")
    double Update(double TargetValue, double CurrentValue, bool bReset = false);

    // 运行时修改 PD 参数
    UFUNCTION(BlueprintCallable, Category = "PD Controller")
    void SetParameters(double NewKp, double NewKd, double NewDiffFilterTau = 0.05, double NewOutputLimit = 0.0);

    // 设置采样时间步长
    UFUNCTION(BlueprintCallable, Category = "PD Controller")
    void SetTimeStep(double NewTimeStep);

    // 重置控制器内部状态
    UFUNCTION(BlueprintCallable, Category = "PD Controller")
    void Reset();

public:
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PD Controller|Parameters")
    double Kp;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PD Controller|Parameters")
    double Kd;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PD Controller|Parameters")
    double DiffFilterTau;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PD Controller|Parameters")
    double OutputLimit;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "PD Controller|State")
    double TimeStep;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "PD Controller|State")
    bool bInitialized;

private:
    double DiffFilterAlpha;
    double DiffFilterBeta;
    double PreviousError;
    double PreviousErrorRate;
};

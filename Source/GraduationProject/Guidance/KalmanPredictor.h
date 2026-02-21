#pragma once

#include "CoreMinimal.h"
#include "ITargetPredictor.h"
#include "KalmanPredictor.generated.h"

/**
 * 卡尔曼滤波目标预测器
 *
 * 状态向量 x = [px, py, pz, vx, vy, vz] (6维)
 * 观测向量 z = [px, py, pz]             (3维)
 * 运动模型：匀速模型 (Constant Velocity)
 *
 * 状态转移:  x_k = F * x_{k-1}
 *   F = | I  dt*I |
 *       | 0    I  |
 *
 * 观测矩阵:  z = H * x
 *   H = | I  0 |
 */
UCLASS(BlueprintType)
class GRADUATIONPROJECT_API UKalmanPredictor : public UObject, public ITargetPredictor
{
    GENERATED_BODY()

public:
    UKalmanPredictor();

    /** 初始化滤波器参数 */
    void Initialize(float ProcessNoise = 1.0f, float MeasurementNoise = 0.5f);

    // ---- ITargetPredictor 接口 ----
    virtual void Update(const FVector& ObservedPos, float DeltaTime) override;
    virtual FVector PredictPosition(float dt) const override;
    virtual FVector GetEstimatedVelocity() const override;
    virtual FVector GetEstimatedPosition() const override;
    virtual void Reset() override;
    virtual bool IsInitialized() const override { return bInitialized; }

    /** 获取预测误差协方差（位置部分的迹，用于评估置信度） */
    float GetPositionUncertainty() const;

private:
    // 6维状态向量
    double X[6];

    // 6×6 协方差矩阵
    double P[6][6];

    // 过程噪声强度
    float QScale;

    // 测量噪声强度
    float RScale;

    bool bInitialized;
    int32 UpdateCount;

    // ---- 矩阵运算辅助 ----
    static void MatMul6(const double A[6][6], const double B[6][6], double C[6][6]);
    static void MatTranspose6(const double A[6][6], double AT[6][6]);
    static void MatAdd6(const double A[6][6], const double B[6][6], double C[6][6]);
    static void MatSub6(const double A[6][6], const double B[6][6], double C[6][6]);
    static void MatIdentity6(double A[6][6]);
    static bool MatInverse3(const double A[3][3], double Ainv[3][3]);
};

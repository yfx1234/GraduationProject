#pragma once

#include "CoreMinimal.h"
#include "ITargetPredictor.h"
#include "KalmanPredictor.generated.h"

/**
 * @brief 卡尔曼滤波目标预测器
 * 状态向量 x = [px, py, pz, vx, vy, vz] 
 * 观测向量 z = [px, py, pz] 
 * 运动模型：匀速模型
 * 状态转移:  x_k = F * x_{k-1}
 *   F = | I  dt*I |
 *       | 0    I  |
 * 观测矩阵:  z = H * x
 *   H = | I  0 |
 */
UCLASS(BlueprintType)
class GRADUATIONPROJECT_API UKalmanPredictor : public UObject, public ITargetPredictor
{
    GENERATED_BODY()

public:
    UKalmanPredictor();

    /**
     * @brief 初始化滤波器参数
     * @param ProcessNoise 过程噪声强度 Q
     * @param MeasurementNoise 测量噪声强度 R
     */
    void Initialize(float ProcessNoise = 1.0f, float MeasurementNoise = 0.5f);

    /**
     * @brief 用新的观测位置更新卡尔曼滤波器
     * @param ObservedPos 目标观测位置
     * @param DeltaTime 距上次更新的时间间隔 (秒)
     */
    virtual void Update(const FVector& ObservedPos, float DeltaTime) override;

    /**
     * @brief 基于匀速模型预测未来位置
     * @param dt 需要预测的时间量 (秒)
     * @return 预测的未来位置
     */
    virtual FVector PredictPosition(float dt) const override;

    /** @brief 获取卡尔曼估计的目标速度向量 [vx, vy, vz] */
    virtual FVector GetEstimatedVelocity() const override;

    /** @brief 获取卡尔曼估计的目标位置向量 [px, py, pz] */
    virtual FVector GetEstimatedPosition() const override;

    /** @brief 重置滤波器状态 */
    virtual void Reset() override;

    /** @brief 滤波器是否已接收过至少一次观测数据 */
    virtual bool IsInitialized() const override { return bInitialized; }

    /**
     * @brief 获取位置估计的不确定性
     * @return 位置协方差矩阵的迹，值越小越确定
     */
    float GetPositionUncertainty() const;

private:
    double X[6];       //6维状态向量: [px, py, pz, vx, vy, vz]
    double P[6][6];    //6×6 误差协方差矩阵

    float QScale;      //过程噪声强度
    float RScale;      //测量噪声强度

    bool bInitialized; //是否已完成首次初始化
    int32 UpdateCount; //累计更新次数

    /** @brief 6×6 矩阵乘法: C = A * B */
    static void MatMul6(const double A[6][6], const double B[6][6], double C[6][6]);

    /** @brief 6×6 矩阵转置: AT = A^T */
    static void MatTranspose6(const double A[6][6], double AT[6][6]);

    /** @brief 6×6 矩阵加法: C = A + B */
    static void MatAdd6(const double A[6][6], const double B[6][6], double C[6][6]);

    /** @brief 6×6 矩阵减法: C = A - B */
    static void MatSub6(const double A[6][6], const double B[6][6], double C[6][6]);

    /** @brief 6×6 单位矩阵: A = I */
    static void MatIdentity6(double A[6][6]);

    /** @brief 3×3 矩阵求逆 */
    static bool MatInverse3(const double A[3][3], double Ainv[3][3]);
};

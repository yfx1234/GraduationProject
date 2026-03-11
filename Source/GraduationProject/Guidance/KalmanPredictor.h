#pragma once

#include "CoreMinimal.h"
#include "ITargetPredictor.h"
#include "KalmanPredictor.generated.h"

/**
 * @brief 卡尔曼滤波目标预测器（9维常加速模型）
 * 状态向量 x = [px, py, pz, vx, vy, vz, ax, ay, az] (9维)
 * 观测向量 z = [px, py, pz]                          (3维)
 * 运动模型：常加速模型 (Constant Acceleration, CA)
 * 状态转移:  x_k = F * x_{k-1}
 *   F = | I   dt*I  0.5*dt²*I |
 *       | 0     I      dt*I   |
 *       | 0     0        I    |
 * 观测矩阵:  z = H * x
 *   H = | I  0  0 |
 *
 * 相比旧版6维匀速(CV)模型，9维CA模型能捕获目标的加速度变化，
 * 对高机动目标（低慢小无人机急转弯、规避机动）的预测精度大幅提升。
 *
 * 自适应噪声：基于残差序列自动调节过程噪声 Q，
 * 当目标机动剧烈时 Q 自动增大以跟随，稳态时 Q 自动减小以平滑。
 */
UCLASS(BlueprintType)
class GRADUATIONPROJECT_API UKalmanPredictor : public UObject, public ITargetPredictor
{
    GENERATED_BODY()

public:
    UKalmanPredictor();

    /**
     * @brief 初始化滤波器参数
     * @param ProcessNoise 过程噪声基准强度 Q
     * @param MeasurementNoise 测量噪声强度 R
     */
    void Initialize(float ProcessNoise = 1.0f, float MeasurementNoise = 0.5f);

    /**
     * @brief 用新的观测位置更新卡尔曼滤波器（预测+更新）
     * @param ObservedPos 目标观测位置
     * @param DeltaTime 距上次更新的时间间隔 (秒)
     */
    virtual void Update(const FVector& ObservedPos, float DeltaTime) override;

    /**
     * @brief 基于常加速模型预测未来位置
     * @param dt 需要预测的时间量 (秒)
     * @return 预测的未来位置: pos + vel*dt + 0.5*acc*dt²
     */
    virtual FVector PredictPosition(float dt) const override;

    /** @brief 获取卡尔曼估计的目标速度向量 [vx, vy, vz] */
    virtual FVector GetEstimatedVelocity() const override;

    /** @brief 获取卡尔曼估计的目标位置向量 [px, py, pz] */
    virtual FVector GetEstimatedPosition() const override;

    /** @brief 获取卡尔曼估计的目标加速度向量 [ax, ay, az] */
    FVector GetEstimatedAcceleration() const;

    /** @brief 重置滤波器状态 */
    virtual void Reset() override;

    /** @brief 滤波器是否已接收过至少一次观测数据 */
    virtual bool IsInitialized() const override { return bInitialized; }

    /**
     * @brief 获取位置估计的不确定性
     * @return 位置协方差矩阵对角线之和（迹），值越小越确定
     */
    float GetPositionUncertainty() const;

    /**
     * @brief 获取当前自适应后的过程噪声强度
     * @return 自适应 Q 值
     */
    float GetAdaptiveProcessNoise() const { return AdaptiveQ; }

private:
    // ---- 9维状态向量和协方差 ----
    static constexpr int32 N_STATE = 9;   // 状态维度
    static constexpr int32 N_OBS   = 3;   // 观测维度

    double X[N_STATE];                     // 状态: [px,py,pz, vx,vy,vz, ax,ay,az]
    double P[N_STATE][N_STATE];            // 误差协方差矩阵

    float QScale;                          // 过程噪声基准强度
    float RScale;                          // 测量噪声强度
    float AdaptiveQ;                       // 自适应后的实际过程噪声

    bool bInitialized;                     // 是否已完成首次初始化
    int32 UpdateCount;                     // 累计更新次数

    // ---- 自适应噪声估计 ----
    /** @brief 残差滑动窗指数加权统计量 (EWMA) */
    double ResidualVariance[N_OBS];        // 各轴残差方差估计
    static constexpr double AdaptiveAlpha = 0.15;  // EWMA 衰减系数
    static constexpr double AdaptiveQMin  = 0.1;   // Q 下限
    static constexpr double AdaptiveQMax  = 200.0;  // Q 上限

    // ---- 矩阵工具函数（N×N 通用） ----
    /** @brief N×N 矩阵乘法: C = A * B (支持就地运算) */
    static void MatMul(const double* A, const double* B, double* C, int32 Dim);

    /** @brief N×N 矩阵转置: AT = A^T */
    static void MatTranspose(const double* A, double* AT, int32 Dim);

    /** @brief N×N 矩阵加法: C = A + B */
    static void MatAdd(const double* A, const double* B, double* C, int32 Dim);

    /** @brief N×N 矩阵减法: C = A - B */
    static void MatSub(const double* A, const double* B, double* C, int32 Dim);

    /** @brief 构建 N×N 单位矩阵: A = I */
    static void MatIdentity(double* A, int32 Dim);

    /** @brief 3×3 矩阵求逆 */
    static bool MatInverse3(const double A[3][3], double Ainv[3][3]);

};

// 解释：使用 `#pragma once` 防止该头文件在编译过程中被重复包含。
#pragma once

// 解释：引入 Unreal 的核心基础头文件，提供常用容器、数学类型和日志宏。
#include "CoreMinimal.h"
// 解释：引入 `ITargetPredictor.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "ITargetPredictor.h"
// 解释：引入 `KalmanPredictor.generated.h`，让 Unreal Header Tool 生成的反射代码在本文件中可见。
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
// 解释：使用 `UCLASS` 宏把当前类型注册到 Unreal 反射系统中，便于蓝图、序列化和编辑器识别。
UCLASS(BlueprintType)
// 解释：这一行声明 类 `UKalmanPredictor`，用于封装ukalman预测器相关的数据与行为。
class GRADUATIONPROJECT_API UKalmanPredictor : public UObject, public ITargetPredictor
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：展开 Unreal Header Tool 生成的样板代码，这是 UObject、Actor、组件等类型正常工作的基础。
    GENERATED_BODY()

// 解释：从这一行开始进入 `public` 访问区，下面的成员会对外部模块公开。
public:
    // 解释：调用 `UKalmanPredictor` 执行当前步骤需要的功能逻辑。
    UKalmanPredictor();

    /**
     * @brief 初始化滤波器参数
     * @param ProcessNoise 过程噪声基准强度 Q
     * @param MeasurementNoise 测量噪声强度 R
     */
    // 解释：这一行把右侧表达式的结果写入 `void Initialize(float ProcessNoise`，完成 voidinitializefloatprocessnoise 的更新。
    void Initialize(float ProcessNoise = 1.0f, float MeasurementNoise = 0.5f);

    /**
     * @brief 用新的观测位置更新卡尔曼滤波器（预测+更新）
     * @param ObservedPos 目标观测位置
     * @param DeltaTime 距上次更新的时间间隔 (秒)
     */
    // 解释：调用 `Update` 执行当前步骤需要的功能逻辑。
    virtual void Update(const FVector& ObservedPos, float DeltaTime) override;

    /**
     * @brief 基于常加速模型预测未来位置
     * @param dt 需要预测的时间量 (秒)
     * @return 预测的未来位置: pos + vel*dt + 0.5*acc*dt²
     */
    // 解释：调用 `PredictPosition` 执行当前步骤需要的功能逻辑。
    virtual FVector PredictPosition(float dt) const override;

    /** @brief 获取卡尔曼估计的目标速度向量 [vx, vy, vz] */
    // 解释：调用 `GetEstimatedVelocity` 执行当前步骤需要的功能逻辑。
    virtual FVector GetEstimatedVelocity() const override;

    /** @brief 获取卡尔曼估计的目标位置向量 [px, py, pz] */
    // 解释：调用 `GetEstimatedPosition` 执行当前步骤需要的功能逻辑。
    virtual FVector GetEstimatedPosition() const override;

    /** @brief 获取卡尔曼估计的目标加速度向量 [ax, ay, az] */
    // 解释：调用 `GetEstimatedAcceleration` 执行当前步骤需要的功能逻辑。
    FVector GetEstimatedAcceleration() const;

    /** @brief 重置滤波器状态 */
    // 解释：调用 `Reset` 执行当前步骤需要的功能逻辑。
    virtual void Reset() override;

    /** @brief 滤波器是否已接收过至少一次观测数据 */
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    virtual bool IsInitialized() const override { return bInitialized; }

    /**
     * @brief 获取位置估计的不确定性
     * @return 位置协方差矩阵对角线之和（迹），值越小越确定
     */
    // 解释：调用 `GetPositionUncertainty` 执行当前步骤需要的功能逻辑。
    float GetPositionUncertainty() const;

    /**
     * @brief 获取当前自适应后的过程噪声强度
     * @return 自适应 Q 值
     */
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    float GetAdaptiveProcessNoise() const { return AdaptiveQ; }

// 解释：从这一行开始进入 `private` 访问区，下面的成员只允许类内部访问。
private:
    // ---- 9维状态向量和协方差 ----
    // 解释：这一行声明成员或局部变量 `N_STATE`，用于保存N状态。
    static constexpr int32 N_STATE = 9;   // 状态维度
    // 解释：这一行声明成员或局部变量 `N_OBS`，用于保存Nobs。
    static constexpr int32 N_OBS   = 3;   // 观测维度

    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    double X[N_STATE];                     // 状态: [px,py,pz, vx,vy,vz, ax,ay,az]
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    double P[N_STATE][N_STATE];            // 误差协方差矩阵

    // 解释：这一行声明成员或局部变量 `QScale`，用于保存过程噪声缩放系数。
    float QScale;                          // 过程噪声基准强度
    // 解释：这一行声明成员或局部变量 `RScale`，用于保存观测噪声缩放系数。
    float RScale;                          // 测量噪声强度
    // 解释：这一行声明成员或局部变量 `AdaptiveQ`，用于保存自适应过程噪声。
    float AdaptiveQ;                       // 自适应后的实际过程噪声

    // 解释：这一行声明成员或局部变量 `bInitialized`，用于保存布尔标志 initialized。
    bool bInitialized;                     // 是否已完成首次初始化
    // 解释：这一行声明成员或局部变量 `UpdateCount`，用于保存更新计数器。
    int32 UpdateCount;                     // 累计更新次数

    // ---- 自适应噪声估计 ----
    /** @brief 残差滑动窗指数加权统计量 (EWMA) */
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    double ResidualVariance[N_OBS];        // 各轴残差方差估计
    // 解释：这一行声明成员或局部变量 `AdaptiveAlpha`，用于保存adaptivealpha。
    static constexpr double AdaptiveAlpha = 0.15;  // EWMA 衰减系数
    // 解释：这一行声明成员或局部变量 `AdaptiveQMin`，用于保存adaptiveqmin。
    static constexpr double AdaptiveQMin  = 0.1;   // Q 下限
    // 解释：这一行声明成员或局部变量 `AdaptiveQMax`，用于保存adaptiveqmax。
    static constexpr double AdaptiveQMax  = 200.0;  // Q 上限

    // ---- 矩阵工具函数（N×N 通用） ----
    /** @brief N×N 矩阵乘法: C = A * B (支持就地运算) */
    // 解释：调用 `MatMul` 执行当前步骤需要的功能逻辑。
    static void MatMul(const double* A, const double* B, double* C, int32 Dim);

    /** @brief N×N 矩阵转置: AT = A^T */
    // 解释：调用 `MatTranspose` 执行当前步骤需要的功能逻辑。
    static void MatTranspose(const double* A, double* AT, int32 Dim);

    /** @brief N×N 矩阵加法: C = A + B */
    // 解释：调用 `MatAdd` 执行当前步骤需要的功能逻辑。
    static void MatAdd(const double* A, const double* B, double* C, int32 Dim);

    /** @brief N×N 矩阵减法: C = A - B */
    // 解释：调用 `MatSub` 执行当前步骤需要的功能逻辑。
    static void MatSub(const double* A, const double* B, double* C, int32 Dim);

    /** @brief 构建 N×N 单位矩阵: A = I */
    // 解释：调用 `MatIdentity` 执行当前步骤需要的功能逻辑。
    static void MatIdentity(double* A, int32 Dim);

    /** @brief 3×3 矩阵求逆 */
    // 解释：调用 `MatInverse3` 执行当前步骤需要的功能逻辑。
    static bool MatInverse3(const double A[3][3], double Ainv[3][3]);

// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
};

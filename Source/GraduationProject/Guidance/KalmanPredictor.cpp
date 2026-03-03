#include "KalmanPredictor.h"

/**
 * @brief 构造函数
 * 初始化滤波器参数默认值，清零状态向量和协方差矩阵
 */
UKalmanPredictor::UKalmanPredictor()
    : QScale(1.0f)
    , RScale(0.5f)
    , bInitialized(false)
    , UpdateCount(0)
{
    FMemory::Memzero(X, sizeof(X));   
    FMemory::Memzero(P, sizeof(P));   
}

/**
 * @brief 初始化滤波器参数并重置状态
 * @param ProcessNoise 过程噪声强度 Q
 * @param MeasurementNoise 测量噪声强度 R
 */
void UKalmanPredictor::Initialize(float ProcessNoise, float MeasurementNoise)
{
    QScale = ProcessNoise;
    RScale = MeasurementNoise;
    Reset();
}

/** @brief 重置滤波器到初始状态 */
void UKalmanPredictor::Reset()
{
    FMemory::Memzero(X, sizeof(X));
    MatIdentity6(P);
    for (int i = 0; i < 6; i++) P[i][i] = 100.0;
    bInitialized = false;
    UpdateCount = 0;
}

/**
 * @brief 卡尔曼滤波器的核心：预测 + 更新步骤
 * @param ObservedPos 目标观测位置
 * @param DeltaTime 距上次更新的时间间隔 (秒)
 * 算法流程：
 * H = [I 0], I为3x3 单位矩阵, 0为3x3零矩阵
 * 用观测值初始化状态
 * 构建状态转移矩阵 F :
 * F = | I  dt*I | = [1  0  0  dt 0  0 ]
 *     | 0    I  |   [0  1  0  0  dt 0 ]
 *                   [0  0  1  0  0  dt]
 *                   [0  0  0  1  0  0 ]
 *                   [0  0  0  0  1  0 ]
 *                   [0  0  0  0  0  1 ]
 * 先验状态估计: XPred = F * X
 * 构建过程噪声矩阵 Q :
 * Q = q * | dt^3/3*I  dt^2/2*I | = [dt^3/3*q  0         0         dt^2/2*q  0         0       ]
 *         | dt^2/2*I    dt*I   |   [0         dt^3/3*q  0         0         dt^2/2*q  0       ]
 *                                  [0         0         dt^3/3*q  0         0         dt^2/2*q]
 *                                  [dt^2/2*q  0         0         dt*q      0         0       ]
 *                                  [0         dt^2/2*q  0         0         dt*q      0       ]
 *                                  [0         0         dt^2/2*q  0         0         dt*q    ]
 * 先验协方差估计: PPred = FP * FT = F * P * F^T + Q
 * 残差: y = Pos - H * XPred
 * 残差协方差: S = H * PPred * H^T + R = [PPred00 + r  PPred01      PPred02     ]
                                        [PPred10      PPred11 + r  PPred12     ]
                                        [PPred20      PPred21      PPred22 + r ]
 * 卡尔曼增益: K = PPred * H^T * S⁻¹
 * 后验状态估计: X = XPred + K * y
 * 后验协方差估计: P = (I - KH) * PPred
 */
void UKalmanPredictor::Update(const FVector& ObservedPos, float DeltaTime)
{
    if (DeltaTime <= 0.0f) return;
    if (!bInitialized)
    {
        X[0] = ObservedPos.X;   // X = [px, py, pz, vx, vy, vz]
        X[1] = ObservedPos.Y;
        X[2] = ObservedPos.Z;
        X[3] = 0.0; X[4] = 0.0; X[5] = 0.0;
        bInitialized = true;
        UpdateCount = 1;
        return;
    }
    double dt = (double)DeltaTime;
    double F[6][6];
    MatIdentity6(F);
    F[0][3] = dt; F[1][4] = dt; F[2][5] = dt;
    double XPred[6];
    for (int i = 0; i < 6; i++)         // XPred = [1  0  0  dt 0  0 ]
    {                                   //         [0  1  0  0  dt 0 ]
        XPred[i] = 0.0;                 //         [0  0  1  0  0  dt] * X
        for (int j = 0; j < 6; j++)     //         [0  0  0  1  0  0 ]
            XPred[i] += F[i][j] * X[j]; //         [0  0  0  0  1  0 ] 
    }                                   //         [0  0  0  0  0  1 ]
    double FT[6][6], FP[6][6], PPred[6][6];
    MatTranspose6(F, FT);     // FT = F^T
    MatMul6(F, P, FP);        // FP = F * P
    MatMul6(FP, FT, PPred);   // PPred = FP * FT = F * P * F^T
    double q = (double)QScale;
    double dt2 = dt * dt;
    double dt3 = dt2 * dt;
    double Q[6][6];
    FMemory::Memzero(Q, sizeof(Q));
    for (int i = 0; i < 3; i++)
    {
        Q[i][i] = q * dt3 / 3.0;          // q*dt^3/3
        Q[i][i + 3] = q * dt2 / 2.0;      // q*dt^2/2
        Q[i + 3][i] = q * dt2 / 2.0;      // q*dt^2/2
        Q[i + 3][i + 3] = q * dt;         // q*dt
    }
    MatAdd6(PPred, Q, PPred);             // PPred = PPred + Q
    double y[3];
    y[0] = ObservedPos.X - XPred[0];    // y[0] = px - XPred[0]
    y[1] = ObservedPos.Y - XPred[1];    // y[1] = py - XPred[1]
    y[2] = ObservedPos.Z - XPred[2];    // y[2] = pz - XPred[2]
    double r = (double)RScale;
    double S[3][3];
    for (int i = 0; i < 3; i++)                         // S = [PPred00 + r  PPred01      PPred02     ]
        for (int j = 0; j < 3; j++)                     //     [PPred10      PPred11 + r  PPred12     ]
            S[i][j] = PPred[i][j] + (i == j ? r : 0.0); //     [PPred20      PPred21      PPred22 + r ]
    double SInv[3][3];
    if (!MatInverse3(S, SInv)) return;
    double K[6][3];
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 3; j++)
        {
            K[i][j] = 0.0;
            for (int k = 0; k < 3; k++)
                K[i][j] += PPred[i][k] * SInv[k][j];    // K = PPred * H^T * S⁻¹
        }
    for (int i = 0; i < 6; i++)
    {
        X[i] = XPred[i];
        for (int j = 0; j < 3; j++)
            X[i] += K[i][j] * y[j];     // X = XPred + K * y
    }
    double KH[6][6];
    FMemory::Memzero(KH, sizeof(KH));
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 3; j++)
            KH[i][j] = K[i][j];     // KH 的前 3 列 = K
    double I_KH[6][6];
    MatIdentity6(I_KH);             // I
    MatSub6(I_KH, KH, I_KH);        // I - KH
    MatMul6(I_KH, PPred, P);        // P = (I - KH) * PPred
    UpdateCount++;
}

/**
 * @brief 基于匀速模型预测未来位置
 * @param dt 预测时间量
 * @return 预测的未来位置: pos + vel * dt
 */
FVector UKalmanPredictor::PredictPosition(float dt) const
{
    if (!bInitialized) return FVector::ZeroVector;
    return FVector(
        X[0] + X[3] * dt,
        X[1] + X[4] * dt,
        X[2] + X[5] * dt
    );
}

/** @brief 获取卡尔曼估计的目标速度 */
FVector UKalmanPredictor::GetEstimatedVelocity() const
{
    return FVector(X[3], X[4], X[5]);
}

/** @brief 获取卡尔曼估计的目标位置 */
FVector UKalmanPredictor::GetEstimatedPosition() const
{
    return FVector(X[0], X[1], X[2]);
}

/** @brief 获取位置估计的不确定性（位置协方差矩阵的迹） */
float UKalmanPredictor::GetPositionUncertainty() const
{
    return (float)(P[0][0] + P[1][1] + P[2][2]);
}

/** @brief 6×6 矩阵乘法: C = A * B */
void UKalmanPredictor::MatMul6(const double A[6][6], const double B[6][6], double C[6][6])
{
    double Temp[6][6];
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
        {
            Temp[i][j] = 0.0;
            for (int k = 0; k < 6; k++)
                Temp[i][j] += A[i][k] * B[k][j];
        }
    FMemory::Memcpy(C, Temp, sizeof(Temp));
}

/** @brief 6×6 矩阵转置: AT = A^T */
void UKalmanPredictor::MatTranspose6(const double A[6][6], double AT[6][6])
{
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            AT[j][i] = A[i][j];
}

/** @brief 6×6 矩阵加法: C = A + B */
void UKalmanPredictor::MatAdd6(const double A[6][6], const double B[6][6], double C[6][6])
{
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            C[i][j] = A[i][j] + B[i][j];
}

/** @brief 6×6 矩阵减法: C = A - B */
void UKalmanPredictor::MatSub6(const double A[6][6], const double B[6][6], double C[6][6])
{
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            C[i][j] = A[i][j] - B[i][j];
}

/** @brief 构建 6×6 单位矩阵: A = I */
void UKalmanPredictor::MatIdentity6(double A[6][6])
{
    FMemory::Memzero(A, sizeof(double) * 36);
    for (int i = 0; i < 6; i++) A[i][i] = 1.0;
}

/**
 * @brief 3×3 矩阵求逆
 * @param A 输入 3×3 矩阵
 * @param Ainv 输出逆矩阵
 * @return true 成功求逆; false 矩阵奇异
 * A⁻¹ = adj(A) / det(A)
 */
bool UKalmanPredictor::MatInverse3(const double A[3][3], double Ainv[3][3])
{
    double det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1])
               - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0])
               + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
    if (FMath::Abs(det) < 1e-12) return false;
    double invDet = 1.0 / det;
    Ainv[0][0] = (A[1][1] * A[2][2] - A[1][2] * A[2][1]) * invDet;
    Ainv[0][1] = (A[0][2] * A[2][1] - A[0][1] * A[2][2]) * invDet;
    Ainv[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) * invDet;
    Ainv[1][0] = (A[1][2] * A[2][0] - A[1][0] * A[2][2]) * invDet;
    Ainv[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) * invDet;
    Ainv[1][2] = (A[0][2] * A[1][0] - A[0][0] * A[1][2]) * invDet;
    Ainv[2][0] = (A[1][0] * A[2][1] - A[1][1] * A[2][0]) * invDet;
    Ainv[2][1] = (A[0][1] * A[2][0] - A[0][0] * A[2][1]) * invDet;
    Ainv[2][2] = (A[0][0] * A[1][1] - A[0][1] * A[1][0]) * invDet;
    return true;
}

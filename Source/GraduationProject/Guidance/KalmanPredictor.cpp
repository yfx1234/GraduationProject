#include "KalmanPredictor.h"

// ============================================================================
//  构造 / 初始化 / 重置
// ============================================================================

UKalmanPredictor::UKalmanPredictor()
    : QScale(1.0f)
    , RScale(0.5f)
    , AdaptiveQ(1.0f)
    , bInitialized(false)
    , UpdateCount(0)
{
    FMemory::Memzero(X, sizeof(X));
    FMemory::Memzero(P, sizeof(P));
    FMemory::Memzero(ResidualVariance, sizeof(ResidualVariance));
}

void UKalmanPredictor::Initialize(float ProcessNoise, float MeasurementNoise)
{
    QScale = ProcessNoise;
    RScale = MeasurementNoise;
    Reset();
}

void UKalmanPredictor::Reset()
{
    FMemory::Memzero(X, sizeof(X));
    FMemory::Memzero(P, sizeof(P));
    // 初始协方差：位置 1000, 速度 100, 加速度 10
    for (int32 i = 0; i < 3; i++) P[i][i]     = 1000.0;  // 位置不确定性
    for (int32 i = 3; i < 6; i++) P[i][i]     = 100.0;   // 速度不确定性
    for (int32 i = 6; i < 9; i++) P[i][i]     = 10.0;    // 加速度不确定性

    AdaptiveQ = (double)QScale;
    FMemory::Memzero(ResidualVariance, sizeof(ResidualVariance));

    bInitialized = false;
    UpdateCount = 0;
}

// ============================================================================
//  核心：预测 + 更新  (9维 CA 模型)
// ============================================================================
// 状态向量 x = [px, py, pz, vx, vy, vz, ax, ay, az]
// 状态转移:
//   F = | I   dt*I   0.5*dt²*I |
//       | 0     I      dt*I    |
//       | 0     0        I     |
// 观测矩阵 H = [I  0  0]  (3×9)
// 过程噪声 Q 基于加加速度(jerk)驱动的连续白噪声离散化
// ============================================================================

void UKalmanPredictor::Update(const FVector& ObservedPos, float DeltaTime)
{
    if (DeltaTime <= 0.0f) return;

    // ---------- 首次观测：直接初始化 ----------
    if (!bInitialized)
    {
        X[0] = ObservedPos.X;   // px
        X[1] = ObservedPos.Y;   // py
        X[2] = ObservedPos.Z;   // pz
        for (int32 i = 3; i < N_STATE; i++) X[i] = 0.0;
        bInitialized = true;
        UpdateCount = 1;
        return;
    }

    const double dt  = (double)DeltaTime;
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt3 * dt;
    const double dt5 = dt4 * dt;

    // ======== 1) 构建状态转移矩阵 F (9×9) ========
    double F[N_STATE * N_STATE];
    MatIdentity(F, N_STATE);
    for (int32 i = 0; i < 3; i++)
    {
        F[i * N_STATE + (i + 3)] = dt;            // pos += vel*dt
        F[i * N_STATE + (i + 6)] = 0.5 * dt2;     // pos += 0.5*acc*dt²
        F[(i + 3) * N_STATE + (i + 6)] = dt;      // vel += acc*dt
    }

    // ======== 2) 先验状态预测: XPred = F * X ========
    double XPred[N_STATE];
    for (int32 i = 0; i < N_STATE; i++)
    {
        XPred[i] = 0.0;
        for (int32 j = 0; j < N_STATE; j++)
            XPred[i] += F[i * N_STATE + j] * X[j];
    }

    // ======== 3) 先验协方差: PPred = F*P*F^T + Q ========
    double FT[N_STATE * N_STATE];
    double FP[N_STATE * N_STATE];
    double PPred[N_STATE * N_STATE];
    MatTranspose(F, FT, N_STATE);
    MatMul(F, (const double*)P, FP, N_STATE);
    MatMul(FP, FT, PPred, N_STATE);

    // 构建过程噪声 Q (基于 jerk 驱动离散化，参考 Bar-Shalom)
    // 对每个轴 i, Q 的 3×3 块 (pos_i, vel_i, acc_i) 为:
    //   q * | dt^5/20  dt^4/8  dt^3/6 |
    //       | dt^4/8   dt^3/3  dt^2/2 |
    //       | dt^3/6   dt^2/2    dt   |
    const double q = AdaptiveQ;
    double Q[N_STATE * N_STATE];
    FMemory::Memzero(Q, sizeof(Q));
    for (int32 axis = 0; axis < 3; axis++)
    {
        const int32 p = axis;       // 位置索引
        const int32 v = axis + 3;   // 速度索引
        const int32 a = axis + 6;   // 加速度索引

        Q[p * N_STATE + p] = q * dt5 / 20.0;
        Q[p * N_STATE + v] = q * dt4 / 8.0;
        Q[v * N_STATE + p] = q * dt4 / 8.0;
        Q[p * N_STATE + a] = q * dt3 / 6.0;
        Q[a * N_STATE + p] = q * dt3 / 6.0;
        Q[v * N_STATE + v] = q * dt3 / 3.0;
        Q[v * N_STATE + a] = q * dt2 / 2.0;
        Q[a * N_STATE + v] = q * dt2 / 2.0;
        Q[a * N_STATE + a] = q * dt;
    }
    MatAdd(PPred, Q, PPred, N_STATE);

    // ======== 4) 计算残差 y = z - H*XPred ========
    double y[N_OBS];
    y[0] = ObservedPos.X - XPred[0];
    y[1] = ObservedPos.Y - XPred[1];
    y[2] = ObservedPos.Z - XPred[2];

    // ======== 5) 残差协方差 S = H*PPred*H^T + R ========
    // H = [I 0 0], 所以 S = PPred[0:3,0:3] + R*I
    const double r = (double)RScale;
    double S[3][3];
    for (int32 i = 0; i < N_OBS; i++)
        for (int32 j = 0; j < N_OBS; j++)
            S[i][j] = PPred[i * N_STATE + j] + (i == j ? r : 0.0);

    double SInv[3][3];
    if (!MatInverse3(S, SInv)) return;

    // ======== 6) 卡尔曼增益 K = PPred * H^T * S^-1  (9×3) ========
    // K[i][j] = sum_k PPred[i][k] * H^T[k][j] * SInv  但 H^T 只在 k<3 时非零
    // 所以 K[i][j] = sum_k PPred[i][k] * SInv[k][j], k=0..2
    double K[N_STATE][N_OBS];
    for (int32 i = 0; i < N_STATE; i++)
        for (int32 j = 0; j < N_OBS; j++)
        {
            K[i][j] = 0.0;
            for (int32 k = 0; k < N_OBS; k++)
                K[i][j] += PPred[i * N_STATE + k] * SInv[k][j];
        }

    // ======== 7) 后验状态: X = XPred + K * y ========
    for (int32 i = 0; i < N_STATE; i++)
    {
        X[i] = XPred[i];
        for (int32 j = 0; j < N_OBS; j++)
            X[i] += K[i][j] * y[j];
    }

    // ======== 8) 后验协方差: P = (I - K*H) * PPred ========
    // KH (9×9) = K * H, 其中 H=[I 0 0], 所以 KH[i][j] = K[i][j] if j<3, else 0
    double IMinusKH[N_STATE * N_STATE];
    MatIdentity(IMinusKH, N_STATE);
    for (int32 i = 0; i < N_STATE; i++)
        for (int32 j = 0; j < N_OBS; j++)
            IMinusKH[i * N_STATE + j] -= K[i][j];

    double PNew[N_STATE * N_STATE];
    MatMul(IMinusKH, PPred, PNew, N_STATE);
    FMemory::Memcpy(P, PNew, sizeof(P));

    // ======== 9) 自适应过程噪声 (EWMA 残差方差估计) ========
    for (int32 i = 0; i < N_OBS; i++)
    {
        const double residSq = y[i] * y[i];
        if (UpdateCount <= 2)
        {
            ResidualVariance[i] = residSq;
        }
        else
        {
            ResidualVariance[i] = AdaptiveAlpha * residSq
                                + (1.0 - AdaptiveAlpha) * ResidualVariance[i];
        }
    }
    // 计算新 Q：若残差方差远高于 R，说明目标在机动, 需要增大 Q
    const double avgResidVar = (ResidualVariance[0] + ResidualVariance[1] + ResidualVariance[2]) / 3.0;
    const double ratio = avgResidVar / FMath::Max(r, 0.001);
    AdaptiveQ = FMath::Clamp(ratio * (double)QScale, AdaptiveQMin, AdaptiveQMax);

    UpdateCount++;
}

// ============================================================================
//  预测 / 输出接口
// ============================================================================

FVector UKalmanPredictor::PredictPosition(float dt) const
{
    if (!bInitialized) return FVector::ZeroVector;
    const double t  = (double)dt;
    const double t2 = t * t;
    return FVector(
        X[0] + X[3] * t + 0.5 * X[6] * t2,   // px + vx*t + 0.5*ax*t²
        X[1] + X[4] * t + 0.5 * X[7] * t2,   // py + vy*t + 0.5*ay*t²
        X[2] + X[5] * t + 0.5 * X[8] * t2    // pz + vz*t + 0.5*az*t²
    );
}

FVector UKalmanPredictor::GetEstimatedVelocity() const
{
    return FVector(X[3], X[4], X[5]);
}

FVector UKalmanPredictor::GetEstimatedPosition() const
{
    return FVector(X[0], X[1], X[2]);
}

FVector UKalmanPredictor::GetEstimatedAcceleration() const
{
    return FVector(X[6], X[7], X[8]);
}

float UKalmanPredictor::GetPositionUncertainty() const
{
    return (float)(P[0][0] + P[1][1] + P[2][2]);
}

// ============================================================================
//  N×N 通用矩阵工具函数
// ============================================================================

void UKalmanPredictor::MatMul(const double* A, const double* B, double* C, int32 Dim)
{
    // 使用临时缓冲区支持就地运算 (C == A 或 C == B)
    double Temp[N_STATE * N_STATE];
    for (int32 i = 0; i < Dim; i++)
        for (int32 j = 0; j < Dim; j++)
        {
            Temp[i * Dim + j] = 0.0;
            for (int32 k = 0; k < Dim; k++)
                Temp[i * Dim + j] += A[i * Dim + k] * B[k * Dim + j];
        }
    FMemory::Memcpy(C, Temp, sizeof(double) * Dim * Dim);
}

void UKalmanPredictor::MatTranspose(const double* A, double* AT, int32 Dim)
{
    double Temp[N_STATE * N_STATE];
    for (int32 i = 0; i < Dim; i++)
        for (int32 j = 0; j < Dim; j++)
            Temp[j * Dim + i] = A[i * Dim + j];
    FMemory::Memcpy(AT, Temp, sizeof(double) * Dim * Dim);
}

void UKalmanPredictor::MatAdd(const double* A, const double* B, double* C, int32 Dim)
{
    const int32 N = Dim * Dim;
    for (int32 i = 0; i < N; i++)
        C[i] = A[i] + B[i];
}

void UKalmanPredictor::MatSub(const double* A, const double* B, double* C, int32 Dim)
{
    const int32 N = Dim * Dim;
    for (int32 i = 0; i < N; i++)
        C[i] = A[i] - B[i];
}

void UKalmanPredictor::MatIdentity(double* A, int32 Dim)
{
    FMemory::Memzero(A, sizeof(double) * Dim * Dim);
    for (int32 i = 0; i < Dim; i++)
        A[i * Dim + i] = 1.0;
}

bool UKalmanPredictor::MatInverse3(const double A[3][3], double Ainv[3][3])
{
    const double det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1])
                     - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0])
                     + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
    if (FMath::Abs(det) < 1e-12) return false;

    const double invDet = 1.0 / det;
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

#include "KalmanPredictor.h"

UKalmanPredictor::UKalmanPredictor()
    : QScale(1.0f)
    , RScale(0.5f)
    , bInitialized(false)
    , UpdateCount(0)
{
    FMemory::Memzero(X, sizeof(X));
    FMemory::Memzero(P, sizeof(P));
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
    MatIdentity6(P);
    // 初始协方差较大，表示不确定
    for (int i = 0; i < 6; i++) P[i][i] = 100.0;
    bInitialized = false;
    UpdateCount = 0;
}

void UKalmanPredictor::Update(const FVector& ObservedPos, float DeltaTime)
{
    if (DeltaTime <= 0.0f) return;

    // 第一帧：直接初始化状态
    if (!bInitialized)
    {
        X[0] = ObservedPos.X;
        X[1] = ObservedPos.Y;
        X[2] = ObservedPos.Z;
        X[3] = 0.0; X[4] = 0.0; X[5] = 0.0;
        bInitialized = true;
        UpdateCount = 1;
        return;
    }

    double dt = (double)DeltaTime;

    // ========== 1. Predict ==========

    // 状态转移矩阵 F
    double F[6][6];
    MatIdentity6(F);
    F[0][3] = dt; F[1][4] = dt; F[2][5] = dt;

    // x_pred = F * x
    double XPred[6];
    for (int i = 0; i < 6; i++)
    {
        XPred[i] = 0.0;
        for (int j = 0; j < 6; j++)
            XPred[i] += F[i][j] * X[j];
    }

    // P_pred = F * P * F^T + Q
    double FT[6][6], FP[6][6], PPred[6][6];
    MatTranspose6(F, FT);
    MatMul6(F, P, FP);
    MatMul6(FP, FT, PPred);

    // 过程噪声 Q（匀速模型的离散噪声）
    // Q = q * | dt^3/3*I  dt^2/2*I |
    //         | dt^2/2*I    dt*I   |
    double q = (double)QScale;
    double dt2 = dt * dt;
    double dt3 = dt2 * dt;
    double Q[6][6];
    FMemory::Memzero(Q, sizeof(Q));
    for (int i = 0; i < 3; i++)
    {
        Q[i][i] = q * dt3 / 3.0;
        Q[i][i + 3] = q * dt2 / 2.0;
        Q[i + 3][i] = q * dt2 / 2.0;
        Q[i + 3][i + 3] = q * dt;
    }

    MatAdd6(PPred, Q, PPred);

    // ========== 2. Update ==========

    // 观测矩阵 H = [I 0] (3×6)，隐式处理
    // y = z - H * x_pred (残差)
    double y[3];
    y[0] = ObservedPos.X - XPred[0];
    y[1] = ObservedPos.Y - XPred[1];
    y[2] = ObservedPos.Z - XPred[2];

    // S = H * P_pred * H^T + R (3×3)
    // 因为 H=[I 0]，S = P_pred[0:3][0:3] + R
    double r = (double)RScale;
    double S[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            S[i][j] = PPred[i][j] + (i == j ? r : 0.0);

    // S⁻¹
    double SInv[3][3];
    if (!MatInverse3(S, SInv))
    {
        // 矩阵奇异，跳过更新
        return;
    }

    // K = P_pred * H^T * S⁻¹ (6×3)
    // H^T = | I |, 所以 P_pred * H^T = PPred[:][0:3]
    //       | 0 |
    double K[6][3];
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 3; j++)
        {
            K[i][j] = 0.0;
            for (int k = 0; k < 3; k++)
                K[i][j] += PPred[i][k] * SInv[k][j];
        }

    // x = x_pred + K * y
    for (int i = 0; i < 6; i++)
    {
        X[i] = XPred[i];
        for (int j = 0; j < 3; j++)
            X[i] += K[i][j] * y[j];
    }

    // P = (I - K*H) * P_pred
    // K*H 是 6×6，其中只有前3列非零
    double KH[6][6];
    FMemory::Memzero(KH, sizeof(KH));
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 3; j++)
            KH[i][j] = K[i][j]; // H=[I 0] 所以 K*H 的前3列就是 K

    double I_KH[6][6];
    MatIdentity6(I_KH);
    MatSub6(I_KH, KH, I_KH);
    MatMul6(I_KH, PPred, P);

    UpdateCount++;
}

FVector UKalmanPredictor::PredictPosition(float dt) const
{
    if (!bInitialized) return FVector::ZeroVector;

    // 匀速外推: pos + vel * dt
    return FVector(
        X[0] + X[3] * dt,
        X[1] + X[4] * dt,
        X[2] + X[5] * dt
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

float UKalmanPredictor::GetPositionUncertainty() const
{
    // 位置协方差矩阵的迹 (P[0][0] + P[1][1] + P[2][2])
    return (float)(P[0][0] + P[1][1] + P[2][2]);
}

// ========== 矩阵运算 ==========

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

void UKalmanPredictor::MatTranspose6(const double A[6][6], double AT[6][6])
{
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            AT[j][i] = A[i][j];
}

void UKalmanPredictor::MatAdd6(const double A[6][6], const double B[6][6], double C[6][6])
{
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            C[i][j] = A[i][j] + B[i][j];
}

void UKalmanPredictor::MatSub6(const double A[6][6], const double B[6][6], double C[6][6])
{
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            C[i][j] = A[i][j] - B[i][j];
}

void UKalmanPredictor::MatIdentity6(double A[6][6])
{
    FMemory::Memzero(A, sizeof(double) * 36);
    for (int i = 0; i < 6; i++) A[i][i] = 1.0;
}

bool UKalmanPredictor::MatInverse3(const double A[3][3], double Ainv[3][3])
{
    // 3×3 矩阵求逆（伴随矩阵法）
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

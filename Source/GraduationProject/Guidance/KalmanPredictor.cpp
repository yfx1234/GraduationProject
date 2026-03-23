#include "KalmanPredictor.h"
UKalmanPredictor::UKalmanPredictor()
    : QScale(1.0f)
    , RScale(0.5f)
    , AdaptiveQ(1.0f)
    , bInitialized(false)
    , UpdateCount(0)
    , BootstrapMeasurement(FVector::ZeroVector)
    , bHasBootstrapMeasurement(false)
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
    for (int32 i = 0; i < 3; i++) P[i][i]     = 1000.0;
    for (int32 i = 3; i < 6; i++) P[i][i]     = 100.0;
    for (int32 i = 6; i < 9; i++) P[i][i]     = 10.0;
    AdaptiveQ = (double)QScale;
    FMemory::Memzero(ResidualVariance, sizeof(ResidualVariance));
    bInitialized = false;
    UpdateCount = 0;
    BootstrapMeasurement = FVector::ZeroVector;
    bHasBootstrapMeasurement = false;
}
void UKalmanPredictor::Update(const FVector& ObservedPos, float DeltaTime)
{
    if (DeltaTime <= 0.0f) return;

    if (!bHasBootstrapMeasurement)
    {
        BootstrapMeasurement = ObservedPos;
        bHasBootstrapMeasurement = true;
        UpdateCount = 1;
        return;
    }

    if (!bInitialized)
    {
        const double SafeDt = FMath::Max(0.001f, DeltaTime);
        const FVector BootstrapVelocity = (ObservedPos - BootstrapMeasurement) / SafeDt;
        X[0] = ObservedPos.X;
        X[1] = ObservedPos.Y;
        X[2] = ObservedPos.Z;
        X[3] = BootstrapVelocity.X;
        X[4] = BootstrapVelocity.Y;
        X[5] = BootstrapVelocity.Z;
        X[6] = 0.0;
        X[7] = 0.0;
        X[8] = 0.0;
        bInitialized = true;
        bHasBootstrapMeasurement = false;
        UpdateCount = 2;
        return;
    }
    const double dt  = (double)DeltaTime;
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt3 * dt;
    const double dt5 = dt4 * dt;
    double F[N_STATE * N_STATE];
    MatIdentity(F, N_STATE);
    for (int32 i = 0; i < 3; i++)
    {
        F[i * N_STATE + (i + 3)] = dt;
        F[i * N_STATE + (i + 6)] = 0.5 * dt2;
        F[(i + 3) * N_STATE + (i + 6)] = dt;
    }
    double XPred[N_STATE];
    for (int32 i = 0; i < N_STATE; i++)
    {
        XPred[i] = 0.0;
        for (int32 j = 0; j < N_STATE; j++)
            XPred[i] += F[i * N_STATE + j] * X[j];
    }
    double FT[N_STATE * N_STATE];
    double FP[N_STATE * N_STATE];
    double PPred[N_STATE * N_STATE];
    MatTranspose(F, FT, N_STATE);
    MatMul(F, (const double*)P, FP, N_STATE);
    MatMul(FP, FT, PPred, N_STATE);
    const double q = AdaptiveQ;
    double Q[N_STATE * N_STATE];
    FMemory::Memzero(Q, sizeof(Q));
    for (int32 axis = 0; axis < 3; axis++)
    {
        const int32 p = axis;
        const int32 v = axis + 3;
        const int32 a = axis + 6;
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
    double y[N_OBS];
    y[0] = ObservedPos.X - XPred[0];
    y[1] = ObservedPos.Y - XPred[1];
    y[2] = ObservedPos.Z - XPred[2];
    const double r = (double)RScale;
    double S[3][3];
    for (int32 i = 0; i < N_OBS; i++)
        for (int32 j = 0; j < N_OBS; j++)
            S[i][j] = PPred[i * N_STATE + j] + (i == j ? r : 0.0);
    double SInv[3][3];
    if (!MatInverse3(S, SInv)) return;
    double K[N_STATE][N_OBS];
    for (int32 i = 0; i < N_STATE; i++)
        for (int32 j = 0; j < N_OBS; j++)
        {
            K[i][j] = 0.0;
            for (int32 k = 0; k < N_OBS; k++)
                K[i][j] += PPred[i * N_STATE + k] * SInv[k][j];
        }
    for (int32 i = 0; i < N_STATE; i++)
    {
        X[i] = XPred[i];
        for (int32 j = 0; j < N_OBS; j++)
            X[i] += K[i][j] * y[j];
    }
    double IMinusKH[N_STATE * N_STATE];
    MatIdentity(IMinusKH, N_STATE);
    for (int32 i = 0; i < N_STATE; i++)
        for (int32 j = 0; j < N_OBS; j++)
            IMinusKH[i * N_STATE + j] -= K[i][j];
    double PNew[N_STATE * N_STATE];
    MatMul(IMinusKH, PPred, PNew, N_STATE);
    FMemory::Memcpy(P, PNew, sizeof(P));
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
    const double avgResidVar = (ResidualVariance[0] + ResidualVariance[1] + ResidualVariance[2]) / 3.0;
    const double ratio = avgResidVar / FMath::Max(r, 0.001);
    AdaptiveQ = FMath::Clamp(ratio * (double)QScale, AdaptiveQMin, AdaptiveQMax);
    UpdateCount++;
}
FVector UKalmanPredictor::PredictPosition(float dt) const
{
    if (!bInitialized) return FVector::ZeroVector;
    const double t  = (double)dt;
    const double t2 = t * t;
    return FVector(
        X[0] + X[3] * t + 0.5 * X[6] * t2,
        X[1] + X[4] * t + 0.5 * X[7] * t2,
        X[2] + X[5] * t + 0.5 * X[8] * t2
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
void UKalmanPredictor::MatMul(const double* A, const double* B, double* C, int32 Dim)
{
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

#pragma once
#include "CoreMinimal.h"
#include "ITargetPredictor.h"
#include "KalmanPredictor.generated.h"
UCLASS(BlueprintType)
class GRADUATIONPROJECT_API UKalmanPredictor : public UObject, public ITargetPredictor
{
    GENERATED_BODY()
public:
    UKalmanPredictor();
    void Initialize(float ProcessNoise = 1.0f, float MeasurementNoise = 0.5f);
    virtual void Update(const FVector& ObservedPos, float DeltaTime) override;
    virtual FVector PredictPosition(float dt) const override;
    virtual FVector GetEstimatedVelocity() const override;
    virtual FVector GetEstimatedPosition() const override;
    FVector GetEstimatedAcceleration() const;
    virtual void Reset() override;
    virtual bool IsInitialized() const override { return bInitialized; }
    float GetPositionUncertainty() const;
    float GetAdaptiveProcessNoise() const { return AdaptiveQ; }
private:
    static constexpr int32 N_STATE = 9;
    static constexpr int32 N_OBS   = 3;
    double X[N_STATE];
    double P[N_STATE][N_STATE];
    float QScale;
    float RScale;
    float AdaptiveQ;
    bool bInitialized;
    int32 UpdateCount;
    FVector BootstrapMeasurement;
    bool bHasBootstrapMeasurement;
    double ResidualVariance[N_OBS];
    static constexpr double AdaptiveAlpha = 0.15;
    static constexpr double AdaptiveQMin  = 0.1;
    static constexpr double AdaptiveQMax  = 200.0;
    static void MatMul(const double* A, const double* B, double* C, int32 Dim);
    static void MatTranspose(const double* A, double* AT, int32 Dim);
    static void MatAdd(const double* A, const double* B, double* C, int32 Dim);
    static void MatSub(const double* A, const double* B, double* C, int32 Dim);
    static void MatIdentity(double* A, int32 Dim);
    static bool MatInverse3(const double A[3][3], double Ainv[3][3]);
};

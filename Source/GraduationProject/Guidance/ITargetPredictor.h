#pragma once
#include "CoreMinimal.h"
class ITargetPredictor
{
public:
    virtual ~ITargetPredictor() = default;
    virtual void Update(const FVector& ObservedPos, float DeltaTime) = 0;
    virtual FVector PredictPosition(float dt) const = 0;
    virtual FVector GetEstimatedVelocity() const = 0;
    virtual FVector GetEstimatedPosition() const = 0;
    virtual void Reset() = 0;
    virtual bool IsInitialized() const = 0;
};

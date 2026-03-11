#pragma once

#include "CoreMinimal.h"

class USceneCaptureComponent2D;

namespace AirSimImageUtils
{
    enum class EImageType : int32
    {
        Scene = 0,
        DepthPlanar = 1,
        DepthVis = 3,
        Segmentation = 5,
        Infrared = 7
    };

    bool TryParseImageType(const FString& RawType, EImageType& OutType);
    FString ToCanonicalString(EImageType Type);
    FString ToDisplayString(EImageType Type);
    TArray<EImageType> SupportedImageTypes();

    bool CapturePixels(
        USceneCaptureComponent2D* Capture,
        int32 Width,
        int32 Height,
        EImageType Type,
        TArray<FColor>& OutPixels,
        float MaxDepthMeters = 200.0f);

    FString CaptureJpegBase64(
        USceneCaptureComponent2D* Capture,
        int32 Width,
        int32 Height,
        EImageType Type,
        int32 Quality = 85,
        float MaxDepthMeters = 200.0f);
}

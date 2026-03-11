#pragma once

#include "CoreMinimal.h"

class USceneCaptureComponent2D;

namespace CameraCaptureUtils
{
    FString CaptureColorJpegBase64(USceneCaptureComponent2D* Capture, int32 Width, int32 Height, int32 Quality);
}

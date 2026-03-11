#pragma once

#include "CoreMinimal.h"

class USceneCaptureComponent2D;

namespace CameraCaptureUtils
{
    /**
     * @brief 捕获彩色场景图并编码为 Base64 JPEG
     * @param Capture 场景捕获组件
     * @param Width 输出宽度
     * @param Height 输出高度
     * @param Quality JPEG 压缩质量
     * @return Base64 编码后的 JPEG 字符串；失败时返回空串
     */
    FString CaptureColorJpegBase64(USceneCaptureComponent2D* Capture, int32 Width, int32 Height, int32 Quality);
}

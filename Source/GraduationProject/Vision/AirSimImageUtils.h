#pragma once

#include "CoreMinimal.h"

class USceneCaptureComponent2D;

namespace AirSimImageUtils
{
    /** @brief 与 AirSim 协议对齐的图像类型枚举 */
    enum class EImageType : int32
    {
        Scene = 0,
        DepthPlanar = 1,
        DepthVis = 3,
        Segmentation = 5,
        Infrared = 7
    };

    /**
     * @brief 解析图像类型字符串或数字
     * @param RawType 原始类型标识
     * @param OutType 输出图像类型
     * @return 解析成功时返回 `true`
     */
    bool TryParseImageType(const FString& RawType, EImageType& OutType);

    /**
     * @brief 获取图像类型的标准协议字符串
     * @param Type 图像类型
     * @return 标准字符串，例如 `scene`、`depth_planar`
     */
    FString ToCanonicalString(EImageType Type);

    /**
     * @brief 获取图像类型的显示字符串
     * @param Type 图像类型
     * @return 用于 HUD 或调试面板的显示名称
     */
    FString ToDisplayString(EImageType Type);

    /**
     * @brief 获取当前支持的全部图像类型
     * @return 图像类型数组
     */
    TArray<EImageType> SupportedImageTypes();

    /**
     * @brief 捕获指定类型的原始像素
     * @param Capture 场景捕获组件
     * @param Width 输出宽度
     * @param Height 输出高度
     * @param Type 图像类型
     * @param OutPixels 输出像素缓存
     * @param MaxDepthMeters 深度图最大映射距离（米）
     * @return 捕获成功时返回 `true`
     */
    bool CapturePixels(
        USceneCaptureComponent2D* Capture,
        int32 Width,
        int32 Height,
        EImageType Type,
        TArray<FColor>& OutPixels,
        float MaxDepthMeters = 200.0f);

    /**
     * @brief 捕获指定类型图像并编码为 Base64 JPEG
     * @param Capture 场景捕获组件
     * @param Width 输出宽度
     * @param Height 输出高度
     * @param Type 图像类型
     * @param Quality JPEG 压缩质量
     * @param MaxDepthMeters 深度图最大映射距离（米）
     * @return Base64 编码后的 JPEG 字符串
     */
    FString CaptureJpegBase64(
        USceneCaptureComponent2D* Capture,
        int32 Width,
        int32 Height,
        EImageType Type,
        int32 Quality = 85,
        float MaxDepthMeters = 200.0f);
}

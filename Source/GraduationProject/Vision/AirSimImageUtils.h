// 解释：使用 `#pragma once` 防止该头文件在编译过程中被重复包含。
#pragma once

// 解释：引入 Unreal 的核心基础头文件，提供常用容器、数学类型和日志宏。
#include "CoreMinimal.h"

// 解释：这一行声明 类 `USceneCaptureComponent2D`，用于封装uscene采集component2D相关的数据与行为。
class USceneCaptureComponent2D;

// 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
namespace AirSimImageUtils
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    /** @brief 与 AirSim 协议对齐的图像类型枚举 */
    // 解释：这一行声明枚举 `EImageType`，用于约束一组有限的状态或模式取值。
    enum class EImageType : int32
    {
        Scene = 0
    };

    /**
     * @brief 解析图像类型字符串或数字
     * @param RawType 原始类型标识
     * @param OutType 输出图像类型
     * @return 解析成功时返回 `true`
     */
    // 解释：调用 `TryParseImageType` 执行当前步骤需要的功能逻辑。
    bool TryParseImageType(const FString& RawType, EImageType& OutType);

    /**
     * @brief 获取图像类型的标准协议字符串
     * @param Type 图像类型
     * @return 标准字符串，例如 `scene`、`depth_planar`
     */
    // 解释：调用 `ToCanonicalString` 执行当前步骤需要的功能逻辑。
    FString ToCanonicalString(EImageType Type);

    /**
     * @brief 获取图像类型的显示字符串
     * @param Type 图像类型
     * @return 用于 HUD 或调试面板的显示名称
     */
    // 解释：调用 `ToDisplayString` 执行当前步骤需要的功能逻辑。
    FString ToDisplayString(EImageType Type);

    /**
     * @brief 获取当前支持的全部图像类型
     * @return 图像类型数组
     */
    // 解释：调用 `SupportedImageTypes` 执行当前步骤需要的功能逻辑。
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
    // 解释：这一行定义函数 `CapturePixels`，开始实现采集pixels的具体逻辑。
    bool CapturePixels(
        // 解释：这一行继续展开 `CapturePixels` 的参数列表，声明参数 `Capture` 用于传入采集。
        USceneCaptureComponent2D* Capture,
        // 解释：这一行继续展开 `CapturePixels` 的参数列表，声明参数 `Width` 用于传入width。
        int32 Width,
        // 解释：这一行继续展开 `CapturePixels` 的参数列表，声明参数 `Height` 用于传入height。
        int32 Height,
        // 解释：这一行继续展开 `CapturePixels` 的参数列表，声明参数 `Type` 用于传入type。
        EImageType Type,
        // 解释：这一行继续展开 `CapturePixels` 的参数列表，声明参数 `OutPixels` 用于传入outpixels。
        TArray<FColor>& OutPixels,
        // 解释：这一行声明成员或局部变量 `MaxDepthMeters`，用于保存maxdepthmeters。
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
    // 解释：这一行定义函数 `CaptureJpegBase64`，开始实现采集jpegbase64的具体逻辑。
    FString CaptureJpegBase64(
        // 解释：这一行继续展开 `CaptureJpegBase64` 的参数列表，声明参数 `Capture` 用于传入采集。
        USceneCaptureComponent2D* Capture,
        // 解释：这一行继续展开 `CaptureJpegBase64` 的参数列表，声明参数 `Width` 用于传入width。
        int32 Width,
        // 解释：这一行继续展开 `CaptureJpegBase64` 的参数列表，声明参数 `Height` 用于传入height。
        int32 Height,
        // 解释：这一行继续展开 `CaptureJpegBase64` 的参数列表，声明参数 `Type` 用于传入type。
        EImageType Type,
        // 解释：这一行声明成员或局部变量 `Quality`，用于保存quality。
        int32 Quality = 85,
        // 解释：这一行声明成员或局部变量 `MaxDepthMeters`，用于保存maxdepthmeters。
        float MaxDepthMeters = 200.0f);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

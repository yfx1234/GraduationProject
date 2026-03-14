// 解释：使用 `#pragma once` 防止该头文件在编译过程中被重复包含。
#pragma once

// 解释：引入 Unreal 的核心基础头文件，提供常用容器、数学类型和日志宏。
#include "CoreMinimal.h"

// 解释：这一行声明 类 `AActor`，用于封装aactor相关的数据与行为。
class AActor;
// 解释：这一行声明 类 `UCineCameraComponent`，用于封装ucine相机组件相关的数据与行为。
class UCineCameraComponent;
// 解释：这一行声明 类 `UObject`，用于封装uobject相关的数据与行为。
class UObject;
// 解释：这一行声明 类 `USceneCaptureComponent2D`，用于封装uscene采集component2D相关的数据与行为。
class USceneCaptureComponent2D;
// 解释：这一行声明 类 `UTextureRenderTarget2D`，用于封装utexturerendertarget2D相关的数据与行为。
class UTextureRenderTarget2D;

/**
 * @brief 相机捕获与后处理辅助工具集
 *
 * 该命名空间把“创建 RenderTarget、抓取图像、同步后处理参数、
 * 设置分割 Stencil”等横跨无人机和其他载体的公共逻辑集中管理，
 * 避免在多个 Pawn 中重复实现相同代码。
 */
// 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
namespace CameraCaptureUtils
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    /**
     * @brief 创建彩色 RenderTarget
     * @param Owner  RenderTarget 的 Outer
     * @param Width  目标宽度
     * @param Height 目标高度
     * @return 新建的 RenderTarget；尺寸非法或创建失败时返回 nullptr
     */
    // 解释：调用 `CreateColorRenderTarget` 执行当前步骤需要的功能逻辑。
    UTextureRenderTarget2D* CreateColorRenderTarget(UObject* Owner, int32 Width, int32 Height);

    /**
     * @brief 抓取一帧彩色图像并编码为 Base64 JPEG
     * @param Capture 场景捕获组件
     * @param Width   输出宽度
     * @param Height  输出高度
     * @param Quality JPEG 质量
     * @return Base64 字符串；失败时返回空串
     */
    // 解释：调用 `CaptureColorJpegBase64` 执行当前步骤需要的功能逻辑。
    FString CaptureColorJpegBase64(USceneCaptureComponent2D* Capture, int32 Width, int32 Height, int32 Quality);

    /**
     * @brief 按 AirSim 风格封装图像 JSON 响应
     * @param Capture        场景捕获组件
     * @param SourceId       图像来源 ID
     * @param Width          图像宽度
     * @param Height         图像高度
     * @param FieldOfView    相机视场角
     * @param DefaultQuality 默认 JPEG 质量
     * @param ImageType      图像类型字符串
     * @param Quality        外部指定 JPEG 质量
     * @param MaxDepthMeters 深度图最大映射距离
     * @return JSON 字符串
     */
    // 解释：这一行定义函数 `CaptureAirSimImageJson`，开始实现采集空中仿真图像json的具体逻辑。
    FString CaptureAirSimImageJson(
        // 解释：这一行继续展开 `CaptureAirSimImageJson` 的参数列表，声明参数 `Capture` 用于传入采集。
        USceneCaptureComponent2D* Capture,
        // 解释：这一行继续展开 `CaptureAirSimImageJson` 的参数列表，声明参数 `SourceId` 用于传入sourceid。
        const FString& SourceId,
        // 解释：这一行继续展开 `CaptureAirSimImageJson` 的参数列表，声明参数 `Width` 用于传入width。
        int32 Width,
        // 解释：这一行继续展开 `CaptureAirSimImageJson` 的参数列表，声明参数 `Height` 用于传入height。
        int32 Height,
        // 解释：这一行继续展开 `CaptureAirSimImageJson` 的参数列表，声明参数 `FieldOfView` 用于传入fieldofview。
        float FieldOfView,
        // 解释：这一行继续展开 `CaptureAirSimImageJson` 的参数列表，声明参数 `DefaultQuality` 用于传入defaultquality。
        int32 DefaultQuality,
        // 解释：这一行继续展开 `CaptureAirSimImageJson` 的参数列表，声明参数 `ImageType` 用于传入图像type。
        const FString& ImageType,
        // 解释：这一行继续展开 `CaptureAirSimImageJson` 的参数列表，声明参数 `Quality` 用于传入quality。
        int32 Quality,
        // 解释：这一行继续补充函数 `CaptureAirSimImageJson` 的参数列表、限定符或返回类型说明。
        float MaxDepthMeters);

    /**
     * @brief 为 Actor 全部 Primitive 组件应用语义分割 Stencil 值
     * @param Owner          目标 Actor
     * @param SegmentationId 0-255 的自定义深度模板值
     */
    // 解释：调用 `ApplySegmentationStencil` 执行当前步骤需要的功能逻辑。
    void ApplySegmentationStencil(AActor* Owner, int32 SegmentationId);

    /**
     * @brief 把 CineCamera 的后处理设置同步到 SceneCapture
     * @param CineCamera         电影相机组件
     * @param Capture            场景捕获组件
     * @param ExposureBias       额外曝光补偿
     * @param bDisableMotionBlur 是否强制关闭动态模糊
     */
    // 解释：这一行定义函数 `SyncPostProcessToCapture`，开始实现syncpostprocessto采集的具体逻辑。
    void SyncPostProcessToCapture(
        // 解释：这一行继续展开 `SyncPostProcessToCapture` 的参数列表，声明参数 `CineCamera` 用于传入cine相机。
        UCineCameraComponent* CineCamera,
        // 解释：这一行继续展开 `SyncPostProcessToCapture` 的参数列表，声明参数 `Capture` 用于传入采集。
        USceneCaptureComponent2D* Capture,
        // 解释：这一行继续展开 `SyncPostProcessToCapture` 的参数列表，声明参数 `ExposureBias` 用于传入exposurebias。
        float ExposureBias,
        // 解释：这一行声明成员或局部变量 `bDisableMotionBlur`，用于保存布尔标志 disablemotionblur。
        bool bDisableMotionBlur = false);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

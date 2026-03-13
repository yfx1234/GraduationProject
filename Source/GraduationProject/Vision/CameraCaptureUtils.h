#pragma once

#include "CoreMinimal.h"

class AActor;
class UCineCameraComponent;
class UObject;
class USceneCaptureComponent2D;
class UTextureRenderTarget2D;

/**
 * @brief 相机捕获与后处理辅助工具集
 *
 * 该命名空间把“创建 RenderTarget、抓取图像、同步后处理参数、
 * 设置分割 Stencil”等横跨无人机和其他载体的公共逻辑集中管理，
 * 避免在多个 Pawn 中重复实现相同代码。
 */
namespace CameraCaptureUtils
{
    /**
     * @brief 创建彩色 RenderTarget
     * @param Owner  RenderTarget 的 Outer
     * @param Width  目标宽度
     * @param Height 目标高度
     * @return 新建的 RenderTarget；尺寸非法或创建失败时返回 nullptr
     */
    UTextureRenderTarget2D* CreateColorRenderTarget(UObject* Owner, int32 Width, int32 Height);

    /**
     * @brief 抓取一帧彩色图像并编码为 Base64 JPEG
     * @param Capture 场景捕获组件
     * @param Width   输出宽度
     * @param Height  输出高度
     * @param Quality JPEG 质量
     * @return Base64 字符串；失败时返回空串
     */
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
    FString CaptureAirSimImageJson(
        USceneCaptureComponent2D* Capture,
        const FString& SourceId,
        int32 Width,
        int32 Height,
        float FieldOfView,
        int32 DefaultQuality,
        const FString& ImageType,
        int32 Quality,
        float MaxDepthMeters);

    /**
     * @brief 为 Actor 全部 Primitive 组件应用语义分割 Stencil 值
     * @param Owner          目标 Actor
     * @param SegmentationId 0-255 的自定义深度模板值
     */
    void ApplySegmentationStencil(AActor* Owner, int32 SegmentationId);

    /**
     * @brief 把 CineCamera 的后处理设置同步到 SceneCapture
     * @param CineCamera         电影相机组件
     * @param Capture            场景捕获组件
     * @param ExposureBias       额外曝光补偿
     * @param bDisableMotionBlur 是否强制关闭动态模糊
     */
    void SyncPostProcessToCapture(
        UCineCameraComponent* CineCamera,
        USceneCaptureComponent2D* Capture,
        float ExposureBias,
        bool bDisableMotionBlur = false);
}
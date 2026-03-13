#include "CameraCaptureUtils.h"

#include "AirSimImageUtils.h"
#include "CineCameraComponent.h"
#include "Components/PrimitiveComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "GameFramework/Actor.h"
#include "HAL/IConsoleManager.h"
#include "IImageWrapper.h"
#include "IImageWrapperModule.h"
#include "Misc/Base64.h"
#include "Modules/ModuleManager.h"

namespace
{
    /**
     * @brief 转义 JSON 中的字符串字段
     * @param In 原始字符串
     * @return 适合直接嵌入 JSON 的转义结果
     */
    FString EscapeJsonValue(const FString& In)
    {
        FString Out = In;
        Out.ReplaceInline(TEXT("\\"), TEXT("\\\\"));
        Out.ReplaceInline(TEXT("\""), TEXT("\\\""));
        return Out;
    }

    /**
     * @brief 解析最终 JPEG 质量
     * @param Quality        外部传入质量
     * @param DefaultQuality 默认质量
     * @return 合法范围 [1, 100] 内的最终质量
     */
    int32 ResolveJpegQuality(int32 Quality, int32 DefaultQuality)
    {
        return (Quality > 0) ? Quality : FMath::Clamp(DefaultQuality, 1, 100);
    }

    /**
     * @brief 把 BGRA 像素编码为 Base64 JPEG
     * @param Pixels    像素数组
     * @param Width     图像宽度
     * @param Height    图像高度
     * @param Quality   JPEG 质量
     * @param OutBase64 输出 Base64 字符串
     * @return 编码成功时返回 true
     */
    bool EncodeBgraToJpegBase64(const TArray<FColor>& Pixels, int32 Width, int32 Height, int32 Quality, FString& OutBase64)
    {
        if (Pixels.Num() != Width * Height || Width <= 0 || Height <= 0)
        {
            return false;
        }

        // `FColor` 以 BGRA 顺序存储，这里显式展开为连续缓冲区供 ImageWrapper 压缩。
        TArray<uint8> RawData;
        RawData.SetNum(Pixels.Num() * 4);
        for (int32 i = 0; i < Pixels.Num(); ++i)
        {
            RawData[i * 4 + 0] = Pixels[i].B;
            RawData[i * 4 + 1] = Pixels[i].G;
            RawData[i * 4 + 2] = Pixels[i].R;
            RawData[i * 4 + 3] = Pixels[i].A;
        }

        IImageWrapperModule& ImageWrapperModule = FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));
        TSharedPtr<IImageWrapper> Wrapper = ImageWrapperModule.CreateImageWrapper(EImageFormat::JPEG);
        if (!Wrapper.IsValid())
        {
            return false;
        }

        if (!Wrapper->SetRaw(RawData.GetData(), RawData.Num(), Width, Height, ERGBFormat::BGRA, 8))
        {
            return false;
        }

        const TArray64<uint8>& JpegData = Wrapper->GetCompressed(FMath::Clamp(Quality, 1, 100));
        OutBase64 = FBase64::Encode(JpegData.GetData(), JpegData.Num());
        return true;
    }
}

/**
 * @brief 创建彩色 RenderTarget
 * @param Owner  RenderTarget 的 Outer
 * @param Width  宽度
 * @param Height 高度
 * @return 新建 RenderTarget；失败时返回 nullptr
 */
UTextureRenderTarget2D* CameraCaptureUtils::CreateColorRenderTarget(UObject* Owner, int32 Width, int32 Height)
{
    if (Width <= 0 || Height <= 0)
    {
        return nullptr;
    }

    UObject* Outer = Owner ? Owner : GetTransientPackage();
    UTextureRenderTarget2D* RenderTarget = NewObject<UTextureRenderTarget2D>(Outer);
    if (!RenderTarget)
    {
        return nullptr;
    }

    RenderTarget->InitCustomFormat(Width, Height, PF_B8G8R8A8, false);
    RenderTarget->ClearColor = FLinearColor::Black;
    return RenderTarget;
}

/**
 * @brief 抓取一帧彩色图像并编码为 Base64 JPEG
 * @param Capture 场景捕获组件
 * @param Width   输出宽度
 * @param Height  输出高度
 * @param Quality JPEG 质量
 * @return Base64 字符串；失败时返回空串
 */
FString CameraCaptureUtils::CaptureColorJpegBase64(USceneCaptureComponent2D* Capture, int32 Width, int32 Height, int32 Quality)
{
    if (!Capture || !Capture->TextureTarget || Width <= 0 || Height <= 0)
    {
        return TEXT("");
    }

    // 若关闭了逐帧捕获，则手动触发一次渲染，确保读取到最新图像。
    if (!Capture->bCaptureEveryFrame)
    {
        Capture->CaptureScene();
    }

    FTextureRenderTargetResource* Resource = Capture->TextureTarget->GameThread_GetRenderTargetResource();
    if (!Resource)
    {
        return TEXT("");
    }

    TArray<FColor> Pixels;
    Pixels.SetNum(Width * Height);

    FReadSurfaceDataFlags ReadFlags(RCM_UNorm, CubeFace_MAX);
    ReadFlags.SetLinearToGamma(false);
    if (!Resource->ReadPixels(Pixels, ReadFlags))
    {
        return TEXT("");
    }

    FString Base64;
    return EncodeBgraToJpegBase64(Pixels, Width, Height, Quality, Base64) ? Base64 : TEXT("");
}

/**
 * @brief 以 AirSim 风格封装图像 JSON
 * @param Capture        场景捕获组件
 * @param SourceId       图像来源 ID
 * @param Width          图像宽度
 * @param Height         图像高度
 * @param FieldOfView    相机视场角
 * @param DefaultQuality 默认 JPEG 质量
 * @param ImageType      图像类型
 * @param Quality        请求 JPEG 质量
 * @param MaxDepthMeters 深度图最大距离
 * @return JSON 字符串
 */
FString CameraCaptureUtils::CaptureAirSimImageJson(
    USceneCaptureComponent2D* Capture,
    const FString& SourceId,
    int32 Width,
    int32 Height,
    float FieldOfView,
    int32 DefaultQuality,
    const FString& ImageType,
    int32 Quality,
    float MaxDepthMeters)
{
    AirSimImageUtils::EImageType ParsedType = AirSimImageUtils::EImageType::Scene;
    if (!AirSimImageUtils::TryParseImageType(ImageType, ParsedType))
    {
        return TEXT("{\"status\":\"error\",\"message\":\"Unsupported image_type\"}");
    }

    const FString Base64 = AirSimImageUtils::CaptureJpegBase64(
        Capture,
        Width,
        Height,
        ParsedType,
        ResolveJpegQuality(Quality, DefaultQuality),
        MaxDepthMeters);
    if (Base64.IsEmpty())
    {
        return TEXT("{\"status\":\"error\",\"message\":\"capture failed\"}");
    }

    const FVector CameraPosition = Capture ? Capture->GetComponentLocation() : FVector::ZeroVector;
    const FRotator CameraRotation = Capture ? Capture->GetComponentRotation() : FRotator::ZeroRotator;
    const FString CanonicalType = AirSimImageUtils::ToCanonicalString(ParsedType);

    return FString::Printf(
        TEXT("{\"status\":\"ok\",\"source\":\"%s\",\"image_type\":\"%s\",\"image_type_id\":%d,\"width\":%d,\"height\":%d,\"format\":\"jpeg\",\"camera_pos\":[%.2f,%.2f,%.2f],\"camera_rot\":[%.2f,%.2f,%.2f],\"fov\":%.2f,\"data\":\"%s\"}"),
        *EscapeJsonValue(SourceId),
        *CanonicalType,
        static_cast<int32>(ParsedType),
        Width,
        Height,
        CameraPosition.X,
        CameraPosition.Y,
        CameraPosition.Z,
        CameraRotation.Pitch,
        CameraRotation.Yaw,
        CameraRotation.Roll,
        FieldOfView,
        *Base64);
}

/**
 * @brief 应用语义分割 Stencil 值
 * @param Owner          目标 Actor
 * @param SegmentationId 模板值
 */
void CameraCaptureUtils::ApplySegmentationStencil(AActor* Owner, int32 SegmentationId)
{
    if (!Owner)
    {
        return;
    }

    // CustomDepth=3 表示启用并允许写入 Stencil，语义分割依赖该模式。
    if (IConsoleVariable* CustomDepthVar = IConsoleManager::Get().FindConsoleVariable(TEXT("r.CustomDepth")))
    {
        if (CustomDepthVar->GetInt() < 3)
        {
            CustomDepthVar->Set(3, ECVF_SetByCode);
        }
    }

    const int32 ClampedId = FMath::Clamp(SegmentationId, 0, 255);

    TArray<UPrimitiveComponent*> PrimitiveComponents;
    Owner->GetComponents<UPrimitiveComponent>(PrimitiveComponents);
    for (UPrimitiveComponent* Primitive : PrimitiveComponents)
    {
        if (!Primitive)
        {
            continue;
        }

        Primitive->SetRenderCustomDepth(true);
        Primitive->SetCustomDepthStencilValue(ClampedId);
    }
}

/**
 * @brief 同步 CineCamera 后处理到 SceneCapture
 * @param CineCamera         电影相机
 * @param Capture            场景捕获组件
 * @param ExposureBias       曝光偏置
 * @param bDisableMotionBlur 是否强制禁用动态模糊
 */
void CameraCaptureUtils::SyncPostProcessToCapture(
    UCineCameraComponent* CineCamera,
    USceneCaptureComponent2D* Capture,
    float ExposureBias,
    bool bDisableMotionBlur)
{
    if (!CineCamera || !Capture)
    {
        return;
    }

    FMinimalViewInfo ViewInfo;
    CineCamera->GetCameraView(0.0f, ViewInfo);

    // 保留已有 Blendables，避免直接覆盖后丢失附加后处理材质。
    const FWeightedBlendables SavedBlendables = Capture->PostProcessSettings.WeightedBlendables;
    Capture->PostProcessSettings = ViewInfo.PostProcessSettings;
    Capture->PostProcessSettings.WeightedBlendables = SavedBlendables;
    Capture->PostProcessSettings.bOverride_AutoExposureBias = true;
    Capture->PostProcessSettings.AutoExposureBias += ExposureBias;

    if (bDisableMotionBlur)
    {
        Capture->PostProcessSettings.bOverride_MotionBlurAmount = true;
        Capture->PostProcessSettings.MotionBlurAmount = 0.0f;
        Capture->PostProcessSettings.bOverride_MotionBlurMax = true;
        Capture->PostProcessSettings.MotionBlurMax = 0.0f;
        Capture->PostProcessSettings.bOverride_MotionBlurPerObjectSize = true;
        Capture->PostProcessSettings.MotionBlurPerObjectSize = 0.0f;
    }
}
#include "CameraCaptureUtils.h"

#include "Components/SceneCaptureComponent2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "IImageWrapper.h"
#include "IImageWrapperModule.h"
#include "Misc/Base64.h"
#include "Modules/ModuleManager.h"

namespace
{
    /**
     * @brief 将 BGRA 像素编码为 Base64 JPEG
     * @param Pixels 输入像素数组
     * @param Width 图像宽度
     * @param Height 图像高度
     * @param Quality JPEG 压缩质量
     * @param OutBase64 输出 Base64 字符串
     * @return 编码成功时返回 `true`
     */
    bool EncodeBgraToJpegBase64(const TArray<FColor>& Pixels, int32 Width, int32 Height, int32 Quality, FString& OutBase64)
    {
        if (Pixels.Num() != Width * Height)
        {
            return false;
        }

        TArray<uint8> RawData;
        RawData.SetNum(Pixels.Num() * 4);
        for (int32 i = 0; i < Pixels.Num(); ++i)
        {
            RawData[i * 4 + 0] = Pixels[i].B;
            RawData[i * 4 + 1] = Pixels[i].G;
            RawData[i * 4 + 2] = Pixels[i].R;
            RawData[i * 4 + 3] = Pixels[i].A;
        }

        IImageWrapperModule& ImgModule = FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));
        TSharedPtr<IImageWrapper> Wrapper = ImgModule.CreateImageWrapper(EImageFormat::JPEG);
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
 * @brief 捕获彩色场景图并编码为 Base64 JPEG
 * @param Capture 场景捕获组件
 * @param Width 输出宽度
 * @param Height 输出高度
 * @param Quality JPEG 压缩质量
 * @return Base64 编码后的 JPEG 字符串
 * 若捕获组件或 RenderTarget 无效，则直接返回空串。
 */
FString CameraCaptureUtils::CaptureColorJpegBase64(USceneCaptureComponent2D* Capture, int32 Width, int32 Height, int32 Quality)
{
    if (!Capture || !Capture->TextureTarget || Width <= 0 || Height <= 0)
    {
        return TEXT("");
    }

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

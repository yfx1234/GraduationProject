#include "ImageCapture.h"
#include "Engine/TextureRenderTarget2D.h"
#include "ImageUtils.h"
#include "Misc/Base64.h"
#include "IImageWrapperModule.h"
#include "IImageWrapper.h"

/** @brief 构造函数 */
UImageCapture::UImageCapture()
{
    PrimaryComponentTick.bCanEverTick = false;
}

/** @brief 游戏开始时调用，初始化采集组件 */
void UImageCapture::BeginPlay()
{
    Super::BeginPlay();
    SetupCapture();
}

/**
 * @brief 创建并配置 SceneCaptureComponent2D 和 RenderTarget
 * 创建 RenderTarget 画布，创建 SceneCaptureComponent2D 并附着到 Owner 根组件，
 */
void UImageCapture::SetupCapture()
{
    RenderTarget = NewObject<UTextureRenderTarget2D>(this);
    RenderTarget->InitAutoFormat(ImageWidth, ImageHeight);
    RenderTarget->ClearColor = FLinearColor::Black;
    CaptureComponent = NewObject<USceneCaptureComponent2D>(GetOwner());
    if (CaptureComponent)
    {
        CaptureComponent->RegisterComponent();
        CaptureComponent->AttachToComponent(
            GetOwner()->GetRootComponent(),
            FAttachmentTransformRules::KeepRelativeTransform);
        CaptureComponent->TextureTarget = RenderTarget;
        CaptureComponent->FOVAngle = CaptureFOV;
        CaptureComponent->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
        CaptureComponent->bCaptureEveryFrame = false;
        CaptureComponent->bCaptureOnMovement = false;
        UE_LOG(LogTemp, Log, TEXT("[ImageCapture] Setup %dx%d FOV=%.0f"),
            ImageWidth, ImageHeight, CaptureFOV);
    }
}

/**
 * @brief 采集一帧图像并编码为 JPEG
 * @param Quality JPEG 压缩质量
 * @return JPEG 编码后的字节数组
 */
TArray<uint8> UImageCapture::CaptureJpeg(int32 Quality)
{
    TArray<uint8> JpegData;
    if (!CaptureComponent || !RenderTarget) return JpegData;
    CaptureComponent->CaptureScene();
    FTextureRenderTargetResource* Resource = RenderTarget->GameThread_GetRenderTargetResource();
    if (!Resource) return JpegData;
    TArray<FColor> Pixels;
    Pixels.SetNum(ImageWidth * ImageHeight);
    FReadSurfaceDataFlags ReadFlags(RCM_UNorm);
    Resource->ReadPixels(Pixels, ReadFlags);
    IImageWrapperModule& ImageWrapperModule = FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));
    TSharedPtr<IImageWrapper> ImageWrapper = ImageWrapperModule.CreateImageWrapper(EImageFormat::JPEG);
    if (ImageWrapper.IsValid())
    {
        TArray<uint8> RawData;
        RawData.SetNum(Pixels.Num() * 4);
        for (int32 i = 0; i < Pixels.Num(); i++)
        {
            RawData[i * 4 + 0] = Pixels[i].B;
            RawData[i * 4 + 1] = Pixels[i].G;
            RawData[i * 4 + 2] = Pixels[i].R;
            RawData[i * 4 + 3] = Pixels[i].A;
        }
        if (ImageWrapper->SetRaw(RawData.GetData(), RawData.Num(), ImageWidth, ImageHeight, ERGBFormat::BGRA, 8))
            JpegData = ImageWrapper->GetCompressed(Quality);
    }
    return JpegData;
}

/**
 * @brief 采集一帧图像并返回 Base64 编码字符串
 * @param Quality JPEG 压缩质量
 * @return Base64 编码的 JPEG 字符串
 */
FString UImageCapture::CaptureBase64(int32 Quality)
{
    TArray<uint8> JpegData = CaptureJpeg(Quality);
    if (JpegData.Num() == 0) return TEXT("");
    return FBase64::Encode(JpegData);
}

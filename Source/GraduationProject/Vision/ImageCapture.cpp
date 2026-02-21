#include "ImageCapture.h"
#include "Engine/TextureRenderTarget2D.h"
#include "ImageUtils.h"
#include "Misc/Base64.h"
#include "IImageWrapperModule.h"
#include "IImageWrapper.h"

UImageCapture::UImageCapture()
{
    PrimaryComponentTick.bCanEverTick = false;
}

void UImageCapture::BeginPlay()
{
    Super::BeginPlay();
    SetupCapture();
}

void UImageCapture::SetupCapture()
{
    // 创建 RenderTarget
    RenderTarget = NewObject<UTextureRenderTarget2D>(this);
    RenderTarget->InitAutoFormat(ImageWidth, ImageHeight);
    RenderTarget->ClearColor = FLinearColor::Black;

    // 创建 SceneCaptureComponent2D
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
        CaptureComponent->bCaptureEveryFrame = false; // 手动触发
        CaptureComponent->bCaptureOnMovement = false;

        UE_LOG(LogTemp, Log, TEXT("[ImageCapture] Setup %dx%d FOV=%.0f"),
            ImageWidth, ImageHeight, CaptureFOV);
    }
}

TArray<uint8> UImageCapture::CaptureJpeg(int32 Quality)
{
    TArray<uint8> JpegData;

    if (!CaptureComponent || !RenderTarget) return JpegData;

    // 手动触发一次采集
    CaptureComponent->CaptureScene();

    // 读取像素数据
    FTextureRenderTargetResource* Resource = RenderTarget->GameThread_GetRenderTargetResource();
    if (!Resource) return JpegData;

    TArray<FColor> Pixels;
    Pixels.SetNum(ImageWidth * ImageHeight);
    FReadSurfaceDataFlags ReadFlags(RCM_UNorm);
    Resource->ReadPixels(Pixels, ReadFlags);

    // 编码为 JPEG
    IImageWrapperModule& ImageWrapperModule = FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));
    TSharedPtr<IImageWrapper> ImageWrapper = ImageWrapperModule.CreateImageWrapper(EImageFormat::JPEG);

    if (ImageWrapper.IsValid())
    {
        // FColor 数组转 raw BGRA
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
        {
            JpegData = ImageWrapper->GetCompressed(Quality);
        }
    }

    return JpegData;
}

FString UImageCapture::CaptureBase64(int32 Quality)
{
    TArray<uint8> JpegData = CaptureJpeg(Quality);
    if (JpegData.Num() == 0) return TEXT("");
    return FBase64::Encode(JpegData);
}

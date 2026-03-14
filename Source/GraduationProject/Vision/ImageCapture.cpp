// 解释：引入当前实现文件对应的头文件 `ImageCapture.h`，使实现部分能够看到类和函数声明。
#include "ImageCapture.h"

// 解释：引入 `TextureRenderTarget2D.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "Engine/TextureRenderTarget2D.h"
// 解释：引入 `ImageUtils.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "ImageUtils.h"
// 解释：引入 `Base64.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "Misc/Base64.h"
// 解释：引入 `IImageWrapperModule.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "IImageWrapperModule.h"
// 解释：引入 `IImageWrapper.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "IImageWrapper.h"

/** @brief 构造图像采集组件 */
// 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
UImageCapture::UImageCapture()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `PrimaryComponentTick.bCanEverTick`，完成 布尔标志 canevertick 的更新。
    PrimaryComponentTick.bCanEverTick = false;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 在游戏开始时初始化图像采集组件 */
// 解释：这一行定义函数 `BeginPlay`，开始实现beginplay的具体逻辑。
void UImageCapture::BeginPlay()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：调用 `BeginPlay` 执行当前步骤需要的功能逻辑。
    Super::BeginPlay();
    // 解释：调用 `SetupCapture` 执行当前步骤需要的功能逻辑。
    SetupCapture();
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 创建并配置采集组件与 RenderTarget
 * 创建专用 `RenderTarget`，并把 `SceneCaptureComponent2D` 挂到拥有者根节点上。
 */
// 解释：这一行定义函数 `SetupCapture`，开始实现setup采集的具体逻辑。
void UImageCapture::SetupCapture()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `RenderTarget`，完成 rendertarget 的更新。
    RenderTarget = NewObject<UTextureRenderTarget2D>(this);
    // 解释：调用 `InitAutoFormat` 执行当前步骤需要的功能逻辑。
    RenderTarget->InitAutoFormat(ImageWidth, ImageHeight);
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    RenderTarget->ClearColor = FLinearColor::Black;

    // 解释：这一行把右侧表达式的结果写入 `CaptureComponent`，完成 采集组件 的更新。
    CaptureComponent = NewObject<USceneCaptureComponent2D>(GetOwner());
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (CaptureComponent)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `RegisterComponent` 执行当前步骤需要的功能逻辑。
        CaptureComponent->RegisterComponent();
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        CaptureComponent->AttachToComponent(
            // 解释：这一行位于构造函数初始化列表中，把 `GetOwner` 直接初始化为 `)->GetRootComponent(`，减少进入函数体后的额外赋值开销。
            GetOwner()->GetRootComponent(),
            // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
            FAttachmentTransformRules::KeepRelativeTransform);

        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        CaptureComponent->TextureTarget = RenderTarget;
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        CaptureComponent->FOVAngle = CaptureFOV;
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        CaptureComponent->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        CaptureComponent->bCaptureEveryFrame = false;
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        CaptureComponent->bCaptureOnMovement = false;

        // 解释：这一行位于构造函数初始化列表中，把 `UE_LOG` 直接初始化为 `LogTemp, Log, TEXT("[ImageCapture] Setup %dx%d FOV=%.0f"`，减少进入函数体后的额外赋值开销。
        UE_LOG(LogTemp, Log, TEXT("[ImageCapture] Setup %dx%d FOV=%.0f"),
            // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
            ImageWidth, ImageHeight, CaptureFOV);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 采集一帧图像并编码为 JPEG
 * @param Quality JPEG 压缩质量
 * @return JPEG 字节数组
 */
// 解释：这一行定义函数 `CaptureJpeg`，开始实现采集jpeg的具体逻辑。
TArray<uint8> UImageCapture::CaptureJpeg(int32 Quality)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行声明成员或局部变量 `JpegData`，用于保存jpegdata。
    TArray<uint8> JpegData;
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!CaptureComponent || !RenderTarget)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return JpegData;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：调用 `CaptureScene` 执行当前步骤需要的功能逻辑。
    CaptureComponent->CaptureScene();

    // 解释：这一行把右侧表达式的结果写入 `FTextureRenderTargetResource* Resource`，完成 ftexturerendertargetresourceresource 的更新。
    FTextureRenderTargetResource* Resource = RenderTarget->GameThread_GetRenderTargetResource();
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!Resource)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return JpegData;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行声明成员或局部变量 `Pixels`，用于保存pixels。
    TArray<FColor> Pixels;
    // 解释：调用 `SetNum` 执行当前步骤需要的功能逻辑。
    Pixels.SetNum(ImageWidth * ImageHeight);

    // 解释：调用 `ReadFlags` 执行当前步骤需要的功能逻辑。
    FReadSurfaceDataFlags ReadFlags(RCM_UNorm);
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!Resource->ReadPixels(Pixels, ReadFlags))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return JpegData;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行把右侧表达式的结果写入 `IImageWrapperModule& ImageWrapperModule`，完成 iimagewrappermodule图像wrappermodule 的更新。
    IImageWrapperModule& ImageWrapperModule = FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));
    // 解释：调用 `CreateImageWrapper` 执行当前步骤需要的功能逻辑。
    TSharedPtr<IImageWrapper> ImageWrapper = ImageWrapperModule.CreateImageWrapper(EImageFormat::JPEG);
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!ImageWrapper.IsValid())
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return JpegData;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 将 `FColor` 数组重排为连续 BGRA 缓冲区，再交由 JPEG 编码器压缩。
    // 解释：这一行声明成员或局部变量 `RawData`，用于保存rawdata。
    TArray<uint8> RawData;
    // 解释：调用 `SetNum` 执行当前步骤需要的功能逻辑。
    RawData.SetNum(Pixels.Num() * 4);
    // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
    for (int32 i = 0; i < Pixels.Num(); ++i)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `RawData[i * 4 + 0]`，完成 rawdata 的更新。
        RawData[i * 4 + 0] = Pixels[i].B;
        // 解释：这一行把右侧表达式的结果写入 `RawData[i * 4 + 1]`，完成 rawdata 的更新。
        RawData[i * 4 + 1] = Pixels[i].G;
        // 解释：这一行把右侧表达式的结果写入 `RawData[i * 4 + 2]`，完成 rawdata 的更新。
        RawData[i * 4 + 2] = Pixels[i].R;
        // 解释：这一行把右侧表达式的结果写入 `RawData[i * 4 + 3]`，完成 rawdata 的更新。
        RawData[i * 4 + 3] = Pixels[i].A;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (ImageWrapper->SetRaw(RawData.GetData(), RawData.Num(), ImageWidth, ImageHeight, ERGBFormat::BGRA, 8))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `JpegData`，完成 jpegdata 的更新。
        JpegData = ImageWrapper->GetCompressed(Quality);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return JpegData;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 采集一帧图像并返回 Base64 编码结果
 * @param Quality JPEG 压缩质量
 * @return Base64 编码后的 JPEG 字符串
 */
// 解释：这一行定义函数 `CaptureBase64`，开始实现采集base64的具体逻辑。
FString UImageCapture::CaptureBase64(int32 Quality)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：调用 `CaptureJpeg` 执行当前步骤需要的功能逻辑。
    const TArray<uint8> JpegData = CaptureJpeg(Quality);
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (JpegData.Num() == 0)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return TEXT("");
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return FBase64::Encode(JpegData);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

// 解释：引入当前实现文件对应的头文件 `CameraCaptureUtils.h`，使实现部分能够看到类和函数声明。
#include "CameraCaptureUtils.h"

// 解释：引入 `AirSimImageUtils.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "AirSimImageUtils.h"
// 解释：引入 `CineCameraComponent.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "CineCameraComponent.h"
// 解释：引入 `PrimitiveComponent.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "Components/PrimitiveComponent.h"
// 解释：引入 `SceneCaptureComponent2D.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "Components/SceneCaptureComponent2D.h"
// 解释：引入 `TextureRenderTarget2D.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "Engine/TextureRenderTarget2D.h"
// 解释：引入 `Actor.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "GameFramework/Actor.h"
// 解释：引入 `IConsoleManager.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "HAL/IConsoleManager.h"
// 解释：引入 `IImageWrapper.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "IImageWrapper.h"
// 解释：引入 `IImageWrapperModule.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "IImageWrapperModule.h"
// 解释：引入 `Base64.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "Misc/Base64.h"
// 解释：引入 `ModuleManager.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "Modules/ModuleManager.h"

// 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
namespace
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    /**
     * @brief 转义 JSON 中的字符串字段
     * @param In 原始字符串
     * @return 适合直接嵌入 JSON 的转义结果
     */
    // 解释：这一行定义函数 `EscapeJsonValue`，开始实现escapejsonvalue的具体逻辑。
    FString EscapeJsonValue(const FString& In)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行声明成员或局部变量 `Out`，用于保存out。
        FString Out = In;
        // 解释：调用 `ReplaceInline` 执行当前步骤需要的功能逻辑。
        Out.ReplaceInline(TEXT("\\"), TEXT("\\\\"));
        // 解释：调用 `ReplaceInline` 执行当前步骤需要的功能逻辑。
        Out.ReplaceInline(TEXT("\""), TEXT("\\\""));
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return Out;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    /**
     * @brief 解析最终 JPEG 质量
     * @param Quality        外部传入质量
     * @param DefaultQuality 默认质量
     * @return 合法范围 [1, 100] 内的最终质量
     */
    // 解释：这一行定义函数 `ResolveJpegQuality`，开始实现resolvejpegquality的具体逻辑。
    int32 ResolveJpegQuality(int32 Quality, int32 DefaultQuality)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行使用三目表达式选择返回值，在正常路径和兜底路径之间做快速切换。
        return (Quality > 0) ? Quality : FMath::Clamp(DefaultQuality, 1, 100);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
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
    // 解释：这一行定义函数 `EncodeBgraToJpegBase64`，开始实现encodebgratojpegbase64的具体逻辑。
    bool EncodeBgraToJpegBase64(const TArray<FColor>& Pixels, int32 Width, int32 Height, int32 Quality, FString& OutBase64)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (Pixels.Num() != Width * Height || Width <= 0 || Height <= 0)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return false;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // `FColor` 以 BGRA 顺序存储，这里显式展开为连续缓冲区供 ImageWrapper 压缩。
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

        // 解释：这一行把右侧表达式的结果写入 `IImageWrapperModule& ImageWrapperModule`，完成 iimagewrappermodule图像wrappermodule 的更新。
        IImageWrapperModule& ImageWrapperModule = FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));
        // 解释：调用 `CreateImageWrapper` 执行当前步骤需要的功能逻辑。
        TSharedPtr<IImageWrapper> Wrapper = ImageWrapperModule.CreateImageWrapper(EImageFormat::JPEG);
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (!Wrapper.IsValid())
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return false;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (!Wrapper->SetRaw(RawData.GetData(), RawData.Num(), Width, Height, ERGBFormat::BGRA, 8))
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return false;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：调用 `GetCompressed` 执行当前步骤需要的功能逻辑。
        const TArray64<uint8>& JpegData = Wrapper->GetCompressed(FMath::Clamp(Quality, 1, 100));
        // 解释：这一行把右侧表达式的结果写入 `OutBase64`，完成 outbase64 的更新。
        OutBase64 = FBase64::Encode(JpegData.GetData(), JpegData.Num());
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return true;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 创建彩色 RenderTarget
 * @param Owner  RenderTarget 的 Outer
 * @param Width  宽度
 * @param Height 高度
 * @return 新建 RenderTarget；失败时返回 nullptr
 */
// 解释：这一行定义函数 `CreateColorRenderTarget`，开始实现createcolorrendertarget的具体逻辑。
UTextureRenderTarget2D* CameraCaptureUtils::CreateColorRenderTarget(UObject* Owner, int32 Width, int32 Height)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (Width <= 0 || Height <= 0)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return nullptr;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行把右侧表达式的结果写入 `UObject* Outer`，完成 uobjectouter 的更新。
    UObject* Outer = Owner ? Owner : GetTransientPackage();
    // 解释：这一行把右侧表达式的结果写入 `UTextureRenderTarget2D* RenderTarget`，完成 utexturerendertarget2Drendertarget 的更新。
    UTextureRenderTarget2D* RenderTarget = NewObject<UTextureRenderTarget2D>(Outer);
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!RenderTarget)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return nullptr;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：调用 `InitCustomFormat` 执行当前步骤需要的功能逻辑。
    RenderTarget->InitCustomFormat(Width, Height, PF_B8G8R8A8, false);
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    RenderTarget->ClearColor = FLinearColor::Black;
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return RenderTarget;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 抓取一帧彩色图像并编码为 Base64 JPEG
 * @param Capture 场景捕获组件
 * @param Width   输出宽度
 * @param Height  输出高度
 * @param Quality JPEG 质量
 * @return Base64 字符串；失败时返回空串
 */
// 解释：这一行定义函数 `CaptureColorJpegBase64`，开始实现采集colorjpegbase64的具体逻辑。
FString CameraCaptureUtils::CaptureColorJpegBase64(USceneCaptureComponent2D* Capture, int32 Width, int32 Height, int32 Quality)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!Capture || !Capture->TextureTarget || Width <= 0 || Height <= 0)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return TEXT("");
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 若关闭了逐帧捕获，则手动触发一次渲染，确保读取到最新图像。
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!Capture->bCaptureEveryFrame)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `CaptureScene` 执行当前步骤需要的功能逻辑。
        Capture->CaptureScene();
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行把右侧表达式的结果写入 `FTextureRenderTargetResource* Resource`，完成 ftexturerendertargetresourceresource 的更新。
    FTextureRenderTargetResource* Resource = Capture->TextureTarget->GameThread_GetRenderTargetResource();
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!Resource)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return TEXT("");
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行声明成员或局部变量 `Pixels`，用于保存pixels。
    TArray<FColor> Pixels;
    // 解释：调用 `SetNum` 执行当前步骤需要的功能逻辑。
    Pixels.SetNum(Width * Height);

    // 解释：调用 `ReadFlags` 执行当前步骤需要的功能逻辑。
    FReadSurfaceDataFlags ReadFlags(RCM_UNorm, CubeFace_MAX);
    // 解释：调用 `SetLinearToGamma` 执行当前步骤需要的功能逻辑。
    ReadFlags.SetLinearToGamma(false);
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!Resource->ReadPixels(Pixels, ReadFlags))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return TEXT("");
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行声明成员或局部变量 `Base64`，用于保存base64。
    FString Base64;
    // 解释：这一行使用三目表达式选择返回值，在正常路径和兜底路径之间做快速切换。
    return EncodeBgraToJpegBase64(Pixels, Width, Height, Quality, Base64) ? Base64 : TEXT("");
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
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
// 解释：这一行定义函数 `CaptureAirSimImageJson`，开始实现采集空中仿真图像json的具体逻辑。
FString CameraCaptureUtils::CaptureAirSimImageJson(
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
    // 解释：这一行收束函数 `CaptureAirSimImageJson` 的签名，后面会进入实现体或以分号结束声明。
    float MaxDepthMeters)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行声明成员或局部变量 `ParsedType`，用于保存parsedtype。
    AirSimImageUtils::EImageType ParsedType = AirSimImageUtils::EImageType::Scene;
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!AirSimImageUtils::TryParseImageType(ImageType, ParsedType))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return TEXT("{\"status\":\"error\",\"message\":\"Unsupported image_type\"}");
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    const FString Base64 = AirSimImageUtils::CaptureJpegBase64(
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Capture,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Width,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Height,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        ParsedType,
        // 解释：这一行位于构造函数初始化列表中，把 `ResolveJpegQuality` 直接初始化为 `Quality, DefaultQuality`，减少进入函数体后的额外赋值开销。
        ResolveJpegQuality(Quality, DefaultQuality),
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        MaxDepthMeters);
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (Base64.IsEmpty())
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return TEXT("{\"status\":\"error\",\"message\":\"capture failed\"}");
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行把右侧表达式的结果写入 `const FVector CameraPosition`，完成 constfvector相机position 的更新。
    const FVector CameraPosition = Capture ? Capture->GetComponentLocation() : FVector::ZeroVector;
    // 解释：这一行把右侧表达式的结果写入 `const FRotator CameraRotation`，完成 constfrotator相机rotation 的更新。
    const FRotator CameraRotation = Capture ? Capture->GetComponentRotation() : FRotator::ZeroRotator;
    // 解释：这一行把右侧表达式的结果写入 `const FString CanonicalType`，完成 constfstringcanonicaltype 的更新。
    const FString CanonicalType = AirSimImageUtils::ToCanonicalString(ParsedType);

    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return FString::Printf(
        // 解释：这一行位于构造函数初始化列表中，把 `TEXT` 直接初始化为 `"{\"status\":\"ok\",\"source\":\"%s\",\"image_type\":\"%s\",\"image_type_id\":%d,\"width\":%d,\"height\":%d,\"format\":\"jpeg\",\"camera_pos\":[%.2f,%.2f,%.2f],\"camera_rot\":[%.2f,%.2f,%.2f],\"fov\":%.2f,\"data\":\"%s\"}"`，减少进入函数体后的额外赋值开销。
        TEXT("{\"status\":\"ok\",\"source\":\"%s\",\"image_type\":\"%s\",\"image_type_id\":%d,\"width\":%d,\"height\":%d,\"format\":\"jpeg\",\"camera_pos\":[%.2f,%.2f,%.2f],\"camera_rot\":[%.2f,%.2f,%.2f],\"fov\":%.2f,\"data\":\"%s\"}"),
        *EscapeJsonValue(SourceId),
        *CanonicalType,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        static_cast<int32>(ParsedType),
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Width,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Height,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        CameraPosition.X,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        CameraPosition.Y,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        CameraPosition.Z,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        CameraRotation.Pitch,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        CameraRotation.Yaw,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        CameraRotation.Roll,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        FieldOfView,
        *Base64);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 应用语义分割 Stencil 值
 * @param Owner          目标 Actor
 * @param SegmentationId 模板值
 */
// 解释：这一行定义函数 `ApplySegmentationStencil`，开始实现applysegmentationstencil的具体逻辑。
void CameraCaptureUtils::ApplySegmentationStencil(AActor* Owner, int32 SegmentationId)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!Owner)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // CustomDepth=3 表示启用并允许写入 Stencil，语义分割依赖该模式。
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (IConsoleVariable* CustomDepthVar = IConsoleManager::Get().FindConsoleVariable(TEXT("r.CustomDepth")))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (CustomDepthVar->GetInt() < 3)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：调用 `Set` 执行当前步骤需要的功能逻辑。
            CustomDepthVar->Set(3, ECVF_SetByCode);
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行先对计算结果做限幅，再写入 `const int32 ClampedId`，防止 constint32clampedid 超出允许范围。
    const int32 ClampedId = FMath::Clamp(SegmentationId, 0, 255);

    // 解释：这一行声明成员或局部变量 `PrimitiveComponents`，用于保存primitivecomponents。
    TArray<UPrimitiveComponent*> PrimitiveComponents;
    // 解释：调用 `UPrimitiveComponent>` 执行当前步骤需要的功能逻辑。
    Owner->GetComponents<UPrimitiveComponent>(PrimitiveComponents);
    // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
    for (UPrimitiveComponent* Primitive : PrimitiveComponents)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (!Primitive)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行跳过本轮循环剩余语句，直接进入下一轮迭代。
            continue;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：调用 `SetRenderCustomDepth` 执行当前步骤需要的功能逻辑。
        Primitive->SetRenderCustomDepth(true);
        // 解释：调用 `SetCustomDepthStencilValue` 执行当前步骤需要的功能逻辑。
        Primitive->SetCustomDepthStencilValue(ClampedId);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 同步 CineCamera 后处理到 SceneCapture
 * @param CineCamera         电影相机
 * @param Capture            场景捕获组件
 * @param ExposureBias       曝光偏置
 * @param bDisableMotionBlur 是否强制禁用动态模糊
 */
// 解释：这一行定义函数 `SyncPostProcessToCapture`，开始实现syncpostprocessto采集的具体逻辑。
void CameraCaptureUtils::SyncPostProcessToCapture(
    // 解释：这一行继续展开 `SyncPostProcessToCapture` 的参数列表，声明参数 `CineCamera` 用于传入cine相机。
    UCineCameraComponent* CineCamera,
    // 解释：这一行继续展开 `SyncPostProcessToCapture` 的参数列表，声明参数 `Capture` 用于传入采集。
    USceneCaptureComponent2D* Capture,
    // 解释：这一行继续展开 `SyncPostProcessToCapture` 的参数列表，声明参数 `ExposureBias` 用于传入exposurebias。
    float ExposureBias,
    // 解释：这一行收束函数 `SyncPostProcessToCapture` 的签名，后面会进入实现体或以分号结束声明。
    bool bDisableMotionBlur)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!CineCamera || !Capture)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行声明成员或局部变量 `ViewInfo`，用于保存viewinfo。
    FMinimalViewInfo ViewInfo;
    // 解释：调用 `GetCameraView` 执行当前步骤需要的功能逻辑。
    CineCamera->GetCameraView(0.0f, ViewInfo);

    // 保留已有 Blendables，避免直接覆盖后丢失附加后处理材质。
    // 解释：这一行声明成员或局部变量 `SavedBlendables`，用于保存savedblendables。
    const FWeightedBlendables SavedBlendables = Capture->PostProcessSettings.WeightedBlendables;
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    Capture->PostProcessSettings = ViewInfo.PostProcessSettings;
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    Capture->PostProcessSettings.WeightedBlendables = SavedBlendables;
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    Capture->PostProcessSettings.bOverride_AutoExposureBias = true;
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    Capture->PostProcessSettings.AutoExposureBias += ExposureBias;

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (bDisableMotionBlur)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Capture->PostProcessSettings.bOverride_MotionBlurAmount = true;
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Capture->PostProcessSettings.MotionBlurAmount = 0.0f;
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Capture->PostProcessSettings.bOverride_MotionBlurMax = true;
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Capture->PostProcessSettings.MotionBlurMax = 0.0f;
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Capture->PostProcessSettings.bOverride_MotionBlurPerObjectSize = true;
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Capture->PostProcessSettings.MotionBlurPerObjectSize = 0.0f;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

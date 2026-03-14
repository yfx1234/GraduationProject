// 解释：引入当前实现文件对应的头文件 `AirSimImageUtils.h`，使实现部分能够看到类和函数声明。
#include "AirSimImageUtils.h"

// 解释：引入 `PrimitiveComponent.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "Components/PrimitiveComponent.h"
// 解释：引入 `SceneCaptureComponent2D.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "Components/SceneCaptureComponent2D.h"
// 解释：引入 `Level.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "Engine/Level.h"
// 解释：引入 `TextureRenderTarget2D.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "Engine/TextureRenderTarget2D.h"
// 解释：引入 `World.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "Engine/World.h"
// 解释：引入 `Actor.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "GameFramework/Actor.h"
// 解释：引入 `IConsoleManager.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "HAL/IConsoleManager.h"
// 解释：引入 `IImageWrapper.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "IImageWrapper.h"
// 解释：引入 `IImageWrapperModule.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "IImageWrapperModule.h"
// 解释：引入 `MaterialInterface.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "Materials/MaterialInterface.h"
// 解释：引入 `Base64.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "Misc/Base64.h"
// 解释：引入 `ModuleManager.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "Modules/ModuleManager.h"
// 解释：引入 `StrongObjectPtr.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "UObject/StrongObjectPtr.h"

// 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
namespace
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行声明 结构体 `FDepthRtCacheEntry`，用于封装fdepthrtcacheentry相关的数据与行为。
    struct FDepthRtCacheEntry
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行声明成员或局部变量 `RenderTarget`，用于保存rendertarget。
        TStrongObjectPtr<UTextureRenderTarget2D> RenderTarget;
        // 解释：这一行声明成员或局部变量 `Width`，用于保存width。
        int32 Width = 0;
        // 解释：这一行声明成员或局部变量 `Height`，用于保存height。
        int32 Height = 0;
        // 解释：这一行声明成员或局部变量 `LastUsedSerial`，用于保存lastusedserial。
        uint64 LastUsedSerial = 0;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    };

    // 解释：这一行声明 结构体 `FColorRtCacheEntry`，用于封装fcolorrtcacheentry相关的数据与行为。
    struct FColorRtCacheEntry
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行声明成员或局部变量 `RenderTarget`，用于保存rendertarget。
        TStrongObjectPtr<UTextureRenderTarget2D> RenderTarget;
        // 解释：这一行声明成员或局部变量 `Width`，用于保存width。
        int32 Width = 0;
        // 解释：这一行声明成员或局部变量 `Height`，用于保存height。
        int32 Height = 0;
        // 解释：这一行声明成员或局部变量 `bForceLinearGamma`，用于保存布尔标志 forcelineargamma。
        bool bForceLinearGamma = false;
        // 解释：这一行声明成员或局部变量 `LastUsedSerial`，用于保存lastusedserial。
        uint64 LastUsedSerial = 0;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    };

    // 解释：这一行声明成员或局部变量 `GDepthRtCache`，用于保存gdepthrtcache。
    TArray<FDepthRtCacheEntry> GDepthRtCache;
    // 解释：这一行声明成员或局部变量 `GDepthRtUseSerial`，用于保存gdepthrtuseserial。
    uint64 GDepthRtUseSerial = 0;
    // 解释：这一行声明成员或局部变量 `kMaxDepthRtCacheEntries`，用于保存Kmaxdepthrtcacheentries。
    constexpr int32 kMaxDepthRtCacheEntries = 4;

    // 解释：这一行声明成员或局部变量 `GColorRtCache`，用于保存gcolorrtcache。
    TArray<FColorRtCacheEntry> GColorRtCache;
    // 解释：这一行声明成员或局部变量 `GColorRtUseSerial`，用于保存gcolorrtuseserial。
    uint64 GColorRtUseSerial = 0;
    // 解释：这一行声明成员或局部变量 `kMaxColorRtCacheEntries`，用于保存Kmaxcolorrtcacheentries。
    constexpr int32 kMaxColorRtCacheEntries = 6;

    // 解释：这一行声明成员或局部变量 `GLastSegmentationWorld`，用于保存glastsegmentationworld。
    TWeakObjectPtr<UWorld> GLastSegmentationWorld;
    // 解释：这一行声明成员或局部变量 `GLastSegmentationUpdateTimeSec`，用于保存glastsegmentationupdatetimesec。
    double GLastSegmentationUpdateTimeSec = -1000.0;
    // 解释：这一行声明成员或局部变量 `kSegmentationRefreshIntervalSec`，用于保存Ksegmentationrefreshintervalsec。
    constexpr double kSegmentationRefreshIntervalSec = 1.0;

    // 解释：这一行声明成员或局部变量 `kDepthVisRangeMeters`，用于保存Kdepthvisrangemeters。
    constexpr float kDepthVisRangeMeters = 100.0f;
    // 解释：这一行把右侧表达式的结果写入 `constexpr TCHAR kAirSimDepthPlanarMaterialPath[]`，完成 constexprtcharK空中仿真depthplanarmaterialpath 的更新。
    constexpr TCHAR kAirSimDepthPlanarMaterialPath[] = TEXT("/Game/AirSimHUDAssets/DepthPlanarMaterial.DepthPlanarMaterial");
    // 解释：这一行把右侧表达式的结果写入 `constexpr TCHAR kAirSimDepthVisMaterialPath[]`，完成 constexprtcharK空中仿真depthvismaterialpath 的更新。
    constexpr TCHAR kAirSimDepthVisMaterialPath[] = TEXT("/Game/AirSimHUDAssets/DepthVisMaterial.DepthVisMaterial");
    // 解释：这一行把右侧表达式的结果写入 `constexpr TCHAR kAirSimSegmentationMaterialPath[]`，完成 constexprtcharK空中仿真segmentationmaterialpath 的更新。
    constexpr TCHAR kAirSimSegmentationMaterialPath[] = TEXT("/Game/AirSimHUDAssets/SegmentationMaterial.SegmentationMaterial");
    // 解释：这一行把右侧表达式的结果写入 `constexpr TCHAR kAirSimInfraredMaterialPath[]`，完成 constexprtcharK空中仿真infraredmaterialpath 的更新。
    constexpr TCHAR kAirSimInfraredMaterialPath[] = TEXT("/Game/AirSimHUDAssets/InfraredMaterial.InfraredMaterial");

    // 解释：这一行定义函数 `ResizePixelsNearest`，开始实现resizepixelsnearest的具体逻辑。
    bool ResizePixelsNearest(
        // 解释：这一行继续展开 `ResizePixelsNearest` 的参数列表，声明参数 `SourcePixels` 用于传入sourcepixels。
        const TArray<FColor>& SourcePixels,
        // 解释：这一行继续展开 `ResizePixelsNearest` 的参数列表，声明参数 `SourceWidth` 用于传入sourcewidth。
        int32 SourceWidth,
        // 解释：这一行继续展开 `ResizePixelsNearest` 的参数列表，声明参数 `SourceHeight` 用于传入sourceheight。
        int32 SourceHeight,
        // 解释：这一行继续展开 `ResizePixelsNearest` 的参数列表，声明参数 `TargetWidth` 用于传入targetwidth。
        int32 TargetWidth,
        // 解释：这一行继续展开 `ResizePixelsNearest` 的参数列表，声明参数 `TargetHeight` 用于传入targetheight。
        int32 TargetHeight,
        // 解释：这一行收束函数 `ResizePixelsNearest` 的签名，后面会进入实现体或以分号结束声明。
        TArray<FColor>& OutPixels)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (SourcePixels.Num() != SourceWidth * SourceHeight ||
            // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
            SourceWidth <= 0 || SourceHeight <= 0 ||
            // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
            TargetWidth <= 0 || TargetHeight <= 0)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return false;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (SourceWidth == TargetWidth && SourceHeight == TargetHeight)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行把右侧表达式的结果写入 `OutPixels`，完成 outpixels 的更新。
            OutPixels = SourcePixels;
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return true;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：调用 `SetNumUninitialized` 执行当前步骤需要的功能逻辑。
        OutPixels.SetNumUninitialized(TargetWidth * TargetHeight);
        // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
        for (int32 Y = 0; Y < TargetHeight; ++Y)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行先对计算结果做限幅，再写入 `const int32 SrcY`，防止 constint32srcY 超出允许范围。
            const int32 SrcY = FMath::Clamp((Y * SourceHeight) / TargetHeight, 0, SourceHeight - 1);
            // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
            for (int32 X = 0; X < TargetWidth; ++X)
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            {
                // 解释：这一行先对计算结果做限幅，再写入 `const int32 SrcX`，防止 constint32srcX 超出允许范围。
                const int32 SrcX = FMath::Clamp((X * SourceWidth) / TargetWidth, 0, SourceWidth - 1);
                // 解释：这一行把右侧表达式的结果写入 `OutPixels[Y * TargetWidth + X]`，完成 outpixels 的更新。
                OutPixels[Y * TargetWidth + X] = SourcePixels[SrcY * SourceWidth + SrcX];
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            }
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return true;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行定义函数 `AcquireDepthRenderTarget`，开始实现acquiredepthrendertarget的具体逻辑。
    UTextureRenderTarget2D* AcquireDepthRenderTarget(int32 Width, int32 Height)
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

        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        ++GDepthRtUseSerial;

        // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
        for (int32 Index = GDepthRtCache.Num() - 1; Index >= 0; --Index)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
            if (!GDepthRtCache[Index].RenderTarget.IsValid())
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            {
                // 解释：调用 `RemoveAtSwap` 执行当前步骤需要的功能逻辑。
                GDepthRtCache.RemoveAtSwap(Index);
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            }
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
        for (FDepthRtCacheEntry& Entry : GDepthRtCache)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
            if (Entry.Width == Width && Entry.Height == Height && Entry.RenderTarget.IsValid())
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            {
                // 解释：这一行把右侧表达式的结果写入 `Entry.LastUsedSerial`，完成 lastusedserial 的更新。
                Entry.LastUsedSerial = GDepthRtUseSerial;
                // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
                return Entry.RenderTarget.Get();
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            }
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (GDepthRtCache.Num() >= kMaxDepthRtCacheEntries)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行声明成员或局部变量 `OldestIndex`，用于保存oldestindex。
            int32 OldestIndex = 0;
            // 解释：这一行把右侧表达式的结果写入 `uint64 OldestSerial`，完成 uint64oldestserial 的更新。
            uint64 OldestSerial = ~static_cast<uint64>(0);
            // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
            for (int32 Index = 0; Index < GDepthRtCache.Num(); ++Index)
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            {
                // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
                if (GDepthRtCache[Index].LastUsedSerial < OldestSerial)
                // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
                {
                    // 解释：这一行把右侧表达式的结果写入 `OldestSerial`，完成 oldestserial 的更新。
                    OldestSerial = GDepthRtCache[Index].LastUsedSerial;
                    // 解释：这一行把右侧表达式的结果写入 `OldestIndex`，完成 oldestindex 的更新。
                    OldestIndex = Index;
                // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
                }
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            }
            // 解释：调用 `RemoveAtSwap` 执行当前步骤需要的功能逻辑。
            GDepthRtCache.RemoveAtSwap(OldestIndex);
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行把右侧表达式的结果写入 `UTextureRenderTarget2D* DepthRT`，完成 utexturerendertarget2Ddepthrt 的更新。
        UTextureRenderTarget2D* DepthRT = NewObject<UTextureRenderTarget2D>(GetTransientPackage(), NAME_None, RF_Transient);
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (!DepthRT)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return nullptr;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：调用 `InitCustomFormat` 执行当前步骤需要的功能逻辑。
        DepthRT->InitCustomFormat(Width, Height, PF_FloatRGBA, false);
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        DepthRT->ClearColor = FLinearColor::Black;

        // 解释：这一行声明成员或局部变量 `NewEntry`，用于保存newentry。
        FDepthRtCacheEntry NewEntry;
        // 解释：调用 `Reset` 执行当前步骤需要的功能逻辑。
        NewEntry.RenderTarget.Reset(DepthRT);
        // 解释：这一行把右侧表达式的结果写入 `NewEntry.Width`，完成 width 的更新。
        NewEntry.Width = Width;
        // 解释：这一行把右侧表达式的结果写入 `NewEntry.Height`，完成 height 的更新。
        NewEntry.Height = Height;
        // 解释：这一行把右侧表达式的结果写入 `NewEntry.LastUsedSerial`，完成 lastusedserial 的更新。
        NewEntry.LastUsedSerial = GDepthRtUseSerial;
        // 解释：调用 `Add` 执行当前步骤需要的功能逻辑。
        GDepthRtCache.Add(MoveTemp(NewEntry));

        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return DepthRT;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行定义函数 `AcquireColorRenderTarget`，开始实现acquirecolorrendertarget的具体逻辑。
    UTextureRenderTarget2D* AcquireColorRenderTarget(int32 Width, int32 Height, bool bForceLinearGamma = false)
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

        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        ++GColorRtUseSerial;

        // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
        for (int32 Index = GColorRtCache.Num() - 1; Index >= 0; --Index)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
            if (!GColorRtCache[Index].RenderTarget.IsValid())
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            {
                // 解释：调用 `RemoveAtSwap` 执行当前步骤需要的功能逻辑。
                GColorRtCache.RemoveAtSwap(Index);
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            }
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
        for (FColorRtCacheEntry& Entry : GColorRtCache)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
            if (Entry.Width == Width &&
                // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                Entry.Height == Height &&
                // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                Entry.bForceLinearGamma == bForceLinearGamma &&
                // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                Entry.RenderTarget.IsValid())
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            {
                // 解释：这一行把右侧表达式的结果写入 `Entry.LastUsedSerial`，完成 lastusedserial 的更新。
                Entry.LastUsedSerial = GColorRtUseSerial;
                // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
                return Entry.RenderTarget.Get();
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            }
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (GColorRtCache.Num() >= kMaxColorRtCacheEntries)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行声明成员或局部变量 `OldestIndex`，用于保存oldestindex。
            int32 OldestIndex = 0;
            // 解释：这一行把右侧表达式的结果写入 `uint64 OldestSerial`，完成 uint64oldestserial 的更新。
            uint64 OldestSerial = ~static_cast<uint64>(0);
            // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
            for (int32 Index = 0; Index < GColorRtCache.Num(); ++Index)
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            {
                // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
                if (GColorRtCache[Index].LastUsedSerial < OldestSerial)
                // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
                {
                    // 解释：这一行把右侧表达式的结果写入 `OldestSerial`，完成 oldestserial 的更新。
                    OldestSerial = GColorRtCache[Index].LastUsedSerial;
                    // 解释：这一行把右侧表达式的结果写入 `OldestIndex`，完成 oldestindex 的更新。
                    OldestIndex = Index;
                // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
                }
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            }
            // 解释：调用 `RemoveAtSwap` 执行当前步骤需要的功能逻辑。
            GColorRtCache.RemoveAtSwap(OldestIndex);
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行把右侧表达式的结果写入 `UTextureRenderTarget2D* ColorRT`，完成 utexturerendertarget2Dcolorrt 的更新。
        UTextureRenderTarget2D* ColorRT = NewObject<UTextureRenderTarget2D>(GetTransientPackage(), NAME_None, RF_Transient);
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (!ColorRT)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return nullptr;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：调用 `InitCustomFormat` 执行当前步骤需要的功能逻辑。
        ColorRT->InitCustomFormat(Width, Height, PF_B8G8R8A8, bForceLinearGamma);
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        ColorRT->ClearColor = FLinearColor::Black;

        // 解释：这一行声明成员或局部变量 `NewEntry`，用于保存newentry。
        FColorRtCacheEntry NewEntry;
        // 解释：调用 `Reset` 执行当前步骤需要的功能逻辑。
        NewEntry.RenderTarget.Reset(ColorRT);
        // 解释：这一行把右侧表达式的结果写入 `NewEntry.Width`，完成 width 的更新。
        NewEntry.Width = Width;
        // 解释：这一行把右侧表达式的结果写入 `NewEntry.Height`，完成 height 的更新。
        NewEntry.Height = Height;
        // 解释：这一行把右侧表达式的结果写入 `NewEntry.bForceLinearGamma`，完成 布尔标志 forcelineargamma 的更新。
        NewEntry.bForceLinearGamma = bForceLinearGamma;
        // 解释：这一行把右侧表达式的结果写入 `NewEntry.LastUsedSerial`，完成 lastusedserial 的更新。
        NewEntry.LastUsedSerial = GColorRtUseSerial;
        // 解释：调用 `Add` 执行当前步骤需要的功能逻辑。
        GColorRtCache.Add(MoveTemp(NewEntry));

        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return ColorRT;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行定义函数 `CaptureWithPostProcessMaterial`，开始实现采集withpostprocessmaterial的具体逻辑。
    bool CaptureWithPostProcessMaterial(
        // 解释：这一行继续展开 `CaptureWithPostProcessMaterial` 的参数列表，声明参数 `Capture` 用于传入采集。
        USceneCaptureComponent2D* Capture,
        // 解释：这一行继续展开 `CaptureWithPostProcessMaterial` 的参数列表，声明参数 `Width` 用于传入width。
        int32 Width,
        // 解释：这一行继续展开 `CaptureWithPostProcessMaterial` 的参数列表，声明参数 `Height` 用于传入height。
        int32 Height,
        // 解释：这一行继续展开 `CaptureWithPostProcessMaterial` 的参数列表，声明参数 `PostMaterial` 用于传入postmaterial。
        UMaterialInterface* PostMaterial,
        // 解释：这一行继续展开 `CaptureWithPostProcessMaterial` 的参数列表，声明参数 `bForceLinearGamma` 用于传入布尔标志 forcelineargamma。
        bool bForceLinearGamma,
        // 解释：这一行收束函数 `CaptureWithPostProcessMaterial` 的签名，后面会进入实现体或以分号结束声明。
        TArray<FColor>& OutPixels)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (!Capture || !PostMaterial || Width <= 0 || Height <= 0)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return false;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行把右侧表达式的结果写入 `UTextureRenderTarget2D* CaptureRT`，完成 utexturerendertarget2D采集rt 的更新。
        UTextureRenderTarget2D* CaptureRT = AcquireColorRenderTarget(Width, Height, bForceLinearGamma);
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (!CaptureRT)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return false;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行声明成员或局部变量 `OriginalTarget`，用于保存originaltarget。
        UTextureRenderTarget2D* OriginalTarget = Capture->TextureTarget;
        // 解释：这一行声明成员或局部变量 `OriginalSource`，用于保存originalsource。
        const ESceneCaptureSource OriginalSource = Capture->CaptureSource;
        // 解释：这一行声明成员或局部变量 `bOriginalEveryFrame`，用于保存布尔标志 originaleveryframe。
        const bool bOriginalEveryFrame = Capture->bCaptureEveryFrame;
        // 解释：这一行声明成员或局部变量 `OriginalPostProcess`，用于保存originalpostprocess。
        const FPostProcessSettings OriginalPostProcess = Capture->PostProcessSettings;
        // 解释：这一行声明成员或局部变量 `OriginalPostProcessBlendWeight`，用于保存originalpostprocessblendweight。
        const float OriginalPostProcessBlendWeight = Capture->PostProcessBlendWeight;

        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Capture->TextureTarget = CaptureRT;
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Capture->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Capture->bCaptureEveryFrame = false;
        // 解释：调用 `FPostProcessSettings` 执行当前步骤需要的功能逻辑。
        Capture->PostProcessSettings = FPostProcessSettings();
        // 解释：调用 `AddBlendable` 执行当前步骤需要的功能逻辑。
        Capture->PostProcessSettings.AddBlendable(PostMaterial, 1.0f);
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Capture->PostProcessBlendWeight = 1.0f;
        // 解释：调用 `CaptureScene` 执行当前步骤需要的功能逻辑。
        Capture->CaptureScene();

        // 解释：这一行声明成员或局部变量 `Pixels`，用于保存pixels。
        TArray<FColor> Pixels;
        // 解释：这一行声明成员或局部变量 `bReadOk`，用于保存布尔标志 readok。
        bool bReadOk = false;
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (FTextureRenderTargetResource* Resource = CaptureRT->GameThread_GetRenderTargetResource())
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：调用 `ReadFlags` 执行当前步骤需要的功能逻辑。
            FReadSurfaceDataFlags ReadFlags(RCM_UNorm, CubeFace_MAX);
            // 解释：调用 `SetLinearToGamma` 执行当前步骤需要的功能逻辑。
            ReadFlags.SetLinearToGamma(false);
            // 解释：这一行把右侧表达式的结果写入 `bReadOk`，完成 布尔标志 readok 的更新。
            bReadOk = Resource->ReadPixels(Pixels, ReadFlags);
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Capture->TextureTarget = OriginalTarget;
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Capture->CaptureSource = OriginalSource;
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Capture->bCaptureEveryFrame = bOriginalEveryFrame;
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Capture->PostProcessSettings = OriginalPostProcess;
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Capture->PostProcessBlendWeight = OriginalPostProcessBlendWeight;

        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (!bReadOk || Pixels.Num() != Width * Height)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return false;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行把右侧表达式的结果写入 `OutPixels`，完成 outpixels 的更新。
        OutPixels = MoveTemp(Pixels);
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return true;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行定义函数 `ComputeAirSimObjectIdFromName`，开始实现compute空中仿真objectidfromname的具体逻辑。
    uint8 ComputeAirSimObjectIdFromName(const FString& Name)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行声明成员或局部变量 `Sum`，用于保存sum。
        int32 Sum = 0;
        // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
        for (TCHAR Ch : Name)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
            if (Ch >= TCHAR('A') && Ch <= TCHAR('Z'))
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            {
                // 解释：这一行把右侧表达式的结果写入 `Ch`，完成 ch 的更新。
                Ch = static_cast<TCHAR>(Ch - TCHAR('A') + TCHAR('a'));
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            }

            // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
            if (Ch >= TCHAR('a') && Ch <= TCHAR('z'))
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            {
                // 解释：这一行在 `Sum` 的原有基础上继续累加新量，用于持续更新 sum。
                Sum += static_cast<int32>(Ch);
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            }
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return static_cast<uint8>(Sum % 255);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行定义函数 `ComputeStencilIdForPrimitive`，开始实现computestencilidforprimitive的具体逻辑。
    uint8 ComputeStencilIdForPrimitive(const UPrimitiveComponent* Primitive, const AActor* Owner)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (Primitive)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行把右侧表达式的结果写入 `const uint8 PrimitiveId`，完成 constuint8primitiveid 的更新。
            const uint8 PrimitiveId = ComputeAirSimObjectIdFromName(Primitive->GetName());
            // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
            if (PrimitiveId != 0)
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            {
                // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
                return PrimitiveId;
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            }
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (Owner)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行把右侧表达式的结果写入 `const uint8 ActorId`，完成 constuint8Actorid 的更新。
            const uint8 ActorId = ComputeAirSimObjectIdFromName(Owner->GetName());
            // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
            if (ActorId != 0)
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            {
                // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
                return ActorId;
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            }
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return 1;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行定义函数 `EnsureWorldSegmentationStencil`，开始实现ensureworldsegmentationstencil的具体逻辑。
    void EnsureWorldSegmentationStencil(UWorld* World)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (!World)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行把右侧表达式的结果写入 `const double Now`，完成 constdoublenow 的更新。
        const double Now = World->GetTimeSeconds();
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (GLastSegmentationWorld.Get() == World &&
            // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
            (Now - GLastSegmentationUpdateTimeSec) < kSegmentationRefreshIntervalSec)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行把右侧表达式的结果写入 `GLastSegmentationWorld`，完成 glastsegmentationworld 的更新。
        GLastSegmentationWorld = World;
        // 解释：这一行把右侧表达式的结果写入 `GLastSegmentationUpdateTimeSec`，完成 glastsegmentationupdatetimesec 的更新。
        GLastSegmentationUpdateTimeSec = Now;

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

        // 解释：调用 `GetLevels` 执行当前步骤需要的功能逻辑。
        const TArray<ULevel*>& Levels = World->GetLevels();
        // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
        for (ULevel* Level : Levels)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
            if (!Level)
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            {
                // 解释：这一行跳过本轮循环剩余语句，直接进入下一轮迭代。
                continue;
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            }

            // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
            for (AActor* Actor : Level->Actors)
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            {
                // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
                if (!Actor || Actor->IsActorBeingDestroyed())
                // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
                {
                    // 解释：这一行跳过本轮循环剩余语句，直接进入下一轮迭代。
                    continue;
                // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
                }

                // 解释：这一行声明成员或局部变量 `PrimitiveComponents`，用于保存primitivecomponents。
                TArray<UPrimitiveComponent*> PrimitiveComponents;
                // 解释：调用 `UPrimitiveComponent>` 执行当前步骤需要的功能逻辑。
                Actor->GetComponents<UPrimitiveComponent>(PrimitiveComponents);
                // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
                for (UPrimitiveComponent* Primitive : PrimitiveComponents)
                // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
                {
                    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
                    if (!Primitive || !Primitive->IsRegistered())
                    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
                    {
                        // 解释：这一行跳过本轮循环剩余语句，直接进入下一轮迭代。
                        continue;
                    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
                    }

                    // 解释：调用 `SetRenderCustomDepth` 执行当前步骤需要的功能逻辑。
                    Primitive->SetRenderCustomDepth(true);
                    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
                    if (Primitive->CustomDepthStencilValue <= 0)
                    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
                    {
                        // 解释：调用 `SetCustomDepthStencilValue` 执行当前步骤需要的功能逻辑。
                        Primitive->SetCustomDepthStencilValue(ComputeStencilIdForPrimitive(Primitive, Actor));
                    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
                    }
                // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
                }
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            }
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行定义函数 `CaptureAirSimMaterialPixels`，开始实现采集空中仿真materialpixels的具体逻辑。
    bool CaptureAirSimMaterialPixels(
        // 解释：这一行继续展开 `CaptureAirSimMaterialPixels` 的参数列表，声明参数 `Capture` 用于传入采集。
        USceneCaptureComponent2D* Capture,
        // 解释：这一行继续展开 `CaptureAirSimMaterialPixels` 的参数列表，声明参数 `Width` 用于传入width。
        int32 Width,
        // 解释：这一行继续展开 `CaptureAirSimMaterialPixels` 的参数列表，声明参数 `Height` 用于传入height。
        int32 Height,
        // 解释：这一行继续展开 `CaptureAirSimMaterialPixels` 的参数列表，声明参数 `MaterialPath` 用于传入materialpath。
        const TCHAR* MaterialPath,
        // 解释：这一行继续展开 `CaptureAirSimMaterialPixels` 的参数列表，声明参数 `bForceLinearGamma` 用于传入布尔标志 forcelineargamma。
        bool bForceLinearGamma,
        // 解释：这一行继续展开 `CaptureAirSimMaterialPixels` 的参数列表，声明参数 `bNeedsSegmentationStencil` 用于传入布尔标志 needssegmentationstencil。
        bool bNeedsSegmentationStencil,
        // 解释：这一行收束函数 `CaptureAirSimMaterialPixels` 的签名，后面会进入实现体或以分号结束声明。
        TArray<FColor>& OutPixels)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (!Capture || !MaterialPath)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return false;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (bNeedsSegmentationStencil)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：调用 `EnsureWorldSegmentationStencil` 执行当前步骤需要的功能逻辑。
            EnsureWorldSegmentationStencil(Capture->GetWorld());
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行把右侧表达式的结果写入 `UMaterialInterface* Material`，完成 umaterialinterfacematerial 的更新。
        UMaterialInterface* Material = LoadObject<UMaterialInterface>(nullptr, MaterialPath);
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (!Material)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行声明成员或局部变量 `MissingMaterialsLogged`，用于保存missingmaterialslogged。
            static TSet<FString> MissingMaterialsLogged;
            // 解释：调用 `MaterialPathStr` 执行当前步骤需要的功能逻辑。
            const FString MaterialPathStr(MaterialPath);
            // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
            if (!MissingMaterialsLogged.Contains(MaterialPathStr))
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            {
                // 解释：调用 `Add` 执行当前步骤需要的功能逻辑。
                MissingMaterialsLogged.Add(MaterialPathStr);
                // 解释：调用 `UE_LOG` 输出调试日志，便于运行时观察状态和排查问题。
                UE_LOG(LogTemp, Warning, TEXT("[AirSimImageUtils] AirSim material missing: %s"), MaterialPath);
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            }
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return false;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return CaptureWithPostProcessMaterial(Capture, Width, Height, Material, bForceLinearGamma, OutPixels);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行定义函数 `ReadScenePixels`，开始实现readscenepixels的具体逻辑。
    bool ReadScenePixels(USceneCaptureComponent2D* Capture, int32 Width, int32 Height, TArray<FColor>& OutPixels)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (!Capture || !Capture->TextureTarget || Width <= 0 || Height <= 0)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return false;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

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
            return false;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行声明成员或局部变量 `Pixels`，用于保存pixels。
        TArray<FColor> Pixels;
        // 解释：调用 `ReadFlags` 执行当前步骤需要的功能逻辑。
        FReadSurfaceDataFlags ReadFlags(RCM_UNorm, CubeFace_MAX);
        // 解释：调用 `SetLinearToGamma` 执行当前步骤需要的功能逻辑。
        ReadFlags.SetLinearToGamma(false);
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (!Resource->ReadPixels(Pixels, ReadFlags))
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return false;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行声明成员或局部变量 `SourceWidth`，用于保存sourcewidth。
        const int32 SourceWidth = Capture->TextureTarget->SizeX;
        // 解释：这一行声明成员或局部变量 `SourceHeight`，用于保存sourceheight。
        const int32 SourceHeight = Capture->TextureTarget->SizeY;
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (SourceWidth <= 0 || SourceHeight <= 0 || Pixels.Num() != SourceWidth * SourceHeight)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return false;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (SourceWidth == Width && SourceHeight == Height)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行把右侧表达式的结果写入 `OutPixels`，完成 outpixels 的更新。
            OutPixels = MoveTemp(Pixels);
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return true;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return ResizePixelsNearest(Pixels, SourceWidth, SourceHeight, Width, Height, OutPixels);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行定义函数 `CaptureDepthLinear`，开始实现采集depthlinear的具体逻辑。
    bool CaptureDepthLinear(USceneCaptureComponent2D* Capture, int32 Width, int32 Height, TArray<FLinearColor>& OutDepthPixels)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (!Capture || Width <= 0 || Height <= 0)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return false;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行声明成员或局部变量 `OriginalTarget`，用于保存originaltarget。
        UTextureRenderTarget2D* OriginalTarget = Capture->TextureTarget;
        // 解释：这一行声明成员或局部变量 `OriginalSource`，用于保存originalsource。
        const ESceneCaptureSource OriginalSource = Capture->CaptureSource;
        // 解释：这一行声明成员或局部变量 `bOriginalEveryFrame`，用于保存布尔标志 originaleveryframe。
        const bool bOriginalEveryFrame = Capture->bCaptureEveryFrame;

        // 解释：这一行把右侧表达式的结果写入 `UTextureRenderTarget2D* DepthRT`，完成 utexturerendertarget2Ddepthrt 的更新。
        UTextureRenderTarget2D* DepthRT = AcquireDepthRenderTarget(Width, Height);
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (!DepthRT)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return false;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Capture->TextureTarget = DepthRT;
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Capture->CaptureSource = ESceneCaptureSource::SCS_SceneDepth;
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Capture->bCaptureEveryFrame = false;
        // 解释：调用 `CaptureScene` 执行当前步骤需要的功能逻辑。
        Capture->CaptureScene();

        // 解释：这一行声明成员或局部变量 `bOk`，用于保存布尔标志 ok。
        bool bOk = false;
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (FTextureRenderTargetResource* Resource = DepthRT->GameThread_GetRenderTargetResource())
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行把右侧表达式的结果写入 `bOk`，完成 布尔标志 ok 的更新。
            bOk = Resource->ReadLinearColorPixels(OutDepthPixels);
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Capture->TextureTarget = OriginalTarget;
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Capture->CaptureSource = OriginalSource;
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Capture->bCaptureEveryFrame = bOriginalEveryFrame;

        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return bOk && (OutDepthPixels.Num() == Width * Height);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行定义函数 `EncodeBgraJpegBase64`，开始实现encodebgrajpegbase64的具体逻辑。
    FString EncodeBgraJpegBase64(const TArray<FColor>& Pixels, int32 Width, int32 Height, int32 Quality)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (Pixels.Num() != Width * Height || Width <= 0 || Height <= 0)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return TEXT("");
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行声明成员或局部变量 `RawData`，用于保存rawdata。
        TArray<uint8> RawData;
        // 解释：调用 `SetNumUninitialized` 执行当前步骤需要的功能逻辑。
        RawData.SetNumUninitialized(Pixels.Num() * 4);
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
            return TEXT("");
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (!Wrapper->SetRaw(RawData.GetData(), RawData.Num(), Width, Height, ERGBFormat::BGRA, 8))
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return TEXT("");
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：调用 `GetCompressed` 执行当前步骤需要的功能逻辑。
        const TArray64<uint8>& JpegData = Wrapper->GetCompressed(FMath::Clamp(Quality, 1, 100));
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return FBase64::Encode(JpegData.GetData(), JpegData.Num());
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行定义函数 `ConvertDepthToPlanar`，开始实现convertdepthtoplanar的具体逻辑。
    void ConvertDepthToPlanar(const TArray<FLinearColor>& DepthPixels, int32 Width, int32 Height, float MaxDepthMeters, TArray<FColor>& OutPixels)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `SetNumUninitialized` 执行当前步骤需要的功能逻辑。
        OutPixels.SetNumUninitialized(Width * Height);
        // 解释：这一行把右侧表达式的结果写入 `const float InvMaxDepth`，完成 constfloatinvmaxdepth 的更新。
        const float InvMaxDepth = (MaxDepthMeters > KINDA_SMALL_NUMBER) ? (1.0f / MaxDepthMeters) : 1.0f;

        // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
        for (int32 i = 0; i < DepthPixels.Num(); ++i)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行通过 `FMath::Max` 给 `const float DepthMeters` 施加下界约束，避免 constfloatdepthmeters 过小。
            const float DepthMeters = FMath::Max(0.0f, DepthPixels[i].R / 100.0f);
            // 解释：这一行先对计算结果做限幅，再写入 `const float Normalized`，防止 constfloatnormalized 超出允许范围。
            const float Normalized = FMath::Clamp(DepthMeters * InvMaxDepth, 0.0f, 1.0f);
            // 解释：这一行把右侧表达式的结果写入 `const uint8 Gray`，完成 constuint8gray 的更新。
            const uint8 Gray = static_cast<uint8>(FMath::RoundToInt(Normalized * 255.0f));
            // 解释：这一行把右侧表达式的结果写入 `OutPixels[i]`，完成 outpixels 的更新。
            OutPixels[i] = FColor(Gray, Gray, Gray, 255);
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行定义函数 `ConvertDepthToVis`，开始实现convertdepthtovis的具体逻辑。
    void ConvertDepthToVis(const TArray<FLinearColor>& DepthPixels, int32 Width, int32 Height, TArray<FColor>& OutPixels)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `SetNumUninitialized` 执行当前步骤需要的功能逻辑。
        OutPixels.SetNumUninitialized(Width * Height);

        // 解释：这一行声明成员或局部变量 `InvRange`，用于保存invrange。
        const float InvRange = 1.0f / kDepthVisRangeMeters;
        // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
        for (int32 i = 0; i < DepthPixels.Num(); ++i)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行通过 `FMath::Max` 给 `const float DepthMeters` 施加下界约束，避免 constfloatdepthmeters 过小。
            const float DepthMeters = FMath::Max(0.0f, DepthPixels[i].R / 100.0f);
            // 解释：这一行先对计算结果做限幅，再写入 `const float Normalized`，防止 constfloatnormalized 超出允许范围。
            const float Normalized = FMath::Clamp(DepthMeters * InvRange, 0.0f, 1.0f);
            // 解释：这一行把右侧表达式的结果写入 `const uint8 Gray`，完成 constuint8gray 的更新。
            const uint8 Gray = static_cast<uint8>(FMath::RoundToInt(Normalized * 255.0f));
            // 解释：这一行把右侧表达式的结果写入 `OutPixels[i]`，完成 outpixels 的更新。
            OutPixels[i] = FColor(Gray, Gray, Gray, 255);
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行定义函数 `DecodeStencilId`，开始实现decodestencilid的具体逻辑。
    uint8 DecodeStencilId(const FColor& Raw)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        const bool bLooksGray = (FMath::Abs(static_cast<int32>(Raw.R) - static_cast<int32>(Raw.G)) <= 2) &&
            // 解释：调用 `Abs` 执行当前步骤需要的功能逻辑。
            (FMath::Abs(static_cast<int32>(Raw.R) - static_cast<int32>(Raw.B)) <= 2);

        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (bLooksGray)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return Raw.R;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return static_cast<uint8>(FMath::Max3(static_cast<int32>(Raw.R), static_cast<int32>(Raw.G), static_cast<int32>(Raw.B)));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行定义函数 `DenoiseStencilIds`，开始实现denoisestencilids的具体逻辑。
    void DenoiseStencilIds(TArray<uint8>& InOutStencilIds, int32 Width, int32 Height)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (InOutStencilIds.Num() != Width * Height || Width < 3 || Height < 3)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行声明成员或局部变量 `SourceIds`，用于保存sourceids。
        const TArray<uint8> SourceIds = InOutStencilIds;
        // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
        for (int32 Y = 1; Y < Height - 1; ++Y)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
            for (int32 X = 1; X < Width - 1; ++X)
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            {
                // 解释：这一行把右侧表达式的结果写入 `uint8 CandidateValues[9]`，完成 uint8candidatevalues 的更新。
                uint8 CandidateValues[9] = {};
                // 解释：这一行把右侧表达式的结果写入 `uint8 CandidateCounts[9]`，完成 uint8candidatecounts 的更新。
                uint8 CandidateCounts[9] = {};
                // 解释：这一行声明成员或局部变量 `CandidateNum`，用于保存candidatenum。
                int32 CandidateNum = 0;

                // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
                for (int32 DY = -1; DY <= 1; ++DY)
                // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
                {
                    // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
                    for (int32 DX = -1; DX <= 1; ++DX)
                    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
                    {
                        // 解释：这一行把右侧表达式的结果写入 `const uint8 Value`，完成 constuint8value 的更新。
                        const uint8 Value = SourceIds[(Y + DY) * Width + (X + DX)];
                        // 解释：这一行声明成员或局部变量 `FoundIndex`，用于保存foundindex。
                        int32 FoundIndex = INDEX_NONE;
                        // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
                        for (int32 I = 0; I < CandidateNum; ++I)
                        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
                        {
                            // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
                            if (CandidateValues[I] == Value)
                            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
                            {
                                // 解释：这一行把右侧表达式的结果写入 `FoundIndex`，完成 foundindex 的更新。
                                FoundIndex = I;
                                // 解释：这一行立即跳出当前循环或 `switch` 分支，避免继续执行后续分支。
                                break;
                            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
                            }
                        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
                        }

                        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
                        if (FoundIndex == INDEX_NONE)
                        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
                        {
                            // 解释：这一行把右侧表达式的结果写入 `FoundIndex`，完成 foundindex 的更新。
                            FoundIndex = CandidateNum++;
                            // 解释：这一行把右侧表达式的结果写入 `CandidateValues[FoundIndex]`，完成 candidatevalues 的更新。
                            CandidateValues[FoundIndex] = Value;
                            // 解释：这一行把右侧表达式的结果写入 `CandidateCounts[FoundIndex]`，完成 candidatecounts 的更新。
                            CandidateCounts[FoundIndex] = 0;
                        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
                        }

                        // 解释：这一行把右侧表达式的结果写入 `CandidateCounts[FoundIndex]`，完成 candidatecounts 的更新。
                        CandidateCounts[FoundIndex] = static_cast<uint8>(CandidateCounts[FoundIndex] + 1);
                    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
                    }
                // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
                }

                // 解释：这一行声明成员或局部变量 `BestIndex`，用于保存bestindex。
                int32 BestIndex = 0;
                // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
                for (int32 I = 1; I < CandidateNum; ++I)
                // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
                {
                    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
                    if (CandidateCounts[I] > CandidateCounts[BestIndex])
                    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
                    {
                        // 解释：这一行把右侧表达式的结果写入 `BestIndex`，完成 bestindex 的更新。
                        BestIndex = I;
                    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
                    }
                // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
                }

                // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
                if (CandidateCounts[BestIndex] >= 4)
                // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
                {
                    // 解释：这一行把右侧表达式的结果写入 `InOutStencilIds[Y * Width + X]`，完成 inoutstencilids 的更新。
                    InOutStencilIds[Y * Width + X] = CandidateValues[BestIndex];
                // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
                }
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            }
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }


    // 解释：这一行定义函数 `RemoveStencilOverlayGlyphs`，开始实现removestenciloverlayglyphs的具体逻辑。
    void RemoveStencilOverlayGlyphs(TArray<FColor>& InOutPixels, int32 Width, int32 Height)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (InOutPixels.Num() != Width * Height || Width < 3 || Height < 3)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行声明成员或局部变量 `SourcePixels`，用于保存sourcepixels。
        const TArray<FColor> SourcePixels = InOutPixels;
        // 解释：这一行声明成员或局部变量 `Radius`，用于保存radius。
        constexpr int32 Radius = 2;
        // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
        for (int32 Y = 0; Y < Height; ++Y)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行通过 `FMath::Max` 给 `const int32 Y0` 施加下界约束，避免 constint32y0 过小。
            const int32 Y0 = FMath::Max(0, Y - Radius);
            // 解释：这一行通过 `FMath::Min` 给 `const int32 Y1` 施加上界约束，避免 constint32y1 过大。
            const int32 Y1 = FMath::Min(Height - 1, Y + Radius);
            // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
            for (int32 X = 0; X < Width; ++X)
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            {
                // 解释：这一行通过 `FMath::Max` 给 `const int32 X0` 施加下界约束，避免 constint32x0 过小。
                const int32 X0 = FMath::Max(0, X - Radius);
                // 解释：这一行通过 `FMath::Min` 给 `const int32 X1` 施加上界约束，避免 constint32x1 过大。
                const int32 X1 = FMath::Min(Width - 1, X + Radius);

                // 解释：这一行声明成员或局部变量 `MaxR`，用于保存maxR。
                uint8 MaxR = 0;
                // 解释：这一行声明成员或局部变量 `MaxG`，用于保存maxG。
                uint8 MaxG = 0;
                // 解释：这一行声明成员或局部变量 `MaxB`，用于保存maxB。
                uint8 MaxB = 0;
                // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
                for (int32 SY = Y0; SY <= Y1; ++SY)
                // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
                {
                    // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
                    for (int32 SX = X0; SX <= X1; ++SX)
                    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
                    {
                        // 解释：这一行声明成员或局部变量 `P`，用于保存协方差矩阵 P。
                        const FColor& P = SourcePixels[SY * Width + SX];
                        // 解释：这一行通过 `FMath::Max` 给 `MaxR` 施加下界约束，避免 maxR 过小。
                        MaxR = FMath::Max(MaxR, P.R);
                        // 解释：这一行通过 `FMath::Max` 给 `MaxG` 施加下界约束，避免 maxG 过小。
                        MaxG = FMath::Max(MaxG, P.G);
                        // 解释：这一行通过 `FMath::Max` 给 `MaxB` 施加下界约束，避免 maxB 过小。
                        MaxB = FMath::Max(MaxB, P.B);
                    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
                    }
                // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
                }

                // 解释：这一行声明成员或局部变量 `Out`，用于保存out。
                FColor& Out = InOutPixels[Y * Width + X];
                // 解释：这一行把右侧表达式的结果写入 `Out.R`，完成 R 的更新。
                Out.R = MaxR;
                // 解释：这一行把右侧表达式的结果写入 `Out.G`，完成 G 的更新。
                Out.G = MaxG;
                // 解释：这一行把右侧表达式的结果写入 `Out.B`，完成 B 的更新。
                Out.B = MaxB;
                // 解释：这一行把右侧表达式的结果写入 `Out.A`，完成 A 的更新。
                Out.A = 255;
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            }
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行定义函数 `CaptureCustomStencilIds`，开始实现采集customstencilids的具体逻辑。
    bool CaptureCustomStencilIds(USceneCaptureComponent2D* Capture, int32 Width, int32 Height, TArray<uint8>& OutStencilIds, int32& OutNonZeroCount)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `Reset` 执行当前步骤需要的功能逻辑。
        OutStencilIds.Reset();
        // 解释：这一行把右侧表达式的结果写入 `OutNonZeroCount`，完成 outnonzerocount 的更新。
        OutNonZeroCount = 0;

        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (!Capture || Width <= 0 || Height <= 0)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return false;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：调用 `EnsureWorldSegmentationStencil` 执行当前步骤需要的功能逻辑。
        EnsureWorldSegmentationStencil(Capture->GetWorld());

        // 解释：这一行声明成员或局部变量 `CachedStencilVizMaterial`，用于保存cachedstencilvizmaterial。
        static TWeakObjectPtr<UMaterialInterface> CachedStencilVizMaterial;
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (!CachedStencilVizMaterial.IsValid())
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行把右侧表达式的结果写入 `CachedStencilVizMaterial`，完成 cachedstencilvizmaterial 的更新。
            CachedStencilVizMaterial = LoadObject<UMaterialInterface>(nullptr, TEXT("/Engine/BufferVisualization/CustomStencil.CustomStencil"));
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行把右侧表达式的结果写入 `UMaterialInterface* StencilVizMaterial`，完成 umaterialinterfacestencilvizmaterial 的更新。
        UMaterialInterface* StencilVizMaterial = CachedStencilVizMaterial.Get();
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (!StencilVizMaterial)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return false;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行把右侧表达式的结果写入 `UTextureRenderTarget2D* StencilRT`，完成 utexturerendertarget2Dstencilrt 的更新。
        UTextureRenderTarget2D* StencilRT = AcquireColorRenderTarget(Width, Height);
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (!StencilRT)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return false;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行声明成员或局部变量 `OriginalTarget`，用于保存originaltarget。
        UTextureRenderTarget2D* OriginalTarget = Capture->TextureTarget;
        // 解释：这一行声明成员或局部变量 `OriginalSource`，用于保存originalsource。
        const ESceneCaptureSource OriginalSource = Capture->CaptureSource;
        // 解释：这一行声明成员或局部变量 `bOriginalEveryFrame`，用于保存布尔标志 originaleveryframe。
        const bool bOriginalEveryFrame = Capture->bCaptureEveryFrame;
        // 解释：这一行声明成员或局部变量 `OriginalPostProcess`，用于保存originalpostprocess。
        const FPostProcessSettings OriginalPostProcess = Capture->PostProcessSettings;
        // 解释：这一行声明成员或局部变量 `OriginalPostProcessBlendWeight`，用于保存originalpostprocessblendweight。
        const float OriginalPostProcessBlendWeight = Capture->PostProcessBlendWeight;

        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Capture->TextureTarget = StencilRT;
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Capture->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Capture->bCaptureEveryFrame = false;
        // 解释：调用 `FPostProcessSettings` 执行当前步骤需要的功能逻辑。
        Capture->PostProcessSettings = FPostProcessSettings();
        // 解释：调用 `AddBlendable` 执行当前步骤需要的功能逻辑。
        Capture->PostProcessSettings.AddBlendable(StencilVizMaterial, 1.0f);
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Capture->PostProcessBlendWeight = 1.0f;
        // 解释：调用 `CaptureScene` 执行当前步骤需要的功能逻辑。
        Capture->CaptureScene();

        // 解释：这一行声明成员或局部变量 `bReadOk`，用于保存布尔标志 readok。
        bool bReadOk = false;
        // 解释：这一行声明成员或局部变量 `StencilPixels`，用于保存stencilpixels。
        TArray<FColor> StencilPixels;
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (FTextureRenderTargetResource* Resource = StencilRT->GameThread_GetRenderTargetResource())
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：调用 `ReadFlags` 执行当前步骤需要的功能逻辑。
            FReadSurfaceDataFlags ReadFlags(RCM_UNorm, CubeFace_MAX);
            // 解释：调用 `SetLinearToGamma` 执行当前步骤需要的功能逻辑。
            ReadFlags.SetLinearToGamma(false);
            // 解释：这一行把右侧表达式的结果写入 `bReadOk`，完成 布尔标志 readok 的更新。
            bReadOk = Resource->ReadPixels(StencilPixels, ReadFlags);
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Capture->TextureTarget = OriginalTarget;
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Capture->CaptureSource = OriginalSource;
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Capture->bCaptureEveryFrame = bOriginalEveryFrame;
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Capture->PostProcessSettings = OriginalPostProcess;
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Capture->PostProcessBlendWeight = OriginalPostProcessBlendWeight;

        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (!bReadOk || StencilPixels.Num() != Width * Height)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return false;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // UE buffer visualization can contain tiny numeric glyph overlays on stencil view.
        // Remove high-frequency dark glyphs before decoding stencil ids.
        // 解释：调用 `RemoveStencilOverlayGlyphs` 执行当前步骤需要的功能逻辑。
        RemoveStencilOverlayGlyphs(StencilPixels, Width, Height);

        // 解释：调用 `SetNumUninitialized` 执行当前步骤需要的功能逻辑。
        OutStencilIds.SetNumUninitialized(Width * Height);
        // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
        for (int32 i = 0; i < StencilPixels.Num(); ++i)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行把右侧表达式的结果写入 `OutStencilIds[i]`，完成 outstencilids 的更新。
            OutStencilIds[i] = DecodeStencilId(StencilPixels[i]);
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：调用 `DenoiseStencilIds` 执行当前步骤需要的功能逻辑。
        DenoiseStencilIds(OutStencilIds, Width, Height);

        // 解释：这一行把右侧表达式的结果写入 `OutNonZeroCount`，完成 outnonzerocount 的更新。
        OutNonZeroCount = 0;
        // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
        for (const uint8 StencilId : OutStencilIds)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
            if (StencilId > 0)
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            {
                // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                ++OutNonZeroCount;
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            }
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return true;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行定义函数 `AirSimSegmentationColorFromId`，开始实现空中仿真segmentationcolorfromid的具体逻辑。
    FColor AirSimSegmentationColorFromId(uint8 Id)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (Id == 0)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return FColor::Black;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行把右侧表达式的结果写入 `const uint32 X`，完成 constuint32X 的更新。
        const uint32 X = static_cast<uint32>(Id);
        // 解释：这一行把右侧表达式的结果写入 `const uint8 R`，完成 constuint8R 的更新。
        const uint8 R = static_cast<uint8>((X * 37u + 13u) % 255u);
        // 解释：这一行把右侧表达式的结果写入 `const uint8 G`，完成 constuint8G 的更新。
        const uint8 G = static_cast<uint8>((X * 73u + 47u) % 255u);
        // 解释：这一行把右侧表达式的结果写入 `const uint8 B`，完成 constuint8B 的更新。
        const uint8 B = static_cast<uint8>((X * 109u + 91u) % 255u);
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return FColor(FMath::Max<uint8>(R, 16), FMath::Max<uint8>(G, 16), FMath::Max<uint8>(B, 16), 255);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行定义函数 `CaptureCustomStencilSegmentation`，开始实现采集customstencilsegmentation的具体逻辑。
    bool CaptureCustomStencilSegmentation(USceneCaptureComponent2D* Capture, int32 Width, int32 Height, TArray<FColor>& OutPixels)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行声明成员或局部变量 `StencilIds`，用于保存stencilids。
        TArray<uint8> StencilIds;
        // 解释：这一行声明成员或局部变量 `NonZeroCount`，用于保存nonzerocount。
        int32 NonZeroCount = 0;
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (!CaptureCustomStencilIds(Capture, Width, Height, StencilIds, NonZeroCount))
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return false;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：调用 `SetNumUninitialized` 执行当前步骤需要的功能逻辑。
        OutPixels.SetNumUninitialized(Width * Height);
        // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
        for (int32 i = 0; i < StencilIds.Num(); ++i)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行把右侧表达式的结果写入 `OutPixels[i]`，完成 outpixels 的更新。
            OutPixels[i] = AirSimSegmentationColorFromId(StencilIds[i]);
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行通过 `FMath::Max` 给 `const int32 MinForegroundPixels` 施加下界约束，避免 constint32minforegroundpixels 过小。
        const int32 MinForegroundPixels = FMath::Max(8, (Width * Height) / 500);
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return NonZeroCount >= MinForegroundPixels;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行定义函数 `CaptureCustomStencilInfrared`，开始实现采集customstencilinfrared的具体逻辑。
    bool CaptureCustomStencilInfrared(USceneCaptureComponent2D* Capture, int32 Width, int32 Height, TArray<FColor>& OutPixels)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行声明成员或局部变量 `StencilIds`，用于保存stencilids。
        TArray<uint8> StencilIds;
        // 解释：这一行声明成员或局部变量 `NonZeroCount`，用于保存nonzerocount。
        int32 NonZeroCount = 0;
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (!CaptureCustomStencilIds(Capture, Width, Height, StencilIds, NonZeroCount))
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return false;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：调用 `SetNumUninitialized` 执行当前步骤需要的功能逻辑。
        OutPixels.SetNumUninitialized(Width * Height);
        // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
        for (int32 i = 0; i < StencilIds.Num(); ++i)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行声明成员或局部变量 `Gray`，用于保存gray。
            const uint8 Gray = StencilIds[i];
            // 解释：这一行把右侧表达式的结果写入 `OutPixels[i]`，完成 outpixels 的更新。
            OutPixels[i] = FColor(Gray, Gray, Gray, 255);
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return NonZeroCount > 0;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行定义函数 `ConvertToPseudoSegmentation`，开始实现converttopseudosegmentation的具体逻辑。
    void ConvertToPseudoSegmentation(TArray<FColor>& InOutPixels, int32 Width, int32 Height)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        static const FColor Palette[12] =
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行位于构造函数初始化列表中，把 `FColor` 直接初始化为 `255, 0, 0`，减少进入函数体后的额外赋值开销。
            FColor(255, 0, 0),
            // 解释：这一行位于构造函数初始化列表中，把 `FColor` 直接初始化为 `255, 128, 0`，减少进入函数体后的额外赋值开销。
            FColor(255, 128, 0),
            // 解释：这一行位于构造函数初始化列表中，把 `FColor` 直接初始化为 `255, 255, 0`，减少进入函数体后的额外赋值开销。
            FColor(255, 255, 0),
            // 解释：这一行位于构造函数初始化列表中，把 `FColor` 直接初始化为 `128, 255, 0`，减少进入函数体后的额外赋值开销。
            FColor(128, 255, 0),
            // 解释：这一行位于构造函数初始化列表中，把 `FColor` 直接初始化为 `0, 255, 0`，减少进入函数体后的额外赋值开销。
            FColor(0, 255, 0),
            // 解释：这一行位于构造函数初始化列表中，把 `FColor` 直接初始化为 `0, 255, 128`，减少进入函数体后的额外赋值开销。
            FColor(0, 255, 128),
            // 解释：这一行位于构造函数初始化列表中，把 `FColor` 直接初始化为 `0, 255, 255`，减少进入函数体后的额外赋值开销。
            FColor(0, 255, 255),
            // 解释：这一行位于构造函数初始化列表中，把 `FColor` 直接初始化为 `0, 128, 255`，减少进入函数体后的额外赋值开销。
            FColor(0, 128, 255),
            // 解释：这一行位于构造函数初始化列表中，把 `FColor` 直接初始化为 `0, 0, 255`，减少进入函数体后的额外赋值开销。
            FColor(0, 0, 255),
            // 解释：这一行位于构造函数初始化列表中，把 `FColor` 直接初始化为 `128, 0, 255`，减少进入函数体后的额外赋值开销。
            FColor(128, 0, 255),
            // 解释：这一行位于构造函数初始化列表中，把 `FColor` 直接初始化为 `255, 0, 255`，减少进入函数体后的额外赋值开销。
            FColor(255, 0, 255),
            // 解释：这一行位于构造函数初始化列表中，把 `FColor` 直接初始化为 `255, 0, 128`，减少进入函数体后的额外赋值开销。
            FColor(255, 0, 128)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        };

        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (InOutPixels.Num() != Width * Height || Width <= 0 || Height <= 0)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
        for (int32 Index = 0; Index < InOutPixels.Num(); ++Index)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行声明成员或局部变量 `Src`，用于保存src。
            const FColor& Src = InOutPixels[Index];
            // 解释：调用 `Rgb` 执行当前步骤需要的功能逻辑。
            const FLinearColor Rgb(Src.R / 255.0f, Src.G / 255.0f, Src.B / 255.0f, 1.0f);
            // 解释：这一行把右侧表达式的结果写入 `const FLinearColor Hsv`，完成 constflinearcolorhsv 的更新。
            const FLinearColor Hsv = Rgb.LinearRGBToHSV();

            // 解释：这一行声明成员或局部变量 `SegColor`，用于保存segcolor。
            FColor SegColor;
            // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
            if (Hsv.B < 0.10f)
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            {
                // 解释：这一行把右侧表达式的结果写入 `SegColor`，完成 segcolor 的更新。
                SegColor = FColor::Black;
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            }
            // 解释：这一行声明补充分支，当上一个条件不成立且当前条件成立时执行。
            else if (Hsv.G < 0.10f)
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            {
                // 解释：这一行先对计算结果做限幅，再写入 `const uint8 Gray`，防止 constuint8gray 超出允许范围。
                const uint8 Gray = static_cast<uint8>(FMath::RoundToInt(FMath::Clamp(Hsv.B, 0.0f, 1.0f) * 255.0f));
                // 解释：这一行把右侧表达式的结果写入 `const uint8 Bucket`，完成 constuint8bucket 的更新。
                const uint8 Bucket = static_cast<uint8>((Gray / 48) * 48);
                // 解释：这一行把右侧表达式的结果写入 `SegColor`，完成 segcolor 的更新。
                SegColor = FColor(Bucket, Bucket, Bucket, 255);
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            }
            // 解释：这一行给出兜底分支，当上面的条件都不满足时执行。
            else
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            {
                // 解释：这一行先对计算结果做限幅，再写入 `const int32 HueBucket`，防止 constint32huebucket 超出允许范围。
                const int32 HueBucket = FMath::Clamp(FMath::FloorToInt(Hsv.R / 30.0f), 0, 11);
                // 解释：这一行把右侧表达式的结果写入 `SegColor`，完成 segcolor 的更新。
                SegColor = Palette[HueBucket];
                // 解释：这一行把右侧表达式的结果写入 `SegColor.A`，完成 A 的更新。
                SegColor.A = 255;
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            }

            // 解释：这一行把右侧表达式的结果写入 `InOutPixels[Index]`，完成 inoutpixels 的更新。
            InOutPixels[Index] = SegColor;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行定义函数 `ConvertToInfraredFallback`，开始实现converttoinfraredfallback的具体逻辑。
    void ConvertToInfraredFallback(TArray<FColor>& InOutPixels)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
        for (FColor& Pixel : InOutPixels)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
            const uint8 Gray = static_cast<uint8>(
                // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                FMath::Clamp(
                    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                    FMath::RoundToInt(0.299f * Pixel.R + 0.587f * Pixel.G + 0.114f * Pixel.B),
                    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                    0,
                    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                    255));
            // 解释：这一行把右侧表达式的结果写入 `Pixel`，完成 pixel 的更新。
            Pixel = FColor(Gray, Gray, Gray, 255);
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 解析图像类型输入
 * @param RawType 外部传入的类型字符串或数值字符串
 * @param OutType 输出图像类型
 * @return 识别成功时返回 `true`
 */
// 解释：这一行定义函数 `TryParseImageType`，开始实现tryparse图像type的具体逻辑。
bool AirSimImageUtils::TryParseImageType(const FString& RawType, EImageType& OutType)
{
    FString Normalized = RawType;
    Normalized.TrimStartAndEndInline();
    Normalized.ToLowerInline();

    if (Normalized.IsEmpty() || Normalized == TEXT("scene") || Normalized == TEXT("0"))
    {
        OutType = EImageType::Scene;
        return true;
    }

    return false;
}

/** @brief 将图像类型转换为协议标准字符串 */
// 解释：这一行定义函数 `ToCanonicalString`，开始实现tocanonicalstring的具体逻辑。
FString AirSimImageUtils::ToCanonicalString(EImageType Type)
{
    return TEXT("scene");
}

/** @brief 将图像类型转换为 UI 显示名称 */
// 解释：这一行定义函数 `ToDisplayString`，开始实现todisplaystring的具体逻辑。
FString AirSimImageUtils::ToDisplayString(EImageType Type)
{
    return TEXT("0:Scene");
}

/** @brief 返回当前支持的全部 AirSim 图像类型 */
// 解释：这一行定义函数 `SupportedImageTypes`，开始实现supported图像types的具体逻辑。
TArray<AirSimImageUtils::EImageType> AirSimImageUtils::SupportedImageTypes()
{
    return {EImageType::Scene};
}

/**
 * @brief 捕获指定类型的原始像素
 * @return 捕获成功时返回 `true`
 * 该函数统一处理 Scene、深度、分割和红外等多种图像模式。
 */
// 解释：这一行定义函数 `CapturePixels`，开始实现采集pixels的具体逻辑。
bool AirSimImageUtils::CapturePixels(
    USceneCaptureComponent2D* Capture,
    int32 Width,
    int32 Height,
    EImageType Type,
    TArray<FColor>& OutPixels,
    float /*MaxDepthMeters*/)
{
    if (!Capture || Width <= 0 || Height <= 0)
    {
        return false;
    }

    if (Type != EImageType::Scene)
    {
        return false;
    }

    return ReadScenePixels(Capture, Width, Height, OutPixels);
}

/**
 * @brief 捕获指定类型图像并编码为 Base64 JPEG
 * @return Base64 图像字符串；失败时返回空串
 */
// 解释：这一行定义函数 `CaptureJpegBase64`，开始实现采集jpegbase64的具体逻辑。
FString AirSimImageUtils::CaptureJpegBase64(
    // 解释：这一行继续展开 `CaptureJpegBase64` 的参数列表，声明参数 `Capture` 用于传入采集。
    USceneCaptureComponent2D* Capture,
    // 解释：这一行继续展开 `CaptureJpegBase64` 的参数列表，声明参数 `Width` 用于传入width。
    int32 Width,
    // 解释：这一行继续展开 `CaptureJpegBase64` 的参数列表，声明参数 `Height` 用于传入height。
    int32 Height,
    // 解释：这一行继续展开 `CaptureJpegBase64` 的参数列表，声明参数 `Type` 用于传入type。
    EImageType Type,
    // 解释：这一行继续展开 `CaptureJpegBase64` 的参数列表，声明参数 `Quality` 用于传入quality。
    int32 Quality,
    // 解释：这一行收束函数 `CaptureJpegBase64` 的签名，后面会进入实现体或以分号结束声明。
    float MaxDepthMeters)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行声明成员或局部变量 `Pixels`，用于保存pixels。
    TArray<FColor> Pixels;
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!CapturePixels(Capture, Width, Height, Type, Pixels, MaxDepthMeters))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return TEXT("");
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return EncodeBgraJpegBase64(Pixels, Width, Height, Quality);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}


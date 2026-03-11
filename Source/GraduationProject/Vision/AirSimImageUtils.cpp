#include "AirSimImageUtils.h"

#include "Components/PrimitiveComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/Level.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "HAL/IConsoleManager.h"
#include "IImageWrapper.h"
#include "IImageWrapperModule.h"
#include "Materials/MaterialInterface.h"
#include "Misc/Base64.h"
#include "Modules/ModuleManager.h"
#include "UObject/StrongObjectPtr.h"

namespace
{
    struct FDepthRtCacheEntry
    {
        TStrongObjectPtr<UTextureRenderTarget2D> RenderTarget;
        int32 Width = 0;
        int32 Height = 0;
        uint64 LastUsedSerial = 0;
    };

    struct FColorRtCacheEntry
    {
        TStrongObjectPtr<UTextureRenderTarget2D> RenderTarget;
        int32 Width = 0;
        int32 Height = 0;
        bool bForceLinearGamma = false;
        uint64 LastUsedSerial = 0;
    };

    TArray<FDepthRtCacheEntry> GDepthRtCache;
    uint64 GDepthRtUseSerial = 0;
    constexpr int32 kMaxDepthRtCacheEntries = 4;

    TArray<FColorRtCacheEntry> GColorRtCache;
    uint64 GColorRtUseSerial = 0;
    constexpr int32 kMaxColorRtCacheEntries = 6;

    TWeakObjectPtr<UWorld> GLastSegmentationWorld;
    double GLastSegmentationUpdateTimeSec = -1000.0;
    constexpr double kSegmentationRefreshIntervalSec = 1.0;

    constexpr float kDepthVisRangeMeters = 100.0f;
    constexpr TCHAR kAirSimDepthPlanarMaterialPath[] = TEXT("/Game/AirSimHUDAssets/DepthPlanarMaterial.DepthPlanarMaterial");
    constexpr TCHAR kAirSimDepthVisMaterialPath[] = TEXT("/Game/AirSimHUDAssets/DepthVisMaterial.DepthVisMaterial");
    constexpr TCHAR kAirSimSegmentationMaterialPath[] = TEXT("/Game/AirSimHUDAssets/SegmentationMaterial.SegmentationMaterial");
    constexpr TCHAR kAirSimInfraredMaterialPath[] = TEXT("/Game/AirSimHUDAssets/InfraredMaterial.InfraredMaterial");

    bool ResizePixelsNearest(
        const TArray<FColor>& SourcePixels,
        int32 SourceWidth,
        int32 SourceHeight,
        int32 TargetWidth,
        int32 TargetHeight,
        TArray<FColor>& OutPixels)
    {
        if (SourcePixels.Num() != SourceWidth * SourceHeight ||
            SourceWidth <= 0 || SourceHeight <= 0 ||
            TargetWidth <= 0 || TargetHeight <= 0)
        {
            return false;
        }

        if (SourceWidth == TargetWidth && SourceHeight == TargetHeight)
        {
            OutPixels = SourcePixels;
            return true;
        }

        OutPixels.SetNumUninitialized(TargetWidth * TargetHeight);
        for (int32 Y = 0; Y < TargetHeight; ++Y)
        {
            const int32 SrcY = FMath::Clamp((Y * SourceHeight) / TargetHeight, 0, SourceHeight - 1);
            for (int32 X = 0; X < TargetWidth; ++X)
            {
                const int32 SrcX = FMath::Clamp((X * SourceWidth) / TargetWidth, 0, SourceWidth - 1);
                OutPixels[Y * TargetWidth + X] = SourcePixels[SrcY * SourceWidth + SrcX];
            }
        }

        return true;
    }

    UTextureRenderTarget2D* AcquireDepthRenderTarget(int32 Width, int32 Height)
    {
        if (Width <= 0 || Height <= 0)
        {
            return nullptr;
        }

        ++GDepthRtUseSerial;

        for (int32 Index = GDepthRtCache.Num() - 1; Index >= 0; --Index)
        {
            if (!GDepthRtCache[Index].RenderTarget.IsValid())
            {
                GDepthRtCache.RemoveAtSwap(Index);
            }
        }

        for (FDepthRtCacheEntry& Entry : GDepthRtCache)
        {
            if (Entry.Width == Width && Entry.Height == Height && Entry.RenderTarget.IsValid())
            {
                Entry.LastUsedSerial = GDepthRtUseSerial;
                return Entry.RenderTarget.Get();
            }
        }

        if (GDepthRtCache.Num() >= kMaxDepthRtCacheEntries)
        {
            int32 OldestIndex = 0;
            uint64 OldestSerial = ~static_cast<uint64>(0);
            for (int32 Index = 0; Index < GDepthRtCache.Num(); ++Index)
            {
                if (GDepthRtCache[Index].LastUsedSerial < OldestSerial)
                {
                    OldestSerial = GDepthRtCache[Index].LastUsedSerial;
                    OldestIndex = Index;
                }
            }
            GDepthRtCache.RemoveAtSwap(OldestIndex);
        }

        UTextureRenderTarget2D* DepthRT = NewObject<UTextureRenderTarget2D>(GetTransientPackage(), NAME_None, RF_Transient);
        if (!DepthRT)
        {
            return nullptr;
        }

        DepthRT->InitCustomFormat(Width, Height, PF_FloatRGBA, false);
        DepthRT->ClearColor = FLinearColor::Black;

        FDepthRtCacheEntry NewEntry;
        NewEntry.RenderTarget.Reset(DepthRT);
        NewEntry.Width = Width;
        NewEntry.Height = Height;
        NewEntry.LastUsedSerial = GDepthRtUseSerial;
        GDepthRtCache.Add(MoveTemp(NewEntry));

        return DepthRT;
    }

    UTextureRenderTarget2D* AcquireColorRenderTarget(int32 Width, int32 Height, bool bForceLinearGamma = false)
    {
        if (Width <= 0 || Height <= 0)
        {
            return nullptr;
        }

        ++GColorRtUseSerial;

        for (int32 Index = GColorRtCache.Num() - 1; Index >= 0; --Index)
        {
            if (!GColorRtCache[Index].RenderTarget.IsValid())
            {
                GColorRtCache.RemoveAtSwap(Index);
            }
        }

        for (FColorRtCacheEntry& Entry : GColorRtCache)
        {
            if (Entry.Width == Width &&
                Entry.Height == Height &&
                Entry.bForceLinearGamma == bForceLinearGamma &&
                Entry.RenderTarget.IsValid())
            {
                Entry.LastUsedSerial = GColorRtUseSerial;
                return Entry.RenderTarget.Get();
            }
        }

        if (GColorRtCache.Num() >= kMaxColorRtCacheEntries)
        {
            int32 OldestIndex = 0;
            uint64 OldestSerial = ~static_cast<uint64>(0);
            for (int32 Index = 0; Index < GColorRtCache.Num(); ++Index)
            {
                if (GColorRtCache[Index].LastUsedSerial < OldestSerial)
                {
                    OldestSerial = GColorRtCache[Index].LastUsedSerial;
                    OldestIndex = Index;
                }
            }
            GColorRtCache.RemoveAtSwap(OldestIndex);
        }

        UTextureRenderTarget2D* ColorRT = NewObject<UTextureRenderTarget2D>(GetTransientPackage(), NAME_None, RF_Transient);
        if (!ColorRT)
        {
            return nullptr;
        }

        ColorRT->InitCustomFormat(Width, Height, PF_B8G8R8A8, bForceLinearGamma);
        ColorRT->ClearColor = FLinearColor::Black;

        FColorRtCacheEntry NewEntry;
        NewEntry.RenderTarget.Reset(ColorRT);
        NewEntry.Width = Width;
        NewEntry.Height = Height;
        NewEntry.bForceLinearGamma = bForceLinearGamma;
        NewEntry.LastUsedSerial = GColorRtUseSerial;
        GColorRtCache.Add(MoveTemp(NewEntry));

        return ColorRT;
    }

    bool CaptureWithPostProcessMaterial(
        USceneCaptureComponent2D* Capture,
        int32 Width,
        int32 Height,
        UMaterialInterface* PostMaterial,
        bool bForceLinearGamma,
        TArray<FColor>& OutPixels)
    {
        if (!Capture || !PostMaterial || Width <= 0 || Height <= 0)
        {
            return false;
        }

        UTextureRenderTarget2D* CaptureRT = AcquireColorRenderTarget(Width, Height, bForceLinearGamma);
        if (!CaptureRT)
        {
            return false;
        }

        UTextureRenderTarget2D* OriginalTarget = Capture->TextureTarget;
        const ESceneCaptureSource OriginalSource = Capture->CaptureSource;
        const bool bOriginalEveryFrame = Capture->bCaptureEveryFrame;
        const FPostProcessSettings OriginalPostProcess = Capture->PostProcessSettings;
        const float OriginalPostProcessBlendWeight = Capture->PostProcessBlendWeight;

        Capture->TextureTarget = CaptureRT;
        Capture->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
        Capture->bCaptureEveryFrame = false;
        Capture->PostProcessSettings = FPostProcessSettings();
        Capture->PostProcessSettings.AddBlendable(PostMaterial, 1.0f);
        Capture->PostProcessBlendWeight = 1.0f;
        Capture->CaptureScene();

        TArray<FColor> Pixels;
        bool bReadOk = false;
        if (FTextureRenderTargetResource* Resource = CaptureRT->GameThread_GetRenderTargetResource())
        {
            FReadSurfaceDataFlags ReadFlags(RCM_UNorm, CubeFace_MAX);
            ReadFlags.SetLinearToGamma(false);
            bReadOk = Resource->ReadPixels(Pixels, ReadFlags);
        }

        Capture->TextureTarget = OriginalTarget;
        Capture->CaptureSource = OriginalSource;
        Capture->bCaptureEveryFrame = bOriginalEveryFrame;
        Capture->PostProcessSettings = OriginalPostProcess;
        Capture->PostProcessBlendWeight = OriginalPostProcessBlendWeight;

        if (!bReadOk || Pixels.Num() != Width * Height)
        {
            return false;
        }

        OutPixels = MoveTemp(Pixels);
        return true;
    }

    uint8 ComputeAirSimObjectIdFromName(const FString& Name)
    {
        int32 Sum = 0;
        for (TCHAR Ch : Name)
        {
            if (Ch >= TCHAR('A') && Ch <= TCHAR('Z'))
            {
                Ch = static_cast<TCHAR>(Ch - TCHAR('A') + TCHAR('a'));
            }

            if (Ch >= TCHAR('a') && Ch <= TCHAR('z'))
            {
                Sum += static_cast<int32>(Ch);
            }
        }

        return static_cast<uint8>(Sum % 255);
    }

    uint8 ComputeStencilIdForPrimitive(const UPrimitiveComponent* Primitive, const AActor* Owner)
    {
        if (Primitive)
        {
            const uint8 PrimitiveId = ComputeAirSimObjectIdFromName(Primitive->GetName());
            if (PrimitiveId != 0)
            {
                return PrimitiveId;
            }
        }

        if (Owner)
        {
            const uint8 ActorId = ComputeAirSimObjectIdFromName(Owner->GetName());
            if (ActorId != 0)
            {
                return ActorId;
            }
        }

        return 1;
    }

    void EnsureWorldSegmentationStencil(UWorld* World)
    {
        if (!World)
        {
            return;
        }

        const double Now = World->GetTimeSeconds();
        if (GLastSegmentationWorld.Get() == World &&
            (Now - GLastSegmentationUpdateTimeSec) < kSegmentationRefreshIntervalSec)
        {
            return;
        }

        GLastSegmentationWorld = World;
        GLastSegmentationUpdateTimeSec = Now;

        if (IConsoleVariable* CustomDepthVar = IConsoleManager::Get().FindConsoleVariable(TEXT("r.CustomDepth")))
        {
            if (CustomDepthVar->GetInt() < 3)
            {
                CustomDepthVar->Set(3, ECVF_SetByCode);
            }
        }

        const TArray<ULevel*>& Levels = World->GetLevels();
        for (ULevel* Level : Levels)
        {
            if (!Level)
            {
                continue;
            }

            for (AActor* Actor : Level->Actors)
            {
                if (!Actor || Actor->IsActorBeingDestroyed())
                {
                    continue;
                }

                TArray<UPrimitiveComponent*> PrimitiveComponents;
                Actor->GetComponents<UPrimitiveComponent>(PrimitiveComponents);
                for (UPrimitiveComponent* Primitive : PrimitiveComponents)
                {
                    if (!Primitive || !Primitive->IsRegistered())
                    {
                        continue;
                    }

                    Primitive->SetRenderCustomDepth(true);
                    if (Primitive->CustomDepthStencilValue <= 0)
                    {
                        Primitive->SetCustomDepthStencilValue(ComputeStencilIdForPrimitive(Primitive, Actor));
                    }
                }
            }
        }
    }

    bool CaptureAirSimMaterialPixels(
        USceneCaptureComponent2D* Capture,
        int32 Width,
        int32 Height,
        const TCHAR* MaterialPath,
        bool bForceLinearGamma,
        bool bNeedsSegmentationStencil,
        TArray<FColor>& OutPixels)
    {
        if (!Capture || !MaterialPath)
        {
            return false;
        }

        if (bNeedsSegmentationStencil)
        {
            EnsureWorldSegmentationStencil(Capture->GetWorld());
        }

        UMaterialInterface* Material = LoadObject<UMaterialInterface>(nullptr, MaterialPath);
        if (!Material)
        {
            static TSet<FString> MissingMaterialsLogged;
            const FString MaterialPathStr(MaterialPath);
            if (!MissingMaterialsLogged.Contains(MaterialPathStr))
            {
                MissingMaterialsLogged.Add(MaterialPathStr);
                UE_LOG(LogTemp, Warning, TEXT("[AirSimImageUtils] AirSim material missing: %s"), MaterialPath);
            }
            return false;
        }

        return CaptureWithPostProcessMaterial(Capture, Width, Height, Material, bForceLinearGamma, OutPixels);
    }

    bool ReadScenePixels(USceneCaptureComponent2D* Capture, int32 Width, int32 Height, TArray<FColor>& OutPixels)
    {
        if (!Capture || !Capture->TextureTarget || Width <= 0 || Height <= 0)
        {
            return false;
        }

        if (!Capture->bCaptureEveryFrame)
        {
            Capture->CaptureScene();
        }

        FTextureRenderTargetResource* Resource = Capture->TextureTarget->GameThread_GetRenderTargetResource();
        if (!Resource)
        {
            return false;
        }

        TArray<FColor> Pixels;
        FReadSurfaceDataFlags ReadFlags(RCM_UNorm, CubeFace_MAX);
        ReadFlags.SetLinearToGamma(false);
        if (!Resource->ReadPixels(Pixels, ReadFlags))
        {
            return false;
        }

        const int32 SourceWidth = Capture->TextureTarget->SizeX;
        const int32 SourceHeight = Capture->TextureTarget->SizeY;
        if (SourceWidth <= 0 || SourceHeight <= 0 || Pixels.Num() != SourceWidth * SourceHeight)
        {
            return false;
        }

        if (SourceWidth == Width && SourceHeight == Height)
        {
            OutPixels = MoveTemp(Pixels);
            return true;
        }

        return ResizePixelsNearest(Pixels, SourceWidth, SourceHeight, Width, Height, OutPixels);
    }

    bool CaptureDepthLinear(USceneCaptureComponent2D* Capture, int32 Width, int32 Height, TArray<FLinearColor>& OutDepthPixels)
    {
        if (!Capture || Width <= 0 || Height <= 0)
        {
            return false;
        }

        UTextureRenderTarget2D* OriginalTarget = Capture->TextureTarget;
        const ESceneCaptureSource OriginalSource = Capture->CaptureSource;
        const bool bOriginalEveryFrame = Capture->bCaptureEveryFrame;

        UTextureRenderTarget2D* DepthRT = AcquireDepthRenderTarget(Width, Height);
        if (!DepthRT)
        {
            return false;
        }

        Capture->TextureTarget = DepthRT;
        Capture->CaptureSource = ESceneCaptureSource::SCS_SceneDepth;
        Capture->bCaptureEveryFrame = false;
        Capture->CaptureScene();

        bool bOk = false;
        if (FTextureRenderTargetResource* Resource = DepthRT->GameThread_GetRenderTargetResource())
        {
            bOk = Resource->ReadLinearColorPixels(OutDepthPixels);
        }

        Capture->TextureTarget = OriginalTarget;
        Capture->CaptureSource = OriginalSource;
        Capture->bCaptureEveryFrame = bOriginalEveryFrame;

        return bOk && (OutDepthPixels.Num() == Width * Height);
    }

    FString EncodeBgraJpegBase64(const TArray<FColor>& Pixels, int32 Width, int32 Height, int32 Quality)
    {
        if (Pixels.Num() != Width * Height || Width <= 0 || Height <= 0)
        {
            return TEXT("");
        }

        TArray<uint8> RawData;
        RawData.SetNumUninitialized(Pixels.Num() * 4);
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
            return TEXT("");
        }

        if (!Wrapper->SetRaw(RawData.GetData(), RawData.Num(), Width, Height, ERGBFormat::BGRA, 8))
        {
            return TEXT("");
        }

        const TArray64<uint8>& JpegData = Wrapper->GetCompressed(FMath::Clamp(Quality, 1, 100));
        return FBase64::Encode(JpegData.GetData(), JpegData.Num());
    }

    void ConvertDepthToPlanar(const TArray<FLinearColor>& DepthPixels, int32 Width, int32 Height, float MaxDepthMeters, TArray<FColor>& OutPixels)
    {
        OutPixels.SetNumUninitialized(Width * Height);
        const float InvMaxDepth = (MaxDepthMeters > KINDA_SMALL_NUMBER) ? (1.0f / MaxDepthMeters) : 1.0f;

        for (int32 i = 0; i < DepthPixels.Num(); ++i)
        {
            const float DepthMeters = FMath::Max(0.0f, DepthPixels[i].R / 100.0f);
            const float Normalized = FMath::Clamp(DepthMeters * InvMaxDepth, 0.0f, 1.0f);
            const uint8 Gray = static_cast<uint8>(FMath::RoundToInt(Normalized * 255.0f));
            OutPixels[i] = FColor(Gray, Gray, Gray, 255);
        }
    }

    void ConvertDepthToVis(const TArray<FLinearColor>& DepthPixels, int32 Width, int32 Height, TArray<FColor>& OutPixels)
    {
        OutPixels.SetNumUninitialized(Width * Height);

        const float InvRange = 1.0f / kDepthVisRangeMeters;
        for (int32 i = 0; i < DepthPixels.Num(); ++i)
        {
            const float DepthMeters = FMath::Max(0.0f, DepthPixels[i].R / 100.0f);
            const float Normalized = FMath::Clamp(DepthMeters * InvRange, 0.0f, 1.0f);
            const uint8 Gray = static_cast<uint8>(FMath::RoundToInt(Normalized * 255.0f));
            OutPixels[i] = FColor(Gray, Gray, Gray, 255);
        }
    }

    uint8 DecodeStencilId(const FColor& Raw)
    {
        const bool bLooksGray = (FMath::Abs(static_cast<int32>(Raw.R) - static_cast<int32>(Raw.G)) <= 2) &&
            (FMath::Abs(static_cast<int32>(Raw.R) - static_cast<int32>(Raw.B)) <= 2);

        if (bLooksGray)
        {
            return Raw.R;
        }

        return static_cast<uint8>(FMath::Max3(static_cast<int32>(Raw.R), static_cast<int32>(Raw.G), static_cast<int32>(Raw.B)));
    }

    void DenoiseStencilIds(TArray<uint8>& InOutStencilIds, int32 Width, int32 Height)
    {
        if (InOutStencilIds.Num() != Width * Height || Width < 3 || Height < 3)
        {
            return;
        }

        const TArray<uint8> SourceIds = InOutStencilIds;
        for (int32 Y = 1; Y < Height - 1; ++Y)
        {
            for (int32 X = 1; X < Width - 1; ++X)
            {
                uint8 CandidateValues[9] = {};
                uint8 CandidateCounts[9] = {};
                int32 CandidateNum = 0;

                for (int32 DY = -1; DY <= 1; ++DY)
                {
                    for (int32 DX = -1; DX <= 1; ++DX)
                    {
                        const uint8 Value = SourceIds[(Y + DY) * Width + (X + DX)];
                        int32 FoundIndex = INDEX_NONE;
                        for (int32 I = 0; I < CandidateNum; ++I)
                        {
                            if (CandidateValues[I] == Value)
                            {
                                FoundIndex = I;
                                break;
                            }
                        }

                        if (FoundIndex == INDEX_NONE)
                        {
                            FoundIndex = CandidateNum++;
                            CandidateValues[FoundIndex] = Value;
                            CandidateCounts[FoundIndex] = 0;
                        }

                        CandidateCounts[FoundIndex] = static_cast<uint8>(CandidateCounts[FoundIndex] + 1);
                    }
                }

                int32 BestIndex = 0;
                for (int32 I = 1; I < CandidateNum; ++I)
                {
                    if (CandidateCounts[I] > CandidateCounts[BestIndex])
                    {
                        BestIndex = I;
                    }
                }

                if (CandidateCounts[BestIndex] >= 4)
                {
                    InOutStencilIds[Y * Width + X] = CandidateValues[BestIndex];
                }
            }
        }
    }


    void RemoveStencilOverlayGlyphs(TArray<FColor>& InOutPixels, int32 Width, int32 Height)
    {
        if (InOutPixels.Num() != Width * Height || Width < 3 || Height < 3)
        {
            return;
        }

        const TArray<FColor> SourcePixels = InOutPixels;
        constexpr int32 Radius = 2;
        for (int32 Y = 0; Y < Height; ++Y)
        {
            const int32 Y0 = FMath::Max(0, Y - Radius);
            const int32 Y1 = FMath::Min(Height - 1, Y + Radius);
            for (int32 X = 0; X < Width; ++X)
            {
                const int32 X0 = FMath::Max(0, X - Radius);
                const int32 X1 = FMath::Min(Width - 1, X + Radius);

                uint8 MaxR = 0;
                uint8 MaxG = 0;
                uint8 MaxB = 0;
                for (int32 SY = Y0; SY <= Y1; ++SY)
                {
                    for (int32 SX = X0; SX <= X1; ++SX)
                    {
                        const FColor& P = SourcePixels[SY * Width + SX];
                        MaxR = FMath::Max(MaxR, P.R);
                        MaxG = FMath::Max(MaxG, P.G);
                        MaxB = FMath::Max(MaxB, P.B);
                    }
                }

                FColor& Out = InOutPixels[Y * Width + X];
                Out.R = MaxR;
                Out.G = MaxG;
                Out.B = MaxB;
                Out.A = 255;
            }
        }
    }

    bool CaptureCustomStencilIds(USceneCaptureComponent2D* Capture, int32 Width, int32 Height, TArray<uint8>& OutStencilIds, int32& OutNonZeroCount)
    {
        OutStencilIds.Reset();
        OutNonZeroCount = 0;

        if (!Capture || Width <= 0 || Height <= 0)
        {
            return false;
        }

        EnsureWorldSegmentationStencil(Capture->GetWorld());

        static TWeakObjectPtr<UMaterialInterface> CachedStencilVizMaterial;
        if (!CachedStencilVizMaterial.IsValid())
        {
            CachedStencilVizMaterial = LoadObject<UMaterialInterface>(nullptr, TEXT("/Engine/BufferVisualization/CustomStencil.CustomStencil"));
        }

        UMaterialInterface* StencilVizMaterial = CachedStencilVizMaterial.Get();
        if (!StencilVizMaterial)
        {
            return false;
        }

        UTextureRenderTarget2D* StencilRT = AcquireColorRenderTarget(Width, Height);
        if (!StencilRT)
        {
            return false;
        }

        UTextureRenderTarget2D* OriginalTarget = Capture->TextureTarget;
        const ESceneCaptureSource OriginalSource = Capture->CaptureSource;
        const bool bOriginalEveryFrame = Capture->bCaptureEveryFrame;
        const FPostProcessSettings OriginalPostProcess = Capture->PostProcessSettings;
        const float OriginalPostProcessBlendWeight = Capture->PostProcessBlendWeight;

        Capture->TextureTarget = StencilRT;
        Capture->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
        Capture->bCaptureEveryFrame = false;
        Capture->PostProcessSettings = FPostProcessSettings();
        Capture->PostProcessSettings.AddBlendable(StencilVizMaterial, 1.0f);
        Capture->PostProcessBlendWeight = 1.0f;
        Capture->CaptureScene();

        bool bReadOk = false;
        TArray<FColor> StencilPixels;
        if (FTextureRenderTargetResource* Resource = StencilRT->GameThread_GetRenderTargetResource())
        {
            FReadSurfaceDataFlags ReadFlags(RCM_UNorm, CubeFace_MAX);
            ReadFlags.SetLinearToGamma(false);
            bReadOk = Resource->ReadPixels(StencilPixels, ReadFlags);
        }

        Capture->TextureTarget = OriginalTarget;
        Capture->CaptureSource = OriginalSource;
        Capture->bCaptureEveryFrame = bOriginalEveryFrame;
        Capture->PostProcessSettings = OriginalPostProcess;
        Capture->PostProcessBlendWeight = OriginalPostProcessBlendWeight;

        if (!bReadOk || StencilPixels.Num() != Width * Height)
        {
            return false;
        }

        // UE buffer visualization can contain tiny numeric glyph overlays on stencil view.
        // Remove high-frequency dark glyphs before decoding stencil ids.
        RemoveStencilOverlayGlyphs(StencilPixels, Width, Height);

        OutStencilIds.SetNumUninitialized(Width * Height);
        for (int32 i = 0; i < StencilPixels.Num(); ++i)
        {
            OutStencilIds[i] = DecodeStencilId(StencilPixels[i]);
        }

        DenoiseStencilIds(OutStencilIds, Width, Height);

        OutNonZeroCount = 0;
        for (const uint8 StencilId : OutStencilIds)
        {
            if (StencilId > 0)
            {
                ++OutNonZeroCount;
            }
        }

        return true;
    }

    FColor AirSimSegmentationColorFromId(uint8 Id)
    {
        if (Id == 0)
        {
            return FColor::Black;
        }

        const uint32 X = static_cast<uint32>(Id);
        const uint8 R = static_cast<uint8>((X * 37u + 13u) % 255u);
        const uint8 G = static_cast<uint8>((X * 73u + 47u) % 255u);
        const uint8 B = static_cast<uint8>((X * 109u + 91u) % 255u);
        return FColor(FMath::Max<uint8>(R, 16), FMath::Max<uint8>(G, 16), FMath::Max<uint8>(B, 16), 255);
    }

    bool CaptureCustomStencilSegmentation(USceneCaptureComponent2D* Capture, int32 Width, int32 Height, TArray<FColor>& OutPixels)
    {
        TArray<uint8> StencilIds;
        int32 NonZeroCount = 0;
        if (!CaptureCustomStencilIds(Capture, Width, Height, StencilIds, NonZeroCount))
        {
            return false;
        }

        OutPixels.SetNumUninitialized(Width * Height);
        for (int32 i = 0; i < StencilIds.Num(); ++i)
        {
            OutPixels[i] = AirSimSegmentationColorFromId(StencilIds[i]);
        }

        const int32 MinForegroundPixels = FMath::Max(8, (Width * Height) / 500);
        return NonZeroCount >= MinForegroundPixels;
    }

    bool CaptureCustomStencilInfrared(USceneCaptureComponent2D* Capture, int32 Width, int32 Height, TArray<FColor>& OutPixels)
    {
        TArray<uint8> StencilIds;
        int32 NonZeroCount = 0;
        if (!CaptureCustomStencilIds(Capture, Width, Height, StencilIds, NonZeroCount))
        {
            return false;
        }

        OutPixels.SetNumUninitialized(Width * Height);
        for (int32 i = 0; i < StencilIds.Num(); ++i)
        {
            const uint8 Gray = StencilIds[i];
            OutPixels[i] = FColor(Gray, Gray, Gray, 255);
        }

        return NonZeroCount > 0;
    }

    void ConvertToPseudoSegmentation(TArray<FColor>& InOutPixels, int32 Width, int32 Height)
    {
        static const FColor Palette[12] =
        {
            FColor(255, 0, 0),
            FColor(255, 128, 0),
            FColor(255, 255, 0),
            FColor(128, 255, 0),
            FColor(0, 255, 0),
            FColor(0, 255, 128),
            FColor(0, 255, 255),
            FColor(0, 128, 255),
            FColor(0, 0, 255),
            FColor(128, 0, 255),
            FColor(255, 0, 255),
            FColor(255, 0, 128)
        };

        if (InOutPixels.Num() != Width * Height || Width <= 0 || Height <= 0)
        {
            return;
        }

        for (int32 Index = 0; Index < InOutPixels.Num(); ++Index)
        {
            const FColor& Src = InOutPixels[Index];
            const FLinearColor Rgb(Src.R / 255.0f, Src.G / 255.0f, Src.B / 255.0f, 1.0f);
            const FLinearColor Hsv = Rgb.LinearRGBToHSV();

            FColor SegColor;
            if (Hsv.B < 0.10f)
            {
                SegColor = FColor::Black;
            }
            else if (Hsv.G < 0.10f)
            {
                const uint8 Gray = static_cast<uint8>(FMath::RoundToInt(FMath::Clamp(Hsv.B, 0.0f, 1.0f) * 255.0f));
                const uint8 Bucket = static_cast<uint8>((Gray / 48) * 48);
                SegColor = FColor(Bucket, Bucket, Bucket, 255);
            }
            else
            {
                const int32 HueBucket = FMath::Clamp(FMath::FloorToInt(Hsv.R / 30.0f), 0, 11);
                SegColor = Palette[HueBucket];
                SegColor.A = 255;
            }

            InOutPixels[Index] = SegColor;
        }
    }

    void ConvertToInfraredFallback(TArray<FColor>& InOutPixels)
    {
        for (FColor& Pixel : InOutPixels)
        {
            const uint8 Gray = static_cast<uint8>(
                FMath::Clamp(
                    FMath::RoundToInt(0.299f * Pixel.R + 0.587f * Pixel.G + 0.114f * Pixel.B),
                    0,
                    255));
            Pixel = FColor(Gray, Gray, Gray, 255);
        }
    }
}

/**
 * @brief 解析图像类型输入
 * @param RawType 外部传入的类型字符串或数值字符串
 * @param OutType 输出图像类型
 * @return 识别成功时返回 `true`
 */
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

    if (Normalized == TEXT("depth") || Normalized == TEXT("depthplanar") || Normalized == TEXT("depth_planar") || Normalized == TEXT("1"))
    {
        OutType = EImageType::DepthPlanar;
        return true;
    }

    if (Normalized == TEXT("depthvis") || Normalized == TEXT("depth_vis") || Normalized == TEXT("3"))
    {
        OutType = EImageType::DepthVis;
        return true;
    }

    if (Normalized == TEXT("segmentation") || Normalized == TEXT("segment") || Normalized == TEXT("seg") || Normalized == TEXT("5"))
    {
        OutType = EImageType::Segmentation;
        return true;
    }

    if (Normalized == TEXT("infrared") || Normalized == TEXT("ir") || Normalized == TEXT("7"))
    {
        OutType = EImageType::Infrared;
        return true;
    }

    return false;
}

/** @brief 将图像类型转换为协议标准字符串 */
FString AirSimImageUtils::ToCanonicalString(EImageType Type)
{
    switch (Type)
    {
    case EImageType::Scene: return TEXT("scene");
    case EImageType::DepthPlanar: return TEXT("depth_planar");
    case EImageType::DepthVis: return TEXT("depth_vis");
    case EImageType::Segmentation: return TEXT("segmentation");
    case EImageType::Infrared: return TEXT("infrared");
    default: return TEXT("scene");
    }
}

/** @brief 将图像类型转换为 UI 显示名称 */
FString AirSimImageUtils::ToDisplayString(EImageType Type)
{
    switch (Type)
    {
    case EImageType::Scene: return TEXT("0:Scene");
    case EImageType::DepthPlanar: return TEXT("1:DepthPlanar");
    case EImageType::DepthVis: return TEXT("3:DepthVis");
    case EImageType::Segmentation: return TEXT("5:Segmentation");
    case EImageType::Infrared: return TEXT("7:Infrared");
    default: return TEXT("0:Scene");
    }
}

/** @brief 返回当前支持的全部 AirSim 图像类型 */
TArray<AirSimImageUtils::EImageType> AirSimImageUtils::SupportedImageTypes()
{
    return {
        EImageType::Scene,
        EImageType::DepthPlanar,
        EImageType::DepthVis,
        EImageType::Segmentation,
        EImageType::Infrared
    };
}

/**
 * @brief 捕获指定类型的原始像素
 * @return 捕获成功时返回 `true`
 * 该函数统一处理 Scene、深度、分割和红外等多种图像模式。
 */
bool AirSimImageUtils::CapturePixels(
    USceneCaptureComponent2D* Capture,
    int32 Width,
    int32 Height,
    EImageType Type,
    TArray<FColor>& OutPixels,
    float MaxDepthMeters)
{
    if (!Capture || Width <= 0 || Height <= 0)
    {
        return false;
    }

    if (Type == EImageType::Scene)
    {
        return ReadScenePixels(Capture, Width, Height, OutPixels);
    }

    if (Type == EImageType::DepthPlanar)
    {
        if (CaptureAirSimMaterialPixels(Capture, Width, Height, kAirSimDepthPlanarMaterialPath, false, false, OutPixels))
        {
            return true;
        }

        TArray<FLinearColor> DepthPixels;
        if (!CaptureDepthLinear(Capture, Width, Height, DepthPixels))
        {
            return false;
        }

        ConvertDepthToPlanar(DepthPixels, Width, Height, MaxDepthMeters, OutPixels);
        return true;
    }

    if (Type == EImageType::DepthVis)
    {
        if (CaptureAirSimMaterialPixels(Capture, Width, Height, kAirSimDepthVisMaterialPath, false, false, OutPixels))
        {
            return true;
        }

        TArray<FLinearColor> DepthPixels;
        if (!CaptureDepthLinear(Capture, Width, Height, DepthPixels))
        {
            return false;
        }

        ConvertDepthToVis(DepthPixels, Width, Height, OutPixels);
        return true;
    }

    if (Type == EImageType::Segmentation)
    {
        if (CaptureAirSimMaterialPixels(Capture, Width, Height, kAirSimSegmentationMaterialPath, true, true, OutPixels))
        {
            return true;
        }

        if (CaptureCustomStencilSegmentation(Capture, Width, Height, OutPixels))
        {
            return true;
        }

        if (!ReadScenePixels(Capture, Width, Height, OutPixels))
        {
            return false;
        }

        ConvertToPseudoSegmentation(OutPixels, Width, Height);
        return true;
    }

    if (Type == EImageType::Infrared)
    {
        if (CaptureAirSimMaterialPixels(Capture, Width, Height, kAirSimInfraredMaterialPath, false, true, OutPixels))
        {
            return true;
        }

        if (CaptureCustomStencilInfrared(Capture, Width, Height, OutPixels))
        {
            return true;
        }

        if (!ReadScenePixels(Capture, Width, Height, OutPixels))
        {
            return false;
        }

        ConvertToInfraredFallback(OutPixels);
        return true;
    }

    return false;
}

/**
 * @brief 捕获指定类型图像并编码为 Base64 JPEG
 * @return Base64 图像字符串；失败时返回空串
 */
FString AirSimImageUtils::CaptureJpegBase64(
    USceneCaptureComponent2D* Capture,
    int32 Width,
    int32 Height,
    EImageType Type,
    int32 Quality,
    float MaxDepthMeters)
{
    TArray<FColor> Pixels;
    if (!CapturePixels(Capture, Width, Height, Type, Pixels, MaxDepthMeters))
    {
        return TEXT("");
    }

    return EncodeBgraJpegBase64(Pixels, Width, Height, Quality);
}


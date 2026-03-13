#include "SimHUD.h"

#include "Components/SceneCaptureComponent2D.h"
#include "Engine/Canvas.h"
#include "Engine/Texture2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Engine/World.h"
#include "Framework/Commands/InputChord.h"
#include "GameFramework/PlayerController.h"
#include "GraduationProject/Core/Manager/AgentManager.h"
#include "GraduationProject/Drone/DronePawn.h"
#include "GraduationProject/Turret/TurretPawn.h"
#include "GraduationProject/Vision/AirSimImageUtils.h"
#include "InputCoreTypes.h"

// ──── PIP 常量与工具 ────
namespace
{
    constexpr int32 kPipSlotCount = 3;       ///< PIP 窗口数量
    constexpr float kPipMargin = 20.0f;      ///< 边距 (px)
    constexpr float kPipGapY = 10.0f;        ///< 窗口间距 (px)
    constexpr float kPipLabelScale = 0.85f;  ///< 标签字体缩放
    constexpr float kPipHeaderHeight = 22.0f;///< 标题栏高度 (px)

    /** @brief Agent 排序优先级（用于 PIP 自动分配） */
    int32 GetPipAgentPriority(const FString& AgentId)
    {
        if (AgentId.Equals(TEXT("drone_0"), ESearchCase::IgnoreCase))
        {
            return 0;
        }
        if (AgentId.StartsWith(TEXT("drone_"), ESearchCase::IgnoreCase))
        {
            return 10;
        }
        if (AgentId.Equals(TEXT("turret_0"), ESearchCase::IgnoreCase))
        {
            return 20;
        }
        if (AgentId.StartsWith(TEXT("turret_"), ESearchCase::IgnoreCase))
        {
            return 30;
        }
        return 40;
    }
}

// ──── 快捷键绑定 ────

/** @brief 绑定 PIP 快捷键：1/2/3 切换窗口，Ctrl+0/1/3/5/7 切换图像类型 */
void ASimHUD::BindPipHotkeys()
{
    if (bPipHotkeysBound)
    {
        return;
    }

    APlayerController* PC = ResolvePlayerController();
    if (!PC)
    {
        UE_LOG(LogTemp, Warning, TEXT("[PIP] Cannot bind hotkeys: no PlayerController"));
        return;
    }

    EnableInput(PC);
    if (!InputComponent)
    {
        UE_LOG(LogTemp, Warning, TEXT("[PIP] Cannot bind hotkeys: InputComponent is null"));
        return;
    }

    BindPipSlotKeys();
    BindPipImageTypeKeys();

    bPipHotkeysBound = true;
    UE_LOG(LogTemp, Log, TEXT("[PIP] Hotkeys bound: 1/2/3 slot toggle, Ctrl+0/1/3/5/7 image type"));
}

/** @brief 绑定 1/2/3 键到 PIP 窗口切换 */
void ASimHUD::BindPipSlotKeys()
{
    InputComponent->BindKey(EKeys::One, IE_Pressed, this, &ASimHUD::OnHotkeyTogglePip1);
    InputComponent->BindKey(EKeys::NumPadOne, IE_Pressed, this, &ASimHUD::OnHotkeyTogglePip1);
    InputComponent->BindKey(EKeys::Two, IE_Pressed, this, &ASimHUD::OnHotkeyTogglePip2);
    InputComponent->BindKey(EKeys::NumPadTwo, IE_Pressed, this, &ASimHUD::OnHotkeyTogglePip2);
    InputComponent->BindKey(EKeys::Three, IE_Pressed, this, &ASimHUD::OnHotkeyTogglePip3);
    InputComponent->BindKey(EKeys::NumPadThree, IE_Pressed, this, &ASimHUD::OnHotkeyTogglePip3);
}

/** @brief 绑定 Ctrl+数字键到图像类型切换 */
void ASimHUD::BindPipImageTypeKeys()
{
    InputComponent->BindKey(FInputChord(EKeys::Zero, false, true, false, false), IE_Pressed, this, &ASimHUD::OnHotkeySetImageTypeScene);
    InputComponent->BindKey(FInputChord(EKeys::One, false, true, false, false), IE_Pressed, this, &ASimHUD::OnHotkeySetImageTypeDepthPlanar);
    InputComponent->BindKey(FInputChord(EKeys::Three, false, true, false, false), IE_Pressed, this, &ASimHUD::OnHotkeySetImageTypeDepthVis);
    InputComponent->BindKey(FInputChord(EKeys::Five, false, true, false, false), IE_Pressed, this, &ASimHUD::OnHotkeySetImageTypeSegmentation);
    InputComponent->BindKey(FInputChord(EKeys::Seven, false, true, false, false), IE_Pressed, this, &ASimHUD::OnHotkeySetImageTypeInfrared);
}

// ──── 快捷键回调 ────

void ASimHUD::OnHotkeyTogglePip1()
{
    TogglePipSlot(0);
}

void ASimHUD::OnHotkeyTogglePip2()
{
    TogglePipSlot(1);
}

void ASimHUD::OnHotkeyTogglePip3()
{
    TogglePipSlot(2);
}

void ASimHUD::OnHotkeySetImageTypeScene()
{
    SetActivePipSlotImageType(AirSimImageUtils::EImageType::Scene);
}

void ASimHUD::OnHotkeySetImageTypeDepthPlanar()
{
    SetActivePipSlotImageType(AirSimImageUtils::EImageType::DepthPlanar);
}

void ASimHUD::OnHotkeySetImageTypeDepthVis()
{
    SetActivePipSlotImageType(AirSimImageUtils::EImageType::DepthVis);
}

void ASimHUD::OnHotkeySetImageTypeSegmentation()
{
    SetActivePipSlotImageType(AirSimImageUtils::EImageType::Segmentation);
}

void ASimHUD::OnHotkeySetImageTypeInfrared()
{
    SetActivePipSlotImageType(AirSimImageUtils::EImageType::Infrared);
}

/** @brief 将图像类型应用到当前激活的 PIP 窗口 */
void ASimHUD::SetActivePipSlotImageType(AirSimImageUtils::EImageType ImageType)
{
    EnsurePipInitialized();
    if (!PipSlots.IsValidIndex(ActivePipSlotIndex))
    {
        return;
    }

    SetPipImageType(ActivePipSlotIndex, static_cast<int32>(ImageType));
}

// ──── PIP 控制接口 ────

/** @brief 切换指定 PIP 窗口显隐，并设为当前激活窗口 */
void ASimHUD::TogglePipSlot(int32 SlotIndex)
{
    EnsurePipInitialized();
    if (!PipSlots.IsValidIndex(SlotIndex))
    {
        return;
    }

    ActivePipSlotIndex = SlotIndex;

    FPipSlotState& Slot = PipSlots[SlotIndex];
    Slot.bVisible = !Slot.bVisible;

    RefreshPipSourcesIfNeeded();
    if (Slot.bVisible && Slot.AgentId.IsEmpty())
    {
        Slot.AgentId = FindNextAgentWithCamera();
    }

    UE_LOG(LogTemp, Log, TEXT("[PIP] Slot %d -> %s (%s, %s)"),
        SlotIndex + 1,
        Slot.bVisible ? TEXT("Visible") : TEXT("Hidden"),
        *Slot.AgentId,
        *GetImageTypeLabel(NormalizeImageType(Slot.ImageType)));
}

/** @brief 设置 PIP 窗口的数据源 Agent */
void ASimHUD::SetPipSource(int32 SlotIndex, const FString& AgentId)
{
    EnsurePipInitialized();
    if (!PipSlots.IsValidIndex(SlotIndex))
    {
        return;
    }

    FPipSlotState& Slot = PipSlots[SlotIndex];
    Slot.AgentId = AgentId;
    Slot.LastUpdateTimeSec = -1.0f;
}

/** @brief 设置 PIP 窗口的图像类型 */
void ASimHUD::SetPipImageType(int32 SlotIndex, int32 ImageType)
{
    EnsurePipInitialized();
    if (!PipSlots.IsValidIndex(SlotIndex))
    {
        return;
    }

    const AirSimImageUtils::EImageType Normalized = NormalizeImageType(ImageType);
    FPipSlotState& Slot = PipSlots[SlotIndex];
    Slot.ImageType = static_cast<int32>(Normalized);
    Slot.LastUpdateTimeSec = -1.0f;

    UE_LOG(LogTemp, Log, TEXT("[PIP] Slot %d image_type -> %s"), SlotIndex + 1, *GetImageTypeLabel(Normalized));
}

// ──── PIP 绘制 ────

/** @brief 绘制所有可见的 PIP 窗口 */
void ASimHUD::DrawPipWindows()
{
    EnsurePipInitialized();

    if (!Canvas || PipSlots.Num() == 0)
    {
        return;
    }

    for (int32 Index = 0; Index < PipSlots.Num(); ++Index)
    {
        FPipSlotState& Slot = PipSlots[Index];
        if (!Slot.bVisible)
        {
            continue;
        }

        const float X = Canvas->ClipX - Slot.Width - kPipMargin;
        const float SlotY = kPipMargin + Index * (Slot.Height + kPipGapY);
        DrawPipSlot(Index, Slot, X, SlotY);
    }
}

/** @brief 绘制单个 PIP 窗口（纹理 + 标题栏） */
void ASimHUD::DrawPipSlot(int32 SlotIndex, FPipSlotState& Slot, float X, float Y)
{
    if (UTexture* Texture = ResolvePipTexture(Slot))
    {
        DrawTexture(
            Texture,
            X, Y,
            Slot.Width, Slot.Height,
            0.0f, 0.0f,
            1.0f, 1.0f,
            FLinearColor::White,
            BLEND_Opaque,
            1.0f,
            false,
            0.0f,
            FVector2D::ZeroVector);
    }
    else
    {
        DrawRect(FLinearColor(0.05f, 0.05f, 0.05f, 0.85f), X, Y, Slot.Width, Slot.Height);
        float EmptyTextY = Y + Slot.Height * 0.45f;
        DrawTextLine(TEXT("No camera source"), X + 10.0f, EmptyTextY, FLinearColor::Yellow, 1.0f);
    }

    DrawRect(FLinearColor(0.0f, 0.0f, 0.0f, 0.40f), X, Y, Slot.Width, kPipHeaderHeight);

    const FString SlotAgentLabel = Slot.AgentId.IsEmpty() ? TEXT("<none>") : Slot.AgentId;
    const FString TypeLabel = GetImageTypeLabel(NormalizeImageType(Slot.ImageType));
    const FString ActiveTag = (SlotIndex == ActivePipSlotIndex) ? TEXT("*") : TEXT(" ");
    DrawText(
        FString::Printf(TEXT("[%d%s] %s | %s"), SlotIndex + 1, *ActiveTag, *SlotAgentLabel, *TypeLabel),
        FLinearColor::White,
        X + 8.0f,
        Y + 2.0f,
        nullptr,
        kPipLabelScale);
}

// ──── Agent 排序与自动分配 ────

/** @brief 获取所有有相机的 Agent ID（按优先级排序） */
TArray<FString> ASimHUD::GetSortedCameraAgentIds() const
{
    UAgentManager* Manager = UAgentManager::GetInstance();
    if (!Manager)
    {
        return {};
    }

    const TArray<FString> Ids = Manager->GetAllAgentIds();
    TArray<FString> CameraIds;
    CameraIds.Reserve(Ids.Num());

    for (const FString& Id : Ids)
    {
        if (!Id.IsEmpty() && ResolvePipRenderTarget(Id))
        {
            CameraIds.Add(Id);
        }
    }

    CameraIds.Sort([](const FString& A, const FString& B)
    {
        const int32 PriorityA = GetPipAgentPriority(A);
        const int32 PriorityB = GetPipAgentPriority(B);
        if (PriorityA != PriorityB)
        {
            return PriorityA < PriorityB;
        }
        return A < B;
    });

    return CameraIds;
}

/** @brief 刷新 PIP 窗口的 Agent 分配：清理失效绑定、自动填充空窗口 */
void ASimHUD::RefreshPipSourcesIfNeeded()
{
    if (PipSlots.Num() == 0)
    {
        return;
    }

    const TArray<FString> OrderedIds = GetSortedCameraAgentIds();
    TSet<FString> AvailableIds(OrderedIds);
    TSet<FString> UsedIds;

    for (FPipSlotState& Slot : PipSlots)
    {
        if (Slot.AgentId.IsEmpty() || !AvailableIds.Contains(Slot.AgentId) || UsedIds.Contains(Slot.AgentId))
        {
            Slot.AgentId = TEXT("");
            continue;
        }

        UsedIds.Add(Slot.AgentId);
    }

    for (FPipSlotState& Slot : PipSlots)
    {
        if (!Slot.AgentId.IsEmpty())
        {
            continue;
        }

        for (const FString& CandidateId : OrderedIds)
        {
            if (UsedIds.Contains(CandidateId))
            {
                continue;
            }

            Slot.AgentId = CandidateId;
            UsedIds.Add(CandidateId);
            break;
        }
    }
}

// ──── PIP 初始化 ────

/** @brief 惰性初始化 PIP 窗口，默认 Scene / DepthVis / Segmentation 三路 */
void ASimHUD::EnsurePipInitialized()
{
    if (!bPipInitialized)
    {
        PipSlots.SetNum(kPipSlotCount);

        const int32 DefaultTypes[kPipSlotCount] =
        {
            static_cast<int32>(AirSimImageUtils::EImageType::Scene),
            static_cast<int32>(AirSimImageUtils::EImageType::DepthVis),
            static_cast<int32>(AirSimImageUtils::EImageType::Segmentation)
        };

        for (int32 i = 0; i < PipSlots.Num(); ++i)
        {
            InitializePipSlot(PipSlots[i], DefaultTypes[i]);
        }

        ActivePipSlotIndex = 0;
        bPipInitialized = true;
    }

    RefreshPipSourcesIfNeeded();
}

/** @brief 初始化单个 PIP 窗口状态为默认值 */
void ASimHUD::InitializePipSlot(FPipSlotState& Slot, int32 DefaultImageType)
{
    Slot.bVisible = false;
    Slot.Width = 320.0f;
    Slot.Height = 180.0f;
    Slot.AgentId = TEXT("");
    Slot.ImageType = DefaultImageType;
    Slot.ProcessedTexture = nullptr;
    Slot.LastUpdateTimeSec = -1.0f;
    Slot.WorkingPixels.Reset();
}

/** @brief 查找下一个有相机的 Agent（用于 PIP 自动分配） */
FString ASimHUD::FindNextAgentWithCamera(const FString& CurrentId) const
{
    const TArray<FString> Ids = GetSortedCameraAgentIds();
    if (Ids.Num() == 0)
    {
        return TEXT("");
    }

    int32 StartIndex = 0;
    if (!CurrentId.IsEmpty())
    {
        const int32 Found = Ids.IndexOfByKey(CurrentId);
        if (Found != INDEX_NONE)
        {
            StartIndex = (Found + 1) % Ids.Num();
        }
    }

    return Ids[StartIndex];
}

// ──── 渲染目标解析 ────

/** @brief 解析 Agent 的 RenderTarget（无人机或炮塔的 SceneCapture 组件） */
UTextureRenderTarget2D* ASimHUD::ResolvePipRenderTarget(const FString& AgentId) const
{
    if (AgentId.IsEmpty())
    {
        return nullptr;
    }

    UAgentManager* Manager = UAgentManager::GetInstance();
    if (!Manager)
    {
        return nullptr;
    }

    AActor* Agent = Manager->GetAgent(AgentId);
    if (!Agent)
    {
        return nullptr;
    }

    if (const ADronePawn* Drone = Cast<ADronePawn>(Agent))
    {
        return Drone->DroneSceneCapture ? Drone->DroneSceneCapture->TextureTarget : nullptr;
    }

    if (const ATurretPawn* Turret = Cast<ATurretPawn>(Agent))
    {
        return Turret->TurretSceneCapture ? Turret->TurretSceneCapture->TextureTarget : nullptr;
    }

    return nullptr;
}

/** @brief 解析 Agent 的 SceneCaptureComponent2D + 图像尺寸 */
bool ASimHUD::ResolvePipCaptureInfo(const FString& AgentId, USceneCaptureComponent2D*& OutCapture, int32& OutWidth, int32& OutHeight) const
{
    OutCapture = nullptr;
    OutWidth = 0;
    OutHeight = 0;

    if (AgentId.IsEmpty())
    {
        return false;
    }

    UAgentManager* Manager = UAgentManager::GetInstance();
    if (!Manager)
    {
        return false;
    }

    AActor* Agent = Manager->GetAgent(AgentId);
    if (!Agent)
    {
        return false;
    }

    if (const ADronePawn* Drone = Cast<ADronePawn>(Agent))
    {
        if (!Drone->DroneSceneCapture)
        {
            return false;
        }

        OutCapture = Drone->DroneSceneCapture;
        OutWidth = Drone->CameraWidth;
        OutHeight = Drone->CameraHeight;
        return true;
    }

    if (const ATurretPawn* Turret = Cast<ATurretPawn>(Agent))
    {
        if (!Turret->TurretSceneCapture)
        {
            return false;
        }

        OutCapture = Turret->TurretSceneCapture;
        OutWidth = Turret->CameraWidth;
        OutHeight = Turret->CameraHeight;
        return true;
    }

    return false;
}

// ──── 纹理解析与生成 ────

/**
 * @brief 根据 PIP 窗口配置解析显示纹理
 *
 * Scene 类型直接使用 RenderTarget；
 * 其他类型通过 CapturePixels 捕获像素 → 上传到 Transient Texture2D。
 */
UTexture* ASimHUD::ResolvePipTexture(FPipSlotState& Slot)
{
    USceneCaptureComponent2D* Capture = nullptr;
    int32 SourceWidth = 0;
    int32 SourceHeight = 0;
    if (!ResolvePipCaptureInfo(Slot.AgentId, Capture, SourceWidth, SourceHeight))
    {
        return nullptr;
    }

    const AirSimImageUtils::EImageType Type = NormalizeImageType(Slot.ImageType);
    if (Type == AirSimImageUtils::EImageType::Scene)
    {
        return Capture->TextureTarget;
    }

    if (CanReuseProcessedTexture(Slot))
    {
        return Slot.ProcessedTexture;
    }

    int32 EffectiveMaxWidth = 0;
    int32 EffectiveMaxHeight = 0;
    GetNonSceneCaptureSizeLimits(Type, EffectiveMaxWidth, EffectiveMaxHeight);

    const float WidthScale = static_cast<float>(EffectiveMaxWidth) / FMath::Max(1, SourceWidth);
    const float HeightScale = static_cast<float>(EffectiveMaxHeight) / FMath::Max(1, SourceHeight);
    const float CaptureScale = FMath::Clamp(FMath::Min3(1.0f, WidthScale, HeightScale), 0.1f, 1.0f);
    const int32 CaptureWidth = FMath::Max(64, FMath::RoundToInt(SourceWidth * CaptureScale));
    const int32 CaptureHeight = FMath::Max(64, FMath::RoundToInt(SourceHeight * CaptureScale));

    if (!AirSimImageUtils::CapturePixels(Capture, CaptureWidth, CaptureHeight, Type, Slot.WorkingPixels, PipMaxDepthMeters))
    {
        return nullptr;
    }

    if (!UpdateSlotProcessedTexture(Slot, Slot.WorkingPixels, CaptureWidth, CaptureHeight, Type))
    {
        return nullptr;
    }

    if (GetWorld())
    {
        Slot.LastUpdateTimeSec = GetWorld()->GetTimeSeconds();
    }

    return Slot.ProcessedTexture;
}

/** @brief 获取非 Scene 图像的最大捕获尺寸 */
void ASimHUD::GetNonSceneCaptureSizeLimits(AirSimImageUtils::EImageType ImageType, int32& OutMaxWidth, int32& OutMaxHeight) const
{
    OutMaxWidth = NonSceneMaxWidth;
    OutMaxHeight = NonSceneMaxHeight;

    if (ImageType == AirSimImageUtils::EImageType::DepthPlanar || ImageType == AirSimImageUtils::EImageType::DepthVis)
    {
        OutMaxWidth = FMath::Min(OutMaxWidth, DepthModeMaxWidth);
        OutMaxHeight = FMath::Min(OutMaxHeight, DepthModeMaxHeight);
    }
}

/** @brief 判断缓存纹理是否仍可复用（基于刷新频率控制） */
bool ASimHUD::CanReuseProcessedTexture(const FPipSlotState& Slot) const
{
    if (NonSceneUpdateHz <= KINDA_SMALL_NUMBER || !Slot.ProcessedTexture)
    {
        return false;
    }

    const UWorld* World = GetWorld();
    if (!World || Slot.LastUpdateTimeSec < 0.0f)
    {
        return false;
    }

    const float MinUpdateInterval = 1.0f / NonSceneUpdateHz;
    return (World->GetTimeSeconds() - Slot.LastUpdateTimeSec) < MinUpdateInterval;
}

/**
 * @brief 将像素数据上传到 Transient Texture2D
 *
 * 如果尺寸变化则重建纹理，否则复用已有纹理。
 * 使用 UpdateTextureRegions 异步上传像素数据。
 */
bool ASimHUD::UpdateSlotProcessedTexture(FPipSlotState& Slot, const TArray<FColor>& Pixels, int32 Width, int32 Height, AirSimImageUtils::EImageType SourceType)
{
    if (Pixels.Num() != Width * Height || Width <= 0 || Height <= 0)
    {
        return false;
    }

    const TextureFilter DesiredFilter = (SourceType == AirSimImageUtils::EImageType::Segmentation) ? TF_Nearest : TF_Bilinear;

    if (!Slot.ProcessedTexture ||
        !Slot.ProcessedTexture->GetPlatformData() ||
        Slot.ProcessedTexture->GetSizeX() != Width ||
        Slot.ProcessedTexture->GetSizeY() != Height)
    {
        Slot.ProcessedTexture = UTexture2D::CreateTransient(Width, Height, PF_B8G8R8A8);
        if (!Slot.ProcessedTexture)
        {
            return false;
        }

        Slot.ProcessedTexture->MipGenSettings = TMGS_NoMipmaps;
        Slot.ProcessedTexture->CompressionSettings = TC_VectorDisplacementmap;
        Slot.ProcessedTexture->SRGB = false;
        Slot.ProcessedTexture->Filter = DesiredFilter;
        Slot.ProcessedTexture->NeverStream = true;
        Slot.ProcessedTexture->UpdateResource();
    }

    else if (Slot.ProcessedTexture->Filter != DesiredFilter)
    {
        Slot.ProcessedTexture->Filter = DesiredFilter;
        Slot.ProcessedTexture->UpdateResource();
    }

    const int64 ByteCount = static_cast<int64>(Pixels.Num()) * sizeof(FColor);
    if (ByteCount <= 0 || ByteCount > MAX_int32)
    {
        return false;
    }

    uint8* UploadData = static_cast<uint8*>(FMemory::Malloc(static_cast<SIZE_T>(ByteCount)));
    if (!UploadData)
    {
        return false;
    }
    FMemory::Memcpy(UploadData, Pixels.GetData(), static_cast<SIZE_T>(ByteCount));

    FUpdateTextureRegion2D* Region = new FUpdateTextureRegion2D(0, 0, 0, 0, Width, Height);
    Slot.ProcessedTexture->UpdateTextureRegions(
        0,
        1,
        Region,
        Width * sizeof(FColor),
        sizeof(FColor),
        UploadData,
        [](uint8* SrcData, const FUpdateTextureRegion2D* Regions)
        {
            if (SrcData)
            {
                FMemory::Free(SrcData);
            }
            if (Regions)
            {
                delete Regions;
            }
        });

    return true;
}

// ──── 图像类型工具 ────

/** @brief 将整数类型码规范化为图像枚举 */
AirSimImageUtils::EImageType ASimHUD::NormalizeImageType(int32 ImageType) const
{
    switch (ImageType)
    {
    case 0: return AirSimImageUtils::EImageType::Scene;
    case 1: return AirSimImageUtils::EImageType::DepthPlanar;
    case 3: return AirSimImageUtils::EImageType::DepthVis;
    case 5: return AirSimImageUtils::EImageType::Segmentation;
    case 7: return AirSimImageUtils::EImageType::Infrared;
    default: return AirSimImageUtils::EImageType::Scene;
    }
}

/** @brief 获取图像类型的可读标签 */
FString ASimHUD::GetImageTypeLabel(AirSimImageUtils::EImageType ImageType) const
{
    return AirSimImageUtils::ToDisplayString(ImageType);
}

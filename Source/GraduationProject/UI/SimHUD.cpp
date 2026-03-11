#include "SimHUD.h"
#include "AgentListWidget.h"

#include "Components/SceneCaptureComponent2D.h"
#include "Engine/Canvas.h"
#include "Engine/Font.h"
#include "Engine/Texture2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Engine/World.h"
#include "Blueprint/UserWidget.h"
#include "Framework/Commands/InputChord.h"
#include "GameFramework/PlayerController.h"
#include "GraduationProject/Core/CameraPawn.h"
#include "GraduationProject/Core/Manager/AgentManager.h"
#include "GraduationProject/Drone/DronePawn.h"
#include "GraduationProject/Turret/TurretPawn.h"
#include "GraduationProject/Vision/AirSimImageUtils.h"
#include "InputCoreTypes.h"

namespace
{
    constexpr int32 kPipSlotCount = 3;

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

/**
 * @brief 初始化 HUD、PIP 状态和 Agent 列表控件
 * 进入游戏后绑定快捷键，并根据配置决定是否创建 Agent 列表 UI。
 */
void ASimHUD::BeginPlay()
{
    Super::BeginPlay();
    EnsureAgentListWidget();
    BindPipHotkeys();
}


/** @brief 确保 Agent 列表控件已创建并附加到视口 */
void ASimHUD::EnsureAgentListWidget()
{
    if (!bEnableAgentListWidget || AgentListWidget)
    {
        return;
    }

    APlayerController* PC = PlayerOwner;
    if (!PC && GetWorld())
    {
        PC = GetWorld()->GetFirstPlayerController();
    }

    if (!PC)
    {
        UE_LOG(LogTemp, Warning, TEXT("[AgentList] Cannot create widget: no PlayerController"));
        return;
    }

    AgentListWidget = CreateWidget<UAgentListWidget>(PC, UAgentListWidget::StaticClass());
    if (!AgentListWidget)
    {
        UE_LOG(LogTemp, Warning, TEXT("[AgentList] Failed to create widget"));
        return;
    }

    if (ACameraPawn* CameraPawn = Cast<ACameraPawn>(PC->GetPawn()))
    {
        AgentListWidget->SetCameraPawn(CameraPawn);
    }

    AgentListWidget->AddToViewport(30);

    if (bEnableAgentListMouse)
    {
        FInputModeGameAndUI InputMode;
        InputMode.SetHideCursorDuringCapture(false);
        InputMode.SetLockMouseToViewportBehavior(EMouseLockMode::DoNotLock);
        PC->SetInputMode(InputMode);
        PC->bShowMouseCursor = true;
    }
}
/** @brief 为 PIP 开关和图像类型切换绑定快捷键 */
void ASimHUD::BindPipHotkeys()
{
    if (bPipHotkeysBound)
    {
        return;
    }

    APlayerController* PC = PlayerOwner;
    if (!PC && GetWorld())
    {
        PC = GetWorld()->GetFirstPlayerController();
    }

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

    // AirSim-like PIP slot keys: 1/2/3
    InputComponent->BindKey(EKeys::One, IE_Pressed, this, &ASimHUD::OnHotkeyTogglePip1);
    InputComponent->BindKey(EKeys::NumPadOne, IE_Pressed, this, &ASimHUD::OnHotkeyTogglePip1);
    InputComponent->BindKey(EKeys::Two, IE_Pressed, this, &ASimHUD::OnHotkeyTogglePip2);
    InputComponent->BindKey(EKeys::NumPadTwo, IE_Pressed, this, &ASimHUD::OnHotkeyTogglePip2);
    InputComponent->BindKey(EKeys::Three, IE_Pressed, this, &ASimHUD::OnHotkeyTogglePip3);
    InputComponent->BindKey(EKeys::NumPadThree, IE_Pressed, this, &ASimHUD::OnHotkeyTogglePip3);

    // Image type set keys for active slot: Ctrl+0/1/3/5/7
    InputComponent->BindKey(FInputChord(EKeys::Zero, false, true, false, false), IE_Pressed, this, &ASimHUD::OnHotkeySetImageTypeScene);
    InputComponent->BindKey(FInputChord(EKeys::One, false, true, false, false), IE_Pressed, this, &ASimHUD::OnHotkeySetImageTypeDepthPlanar);
    InputComponent->BindKey(FInputChord(EKeys::Three, false, true, false, false), IE_Pressed, this, &ASimHUD::OnHotkeySetImageTypeDepthVis);
    InputComponent->BindKey(FInputChord(EKeys::Five, false, true, false, false), IE_Pressed, this, &ASimHUD::OnHotkeySetImageTypeSegmentation);
    InputComponent->BindKey(FInputChord(EKeys::Seven, false, true, false, false), IE_Pressed, this, &ASimHUD::OnHotkeySetImageTypeInfrared);

    bPipHotkeysBound = true;
    UE_LOG(LogTemp, Log, TEXT("[PIP] Hotkeys bound: 1/2/3 slot toggle, Ctrl+0/1/3/5/7 image type"));
}

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

/**
 * @brief 将图像类型应用到当前激活的 PIP 窗口
 * @param ImageType 目标图像类型
 */
void ASimHUD::SetActivePipSlotImageType(AirSimImageUtils::EImageType ImageType)
{
    EnsurePipInitialized();
    if (!PipSlots.IsValidIndex(ActivePipSlotIndex))
    {
        return;
    }

    SetPipImageType(ActivePipSlotIndex, static_cast<int32>(ImageType));
}

/**
 * @brief 每帧绘制 HUD
 * 依次绘制文本信息、PIP 窗口和调试状态摘要。
 */
void ASimHUD::DrawHUD()
{
    Super::DrawHUD();

    float Y = 20.0f;
    DrawAgentInfo(Y);
    Y += 10.0f;
    DrawGuidanceInfo(Y);
    Y += 10.0f;
    DrawFPS(Y);

    DrawPipWindows();
}

/** @brief 绘制一行文本，并自动推进 Y 坐标 */
void ASimHUD::DrawTextLine(const FString& Text, float X, float& Y, FLinearColor Color, float Scale)
{
    DrawText(Text, Color, X, Y, nullptr, Scale);
    Y += 18.0f * Scale;
}

/** @brief 绘制 HUD 分节标题 */
void ASimHUD::DrawSectionHeader(const FString& Title, float X, float& Y)
{
    DrawText(Title, FLinearColor(0.3f, 0.8f, 1.0f), X, Y, nullptr, 1.2f);
    Y += 22.0f;
}

/** @brief 绘制当前 Agent 列表和关键状态 */
void ASimHUD::DrawAgentInfo(float& Y)
{
    const float X = 20.0f;

    UAgentManager* Manager = UAgentManager::GetInstance();
    if (!Manager)
    {
        return;
    }

    const TArray<FString> Ids = Manager->GetAllAgentIds();
    DrawSectionHeader(FString::Printf(TEXT("Agents (%d)"), Ids.Num()), X, Y);

    for (const FString& Id : Ids)
    {
        AActor* Agent = Manager->GetAgent(Id);
        if (!Agent)
        {
            continue;
        }

        const FVector Pos = Agent->GetActorLocation();
        if (ADronePawn* Drone = Cast<ADronePawn>(Agent))
        {
            const FVector Vel = Drone->GetCurrentVelocity();
            DrawTextLine(
                FString::Printf(TEXT("  [Drone] %s  pos=(%.0f, %.0f, %.0f)  vel=(%.1f, %.1f, %.1f)"),
                    *Id, Pos.X, Pos.Y, Pos.Z, Vel.X, Vel.Y, Vel.Z),
                X, Y, FLinearColor::Green);
        }
        else if (ATurretPawn* Turret = Cast<ATurretPawn>(Agent))
        {
            DrawTextLine(
                FString::Printf(TEXT("  [Turret] %s  pos=(%.0f, %.0f, %.0f)  pitch=%.1f yaw=%.1f  tracking=%s"),
                    *Id, Pos.X, Pos.Y, Pos.Z,
                    Turret->GetCurrentPitch(), Turret->GetCurrentYaw(),
                    Turret->IsTracking() ? TEXT("ON") : TEXT("off")),
                X, Y, FLinearColor::Yellow);
        }
        else
        {
            DrawTextLine(
                FString::Printf(TEXT("  [Agent] %s  pos=(%.0f, %.0f, %.0f)"), *Id, Pos.X, Pos.Y, Pos.Z),
                X, Y);
        }
    }
}

/** @brief 绘制制导和拦截相关状态摘要 */
void ASimHUD::DrawGuidanceInfo(float& Y)
{
    const float X = 20.0f;
    DrawSectionHeader(TEXT("Guidance"), X, Y);
    DrawTextLine(TEXT("  (use get_guidance_state via TCP for details)"), X, Y, FLinearColor(0.6f, 0.6f, 0.6f));
}

/** @brief 绘制帧率信息并根据 FPS 变化颜色 */
void ASimHUD::DrawFPS(float& Y)
{
    const float X = 20.0f;
    const float FPS = 1.0f / FMath::Max(GetWorld()->GetDeltaSeconds(), 0.001f);
    const FLinearColor FPSColor = (FPS >= 30.0f)
        ? FLinearColor::Green
        : (FPS >= 15.0f ? FLinearColor::Yellow : FLinearColor::Red);

    DrawTextLine(FString::Printf(TEXT("FPS: %.0f"), FPS), X, Y, FPSColor, 1.1f);
}

/**
 * @brief 切换指定 PIP 窗口显隐
 * @param SlotIndex 窗口索引
 */
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

/**
 * @brief 设置指定 PIP 窗口的数据源
 * @param SlotIndex 窗口索引
 * @param AgentId 目标 Agent ID
 */
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

/**
 * @brief 设置指定 PIP 窗口的图像类型
 * @param SlotIndex 窗口索引
 * @param ImageType 图像类型码
 */
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

/** @brief 绘制全部 PIP 窗口及其标题栏 */
void ASimHUD::DrawPipWindows()
{
    EnsurePipInitialized();

    if (!Canvas || PipSlots.Num() == 0)
    {
        return;
    }

    const float Margin = 20.0f;
    const float GapY = 10.0f;
    const float LabelScale = 0.85f;

    for (int32 Index = 0; Index < PipSlots.Num(); ++Index)
    {
        FPipSlotState& Slot = PipSlots[Index];
        if (!Slot.bVisible)
        {
            continue;
        }

        const float X = Canvas->ClipX - Slot.Width - Margin;
        const float Y = Margin + Index * (Slot.Height + GapY);

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
            float TextY = Y + Slot.Height * 0.45f;
            DrawTextLine(TEXT("No camera source"), X + 10.0f, TextY, FLinearColor::Yellow, 1.0f);
        }

        DrawRect(FLinearColor(0.0f, 0.0f, 0.0f, 0.40f), X, Y, Slot.Width, 22.0f);
        float LabelY = Y + 2.0f;
        const FString SlotAgentLabel = Slot.AgentId.IsEmpty() ? TEXT("<none>") : Slot.AgentId;
        const FString TypeLabel = GetImageTypeLabel(NormalizeImageType(Slot.ImageType));
        const FString ActiveTag = (Index == ActivePipSlotIndex) ? TEXT("*") : TEXT(" ");
        DrawTextLine(
            FString::Printf(TEXT("[%d%s] %s | %s"), Index + 1, *ActiveTag, *SlotAgentLabel, *TypeLabel),
            X + 8.0f,
            LabelY,
            FLinearColor::White,
            LabelScale);
    }
}

/** @brief 获取可提供相机输出的 Agent ID，并按显示顺序排序 */
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

/** @brief 在 Agent 集合变化后修复 PIP 的数据源绑定 */
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

/** @brief 初始化默认 PIP 槽位配置 */
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
            FPipSlotState& Slot = PipSlots[i];
            Slot.bVisible = false;
            Slot.Width = 320.0f;
            Slot.Height = 180.0f;
            Slot.AgentId = TEXT("");
            Slot.ImageType = DefaultTypes[i];
            Slot.ProcessedTexture = nullptr;
            Slot.LastUpdateTimeSec = -1.0f;
            Slot.WorkingPixels.Reset();
        }

        ActivePipSlotIndex = 0;
        bPipInitialized = true;
    }

    RefreshPipSourcesIfNeeded();
}

/** @brief 查找下一个具备相机输出能力的 Agent */
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

/** @brief 解析指定 Agent 当前使用的 RenderTarget */
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

/**
 * @brief 解析指定 Agent 的捕获组件和图像尺寸
 * @return 解析成功时返回 `true`
 */
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

/**
 * @brief 根据当前窗口配置生成要显示的纹理
 * @param Slot PIP 窗口状态
 * @return 可直接用于 HUD 绘制的纹理对象
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

    if (NonSceneUpdateHz > KINDA_SMALL_NUMBER && Slot.ProcessedTexture && GetWorld())
    {
        const float MinUpdateInterval = 1.0f / NonSceneUpdateHz;
        const float Now = GetWorld()->GetTimeSeconds();
        if (Slot.LastUpdateTimeSec >= 0.0f && (Now - Slot.LastUpdateTimeSec) < MinUpdateInterval)
        {
            return Slot.ProcessedTexture;
        }
    }

    int32 EffectiveMaxWidth = NonSceneMaxWidth;
    int32 EffectiveMaxHeight = NonSceneMaxHeight;
    if (Type == AirSimImageUtils::EImageType::DepthPlanar || Type == AirSimImageUtils::EImageType::DepthVis)
    {
        EffectiveMaxWidth = FMath::Min(EffectiveMaxWidth, DepthModeMaxWidth);
        EffectiveMaxHeight = FMath::Min(EffectiveMaxHeight, DepthModeMaxHeight);
    }

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

/**
 * @brief 将处理后的像素缓存上传到瞬态纹理
 * @param Slot PIP 窗口状态
 * @param Pixels 像素数据
 * @param Width 图像宽度
 * @param Height 图像高度
 * @param SourceType 源图像类型
 * @return 上传成功时返回 `true`
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

/** @brief 将整数类型码转换为受支持的图像类型枚举 */
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

/** @brief 获取图像类型对应的显示名称 */
FString ASimHUD::GetImageTypeLabel(AirSimImageUtils::EImageType ImageType) const
{
    return AirSimImageUtils::ToDisplayString(ImageType);
}




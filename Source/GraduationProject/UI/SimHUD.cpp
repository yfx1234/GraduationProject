#include "SimHUD.h"
#include "AgentListWidget.h"

#include "Blueprint/UserWidget.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"
#include "GraduationProject/Core/CameraPawn.h"
#include "GraduationProject/Core/Manager/AgentManager.h"
#include "GraduationProject/Drone/DronePawn.h"
#include "GraduationProject/Turret/TurretPawn.h"

// ──── 常量 ────
namespace
{
    constexpr float kHudTextX = 20.0f;  ///< HUD 文字左侧起始 X 坐标
}

// ──── 生命周期 ────

/** @brief 初始化 HUD：创建 Agent 列表控件、绑定 PIP 快捷键 */
void ASimHUD::BeginPlay()
{
    Super::BeginPlay();
    EnsureAgentListWidget();
    BindPipHotkeys();
}

/** @brief 获取 PlayerController：优先 PlayerOwner，否则回退到 FirstPlayerController */
APlayerController* ASimHUD::ResolvePlayerController() const
{
    if (PlayerOwner)
    {
        return PlayerOwner;
    }

    UWorld* World = GetWorld();
    return World ? World->GetFirstPlayerController() : nullptr;
}

/** @brief 创建并显示 Agent 列表 Slate 控件，可选启用鼠标交互 */
void ASimHUD::EnsureAgentListWidget()
{
    if (!bEnableAgentListWidget || AgentListWidget)
    {
        return;
    }

    APlayerController* PC = ResolvePlayerController();
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

// ──── 每帧绘制 ────

/** @brief 每帧绘制：Agent 状态 → 制导信息 → FPS → PIP 窗口 */
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

/** @brief 绘制一行 HUD 文字并自动推进 Y 坐标 */
void ASimHUD::DrawTextLine(const FString& Text, float X, float& Y, FLinearColor Color, float Scale)
{
    DrawText(Text, Color, X, Y, nullptr, Scale);
    Y += 18.0f * Scale;
}

/** @brief 绘制分节标题（浅蓝色，较大字号） */
void ASimHUD::DrawSectionHeader(const FString& Title, float X, float& Y)
{
    DrawText(Title, FLinearColor(0.3f, 0.8f, 1.0f), X, Y, nullptr, 1.2f);
    Y += 22.0f;
}

// ──── Agent 信息绘制 ────

/** @brief 绘制所有已注册智能体的摘要信息 */
void ASimHUD::DrawAgentInfo(float& Y)
{
    UAgentManager* Manager = UAgentManager::GetInstance();
    if (!Manager)
    {
        return;
    }

    const TArray<FString> Ids = Manager->GetAllAgentIds();
    DrawSectionHeader(FString::Printf(TEXT("Agents (%d)"), Ids.Num()), kHudTextX, Y);

    for (const FString& Id : Ids)
    {
        AActor* Agent = Manager->GetAgent(Id);
        if (!Agent)
        {
            continue;
        }

        DrawAgentSummaryLine(Id, Agent, kHudTextX, Y);
    }
}

/** @brief 根据 Actor 类型绘制单行摘要（无人机/炮塔/普通） */
void ASimHUD::DrawAgentSummaryLine(const FString& AgentId, AActor* Agent, float X, float& Y)
{
    const FVector Pos = Agent->GetActorLocation();

    if (ADronePawn* Drone = Cast<ADronePawn>(Agent))
    {
        const FVector Vel = Drone->GetCurrentVelocity();
        DrawTextLine(
            FString::Printf(TEXT("  [Drone] %s  pos=(%.0f, %.0f, %.0f)  vel=(%.1f, %.1f, %.1f)"),
                *AgentId, Pos.X, Pos.Y, Pos.Z, Vel.X, Vel.Y, Vel.Z),
            X, Y, FLinearColor::Green);
        return;
    }

    if (ATurretPawn* Turret = Cast<ATurretPawn>(Agent))
    {
        DrawTextLine(
            FString::Printf(TEXT("  [Turret] %s  pos=(%.0f, %.0f, %.0f)  pitch=%.1f yaw=%.1f  tracking=%s"),
                *AgentId, Pos.X, Pos.Y, Pos.Z,
                Turret->GetCurrentPitch(), Turret->GetCurrentYaw(),
                Turret->IsTracking() ? TEXT("ON") : TEXT("off")),
            X, Y, FLinearColor::Yellow);
        return;
    }

    DrawTextLine(
        FString::Printf(TEXT("  [Agent] %s  pos=(%.0f, %.0f, %.0f)"), *AgentId, Pos.X, Pos.Y, Pos.Z),
        X, Y);
}

// ──── 制导信息 ────

/** @brief 绘制制导状态提示 */
void ASimHUD::DrawGuidanceInfo(float& Y)
{
    DrawSectionHeader(TEXT("Guidance"), kHudTextX, Y);
    DrawTextLine(TEXT("  (use call_actor on guidance_0:GetState for details)"), kHudTextX, Y, FLinearColor(0.6f, 0.6f, 0.6f));
}

// ──── FPS ────

/** @brief 绘制帧率信息（绿色/黄色/红色根据帧率分级） */
void ASimHUD::DrawFPS(float& Y)
{
    const float FPS = 1.0f / FMath::Max(GetWorld()->GetDeltaSeconds(), 0.001f);  // 计算当前帧率
    const FLinearColor FPSColor = (FPS >= 30.0f)                                  // 根据帧率着色
        ? FLinearColor::Green
        : (FPS >= 15.0f ? FLinearColor::Yellow : FLinearColor::Red);

    DrawTextLine(FString::Printf(TEXT("FPS: %.0f"), FPS), kHudTextX, Y, FPSColor, 1.1f);
}

#include "SimHUD.h"

#include "AgentListWidget.h"
#include "Blueprint/UserWidget.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"
#include "GraduationProject/Core/CameraPawn.h"
#include "GraduationProject/Core/Manager/AgentManager.h"
#include "GraduationProject/Drone/DronePawn.h"
#include "GraduationProject/Turret/TurretPawn.h"

namespace
{
    constexpr float kHudTextX = 20.0f;
}

void ASimHUD::BeginPlay()
{
    Super::BeginPlay();
    EnsureAgentListWidget();
}

APlayerController* ASimHUD::ResolvePlayerController() const
{
    if (PlayerOwner)
    {
        return PlayerOwner;
    }

    UWorld* World = GetWorld();
    return World ? World->GetFirstPlayerController() : nullptr;
}

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

void ASimHUD::DrawHUD()
{
    Super::DrawHUD();

    float Y = 20.0f;
    DrawAgentInfo(Y);
    Y += 10.0f;
    DrawGuidanceInfo(Y);
    Y += 10.0f;
    DrawFPS(Y);
}

void ASimHUD::DrawTextLine(const FString& Text, float X, float& Y, FLinearColor Color, float Scale)
{
    DrawText(Text, Color, X, Y, nullptr, Scale);
    Y += 18.0f * Scale;
}

void ASimHUD::DrawSectionHeader(const FString& Title, float X, float& Y)
{
    DrawText(Title, FLinearColor(0.3f, 0.8f, 1.0f), X, Y, nullptr, 1.2f);
    Y += 22.0f;
}

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
                *AgentId,
                Pos.X,
                Pos.Y,
                Pos.Z,
                Turret->GetCurrentPitch(),
                Turret->GetCurrentYaw(),
                Turret->IsTracking() ? TEXT("ON") : TEXT("off")),
            X, Y, FLinearColor::Yellow);
        return;
    }

    DrawTextLine(
        FString::Printf(TEXT("  [Agent] %s  pos=(%.0f, %.0f, %.0f)"), *AgentId, Pos.X, Pos.Y, Pos.Z),
        X, Y);
}

void ASimHUD::DrawGuidanceInfo(float& Y)
{
    DrawSectionHeader(TEXT("Guidance"), kHudTextX, Y);
    DrawTextLine(TEXT("  (use call_actor on guidance_0:GetState for details)"), kHudTextX, Y, FLinearColor(0.6f, 0.6f, 0.6f));
}

void ASimHUD::DrawFPS(float& Y)
{
    const UWorld* World = GetWorld();
    if (!World)
    {
        return;
    }

    const float FPS = 1.0f / FMath::Max(World->GetDeltaSeconds(), 0.001f);
    const FLinearColor FPSColor = (FPS >= 30.0f)
        ? FLinearColor::Green
        : (FPS >= 15.0f ? FLinearColor::Yellow : FLinearColor::Red);

    DrawTextLine(FString::Printf(TEXT("FPS: %.0f"), FPS), kHudTextX, Y, FPSColor, 1.1f);
}

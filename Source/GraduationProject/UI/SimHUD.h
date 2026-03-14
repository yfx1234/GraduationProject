#pragma once

#include "CoreMinimal.h"
#include "GameFramework/HUD.h"
#include "SimHUD.generated.h"

class UAgentListWidget;
class AActor;
class APlayerController;

// Text HUD and agent list only.
UCLASS()
class GRADUATIONPROJECT_API ASimHUD : public AHUD
{
    GENERATED_BODY()

public:
    virtual void BeginPlay() override;
    virtual void DrawHUD() override;

private:
    void EnsureAgentListWidget();
    APlayerController* ResolvePlayerController() const;
    void DrawAgentInfo(float& Y);
    void DrawAgentSummaryLine(const FString& AgentId, AActor* Agent, float X, float& Y);
    void DrawGuidanceInfo(float& Y);
    void DrawFPS(float& Y);
    void DrawTextLine(const FString& Text, float X, float& Y, FLinearColor Color = FLinearColor::White, float Scale = 1.0f);
    void DrawSectionHeader(const FString& Title, float X, float& Y);

private:
    UPROPERTY(Transient)
    UAgentListWidget* AgentListWidget = nullptr;

    UPROPERTY(EditAnywhere, Category = "AgentList")
    bool bEnableAgentListWidget = true;

    UPROPERTY(EditAnywhere, Category = "AgentList")
    bool bEnableAgentListMouse = false;
};

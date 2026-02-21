#pragma once

#include "CoreMinimal.h"
#include "GameFramework/HUD.h"
#include "SimHUD.generated.h"

/**
 * 仿真调试 HUD
 * DrawHUD 显示 Agent 列表、无人机/转台状态、制导信息、FPS
 */
UCLASS()
class GRADUATIONPROJECT_API ASimHUD : public AHUD
{
    GENERATED_BODY()

public:
    virtual void DrawHUD() override;

private:
    void DrawAgentInfo(float& Y);
    void DrawGuidanceInfo(float& Y);
    void DrawFPS(float& Y);

    // 绘制辅助
    void DrawTextLine(const FString& Text, float X, float& Y, FLinearColor Color = FLinearColor::White, float Scale = 1.0f);
    void DrawSectionHeader(const FString& Title, float X, float& Y);
};

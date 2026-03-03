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
    /** @brief 每帧绘制 HUD 内容 */
    virtual void DrawHUD() override;

private:
    /**
     * @brief 绘制所有已注册智能体的状态信息
     * @param Y 当前绘制 Y 坐标，绘制完成后自动递增
     */
    void DrawAgentInfo(float& Y);

    /**
     * @brief 绘制制导系统信息
     * @param Y 当前绘制 Y 坐标，绘制完成后自动递增
     */
    void DrawGuidanceInfo(float& Y);

    /**
     * @brief 绘制帧率信息
     * @param Y 当前绘制 Y 坐标，绘制完成后自动递增
     */
    void DrawFPS(float& Y);

    /**
     * @brief 绘制一行文本并自动换行
     * @param Text 要显示的文本
     * @param X 绘制 X 坐标
     * @param Y 当前绘制 Y 坐标，绘制完成后自动递增
     * @param Color 文本颜色
     * @param Scale 文本缩放比例
     */
    void DrawTextLine(const FString& Text, float X, float& Y, FLinearColor Color = FLinearColor::White, float Scale = 1.0f);

    /**
     * @brief 绘制分区标题
     * @param Title 标题文本
     * @param X 绘制 X 坐标
     * @param Y 当前绘制 Y 坐标，绘制完成后自动递增
     */
    void DrawSectionHeader(const FString& Title, float X, float& Y);
};

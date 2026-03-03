#include "SimHUD.h"
#include "Engine/Canvas.h"
#include "Engine/Font.h"
#include "GraduationProject/Core/Manager/AgentManager.h"
#include "GraduationProject/Drone/DronePawn.h"
#include "GraduationProject/Turret/TurretPawn.h"

/**
 * @brief 每帧绘制 HUD 内容
 * 依次绘制智能体信息、制导信息和帧率
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
}

/**
 * @brief 绘制一行文本并自动递增 Y 坐标
 * @param Text 要显示的文本
 * @param X 绘制 X 坐标
 * @param Y 当前绘制 Y 坐标
 * @param Color 文本颜色
 * @param Scale 文本缩放比例
 */
void ASimHUD::DrawTextLine(const FString& Text, float X, float& Y, FLinearColor Color, float Scale)
{
    DrawText(Text, Color, X, Y, nullptr, Scale);
    Y += 18.0f * Scale;
}

/**
 * @brief 绘制分区标题
 * @param Title 标题文本
 * @param X 绘制 X 坐标
 * @param Y 当前绘制 Y 坐标
 */
void ASimHUD::DrawSectionHeader(const FString& Title, float X, float& Y)
{
    DrawText(Title, FLinearColor(0.3f, 0.8f, 1.0f), X, Y, nullptr, 1.2f);
    Y += 22.0f;
}

/**
 * @brief 绘制所有已注册智能体的状态信息
 * @param Y 当前绘制 Y 坐标
 */
void ASimHUD::DrawAgentInfo(float& Y)
{
    float X = 20.0f;
    UAgentManager* Manager = UAgentManager::GetInstance();
    if (!Manager) return;
    TArray<FString> Ids = Manager->GetAllAgentIds();
    DrawSectionHeader(FString::Printf(TEXT("Agents (%d)"), Ids.Num()), X, Y);
    for (const FString& Id : Ids)
    {
        AActor* Agent = Manager->GetAgent(Id);
        if (!Agent) continue;
        FVector Pos = Agent->GetActorLocation();
        if (ADronePawn* Drone = Cast<ADronePawn>(Agent))
        {
            FVector Vel = Drone->GetCurrentVelocity();
            DrawTextLine(FString::Printf(TEXT("  [Drone] %s  pos=(%.0f, %.0f, %.0f)  vel=(%.1f, %.1f, %.1f)"),
                *Id, Pos.X, Pos.Y, Pos.Z, Vel.X, Vel.Y, Vel.Z),
                X, Y, FLinearColor::Green);
        }
        else if (ATurretPawn* Turret = Cast<ATurretPawn>(Agent))
        {
            DrawTextLine(FString::Printf(TEXT("  [Turret] %s  pos=(%.0f, %.0f, %.0f)  pitch=%.1f yaw=%.1f  tracking=%s"),
                *Id, Pos.X, Pos.Y, Pos.Z,
                Turret->GetCurrentPitch(), Turret->GetCurrentYaw(),
                Turret->IsTracking() ? TEXT("ON") : TEXT("off")),
                X, Y, FLinearColor::Yellow);
        }
        else
        {
            DrawTextLine(FString::Printf(TEXT("  [Agent] %s  pos=(%.0f, %.0f, %.0f)"),
                *Id, Pos.X, Pos.Y, Pos.Z),
                X, Y);
        }
    }
}

/**
 * @brief 绘制制导系统信息区域
 * @param Y 当前绘制 Y 坐标
 */
void ASimHUD::DrawGuidanceInfo(float& Y)
{
    float X = 20.0f;
    DrawSectionHeader(TEXT("Guidance"), X, Y);
    DrawTextLine(TEXT("  (use get_guidance_state via TCP for details)"), X, Y, FLinearColor(0.6f, 0.6f, 0.6f));
}

/**
 * @brief 绘制帧率信息
 * @param Y 当前绘制 Y 坐标
 */
void ASimHUD::DrawFPS(float& Y)
{
    float X = 20.0f;
    float FPS = 1.0f / FMath::Max(GetWorld()->GetDeltaSeconds(), 0.001f);
    FLinearColor FPSColor = (FPS >= 30.0f) ? FLinearColor::Green :
                            (FPS >= 15.0f) ? FLinearColor::Yellow : FLinearColor::Red;
    DrawTextLine(FString::Printf(TEXT("FPS: %.0f"), FPS), X, Y, FPSColor, 1.1f);
}

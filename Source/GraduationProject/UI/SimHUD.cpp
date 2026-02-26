/**
 * @file SimHUD.cpp
 * @brief 仿真 HUD 的实现文件
 *
 * 实现 DrawHUD() 方法，使用 Canvas 在屏幕上绘制
 * 仿真状态信息（无人机/转台/制导 OSD）。
 */

#include "SimHUD.h"
#include "Engine/Canvas.h"
#include "Engine/Font.h"
#include "GraduationProject/Core/Manager/AgentManager.h"
#include "GraduationProject/Drone/DronePawn.h"
#include "GraduationProject/Turret/TurretPawn.h"

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

        // 判断类型
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

void ASimHUD::DrawGuidanceInfo(float& Y)
{
    float X = 20.0f;
    DrawSectionHeader(TEXT("Guidance"), X, Y);
    DrawTextLine(TEXT("  (use get_guidance_state via TCP for details)"), X, Y, FLinearColor(0.6f, 0.6f, 0.6f));
}

void ASimHUD::DrawFPS(float& Y)
{
    float X = 20.0f;
    float FPS = 1.0f / FMath::Max(GetWorld()->GetDeltaSeconds(), 0.001f);
    FLinearColor FPSColor = (FPS >= 30.0f) ? FLinearColor::Green :
                            (FPS >= 15.0f) ? FLinearColor::Yellow : FLinearColor::Red;
    DrawTextLine(FString::Printf(TEXT("FPS: %.0f"), FPS), X, Y, FPSColor, 1.1f);
}

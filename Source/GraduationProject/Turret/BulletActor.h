#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Components/StaticMeshComponent.h"
#include "DrawDebugHelpers.h"
#include "BulletActor.generated.h"

/**
 * 子弹 Actor — 沿预计算的弹道轨迹点飞行
 * 由 TurretPawn::FireX() 生成
 */
UCLASS()
class GRADUATIONPROJECT_API ABulletActor : public AActor
{
    GENERATED_BODY()

public:
    ABulletActor();

protected:
    virtual void BeginPlay() override;

public:
    virtual void Tick(float DeltaTime) override;

    /** 初始化轨迹数据，传入预计算的轨迹点 */
    void InitTrajectory(const TArray<FVector>& InTrajectoryPoints, float InTotalTime = 1.0f);

    /** 绘制预览弹道 */
    static void DrawPreviewTrajectory(const UObject* WorldContext, const TArray<FVector>& Points,
        FColor LineColor, float Thickness, float HitRadius, FColor HitColor);

    // ---- 组件 ----
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components")
    UStaticMeshComponent* BulletMesh;

    // ---- 调试可视化 ----
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug")
    bool bShowTrajectory = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug")
    bool bDrawWholePathAtStart = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug")
    float TraceThickness = 5.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug")
    FColor TraceColor = FColor::Green;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug")
    float ImpactMarkerRadius = 30.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug")
    FColor ImpactMarkerColor = FColor::Red;

private:
    void SetupBulletMesh();

    TArray<FVector> TrajectoryPoints;
    float CurrentTime = 0.0f;
    float TotalLifeTime = 0.0f;
    float TimeStep = 0.001f;
    bool bIsFlying = false;
};

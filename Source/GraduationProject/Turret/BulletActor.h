#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Components/StaticMeshComponent.h"
#include "DrawDebugHelpers.h"
#include "BulletActor.generated.h"

/**
 * @brief 弹丸 Actor
 * 基于预计算弹道轨迹点进行运动
 */
UCLASS()
class GRADUATIONPROJECT_API ABulletActor : public AActor
{
    GENERATED_BODY()
public:
    /** @brief 构造函数，启用 Tick 并创建弹丸网格 */
    ABulletActor();
protected:
    /** @brief 游戏开始时调用，设置与转台之间的碰撞忽略 */
    virtual void BeginPlay() override;
public:
    /**
     * @brief 每帧更新，驱动弹丸沿轨迹飞行
     * @param DeltaTime 帧间隔时间（秒）
     */
    virtual void Tick(float DeltaTime) override;

    /**
     * @brief 初始化弹道轨迹数据
     * @param InTrajectoryPoints 预计算的轨迹点数组
     * @param InTotalTime 弹丸总飞行时间（秒）
     */
    void InitTrajectory(const TArray<FVector>& InTrajectoryPoints, float InTotalTime = 1.0f);

    /**
     * @brief 绘制预测弹道线
     * @param WorldContext 世界上下文对象
     * @param Points 轨迹点数组
     * @param LineColor 弹道线颜色
     * @param Thickness 弹道线宽度
     * @param HitRadius 命中标记球体半径
     * @param HitColor 命中标记颜色
     */
    static void DrawPreviewTrajectory(const UObject* WorldContext, const TArray<FVector>& Points,
        FColor LineColor, float Thickness, float HitRadius, FColor HitColor);

    /** @brief 弹丸网格组件 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components")
    UStaticMeshComponent* BulletMesh;

    /** @brief 是否显示飞行轨迹线 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug")
    bool bShowTrajectory = true;

    /** @brief 是否在发射时一次性绘制完整弹道预览 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug")
    bool bDrawWholePathAtStart = true;

    /** @brief 实时飞行轨迹线宽度 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug")
    float TraceThickness = 3.0f;

    /** @brief 实时飞行轨迹线颜色 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug")
    FColor TraceColor = FColor::Green;

    /** @brief 命中标记球体半径 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug")
    float ImpactMarkerRadius = 10.0f;

    /** @brief 命中标记球体颜色 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug")
    FColor ImpactMarkerColor = FColor::Red;

private:
    /** @brief 创建弹丸网格组件 */
    void SetupBulletMesh();

    /** @brief 预计算的弹道轨迹点数组 */
    TArray<FVector> TrajectoryPoints;

    /** @brief 当前飞行时间（秒） */
    float CurrentTime = 0.0f;

    /** @brief 弹丸总生命时间（秒） */
    float TotalLifeTime = 0.0f;

    /** @brief 轨迹点之间的时间间隔（秒） */
    float TimeStep = 0.001f;

    /** @brief 弹丸是否正在飞行中 */
    bool bIsFlying = false;
};

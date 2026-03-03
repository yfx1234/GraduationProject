#include "BulletActor.h"
#include "UObject/ConstructorHelpers.h"

/**
 * @brief 构造函数
 * 启用 Tick，调用 SetupBulletMesh() 创建弹丸网格组件。
 */
ABulletActor::ABulletActor()
{
    PrimaryActorTick.bCanEverTick = true;
    SetupBulletMesh();
}

/**
 * @brief 设置弹丸忽略与 Owner 之间的碰撞，
 * 遍历 Owner 的所有 PrimitiveComponent 逐一忽略，
 * 避免弹丸在枪口生成时立即与转台发生碰撞。
 */
void ABulletActor::BeginPlay()
{
    Super::BeginPlay();
    if (GetOwner())
    {
        BulletMesh->IgnoreActorWhenMoving(GetOwner(), true);
        TArray<UPrimitiveComponent*> OwnerComponents;
        GetOwner()->GetComponents<UPrimitiveComponent>(OwnerComponents);
        for (auto* Comp : OwnerComponents)
        {
            if (Comp) BulletMesh->IgnoreComponentWhenMoving(Comp, true);
        }
    }
}

/**
 * @brief 创建弹丸网格组件
 * 从文件夹加载资源，
 * 设置碰撞配置为 BlockAllDynamic，
 */
void ABulletActor::SetupBulletMesh()
{
    BulletMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("BulletMesh"));
    RootComponent = BulletMesh;
    static ConstructorHelpers::FObjectFinder<UStaticMesh> BulletMeshFinder(
        TEXT("StaticMesh'/Game/TurretAssets/Bullet.Bullet'"));
    if (BulletMeshFinder.Succeeded())
    {
        BulletMesh->SetStaticMesh(BulletMeshFinder.Object);
        BulletMesh->SetRelativeScale3D(FVector(1.0f));
    }
    BulletMesh->SetCollisionProfileName(TEXT("BlockAllDynamic"));
    BulletMesh->SetGenerateOverlapEvents(true);
    BulletMesh->SetNotifyRigidBodyCollision(true);
}

/**
 * @brief 初始化弹道轨迹数据
 * @param InTrajectoryPoints 预计算的轨迹点数组
 * @param InTotalTime 弹丸总飞行时间（秒）
 */
void ABulletActor::InitTrajectory(const TArray<FVector>& InTrajectoryPoints, float InTotalTime)
{
    if (InTrajectoryPoints.Num() < 2)
    {
        UE_LOG(LogTemp, Warning, TEXT("[Turret] Trajectory points too few!"));
        Destroy();
        return;
    }
    TrajectoryPoints = InTrajectoryPoints;
    TotalLifeTime = InTotalTime;
    CurrentTime = 0.0f;
    bIsFlying = true;
    TimeStep = TotalLifeTime / (TrajectoryPoints.Num() - 1);
    SetActorLocation(TrajectoryPoints[0]);
    // 绘制完整轨迹
    if (bShowTrajectory && bDrawWholePathAtStart)
    {
        for (int32 i = 0; i < TrajectoryPoints.Num() - 1; i++)
        {
            DrawDebugLine(GetWorld(), TrajectoryPoints[i], TrajectoryPoints[i + 1],
                FColor::Yellow, false, 10.0f, 0, 3.0f);
        }
    }
}

/**
 * @brief 每帧更新，驱动弹丸沿轨迹飞行
 * @param DeltaTime 帧间隔时间
 * 计算当前所在轨迹段的索引
 * 在相邻两个轨迹点之间做线性插值（Lerp）得到新位置
 * SetActorLocation(Sweep=true) 移动并检测碰撞
 */
void ABulletActor::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
    if (!bIsFlying || TrajectoryPoints.Num() < 2) return;
    CurrentTime += DeltaTime;
    if (CurrentTime >= TotalLifeTime)
    {
        Destroy();
        return;
    }
    float IndexFloat = CurrentTime / TimeStep;
    int32 IndexLast = FMath::FloorToInt(IndexFloat);
    int32 IndexNext = IndexLast + 1;
    if (IndexLast < 0) IndexLast = 0;
    if (IndexNext >= TrajectoryPoints.Num())
    {
        Destroy();
        return;
    }
    float Progress = FMath::Clamp(IndexFloat - IndexLast, 0.0f, 1.0f);
    FVector StartPos = GetActorLocation();
    FVector NewLocation = FMath::Lerp(TrajectoryPoints[IndexLast], TrajectoryPoints[IndexNext], Progress);
    // Sweep 碰撞检测：移动时检查路径上的碰撞
    FHitResult Hit;
    SetActorLocation(NewLocation, true, &Hit);
    if (bShowTrajectory) DrawDebugLine(GetWorld(), StartPos, GetActorLocation(), TraceColor, false, 10.0f, 0, TraceThickness);
    FVector Direction = (TrajectoryPoints[IndexNext] - TrajectoryPoints[IndexLast]).GetSafeNormal();
    if (!Direction.IsZero()) SetActorRotation(Direction.Rotation());
    if (Hit.bBlockingHit)
    {
        DrawDebugSphere(GetWorld(), Hit.ImpactPoint, ImpactMarkerRadius * 2.0f, 16, ImpactMarkerColor, false, 10.0f);
        Destroy();
    }
}

/**
 * @brief 绘制预测弹道线
 * @param WorldContext 世界上下文对象
 * @param Points 轨迹点数组
 * @param LineColor 弹道线颜色
 * @param Thickness 弹道线宽度
 * @param HitRadius 命中标记半径
 * @param HitColor 命中标记颜色
 */
void ABulletActor::DrawPreviewTrajectory(const UObject* WorldContext, const TArray<FVector>& Points,
    FColor LineColor, float Thickness, float HitRadius, FColor HitColor)
{
    if (!WorldContext || Points.Num() < 2) return;
    UWorld* World = WorldContext->GetWorld();
    if (!World) return;
    for (int32 i = 0; i < Points.Num() - 1; i++)
    {
        DrawDebugLine(World, Points[i], Points[i + 1], LineColor, false, -1.0f, 0, Thickness);
    }
}

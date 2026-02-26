/**
 * @file BulletActor.cpp
 * @brief 弹丸 Actor 的实现文件
 *
 * 实现弹丸的物理仿真（重力+阻力弹道）、
 * 生命周期管理和碰撞检测逻辑。
 */

#include "BulletActor.h"
#include "UObject/ConstructorHelpers.h"

ABulletActor::ABulletActor()
{
    PrimaryActorTick.bCanEverTick = true;
    SetupBulletMesh();
}

void ABulletActor::BeginPlay()
{
    Super::BeginPlay();

    // 让子弹忽略与转台(Owner)的碰撞
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

void ABulletActor::SetupBulletMesh()
{
    BulletMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("BulletMesh"));
    RootComponent = BulletMesh;

    // 子弹模型 — 从 TurretAssets 加载
    static ConstructorHelpers::FObjectFinder<UStaticMesh> BulletMeshFinder(
        TEXT("StaticMesh'/Game/TurretAssets/Bullet.Bullet'"));
    if (BulletMeshFinder.Succeeded())
    {
        BulletMesh->SetStaticMesh(BulletMeshFinder.Object);
        BulletMesh->SetRelativeScale3D(FVector(1.0f));
    }

    // 碰撞设置
    BulletMesh->SetCollisionProfileName(TEXT("BlockAllDynamic"));
    BulletMesh->SetGenerateOverlapEvents(true);
    BulletMesh->SetNotifyRigidBodyCollision(true);
}

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

    // 根据轨迹点数量计算时间步长
    TimeStep = TotalLifeTime / (TrajectoryPoints.Num() - 1);

    // 设置初始位置
    SetActorLocation(TrajectoryPoints[0]);

    // 预览完整轨迹
    if (bShowTrajectory && bDrawWholePathAtStart)
    {
        for (int32 i = 0; i < TrajectoryPoints.Num() - 1; i++)
        {
            DrawDebugLine(GetWorld(), TrajectoryPoints[i], TrajectoryPoints[i + 1],
                FColor::Yellow, false, 10.0f, 0, 3.0f);
        }
    }
}

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

    // sweep 碰撞检测
    FHitResult Hit;
    SetActorLocation(NewLocation, true, &Hit);

    // 实时轨迹线
    if (bShowTrajectory)
    {
        DrawDebugLine(GetWorld(), StartPos, GetActorLocation(), TraceColor, false, 10.0f, 0, TraceThickness);
    }

    // 设置朝向
    FVector Direction = (TrajectoryPoints[IndexNext] - TrajectoryPoints[IndexLast]).GetSafeNormal();
    if (!Direction.IsZero())
    {
        SetActorRotation(Direction.Rotation());
    }

    // 碰撞检测
    if (Hit.bBlockingHit)
    {
        DrawDebugSphere(GetWorld(), Hit.ImpactPoint, ImpactMarkerRadius * 2.0f, 16, ImpactMarkerColor, false, 10.0f);
        Destroy();
    }
}

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

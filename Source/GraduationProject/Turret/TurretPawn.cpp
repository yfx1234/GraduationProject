#include "TurretPawn.h"
#include "BulletActor.h"
#include "TurretAiming.h"
#include "DrawDebugHelpers.h"
#include "UObject/ConstructorHelpers.h"
#include "GraduationProject/Core/Manager/AgentManager.h"
#include "Misc/Base64.h"
#include "IImageWrapperModule.h"
#include "IImageWrapper.h"
#include "BC_Unit.hpp"
#include "BC_DragTables.hpp"
#include "BC_DragModel.hpp"
#include "BC_Ammunition.hpp"
#include "BC_Atmosphere.hpp"
#include "BC_Wind.hpp"
#include "BC_Weapon.hpp"
#include "BC_Shot.hpp"
#include "BC_Calculator.hpp"
#include "GraduationProject/Vision/CameraCaptureUtils.h"
#include "Components/PrimitiveComponent.h"
#include "HAL/IConsoleManager.h"

/**
 * @brief 构造炮台 Pawn
 * 创建炮台层级、瞄准组件和相机采集组件。
 */
ATurretPawn::ATurretPawn()
{
    PrimaryActorTick.bCanEverTick = true;
    SetupTurretMesh();
    BulletClass = ABulletActor::StaticClass();
    AimingComponent = CreateDefaultSubobject<UTurretAiming>(TEXT("AimingComponent"));
    TurretSceneCapture = CreateDefaultSubobject<USceneCaptureComponent2D>(TEXT("TurretSceneCapture"));
    TurretSceneCapture->SetupAttachment(GunMesh);
    TurretSceneCapture->SetRelativeLocation(MuzzleOffset);
    TurretSceneCapture->SetRelativeRotation(FRotator(0.0f, 180.0f, 0.0f));
    TurretSceneCapture->bCaptureEveryFrame = true;
    TurretSceneCapture->bCaptureOnMovement = false;
    TurretSceneCapture->bAlwaysPersistRenderingState = true;
    TurretSceneCapture->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
    TurretCineCamera = CreateDefaultSubobject<UCineCameraComponent>(TEXT("TurretCineCamera"));
    TurretCineCamera->SetupAttachment(GunMesh);
    TurretCineCamera->SetRelativeLocation(MuzzleOffset);
    TurretCineCamera->SetRelativeRotation(FRotator(0.0f, 180.0f, 0.0f));
    TurretCineCamera->SetActive(false);
}

/**
 * @brief 初始化炮台运行时资源
 * 创建 `RenderTarget`、注册到 `AgentManager`，并初始化相机分割设置。
 */
void ATurretPawn::BeginPlay()
{
    Super::BeginPlay();
    CurrentPitch = 0.0f;
    CurrentYaw = 0.0f;
    UAgentManager::GetInstance()->RegisterAgent(TurretId, this);
    UTextureRenderTarget2D* RT = NewObject<UTextureRenderTarget2D>(this);
    RT->InitCustomFormat(CameraWidth, CameraHeight, PF_B8G8R8A8, false);
    RT->ClearColor = FLinearColor::Black;
    TurretSceneCapture->TextureTarget = RT;
    TurretSceneCapture->FOVAngle = CameraFOV;
    ApplySegmentationStencil();
    UE_LOG(LogTemp, Log, TEXT("[TurretPawn] BeginPlay: %s at %s, Camera %dx%d FOV=%.0f"),
        *TurretId, *GetActorLocation().ToString(), CameraWidth, CameraHeight, CameraFOV);
}

/**
 * @brief 每帧更新炮台姿态
 * @param DeltaTime 帧间隔（秒）
 * 同步后处理设置，并插值更新云台和枪管角度；若开启调试则绘制预测轨迹。
 */
void ATurretPawn::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
    SyncPostProcessToCapture();
    float YawDiff = FRotator::NormalizeAxis(TargetYaw - CurrentYaw);
    float AdjustedTargetYaw = CurrentYaw + YawDiff;
    CurrentYaw = FMath::FInterpTo(CurrentYaw, AdjustedTargetYaw, DeltaTime, RotationSpeed);
    CurrentPitch = FMath::FInterpTo(CurrentPitch, TargetPitch, DeltaTime, RotationSpeed);
    if (GimbalMesh) GimbalMesh->SetRelativeRotation(FRotator(0.0f, CurrentYaw, 0.0f));
    if (GunMesh) GunMesh->SetRelativeRotation(FRotator(-CurrentPitch, 0.0f, 0.0f));
    if (bShowPredictionLine) DrawPredictionLine();
}

/**
 * @brief 设置炮台目标角度
 * @param NewTargetPitch 目标俯仰角（度）
 * @param NewTargetYaw 目标偏航角（度）
 */
void ATurretPawn::SetTargetAngles(float NewTargetPitch, float NewTargetYaw)
{
    TargetPitch = NewTargetPitch;
    TargetYaw = NewTargetYaw;
}

/**
 * @brief 开始跟踪目标 Agent
 * @param TargetActorID 目标 Agent ID
 */
void ATurretPawn::StartTracking(const FString& TargetActorID)
{
    if (AimingComponent) AimingComponent->StartTracking(TargetActorID);
}

/** @brief 停止自动跟踪 */
void ATurretPawn::StopTracking()
{
    if (AimingComponent) AimingComponent->StopTracking();
}

/**
 * @brief 查询当前是否正在跟踪目标
 * @return 正在跟踪时返回 `true`
 */
bool ATurretPawn::IsTracking() const
{
    return AimingComponent ? AimingComponent->IsTracking() : false;
}

/**
 * @brief 发射弹丸
 * @param InitialSpeed 弹丸初速度（m/s）
 * 先根据当前枪管姿态计算弹道，再生成 `ABulletActor` 沿轨迹运动。
 */
void ATurretPawn::FireX(float InitialSpeed)
{
    if (!GunMesh || !GetWorld()) return;
    FRotator CurrentGunRot = GunMesh->GetComponentRotation();
    FRotator MuzzleRotation = CurrentGunRot;
    MuzzleRotation.Pitch *= -1.0f;
    MuzzleRotation.Yaw += 180.0f;
    FVector MuzzleLocation = GunMesh->GetComponentLocation() + CurrentGunRot.RotateVector(MuzzleOffset);
    UE_LOG(LogTemp, Log, TEXT("[Turret] Fire! Speed: %.1f"), InitialSpeed);
    float TrajectoryDuration = 0.0f;
    TArray<FVector> PathPoints = Ballistic(MuzzleLocation, MuzzleRotation, InitialSpeed, TrajectoryDuration);
    if (BulletClass && PathPoints.Num() > 0)
    {
        FActorSpawnParameters SpawnParams;
        SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
        SpawnParams.Owner = this;
        ABulletActor* Bullet = GetWorld()->SpawnActor<ABulletActor>(BulletClass, MuzzleLocation, MuzzleRotation, SpawnParams);
        if (Bullet) Bullet->InitTrajectory(PathPoints, TrajectoryDuration);
    }
}

/**
 * @brief 使用 BC 弹道库计算轨迹点
 * @param StartPos 发射起点
 * @param ShootDir 发射方向
 * @param InitialSpeed 初速度（m/s）
 * @param OutTotalTime 输出总飞行时间
 * @return 轨迹点数组
 * 当前配置使用 G7 阻力模型、5g 弹丸、3.8mm 口径，并按 0.01s 步长积分到 500m。
 */
TArray<FVector> ATurretPawn::Ballistic(FVector StartPos, FRotator ShootDir, float InitialSpeed, float& OutTotalTime)
{
    TArray<FVector> ResultPath;
    OutTotalTime = 0.0f;

    std::vector<DragDataPoint> dragTable;
    try { dragTable = DragTables::getTable("G7"); }
    catch (...) { dragTable = DragTables::getTable("G1"); }

    double bc_value = 0.295;
    Weight bulletWeight = Weight::Grams(5.0);
    Distance bulletDiameter = Distance::Millimeters(3.8);
    Distance bulletLength = Distance::Millimeters(3.8);
    DragModel dragModel(bc_value, dragTable, bulletWeight, bulletDiameter, bulletLength);
    Velocity muzzleVel = Velocity::MPS(InitialSpeed);
    Temperature powderTemp = Temperature::Celsius(15.0);
    Ammunition ammo(dragModel, muzzleVel, powderTemp, 0.0, true);
    Distance altitude = Distance::Meters(StartPos.Z / 100.0);
    Atmosphere atmo = Atmosphere::ICAO(altitude, Temperature::Celsius(15), 0.0);
    Distance sightHeight = Distance::Centimeters(0.0);
    Distance twist = Distance::Inches(0.0);
    Weapon weapon(sightHeight, twist, Angular::Degrees(0));
    Velocity windSpeed = Velocity::MPS(0.0);
    Angular windDir = Angular::Degrees(0.0);
    Wind wind1(windSpeed, windDir);
    std::vector<Wind> winds = { wind1 };
    Angular lookAngle = Angular::Degrees(0);
    double azimuth = 0;
    double latitude = 0.0;
    Angular cantAngle = Angular::Degrees(0);
    Shot shot(ammo, atmo, weapon, winds, lookAngle, Angular::Degrees(0), cantAngle, azimuth, latitude);
    Calculator calculator;
    double time_step = 0.01;
    double max_dist = 500.0;
    std::vector<TrajectoryPoint> vT = calculator.fire(shot, Distance::Meters(max_dist), Distance::Meters(0), time_step);
    if (!vT.empty()) OutTotalTime = vT.back().time;

    FTransform GunTransform(ShootDir, StartPos);
    for (const auto& Pt : vT)
    {
        double RelX_m = Pt.distance.Meters();
        double RelZ_m = Pt.height.Meters();
        double RelY_m = Pt.windage.Meters();
        FVector LocalPos(RelX_m * 100.0, RelY_m * 100.0, RelZ_m * 100.0);
        ResultPath.Add(GunTransform.TransformPosition(LocalPos));
    }
    return ResultPath;
}

/** @brief 创建底座、云台和枪管网格组件 */
void ATurretPawn::SetupTurretMesh()
{
    CollisionComponent = CreateDefaultSubobject<USphereComponent>(TEXT("RootCollision"));
    RootComponent = CollisionComponent;
    CollisionComponent->InitSphereRadius(60.0f);
    CollisionComponent->SetCollisionProfileName(UCollisionProfile::Pawn_ProfileName);
    BaseMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("BaseMesh"));
    BaseMesh->SetupAttachment(RootComponent);
    GimbalMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("GimbalMesh"));
    GimbalMesh->SetupAttachment(BaseMesh);
    GunMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("GunMesh"));
    GunMesh->SetupAttachment(GimbalMesh);
}

/** @brief 开启预测弹道线显示 */
void ATurretPawn::ShowPredictionLine()
{
    bShowPredictionLine = true;
    UE_LOG(LogTemp, Log, TEXT("[Turret] PredictionLine enabled"));
}

/** @brief 关闭预测弹道线显示 */
void ATurretPawn::HidePredictionLine()
{
    bShowPredictionLine = false;
    UE_LOG(LogTemp, Log, TEXT("[Turret] PredictionLine disabled"));
}

/**
 * @brief 绘制预测弹道线和命中点
 * 使用默认弹丸初速度进行一条调试弹道预览，并对首个命中点绘制球标记。
 */
void ATurretPawn::DrawPredictionLine()
{
    if (!GunMesh || !GetWorld()) return;
    FRotator CurrentGunRot = GunMesh->GetComponentRotation();
    FRotator MuzzleRotation = CurrentGunRot;
    MuzzleRotation.Pitch *= -1.0f;
    MuzzleRotation.Yaw += 180.0f;
    FVector MuzzleLocation = GunMesh->GetComponentLocation() + CurrentGunRot.RotateVector(MuzzleOffset);
    float TrajectoryDuration = 0.0f;
    TArray<FVector> PathPoints = Ballistic(MuzzleLocation, MuzzleRotation, DefaultMuzzleSpeed, TrajectoryDuration);
    if (PathPoints.Num() < 2) return;
    for (int32 i = 0; i < PathPoints.Num() - 1; ++i)
    {
        DrawDebugLine(GetWorld(), PathPoints[i], PathPoints[i + 1], PredictionLineColor, false, -1.0f, 0, PredictionLineThickness);
    }
    FHitResult Hit;
    FCollisionQueryParams QueryParams;
    QueryParams.AddIgnoredActor(this);
    for (int32 i = 0; i < PathPoints.Num() - 1; ++i)
    {
        if (GetWorld()->LineTraceSingleByChannel(Hit, PathPoints[i], PathPoints[i + 1], ECC_Visibility, QueryParams))
        {
            DrawDebugSphere(GetWorld(), Hit.ImpactPoint, PredictionHitRadius, 8, PredictionHitColor, false, -1.0f);
            break;
        }
    }
}

/**
 * @brief 捕获当前炮台相机画面并返回 Base64 JPEG
 * @param Quality JPEG 压缩质量；小于等于 0 时使用 `JpegQuality`
 * @return Base64 编码后的 JPEG 图像数据
 */
FString ATurretPawn::CaptureImageBase64(int32 Quality)
{
    if (Quality <= 0)
    {
        Quality = JpegQuality;
    }

    return CameraCaptureUtils::CaptureColorJpegBase64(TurretSceneCapture, CameraWidth, CameraHeight, Quality);
}

/** @brief 将可见网格写入 CustomDepth/Stencil，供分割图像使用 */
void ATurretPawn::ApplySegmentationStencil()
{
    IConsoleVariable* CustomDepthVar = IConsoleManager::Get().FindConsoleVariable(TEXT("r.CustomDepth"));
    if (CustomDepthVar && CustomDepthVar->GetInt() < 3)
    {
        CustomDepthVar->Set(3, ECVF_SetByCode);
    }

    const int32 ClampedId = FMath::Clamp(SegmentationId, 0, 255);

    TArray<UPrimitiveComponent*> PrimitiveComponents;
    GetComponents<UPrimitiveComponent>(PrimitiveComponents);
    for (UPrimitiveComponent* Primitive : PrimitiveComponents)
    {
        if (!Primitive)
        {
            continue;
        }

        Primitive->SetRenderCustomDepth(true);
        Primitive->SetCustomDepthStencilValue(ClampedId);
    }
}

/**
 * @brief 将 CineCamera 的后处理设置同步给 SceneCapture
 * 自动叠加曝光补偿，保证导出图像与场景观察更一致。
 */
void ATurretPawn::SyncPostProcessToCapture()
{
    if (!TurretCineCamera || !TurretSceneCapture) return;
    FMinimalViewInfo ViewInfo;
    TurretCineCamera->GetCameraView(0.0f, ViewInfo);
    FWeightedBlendables SavedBlendables = TurretSceneCapture->PostProcessSettings.WeightedBlendables;
    TurretSceneCapture->PostProcessSettings = ViewInfo.PostProcessSettings;
    TurretSceneCapture->PostProcessSettings.WeightedBlendables = SavedBlendables;
    TurretSceneCapture->PostProcessSettings.bOverride_AutoExposureBias = true;
    TurretSceneCapture->PostProcessSettings.AutoExposureBias += ExposureBias;
}

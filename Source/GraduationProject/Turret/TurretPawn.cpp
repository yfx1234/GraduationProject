/**
 * @file TurretPawn.cpp
 * @brief 转台/炮塔 Pawn 的实现文件
 *
 * 实现转台组件创建、目标跟踪、射击逻辑、
 * SceneCaptureComponent2D 图像采集和 Base64 编码。
 */

#include "TurretPawn.h"
#include "BulletActor.h"
#include "TurretAiming.h"
#include "DrawDebugHelpers.h"
#include "UObject/ConstructorHelpers.h"
#include "GraduationProject/Core/Manager/AgentManager.h"
#include "Misc/Base64.h"
#include "IImageWrapperModule.h"
#include "IImageWrapper.h"

// BC弹道库
#include "BC_Unit.hpp"
#include "BC_DragTables.hpp"
#include "BC_DragModel.hpp"
#include "BC_Ammunition.hpp"
#include "BC_Atmosphere.hpp"
#include "BC_Wind.hpp"
#include "BC_Weapon.hpp"
#include "BC_Shot.hpp"
#include "BC_Calculator.hpp"

ATurretPawn::ATurretPawn()
{
    PrimaryActorTick.bCanEverTick = true;

    SetupTurretMesh();

    // 指定子弹 C++ 类
    BulletClass = ABulletActor::StaticClass();

    // 创建瞄准组件
    AimingComponent = CreateDefaultSubobject<UTurretAiming>(TEXT("AimingComponent"));

    // 创建枪管摄像头 — 附着 GunMesh，偏移到枪口位置（MuzzleOffset）
    TurretSceneCapture = CreateDefaultSubobject<USceneCaptureComponent2D>(TEXT("TurretSceneCapture"));
    TurretSceneCapture->SetupAttachment(GunMesh);
    // 底座/云台/枪管中心都在同一点，MuzzleOffset 才是枪口位置
    TurretSceneCapture->SetRelativeLocation(MuzzleOffset);
    // 枪管方向是 -X (MuzzleOffset.X=-249)，旋转180°朝外看
    TurretSceneCapture->SetRelativeRotation(FRotator(0.0f, 180.0f, 0.0f));
    TurretSceneCapture->bCaptureEveryFrame = true;   // 每帧捕获，让自动曝光收敛
    TurretSceneCapture->bCaptureOnMovement = false;
    TurretSceneCapture->bAlwaysPersistRenderingState = true;  // 持久化渲染状态（曝光等）
    // FinalColorLDR = 完整后处理链 + gamma，与编辑器视口一致
    TurretSceneCapture->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;

    // CineCamera — 与 SceneCapture 同位置同朝向，用于继承场景 PostProcessVolume (仿 AirSim)
    TurretCineCamera = CreateDefaultSubobject<UCineCameraComponent>(TEXT("TurretCineCamera"));
    TurretCineCamera->SetupAttachment(GunMesh);
    TurretCineCamera->SetRelativeLocation(MuzzleOffset);
    TurretCineCamera->SetRelativeRotation(FRotator(0.0f, 180.0f, 0.0f));
    TurretCineCamera->SetActive(false);  // 不实际渲染，只借用 PP 设置
}

void ATurretPawn::BeginPlay()
{
    Super::BeginPlay();

    CurrentPitch = 0.0f;
    CurrentYaw = 0.0f;

    // 注册到 AgentManager
    UAgentManager::GetInstance()->RegisterAgent(TurretId, this);

    // 初始化 RenderTarget
    UTextureRenderTarget2D* RT = NewObject<UTextureRenderTarget2D>(this);
    RT->InitCustomFormat(CameraWidth, CameraHeight, PF_B8G8R8A8, false);  // 8-bit sRGB
    RT->ClearColor = FLinearColor::Black;
    TurretSceneCapture->TextureTarget = RT;
    TurretSceneCapture->FOVAngle = CameraFOV;

    UE_LOG(LogTemp, Log, TEXT("[TurretPawn] BeginPlay: %s at %s, Camera %dx%d FOV=%.0f"),
        *TurretId, *GetActorLocation().ToString(), CameraWidth, CameraHeight, CameraFOV);
}

void ATurretPawn::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    // 同步 CineCamera 的 PostProcess 到 SceneCapture (仿 AirSim)
    SyncPostProcessToCapture();

    // Yaw 角度平滑插值（处理 +-180 边界）
    float YawDiff = FRotator::NormalizeAxis(TargetYaw - CurrentYaw);
    float AdjustedTargetYaw = CurrentYaw + YawDiff;

    CurrentYaw = FMath::FInterpTo(CurrentYaw, AdjustedTargetYaw, DeltaTime, RotationSpeed);
    CurrentPitch = FMath::FInterpTo(CurrentPitch, TargetPitch, DeltaTime, RotationSpeed);

    // 应用旋转到网格组件
    if (GimbalMesh)
    {
        GimbalMesh->SetRelativeRotation(FRotator(0.0f, CurrentYaw, 0.0f));
    }

    if (GunMesh)
    {
        GunMesh->SetRelativeRotation(FRotator(-CurrentPitch, 0.0f, 0.0f));
    }

    // 绘制预测弹道线
    if (bShowPredictionLine)
    {
        DrawPredictionLine();
    }
}


void ATurretPawn::SetTargetAngles(float NewTargetPitch, float NewTargetYaw)
{
    TargetPitch = NewTargetPitch;
    TargetYaw = NewTargetYaw;
}

void ATurretPawn::StartTracking(const FString& TargetActorID)
{
    if (AimingComponent)
    {
        AimingComponent->StartTracking(TargetActorID);
    }
}

void ATurretPawn::StopTracking()
{
    if (AimingComponent)
    {
        AimingComponent->StopTracking();
    }
}

bool ATurretPawn::IsTracking() const
{
    return AimingComponent ? AimingComponent->IsTracking() : false;
}

void ATurretPawn::FireX(float InitialSpeed)
{
    if (!GunMesh || !GetWorld()) return;

    // 获取当前枪管旋转角度
    FRotator CurrentGunRot = GunMesh->GetComponentRotation();

    // 修正发射方向
    FRotator MuzzleRotation = CurrentGunRot;
    MuzzleRotation.Pitch *= -1.0f;
    MuzzleRotation.Yaw += 180.0f;

    // 计算枪口位置
    FVector MuzzleLocation = GunMesh->GetComponentLocation() + CurrentGunRot.RotateVector(MuzzleOffset);

    UE_LOG(LogTemp, Log, TEXT("[Turret] Fire! Speed: %.1f"), InitialSpeed);

    // 计算弹道轨迹
    float TrajectoryDuration = 0.0f;
    TArray<FVector> PathPoints = Ballistic(MuzzleLocation, MuzzleRotation, InitialSpeed, TrajectoryDuration);

    if (BulletClass && PathPoints.Num() > 0)
    {
        FActorSpawnParameters SpawnParams;
        SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
        SpawnParams.Owner = this;

        ABulletActor* Bullet = GetWorld()->SpawnActor<ABulletActor>(BulletClass, MuzzleLocation, MuzzleRotation, SpawnParams);
        if (Bullet)
        {
            Bullet->InitTrajectory(PathPoints, TrajectoryDuration);
        }
    }
}


TArray<FVector> ATurretPawn::Ballistic(FVector StartPos, FRotator ShootDir, float InitialSpeed, float& OutTotalTime)
{
    TArray<FVector> ResultPath;
    OutTotalTime = 0.0f;

    // 阻力表
    std::vector<DragDataPoint> dragTable;
    try {
        dragTable = DragTables::getTable("G7");
    }
    catch (...) {
        dragTable = DragTables::getTable("G1");
    }

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

    if (!vT.empty())
    {
        OutTotalTime = vT.back().time;
    }

    // 将弹道计算结果转换为UE世界坐标
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


void ATurretPawn::SetupTurretMesh()
{
    // 碰撞根组件
    CollisionComponent = CreateDefaultSubobject<USphereComponent>(TEXT("RootCollision"));
    RootComponent = CollisionComponent;
    CollisionComponent->InitSphereRadius(60.0f);
    CollisionComponent->SetCollisionProfileName(UCollisionProfile::Pawn_ProfileName);

    // 底座 — Blueprint 中配置 StaticMesh
    BaseMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("BaseMesh"));
    BaseMesh->SetupAttachment(RootComponent);

    // 云台 — Yaw 旋转
    GimbalMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("GimbalMesh"));
    GimbalMesh->SetupAttachment(BaseMesh);

    // 枪管 — Pitch 旋转
    GunMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("GunMesh"));
    GunMesh->SetupAttachment(GimbalMesh);
}


void ATurretPawn::ShowPredictionLine()
{
    bShowPredictionLine = true;
    UE_LOG(LogTemp, Log, TEXT("[Turret] PredictionLine enabled"));
}

void ATurretPawn::HidePredictionLine()
{
    bShowPredictionLine = false;
    UE_LOG(LogTemp, Log, TEXT("[Turret] PredictionLine disabled"));
}

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

    // 绘制弹道线
    for (int32 i = 0; i < PathPoints.Num() - 1; ++i)
    {
        DrawDebugLine(GetWorld(), PathPoints[i], PathPoints[i + 1], PredictionLineColor, false, -1.0f, 0, PredictionLineThickness);
    }

    // 碰撞检测找预测命中点
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


FString ATurretPawn::CaptureImageBase64(int32 Quality)
{
    // 如果传入默认值(-1), 使用属性中的 JpegQuality
    if (Quality <= 0) Quality = JpegQuality;
    if (!TurretSceneCapture || !TurretSceneCapture->TextureTarget)
        return TEXT("");

    // 手动触发一次采集
    TurretSceneCapture->CaptureScene();

    FTextureRenderTargetResource* Resource = TurretSceneCapture->TextureTarget->GameThread_GetRenderTargetResource();
    if (!Resource) return TEXT("");

    TArray<FColor> Pixels;
    int32 W = CameraWidth;
    int32 H = CameraHeight;
    Pixels.SetNum(W * H);
    FReadSurfaceDataFlags ReadFlags(RCM_UNorm, CubeFace_MAX);
    ReadFlags.SetLinearToGamma(false);  // LDR 已含 gamma，不做额外转换
    Resource->ReadPixels(Pixels, ReadFlags);

    // JPEG 编码
    IImageWrapperModule& ImgModule = FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));
    TSharedPtr<IImageWrapper> Wrapper = ImgModule.CreateImageWrapper(EImageFormat::JPEG);

    TArray<uint8> RawData;
    RawData.SetNum(Pixels.Num() * 4);
    for (int32 i = 0; i < Pixels.Num(); i++)
    {
        RawData[i * 4 + 0] = Pixels[i].B;
        RawData[i * 4 + 1] = Pixels[i].G;
        RawData[i * 4 + 2] = Pixels[i].R;
        RawData[i * 4 + 3] = Pixels[i].A;
    }

    if (!Wrapper.IsValid()) return TEXT("");
    if (!Wrapper->SetRaw(RawData.GetData(), RawData.Num(), W, H, ERGBFormat::BGRA, 8))
        return TEXT("");

    auto JpegData = Wrapper->GetCompressed(Quality);
    return FBase64::Encode(JpegData.GetData(), JpegData.Num());
}


void ATurretPawn::SyncPostProcessToCapture()
{
    if (!TurretCineCamera || !TurretSceneCapture) return;

    // 获取 CineCamera 的视图信息 (包含场景 PostProcessVolume 叠加后的 PP 设置)
    FMinimalViewInfo ViewInfo;
    TurretCineCamera->GetCameraView(0.0f, ViewInfo);

    // 保存 SceneCapture 自身的 Blendables (如有自定义后处理材质)
    FWeightedBlendables SavedBlendables = TurretSceneCapture->PostProcessSettings.WeightedBlendables;

    // 整体拷贝 PP 设置
    TurretSceneCapture->PostProcessSettings = ViewInfo.PostProcessSettings;

    // 恢复 Blendables
    TurretSceneCapture->PostProcessSettings.WeightedBlendables = SavedBlendables;

    // 叠加用户的曝光补偿 (在 CineCamera PP 基础上微调)
    TurretSceneCapture->PostProcessSettings.bOverride_AutoExposureBias = true;
    TurretSceneCapture->PostProcessSettings.AutoExposureBias += ExposureBias;
}

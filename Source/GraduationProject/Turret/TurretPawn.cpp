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

/**
 * @brief 构造函数
 * 创建转台网格层级
 * 设置默认弹丸类为 ABulletActor
 * 创建 UTurretAiming 瞄准组件
 * 创建 SceneCaptureComponent2D 和 CineCameraComponent 并附着到枪管
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

/** @brief 创建 RenderTarget2D 画布，投射到 TurretSceneCapture */
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
    UE_LOG(LogTemp, Log, TEXT("[TurretPawn] BeginPlay: %s at %s, Camera %dx%d FOV=%.0f"),
        *TurretId, *GetActorLocation().ToString(), CameraWidth, CameraHeight, CameraFOV);
}

/**
 * @param DeltaTime 帧间隔时间（秒）
 *  同步 CineCamera PP 到 SceneCapture
 *  GimbalMesh、GunMesh 旋转
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
 * @brief 设置转台目标角度
 * @param NewTargetPitch 目标俯仰角
 * @param NewTargetYaw 目标偏航角
 */
void ATurretPawn::SetTargetAngles(float NewTargetPitch, float NewTargetYaw)
{
    TargetPitch = NewTargetPitch;
    TargetYaw = NewTargetYaw;
}

/**
 * @brief 通过 Agent ID 开始自动跟踪目标
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
 * @brief 查询是否正在跟踪目标
 * @return true 表示正在跟踪
 */
bool ATurretPawn::IsTracking() const
{
    return AimingComponent ? AimingComponent->IsTracking() : false;
}

/**
 * @brief 发射弹丸
 * @param InitialSpeed 弹丸初始速度
 * 生成 ABulletActor 并传入轨迹
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
 * @brief 使用 BC 弹道库计算弹道轨迹
 * @param StartPos 发射起点
 * @param ShootDir 发射方向
 * @param InitialSpeed 初始速度
 * @param OutTotalTime 弹道总飞行时间
 * @return 弹道轨迹点数组
 * 弹药参数：BC=0.295，弹重 5g，弹径 3.8mm
 * 阻力表：G7
 * 大气：ICAO 标准大气，温度 15°C
 * 最大射程：500m，时间步长 0.01s
 */
TArray<FVector> ATurretPawn::Ballistic(FVector StartPos, FRotator ShootDir, float InitialSpeed, float& OutTotalTime)
{
    TArray<FVector> ResultPath;
    OutTotalTime = 0.0f;
    std::vector<DragDataPoint> dragTable;   // 阻力表
    try {dragTable = DragTables::getTable("G7");}
    catch (...) {dragTable = DragTables::getTable("G1");}
    double bc_value = 0.295;    // 弹丸弹道系数
    Weight bulletWeight = Weight::Grams(5.0);  // 弹丸重量
    Distance bulletDiameter = Distance::Millimeters(3.8);  // 弹丸直径
    Distance bulletLength = Distance::Millimeters(3.8);  // 弹丸长度
    DragModel dragModel(bc_value, dragTable, bulletWeight, bulletDiameter, bulletLength);   //// 子弹阻力模型
    Velocity muzzleVel = Velocity::MPS(InitialSpeed);  // 弹丸初速度
    Temperature powderTemp = Temperature::Celsius(15.0);    // 弹药温度
    Ammunition ammo(dragModel, muzzleVel, powderTemp, 0.0, true);   //// 弹药参数
    Distance altitude = Distance::Meters(StartPos.Z / 100.0);  // 海拔高度
    Atmosphere atmo = Atmosphere::ICAO(altitude, Temperature::Celsius(15), 0.0);  //// 大气模型
    Distance sightHeight = Distance::Centimeters(0.0);  // 瞄准镜高度
    Distance twist = Distance::Inches(0.0);  // 膛线缠距
    Weapon weapon(sightHeight, twist, Angular::Degrees(0));  //// 武器参数
    Velocity windSpeed = Velocity::MPS(0.0);  // 风速
    Angular windDir = Angular::Degrees(0.0);  // 风向
    Wind wind1(windSpeed, windDir);     //// 风
    std::vector<Wind> winds = { wind1 };    
    Angular lookAngle = Angular::Degrees(0);  // 瞄准方向
    double azimuth = 0;  // 方位角
    double latitude = 0.0;  // 纬度
    Angular cantAngle = Angular::Degrees(0);  // 枪口倾角
    Shot shot(ammo, atmo, weapon, winds, lookAngle, Angular::Degrees(0), cantAngle, azimuth, latitude);     //// 全部参数
    Calculator calculator;
    double time_step = 0.01;    // 时间步长
    double max_dist = 500.0;    // 最大射程
    std::vector<TrajectoryPoint> vT = calculator.fire(shot, Distance::Meters(max_dist), Distance::Meters(0), time_step);
    if (!vT.empty()) OutTotalTime = vT.back().time;
    FTransform GunTransform(ShootDir, StartPos);    
    for (const auto& Pt : vT)
    {
        double RelX_m = Pt.distance.Meters();    // 前方距离
        double RelZ_m = Pt.height.Meters();      // 弹道高度（含重力下降）
        double RelY_m = Pt.windage.Meters();     // 横向偏移（风偏）
        FVector LocalPos(RelX_m * 100.0, RelY_m * 100.0, RelZ_m * 100.0);
        ResultPath.Add(GunTransform.TransformPosition(LocalPos));
    }
    return ResultPath;
}


/** @brief 创建转台网格组件 */
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

/** @brief 启用预测弹道线显示 */
void ATurretPawn::ShowPredictionLine()
{
    bShowPredictionLine = true;
    UE_LOG(LogTemp, Log, TEXT("[Turret] PredictionLine enabled"));
}

/** @brief 禁用预测弹道线显示 */
void ATurretPawn::HidePredictionLine()
{
    bShowPredictionLine = false;
    UE_LOG(LogTemp, Log, TEXT("[Turret] PredictionLine disabled"));
}

/** @brief 绘制预测弹道线和命中点 */
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
 * @brief 采集一帧图像并返回 Base64 编码的 JPEG 字符串
 * @param Quality JPEG 压缩质量
 * @return Base64 编码的 JPEG 数据
 * 手动触发 SceneCapture 采集一帧
 * 从 RenderTarget 读取像素数据
 * 转换为 BGRA 原始字节数组
 * 使用 IImageWrapper 编码为 JPEG
 * Base64 编码后返回
 */
FString ATurretPawn::CaptureImageBase64(int32 Quality)
{
    if (Quality <= 0) Quality = JpegQuality;
    if (!TurretSceneCapture || !TurretSceneCapture->TextureTarget) return TEXT("");
    TurretSceneCapture->CaptureScene();
    FTextureRenderTargetResource* Resource = TurretSceneCapture->TextureTarget->GameThread_GetRenderTargetResource();
    if (!Resource) return TEXT("");
    TArray<FColor> Pixels;
    int32 W = CameraWidth;
    int32 H = CameraHeight;
    Pixels.SetNum(W * H);
    FReadSurfaceDataFlags ReadFlags(RCM_UNorm, CubeFace_MAX);
    ReadFlags.SetLinearToGamma(false);
    if (!Resource->ReadPixels(Pixels, ReadFlags)) return TEXT("");
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
    if (!Wrapper->SetRaw(RawData.GetData(), RawData.Num(), W, H, ERGBFormat::BGRA, 8)) return TEXT("");
    auto JpegData = Wrapper->GetCompressed(Quality);
    return FBase64::Encode(JpegData.GetData(), JpegData.Num());
}

/** @brief 将 CineCamera 的 PostProcess 设置同步到 SceneCapture */
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

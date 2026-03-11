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
 * @brief 鏋勯€犲嚱鏁?
 * 鍒涘缓杞彴缃戞牸灞傜骇
 * 璁剧疆榛樿寮逛父绫讳负 ABulletActor
 * 鍒涘缓 UTurretAiming 鐬勫噯缁勪欢
 * 鍒涘缓 SceneCaptureComponent2D 鍜?CineCameraComponent 骞堕檮鐫€鍒版灙绠?
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

/** @brief 鍒涘缓 RenderTarget2D 鐢诲竷锛屾姇灏勫埌 TurretSceneCapture */
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
 * @param DeltaTime 甯ч棿闅旀椂闂达紙绉掞級
 *  鍚屾 CineCamera PP 鍒?SceneCapture
 *  GimbalMesh銆丟unMesh 鏃嬭浆
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
 * @brief 璁剧疆杞彴鐩爣瑙掑害
 * @param NewTargetPitch 鐩爣淇话瑙?
 * @param NewTargetYaw 鐩爣鍋忚埅瑙?
 */
void ATurretPawn::SetTargetAngles(float NewTargetPitch, float NewTargetYaw)
{
    TargetPitch = NewTargetPitch;
    TargetYaw = NewTargetYaw;
}

/**
 * @brief 閫氳繃 Agent ID 寮€濮嬭嚜鍔ㄨ窡韪洰鏍?
 * @param TargetActorID 鐩爣 Agent ID
 */
void ATurretPawn::StartTracking(const FString& TargetActorID)
{
    if (AimingComponent) AimingComponent->StartTracking(TargetActorID);
}

/** @brief 鍋滄鑷姩璺熻釜 */
void ATurretPawn::StopTracking()
{
    if (AimingComponent) AimingComponent->StopTracking();
}

/**
 * @brief 鏌ヨ鏄惁姝ｅ湪璺熻釜鐩爣
 * @return true 琛ㄧず姝ｅ湪璺熻釜
 */
bool ATurretPawn::IsTracking() const
{
    return AimingComponent ? AimingComponent->IsTracking() : false;
}

/**
 * @brief 鍙戝皠寮逛父
 * @param InitialSpeed 寮逛父鍒濆閫熷害
 * 鐢熸垚 ABulletActor 骞朵紶鍏ヨ建杩?
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
 * @brief 浣跨敤 BC 寮归亾搴撹绠楀脊閬撹建杩?
 * @param StartPos 鍙戝皠璧风偣
 * @param ShootDir 鍙戝皠鏂瑰悜
 * @param InitialSpeed 鍒濆閫熷害
 * @param OutTotalTime 寮归亾鎬婚琛屾椂闂?
 * @return 寮归亾杞ㄨ抗鐐规暟缁?
 * 寮硅嵂鍙傛暟锛欱C=0.295锛屽脊閲?5g锛屽脊寰?3.8mm
 * 闃诲姏琛細G7
 * 澶ф皵锛欼CAO 鏍囧噯澶ф皵锛屾俯搴?15掳C
 * 鏈€澶у皠绋嬶細500m锛屾椂闂存闀?0.01s
 */
TArray<FVector> ATurretPawn::Ballistic(FVector StartPos, FRotator ShootDir, float InitialSpeed, float& OutTotalTime)
{
    TArray<FVector> ResultPath;
    OutTotalTime = 0.0f;
    std::vector<DragDataPoint> dragTable;   // 闃诲姏琛?
    try {dragTable = DragTables::getTable("G7");}
    catch (...) {dragTable = DragTables::getTable("G1");}
    double bc_value = 0.295;    // 寮逛父寮归亾绯绘暟
    Weight bulletWeight = Weight::Grams(5.0);  // 寮逛父閲嶉噺
    Distance bulletDiameter = Distance::Millimeters(3.8);  // 寮逛父鐩村緞
    Distance bulletLength = Distance::Millimeters(3.8);  // 寮逛父闀垮害
    DragModel dragModel(bc_value, dragTable, bulletWeight, bulletDiameter, bulletLength);   //// 瀛愬脊闃诲姏妯″瀷
    Velocity muzzleVel = Velocity::MPS(InitialSpeed);  // 寮逛父鍒濋€熷害
    Temperature powderTemp = Temperature::Celsius(15.0);    // 寮硅嵂娓╁害
    Ammunition ammo(dragModel, muzzleVel, powderTemp, 0.0, true);   //// 寮硅嵂鍙傛暟
    Distance altitude = Distance::Meters(StartPos.Z / 100.0);  // 娴锋嫈楂樺害
    Atmosphere atmo = Atmosphere::ICAO(altitude, Temperature::Celsius(15), 0.0);  //// 澶ф皵妯″瀷
    Distance sightHeight = Distance::Centimeters(0.0);  // 鐬勫噯闀滈珮搴?
    Distance twist = Distance::Inches(0.0);  // 鑶涚嚎缂犺窛
    Weapon weapon(sightHeight, twist, Angular::Degrees(0));  //// 姝﹀櫒鍙傛暟
    Velocity windSpeed = Velocity::MPS(0.0);  // 椋庨€?
    Angular windDir = Angular::Degrees(0.0);  // 椋庡悜
    Wind wind1(windSpeed, windDir);     //// 椋?
    std::vector<Wind> winds = { wind1 };    
    Angular lookAngle = Angular::Degrees(0);  // 鐬勫噯鏂瑰悜
    double azimuth = 0;  // 鏂逛綅瑙?
    double latitude = 0.0;  // 绾害
    Angular cantAngle = Angular::Degrees(0);  // 鏋彛鍊捐
    Shot shot(ammo, atmo, weapon, winds, lookAngle, Angular::Degrees(0), cantAngle, azimuth, latitude);     //// 鍏ㄩ儴鍙傛暟
    Calculator calculator;
    double time_step = 0.01;    // 鏃堕棿姝ラ暱
    double max_dist = 500.0;    // 鏈€澶у皠绋?
    std::vector<TrajectoryPoint> vT = calculator.fire(shot, Distance::Meters(max_dist), Distance::Meters(0), time_step);
    if (!vT.empty()) OutTotalTime = vT.back().time;
    FTransform GunTransform(ShootDir, StartPos);    
    for (const auto& Pt : vT)
    {
        double RelX_m = Pt.distance.Meters();    // 鍓嶆柟璺濈
        double RelZ_m = Pt.height.Meters();      // 寮归亾楂樺害锛堝惈閲嶅姏涓嬮檷锛?
        double RelY_m = Pt.windage.Meters();     // 妯悜鍋忕Щ锛堥鍋忥級
        FVector LocalPos(RelX_m * 100.0, RelY_m * 100.0, RelZ_m * 100.0);
        ResultPath.Add(GunTransform.TransformPosition(LocalPos));
    }
    return ResultPath;
}


/** @brief 鍒涘缓杞彴缃戞牸缁勪欢 */
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

/** @brief 鍚敤棰勬祴寮归亾绾挎樉绀?*/
void ATurretPawn::ShowPredictionLine()
{
    bShowPredictionLine = true;
    UE_LOG(LogTemp, Log, TEXT("[Turret] PredictionLine enabled"));
}

/** @brief 绂佺敤棰勬祴寮归亾绾挎樉绀?*/
void ATurretPawn::HidePredictionLine()
{
    bShowPredictionLine = false;
    UE_LOG(LogTemp, Log, TEXT("[Turret] PredictionLine disabled"));
}

/** @brief 缁樺埗棰勬祴寮归亾绾垮拰鍛戒腑鐐?*/
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
 * @brief 閲囬泦涓€甯у浘鍍忓苟杩斿洖 Base64 缂栫爜鐨?JPEG 瀛楃涓?
 * @param Quality JPEG 鍘嬬缉璐ㄩ噺
 * @return Base64 缂栫爜鐨?JPEG 鏁版嵁
 * 鎵嬪姩瑙﹀彂 SceneCapture 閲囬泦涓€甯?
 * 浠?RenderTarget 璇诲彇鍍忕礌鏁版嵁
 * 杞崲涓?BGRA 鍘熷瀛楄妭鏁扮粍
 * 浣跨敤 IImageWrapper 缂栫爜涓?JPEG
 * Base64 缂栫爜鍚庤繑鍥?
 */
FString ATurretPawn::CaptureImageBase64(int32 Quality)
{
    if (Quality <= 0)
    {
        Quality = JpegQuality;
    }

    return CameraCaptureUtils::CaptureColorJpegBase64(TurretSceneCapture, CameraWidth, CameraHeight, Quality);
}

/** @brief 灏?CineCamera 鐨?PostProcess 璁剧疆鍚屾鍒?SceneCapture */
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




#include "TurretPawn.h"

#include "BC_Ammunition.hpp"
#include "BC_Atmosphere.hpp"
#include "BC_Calculator.hpp"
#include "BC_DragModel.hpp"
#include "BC_DragTables.hpp"
#include "BC_Shot.hpp"
#include "BC_Unit.hpp"
#include "BC_Weapon.hpp"
#include "BC_Wind.hpp"
#include "BulletActor.h"
#include "DrawDebugHelpers.h"
#include "Engine/CollisionProfile.h"
#include "GraduationProject/Core/Manager/AgentManager.h"
#include "GraduationProject/Vision/CameraCaptureUtils.h"
#include "TurretAiming.h"

namespace
{
    FString JsonEscape(const FString& In)
    {
        FString Out = In;
        Out.ReplaceInline(TEXT("\\"), TEXT("\\\\"));
        Out.ReplaceInline(TEXT("\""), TEXT("\\\""));
        return Out;
    }
}

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

void ATurretPawn::BeginPlay()
{
    Super::BeginPlay();

    CurrentPitch = 0.0f;
    CurrentYaw = 0.0f;

    if (UAgentManager* Manager = UAgentManager::GetInstance())
    {
        const FString ResolvedId = Manager->RegisterOrResolveAgent(TurretId, this);
        if (!ResolvedId.IsEmpty())
        {
            TurretId = ResolvedId;
        }
    }

    if (UTextureRenderTarget2D* RT = CameraCaptureUtils::CreateColorRenderTarget(this, CameraWidth, CameraHeight))
    {
        TurretSceneCapture->TextureTarget = RT;
    }
    TurretSceneCapture->FOVAngle = CameraFOV;
    CameraCaptureUtils::ApplySegmentationStencil(this, SegmentationId);
}

void ATurretPawn::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    CameraCaptureUtils::SyncPostProcessToCapture(TurretCineCamera, TurretSceneCapture, ExposureBias);

    const float YawDiff = FRotator::NormalizeAxis(TargetYaw - CurrentYaw);
    const float AdjustedTargetYaw = CurrentYaw + YawDiff;
    CurrentYaw = FMath::FInterpTo(CurrentYaw, AdjustedTargetYaw, DeltaTime, RotationSpeed);
    CurrentPitch = FMath::FInterpTo(CurrentPitch, TargetPitch, DeltaTime, RotationSpeed);

    if (GimbalMesh)
    {
        GimbalMesh->SetRelativeRotation(FRotator(0.0f, CurrentYaw, 0.0f));
    }
    if (GunMesh)
    {
        GunMesh->SetRelativeRotation(FRotator(-CurrentPitch, 0.0f, 0.0f));
    }
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
    if (!GunMesh || !GetWorld())
    {
        return;
    }

    const FRotator CurrentGunRot = GunMesh->GetComponentRotation();
    FRotator MuzzleRotation = CurrentGunRot;
    MuzzleRotation.Pitch *= -1.0f;
    MuzzleRotation.Yaw += 180.0f;
    const FVector MuzzleLocation = GunMesh->GetComponentLocation() + CurrentGunRot.RotateVector(MuzzleOffset);

    float TrajectoryDuration = 0.0f;
    const TArray<FVector> PathPoints = Ballistic(MuzzleLocation, MuzzleRotation, InitialSpeed, TrajectoryDuration);
    if (!BulletClass || PathPoints.Num() == 0)
    {
        return;
    }

    FActorSpawnParameters SpawnParams;
    SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    SpawnParams.Owner = this;

    if (ABulletActor* Bullet = GetWorld()->SpawnActor<ABulletActor>(BulletClass, MuzzleLocation, MuzzleRotation, SpawnParams))
    {
        Bullet->InitTrajectory(PathPoints, TrajectoryDuration);
    }
}

TArray<FVector> ATurretPawn::Ballistic(FVector StartPos, FRotator ShootDir, float InitialSpeed, float& OutTotalTime)
{
    TArray<FVector> ResultPath;
    OutTotalTime = 0.0f;

    std::vector<DragDataPoint> DragTable;
    try
    {
        DragTable = DragTables::getTable("G7");
    }
    catch (...)
    {
        DragTable = DragTables::getTable("G1");
    }

    const double BcValue = 0.295;
    const Weight BulletWeight = Weight::Grams(5.0);
    const Distance BulletDiameter = Distance::Millimeters(3.8);
    const Distance BulletLength = Distance::Millimeters(3.8);
    const DragModel DragModelValue(BcValue, DragTable, BulletWeight, BulletDiameter, BulletLength);
    const Velocity MuzzleVel = Velocity::MPS(InitialSpeed);
    const Temperature PowderTemp = Temperature::Celsius(15.0);
    const Ammunition Ammo(DragModelValue, MuzzleVel, PowderTemp, 0.0, true);
    const Distance Altitude = Distance::Meters(StartPos.Z / 100.0);
    const Atmosphere Atmo = Atmosphere::ICAO(Altitude, Temperature::Celsius(15), 0.0);
    const Weapon WeaponValue(Distance::Centimeters(0.0), Distance::Inches(0.0), Angular::Degrees(0));
    const std::vector<Wind> Winds = { Wind(Velocity::MPS(0.0), Angular::Degrees(0.0)) };
    Shot ShotValue(Ammo, Atmo, WeaponValue, Winds, Angular::Degrees(0), Angular::Degrees(0), Angular::Degrees(0), 0, 0.0);
    Calculator CalculatorValue;

    const std::vector<TrajectoryPoint> Trajectory = CalculatorValue.fire(ShotValue, Distance::Meters(500.0), Distance::Meters(0), 0.01);
    if (!Trajectory.empty())
    {
        OutTotalTime = static_cast<float>(Trajectory.back().time);
    }

    const FTransform GunTransform(ShootDir, StartPos);
    for (const TrajectoryPoint& Point : Trajectory)
    {
        const FVector LocalPos(
            Point.distance.Meters() * 100.0,
            Point.windage.Meters() * 100.0,
            Point.height.Meters() * 100.0);
        ResultPath.Add(GunTransform.TransformPosition(LocalPos));
    }

    return ResultPath;
}

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

void ATurretPawn::ShowPredictionLine()
{
    bShowPredictionLine = true;
}

void ATurretPawn::HidePredictionLine()
{
    bShowPredictionLine = false;
}

void ATurretPawn::EnableApiControl(bool bEnable)
{
    bApiControlEnabled = bEnable;
}

void ATurretPawn::ResetTurret()
{
    SetTargetAngles(0.0f, 0.0f);
    StopTracking();
    HidePredictionLine();
}

FString ATurretPawn::GetState()
{
    const FVector Pos = GetActorLocation();
    return FString::Printf(
        TEXT("{\"status\":\"ok\",\"id\":\"%s\",\"pitch\":%.2f,\"yaw\":%.2f,\"target_pitch\":%.2f,\"target_yaw\":%.2f,\"is_tracking\":%s,\"api_control\":%s,\"position\":[%.1f,%.1f,%.1f]}"),
        *JsonEscape(TurretId),
        GetCurrentPitch(),
        GetCurrentYaw(),
        GetTargetPitchAngle(),
        GetTargetYawAngle(),
        IsTracking() ? TEXT("true") : TEXT("false"),
        bApiControlEnabled ? TEXT("true") : TEXT("false"),
        Pos.X,
        Pos.Y,
        Pos.Z);
}

FString ATurretPawn::GetImage(FString ImageType, int32 Quality, float MaxDepthMeters)
{
    return CameraCaptureUtils::CaptureAirSimImageJson(
        TurretSceneCapture,
        TurretId,
        CameraWidth,
        CameraHeight,
        CameraFOV,
        JpegQuality,
        ImageType,
        Quality,
        MaxDepthMeters);
}

void ATurretPawn::DrawPredictionLine()
{
    if (!GunMesh || !GetWorld())
    {
        return;
    }

    const FRotator CurrentGunRot = GunMesh->GetComponentRotation();
    FRotator MuzzleRotation = CurrentGunRot;
    MuzzleRotation.Pitch *= -1.0f;
    MuzzleRotation.Yaw += 180.0f;
    const FVector MuzzleLocation = GunMesh->GetComponentLocation() + CurrentGunRot.RotateVector(MuzzleOffset);

    float TrajectoryDuration = 0.0f;
    const TArray<FVector> PathPoints = Ballistic(MuzzleLocation, MuzzleRotation, DefaultMuzzleSpeed, TrajectoryDuration);
    if (PathPoints.Num() < 2)
    {
        return;
    }

    for (int32 Index = 0; Index < PathPoints.Num() - 1; ++Index)
    {
        DrawDebugLine(GetWorld(), PathPoints[Index], PathPoints[Index + 1], PredictionLineColor, false, -1.0f, 0, PredictionLineThickness);
    }

    FHitResult Hit;
    FCollisionQueryParams QueryParams;
    QueryParams.AddIgnoredActor(this);
    for (int32 Index = 0; Index < PathPoints.Num() - 1; ++Index)
    {
        if (GetWorld()->LineTraceSingleByChannel(Hit, PathPoints[Index], PathPoints[Index + 1], ECC_Visibility, QueryParams))
        {
            DrawDebugSphere(GetWorld(), Hit.ImpactPoint, PredictionHitRadius, 8, PredictionHitColor, false, -1.0f);
            break;
        }
    }
}

FString ATurretPawn::CaptureImageBase64(int32 Quality)
{
    if (Quality <= 0)
    {
        Quality = JpegQuality;
    }

    return CameraCaptureUtils::CaptureColorJpegBase64(TurretSceneCapture, CameraWidth, CameraHeight, Quality);
}


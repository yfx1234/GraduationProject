/**
 * @file TurretAiming.cpp
 * @brief 转台瞄准计算组件的实现文件
 *
 * 实现 CalculateAimAngles() 瞄准角度计算，
 * 支持从转台 Yaw 轴到目标方向的角度分解。
 */

#include "TurretAiming.h"
#include "TurretPawn.h"
#include "GraduationProject/Core/Manager/AgentManager.h"

UTurretAiming::UTurretAiming()
{
    PrimaryComponentTick.bCanEverTick = true;
}

void UTurretAiming::BeginPlay()
{
    Super::BeginPlay();

    OwnerTurret = Cast<ATurretPawn>(GetOwner());
    if (!OwnerTurret)
    {
        UE_LOG(LogTemp, Warning, TEXT("[TurretAiming] Owner is not ATurretPawn!"));
    }
}

void UTurretAiming::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    if (bIsTracking)
    {
        UpdateTracking(DeltaTime);
    }
}

void UTurretAiming::StartTracking(const FString& TargetActorID)
{
    if (!GetWorld()) return;

    // 通过 AgentManager 查找已注册的 Agent
    AActor* FoundActor = UAgentManager::GetInstance()->GetAgent(TargetActorID);

    if (FoundActor)
    {
        SetTrackingTarget(FoundActor);
        UE_LOG(LogTemp, Log, TEXT("[TurretAiming] Start tracking: %s (Actor: %s)"), *TargetActorID, *FoundActor->GetName());
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("[TurretAiming] Target not found in AgentManager: %s"), *TargetActorID);
    }
}

void UTurretAiming::StopTracking()
{
    bIsTracking = false;
    TrackingTarget = nullptr;
    UE_LOG(LogTemp, Log, TEXT("[TurretAiming] Stop tracking"));
}

void UTurretAiming::SetTrackingTarget(AActor* Target)
{
    TrackingTarget = Target;
    bIsTracking = (Target != nullptr);
}

void UTurretAiming::UpdateTracking(float DeltaTime)
{
    if (!OwnerTurret || !TrackingTarget || !IsValid(TrackingTarget))
    {
        StopTracking();
        return;
    }

    FVector TargetPos = GetTargetPosition();
    float MuzzleSpeed = OwnerTurret->DefaultMuzzleSpeed;

    float AimPitch, AimYaw;
    if (CalculateAimAngles(TargetPos, MuzzleSpeed, AimPitch, AimYaw))
    {
        OwnerTurret->SetTargetAngles(AimPitch, AimYaw);
    }
}

FVector UTurretAiming::GetTargetPosition() const
{
    if (TrackingTarget)
    {
        return TrackingTarget->GetActorLocation();
    }
    return FVector::ZeroVector;
}

bool UTurretAiming::CalculateAimAngles(const FVector& TargetPos, float MuzzleSpeed, float& OutPitch, float& OutYaw)
{
    if (!OwnerTurret || !OwnerTurret->GunMesh)
    {
        return false;
    }

    FVector TurretPos = OwnerTurret->GetActorLocation();
    FRotator TurretRot = OwnerTurret->GetActorRotation();

    FVector MuzzleOff = OwnerTurret->MuzzleOffset;

    // 计算目标相对转台方向
    FVector ToTargetFromTurret = TargetPos - TurretPos;
    FRotator InitialDirection = ToTargetFromTurret.Rotation();
    FRotator LocalDirection = InitialDirection - TurretRot;

    // 估算枪口位置
    FRotator EstimatedGunRot = TurretRot;
    EstimatedGunRot.Yaw += LocalDirection.Yaw + 180.0f;
    EstimatedGunRot.Pitch = -LocalDirection.Pitch;
    FVector EstimatedMuzzlePos = TurretPos + EstimatedGunRot.RotateVector(MuzzleOff);

    // 从枪口位置重新计算方向
    FVector ToTargetFromMuzzle = TargetPos - EstimatedMuzzlePos;
    FRotator FinalDirection = ToTargetFromMuzzle.Rotation();
    FRotator FinalLocalDirection = FinalDirection - TurretRot;

    float RawYaw = FinalLocalDirection.Yaw + 180.0f;
    float RawPitch = FinalLocalDirection.Pitch;

    // 规范 Yaw 在 -180 到 180 之间
    OutYaw = FMath::Fmod(RawYaw + 180.0f, 360.0f) - 180.0f;
    OutPitch = RawPitch;

    return true;
}

FVector UTurretAiming::GetMuzzleWorldPosition(float Pitch, float Yaw) const
{
    if (!OwnerTurret || !OwnerTurret->GunMesh)
    {
        return FVector::ZeroVector;
    }

    FVector TurretPos = OwnerTurret->GetActorLocation();
    FRotator TurretRot = OwnerTurret->GetActorRotation();

    FRotator GunWorldRot = TurretRot;
    GunWorldRot.Yaw += Yaw;
    GunWorldRot.Pitch = -Pitch;

    FVector MuzzleOff = OwnerTurret->MuzzleOffset;
    FVector MuzzleWorldPos = TurretPos + GunWorldRot.RotateVector(MuzzleOff);

    return MuzzleWorldPos;
}

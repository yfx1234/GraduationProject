#include "TurretAiming.h"
#include "TurretPawn.h"
#include "GraduationProject/Core/Manager/AgentManager.h"

/**
 * @brief 构造函数
 * 组件在跟踪状态时每帧更新瞄准角度
 */
UTurretAiming::UTurretAiming()
{
    PrimaryComponentTick.bCanEverTick = true;
}

/**
 * @brief 游戏开始时调用
 * 获取 Owner Actor 并转型为 ATurretPawn
 */
void UTurretAiming::BeginPlay()
{
    Super::BeginPlay();
    OwnerTurret = Cast<ATurretPawn>(GetOwner());
    if (!OwnerTurret) UE_LOG(LogTemp, Warning, TEXT("[TurretAiming] Owner is not ATurretPawn!"));
}

/**
 * @brief 每帧组件更新
 * @param DeltaTime 帧间隔时间（秒）
 * @param TickType Tick 类型
 * @param ThisTickFunction Tick 函数指针
 */
void UTurretAiming::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
    if (bIsTracking) UpdateTracking(DeltaTime);
}

/**
 * @brief 通过 Agent ID 开始跟踪目标
 * @param TargetActorID 目标 Agent ID
 */
void UTurretAiming::StartTracking(const FString& TargetActorID)
{
    if (!GetWorld()) return;
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

/**
 * @brief 停止跟踪
 */
void UTurretAiming::StopTracking()
{
    bIsTracking = false;
    TrackingTarget = nullptr;
    UE_LOG(LogTemp, Log, TEXT("[TurretAiming] Stop tracking"));
}

/**
 * @brief 直接传入 Actor 指针设置跟踪目标
 * @param Target 目标 Actor 指针
 */
void UTurretAiming::SetTrackingTarget(AActor* Target)
{
    TrackingTarget = Target;
    bIsTracking = (Target != nullptr);
}

/**
 * @brief 每帧更新跟踪
 * @param DeltaTime 帧间隔（秒）
 * CalculateAimAngles() 计算瞄准角度，
 */
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
        OwnerTurret->SetTargetAngles(AimPitch, AimYaw);
}

/**
 * @brief 获取跟踪目标的当前世界位置
 * @return 目标 Actor 的世界位置
 */
FVector UTurretAiming::GetTargetPosition() const
{
    if (TrackingTarget) return TrackingTarget->GetActorLocation();
    return FVector::ZeroVector;
}

/**
 * @brief 计算瞄准角度（Pitch/Yaw）
 * @param TargetPos 目标世界位置
 * @param MuzzleSpeed 弹丸出膛速度
 * @param OutPitch 计算得到的俯仰角
 * @param OutYaw 计算得到的偏航角
 * @return true 表示计算成功
 * 两步计算：
 * 先从转台中心到目标的方向估算初始角度
 * 用初始角度估算枪口位置，再从枪口到目标重新计算最终角度
 * 补偿枪口相对转台中心的偏移量。
 */
bool UTurretAiming::CalculateAimAngles(const FVector& TargetPos, float MuzzleSpeed, float& OutPitch, float& OutYaw)
{
    if (!OwnerTurret || !OwnerTurret->GunMesh) return false;

    FVector TurretPos = OwnerTurret->GetActorLocation();
    FRotator TurretRot = OwnerTurret->GetActorRotation();
    FVector ToTargetFromTurret = TargetPos - TurretPos;
    FRotator InitialDirection = ToTargetFromTurret.Rotation();
    FRotator LocalDirection = InitialDirection - TurretRot;

    // Reuse helper to avoid duplicated muzzle-transform logic.
    FVector EstimatedMuzzlePos = GetMuzzleWorldPosition(LocalDirection.Pitch, LocalDirection.Yaw + 180.0f);
    FVector ToTargetFromMuzzle = TargetPos - EstimatedMuzzlePos;
    FRotator FinalDirection = ToTargetFromMuzzle.Rotation();
    FRotator FinalLocalDirection = FinalDirection - TurretRot;

    float RawYaw = FinalLocalDirection.Yaw + 180.0f;
    float RawPitch = FinalLocalDirection.Pitch;
    OutYaw = FMath::Fmod(RawYaw + 180.0f, 360.0f) - 180.0f;
    OutPitch = RawPitch;
    return true;
}

/**
 * @brief 根据给定角度计算枪口世界位置
 * @param Pitch 俯仰角
 * @param Yaw 偏航角
 * @return 枪口世界位置
 * 将转台世界旋转叠加给定的 Yaw 和 Pitch
 * 再用该旋转变换 MuzzleOffset 得到枪口世界位置
 */
FVector UTurretAiming::GetMuzzleWorldPosition(float Pitch, float Yaw) const
{
    if (!OwnerTurret || !OwnerTurret->GunMesh) return FVector::ZeroVector;
    FVector TurretPos = OwnerTurret->GetActorLocation();
    FRotator TurretRot = OwnerTurret->GetActorRotation();
    FRotator GunWorldRot = TurretRot;
    GunWorldRot.Yaw += Yaw;
    GunWorldRot.Pitch = -Pitch;
    FVector MuzzleOff = OwnerTurret->MuzzleOffset;
    FVector MuzzleWorldPos = TurretPos + GunWorldRot.RotateVector(MuzzleOff);
    return MuzzleWorldPos;
}




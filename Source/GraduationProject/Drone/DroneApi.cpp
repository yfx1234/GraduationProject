#include "DroneApi.h"
#include "DronePawn.h"
#include "DroneMovementComponent.h"

/**
 * @brief 鍒濆鍖?API锛岀粦瀹氬埌鎸囧畾鐨?DronePawn
 * @param Owner 鎷ユ湁姝?API 鐨?DronePawn 瀹炰緥
 */
void UDroneApi::Initialize(ADronePawn* Owner)
{
    OwnerPawn = Owner;
}

/**
 * @brief 绉诲姩鍒版寚瀹氬潗鏍囦綅缃?
 * @param X 鐩爣 X 鍧愭爣 (m)
 * @param Y 鐩爣 Y 鍧愭爣 (m)
 * @param Z 鐩爣 Z 鍧愭爣 (m)
 * @param Speed 绉诲姩閫熷害 (m/s)
 */
void UDroneApi::MoveToPosition(float X, float Y, float Z, float Speed)
{
    if (OwnerPawn)
    {
        OwnerPawn->SetTargetPosition(FVector(X, Y, Z), Speed);
        UE_LOG(LogTemp, Log, TEXT("[DroneApi] MoveToPosition(%.1f, %.1f, %.1f), speed=%.2f"), X, Y, Z, Speed);
    }
}

/** @brief 鍦ㄥ綋鍓嶄綅缃偓鍋?*/
void UDroneApi::Hover()
{
    if (OwnerPawn) OwnerPawn->Hover();
}

/**
 * @brief 璧烽鍒版寚瀹氶珮搴?
 * @param Altitude 鐩爣椋炶楂樺害 (m)
 */
void UDroneApi::Takeoff(float Altitude)
{
    if (OwnerPawn) OwnerPawn->Takeoff(Altitude);
}

/** @brief 闄嶈惤鍒板湴闈?*/
void UDroneApi::Land()
{
    if (OwnerPawn) OwnerPawn->Land();
}

/**
 * @brief 鎸夐€熷害椋炶
 * @param Vx X 鏂瑰悜閫熷害 (m/s)
 * @param Vy Y 鏂瑰悜閫熷害 (m/s)
 * @param Vz Z 鏂瑰悜閫熷害 (m/s)
 */
void UDroneApi::MoveByVelocity(float Vx, float Vy, float Vz)
{
    if (OwnerPawn)
    {
        OwnerPawn->SetTargetVelocity(FVector(Vx, Vy, Vz));
        UE_LOG(LogTemp, Log, TEXT("[DroneApi] MoveByVelocity(%.1f, %.1f, %.1f)"), Vx, Vy, Vz);
    }
}
void UDroneApi::SetTargetAttitude(float RollDeg, float PitchDeg, float YawDeg, float Thrust)
{
    if (OwnerPawn && OwnerPawn->MovementComp)
    {
        OwnerPawn->MovementComp->SetControlMode(EDroneControlMode::AttitudeThrust);
        OwnerPawn->MovementComp->SetTargetAttitude(FRotator(PitchDeg, YawDeg, RollDeg), Thrust);
        OwnerPawn->ControlMode = EDroneControlMode::AttitudeThrust;
    }
}

void UDroneApi::SetMotorSpeeds(float M0, float M1, float M2, float M3)
{
    if (OwnerPawn && OwnerPawn->MovementComp)
    {
        OwnerPawn->MovementComp->SetControlMode(EDroneControlMode::MotorSpeed);
        OwnerPawn->MovementComp->SetControlCommand({ M0, M1, M2, M3 });
        OwnerPawn->ControlMode = EDroneControlMode::MotorSpeed;
    }
}
void UDroneApi::SetHeadingControl(EDroneYawMode YawMode, EDroneDrivetrainMode Drivetrain, float YawDeg)
{
    if (OwnerPawn)
    {
        OwnerPawn->SetHeadingControl(YawMode, Drivetrain, YawDeg);
    }
}

/**
 * @brief 鑾峰彇褰撳墠浣嶇疆
 * @return 浣嶇疆鍚戦噺 (m)
 */
FVector UDroneApi::GetPosition() const
{
    return OwnerPawn ? OwnerPawn->GetCurrentPosition() : FVector::ZeroVector;
}

/**
 * @brief 鑾峰彇褰撳墠閫熷害
 * @return 閫熷害鍚戦噺 (m/s)
 */
FVector UDroneApi::GetVelocity() const
{
    return OwnerPawn ? OwnerPawn->GetCurrentVelocity() : FVector::ZeroVector;
}

/**
 * @brief 鑾峰彇褰撳墠濮挎€?
 * @return 娆ф媺瑙?(Roll, Pitch, Yaw)
 */
FRotator UDroneApi::GetOrientation() const
{
    return OwnerPawn ? OwnerPawn->CurrentState.GetRotator() : FRotator::ZeroRotator;
}

/**
 * @brief 鑾峰彇鍥涗釜鐢垫満鐨勮浆閫?
 * @return 杞€熸暟缁?(rad/s)
 */
TArray<float> UDroneApi::GetMotorSpeeds() const
{
    TArray<float> Result;
    if (OwnerPawn && OwnerPawn->CurrentState.MotorSpeeds.Num() == 4)
    {
        for (double Speed : OwnerPawn->CurrentState.MotorSpeeds)
            Result.Add(static_cast<float>(Speed));
    }
    return Result;
}

/**
 * @brief 鑾峰彇褰撳墠鎺у埗妯″紡
 * @return 鎺у埗妯″紡鏋氫妇鍊?
 */
EDroneControlMode UDroneApi::GetControlMode() const
{
    return OwnerPawn ? OwnerPawn->ControlMode : EDroneControlMode::Idle;
}

/**
 * @brief 璁剧疆浣嶇疆鎺у埗鍣ㄥ鐩?
 * @param Kp 姣斾緥澧炵泭
 * @param Kd 寰垎澧炵泭
 */
void UDroneApi::SetPositionControllerGains(float Kp, float Kd)
{
    if (OwnerPawn && OwnerPawn->MovementComp)
        OwnerPawn->MovementComp->SetPositionGains(Kp, Kd);
}

/**
 * @brief 璁剧疆閫熷害鎺у埗鍣ㄥ鐩?
 * @param Kp 姣斾緥澧炵泭
 * @param Ki 绉垎澧炵泭
 * @param Kd 寰垎澧炵泭
 */
void UDroneApi::SetVelocityControllerGains(float Kp, float Ki, float Kd)
{
    if (OwnerPawn && OwnerPawn->MovementComp)
        OwnerPawn->MovementComp->SetVelocityGains(Kp, Ki, Kd);
}

/**
 * @brief 璁剧疆濮挎€佹帶鍒跺櫒澧炵泭
 * @param Kp 姣斾緥澧炵泭
 * @param Kd 寰垎澧炵泭
 */
void UDroneApi::SetAttitudeControllerGains(float Kp, float Kd)
{
    if (OwnerPawn && OwnerPawn->MovementComp)
        OwnerPawn->MovementComp->SetAttitudeGains(Kp, Kd);
}

/**
 * @brief 璁剧疆瑙掗€熺巼鎺у埗鍣ㄥ鐩?
 * @param Kp 姣斾緥澧炵泭
 */
void UDroneApi::SetAngleRateControllerGains(float Kp)
{
    if (OwnerPawn && OwnerPawn->MovementComp)
        OwnerPawn->MovementComp->SetAngleRateGains(Kp);
}

/**
 * @brief 閲嶇疆鏃犱汉鏈哄埌鎸囧畾浣嶇疆鍜屽Э鎬?
 * @param Position 鐩爣浣嶇疆 (m)
 * @param Rotation 鐩爣濮挎€?
 */
void UDroneApi::Reset(FVector Position, FRotator Rotation)
{
    if (OwnerPawn) OwnerPawn->ResetDrone(Position, Rotation);
}


#include "DroneApi.h"
#include "DronePawn.h"
#include "DroneMovementComponent.h"

/**
 * @brief 初始化无人机 API
 * @param Owner 目标无人机 Pawn
 */
void UDroneApi::Initialize(ADronePawn* Owner)
{
    OwnerPawn = Owner;
}

/**
 * @brief 以位置模式飞往指定坐标
 * @param X 目标 X 坐标（米）
 * @param Y 目标 Y 坐标（米）
 * @param Z 目标 Z 坐标（米）
 * @param Speed 飞行速度（米/秒）
 */
void UDroneApi::MoveToPosition(float X, float Y, float Z, float Speed)
{
    if (OwnerPawn)
    {
        OwnerPawn->SetTargetPosition(FVector(X, Y, Z), Speed);
        UE_LOG(LogTemp, Log, TEXT("[DroneApi] MoveToPosition(%.1f, %.1f, %.1f), speed=%.2f"), X, Y, Z, Speed);
    }
}

/** @brief 在当前位置悬停 */
void UDroneApi::Hover()
{
    if (OwnerPawn) OwnerPawn->Hover();
}

/**
 * @brief 起飞到指定高度
 * @param Altitude 目标飞行高度（米）
 */
void UDroneApi::Takeoff(float Altitude)
{
    if (OwnerPawn) OwnerPawn->Takeoff(Altitude);
}

/** @brief 执行降落命令 */
void UDroneApi::Land()
{
    if (OwnerPawn) OwnerPawn->Land();
}

/**
 * @brief 以速度模式飞行
 * @param Vx X 方向速度（米/秒）
 * @param Vy Y 方向速度（米/秒）
 * @param Vz Z 方向速度（米/秒）
 */
void UDroneApi::MoveByVelocity(float Vx, float Vy, float Vz)
{
    if (OwnerPawn)
    {
        OwnerPawn->SetTargetVelocity(FVector(Vx, Vy, Vz));
        UE_LOG(LogTemp, Log, TEXT("[DroneApi] MoveByVelocity(%.1f, %.1f, %.1f)"), Vx, Vy, Vz);
    }
}

/**
 * @brief 设置目标姿态和总推力
 * @param RollDeg 目标横滚角（度）
 * @param PitchDeg 目标俯仰角（度）
 * @param YawDeg 目标偏航角（度）
 * @param Thrust 目标总推力
 */
void UDroneApi::SetTargetAttitude(float RollDeg, float PitchDeg, float YawDeg, float Thrust)
{
    if (OwnerPawn && OwnerPawn->MovementComp)
    {
        OwnerPawn->MovementComp->SetControlMode(EDroneControlMode::AttitudeThrust);
        OwnerPawn->MovementComp->SetTargetAttitude(FRotator(PitchDeg, YawDeg, RollDeg), Thrust);
        OwnerPawn->ControlMode = EDroneControlMode::AttitudeThrust;
    }
}

/**
 * @brief 直接设置电机转速命令
 * @param M0 电机 0 转速（rad/s）
 * @param M1 电机 1 转速（rad/s）
 * @param M2 电机 2 转速（rad/s）
 * @param M3 电机 3 转速（rad/s）
 */
void UDroneApi::SetMotorSpeeds(float M0, float M1, float M2, float M3)
{
    if (OwnerPawn && OwnerPawn->MovementComp)
    {
        OwnerPawn->MovementComp->SetControlMode(EDroneControlMode::MotorSpeed);
        OwnerPawn->MovementComp->SetControlCommand({ M0, M1, M2, M3 });
        OwnerPawn->ControlMode = EDroneControlMode::MotorSpeed;
    }
}

/**
 * @brief 设置航向控制模式
 * @param YawMode 偏航控制模式
 * @param Drivetrain 运动学约束模式
 * @param YawDeg 目标偏航角
 */
void UDroneApi::SetHeadingControl(EDroneYawMode YawMode, EDroneDrivetrainMode Drivetrain, float YawDeg)
{
    if (OwnerPawn)
    {
        OwnerPawn->SetHeadingControl(YawMode, Drivetrain, YawDeg);
    }
}

/**
 * @brief 获取当前位置
 * @return 位置向量（米）
 */
FVector UDroneApi::GetPosition() const
{
    return OwnerPawn ? OwnerPawn->GetCurrentPosition() : FVector::ZeroVector;
}

/**
 * @brief 获取当前速度
 * @return 速度向量（米/秒）
 */
FVector UDroneApi::GetVelocity() const
{
    return OwnerPawn ? OwnerPawn->GetCurrentVelocity() : FVector::ZeroVector;
}

/**
 * @brief 获取当前姿态
 * @return 欧拉角姿态
 */
FRotator UDroneApi::GetOrientation() const
{
    return OwnerPawn ? OwnerPawn->CurrentState.GetRotator() : FRotator::ZeroRotator;
}

/**
 * @brief 获取当前四个电机转速
 * @return 电机转速数组（rad/s）
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
 * @brief 获取当前控制模式
 * @return 控制模式枚举值
 */
EDroneControlMode UDroneApi::GetControlMode() const
{
    return OwnerPawn ? OwnerPawn->ControlMode : EDroneControlMode::Idle;
}

/**
 * @brief 设置位置控制器增益
 * @param Kp 比例增益
 * @param Kd 微分增益
 */
void UDroneApi::SetPositionControllerGains(float Kp, float Kd)
{
    if (OwnerPawn && OwnerPawn->MovementComp)
        OwnerPawn->MovementComp->SetPositionGains(Kp, Kd);
}

/**
 * @brief 设置速度控制器增益
 * @param Kp 比例增益
 * @param Ki 积分增益
 * @param Kd 微分增益
 */
void UDroneApi::SetVelocityControllerGains(float Kp, float Ki, float Kd)
{
    if (OwnerPawn && OwnerPawn->MovementComp)
        OwnerPawn->MovementComp->SetVelocityGains(Kp, Ki, Kd);
}

/**
 * @brief 设置姿态控制器增益
 * @param Kp 比例增益
 * @param Kd 微分增益
 */
void UDroneApi::SetAttitudeControllerGains(float Kp, float Kd)
{
    if (OwnerPawn && OwnerPawn->MovementComp)
        OwnerPawn->MovementComp->SetAttitudeGains(Kp, Kd);
}

/**
 * @brief 设置角速度控制器比例增益
 * @param Kp 比例增益
 */
void UDroneApi::SetAngleRateControllerGains(float Kp)
{
    if (OwnerPawn && OwnerPawn->MovementComp)
        OwnerPawn->MovementComp->SetAngleRateGains(Kp);
}

/**
 * @brief 重置无人机位置和姿态
 * @param Position 目标位置（米）
 * @param Rotation 目标姿态
 */
void UDroneApi::Reset(FVector Position, FRotator Rotation)
{
    if (OwnerPawn) OwnerPawn->ResetDrone(Position, Rotation);
}

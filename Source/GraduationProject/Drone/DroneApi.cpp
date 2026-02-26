/**
 * @file DroneApi.cpp
 * @brief 无人机统一 API 接口的实现文件
 *
 * 实现 AirSim 风格的统一 API，将高层控制命令委托给 DronePawn 执行。
 * 所有方法在 OwnerPawn 为空时安全返回默认值。
 */

#include "DroneApi.h"
#include "DronePawn.h"
#include "DroneMovementComponent.h"

/**
 * @brief 初始化 API，绑定到指定的 DronePawn
 * @param Owner 拥有此 API 的 DronePawn 实例
 */
void UDroneApi::Initialize(ADronePawn* Owner)
{
    OwnerPawn = Owner;
}

/**
 * @brief 移动到指定 SI 坐标位置
 * @param X 目标 X 坐标 (m)
 * @param Y 目标 Y 坐标 (m)
 * @param Z 目标 Z 坐标 (m)
 * @param Speed 移动速度 (m/s)，当前未直接使用
 *
 * 将目标位置传给 DronePawn::SetTargetPosition()，
 * 内部通过级联 PID 控制器自动飞向目标。
 */
void UDroneApi::MoveToPosition(float X, float Y, float Z, float Speed)
{
    if (OwnerPawn)
    {
        OwnerPawn->SetTargetPosition(FVector(X, Y, Z));
        UE_LOG(LogTemp, Log, TEXT("[DroneApi] MoveToPosition(%.1f, %.1f, %.1f)"), X, Y, Z);
    }
}

/** @brief 在当前位置悬停 */
void UDroneApi::Hover()
{
    if (OwnerPawn) OwnerPawn->Hover();
}

/**
 * @brief 起飞到指定高度
 * @param Altitude 目标飞行高度 (m)
 */
void UDroneApi::Takeoff(float Altitude)
{
    if (OwnerPawn) OwnerPawn->Takeoff(Altitude);
}

/** @brief 降落到地面 */
void UDroneApi::Land()
{
    if (OwnerPawn) OwnerPawn->Land();
}

/**
 * @brief 按速度飞行
 * @param Vx X 方向速度 (m/s)
 * @param Vy Y 方向速度 (m/s)
 * @param Vz Z 方向速度 (m/s)
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
 * @brief 获取当前位置
 * @return 位置向量 (m)，无人机无效时返回零向量
 */
FVector UDroneApi::GetPosition() const
{
    return OwnerPawn ? OwnerPawn->GetCurrentPosition() : FVector::ZeroVector;
}

/**
 * @brief 获取当前速度
 * @return 速度向量 (m/s)，无人机无效时返回零向量
 */
FVector UDroneApi::GetVelocity() const
{
    return OwnerPawn ? OwnerPawn->GetCurrentVelocity() : FVector::ZeroVector;
}

/**
 * @brief 获取当前姿态
 * @return 欧拉角 (Roll, Pitch, Yaw)，无人机无效时返回零旋转
 */
FRotator UDroneApi::GetOrientation() const
{
    return OwnerPawn ? OwnerPawn->CurrentState.GetRotator() : FRotator::ZeroRotator;
}

/**
 * @brief 获取四个电机的转速
 * @return 转速数组 (rad/s)，长度为 4
 *
 * 将 double 转速转换为 float 输出。
 * 如果电机数据无效（不足4个），返回空数组。
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
 * @return 控制模式枚举值，无人机无效时返回 Idle
 */
EDroneControlMode UDroneApi::GetControlMode() const
{
    return OwnerPawn ? OwnerPawn->ControlMode : EDroneControlMode::Idle;
}

// ---- PID 参数调整（委托给 MovementComponent） ----

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
 * @brief 设置角速率控制器增益
 * @param Kp 比例增益
 */
void UDroneApi::SetAngleRateControllerGains(float Kp)
{
    if (OwnerPawn && OwnerPawn->MovementComp)
        OwnerPawn->MovementComp->SetAngleRateGains(Kp);
}

/**
 * @brief 重置无人机到指定位置和姿态
 * @param Position 目标位置 (m)
 * @param Rotation 目标姿态
 */
void UDroneApi::Reset(FVector Position, FRotator Rotation)
{
    if (OwnerPawn) OwnerPawn->ResetDrone(Position, Rotation);
}

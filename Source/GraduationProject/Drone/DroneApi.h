/**
 * @file DroneApi.h
 * @brief 无人机统一 API 接口的头文件
 *
 * 本文件定义了 UDroneApi 类，提供 AirSim 风格的统一无人机控制接口。
 * 作为 TCP CommandHandler 和 DronePawn 之间的中间层：
 *   TCP Handler → DroneApi → DronePawn → DroneMovementComponent
 *
 * 所有坐标使用 SI 单位（米、m/s）。
 * 参考 AirSim MultirotorApiBase 设计。
 */

#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "DroneParameters.h"
#include "DroneApi.generated.h"

class ADronePawn;

/**
 * 无人机 API — AirSim 风格统一接口
 *
 * 提供高层控制接口，内部委托给 DronePawn 执行。
 * 支持位置控制、速度控制、起飞/降落/悬停、状态查询、PID 参数调整等。
 *
 * 调用链路：TCP Handler → DroneApi → DronePawn → MovementComp
 */
UCLASS()
class GRADUATIONPROJECT_API UDroneApi : public UObject
{
    GENERATED_BODY()

public:
    /**
     * @brief 初始化 API，绑定到目标 DronePawn
     * @param Owner 拥有此 API 的 DronePawn 实例
     */
    void Initialize(ADronePawn* Owner);

    // ---- 位置控制 (SI 坐标, 单位: 米) ----

    /**
     * @brief 移动到指定位置
     * @param X 目标 X 坐标 (m)
     * @param Y 目标 Y 坐标 (m)
     * @param Z 目标 Z 坐标 (m)
     * @param Speed 移动速度 (m/s)，默认 2.0
     */
    void MoveToPosition(float X, float Y, float Z, float Speed = 2.0f);

    /** @brief 在当前位置悬停 */
    void Hover();

    /**
     * @brief 起飞到指定高度
     * @param Altitude 目标高度 (m)
     */
    void Takeoff(float Altitude);

    /** @brief 降落到地面（Z=0） */
    void Land();

    // ---- 速度控制 (单位: m/s) ----

    /**
     * @brief 按速度飞行
     * @param Vx X 方向速度 (m/s)
     * @param Vy Y 方向速度 (m/s)
     * @param Vz Z 方向速度 (m/s)
     */
    void MoveByVelocity(float Vx, float Vy, float Vz);

    // ---- 状态查询 ----

    /**
     * @brief 获取当前位置
     * @return 位置向量 (m)
     */
    FVector GetPosition() const;

    /**
     * @brief 获取当前速度
     * @return 速度向量 (m/s)
     */
    FVector GetVelocity() const;

    /**
     * @brief 获取当前姿态
     * @return 欧拉角 (Roll, Pitch, Yaw)
     */
    FRotator GetOrientation() const;

    /**
     * @brief 获取四个电机的转速
     * @return 转速数组 (rad/s)，长度为 4；无人机无效时返回空数组
     */
    TArray<float> GetMotorSpeeds() const;

    /**
     * @brief 获取当前控制模式
     * @return 控制模式枚举值
     */
    EDroneControlMode GetControlMode() const;

    // ---- PID 参数调整 ----

    /**
     * @brief 设置位置控制器（PD）增益
     * @param Kp 比例增益
     * @param Kd 微分增益
     */
    void SetPositionControllerGains(float Kp, float Kd);

    /**
     * @brief 设置速度控制器（PID）增益
     * @param Kp 比例增益
     * @param Ki 积分增益
     * @param Kd 微分增益
     */
    void SetVelocityControllerGains(float Kp, float Ki, float Kd);

    /**
     * @brief 设置姿态控制器（PD）增益
     * @param Kp 比例增益
     * @param Kd 微分增益
     */
    void SetAttitudeControllerGains(float Kp, float Kd);

    /**
     * @brief 设置角速率控制器增益
     * @param Kp 比例增益
     */
    void SetAngleRateControllerGains(float Kp);

    // ---- 重置 ----

    /**
     * @brief 重置无人机到指定位置和姿态
     * @param Position 目标位置 (m)，默认原点
     * @param Rotation 目标姿态，默认水平
     */
    void Reset(FVector Position = FVector::ZeroVector, FRotator Rotation = FRotator::ZeroRotator);

private:
    /** @brief 拥有此 API 的 DronePawn 实例 */
    UPROPERTY()
    ADronePawn* OwnerPawn = nullptr;
};

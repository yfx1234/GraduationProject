#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "DroneParameters.h"
#include "DroneApi.generated.h"

class ADronePawn;

/**
 * @brief 无人机高层控制接口
 * 对外提供位置、速度、姿态和控制器参数调节等便捷调用，
 * 本质上是对 `ADronePawn` 和 `UDroneMovementComponent` 的一层轻量包装。
 */
UCLASS()
class GRADUATIONPROJECT_API UDroneApi : public UObject
{
    GENERATED_BODY()

public:
    /**
     * @brief 初始化 API，并绑定到目标无人机
     * @param Owner 拥有该 API 的 `ADronePawn`
     */
    void Initialize(ADronePawn* Owner);

    /**
     * @brief 以位置模式飞往指定坐标
     * @param X 目标 X 坐标（米）
     * @param Y 目标 Y 坐标（米）
     * @param Z 目标 Z 坐标（米）
     * @param Speed 飞行速度（米/秒）
     */
    void MoveToPosition(float X, float Y, float Z, float Speed = 2.0f);

    /** @brief 在当前位置悬停 */
    void Hover();

    /**
     * @brief 起飞到指定高度
     * @param Altitude 目标高度（米）
     */
    void Takeoff(float Altitude);

    /** @brief 降落到地面 */
    void Land();

    /**
     * @brief 以速度模式飞行
     * @param Vx X 方向速度（米/秒）
     * @param Vy Y 方向速度（米/秒）
     * @param Vz Z 方向速度（米/秒）
     */
    void MoveByVelocity(float Vx, float Vy, float Vz);

    /**
     * @brief 设置目标姿态和总推力
     * @param RollDeg 目标横滚角（度）
     * @param PitchDeg 目标俯仰角（度）
     * @param YawDeg 目标偏航角（度）
     * @param Thrust 目标总推力
     */
    void SetTargetAttitude(float RollDeg, float PitchDeg, float YawDeg, float Thrust);

    /**
     * @brief 直接设置四个电机转速命令
     * @param M0 电机 0 转速（rad/s）
     * @param M1 电机 1 转速（rad/s）
     * @param M2 电机 2 转速（rad/s）
     * @param M3 电机 3 转速（rad/s）
     */
    void SetMotorSpeeds(float M0, float M1, float M2, float M3);

    /**
     * @brief 设置航向控制模式
     * @param YawMode 偏航控制模式
     * @param Drivetrain 运动学约束模式
     * @param YawDeg 目标偏航角；仅在角度模式下生效
     */
    void SetHeadingControl(EDroneYawMode YawMode, EDroneDrivetrainMode Drivetrain, float YawDeg = 0.0f);

    /**
     * @brief 获取当前无人机位置
     * @return 位置向量（米）
     */
    FVector GetPosition() const;

    /**
     * @brief 获取当前无人机速度
     * @return 速度向量（米/秒）
     */
    FVector GetVelocity() const;

    /**
     * @brief 获取当前姿态
     * @return 欧拉角姿态 `(Roll, Pitch, Yaw)`
     */
    FRotator GetOrientation() const;

    /**
     * @brief 获取当前四个电机转速
     * @return 电机转速数组（rad/s）
     */
    TArray<float> GetMotorSpeeds() const;

    /**
     * @brief 获取当前控制模式
     * @return 控制模式枚举值
     */
    EDroneControlMode GetControlMode() const;

    /**
     * @brief 设置位置控制器增益
     * @param Kp 比例增益
     * @param Kd 微分增益
     */
    void SetPositionControllerGains(float Kp, float Kd);

    /**
     * @brief 设置速度控制器增益
     * @param Kp 比例增益
     * @param Ki 积分增益
     * @param Kd 微分增益
     */
    void SetVelocityControllerGains(float Kp, float Ki, float Kd);

    /**
     * @brief 设置姿态控制器增益
     * @param Kp 比例增益
     * @param Kd 微分增益
     */
    void SetAttitudeControllerGains(float Kp, float Kd);

    /**
     * @brief 设置角速度控制器比例增益
     * @param Kp 比例增益
     */
    void SetAngleRateControllerGains(float Kp);

    /**
     * @brief 将无人机重置到指定位置和姿态
     * @param Position 目标位置（米）
     * @param Rotation 目标姿态
     */
    void Reset(FVector Position = FVector::ZeroVector, FRotator Rotation = FRotator::ZeroRotator);

private:
    /** @brief 绑定的无人机 Pawn */
    UPROPERTY()
    ADronePawn* OwnerPawn = nullptr;
};

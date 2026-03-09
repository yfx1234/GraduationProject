#pragma once

#include "CoreMinimal.h"
#include "DroneState.generated.h"

/**
 * @brief 无人机状态结构体
 * 包含：
 * - 位置 (X, Y, Z)
 * - 线速度 (Vx, Vy, Vz)
 * - 姿态四元数 (Qw, Qx, Qy, Qz)
 * - 角速度 (AngRollRate, AngPitchRate, AngYawRate)
 * - 电机转速 [4]
 */
USTRUCT(BlueprintType)
struct GRADUATIONPROJECT_API FDroneState
{
    GENERATED_BODY()

    /** @brief X 坐标 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|Position")
    double X = 0.0;
    
    /** @brief Y 坐标 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|Position")
    double Y = 0.0;
    
    /** @brief Z 坐标 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|Position")
    double Z = 0.0;

    /** @brief X 方向速度 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|Velocity")
    double Vx = 0.0;
    
    /** @brief Y 方向速度 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|Velocity")
    double Vy = 0.0;
    
    /** @brief Z 方向速度 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|Velocity")
    double Vz = 0.0;

    /** @brief 四元数 W 分量 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|Attitude")
    double Qw = 1.0;
    
    /** @brief 四元数 X 分量 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|Attitude")
    double Qx = 0.0;
    
    /** @brief 四元数 Y 分量 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|Attitude")
    double Qy = 0.0;
    
    /** @brief 四元数 Z 分量 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|Attitude")
    double Qz = 0.0;

    /** @brief 滚转角速率 p */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|AngularVelocity")
    double AngRollRate = 0.0;

    /** @brief 俯仰角速率 q */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|AngularVelocity")
    double AngPitchRate = 0.0;

    /** @brief 偏航角速率 r */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|AngularVelocity")
    double AngYawRate = 0.0;

    /** @brief 四个电机的角速度数组 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|Motors")
    TArray<double> MotorSpeeds = {0.0, 0.0, 0.0, 0.0};

    /** @brief 获取位置向量 */
    FVector GetPosition() const { return FVector(X, Y, Z); }

    /**
     * @brief 设置位置
     * @param Pos 位置向量
     */
    void SetPosition(const FVector& Pos) { X = Pos.X; Y = Pos.Y; Z = Pos.Z; }
    
    /** @brief 获取速度向量 */
    FVector GetVelocity() const { return FVector(Vx, Vy, Vz); }

    /**
     * @brief 设置速度
     * @param Vel 速度向量
     */
    void SetVelocity(const FVector& Vel) { Vx = Vel.X; Vy = Vel.Y; Vz = Vel.Z; }
    
    /**
     * @brief 获取姿态四元数
     * @return UE FQuat
     */
    FQuat GetQuaternion() const { return FQuat(Qx, Qy, Qz, Qw); }

    /**
     * @brief 设置姿态四元数
     * @param InQuat UE FQuat
     */
    void SetQuaternion(const FQuat& InQuat) { Qw = InQuat.W; Qx = InQuat.X; Qy = InQuat.Y; Qz = InQuat.Z; }
    
    /**
     * @brief 获取姿态欧拉角
     * @return UE FRotator
     */
    FRotator GetRotator() const { return GetQuaternion().Rotator(); }
    
    /**
     * @brief 获取角速度向量
     * @return FVector(p, q, r)
     */
    FVector GetAngularVelocity() const { return FVector(AngRollRate, AngPitchRate, AngYawRate); }
    
    /** @brief 归一化四元数，防止数值积分累积误差导致四元数偏离单位长度 */
    void NormalizeQuaternion()
    {
        double Mag = FMath::Sqrt(Qw*Qw + Qx*Qx + Qy*Qy + Qz*Qz);
        if (Mag > KINDA_SMALL_NUMBER)
        {
            Qw /= Mag;
            Qx /= Mag;
            Qy /= Mag;
            Qz /= Mag;
        }
        else
        {
            // 四元数退化，重置为单位四元数
            Qw = 1.0; Qx = 0.0; Qy = 0.0; Qz = 0.0;
        }
    }

    /** @brief 检测状态中是否包含 NaN 或 Inf */
    bool HasNaN() const
    {
        return !FMath::IsFinite(X) || !FMath::IsFinite(Y) || !FMath::IsFinite(Z)
            || !FMath::IsFinite(Vx) || !FMath::IsFinite(Vy) || !FMath::IsFinite(Vz)
            || !FMath::IsFinite(Qw) || !FMath::IsFinite(Qx) || !FMath::IsFinite(Qy) || !FMath::IsFinite(Qz)
            || !FMath::IsFinite(AngRollRate) || !FMath::IsFinite(AngPitchRate) || !FMath::IsFinite(AngYawRate);
    }

    /**
     * @brief 裁剪速度和角速度，防止数值发散
     * @param MaxLinearSpeed 最大线速度 m/s
     * @param MaxAngularSpeed 最大角速度 rad/s
     */
    void ClampVelocities(double MaxLinearSpeed = 100.0, double MaxAngularSpeed = 100.0)
    {
        FVector Vel(Vx, Vy, Vz);
        if (Vel.SizeSquared() > MaxLinearSpeed * MaxLinearSpeed)
        {
            Vel = Vel.GetSafeNormal() * MaxLinearSpeed;
            Vx = Vel.X; Vy = Vel.Y; Vz = Vel.Z;
        }
        FVector AngVel(AngRollRate, AngPitchRate, AngYawRate);
        if (AngVel.SizeSquared() > MaxAngularSpeed * MaxAngularSpeed)
        {
            AngVel = AngVel.GetSafeNormal() * MaxAngularSpeed;
            AngRollRate = AngVel.X; AngPitchRate = AngVel.Y; AngYawRate = AngVel.Z;
        }
    }
};

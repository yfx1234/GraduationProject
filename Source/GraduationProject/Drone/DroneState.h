/**
 * @file DroneState.h
 * @brief 无人机状态结构体的头文件
 *
 * 本文件定义了 FDroneState 结构体，封装了四旋翼无人机仿真所需的全部状态变量：
 * 位置、速度、姿态四元数、角速度和电机转速。
 * 所有状态变量使用 SI 国际单位制（米、弧度/秒、rad/s）。
 * 提供了 FVector/FQuat/FRotator 的便捷转换函数。
 */

#pragma once

#include "CoreMinimal.h"
#include "DroneState.generated.h"

/**
 * 无人机状态结构体
 *
 * 封装四旋翼无人机在仿真中的完整状态，包含 13 个状态变量：
 * - 位置 (X, Y, Z) — 世界坐标系，单位：米
 * - 线速度 (Vx, Vy, Vz) — 世界坐标系，单位：m/s
 * - 姿态四元数 (Qw, Qx, Qy, Qz) — 机体到世界的旋转
 * - 角速度 (AngRollRate, AngPitchRate, AngYawRate) — 机体坐标系，单位：rad/s
 * - 电机转速 [4] — 四个电机的角速度，单位：rad/s
 *
 * 此结构体同时用于 RK4 积分器的状态向量和控制器的反馈信号。
 */
USTRUCT(BlueprintType)
struct GRADUATIONPROJECT_API FDroneState
{
    GENERATED_BODY()

    // ---- 位置 (世界坐标系, 单位: 米) ----

    /** @brief X 坐标（前方） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|Position")
    double X = 0.0;
    
    /** @brief Y 坐标（右方） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|Position")
    double Y = 0.0;
    
    /** @brief Z 坐标（上方） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|Position")
    double Z = 0.0;

    // ---- 线速度 (世界坐标系, 单位: m/s) ----

    /** @brief X 方向速度 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|Velocity")
    double Vx = 0.0;
    
    /** @brief Y 方向速度 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|Velocity")
    double Vy = 0.0;
    
    /** @brief Z 方向速度 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|Velocity")
    double Vz = 0.0;

    // ---- 姿态四元数 (机体→世界旋转，Hamilton 约定) ----

    /** @brief 四元数 W 分量（标量部分） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|Attitude")
    double Qw = 1.0;
    
    /** @brief 四元数 X 分量（Roll 轴） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|Attitude")
    double Qx = 0.0;
    
    /** @brief 四元数 Y 分量（Pitch 轴） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|Attitude")
    double Qy = 0.0;
    
    /** @brief 四元数 Z 分量（Yaw 轴） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|Attitude")
    double Qz = 0.0;

    // ---- 角速度 (机体坐标系, 单位: rad/s) ----

    /** @brief 滚转角速率 p（绕机体 X 轴） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|AngularVelocity")
    double AngRollRate = 0.0;

    /** @brief 俯仰角速率 q（绕机体 Y 轴） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|AngularVelocity")
    double AngPitchRate = 0.0;

    /** @brief 偏航角速率 r（绕机体 Z 轴） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|AngularVelocity")
    double AngYawRate = 0.0;

    // ---- 电机转速 (单位: rad/s) ----

    /** @brief 四个电机的角速度数组，顺序 [电机0, 电机1, 电机2, 电机3] */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|Motors")
    TArray<double> MotorSpeeds = {0.0, 0.0, 0.0, 0.0};

    // ---- 辅助转换函数 ----

    /** @brief 获取位置向量 (m) */
    FVector GetPosition() const { return FVector(X, Y, Z); }

    /**
     * @brief 设置位置
     * @param Pos 位置向量 (m)
     */
    void SetPosition(const FVector& Pos) { X = Pos.X; Y = Pos.Y; Z = Pos.Z; }
    
    /** @brief 获取速度向量 (m/s) */
    FVector GetVelocity() const { return FVector(Vx, Vy, Vz); }

    /**
     * @brief 设置速度
     * @param Vel 速度向量 (m/s)
     */
    void SetVelocity(const FVector& Vel) { Vx = Vel.X; Vy = Vel.Y; Vz = Vel.Z; }
    
    /**
     * @brief 获取姿态四元数
     * @return UE FQuat（注意 UE 的构造顺序为 X,Y,Z,W）
     */
    FQuat GetQuaternion() const { return FQuat(Qx, Qy, Qz, Qw); }

    /**
     * @brief 设置姿态四元数
     * @param InQuat UE FQuat
     */
    void SetQuaternion(const FQuat& InQuat) { Qw = InQuat.W; Qx = InQuat.X; Qy = InQuat.Y; Qz = InQuat.Z; }
    
    /**
     * @brief 获取姿态欧拉角（通过四元数→旋转器转换）
     * @return UE FRotator (Pitch, Yaw, Roll)
     */
    FRotator GetRotator() const { return GetQuaternion().Rotator(); }
    
    /**
     * @brief 获取角速度向量 (机体坐标系, rad/s)
     * @return FVector(p, q, r)
     */
    FVector GetAngularVelocity() const { return FVector(AngRollRate, AngPitchRate, AngYawRate); }
    
    /**
     * @brief 归一化四元数，防止数值积分累积误差导致四元数偏离单位长度
     *
     * 在 RK4 积分后调用，确保 |Q| = 1。
     */
    void NormalizeQuaternion()
    {
        double Mag = FMath::Sqrt(Qw*Qw + Qx*Qx + Qy*Qy + Qz*Qz);
        if (Mag > KINDA_SMALL_NUMBER)
        {
            Qw /= Mag; Qx /= Mag; Qy /= Mag; Qz /= Mag;
        }
    }
};

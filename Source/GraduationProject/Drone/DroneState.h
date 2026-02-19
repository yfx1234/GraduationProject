#pragma once

#include "CoreMinimal.h"
#include "DroneState.generated.h"

/**
 * 无人机状态结构体
 * 包含位置、速度、姿态、角速度等状态
 */
USTRUCT(BlueprintType)
struct GRADUATIONPROJECT_API FDroneState
{
    GENERATED_BODY()

    // ---- 位置 (世界坐标系, m) ----
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|Position")
    double X = 0.0;
    
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|Position")
    double Y = 0.0;
    
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|Position")
    double Z = 0.0;

    // ---- 线速度 (世界坐标系, m/s) ----
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|Velocity")
    double Vx = 0.0;
    
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|Velocity")
    double Vy = 0.0;
    
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|Velocity")
    double Vz = 0.0;

    // ---- 姿态四元数 ----
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|Attitude")
    double Qw = 1.0;
    
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|Attitude")
    double Qx = 0.0;
    
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|Attitude")
    double Qy = 0.0;
    
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|Attitude")
    double Qz = 0.0;

    // ---- 角速度 (机体坐标系, rad/s) ----
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|AngularVelocity")
    double AngRollRate = 0.0;  
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|AngularVelocity")
    double AngPitchRate = 0.0;  
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|AngularVelocity")
    double AngYawRate = 0.0;  

    // ---- 电机转速 (rad/s) ----
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|Motors")
    TArray<double> MotorSpeeds = {0.0, 0.0, 0.0, 0.0};

    // ---- 辅助函数 ----
    FVector GetPosition() const { return FVector(X, Y, Z); }
    void SetPosition(const FVector& Pos) { X = Pos.X; Y = Pos.Y; Z = Pos.Z; }
    
    FVector GetVelocity() const { return FVector(Vx, Vy, Vz); }
    void SetVelocity(const FVector& Vel) { Vx = Vel.X; Vy = Vel.Y; Vz = Vel.Z; }
    
    FQuat GetQuaternion() const { return FQuat(Qx, Qy, Qz, Qw); }
    void SetQuaternion(const FQuat& InQuat) { Qw = InQuat.W; Qx = InQuat.X; Qy = InQuat.Y; Qz = InQuat.Z; }
    
    FRotator GetRotator() const { return GetQuaternion().Rotator(); }
    
    FVector GetAngularVelocity() const { return FVector(AngRollRate, AngPitchRate, AngYawRate); }
    
    void NormalizeQuaternion()
    {
        double Mag = FMath::Sqrt(Qw*Qw + Qx*Qx + Qy*Qy + Qz*Qz);
        if (Mag > KINDA_SMALL_NUMBER)
        {
            Qw /= Mag; Qx /= Mag; Qy /= Mag; Qz /= Mag;
        }
    }
};

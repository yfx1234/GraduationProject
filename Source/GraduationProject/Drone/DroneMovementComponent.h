#pragma once
#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "DroneState.h"
#include "DroneParameters.h"
#include "DroneMovementComponent.generated.h"

class UPDController;
class UPIDController;

/**
 * 无人机运动仿真组件
 * 实现四旋翼动力学模型（RK4积分）和级联PID控制器
 * 直接从旧项目复制，仅调整 include 路径
 */
UCLASS(ClassGroup=(CustomDrone), meta=(BlueprintSpawnableComponent))
class GRADUATIONPROJECT_API UDroneMovementComponent : public UActorComponent
{
    GENERATED_BODY()

public:
    UDroneMovementComponent();

    virtual void BeginPlay() override;
    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

    UFUNCTION(BlueprintCallable, Category = "DroneMovement|Setup")
    void SetInitialState(const FDroneState& InitialState);

    UFUNCTION(BlueprintCallable, Category = "DroneMovement|Setup")
    void SetParameters(const FDroneParameters& NewParameters);

    UFUNCTION(BlueprintCallable, Category = "DroneMovement|Setup")
    void SetControlMode(EDroneControlMode NewMode);

    UFUNCTION(BlueprintCallable, Category = "DroneMovement|Control")
    void SetControlCommand(const TArray<double>& Command);

    UFUNCTION(BlueprintCallable, Category = "DroneMovement|Control")
    void SetTargetPosition(const FVector& TargetPos);

    UFUNCTION(BlueprintCallable, Category = "DroneMovement|Control")
    void SetTargetVelocity(const FVector& TargetVel);

    UFUNCTION(BlueprintCallable, Category = "DroneMovement|Control")
    void SetTargetAttitude(const FRotator& Attitude, float Thrust);

    UFUNCTION(BlueprintPure, Category = "DroneMovement|State")
    FDroneState GetCurrentState() const { return CurrentState; }

    UFUNCTION(BlueprintCallable, Category = "DroneMovement|State")
    void ResetState(const FDroneState& NewState);

    // ---- PID 增益运行时调参（新增，供 DroneApi 使用）----
    void SetPositionGains(float Kp, float Kd);
    void SetVelocityGains(float Kp, float Ki, float Kd);
    void SetAttitudeGains(float Kp, float Kd);
    void SetAngleRateGains(float Kp);

public:
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DroneMovement|Parameters")
    FDroneParameters Parameters;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DroneMovement|Control")
    EDroneControlMode CurrentControlMode;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "DroneMovement|State")
    FDroneState CurrentState;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "DroneMovement|State")
    bool bInitialized = false;

protected:
    void ControlUpdate(double DeltaTime);
    
    // 级联控制环路
    FVector PositionLoop(const FVector& PositionCommand);
    FVector VelocityLoop(const FVector& VelocityCommand, double& OutThrust);
    FVector AttitudeLoop(const FVector& AccelerationCommand);
    FVector AngularVelocityLoop(const FVector& AngularVelocityCommand);
    TArray<double> CalculateMotorSpeeds(const FVector& TorqueCommand, double Thrust);

    // 动力学仿真
    FDroneState RK4Update(const FDroneState& State, double TimeStep);
    void CalculateTotalForcesAndTorques(const FDroneState& State, TArray<double>& OutTotalForce, TArray<double>& OutTorque);
    TArray<double> Derivatives(const FDroneState& State, const TArray<double>& TotalForce, const TArray<double>& Torque);
    FDroneState StateAdd(const FDroneState& State, const TArray<double>& Derivative, double dt);

    // 辅助函数
    FVector RotateBodyToWorld(const FVector& BodyVector, const FQuat& Orientation);
    void InitializeControllers();
    void ResetAllControllers();
    static double NormalizeAngle(double AngleRad);
    void ComputeControlAllocation();

private:
    TArray<double> ControlCommands;
    FVector TargetPosition;
    FVector TargetVelocity;
    FRotator TargetAttitude;
    double TargetThrust;

    // 位置控制器 (PD)
    UPROPERTY() UPDController* PxController;
    UPROPERTY() UPDController* PyController;
    UPROPERTY() UPDController* PzController;

    // 速度控制器 (PID)
    UPROPERTY() UPIDController* VxController;
    UPROPERTY() UPIDController* VyController;
    UPROPERTY() UPIDController* VzController;

    // 姿态控制器 (PD)
    UPROPERTY() UPDController* RollController;
    UPROPERTY() UPDController* PitchController;
    UPROPERTY() UPDController* YawController;

    // 角速率控制器 (PID)
    UPROPERTY() UPIDController* RollRateController;
    UPROPERTY() UPIDController* PitchRateController;
    UPROPERTY() UPIDController* YawRateController;

    // 控制分配矩阵
    double G[4][4];
    double GInv[4][4];
};

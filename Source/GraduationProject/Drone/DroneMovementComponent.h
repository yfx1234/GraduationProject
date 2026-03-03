#pragma once
#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "DroneState.h"
#include "DroneParameters.h"
#include "DroneMovementComponent.generated.h"

class UPDController;
class UPIDController;

UCLASS(ClassGroup=(CustomDrone), meta=(BlueprintSpawnableComponent))
class GRADUATIONPROJECT_API UDroneMovementComponent : public UActorComponent
{
    GENERATED_BODY()

public:
    /** @brief 构造函数，初始化默认控制模式和命令 */
    UDroneMovementComponent();

    /**
     * @brief 组件初始化
     * 创建所有 PD/PID 控制器实例，计算控制分配矩阵。
     */
    virtual void BeginPlay() override;

    /**
     * @brief 每帧 Tick 执行控制更新和物理仿真
     * @param DeltaTime 帧间隔时间（秒）
     * @param TickType Tick 类型
     * @param ThisTickFunction Tick 函数引用
     */
    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

    /**
     * @brief 设置初始状态
     * @param InitialState 初始无人机状态
     */
    UFUNCTION(BlueprintCallable, Category = "DroneMovement|Setup")
    void SetInitialState(const FDroneState& InitialState);

    /**
     * @brief 设置物理参数并重新计算控制分配矩阵
     * @param NewParameters 新的物理参数
     */
    UFUNCTION(BlueprintCallable, Category = "DroneMovement|Setup")
    void SetParameters(const FDroneParameters& NewParameters);

    /**
     * @brief 切换控制模式，同时重置所有控制器状态
     * @param NewMode 新的控制模式
     */
    UFUNCTION(BlueprintCallable, Category = "DroneMovement|Setup")
    void SetControlMode(EDroneControlMode NewMode);

    /**
     * @brief 设置原始控制命令
     * @param Command 命令数组
     */
    UFUNCTION(BlueprintCallable, Category = "DroneMovement|Control")
    void SetControlCommand(const TArray<double>& Command);

    /**
     * @brief 设置目标位置
     * @param TargetPos 目标位置
     */
    UFUNCTION(BlueprintCallable, Category = "DroneMovement|Control")
    void SetTargetPosition(const FVector& TargetPos);

    /**
     * @brief 设置目标速度
     * @param TargetVel 目标速度
     */
    UFUNCTION(BlueprintCallable, Category = "DroneMovement|Control")
    void SetTargetVelocity(const FVector& TargetVel);

    /**
     * @brief 设置目标姿态和推力
     * @param Attitude 目标姿态
     * @param Thrust 推力系数
     */
    UFUNCTION(BlueprintCallable, Category = "DroneMovement|Control")
    void SetTargetAttitude(const FRotator& Attitude, float Thrust);

    /**
     * @brief 获取当前无人机状态
     * @return 当前状态的拷贝
     */
    UFUNCTION(BlueprintPure, Category = "DroneMovement|State")
    FDroneState GetCurrentState() const { return CurrentState; }

    /**
     * @brief 重置状态并清零所有控制器
     * @param NewState 新的初始状态
     */
    UFUNCTION(BlueprintCallable, Category = "DroneMovement|State")
    void ResetState(const FDroneState& NewState);

    /**
     * @brief 设置位置控制器(PD)增益
     * @param Kp 比例增益
     * @param Kd 微分增益
     */
    void SetPositionGains(float Kp, float Kd);

    /**
     * @brief 设置速度控制器(PID)增益
     * @param Kp 比例增益
     * @param Ki 积分增益
     * @param Kd 微分增益
     */
    void SetVelocityGains(float Kp, float Ki, float Kd);

    /**
     * @brief 设置姿态控制器(PD)增益
     * @param Kp 比例增益
     * @param Kd 微分增益
     */
    void SetAttitudeGains(float Kp, float Kd);

    /**
     * @brief 设置角速率控制器增益
     * @param Kp 比例增益
     */
    void SetAngleRateGains(float Kp);

public:
    /** @brief 无人机物理参数 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DroneMovement|Parameters")
    FDroneParameters Parameters;

    /** @brief 当前控制模式 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DroneMovement|Control")
    EDroneControlMode CurrentControlMode;

    /** @brief 当前无人机状态 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "DroneMovement|State")
    FDroneState CurrentState;

    /** @brief 组件是否已完成初始化 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "DroneMovement|State")
    bool bInitialized = false;

protected:
    /**
     * @brief 执行一帧的控制更新
     * @param DeltaTime 帧间隔
     */
    void ControlUpdate(double DeltaTime);

    /**
     * @brief 位置控制环
     * @param PositionCommand 目标位置
     * @return 速度命令
     */
    FVector PositionLoop(const FVector& PositionCommand);

    /**
     * @brief 速度控制环
     * @param VelocityCommand 目标速度
     * @param OutThrust 输出：计算得到的总推力
     * @return 加速度命令
     */
    FVector VelocityLoop(const FVector& VelocityCommand, double& OutThrust);

    /**
     * @brief 姿态控制环
     * @param AccelerationCommand 期望加速度
     * @return 角速率命令
     */
    FVector AttitudeLoop(const FVector& AccelerationCommand);

    /**
     * @brief 角速率控制环
     * @param AngularVelocityCommand 目标角速率
     * @return 力矩命令
     */
    FVector AngularVelocityLoop(const FVector& AngularVelocityCommand);

    /**
     * @brief 从力矩和推力命令计算电机转速
     * @param TorqueCommand 三轴力矩命令
     * @param Thrust 总推力命令
     * @return 四个电机的转速
     */
    TArray<double> CalculateMotorSpeeds(const FVector& TorqueCommand, double Thrust);

    /**
     * @brief RK4 四阶 Runge-Kutta 积分一步
     * @param State 当前状态
     * @param TimeStep 积分步长
     * @return 积分后的新状态
     */
    FDroneState RK4Update(const FDroneState& State, double TimeStep);

    /**
     * @brief 计算给定状态下的合外力和合力矩
     * @param State 当前状态
     * @param OutTotalForce 输出：世界坐标系下的合外力
     * @param OutTorque 输出：机体坐标系下的合力矩
     */
    void CalculateTotalForcesAndTorques(const FDroneState& State, TArray<double>& OutTotalForce, TArray<double>& OutTorque);

    /**
     * @brief 计算状态导数向量（13维）
     * @param State 当前状态
     * @param TotalForce 世界坐标系合外力
     * @param Torque 机体坐标系合力矩
     * @return 13 维导数向量 [ẋ, ẏ, ż, v̇x, v̇y, v̇z, q̇w, q̇x, q̇y, q̇z, ṗ, q̇, ṙ]
     */
    TArray<double> Derivatives(const FDroneState& State, const TArray<double>& TotalForce, const TArray<double>& Torque);

    /**
     * @brief 状态向量加法：State + Derivative * @brief dt
     * @param State 基准状态
     * @param Derivative 导数向量（13维）
     * @param dt 时间步长
     * @return 更新后的状态
     */
    FDroneState StateAdd(const FDroneState& State, const TArray<double>& Derivative, double dt);

    /**
     * @brief 将机体坐标系向量旋转到世界坐标系
     * @param BodyVector 机体坐标系向量
     * @param Orientation 姿态四元数
     * @return 世界坐标系向量
     */
    FVector RotateBodyToWorld(const FVector& BodyVector, const FQuat& Orientation);

    /** @brief 创建并初始化所有 PD/PID 控制器实例 */
    void InitializeControllers();

    /** @brief 重置所有控制器的内部状态 */
    void ResetAllControllers();

    /**
     * @brief 角度归一化到 [-π, π] 范围
     * @param AngleRad 输入角度（弧度）
     * @return 归一化后的角度
     */
    static double NormalizeAngle(double AngleRad);

    /** @brief 计算控制分配矩阵 G 及其逆矩阵 GInv */
    void ComputeControlAllocation();

private:
    /** @brief 原始控制命令 */
    TArray<double> ControlCommands;

    /** @brief 目标位置 */
    FVector TargetPosition;

    /** @brief 目标速度 */
    FVector TargetVelocity;

    /** @brief 目标姿态 */
    FRotator TargetAttitude;

    /** @brief 目标推力 */
    double TargetThrust;

    // 位置控制器 PD
    UPROPERTY() UPDController* PxController;    // X 轴位置控制器
    UPROPERTY() UPDController* PyController;    // Y 轴位置控制器
    UPROPERTY() UPDController* PzController;    // Z 轴位置控制器

    // 速度控制器 PID
    UPROPERTY() UPIDController* VxController;   // X 轴速度控制器
    UPROPERTY() UPIDController* VyController;   // Y 轴速度控制器
    UPROPERTY() UPIDController* VzController;   // Z 轴速度控制器

    // 姿态控制器 PD
    UPROPERTY() UPDController* RollController;  // 滚转角控制器
    UPROPERTY() UPDController* PitchController; // 俯仰角控制器
    UPROPERTY() UPDController* YawController;   // 偏航角控制器

    // 角速率控制器 PID
    UPROPERTY() UPIDController* RollRateController;   // 滚转角速率控制器
    UPROPERTY() UPIDController* PitchRateController;  // 俯仰角速率控制器
    UPROPERTY() UPIDController* YawRateController;    // 偏航角速率控制器

    /**
     * @brief 控制分配矩阵 G (4×4)
     * G[0][i] = kT (推力行)
     * G[1][i] = 滚转力矩贡献
     * G[2][i] = 俯仰力矩贡献
     * G[3][i] = 偏航力矩贡献
     */
    double G[4][4];

    /** @brief 控制分配逆矩阵 GInv (4×4) */
    double GInv[4][4];
};

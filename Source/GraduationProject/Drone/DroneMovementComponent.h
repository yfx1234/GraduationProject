#pragma once
#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "DroneState.h"
#include "DroneParameters.h"
#include "DroneMovementComponent.generated.h"

class UPDController;
class UPIDController;

/**
 * @brief 无人机飞行动力学与级联控制组件
 * 负责根据控制模式执行位置环、速度环、姿态环和角速度环，
 * 并用固定步长 Velocity Verlet 积分更新无人机状态。
 */
UCLASS(ClassGroup=(CustomDrone), meta=(BlueprintSpawnableComponent))
class GRADUATIONPROJECT_API UDroneMovementComponent : public UActorComponent
{
    GENERATED_BODY()

public:
    /** @brief 构造飞行动力学组件 */
    UDroneMovementComponent();

    /**
     * @brief 初始化控制器和控制分配矩阵
     * 创建各级 PD/PID 控制器，并根据物理参数预计算四旋翼控制分配矩阵。
     */
    virtual void BeginPlay() override;

    /**
     * @brief 每帧推进控制与动力学仿真
     * @param DeltaTime 当前帧间隔（秒）
     * @param TickType Tick 类型
     * @param ThisTickFunction 当前 Tick 函数
     */
    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

    /**
     * @brief 设置无人机初始状态
     * @param InitialState 初始状态
     */
    UFUNCTION(BlueprintCallable, Category = "DroneMovement|Setup")
    void SetInitialState(const FDroneState& InitialState);

    /**
     * @brief 更新物理参数并重新初始化内部控制器
     * @param NewParameters 新的物理参数
     */
    UFUNCTION(BlueprintCallable, Category = "DroneMovement|Setup")
    void SetParameters(const FDroneParameters& NewParameters);

    /**
     * @brief 切换控制模式
     * @param NewMode 新的控制模式
     */
    UFUNCTION(BlueprintCallable, Category = "DroneMovement|Setup")
    void SetControlMode(EDroneControlMode NewMode);

    /**
     * @brief 直接写入底层控制命令
     * @param Command 控制命令数组
     */
    UFUNCTION(BlueprintCallable, Category = "DroneMovement|Control")
    void SetControlCommand(const TArray<double>& Command);

    /**
     * @brief 设置位置控制目标
     * @param TargetPos 目标位置
     * @param Speed 期望飞行速度上限；小于等于 0 时恢复默认值
     */
    UFUNCTION(BlueprintCallable, Category = "DroneMovement|Control")
    void SetTargetPosition(const FVector& TargetPos, float Speed = 0.0f);

    /**
     * @brief 设置速度控制目标
     * @param TargetVel 目标速度
     */
    UFUNCTION(BlueprintCallable, Category = "DroneMovement|Control")
    void SetTargetVelocity(const FVector& TargetVel);

    /**
     * @brief 设置姿态和推力目标
     * @param Attitude 目标姿态
     * @param Thrust 推力系数
     */
    UFUNCTION(BlueprintCallable, Category = "DroneMovement|Control")
    void SetTargetAttitude(const FRotator& Attitude, float Thrust);

    /**
     * @brief 获取当前仿真状态
     * @return 当前无人机状态
     */
    UFUNCTION(BlueprintPure, Category = "DroneMovement|State")
    FDroneState GetCurrentState() const { return CurrentState; }

    /**
     * @brief 直接重置当前状态
     * @param NewState 新状态
     */
    UFUNCTION(BlueprintCallable, Category = "DroneMovement|State")
    void ResetState(const FDroneState& NewState);

    /**
     * @brief 设置位置环 PD 增益
     * @param Kp 比例增益
     * @param Kd 微分增益
     */
    void SetPositionGains(float Kp, float Kd);

    /**
     * @brief 设置速度环 PID 增益
     * @param Kp 比例增益
     * @param Ki 积分增益
     * @param Kd 微分增益
     */
    void SetVelocityGains(float Kp, float Ki, float Kd);

    /**
     * @brief 设置姿态环 PD 增益
     * @param Kp 比例增益
     * @param Kd 微分增益
     */
    void SetAttitudeGains(float Kp, float Kd);

    /**
     * @brief 设置角速度环比例增益
     * @param Kp 比例增益
     */
    void SetAngleRateGains(float Kp);

    /**
     * @brief 设置航向控制模式
     * @param NewYawMode 偏航模式
     * @param NewDrivetrain 驱动模式
     * @param YawDeg 目标偏航角；仅在角度模式下使用
     */
    void SetHeadingControl(EDroneYawMode NewYawMode, EDroneDrivetrainMode NewDrivetrain, float YawDeg = 0.0f);

    /** @brief 获取当前偏航模式 */
    EDroneYawMode GetYawMode() const { return YawMode; }

    /** @brief 获取当前驱动模式 */
    EDroneDrivetrainMode GetDrivetrainMode() const { return DrivetrainMode; }

public:
    /** @brief 当前使用的物理参数 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DroneMovement|Parameters")
    FDroneParameters Parameters;

    /** @brief 当前控制模式 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DroneMovement|Control")
    EDroneControlMode CurrentControlMode;

    /** @brief 当前仿真状态 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "DroneMovement|State")
    FDroneState CurrentState;

    /** @brief 控制器和控制分配是否已初始化 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "DroneMovement|State")
    bool bInitialized = false;

    /**
     * @brief 自动偏航触发阈值（m/s）
     * 当平面速度或指向目标的位置变化过小时，保持当前航向而不是重新对准。
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DroneMovement|Parameters")
    float YawSpeedThreshold = 0.3f;

    /**
     * @brief 单帧允许执行的最大固定子步数
     * 用于限制低帧率时的子步积压。
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DroneMovement|Parameters", meta=(ClampMin="1", ClampMax="64"))
    int32 MaxSubStepsPerTick = 16;

    /**
     * @brief 自动偏航附加角度偏移（度）
     * 0 表示机头对准 Actor +X 方向；如果模型前向与控制约定不一致，可在此修正。
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DroneMovement|Parameters")
    float YawOffset = 0.0f;

protected:
    /**
     * @brief 执行一帧级联控制更新
     * @param DeltaTime 控制步长
     */
    void ControlUpdate(double DeltaTime);

    /**
     * @brief 位置环
     * @param PositionCommand 目标位置
     * @return 速度命令
     */
    FVector PositionLoop(const FVector& PositionCommand);

    /**
     * @brief 速度环
     * @param VelocityCommand 目标速度
     * @param OutThrust 输出总推力
     * @return 加速度命令
     */
    FVector VelocityLoop(const FVector& VelocityCommand, double& OutThrust);

    /**
     * @brief 姿态环
     * @param AccelerationCommand 期望加速度
     * @return 角速度命令
     */
    FVector AttitudeLoop(const FVector& AccelerationCommand);

    /**
     * @brief 角速度环
     * @param AngularVelocityCommand 目标角速度
     * @return 力矩命令
     */
    FVector AngularVelocityLoop(const FVector& AngularVelocityCommand);

    /**
     * @brief 将总推力和力矩分配为四个电机转速
     * @param TorqueCommand 力矩命令
     * @param Thrust 总推力
     * @return 四个电机转速
     */
    TArray<double> CalculateMotorSpeeds(const FVector& TorqueCommand, double Thrust);

    /**
     * @brief 使用 Velocity Verlet 更新动力学状态
     * @param DeltaTime 积分步长
     */
    void VerletUpdate(double DeltaTime);

    /**
     * @brief 计算当前状态下的合力和合力矩
     * @param State 当前状态
     * @param OutForce 输出世界坐标系合力
     * @param OutTorque 输出机体系合力矩
     */
    void CalculateTotalForcesAndTorques(const FDroneState& State, FVector& OutForce, FVector& OutTorque);

    /**
     * @brief 检查并处理地面碰撞
     * @param GroundZ 地面高度
     */
    void CheckGroundCollision(double GroundZ = 0.0);

    /**
     * @brief 将机体系向量旋转到世界坐标系
     * @param BodyVector 机体系向量
     * @param Orientation 机体姿态四元数
     * @return 世界系向量
     */
    FVector RotateBodyToWorld(const FVector& BodyVector, const FQuat& Orientation);

    /** @brief 创建并初始化各级控制器 */
    void InitializeControllers();

    /** @brief 重置所有控制器内部状态 */
    void ResetAllControllers();
    void SyncControllerTimeSteps(double ControlTimeStep);
    void UpdateDesiredYawFromPlanarVector(double DirX, double DirY, double CurrentYaw);

    /**
     * @brief 将角度归一化到 [-PI, PI]
     * @param AngleRad 输入角度（弧度）
     * @return 归一化后的角度
     */
    static double NormalizeAngle(double AngleRad);

    /** @brief 根据当前参数计算控制分配矩阵及其逆矩阵 */
    void ComputeControlAllocation();

private:
    /** @brief 直接控制模式使用的原始命令缓存 */
    TArray<double> ControlCommands;

    /** @brief 当前是否处于地面锁定状态 */
    bool bGrounded = false;

    /** @brief 上一积分步的线加速度 */
    FVector PrevLinearAcceleration = FVector::ZeroVector;

    /** @brief 上一积分步的角加速度 */
    FVector PrevAngularAcceleration = FVector::ZeroVector;

    /** @brief 初始地面高度 */
    double InitialGroundZ = 0.0;

    /** @brief 电机一阶滤波后的转速结果 */
    TArray<double> MotorSpeedsFiltered = {0.0, 0.0, 0.0, 0.0};

    /** @brief 最近一次控制更新使用的步长 */
    double LastControlDeltaTime = 0.003;

    /** @brief 固定步长积分的累计时间 */
    double FixedStepAccumulator = 0.0;

    /** @brief 保持或自动偏航模式下锁定的目标航向 */
    double LockedYaw = 0.0;

    /** @brief 偏航参考是否已初始化 */
    bool bYawInitialized = false;

    /** @brief 本帧期望偏航角，由 ControlUpdate 计算 */
    double DesiredYaw = 0.0;

    /** @brief 位置模式目标位置 */
    FVector TargetPosition;

    /** @brief 速度模式目标速度 */
    FVector TargetVelocity;

    /** @brief 姿态模式目标姿态 */
    FRotator TargetAttitude;

    /** @brief 姿态模式目标总推力 */
    double TargetThrust;

    /** @brief 位置环默认速度限制 */
    double DefaultPositionSpeedLimit = 2.0;

    /** @brief 位置环各轴速度限制 */
    double PositionAxisSpeedLimit = 2.0;

    /** @brief 当前目标位置使用的速度限制 */
    double TargetPositionSpeedLimit = 2.0;

    /** @brief 当前偏航模式 */
    EDroneYawMode YawMode = EDroneYawMode::Auto;

    /** @brief 当前驱动模式 */
    EDroneDrivetrainMode DrivetrainMode = EDroneDrivetrainMode::ForwardOnly;

    /** @brief 外部指定的偏航角命令（弧度） */
    double CommandedYaw = 0.0;

    // 位置环 PD
    UPROPERTY() UPDController* PxController;
    UPROPERTY() UPDController* PyController;
    UPROPERTY() UPDController* PzController;

    // 速度环 PID
    UPROPERTY() UPIDController* VxController;
    UPROPERTY() UPIDController* VyController;
    UPROPERTY() UPIDController* VzController;

    // 姿态环 PD
    UPROPERTY() UPDController* RollController;
    UPROPERTY() UPDController* PitchController;
    UPROPERTY() UPDController* YawController;

    // 角速度环 PID
    UPROPERTY() UPIDController* RollRateController;
    UPROPERTY() UPIDController* PitchRateController;
    UPROPERTY() UPIDController* YawRateController;

    /**
     * @brief 控制分配矩阵 G（4x4）
     * 第 0 行对应总推力，第 1~3 行分别对应滚转、俯仰和偏航力矩。
     */
    double G[4][4];

    /** @brief 控制分配矩阵的逆矩阵 GInv（4x4） */
    double GInv[4][4];
};
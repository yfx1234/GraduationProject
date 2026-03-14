// 解释：使用 `#pragma once` 防止该头文件在编译过程中被重复包含。
#pragma once
// 解释：引入 Unreal 的核心基础头文件，提供常用容器、数学类型和日志宏。
#include "CoreMinimal.h"
// 解释：引入 `ActorComponent.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "Components/ActorComponent.h"
// 解释：引入 `DroneState.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "DroneState.h"
// 解释：引入 `DroneParameters.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "DroneParameters.h"
// 解释：引入 `DroneMovementComponent.generated.h`，让 Unreal Header Tool 生成的反射代码在本文件中可见。
#include "DroneMovementComponent.generated.h"

// 解释：这一行声明 类 `UPDController`，用于封装updcontroller相关的数据与行为。
class UPDController;
// 解释：这一行声明 类 `UPIDController`，用于封装upidcontroller相关的数据与行为。
class UPIDController;

/**
 * @brief 无人机飞行动力学与级联控制组件
 * 负责根据控制模式执行位置环、速度环、姿态环和角速度环，
 * 并用固定步长 Velocity Verlet 积分更新无人机状态。
 */
// 解释：使用 `UCLASS` 宏把当前类型注册到 Unreal 反射系统中，便于蓝图、序列化和编辑器识别。
UCLASS(ClassGroup=(CustomDrone), meta=(BlueprintSpawnableComponent))
// 解释：这一行声明 类 `UDroneMovementComponent`，用于封装udrone运动组件相关的数据与行为。
class GRADUATIONPROJECT_API UDroneMovementComponent : public UActorComponent
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：展开 Unreal Header Tool 生成的样板代码，这是 UObject、Actor、组件等类型正常工作的基础。
    GENERATED_BODY()

// 解释：从这一行开始进入 `public` 访问区，下面的成员会对外部模块公开。
public:
    /** @brief 构造飞行动力学组件 */
    // 解释：调用 `UDroneMovementComponent` 执行当前步骤需要的功能逻辑。
    UDroneMovementComponent();

    /**
     * @brief 初始化控制器和控制分配矩阵
     * 创建各级 PD/PID 控制器，并根据物理参数预计算四旋翼控制分配矩阵。
     */
    // 解释：调用 `BeginPlay` 执行当前步骤需要的功能逻辑。
    virtual void BeginPlay() override;

    /**
     * @brief 每帧推进控制与动力学仿真
     * @param DeltaTime 当前帧间隔（秒）
     * @param TickType Tick 类型
     * @param ThisTickFunction 当前 Tick 函数
     */
    // 解释：调用 `TickComponent` 执行当前步骤需要的功能逻辑。
    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

    /**
     * @brief 设置无人机初始状态
     * @param InitialState 初始状态
     */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "DroneMovement|Setup")
    // 解释：调用 `SetInitialState` 执行当前步骤需要的功能逻辑。
    void SetInitialState(const FDroneState& InitialState);

    /**
     * @brief 更新物理参数并重新初始化内部控制器
     * @param NewParameters 新的物理参数
     */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "DroneMovement|Setup")
    // 解释：调用 `SetParameters` 执行当前步骤需要的功能逻辑。
    void SetParameters(const FDroneParameters& NewParameters);

    /**
     * @brief 切换控制模式
     * @param NewMode 新的控制模式
     */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "DroneMovement|Setup")
    // 解释：调用 `SetControlMode` 执行当前步骤需要的功能逻辑。
    void SetControlMode(EDroneControlMode NewMode);

    /**
     * @brief 直接写入底层控制命令
     * @param Command 控制命令数组
     */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "DroneMovement|Control")
    // 解释：调用 `SetControlCommand` 执行当前步骤需要的功能逻辑。
    void SetControlCommand(const TArray<double>& Command);

    /**
     * @brief 设置位置控制目标
     * @param TargetPos 目标位置
     * @param Speed 期望飞行速度上限；小于等于 0 时恢复默认值
     */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "DroneMovement|Control")
    // 解释：这一行把右侧表达式的结果写入 `void SetTargetPosition(const FVector& TargetPos, float Speed`，完成 voidsettargetpositionconstfvectortargetposfloatspeed 的更新。
    void SetTargetPosition(const FVector& TargetPos, float Speed = 0.0f);

    /**
     * @brief 设置速度控制目标
     * @param TargetVel 目标速度
     */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "DroneMovement|Control")
    // 解释：调用 `SetTargetVelocity` 执行当前步骤需要的功能逻辑。
    void SetTargetVelocity(const FVector& TargetVel);

    /**
     * @brief 设置姿态和推力目标
     * @param Attitude 目标姿态
     * @param Thrust 推力系数
     */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "DroneMovement|Control")
    // 解释：调用 `SetTargetAttitude` 执行当前步骤需要的功能逻辑。
    void SetTargetAttitude(const FRotator& Attitude, float Thrust);

    /**
     * @brief 获取当前仿真状态
     * @return 当前无人机状态
     */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintPure, Category = "DroneMovement|State")
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    FDroneState GetCurrentState() const { return CurrentState; }

    /**
     * @brief 直接重置当前状态
     * @param NewState 新状态
     */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "DroneMovement|State")
    // 解释：调用 `ResetState` 执行当前步骤需要的功能逻辑。
    void ResetState(const FDroneState& NewState);

    /**
     * @brief 设置位置环 PD 增益
     * @param Kp 比例增益
     * @param Kd 微分增益
     */
    // 解释：调用 `SetPositionGains` 执行当前步骤需要的功能逻辑。
    void SetPositionGains(float Kp, float Kd);

    /**
     * @brief 设置速度环 PID 增益
     * @param Kp 比例增益
     * @param Ki 积分增益
     * @param Kd 微分增益
     */
    // 解释：调用 `SetVelocityGains` 执行当前步骤需要的功能逻辑。
    void SetVelocityGains(float Kp, float Ki, float Kd);

    /**
     * @brief 设置姿态环 PD 增益
     * @param Kp 比例增益
     * @param Kd 微分增益
     */
    // 解释：调用 `SetAttitudeGains` 执行当前步骤需要的功能逻辑。
    void SetAttitudeGains(float Kp, float Kd);

    /**
     * @brief 设置角速度环比例增益
     * @param Kp 比例增益
     */
    // 解释：调用 `SetAngleRateGains` 执行当前步骤需要的功能逻辑。
    void SetAngleRateGains(float Kp);

    /**
     * @brief 设置航向控制模式
     * @param NewYawMode 偏航模式
     * @param NewDrivetrain 驱动模式
     * @param YawDeg 目标偏航角；仅在角度模式下使用
     */
    // 解释：这一行把右侧表达式的结果写入 `void SetHeadingControl(EDroneYawMode NewYawMode, EDroneDrivetrainMode NewDrivetrain, float YawDeg`，完成 voidsetheadingcontroledroneyaw模式newyaw模式edronedrivetrain模式newdrivetrainfloatyawdeg 的更新。
    void SetHeadingControl(EDroneYawMode NewYawMode, EDroneDrivetrainMode NewDrivetrain, float YawDeg = 0.0f);

    /** @brief 获取当前偏航模式 */
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    EDroneYawMode GetYawMode() const { return YawMode; }

    /** @brief 获取当前驱动模式 */
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    EDroneDrivetrainMode GetDrivetrainMode() const { return DrivetrainMode; }

// 解释：从这一行开始进入 `public` 访问区，下面的成员会对外部模块公开。
public:
    /** @brief 当前使用的物理参数 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DroneMovement|Parameters")
    // 解释：这一行声明成员或局部变量 `Parameters`，用于保存参数。
    FDroneParameters Parameters;

    /** @brief 当前控制模式 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DroneMovement|Control")
    // 解释：这一行声明成员或局部变量 `CurrentControlMode`，用于保存currentcontrol模式。
    EDroneControlMode CurrentControlMode;

    /** @brief 当前仿真状态 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "DroneMovement|State")
    // 解释：这一行声明成员或局部变量 `CurrentState`，用于保存current状态。
    FDroneState CurrentState;

    /** @brief 控制器和控制分配是否已初始化 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "DroneMovement|State")
    // 解释：这一行声明成员或局部变量 `bInitialized`，用于保存布尔标志 initialized。
    bool bInitialized = false;

    /**
     * @brief 自动偏航触发阈值（m/s）
     * 当平面速度或指向目标的位置变化过小时，保持当前航向而不是重新对准。
     */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DroneMovement|Parameters")
    // 解释：这一行声明成员或局部变量 `YawSpeedThreshold`，用于保存yawspeedthreshold。
    float YawSpeedThreshold = 0.3f;

    /**
     * @brief 单帧允许执行的最大固定子步数
     * 用于限制低帧率时的子步积压。
     */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DroneMovement|Parameters", meta=(ClampMin="1", ClampMax="64"))
    // 解释：这一行声明成员或局部变量 `MaxSubStepsPerTick`，用于保存maxsubstepspertick。
    int32 MaxSubStepsPerTick = 16;

    /**
     * @brief 自动偏航附加角度偏移（度）
     * 0 表示机头对准 Actor +X 方向；如果模型前向与控制约定不一致，可在此修正。
     */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DroneMovement|Parameters")
    // 解释：这一行声明成员或局部变量 `YawOffset`，用于保存yawoffset。
    float YawOffset = 0.0f;

// 解释：从这一行开始进入 `protected` 访问区，下面的成员只对本类及其派生类可见。
protected:
    /**
     * @brief 执行一帧级联控制更新
     * @param DeltaTime 控制步长
     */
    // 解释：调用 `ControlUpdate` 执行当前步骤需要的功能逻辑。
    void ControlUpdate(double DeltaTime);

    /**
     * @brief 位置环
     * @param PositionCommand 目标位置
     * @return 速度命令
     */
    // 解释：调用 `PositionLoop` 执行当前步骤需要的功能逻辑。
    FVector PositionLoop(const FVector& PositionCommand);

    /**
     * @brief 速度环
     * @param VelocityCommand 目标速度
     * @param OutThrust 输出总推力
     * @return 加速度命令
     */
    // 解释：调用 `VelocityLoop` 执行当前步骤需要的功能逻辑。
    FVector VelocityLoop(const FVector& VelocityCommand, double& OutThrust);

    /**
     * @brief 姿态环
     * @param AccelerationCommand 期望加速度
     * @return 角速度命令
     */
    // 解释：调用 `AttitudeLoop` 执行当前步骤需要的功能逻辑。
    FVector AttitudeLoop(const FVector& AccelerationCommand);

    /**
     * @brief 角速度环
     * @param AngularVelocityCommand 目标角速度
     * @return 力矩命令
     */
    // 解释：调用 `AngularVelocityLoop` 执行当前步骤需要的功能逻辑。
    FVector AngularVelocityLoop(const FVector& AngularVelocityCommand);

    /**
     * @brief 将总推力和力矩分配为四个电机转速
     * @param TorqueCommand 力矩命令
     * @param Thrust 总推力
     * @return 四个电机转速
     */
    // 解释：调用 `CalculateMotorSpeeds` 执行当前步骤需要的功能逻辑。
    TArray<double> CalculateMotorSpeeds(const FVector& TorqueCommand, double Thrust);

    /**
     * @brief 使用 Velocity Verlet 更新动力学状态
     * @param DeltaTime 积分步长
     */
    // 解释：调用 `VerletUpdate` 执行当前步骤需要的功能逻辑。
    void VerletUpdate(double DeltaTime);

    /**
     * @brief 计算当前状态下的合力和合力矩
     * @param State 当前状态
     * @param OutForce 输出世界坐标系合力
     * @param OutTorque 输出机体系合力矩
     */
    // 解释：调用 `CalculateTotalForcesAndTorques` 执行当前步骤需要的功能逻辑。
    void CalculateTotalForcesAndTorques(const FDroneState& State, FVector& OutForce, FVector& OutTorque);

    /**
     * @brief 检查并处理地面碰撞
     * @param GroundZ 地面高度
     */
    // 解释：这一行把右侧表达式的结果写入 `void CheckGroundCollision(double GroundZ`，完成 voidcheckgroundcollisiondoublegroundZ 的更新。
    void CheckGroundCollision(double GroundZ = 0.0);

    /**
     * @brief 将机体系向量旋转到世界坐标系
     * @param BodyVector 机体系向量
     * @param Orientation 机体姿态四元数
     * @return 世界系向量
     */
    // 解释：调用 `RotateBodyToWorld` 执行当前步骤需要的功能逻辑。
    FVector RotateBodyToWorld(const FVector& BodyVector, const FQuat& Orientation);

    /** @brief 创建并初始化各级控制器 */
    // 解释：调用 `InitializeControllers` 执行当前步骤需要的功能逻辑。
    void InitializeControllers();

    /** @brief 重置所有控制器内部状态 */
    // 解释：调用 `ResetAllControllers` 执行当前步骤需要的功能逻辑。
    void ResetAllControllers();
    // 解释：调用 `SyncControllerTimeSteps` 执行当前步骤需要的功能逻辑。
    void SyncControllerTimeSteps(double ControlTimeStep);
    // 解释：调用 `UpdateDesiredYawFromPlanarVector` 执行当前步骤需要的功能逻辑。
    void UpdateDesiredYawFromPlanarVector(double DirX, double DirY, double CurrentYaw);

    /**
     * @brief 将角度归一化到 [-PI, PI]
     * @param AngleRad 输入角度（弧度）
     * @return 归一化后的角度
     */
    // 解释：调用 `NormalizeAngle` 执行当前步骤需要的功能逻辑。
    static double NormalizeAngle(double AngleRad);

    /** @brief 根据当前参数计算控制分配矩阵及其逆矩阵 */
    // 解释：调用 `ComputeControlAllocation` 执行当前步骤需要的功能逻辑。
    void ComputeControlAllocation();

// 解释：从这一行开始进入 `private` 访问区，下面的成员只允许类内部访问。
private:
    /** @brief 直接控制模式使用的原始命令缓存 */
    // 解释：这一行声明成员或局部变量 `ControlCommands`，用于保存controlcommands。
    TArray<double> ControlCommands;

    /** @brief 当前是否处于地面锁定状态 */
    // 解释：这一行声明成员或局部变量 `bGrounded`，用于保存布尔标志 grounded。
    bool bGrounded = false;

    /** @brief 上一积分步的线加速度 */
    // 解释：这一行声明成员或局部变量 `PrevLinearAcceleration`，用于保存prevlinearacceleration。
    FVector PrevLinearAcceleration = FVector::ZeroVector;

    /** @brief 上一积分步的角加速度 */
    // 解释：这一行声明成员或局部变量 `PrevAngularAcceleration`，用于保存prevangularacceleration。
    FVector PrevAngularAcceleration = FVector::ZeroVector;

    /** @brief 初始地面高度 */
    // 解释：这一行声明成员或局部变量 `InitialGroundZ`，用于保存initialgroundZ。
    double InitialGroundZ = 0.0;

    /** @brief 电机一阶滤波后的转速结果 */
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    TArray<double> MotorSpeedsFiltered = {0.0, 0.0, 0.0, 0.0};

    /** @brief 最近一次控制更新使用的步长 */
    // 解释：这一行声明成员或局部变量 `LastControlDeltaTime`，用于保存lastcontroldeltatime。
    double LastControlDeltaTime = 0.003;

    /** @brief 固定步长积分的累计时间 */
    // 解释：这一行声明成员或局部变量 `FixedStepAccumulator`，用于保存fixedstepaccumulator。
    double FixedStepAccumulator = 0.0;

    /** @brief 保持或自动偏航模式下锁定的目标航向 */
    // 解释：这一行声明成员或局部变量 `LockedYaw`，用于保存lockedyaw。
    double LockedYaw = 0.0;

    /** @brief 偏航参考是否已初始化 */
    // 解释：这一行声明成员或局部变量 `bYawInitialized`，用于保存布尔标志 yawinitialized。
    bool bYawInitialized = false;

    /** @brief 本帧期望偏航角，由 ControlUpdate 计算 */
    // 解释：这一行声明成员或局部变量 `DesiredYaw`，用于保存desiredyaw。
    double DesiredYaw = 0.0;

    /** @brief 位置模式目标位置 */
    // 解释：这一行声明成员或局部变量 `TargetPosition`，用于保存targetposition。
    FVector TargetPosition;

    /** @brief 速度模式目标速度 */
    // 解释：这一行声明成员或局部变量 `TargetVelocity`，用于保存targetvelocity。
    FVector TargetVelocity;

    /** @brief 姿态模式目标姿态 */
    // 解释：这一行声明成员或局部变量 `TargetAttitude`，用于保存targetattitude。
    FRotator TargetAttitude;

    /** @brief 姿态模式目标总推力 */
    // 解释：这一行声明成员或局部变量 `TargetThrust`，用于保存targetthrust。
    double TargetThrust;

    /** @brief 位置环默认速度限制 */
    // 解释：这一行声明成员或局部变量 `DefaultPositionSpeedLimit`，用于保存defaultpositionspeedlimit。
    double DefaultPositionSpeedLimit = 2.0;

    /** @brief 位置环各轴速度限制 */
    // 解释：这一行声明成员或局部变量 `PositionAxisSpeedLimit`，用于保存positionaxisspeedlimit。
    double PositionAxisSpeedLimit = 2.0;

    /** @brief 当前目标位置使用的速度限制 */
    // 解释：这一行声明成员或局部变量 `TargetPositionSpeedLimit`，用于保存targetpositionspeedlimit。
    double TargetPositionSpeedLimit = 2.0;

    /** @brief 当前偏航模式 */
    // 解释：这一行声明成员或局部变量 `YawMode`，用于保存yaw模式。
    EDroneYawMode YawMode = EDroneYawMode::Auto;

    /** @brief 当前驱动模式 */
    // 解释：这一行声明成员或局部变量 `DrivetrainMode`，用于保存drivetrain模式。
    EDroneDrivetrainMode DrivetrainMode = EDroneDrivetrainMode::ForwardOnly;

    /** @brief 外部指定的偏航角命令（弧度） */
    // 解释：这一行声明成员或局部变量 `CommandedYaw`，用于保存commandedyaw。
    double CommandedYaw = 0.0;

    // 位置环 PD
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY() UPDController* PxController;
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY() UPDController* PyController;
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY() UPDController* PzController;

    // 速度环 PID
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY() UPIDController* VxController;
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY() UPIDController* VyController;
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY() UPIDController* VzController;

    // 姿态环 PD
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY() UPDController* RollController;
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY() UPDController* PitchController;
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY() UPDController* YawController;

    // 角速度环 PID
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY() UPIDController* RollRateController;
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY() UPIDController* PitchRateController;
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY() UPIDController* YawRateController;

    /**
     * @brief 控制分配矩阵 G（4x4）
     * 第 0 行对应总推力，第 1~3 行分别对应滚转、俯仰和偏航力矩。
     */
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    double G[4][4];

    /** @brief 控制分配矩阵的逆矩阵 GInv（4x4） */
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    double GInv[4][4];
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
};

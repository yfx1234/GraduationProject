// 解释：引入当前实现文件对应的头文件 `DroneMovementComponent.h`，使实现部分能够看到类和函数声明。
#include "DroneMovementComponent.h"
// 解释：引入 `PDController.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "GraduationProject/Core/Controller/PDController.h"
// 解释：引入 `PIDController.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "GraduationProject/Core/Controller/PIDController.h"

/** @brief 构造函数 */
// 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
UDroneMovementComponent::UDroneMovementComponent()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `PrimaryComponentTick.bCanEverTick`，完成 布尔标志 canevertick 的更新。
    PrimaryComponentTick.bCanEverTick = true;
    // 解释：这一行把右侧表达式的结果写入 `PrimaryComponentTick.TickGroup`，完成 tickgroup 的更新。
    PrimaryComponentTick.TickGroup = TG_PrePhysics; 
    // 解释：这一行把右侧表达式的结果写入 `CurrentControlMode`，完成 currentcontrol模式 的更新。
    CurrentControlMode = EDroneControlMode::Idle;
    // 解释：这一行把右侧表达式的结果写入 `ControlCommands`，完成 controlcommands 的更新。
    ControlCommands = {0.0, 0.0, 0.0, 0.0};
    // 解释：这一行把右侧表达式的结果写入 `TargetPosition`，完成 targetposition 的更新。
    TargetPosition = FVector::ZeroVector;
    // 解释：这一行把右侧表达式的结果写入 `TargetVelocity`，完成 targetvelocity 的更新。
    TargetVelocity = FVector::ZeroVector;
    // 解释：这一行把右侧表达式的结果写入 `TargetAttitude`，完成 targetattitude 的更新。
    TargetAttitude = FRotator::ZeroRotator;
    // 解释：这一行把右侧表达式的结果写入 `TargetThrust`，完成 targetthrust 的更新。
    TargetThrust = 0.0;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 组件初始化
 * 创建 12 个 PD/PID 控制器实例，计算控制分配矩阵及其逆矩阵。
 */
// 解释：这一行定义函数 `BeginPlay`，开始实现beginplay的具体逻辑。
void UDroneMovementComponent::BeginPlay()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：调用 `BeginPlay` 执行当前步骤需要的功能逻辑。
    Super::BeginPlay();
    // 解释：调用 `InitializeComputed` 执行当前步骤需要的功能逻辑。
    Parameters.InitializeComputed();
    // 解释：调用 `InitializeControllers` 执行当前步骤需要的功能逻辑。
    InitializeControllers();
    // 解释：调用 `ComputeControlAllocation` 执行当前步骤需要的功能逻辑。
    ComputeControlAllocation();
    // 解释：这一行把右侧表达式的结果写入 `bInitialized`，完成 布尔标志 initialized 的更新。
    bInitialized = true;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 每帧 Tick 控制更新
 * @param DeltaTime 帧间隔时间
 */
// 解释：这一行定义函数 `TickComponent`，开始实现tick组件的具体逻辑。
void UDroneMovementComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：调用 `TickComponent` 执行当前步骤需要的功能逻辑。
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!bInitialized) return;
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (CurrentControlMode == EDroneControlMode::Idle) return;

    // 解释：这一行通过 `FMath::Max` 给 `const double FixedStep` 施加下界约束，避免 constdoublefixedstep 过小。
    const double FixedStep = FMath::Max(0.0005, static_cast<double>(Parameters.TimeStep));
    // 解释：这一行通过 `FMath::Max` 给 `const int32 MaxSubSteps` 施加下界约束，避免 constint32maxsubsteps 过小。
    const int32 MaxSubSteps = FMath::Max(1, MaxSubStepsPerTick);

    // 固定步长积分：累计真实帧时间，按 TimeStep 离散推进。
    // 解释：这一行先对计算结果做限幅，再写入 `FixedStepAccumulator`，防止 fixedstepaccumulator 超出允许范围。
    FixedStepAccumulator = FMath::Clamp(FixedStepAccumulator + static_cast<double>(DeltaTime), 0.0, FixedStep * MaxSubSteps);

    // 解释：这一行声明成员或局部变量 `BackupState`，用于保存backup状态。
    FDroneState BackupState = CurrentState;
    // 解释：这一行声明成员或局部变量 `ExecutedSteps`，用于保存executedsteps。
    int32 ExecutedSteps = 0;
    // 解释：这一行开始 `while` 循环，只要条件保持为真就持续重复执行。
    while (FixedStepAccumulator >= FixedStep && ExecutedSteps < MaxSubSteps)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 控制更新（每个子步都运行，与物理同频）
        // 解释：调用 `ControlUpdate` 执行当前步骤需要的功能逻辑。
        ControlUpdate(FixedStep);

        // 物理更新
        // 解释：调用 `VerletUpdate` 执行当前步骤需要的功能逻辑。
        VerletUpdate(FixedStep);
        // 解释：调用 `CheckGroundCollision` 执行当前步骤需要的功能逻辑。
        CheckGroundCollision(InitialGroundZ);

        // 解释：这一行从 `FixedStepAccumulator` 中减去新量，用于修正或抵消 fixedstepaccumulator。
        FixedStepAccumulator -= FixedStep;
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        ++ExecutedSteps;

        // NaN 保护
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (CurrentState.HasNaN())
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：调用 `UE_LOG` 输出调试日志，便于运行时观察状态和排查问题。
            UE_LOG(LogTemp, Warning, TEXT("[DroneMovement] NaN detected, reverting to backup state"));
            // 解释：这一行把右侧表达式的结果写入 `CurrentState`，完成 current状态 的更新。
            CurrentState = BackupState;
            // 解释：这一行把右侧表达式的结果写入 `CurrentState.AngRollRate`，完成 angrollrate 的更新。
            CurrentState.AngRollRate = 0.0;
            // 解释：这一行把右侧表达式的结果写入 `CurrentState.AngPitchRate`，完成 angpitchrate 的更新。
            CurrentState.AngPitchRate = 0.0;
            // 解释：这一行把右侧表达式的结果写入 `CurrentState.AngYawRate`，完成 angyawrate 的更新。
            CurrentState.AngYawRate = 0.0;
            // 解释：这一行把右侧表达式的结果写入 `PrevLinearAcceleration`，完成 prevlinearacceleration 的更新。
            PrevLinearAcceleration = FVector::ZeroVector;
            // 解释：这一行把右侧表达式的结果写入 `PrevAngularAcceleration`，完成 prevangularacceleration 的更新。
            PrevAngularAcceleration = FVector::ZeroVector;
            // 解释：这一行把右侧表达式的结果写入 `FixedStepAccumulator`，完成 fixedstepaccumulator 的更新。
            FixedStepAccumulator = 0.0;
            // 解释：这一行立即跳出当前循环或 `switch` 分支，避免继续执行后续分支。
            break;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 速度裁剪防发散
    // 解释：调用 `ClampVelocities` 执行当前步骤需要的功能逻辑。
    CurrentState.ClampVelocities(50.0, 50.0);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 设置物理参数并重新初始化
 * @param NewParameters 新的物理参数
 * 更新参数后重新计算控制分配矩阵和重新初始化控制器
 */
// 解释：这一行定义函数 `SetParameters`，开始实现set参数的具体逻辑。
void UDroneMovementComponent::SetParameters(const FDroneParameters& NewParameters)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `Parameters`，完成 参数 的更新。
    Parameters = NewParameters;
    // 解释：调用 `InitializeComputed` 执行当前步骤需要的功能逻辑。
    Parameters.InitializeComputed();
    // 解释：调用 `ComputeControlAllocation` 执行当前步骤需要的功能逻辑。
    ComputeControlAllocation();
    // 解释：调用 `InitializeControllers` 执行当前步骤需要的功能逻辑。
    InitializeControllers();
    // 解释：这一行把右侧表达式的结果写入 `FixedStepAccumulator`，完成 fixedstepaccumulator 的更新。
    FixedStepAccumulator = 0.0;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 创建并初始化所有 PD/PID 控制器
 * 位置环(PD)：Kp=1.0, Kd=0.3, 输出限幅 2.0 m/s
 * 速度环(PID)：Kp=2.0, Ki=0.1, Kd=0.1, 输出限幅 3.0 m/s²
 * 姿态环(PD)：Kp=6.0, Kd=0.3（Yaw 减半），输出限幅 3.0 rad/s
 * 角速率环(PID)：Kp=0.5, Ki=0.0, Kd=0.0，输出限幅 1.0 N·m
 */
// 解释：这一行定义函数 `InitializeControllers`，开始实现initializecontrollers的具体逻辑。
void UDroneMovementComponent::InitializeControllers()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行声明成员或局部变量 `Ts`，用于保存ts。
    double Ts = Parameters.TimeStep;

    // ── 位置环 PD ── Kp=1.0, Kd=0.3
    // 解释：这一行声明成员或局部变量 `PosPGain`，用于保存pospgain。
    float PosPGain = 1.0f;
    // 解释：这一行声明成员或局部变量 `PosDGain`，用于保存posdgain。
    float PosDGain = 0.3f;
    // 解释：这一行把右侧表达式的结果写入 `PositionAxisSpeedLimit`，完成 positionaxisspeedlimit 的更新。
    PositionAxisSpeedLimit = DefaultPositionSpeedLimit;
    // 解释：这一行把右侧表达式的结果写入 `TargetPositionSpeedLimit`，完成 targetpositionspeedlimit 的更新。
    TargetPositionSpeedLimit = PositionAxisSpeedLimit;
    // 解释：这一行把右侧表达式的结果写入 `float VelMaxLimit`，完成 floatvelmaxlimit 的更新。
    float VelMaxLimit = static_cast<float>(PositionAxisSpeedLimit);
    // 解释：这一行把右侧表达式的结果写入 `PxController`，完成 px控制器 的更新。
    PxController = NewObject<UPDController>(this);
    // 解释：调用 `Initialize` 执行当前步骤需要的功能逻辑。
    PxController->Initialize(PosPGain, PosDGain, 0.1, VelMaxLimit, Ts);
    // 解释：这一行把右侧表达式的结果写入 `PyController`，完成 py控制器 的更新。
    PyController = NewObject<UPDController>(this);
    // 解释：调用 `Initialize` 执行当前步骤需要的功能逻辑。
    PyController->Initialize(PosPGain, PosDGain, 0.1, VelMaxLimit, Ts);
    // 解释：这一行把右侧表达式的结果写入 `PzController`，完成 pz控制器 的更新。
    PzController = NewObject<UPDController>(this);
    // 解释：调用 `Initialize` 执行当前步骤需要的功能逻辑。
    PzController->Initialize(PosPGain, PosDGain, 0.1, VelMaxLimit, Ts);

    // ── 速度环 PID ── Kp=2.0, Ki=0.1, Kd=0.1
    // 解释：这一行声明成员或局部变量 `VelPGain`，用于保存velpgain。
    float VelPGain = 2.0f;
    // 解释：这一行声明成员或局部变量 `VelIGain`，用于保存veligain。
    float VelIGain = 0.1f;
    // 解释：这一行声明成员或局部变量 `VelDGain`，用于保存veldgain。
    float VelDGain = 0.1f;
    // 解释：这一行把右侧表达式的结果写入 `VxController`，完成 vx控制器 的更新。
    VxController = NewObject<UPIDController>(this);
    // 解释：调用 `Initialize` 执行当前步骤需要的功能逻辑。
    VxController->Initialize(VelPGain, VelIGain, VelDGain, 0.1, 2.0, Ts, -5.0, 5.0);
    // 解释：这一行把右侧表达式的结果写入 `VyController`，完成 vy控制器 的更新。
    VyController = NewObject<UPIDController>(this);
    // 解释：调用 `Initialize` 执行当前步骤需要的功能逻辑。
    VyController->Initialize(VelPGain, VelIGain, VelDGain, 0.1, 2.0, Ts, -5.0, 5.0);
    // 解释：这一行把右侧表达式的结果写入 `VzController`，完成 vz控制器 的更新。
    VzController = NewObject<UPIDController>(this);
    // 解释：调用 `Initialize` 执行当前步骤需要的功能逻辑。
    VzController->Initialize(VelPGain, VelIGain, VelDGain, 0.1, 2.0, Ts, -5.0, 5.0);

    // ── 姿态环 PD ── Kp=6.0, Kd=0.3
    // 解释：这一行声明成员或局部变量 `AnglePGain`，用于保存anglepgain。
    float AnglePGain = 6.0f;
    // 解释：这一行声明成员或局部变量 `AngleDGain`，用于保存angledgain。
    float AngleDGain = 0.3f;
    // 解释：这一行声明成员或局部变量 `AngVelMaxLimit`，用于保存angvelmaxlimit。
    float AngVelMaxLimit = 3.0f;
    // 解释：这一行把右侧表达式的结果写入 `RollController`，完成 roll控制器 的更新。
    RollController = NewObject<UPDController>(this);
    // 解释：调用 `Initialize` 执行当前步骤需要的功能逻辑。
    RollController->Initialize(AnglePGain, AngleDGain, 0.1, AngVelMaxLimit, Ts);
    // 解释：这一行把右侧表达式的结果写入 `PitchController`，完成 pitch控制器 的更新。
    PitchController = NewObject<UPDController>(this);
    // 解释：调用 `Initialize` 执行当前步骤需要的功能逻辑。
    PitchController->Initialize(AnglePGain, AngleDGain, 0.1, AngVelMaxLimit, Ts);
    // 解释：这一行把右侧表达式的结果写入 `YawController`，完成 yaw控制器 的更新。
    YawController = NewObject<UPDController>(this);
    // 解释：调用 `Initialize` 执行当前步骤需要的功能逻辑。
    YawController->Initialize(AnglePGain * 0.5f, AngleDGain * 0.5f, 0.1, AngVelMaxLimit, Ts);

    // ── 角速率环 PID ── Kp=0.5, Ki=0, Kd=0（内环宜快，纯P足够）
    // 解释：这一行声明成员或局部变量 `RatePGain`，用于保存ratepgain。
    float RatePGain = 0.5f;
    // 解释：这一行声明成员或局部变量 `TorqueMaxLimit`，用于保存torquemaxlimit。
    float TorqueMaxLimit = 1.0f;
    // 解释：这一行把右侧表达式的结果写入 `RollRateController`，完成 rollrate控制器 的更新。
    RollRateController = NewObject<UPIDController>(this);
    // 解释：调用 `Initialize` 执行当前步骤需要的功能逻辑。
    RollRateController->Initialize(RatePGain, 0.0, 0.0, 0.05, TorqueMaxLimit, Ts, -2.0, 2.0);
    // 解释：这一行把右侧表达式的结果写入 `PitchRateController`，完成 pitchrate控制器 的更新。
    PitchRateController = NewObject<UPIDController>(this);
    // 解释：调用 `Initialize` 执行当前步骤需要的功能逻辑。
    PitchRateController->Initialize(RatePGain, 0.0, 0.0, 0.05, TorqueMaxLimit, Ts, -2.0, 2.0);
    // 解释：这一行把右侧表达式的结果写入 `YawRateController`，完成 yawrate控制器 的更新。
    YawRateController = NewObject<UPIDController>(this);
    // 解释：调用 `Initialize` 执行当前步骤需要的功能逻辑。
    YawRateController->Initialize(RatePGain * 0.5f, 0.0, 0.0, 0.05, TorqueMaxLimit * 0.5f, Ts, -1.0, 1.0);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 计算控制分配矩阵 G 及其逆矩阵 GInv
 *  总推力 T = F1 + F2 + F3 + F4 = kT * ω1² + kT * ω2² + kT * ω3² + kT * ω4²
 *  滚转力矩 τx = F1 * L * sinβ - F2 * L * sinβ - F3 * L * sinβ + F4 * L * sinβ 
 *  俯仰力矩 τy = -F1 * L * cosβ - F2 * L * cosβ + F3 * L * cosβ + F4 * L * cosβ
 *  偏航力矩 τz = ω1² * kQ - ω2² * kQ + ω3² * kQ - ω4² * kQ
 *  控制分配矩阵 [T, τx, τy, τz]^T = G * [ω1², ω2², ω3², ω4²]^T
 */
// 解释：这一行定义函数 `ComputeControlAllocation`，开始实现computecontrolallocation的具体逻辑。
void UDroneMovementComponent::ComputeControlAllocation()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行声明成员或局部变量 `kT`，用于保存KT。
    double kT = Parameters.ThrustCoefficient;
    // 解释：这一行声明成员或局部变量 `kQ`，用于保存KQ。
    double kQ = Parameters.TorqueCoefficient;
    // 解释：这一行声明成员或局部变量 `L`，用于保存L。
    double L = Parameters.ArmLength;
    // 解释：这一行把右侧表达式的结果写入 `double Beta`，完成 doublebeta 的更新。
    double Beta = FMath::DegreesToRadians(Parameters.MotorAngle); // 电机臂角度
    // 解释：这一行把右侧表达式的结果写入 `double SinB`，完成 doublesinB 的更新。
    double SinB = FMath::Sin(Beta);
    // 解释：这一行把右侧表达式的结果写入 `double CosB`，完成 doublecosB 的更新。
    double CosB = FMath::Cos(Beta);
    // 解释：这一行把右侧表达式的结果写入 `G[0][0]`，完成 G 的更新。
    G[0][0] = kT; 
    // 解释：这一行把右侧表达式的结果写入 `G[0][1]`，完成 G 的更新。
    G[0][1] = kT; 
    // 解释：这一行把右侧表达式的结果写入 `G[0][2]`，完成 G 的更新。
    G[0][2] = kT; 
    // 解释：这一行把右侧表达式的结果写入 `G[0][3]`，完成 G 的更新。
    G[0][3] = kT;
    // 解释：这一行把右侧表达式的结果写入 `G[1][0]`，完成 G 的更新。
    G[1][0] = kT*L*SinB; 
    // 解释：这一行把右侧表达式的结果写入 `G[1][1]`，完成 G 的更新。
    G[1][1] = -kT*L*SinB; 
    // 解释：这一行把右侧表达式的结果写入 `G[1][2]`，完成 G 的更新。
    G[1][2] = -kT*L*SinB; 
    // 解释：这一行把右侧表达式的结果写入 `G[1][3]`，完成 G 的更新。
    G[1][3] = kT*L*SinB;
    // 解释：这一行把右侧表达式的结果写入 `G[2][0]`，完成 G 的更新。
    G[2][0] = kT*L*CosB; 
    // 解释：这一行把右侧表达式的结果写入 `G[2][1]`，完成 G 的更新。
    G[2][1] = kT*L*CosB; 
    // 解释：这一行把右侧表达式的结果写入 `G[2][2]`，完成 G 的更新。
    G[2][2] = -kT*L*CosB; 
    // 解释：这一行把右侧表达式的结果写入 `G[2][3]`，完成 G 的更新。
    G[2][3] = -kT*L*CosB;
    // 解释：这一行把右侧表达式的结果写入 `G[3][0]`，完成 G 的更新。
    G[3][0] = kQ; 
    // 解释：这一行把右侧表达式的结果写入 `G[3][1]`，完成 G 的更新。
    G[3][1] = -kQ; 
    // 解释：这一行把右侧表达式的结果写入 `G[3][2]`，完成 G 的更新。
    G[3][2] = kQ; 
    // 解释：这一行把右侧表达式的结果写入 `G[3][3]`，完成 G 的更新。
    G[3][3] = -kQ;
    // 解释：这一行把右侧表达式的结果写入 `double a`，完成 doubleA 的更新。
    double a = 1.0 / (4.0 * kT);             
    // 解释：这一行把右侧表达式的结果写入 `double b`，完成 doubleB 的更新。
    double b = 1.0 / (4.0 * kT * L * SinB);  
    // 解释：这一行把右侧表达式的结果写入 `double c`，完成 doubleC 的更新。
    double c = 1.0 / (4.0 * kT * L * CosB);  
    // 解释：这一行把右侧表达式的结果写入 `double d`，完成 doubleD 的更新。
    double d = 1.0 / (4.0 * kQ);             
    // 解释：这一行把右侧表达式的结果写入 `GInv[0][0]`，完成 ginv 的更新。
    GInv[0][0] = a; GInv[0][1] = b;  GInv[0][2] = c;  GInv[0][3] = d;
    // 解释：这一行把右侧表达式的结果写入 `GInv[1][0]`，完成 ginv 的更新。
    GInv[1][0] = a; GInv[1][1] = -b; GInv[1][2] = c;  GInv[1][3] = -d;
    // 解释：这一行把右侧表达式的结果写入 `GInv[2][0]`，完成 ginv 的更新。
    GInv[2][0] = a; GInv[2][1] = -b; GInv[2][2] = -c; GInv[2][3] = d;
    // 解释：这一行把右侧表达式的结果写入 `GInv[3][0]`，完成 ginv 的更新。
    GInv[3][0] = a; GInv[3][1] = b;  GInv[3][2] = -c; GInv[3][3] = -d;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

// 解释：这一行定义函数 `SyncControllerTimeSteps`，开始实现sync控制器timesteps的具体逻辑。
void UDroneMovementComponent::SyncControllerTimeSteps(double ControlTimeStep)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (PxController) PxController->SetTimeStep(ControlTimeStep);
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (PyController) PyController->SetTimeStep(ControlTimeStep);
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (PzController) PzController->SetTimeStep(ControlTimeStep);
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (VxController) VxController->SetTimeStep(ControlTimeStep);
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (VyController) VyController->SetTimeStep(ControlTimeStep);
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (VzController) VzController->SetTimeStep(ControlTimeStep);
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (RollController) RollController->SetTimeStep(ControlTimeStep);
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (PitchController) PitchController->SetTimeStep(ControlTimeStep);
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (YawController) YawController->SetTimeStep(ControlTimeStep);
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (RollRateController) RollRateController->SetTimeStep(ControlTimeStep);
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (PitchRateController) PitchRateController->SetTimeStep(ControlTimeStep);
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (YawRateController) YawRateController->SetTimeStep(ControlTimeStep);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

// 解释：这一行定义函数 `UpdateDesiredYawFromPlanarVector`，开始实现updatedesiredyawfromplanarvector的具体逻辑。
void UDroneMovementComponent::UpdateDesiredYawFromPlanarVector(double DirX, double DirY, double CurrentYaw)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `const double DirectionMagnitude`，完成 constdoubledirectionmagnitude 的更新。
    const double DirectionMagnitude = FMath::Sqrt(DirX * DirX + DirY * DirY);

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (YawMode == EDroneYawMode::Angle)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `DesiredYaw`，完成 desiredyaw 的更新。
        DesiredYaw = CommandedYaw;
        // 解释：这一行把右侧表达式的结果写入 `LockedYaw`，完成 lockedyaw 的更新。
        LockedYaw = DesiredYaw;
        // 解释：这一行把右侧表达式的结果写入 `bYawInitialized`，完成 布尔标志 yawinitialized 的更新。
        bYawInitialized = true;
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (YawMode == EDroneYawMode::Hold || DrivetrainMode == EDroneDrivetrainMode::MaxDegreeOfFreedom)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (!bYawInitialized)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行把右侧表达式的结果写入 `LockedYaw`，完成 lockedyaw 的更新。
            LockedYaw = CurrentYaw;
            // 解释：这一行把右侧表达式的结果写入 `bYawInitialized`，完成 布尔标志 yawinitialized 的更新。
            bYawInitialized = true;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
        // 解释：这一行把右侧表达式的结果写入 `DesiredYaw`，完成 desiredyaw 的更新。
        DesiredYaw = LockedYaw;
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (DirectionMagnitude > YawSpeedThreshold)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `DesiredYaw`，完成 desiredyaw 的更新。
        DesiredYaw = FMath::Atan2(DirY, DirX) + FMath::DegreesToRadians(YawOffset);
        // 解释：这一行把右侧表达式的结果写入 `LockedYaw`，完成 lockedyaw 的更新。
        LockedYaw = DesiredYaw;
        // 解释：这一行把右侧表达式的结果写入 `bYawInitialized`，完成 布尔标志 yawinitialized 的更新。
        bYawInitialized = true;
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行把右侧表达式的结果写入 `DesiredYaw`，完成 desiredyaw 的更新。
    DesiredYaw = bYawInitialized ? LockedYaw : CurrentYaw;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 执行一帧的级联控制更新
 * @param DeltaTime 帧间隔
 * 根据控制模式更新推力力矩，再得到电机转速
 */
// 解释：这一行定义函数 `ControlUpdate`，开始实现controlupdate的具体逻辑。
void UDroneMovementComponent::ControlUpdate(double DeltaTime)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `LastControlDeltaTime`，完成 lastcontroldeltatime 的更新。
    LastControlDeltaTime = DeltaTime;  // 保存帧时间供电机滤波器使用
    // 解释：这一行通过 `FMath::Max` 给 `const double ControlTimeStep` 施加下界约束，避免 constdoublecontroltimestep 过小。
    const double ControlTimeStep = FMath::Max(0.001, static_cast<double>(DeltaTime));
    // 解释：调用 `SyncControllerTimeSteps` 执行当前步骤需要的功能逻辑。
    SyncControllerTimeSteps(ControlTimeStep);

    // 解释：这一行声明成员或局部变量 `TorqueCommand`，用于保存torque命令。
    FVector TorqueCommand = FVector::ZeroVector;
    // 解释：这一行声明成员或局部变量 `ThrustCommand`，用于保存thrust命令。
    double ThrustCommand = Parameters.Mass * Parameters.Gravity;
    // 解释：这一行进入 `switch` 分支分派结构，后续会按不同枚举或状态值执行不同逻辑。
    switch (CurrentControlMode)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
    // 解释：这一行声明 `switch` 中的一个分支标签，对应某个具体取值的处理路径。
    case EDroneControlMode::MotorSpeed:
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (ControlCommands.Num() >= 4) CurrentState.MotorSpeeds = ControlCommands;
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return;
    // 解释：这一行声明 `switch` 中的一个分支标签，对应某个具体取值的处理路径。
    case EDroneControlMode::Position:
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行把右侧表达式的结果写入 `const FVector Pos`，完成 constfvectorpos 的更新。
            const FVector Pos = CurrentState.GetPosition();
            // 解释：这一行把右侧表达式的结果写入 `const double CurrentYaw`，完成 constdoublecurrentyaw 的更新。
            const double CurrentYaw = FMath::DegreesToRadians(CurrentState.GetRotator().Yaw);
            // 解释：调用 `UpdateDesiredYawFromPlanarVector` 执行当前步骤需要的功能逻辑。
            UpdateDesiredYawFromPlanarVector(TargetPosition.X - Pos.X, TargetPosition.Y - Pos.Y, CurrentYaw);

            // 解释：这一行把右侧表达式的结果写入 `FVector VelCmd`，完成 fvectorvelcmd 的更新。
            FVector VelCmd = PositionLoop(TargetPosition);
            // 解释：这一行把右侧表达式的结果写入 `FVector AccCmd`，完成 fvectoracccmd 的更新。
            FVector AccCmd = VelocityLoop(VelCmd, ThrustCommand);
            // 解释：这一行把右侧表达式的结果写入 `FVector AngVelCmd`，完成 fvectorangvelcmd 的更新。
            FVector AngVelCmd = AttitudeLoop(AccCmd);
            // 解释：这一行把右侧表达式的结果写入 `TorqueCommand`，完成 torque命令 的更新。
            TorqueCommand = AngularVelocityLoop(AngVelCmd);
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
        // 解释：这一行立即跳出当前循环或 `switch` 分支，避免继续执行后续分支。
        break;
    // 解释：这一行声明 `switch` 中的一个分支标签，对应某个具体取值的处理路径。
    case EDroneControlMode::Velocity:
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行把右侧表达式的结果写入 `const double CurrentYaw`，完成 constdoublecurrentyaw 的更新。
            const double CurrentYaw = FMath::DegreesToRadians(CurrentState.GetRotator().Yaw);
            // 解释：调用 `UpdateDesiredYawFromPlanarVector` 执行当前步骤需要的功能逻辑。
            UpdateDesiredYawFromPlanarVector(TargetVelocity.X, TargetVelocity.Y, CurrentYaw);

            // 解释：这一行把右侧表达式的结果写入 `FVector AccCmd`，完成 fvectoracccmd 的更新。
            FVector AccCmd = VelocityLoop(TargetVelocity, ThrustCommand);
            // 解释：这一行把右侧表达式的结果写入 `FVector AngVelCmd`，完成 fvectorangvelcmd 的更新。
            FVector AngVelCmd = AttitudeLoop(AccCmd);
            // 解释：这一行把右侧表达式的结果写入 `TorqueCommand`，完成 torque命令 的更新。
            TorqueCommand = AngularVelocityLoop(AngVelCmd);
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
        // 解释：这一行立即跳出当前循环或 `switch` 分支，避免继续执行后续分支。
        break;
    // 解释：这一行声明 `switch` 中的一个分支标签，对应某个具体取值的处理路径。
    case EDroneControlMode::AttitudeThrust:
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行把右侧表达式的结果写入 `double RollDes`，完成 doublerolldes 的更新。
            double RollDes = FMath::DegreesToRadians(TargetAttitude.Roll);
            // 解释：这一行把右侧表达式的结果写入 `double PitchDes`，完成 doublepitchdes 的更新。
            double PitchDes = FMath::DegreesToRadians(TargetAttitude.Pitch);
            // 解释：这一行把右侧表达式的结果写入 `double YawDes`，完成 doubleyawdes 的更新。
            double YawDes = FMath::DegreesToRadians(TargetAttitude.Yaw);
            // UE FRotator 的 Roll/Pitch 与控制分配矩阵符号相反，需要取反
            // 解释：这一行把右侧表达式的结果写入 `FRotator CurrentRot`，完成 frotatorcurrentrot 的更新。
            FRotator CurrentRot = CurrentState.GetRotator();
            // 解释：这一行把右侧表达式的结果写入 `double Roll`，完成 doubleroll 的更新。
            double Roll = -FMath::DegreesToRadians(CurrentRot.Roll);
            // 解释：这一行把右侧表达式的结果写入 `double Pitch`，完成 doublepitch 的更新。
            double Pitch = -FMath::DegreesToRadians(CurrentRot.Pitch);
            // 解释：这一行把右侧表达式的结果写入 `double Yaw`，完成 doubleyaw 的更新。
            double Yaw = FMath::DegreesToRadians(CurrentRot.Yaw);
            // 解释：这一行把右侧表达式的结果写入 `double RollTarget`，完成 doublerolltarget 的更新。
            double RollTarget = Roll + NormalizeAngle(RollDes - Roll);
            // 解释：这一行把右侧表达式的结果写入 `double PitchTarget`，完成 doublepitchtarget 的更新。
            double PitchTarget = Pitch + NormalizeAngle(PitchDes - Pitch);
            // 解释：这一行把右侧表达式的结果写入 `double YawTarget`，完成 doubleyawtarget 的更新。
            double YawTarget = Yaw + NormalizeAngle(YawDes - Yaw);
            // 解释：这一行把右侧表达式的结果写入 `double RollRateCmd`，完成 doublerollratecmd 的更新。
            double RollRateCmd = RollController ? RollController->Update(RollTarget, Roll) : 0.0;
            // 解释：这一行把右侧表达式的结果写入 `double PitchRateCmd`，完成 doublepitchratecmd 的更新。
            double PitchRateCmd = PitchController ? PitchController->Update(PitchTarget, Pitch) : 0.0;
            // 解释：这一行把右侧表达式的结果写入 `double YawRateCmd`，完成 doubleyawratecmd 的更新。
            double YawRateCmd = YawController ? YawController->Update(YawTarget, Yaw) : 0.0;
            // 解释：这一行把右侧表达式的结果写入 `TorqueCommand`，完成 torque命令 的更新。
            TorqueCommand = AngularVelocityLoop(FVector(RollRateCmd, PitchRateCmd, YawRateCmd));
            // 解释：这一行把右侧表达式的结果写入 `ThrustCommand`，完成 thrust命令 的更新。
            ThrustCommand = TargetThrust;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
        // 解释：这一行立即跳出当前循环或 `switch` 分支，避免继续执行后续分支。
        break;
    // 解释：这一行声明 `switch` 中的一个分支标签，对应某个具体取值的处理路径。
    case EDroneControlMode::TorqueThrust:
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (ControlCommands.Num() >= 4)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行把右侧表达式的结果写入 `TorqueCommand`，完成 torque命令 的更新。
            TorqueCommand = FVector(ControlCommands[0], ControlCommands[1], ControlCommands[2]);
            // 解释：这一行把右侧表达式的结果写入 `ThrustCommand`，完成 thrust命令 的更新。
            ThrustCommand = ControlCommands[3];
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
        // 解释：这一行立即跳出当前循环或 `switch` 分支，避免继续执行后续分支。
        break;
    // 解释：这一行声明 `switch` 中的一个分支标签，对应某个具体取值的处理路径。
    case EDroneControlMode::Idle:
    // 解释：这一行声明 `switch` 的默认分支，当前面所有 `case` 都不匹配时执行。
    default:
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：调用 `CalculateMotorSpeeds` 执行当前步骤需要的功能逻辑。
    TArray<double> MotorSpeeds = CalculateMotorSpeeds(TorqueCommand, ThrustCommand);
    // 解释：这一行把右侧表达式的结果写入 `CurrentState.MotorSpeeds`，完成 motorspeeds 的更新。
    CurrentState.MotorSpeeds = MotorSpeeds;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 从力矩和推力命令计算四个电机的转速
 * @param TorqueCommand 三轴力矩命令
 * @param Thrust 总推力命令
 * @return 四个电机转速
 * 1. 逆分配矩阵求解 ω²
 * 2. Mixer 饱和补偿（下溢偏移 + 上溢缩放）
 * 3. 一阶低通滤波模拟电机惯性
 */
// 解释：这一行定义函数 `CalculateMotorSpeeds`，开始实现calculatemotorspeeds的具体逻辑。
TArray<double> UDroneMovementComponent::CalculateMotorSpeeds(const FVector& TorqueCommand, double Thrust)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    TArray<double> Input = {Thrust, TorqueCommand.X, TorqueCommand.Y, TorqueCommand.Z};
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    TArray<double> OmegaSquared = {0.0, 0.0, 0.0, 0.0};
    // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
    for (int32 i = 0; i < 4; ++i)
        // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
        for (int32 j = 0; j < 4; ++j)
            // 解释：这一行在 `OmegaSquared[i]` 的原有基础上继续累加新量，用于持续更新 omegasquared。
            OmegaSquared[i] += GInv[i][j] * Input[j];

    // 解释：这一行声明成员或局部变量 `MotorSpeeds`，用于保存motorspeeds。
    TArray<double> MotorSpeeds;
    // 解释：调用 `SetNum` 执行当前步骤需要的功能逻辑。
    MotorSpeeds.SetNum(4);
    // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
    for (int32 i = 0; i < 4; ++i)
        // 解释：这一行通过 `FMath::Max` 给 `MotorSpeeds[i]` 施加下界约束，避免 motorspeeds 过小。
        MotorSpeeds[i] = FMath::Sqrt(FMath::Max(0.0, OmegaSquared[i]));

    // --- Mixer 饱和补偿 ---
    // 下溢补偿：若任一电机低于最小值，所有电机加偏移
    // 解释：这一行声明成员或局部变量 `MinSpeed`，用于保存minspeed。
    double MinSpeed = MotorSpeeds[0];
    // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
    for (int32 i = 1; i < 4; ++i)
        // 解释：这一行通过 `FMath::Min` 给 `MinSpeed` 施加上界约束，避免 minspeed 过大。
        MinSpeed = FMath::Min(MinSpeed, MotorSpeeds[i]);
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (MinSpeed < Parameters.MinMotorSpeed)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行声明成员或局部变量 `Undershoot`，用于保存undershoot。
        double Undershoot = Parameters.MinMotorSpeed - MinSpeed;
        // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
        for (int32 i = 0; i < 4; ++i)
            // 解释：这一行在 `MotorSpeeds[i]` 的原有基础上继续累加新量，用于持续更新 motorspeeds。
            MotorSpeeds[i] += Undershoot;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 上溢补偿：若任一电机超过最大值，等比缩放
    // 解释：这一行声明成员或局部变量 `MaxSpeed`，用于保存最大速度约束。
    double MaxSpeed = MotorSpeeds[0];
    // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
    for (int32 i = 1; i < 4; ++i)
        // 解释：这一行通过 `FMath::Max` 给 `MaxSpeed` 施加下界约束，避免 最大速度约束 过小。
        MaxSpeed = FMath::Max(MaxSpeed, MotorSpeeds[i]);
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (MaxSpeed > Parameters.MaxMotorSpeed)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行声明成员或局部变量 `Scale`，用于保存scale。
        double Scale = Parameters.MaxMotorSpeed / MaxSpeed;
        // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
        for (int32 i = 0; i < 4; ++i)
            // 解释：这一行把右侧表达式的结果写入 `MotorSpeeds[i]`，完成 motorspeeds 的更新。
            MotorSpeeds[i] *= Scale;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 最终 Clamp
    // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
    for (int32 i = 0; i < 4; ++i)
        // 解释：这一行先对计算结果做限幅，再写入 `MotorSpeeds[i]`，防止 motorspeeds 超出允许范围。
        MotorSpeeds[i] = FMath::Clamp(MotorSpeeds[i], Parameters.MinMotorSpeed, Parameters.MaxMotorSpeed);

    // --- 一阶低通滤波模拟电机惯性 ---
    // 使用控制更新的实际 DeltaTime，而非物理子步步长
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (MotorSpeedsFiltered.Num() < 4)
        // 解释：这一行把右侧表达式的结果写入 `MotorSpeedsFiltered`，完成 motorspeedsfiltered 的更新。
        MotorSpeedsFiltered = {0.0, 0.0, 0.0, 0.0};
    // 解释：这一行通过 `FMath::Max` 给 `double FilterDt` 施加下界约束，避免 doublefilterdt 过小。
    double FilterDt = FMath::Max(0.001, LastControlDeltaTime);
    // 解释：这一行通过 `FMath::Max` 给 `double Alpha` 施加下界约束，避免 doublealpha 过小。
    double Alpha = FMath::Exp(-FilterDt / FMath::Max(0.001, Parameters.MotorFilterTC));
    // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
    for (int32 i = 0; i < 4; ++i)
        // 解释：这一行把右侧表达式的结果写入 `MotorSpeedsFiltered[i]`，完成 motorspeedsfiltered 的更新。
        MotorSpeedsFiltered[i] = MotorSpeedsFiltered[i] * Alpha + MotorSpeeds[i] * (1.0 - Alpha);

    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return MotorSpeedsFiltered;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief Verlet 积分一步
 * @param DeltaTime 积分步长
 * 使用 Velocity Verlet 方法：
 *   x(t+dt) = x(t) + v(t)*dt + 0.5*a(t)*dt²
 *   a(t+dt) = F(t+dt) / m
 *   v(t+dt) = v(t) + 0.5*(a(t) + a(t+dt))*dt
 * 角度更新使用角速度和 AngleAxis 方法
 */
// 解释：这一行定义函数 `VerletUpdate`，开始实现verletupdate的具体逻辑。
void UDroneMovementComponent::VerletUpdate(double DeltaTime)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行声明成员或局部变量 `dt`，用于保存dt。
    const double dt = DeltaTime;
    // 解释：这一行声明成员或局部变量 `halfDt`，用于保存halfdt。
    const double halfDt = 0.5 * dt;
    // 解释：这一行声明成员或局部变量 `halfDtSq`，用于保存halfdtsq。
    const double halfDtSq = 0.5 * dt * dt;

    // --- Ground Lock：地面上时检查推力的垂直分量是否超过重力 ---
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (bGrounded)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行声明成员或局部变量 `Torque`，用于保存torque。
        FVector Force, Torque;
        // 解释：调用 `CalculateTotalForcesAndTorques` 执行当前步骤需要的功能逻辑。
        CalculateTotalForcesAndTorques(CurrentState, Force, Torque);
        // 检查推力在世界 Z 轴的分量是否超过重力
        // 解释：这一行声明成员或局部变量 `WeightForce`，用于保存weightforce。
        double WeightForce = Parameters.Mass * Parameters.Gravity;
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (Force.Z > WeightForce * 1.05)  // 5% 裕量防止抢起又落下
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行把右侧表达式的结果写入 `bGrounded`，完成 布尔标志 grounded 的更新。
            bGrounded = false;
            // 解释：这一行位于构造函数初始化列表中，把 `UE_LOG` 直接初始化为 `LogTemp, Log, TEXT("[DroneMovement] Lifting off ground, Force.Z=%.2f > Weight=%.2f"`，减少进入函数体后的额外赋值开销。
            UE_LOG(LogTemp, Log, TEXT("[DroneMovement] Lifting off ground, Force.Z=%.2f > Weight=%.2f"),
                // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                Force.Z, WeightForce);
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
        // 解释：这一行给出兜底分支，当上面的条件都不满足时执行。
        else
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return; // 留在地面，不更新状态
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // --- 位置更新：x(t+dt) = x(t) + v(t)*dt + 0.5*a(t)*dt² ---
    // 解释：这一行把右侧表达式的结果写入 `FVector Pos`，完成 fvectorpos 的更新。
    FVector Pos = CurrentState.GetPosition();
    // 解释：这一行把右侧表达式的结果写入 `FVector Vel`，完成 fvectorvel 的更新。
    FVector Vel = CurrentState.GetVelocity();
    // 解释：这一行把右侧表达式的结果写入 `FVector AngVel`，完成 fvectorangvel 的更新。
    FVector AngVel = CurrentState.GetAngularVelocity();

    // 解释：这一行在 `Pos` 的原有基础上继续累加新量，用于持续更新 pos。
    Pos += Vel * dt + PrevLinearAcceleration * halfDtSq;
    // 解释：调用 `SetPosition` 执行当前步骤需要的功能逻辑。
    CurrentState.SetPosition(Pos);

    // --- 姿态更新：用角速度计算角度增量 ---
    // 解释：这一行声明成员或局部变量 `AvgAngVel`，用于保存avgangvel。
    FVector AvgAngVel = AngVel + PrevAngularAcceleration * halfDt;
    // 解释：这一行把向量模长写入 `double AnglePerUnit`，用于表示距离、速度大小或不确定度。
    double AnglePerUnit = AvgAngVel.Size();
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (AnglePerUnit > KINDA_SMALL_NUMBER)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `DeltaQ` 执行当前步骤需要的功能逻辑。
        FQuat DeltaQ(AvgAngVel / AnglePerUnit, AnglePerUnit * dt);
        // 解释：这一行把右侧表达式的结果写入 `FQuat CurrentQ`，完成 fquatcurrentQ 的更新。
        FQuat CurrentQ = CurrentState.GetQuaternion();
        // 解释：这一行声明成员或局部变量 `NewQ`，用于保存newQ。
        FQuat NewQ = CurrentQ * DeltaQ;
        // 解释：调用 `Normalize` 执行当前步骤需要的功能逻辑。
        NewQ.Normalize();
        // 解释：调用 `SetQuaternion` 执行当前步骤需要的功能逻辑。
        CurrentState.SetQuaternion(NewQ);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // --- 计算新位置处的力和力矩 ---
    // 解释：这一行声明成员或局部变量 `Torque`，用于保存torque。
    FVector Force, Torque;
    // 解释：调用 `CalculateTotalForcesAndTorques` 执行当前步骤需要的功能逻辑。
    CalculateTotalForcesAndTorques(CurrentState, Force, Torque);

    // --- 线加速度：a(t+dt) = F/m ---
    // 解释：这一行把右侧表达式的结果写入 `FVector NewLinearAcc`，完成 fvectornewlinearacc 的更新。
    FVector NewLinearAcc = Force / Parameters.Mass + FVector(0.0, 0.0, -Parameters.Gravity);

    // --- 角加速度：欧拉旋转方程 ---
    // 解释：这一行声明成员或局部变量 `Jx`，用于保存jx。
    double Jx = Parameters.Jx, Jy = Parameters.Jy, Jz = Parameters.Jz;
    // 解释：这一行声明成员或局部变量 `p`，用于保存P。
    double p = CurrentState.AngRollRate;
    // 解释：这一行声明成员或局部变量 `q`，用于保存Q。
    double q = CurrentState.AngPitchRate;
    // 解释：这一行声明成员或局部变量 `r`，用于保存R。
    double r = CurrentState.AngYawRate;
    // 解释：这一行声明成员或局部变量 `AngMomentumRate`，用于保存angmomentumrate。
    FVector AngMomentumRate;
    // 解释：这一行把右侧表达式的结果写入 `AngMomentumRate.X`，完成 状态向量 X 的更新。
    AngMomentumRate.X = (Torque.X - (Jz - Jy) * q * r) / Jx;
    // 解释：这一行把右侧表达式的结果写入 `AngMomentumRate.Y`，完成 Y 的更新。
    AngMomentumRate.Y = (Torque.Y - (Jx - Jz) * p * r) / Jy;
    // 解释：这一行把右侧表达式的结果写入 `AngMomentumRate.Z`，完成 Z 的更新。
    AngMomentumRate.Z = (Torque.Z - (Jy - Jx) * p * q) / Jz;

    // --- 速度更新：v(t+dt) = v(t) + 0.5*(a(t) + a(t+dt))*dt ---
    // 解释：这一行把右侧表达式的结果写入 `FVector NewVel`，完成 fvectornewvel 的更新。
    FVector NewVel = Vel + (PrevLinearAcceleration + NewLinearAcc) * halfDt;
    // 解释：调用 `SetVelocity` 执行当前步骤需要的功能逻辑。
    CurrentState.SetVelocity(NewVel);

    // 解释：这一行把右侧表达式的结果写入 `FVector NewAngVel`，完成 fvectornewangvel 的更新。
    FVector NewAngVel = AngVel + (PrevAngularAcceleration + AngMomentumRate) * halfDt;
    // 解释：这一行把右侧表达式的结果写入 `CurrentState.AngRollRate`，完成 angrollrate 的更新。
    CurrentState.AngRollRate = NewAngVel.X;
    // 解释：这一行把右侧表达式的结果写入 `CurrentState.AngPitchRate`，完成 angpitchrate 的更新。
    CurrentState.AngPitchRate = NewAngVel.Y;
    // 解释：这一行把右侧表达式的结果写入 `CurrentState.AngYawRate`，完成 angyawrate 的更新。
    CurrentState.AngYawRate = NewAngVel.Z;

    // --- 保存加速度供下一帧使用 ---
    // 解释：这一行把右侧表达式的结果写入 `PrevLinearAcceleration`，完成 prevlinearacceleration 的更新。
    PrevLinearAcceleration = NewLinearAcc;
    // 解释：这一行把右侧表达式的结果写入 `PrevAngularAcceleration`，完成 prevangularacceleration 的更新。
    PrevAngularAcceleration = AngMomentumRate;

    // --- 四元数归一化 ---
    // 解释：调用 `NormalizeQuaternion` 执行当前步骤需要的功能逻辑。
    CurrentState.NormalizeQuaternion();
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 计算给定状态下的合外力和合力矩
 * @param State 当前状态
 * @param OutForce 世界坐标系合外力（不含重力，重力在 Verlet 中单独处理）
 * @param OutTorque 机体坐标系合力矩
 */
// 解释：这一行定义函数 `CalculateTotalForcesAndTorques`，开始实现calculatetotalforcesandtorques的具体逻辑。
void UDroneMovementComponent::CalculateTotalForcesAndTorques(const FDroneState& State, FVector& OutForce, FVector& OutTorque)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `OutForce`，完成 outforce 的更新。
    OutForce = FVector::ZeroVector;
    // 解释：这一行把右侧表达式的结果写入 `OutTorque`，完成 outtorque 的更新。
    OutTorque = FVector::ZeroVector;

    // 解释：这一行声明成员或局部变量 `TotalThrust`，用于保存totalthrust。
    double TotalThrust = 0.0;
    // 解释：这一行声明成员或局部变量 `TotalTorque`，用于保存totaltorque。
    FVector TotalTorque = FVector::ZeroVector;

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (State.MotorSpeeds.Num() >= 4)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        TArray<double> OmegaSq = {
            // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
            State.MotorSpeeds[0] * State.MotorSpeeds[0],
            // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
            State.MotorSpeeds[1] * State.MotorSpeeds[1],
            // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
            State.MotorSpeeds[2] * State.MotorSpeeds[2],
            // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
            State.MotorSpeeds[3] * State.MotorSpeeds[3]
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        };
        // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
        for (int32 i = 0; i < 4; ++i)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行在 `TotalThrust` 的原有基础上继续累加新量，用于持续更新 totalthrust。
            TotalThrust += G[0][i] * OmegaSq[i];
            // 解释：这一行在 `TotalTorque.X` 的原有基础上继续累加新量，用于持续更新 状态向量 X。
            TotalTorque.X += G[1][i] * OmegaSq[i];
            // 解释：这一行在 `TotalTorque.Y` 的原有基础上继续累加新量，用于持续更新 Y。
            TotalTorque.Y += G[2][i] * OmegaSq[i];
            // 解释：这一行在 `TotalTorque.Z` 的原有基础上继续累加新量，用于持续更新 Z。
            TotalTorque.Z += G[3][i] * OmegaSq[i];
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 推力向量从机体 Z 轴转到世界坐标系
    // 解释：调用 `ThrustVectorBody` 执行当前步骤需要的功能逻辑。
    FVector ThrustVectorBody(0.0, 0.0, TotalThrust);
    // 解释：这一行把右侧表达式的结果写入 `FQuat Orientation`，完成 fquatorientation 的更新。
    FQuat Orientation = State.GetQuaternion();
    // 解释：这一行把右侧表达式的结果写入 `FVector ThrustVectorWorld`，完成 fvectorthrustvectorworld 的更新。
    FVector ThrustVectorWorld = RotateBodyToWorld(ThrustVectorBody, Orientation);

    // 阻力（简化模型，Phase 4 将升级为各向异性）
    // 解释：这一行把右侧表达式的结果写入 `FVector Velocity`，完成 fvectorvelocity 的更新。
    FVector Velocity = State.GetVelocity();
    // 解释：这一行把向量模长写入 `FVector Drag`，用于表示距离、速度大小或不确定度。
    FVector Drag = -Parameters.DragCoefficient * Velocity.Size() * Velocity;

    // 解释：这一行把右侧表达式的结果写入 `OutForce`，完成 outforce 的更新。
    OutForce = ThrustVectorWorld + Drag;
    // 解释：这一行把右侧表达式的结果写入 `OutTorque`，完成 outtorque 的更新。
    OutTorque = TotalTorque;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 检查并处理地面碰撞
 * @param GroundZ 地面 Z 坐标（仿真坐标系）
 * 当无人机位置低于地面且向下运动时：
 * - 将位置钦定到地面
 * - 清除向下速度
 * - 使用恢复系数处理弹跳
 * - Ground Lock：低速着陆时锁定姿态
 */
// 解释：这一行定义函数 `CheckGroundCollision`，开始实现checkgroundcollision的具体逻辑。
void UDroneMovementComponent::CheckGroundCollision(double GroundZ)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (CurrentState.Z <= GroundZ && CurrentState.Vz < 0.0)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `CurrentState.Z`，完成 Z 的更新。
        CurrentState.Z = GroundZ;

        // 判断是否为低速着陆（垂直速度占主导）
        // 解释：这一行把右侧表达式的结果写入 `double SpeedXY`，完成 doublespeedxy 的更新。
        double SpeedXY = FMath::Sqrt(CurrentState.Vx * CurrentState.Vx + CurrentState.Vy * CurrentState.Vy);
        // 解释：这一行把右侧表达式的结果写入 `bool bLanding`，完成 boolBlanding 的更新。
        bool bLanding = FMath::Abs(CurrentState.Vz) > SpeedXY;

        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (bLanding && FMath::Abs(CurrentState.Vz) < 1.0)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // Ground Lock：低速着陆，锁定在地面
            // 解释：这一行把右侧表达式的结果写入 `bGrounded`，完成 布尔标志 grounded 的更新。
            bGrounded = true;
            // 解释：这一行把右侧表达式的结果写入 `CurrentState.Vx`，完成 vx 的更新。
            CurrentState.Vx = 0.0;
            // 解释：这一行把右侧表达式的结果写入 `CurrentState.Vy`，完成 vy 的更新。
            CurrentState.Vy = 0.0;
            // 解释：这一行把右侧表达式的结果写入 `CurrentState.Vz`，完成 vz 的更新。
            CurrentState.Vz = 0.0;
            // 解释：这一行把右侧表达式的结果写入 `CurrentState.AngRollRate`，完成 angrollrate 的更新。
            CurrentState.AngRollRate = 0.0;
            // 解释：这一行把右侧表达式的结果写入 `CurrentState.AngPitchRate`，完成 angpitchrate 的更新。
            CurrentState.AngPitchRate = 0.0;
            // 解释：这一行把右侧表达式的结果写入 `CurrentState.AngYawRate`，完成 angyawrate 的更新。
            CurrentState.AngYawRate = 0.0;

            // 矫正姿态：Roll/Pitch 归零，保留 Yaw
            // 解释：这一行把右侧表达式的结果写入 `FRotator Rot`，完成 frotatorrot 的更新。
            FRotator Rot = CurrentState.GetRotator();
            // 解释：这一行把右侧表达式的结果写入 `Rot.Roll`，完成 roll 的更新。
            Rot.Roll = 0.0f;
            // 解释：这一行把右侧表达式的结果写入 `Rot.Pitch`，完成 pitch 的更新。
            Rot.Pitch = 0.0f;
            // 解释：调用 `SetQuaternion` 执行当前步骤需要的功能逻辑。
            CurrentState.SetQuaternion(Rot.Quaternion());

            // 解释：这一行把右侧表达式的结果写入 `PrevLinearAcceleration`，完成 prevlinearacceleration 的更新。
            PrevLinearAcceleration = FVector::ZeroVector;
            // 解释：这一行把右侧表达式的结果写入 `PrevAngularAcceleration`，完成 prevangularacceleration 的更新。
            PrevAngularAcceleration = FVector::ZeroVector;

            // 解释：调用 `UE_LOG` 输出调试日志，便于运行时观察状态和排查问题。
            UE_LOG(LogTemp, Log, TEXT("[DroneMovement] Ground locked"));
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
        // 解释：这一行给出兜底分支，当上面的条件都不满足时执行。
        else
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 弹跳响应：反弹垂直速度
            // 解释：这一行声明成员或局部变量 `Restitution`，用于保存restitution。
            double Restitution = Parameters.Restitution;
            // 解释：这一行把右侧表达式的结果写入 `CurrentState.Vz`，完成 vz 的更新。
            CurrentState.Vz = -CurrentState.Vz * Restitution;

            // 摩擦衰减水平速度
            // 解释：这一行声明成员或局部变量 `FrictionFactor`，用于保存frictionfactor。
            double FrictionFactor = 1.0 - Parameters.Friction * 0.5;
            // 解释：这一行把右侧表达式的结果写入 `CurrentState.Vx`，完成 vx 的更新。
            CurrentState.Vx *= FrictionFactor;
            // 解释：这一行把右侧表达式的结果写入 `CurrentState.Vy`，完成 vy 的更新。
            CurrentState.Vy *= FrictionFactor;

            // 角速度衰减
            // 解释：这一行把右侧表达式的结果写入 `CurrentState.AngRollRate`，完成 angrollrate 的更新。
            CurrentState.AngRollRate *= 0.9;
            // 解释：这一行把右侧表达式的结果写入 `CurrentState.AngPitchRate`，完成 angpitchrate 的更新。
            CurrentState.AngPitchRate *= 0.9;
            // 解释：这一行把右侧表达式的结果写入 `CurrentState.AngYawRate`，完成 angyawrate 的更新。
            CurrentState.AngYawRate *= 0.9;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 位置控制环 PD控制器
 * @param PositionCommand 目标位置
 * @return 速度命令
 */
// 解释：这一行定义函数 `PositionLoop`，开始实现positionloop的具体逻辑。
FVector UDroneMovementComponent::PositionLoop(const FVector& PositionCommand)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `FVector Pos`，完成 fvectorpos 的更新。
    FVector Pos = CurrentState.GetPosition();
    // 解释：这一行把右侧表达式的结果写入 `double VxCmd`，完成 doublevxcmd 的更新。
    double VxCmd = PxController ? PxController->Update(PositionCommand.X, Pos.X) : 0.0;
    // 解释：这一行把右侧表达式的结果写入 `double VyCmd`，完成 doublevycmd 的更新。
    double VyCmd = PyController ? PyController->Update(PositionCommand.Y, Pos.Y) : 0.0;
    // 解释：这一行把右侧表达式的结果写入 `double VzCmd`，完成 doublevzcmd 的更新。
    double VzCmd = PzController ? PzController->Update(PositionCommand.Z, Pos.Z) : 0.0;
    // 解释：调用 `VelocityCommand` 执行当前步骤需要的功能逻辑。
    FVector VelocityCommand(VxCmd, VyCmd, VzCmd);
    // 解释：这一行通过 `FMath::Max` 给 `const double MaxSpeed` 施加下界约束，避免 constdoublemaxspeed 过小。
    const double MaxSpeed = FMath::Max(0.1, TargetPositionSpeedLimit);
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (VelocityCommand.SizeSquared() > MaxSpeed * MaxSpeed)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `VelocityCommand`，完成 velocity命令 的更新。
        VelocityCommand = VelocityCommand.GetSafeNormal() * MaxSpeed;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return VelocityCommand;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 速度控制环 PID控制器
 * @param VelocityCommand 目标速度
 * @param OutThrust 输出推力
 * @return 加速度命令
 * Z 轴加速度命令经过重力补偿：Thrust = m * (Az + g)
 * 倾斜补偿：实际推力 = Thrust / (cosRoll * cosPitch)
 */
// 解释：这一行定义函数 `VelocityLoop`，开始实现velocityloop的具体逻辑。
FVector UDroneMovementComponent::VelocityLoop(const FVector& VelocityCommand, double& OutThrust)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `FVector Vel`，完成 fvectorvel 的更新。
    FVector Vel = CurrentState.GetVelocity();
    // 解释：这一行把右侧表达式的结果写入 `double AxCmd`，完成 doubleaxcmd 的更新。
    double AxCmd = VxController ? VxController->Update(VelocityCommand.X, Vel.X) : 0.0;
    // 解释：这一行把右侧表达式的结果写入 `double AyCmd`，完成 doubleaycmd 的更新。
    double AyCmd = VyController ? VyController->Update(VelocityCommand.Y, Vel.Y) : 0.0;
    // 解释：这一行把右侧表达式的结果写入 `double AzCmd`，完成 doubleazcmd 的更新。
    double AzCmd = VzController ? VzController->Update(VelocityCommand.Z, Vel.Z) : 0.0;
    // 解释：这一行把右侧表达式的结果写入 `OutThrust`，完成 outthrust 的更新。
    OutThrust = Parameters.Mass * (AzCmd + Parameters.Gravity);
    // 解释：这一行通过 `FMath::Max` 给 `OutThrust` 施加下界约束，避免 outthrust 过小。
    OutThrust = FMath::Max(0.0, OutThrust);
    // 解释：这一行把右侧表达式的结果写入 `FRotator CurrentRot`，完成 frotatorcurrentrot 的更新。
    FRotator CurrentRot = CurrentState.GetRotator();
    // 解释：这一行把右侧表达式的结果写入 `double CosRoll`，完成 doublecosroll 的更新。
    double CosRoll = FMath::Cos(FMath::DegreesToRadians(CurrentRot.Roll));
    // 解释：这一行把右侧表达式的结果写入 `double CosPitch`，完成 doublecospitch 的更新。
    double CosPitch = FMath::Cos(FMath::DegreesToRadians(CurrentRot.Pitch));
    // 解释：这一行声明成员或局部变量 `CosAttitude`，用于保存cosattitude。
    double CosAttitude = CosRoll * CosPitch;
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (CosAttitude > 0.5) OutThrust /= CosAttitude;
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return FVector(AxCmd, AyCmd, AzCmd);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 姿态控制环 PD控制器
 * @param AccelerationCommand 期望加速度
 * @return 角速率命令
 * 从期望水平加速度计算期望倾斜角度：
 *   RollDes = atan2(-Ay, g)  —— 右移需要右倾
 *   PitchDes = atan2(Ax, g)  —— 前进需要低头
 */
// 解释：这一行定义函数 `AttitudeLoop`，开始实现attitudeloop的具体逻辑。
FVector UDroneMovementComponent::AttitudeLoop(const FVector& AccelerationCommand)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行声明成员或局部变量 `AxDes`，用于保存axdes。
    double AxDes = AccelerationCommand.X;
    // 解释：这一行声明成员或局部变量 `AyDes`，用于保存aydes。
    double AyDes = AccelerationCommand.Y;
    // 解释：这一行声明成员或局部变量 `g`，用于保存G。
    double g = Parameters.Gravity;

    // 从期望水平加速度计算期望倾斜角
    // 解释：这一行把右侧表达式的结果写入 `double RollDes`，完成 doublerolldes 的更新。
    double RollDes = FMath::Atan2(-AyDes, g);
    // 解释：这一行把右侧表达式的结果写入 `double PitchDes`，完成 doublepitchdes 的更新。
    double PitchDes = FMath::Atan2(AxDes, g);

    // 自动偏航角已在 ControlUpdate 中从目标方向计算好（存在 DesiredYaw）
    // 解释：这一行声明成员或局部变量 `YawDes`，用于保存yawdes。
    double YawDes = DesiredYaw;

    // 限制最大倾斜角（约12°，比 AirSim 默认的 ~15° 稍保守，确保平滑）
    // 解释：这一行先对计算结果做限幅，再写入 `RollDes`，防止 rolldes 超出允许范围。
    RollDes = FMath::Clamp(RollDes, -0.21, 0.21);
    // 解释：这一行先对计算结果做限幅，再写入 `PitchDes`，防止 pitchdes 超出允许范围。
    PitchDes = FMath::Clamp(PitchDes, -0.21, 0.21);

    // UE FRotator 的 Roll/Pitch 正方向与控制分配矩阵 G 的力矩符号相反
    // 取反使 PD 控制器的误差方向与实际力矩方向一致
    // 解释：这一行把右侧表达式的结果写入 `FRotator CurrentRot`，完成 frotatorcurrentrot 的更新。
    FRotator CurrentRot = CurrentState.GetRotator();
    // 解释：这一行把右侧表达式的结果写入 `double Roll`，完成 doubleroll 的更新。
    double Roll = -FMath::DegreesToRadians(CurrentRot.Roll);
    // 解释：这一行把右侧表达式的结果写入 `double Pitch`，完成 doublepitch 的更新。
    double Pitch = -FMath::DegreesToRadians(CurrentRot.Pitch);
    // 解释：这一行把右侧表达式的结果写入 `double Yaw`，完成 doubleyaw 的更新。
    double Yaw = FMath::DegreesToRadians(CurrentRot.Yaw);

    // 解释：这一行把右侧表达式的结果写入 `double RollTarget`，完成 doublerolltarget 的更新。
    double RollTarget = Roll + NormalizeAngle(RollDes - Roll);
    // 解释：这一行把右侧表达式的结果写入 `double PitchTarget`，完成 doublepitchtarget 的更新。
    double PitchTarget = Pitch + NormalizeAngle(PitchDes - Pitch);
    // 解释：这一行把右侧表达式的结果写入 `double YawTarget`，完成 doubleyawtarget 的更新。
    double YawTarget = Yaw + NormalizeAngle(YawDes - Yaw);

    // 解释：这一行把右侧表达式的结果写入 `double RollRateCmd`，完成 doublerollratecmd 的更新。
    double RollRateCmd = RollController ? RollController->Update(RollTarget, Roll) : 0.0;
    // 解释：这一行把右侧表达式的结果写入 `double PitchRateCmd`，完成 doublepitchratecmd 的更新。
    double PitchRateCmd = PitchController ? PitchController->Update(PitchTarget, Pitch) : 0.0;
    // 解释：这一行把右侧表达式的结果写入 `double YawRateCmd`，完成 doubleyawratecmd 的更新。
    double YawRateCmd = YawController ? YawController->Update(YawTarget, Yaw) : 0.0;
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return FVector(RollRateCmd, PitchRateCmd, YawRateCmd);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 角速率控制环 PID控制器
 * @param AngularVelocityCommand 目标角速率
 * @return 力矩命令
 */
// 解释：这一行定义函数 `AngularVelocityLoop`，开始实现angularvelocityloop的具体逻辑。
FVector UDroneMovementComponent::AngularVelocityLoop(const FVector& AngularVelocityCommand)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `double TorqueX`，完成 doubletorqueX 的更新。
    double TorqueX = RollRateController ? RollRateController->Update(AngularVelocityCommand.X, CurrentState.AngRollRate) : 0.0;
    // 解释：这一行把右侧表达式的结果写入 `double TorqueY`，完成 doubletorqueY 的更新。
    double TorqueY = PitchRateController ? PitchRateController->Update(AngularVelocityCommand.Y, CurrentState.AngPitchRate) : 0.0;
    // 解释：这一行把右侧表达式的结果写入 `double TorqueZ`，完成 doubletorqueZ 的更新。
    double TorqueZ = YawRateController ? YawRateController->Update(AngularVelocityCommand.Z, CurrentState.AngYawRate) : 0.0;
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return FVector(TorqueX, TorqueY, TorqueZ);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 设置初始状态
 * @param InitialState 初始状态
 * 记录起始高度用于地面碰撞检测，初始化电机滤波器
 */
// 解释：这一行定义函数 `SetInitialState`，开始实现setinitial状态的具体逻辑。
void UDroneMovementComponent::SetInitialState(const FDroneState& InitialState)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `CurrentState`，完成 current状态 的更新。
    CurrentState = InitialState;
    // 解释：这一行把右侧表达式的结果写入 `double HoverSpeed`，完成 doublehoverspeed 的更新。
    double HoverSpeed = Parameters.GetHoverMotorSpeed();
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (CurrentState.MotorSpeeds.Num() < 4)
        // 解释：这一行把右侧表达式的结果写入 `CurrentState.MotorSpeeds`，完成 motorspeeds 的更新。
        CurrentState.MotorSpeeds = {HoverSpeed, HoverSpeed, HoverSpeed, HoverSpeed};

    // 记录起始地面高度：用初始 Z 位置作为地面参考
    // 无人机初始生成在地面上，起始为 Ground Lock 状态
    // 解释：这一行把右侧表达式的结果写入 `InitialGroundZ`，完成 initialgroundZ 的更新。
    InitialGroundZ = CurrentState.Z;
    // 解释：这一行把右侧表达式的结果写入 `bGrounded`，完成 布尔标志 grounded 的更新。
    bGrounded = true;

    // 初始化电机滤波器到悬停转速
    // 解释：这一行把右侧表达式的结果写入 `MotorSpeedsFiltered`，完成 motorspeedsfiltered 的更新。
    MotorSpeedsFiltered = CurrentState.MotorSpeeds;

    // 清零加速度历史
    // 解释：这一行把右侧表达式的结果写入 `PrevLinearAcceleration`，完成 prevlinearacceleration 的更新。
    PrevLinearAcceleration = FVector::ZeroVector;
    // 解释：这一行把右侧表达式的结果写入 `PrevAngularAcceleration`，完成 prevangularacceleration 的更新。
    PrevAngularAcceleration = FVector::ZeroVector;
    // 解释：这一行把右侧表达式的结果写入 `FixedStepAccumulator`，完成 fixedstepaccumulator 的更新。
    FixedStepAccumulator = 0.0;

    // 重置自动偏航状态
    // 解释：这一行把右侧表达式的结果写入 `LockedYaw`，完成 lockedyaw 的更新。
    LockedYaw = FMath::DegreesToRadians(CurrentState.GetRotator().Yaw);
    // 解释：这一行把右侧表达式的结果写入 `CommandedYaw`，完成 commandedyaw 的更新。
    CommandedYaw = LockedYaw;
    // 解释：这一行把右侧表达式的结果写入 `bYawInitialized`，完成 布尔标志 yawinitialized 的更新。
    bYawInitialized = false;

    // 解释：这一行位于构造函数初始化列表中，把 `UE_LOG` 直接初始化为 `LogTemp, Log, TEXT("[DroneMovement] InitialState set, HoverSpeed=%.1f rad/s, GroundZ=%.2f, Grounded=true"`，减少进入函数体后的额外赋值开销。
    UE_LOG(LogTemp, Log, TEXT("[DroneMovement] InitialState set, HoverSpeed=%.1f rad/s, GroundZ=%.2f, Grounded=true"),
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        HoverSpeed, InitialGroundZ);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 切换控制模式并重置所有控制器
 * @param NewMode 新的控制模式
 */
// 解释：这一行定义函数 `SetControlMode`，开始实现setcontrol模式的具体逻辑。
void UDroneMovementComponent::SetControlMode(EDroneControlMode NewMode)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `CurrentControlMode`，完成 currentcontrol模式 的更新。
    CurrentControlMode = NewMode;
    // 解释：调用 `ResetAllControllers` 执行当前步骤需要的功能逻辑。
    ResetAllControllers();
    // 解释：这一行把右侧表达式的结果写入 `FixedStepAccumulator`，完成 fixedstepaccumulator 的更新。
    FixedStepAccumulator = 0.0;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 设置原始控制命令 */
// 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
void UDroneMovementComponent::SetControlCommand(const TArray<double>& Command) { ControlCommands = Command; }

/** @brief 设置目标位置 */
// 解释：这一行定义函数 `SetTargetPosition`，开始实现settargetposition的具体逻辑。
void UDroneMovementComponent::SetTargetPosition(const FVector& TargetPos, float Speed)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `TargetPosition`，完成 targetposition 的更新。
    TargetPosition = TargetPos;

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (Speed > KINDA_SMALL_NUMBER)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行通过 `FMath::Max` 给 `PositionAxisSpeedLimit` 施加下界约束，避免 positionaxisspeedlimit 过小。
        PositionAxisSpeedLimit = FMath::Max(0.1, static_cast<double>(Speed));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行给出兜底分支，当上面的条件都不满足时执行。
    else
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `PositionAxisSpeedLimit`，完成 positionaxisspeedlimit 的更新。
        PositionAxisSpeedLimit = DefaultPositionSpeedLimit;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行把右侧表达式的结果写入 `TargetPositionSpeedLimit`，完成 targetpositionspeedlimit 的更新。
    TargetPositionSpeedLimit = PositionAxisSpeedLimit;

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (PxController) PxController->SetParameters(PxController->Kp, PxController->Kd, PxController->DiffFilterTau, PositionAxisSpeedLimit);
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (PyController) PyController->SetParameters(PyController->Kp, PyController->Kd, PyController->DiffFilterTau, PositionAxisSpeedLimit);
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (PzController) PzController->SetParameters(PzController->Kp, PzController->Kd, PzController->DiffFilterTau, PositionAxisSpeedLimit);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 设置目标速度 */
// 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
void UDroneMovementComponent::SetTargetVelocity(const FVector& TargetVel) { TargetVelocity = TargetVel; }

/**
 * @brief 设置目标姿态和推力
 * @param Attitude 目标姿态
 * @param Thrust 推力系数
 */
// 解释：这一行定义函数 `SetTargetAttitude`，开始实现settargetattitude的具体逻辑。
void UDroneMovementComponent::SetTargetAttitude(const FRotator& Attitude, float Thrust)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `TargetAttitude`，完成 targetattitude 的更新。
    TargetAttitude = Attitude;
    // 解释：这一行把右侧表达式的结果写入 `TargetThrust`，完成 targetthrust 的更新。
    TargetThrust = Thrust * Parameters.Mass * Parameters.Gravity;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 重置状态并清零所有控制器
 * @param NewState 新的初始状态
 */
// 解释：这一行定义函数 `ResetState`，开始实现reset状态的具体逻辑。
void UDroneMovementComponent::ResetState(const FDroneState& NewState)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `CurrentState`，完成 current状态 的更新。
    CurrentState = NewState;
    // 解释：调用 `ResetAllControllers` 执行当前步骤需要的功能逻辑。
    ResetAllControllers();
    // 解释：这一行把右侧表达式的结果写入 `FixedStepAccumulator`，完成 fixedstepaccumulator 的更新。
    FixedStepAccumulator = 0.0;

    // 重置自动偏航状态
    // 解释：这一行把右侧表达式的结果写入 `LockedYaw`，完成 lockedyaw 的更新。
    LockedYaw = FMath::DegreesToRadians(NewState.GetRotator().Yaw);
    // 解释：这一行把右侧表达式的结果写入 `CommandedYaw`，完成 commandedyaw 的更新。
    CommandedYaw = LockedYaw;
    // 解释：这一行把右侧表达式的结果写入 `bYawInitialized`，完成 布尔标志 yawinitialized 的更新。
    bYawInitialized = false;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 重置所有 12 个控制器的内部状态 */
// 解释：这一行定义函数 `ResetAllControllers`，开始实现resetallcontrollers的具体逻辑。
void UDroneMovementComponent::ResetAllControllers()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (PxController) PxController->Reset();
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (PyController) PyController->Reset();
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (PzController) PzController->Reset();
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (VxController) VxController->Reset();
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (VyController) VyController->Reset();
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (VzController) VzController->Reset();
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (RollController) RollController->Reset();
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (PitchController) PitchController->Reset();
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (YawController) YawController->Reset();
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (RollRateController) RollRateController->Reset();
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (PitchRateController) PitchRateController->Reset();
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (YawRateController) YawRateController->Reset();
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 设置位置控制器 PD 增益
 * @param Kp 比例增益
 * @param Kd 微分增益
 */
// 解释：这一行定义函数 `SetPositionGains`，开始实现setpositiongains的具体逻辑。
void UDroneMovementComponent::SetPositionGains(float Kp, float Kd)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (PxController) PxController->SetParameters(Kp, Kd, 0.0, PositionAxisSpeedLimit);
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (PyController) PyController->SetParameters(Kp, Kd, 0.0, PositionAxisSpeedLimit);
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (PzController) PzController->SetParameters(Kp, Kd, 0.0, PositionAxisSpeedLimit);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 设置速度控制器 PID 增益
 * @param Kp 比例增益
 * @param Ki 积分增益
 * @param Kd 微分增益
 */
// 解释：这一行定义函数 `SetVelocityGains`，开始实现setvelocitygains的具体逻辑。
void UDroneMovementComponent::SetVelocityGains(float Kp, float Ki, float Kd)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (VxController) VxController->SetParameters(Kp, Ki, Kd, 0.05, 3.0);
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (VyController) VyController->SetParameters(Kp, Ki, Kd, 0.05, 3.0);
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (VzController) VzController->SetParameters(Kp, Ki, Kd, 0.05, 3.0);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 设置姿态控制器 PD 增益
 * @param Kp 比例增益
 * @param Kd 微分增益
 */
// 解释：这一行定义函数 `SetAttitudeGains`，开始实现setattitudegains的具体逻辑。
void UDroneMovementComponent::SetAttitudeGains(float Kp, float Kd)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (RollController) RollController->SetParameters(Kp, Kd, 0.0, 3.0);
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (PitchController) PitchController->SetParameters(Kp, Kd, 0.0, 3.0);
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (YawController) YawController->SetParameters(Kp * 0.5f, Kd, 0.0, 3.0);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 设置角速率控制器增益
 * @param Kp 比例增益
 */
// 解释：这一行定义函数 `SetAngleRateGains`，开始实现setanglerategains的具体逻辑。
void UDroneMovementComponent::SetAngleRateGains(float Kp)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (RollRateController) RollRateController->SetParameters(Kp, 0, 0, 0.05, 1.0);
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (PitchRateController) PitchRateController->SetParameters(Kp, 0, 0, 0.05, 1.0);
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (YawRateController) YawRateController->SetParameters(Kp * 0.5f, 0, 0, 0.05, 0.5);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

// 解释：这一行定义函数 `SetHeadingControl`，开始实现setheadingcontrol的具体逻辑。
void UDroneMovementComponent::SetHeadingControl(EDroneYawMode NewYawMode, EDroneDrivetrainMode NewDrivetrain, float YawDeg)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `YawMode`，完成 yaw模式 的更新。
    YawMode = NewYawMode;
    // 解释：这一行把右侧表达式的结果写入 `DrivetrainMode`，完成 drivetrain模式 的更新。
    DrivetrainMode = NewDrivetrain;

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (YawMode == EDroneYawMode::Angle)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `CommandedYaw`，完成 commandedyaw 的更新。
        CommandedYaw = FMath::DegreesToRadians(YawDeg);
        // 解释：这一行把右侧表达式的结果写入 `LockedYaw`，完成 lockedyaw 的更新。
        LockedYaw = CommandedYaw;
        // 解释：这一行把右侧表达式的结果写入 `bYawInitialized`，完成 布尔标志 yawinitialized 的更新。
        bYawInitialized = true;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行声明补充分支，当上一个条件不成立且当前条件成立时执行。
    else if (YawMode == EDroneYawMode::Hold)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `LockedYaw`，完成 lockedyaw 的更新。
        LockedYaw = FMath::DegreesToRadians(CurrentState.GetRotator().Yaw);
        // 解释：这一行把右侧表达式的结果写入 `bYawInitialized`，完成 布尔标志 yawinitialized 的更新。
        bYawInitialized = true;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 将机体坐标系向量旋转到世界坐标系
 * @param BodyVector 机体坐标系向量
 * @param Orientation 姿态四元数
 * @return 世界坐标系向量
 */
// 解释：这一行定义函数 `RotateBodyToWorld`，开始实现rotatebodytoworld的具体逻辑。
FVector UDroneMovementComponent::RotateBodyToWorld(const FVector& BodyVector, const FQuat& Orientation)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return Orientation.RotateVector(BodyVector);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 角度归一化到 [-π, π] 范围
 * @param AngleRad 输入角度
 * @return 归一化后的角度
 */
// 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
double UDroneMovementComponent::NormalizeAngle(double AngleRad)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行开始 `while` 循环，只要条件保持为真就持续重复执行。
    while (AngleRad > PI) AngleRad -= 2.0 * PI;
    // 解释：这一行开始 `while` 循环，只要条件保持为真就持续重复执行。
    while (AngleRad < -PI) AngleRad += 2.0 * PI;
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return AngleRad;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

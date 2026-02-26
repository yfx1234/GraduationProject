/**
 * @file DronePawn.cpp
 * @brief 无人机 Pawn 的实现文件
 *
 * 实现 ADronePawn 的组件创建、状态同步、控制接口委托和螺旋桨动画逻辑。
 */

#include "DronePawn.h"
#include "DroneMovementComponent.h"
#include "DroneApi.h"
#include "GraduationProject/Core/Manager/AgentManager.h"
#include "Camera/CameraComponent.h"

/**
 * @brief 构造函数
 *
 * 创建组件层级：
 * RootComp → BodyMesh → Fan0~Fan3 (Socket 挂载) + FPVCamera + MovementComp
 */
ADronePawn::ADronePawn()
{
    PrimaryActorTick.bCanEverTick = true;

    // 根场景组件
    RootComp = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
    SetRootComponent(RootComp);

    // 机身静态网格（Blueprint 中配置具体 StaticMesh）
    BodyMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("BodyMesh"));
    BodyMesh->SetupAttachment(RootComp);

    // 4 个螺旋桨 — 挂载到 BodyMesh 的骨骼 Socket
    Fan0 = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Fan0"));
    Fan0->SetupAttachment(BodyMesh, TEXT("Drone_Fan_002"));
    Fan1 = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Fan1"));
    Fan1->SetupAttachment(BodyMesh, TEXT("Drone_Fan1_002"));
    Fan2 = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Fan2"));
    Fan2->SetupAttachment(BodyMesh, TEXT("Drone_Fan2_002"));
    Fan3 = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Fan3"));
    Fan3->SetupAttachment(BodyMesh, TEXT("Drone_Fan3_002"));

    // FPV 相机 — 挂载到相机插槽
    FPVCamera = CreateDefaultSubobject<UCameraComponent>(TEXT("FPVCamera"));
    FPVCamera->SetupAttachment(BodyMesh, TEXT("Camera_Yaw_002"));

    // 运动仿真组件（包含物理模型和控制器）
    MovementComp = CreateDefaultSubobject<UDroneMovementComponent>(TEXT("Movement"));
}

/**
 * @brief 游戏开始时初始化
 *
 * 流程：
 * 1. 将物理参数传给 MovementComp
 * 2. 从 Actor 的 UE 位置转换为 SI 单位设置初始状态
 * 3. 创建 DroneApi 实例
 * 4. 注册到 AgentManager
 */
void ADronePawn::BeginPlay()
{
    Super::BeginPlay();

    // 初始化运动组件
    if (MovementComp)
    {
        // 传递物理参数
        MovementComp->SetParameters(Parameters);

        // 从 Actor 位置读取初始状态（UE cm → SI m）
        FDroneState InitState;
        FVector Pos = GetActorLocation() / 100.0f; // UE cm -> SI m
        InitState.SetPosition(Pos);
        MovementComp->SetInitialState(InitState);
        MovementComp->SetControlMode(ControlMode);
    }

    // 创建 API 接口
    Api = NewObject<UDroneApi>(this);
    Api->Initialize(this);

    // 注册到全局智能体管理器
    UAgentManager::GetInstance()->RegisterAgent(DroneId, this);

    UE_LOG(LogTemp, Log, TEXT("[DronePawn] BeginPlay: %s at %s"),
        *DroneId, *GetActorLocation().ToString());
}

/**
 * @brief 每帧更新
 * @param DeltaTime 帧间隔时间
 *
 * MovementComp 在自己的 TickComponent 中完成物理仿真，
 * 此处读取最新状态并同步到 UE Actor Transform。
 */
void ADronePawn::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    // 从 MovementComp 读取最新的物理状态
    if (MovementComp)
    {
        CurrentState = MovementComp->GetCurrentState();
    }

    // 将物理状态同步到 Actor 的 Transform（SI → UE 单位转换）
    ApplyStateToActor(CurrentState);

    // 更新螺旋桨旋转动画
    UpdatePropellerAnimation(DeltaTime);
}

/**
 * @brief 将仿真状态同步到 UE Actor Transform
 * @param State 无人机当前状态
 *
 * 坐标转换：SI 米 × 100 = UE 厘米
 * 姿态转换：四元数 → FRotator
 */
void ADronePawn::ApplyStateToActor(const FDroneState& State)
{
    // SI (m) -> UE (cm)
    FVector UEPos = State.GetPosition() * 100.0f;
    FRotator UERot = State.GetRotator();
    SetActorLocationAndRotation(UEPos, UERot);
}

/**
 * @brief 更新螺旋桨旋转动画
 * @param DeltaTime 帧间隔（秒）
 *
 * 将电机转速 (rad/s) 转换为每帧旋转角度：
 *   RPM = ω * 60 / (2π)
 *   ΔAngle = RPM * 360 / 60 * DeltaTime
 *
 * 旋向交替：电机 0/2 顺时针，电机 1/3 逆时针。
 */
void ADronePawn::UpdatePropellerAnimation(float DeltaTime)
{
    if (CurrentState.MotorSpeeds.Num() < 4) return;

    for (int32 i = 0; i < 4; i++)
    {
        UStaticMeshComponent* Fan = GetFanMesh(i);
        if (Fan)
        {
            // rad/s -> RPM -> degrees/frame
            float RPM = CurrentState.MotorSpeeds[i] * 60.0f / (2.0f * PI);
            float DeltaAngle = RPM * 360.0f / 60.0f * DeltaTime;
            // 交替旋向：0,2 顺时针，1,3 逆时针
            float Direction = (i % 2 == 0) ? 1.0f : -1.0f;
            Fan->AddLocalRotation(FRotator(0.0f, DeltaAngle * Direction, 0.0f));
        }
    }
}

/**
 * @brief 根据索引获取风扇网格组件
 * @param Index 风扇索引 (0-3)
 * @return 对应的 UStaticMeshComponent，索引越界返回 nullptr
 */
UStaticMeshComponent* ADronePawn::GetFanMesh(int32 Index) const
{
    switch (Index)
    {
    case 0: return Fan0;
    case 1: return Fan1;
    case 2: return Fan2;
    case 3: return Fan3;
    default: return nullptr;
    }
}

// ====================================================================
// 控制接口（由 DroneApi 调用）
// ====================================================================

/**
 * @brief 设置目标位置
 * @param TargetPos 目标位置 (SI, 米)
 *
 * 仅在控制模式变化时才切换模式，
 * 避免重复切换导致 PID 积分器清零而引起推力突变。
 */
void ADronePawn::SetTargetPosition(const FVector& TargetPos)
{
    if (MovementComp)
    {
        // 只在模式变化时才切换（避免清零 PID 积分器导致推力丢失）
        if (ControlMode != EDroneControlMode::Position)
        {
            MovementComp->SetControlMode(EDroneControlMode::Position);
            ControlMode = EDroneControlMode::Position;
        }
        MovementComp->SetTargetPosition(TargetPos);
    }
}

/**
 * @brief 设置目标速度
 * @param TargetVel 目标速度 (SI, m/s)
 *
 * 仅在控制模式变化时才切换模式。
 */
void ADronePawn::SetTargetVelocity(const FVector& TargetVel)
{
    if (MovementComp)
    {
        // 只在模式变化时才切换（避免清零 PID 积分器）
        if (ControlMode != EDroneControlMode::Velocity)
        {
            MovementComp->SetControlMode(EDroneControlMode::Velocity);
            ControlMode = EDroneControlMode::Velocity;
        }
        MovementComp->SetTargetVelocity(TargetVel);
    }
}

/**
 * @brief 起飞到指定高度
 * @param Altitude 目标高度 (m)
 *
 * 保持当前 XY 位置不变，设置 Z 为目标高度。
 */
void ADronePawn::Takeoff(float Altitude)
{
    FVector CurrentPos = CurrentState.GetPosition();
    FVector TargetPos(CurrentPos.X, CurrentPos.Y, Altitude);
    SetTargetPosition(TargetPos);
    UE_LOG(LogTemp, Log, TEXT("[DronePawn] Takeoff to %.1f m"), Altitude);
}

/**
 * @brief 降落到地面
 *
 * 保持当前 XY 位置不变，设置 Z=0。
 */
void ADronePawn::Land()
{
    FVector CurrentPos = CurrentState.GetPosition();
    FVector TargetPos(CurrentPos.X, CurrentPos.Y, 0.0f);
    SetTargetPosition(TargetPos);
    UE_LOG(LogTemp, Log, TEXT("[DronePawn] Landing"));
}

/**
 * @brief 在当前位置悬停
 *
 * 将当前位置设置为目标位置，位置控制器会自动维持。
 */
void ADronePawn::Hover()
{
    // 保持当前位置
    FVector CurrentPos = CurrentState.GetPosition();
    SetTargetPosition(CurrentPos);
    UE_LOG(LogTemp, Log, TEXT("[DronePawn] Hover at %s"), *CurrentPos.ToString());
}

/** @brief 获取当前位置 (SI, 米) */
FVector ADronePawn::GetCurrentPosition() const
{
    return CurrentState.GetPosition();
}

/** @brief 获取当前速度 (SI, m/s) */
FVector ADronePawn::GetCurrentVelocity() const
{
    return CurrentState.GetVelocity();
}

/**
 * @brief 重置无人机到指定位置和姿态
 * @param NewLocation 目标位置 (m)
 * @param NewRotation 目标姿态
 *
 * 重置 MovementComp 的物理状态和所有控制器，
 * 切换到 Idle 模式，并同步 UE Actor Transform。
 */
void ADronePawn::ResetDrone(const FVector& NewLocation, const FRotator& NewRotation)
{
    FDroneState NewState;
    NewState.SetPosition(NewLocation);
    NewState.SetQuaternion(NewRotation.Quaternion());

    if (MovementComp)
    {
        MovementComp->ResetState(NewState);
        MovementComp->SetControlMode(EDroneControlMode::Idle);
    }

    CurrentState = NewState;
    ControlMode = EDroneControlMode::Idle;
    ApplyStateToActor(NewState);

    UE_LOG(LogTemp, Log, TEXT("[DronePawn] Reset to %s"), *NewLocation.ToString());
}

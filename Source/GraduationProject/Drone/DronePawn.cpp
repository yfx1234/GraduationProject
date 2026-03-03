#include "DronePawn.h"
#include "DroneMovementComponent.h"
#include "DroneApi.h"
#include "GraduationProject/Core/Manager/AgentManager.h"
#include "Camera/CameraComponent.h"

/** @brief 构造函数 */
ADronePawn::ADronePawn()
{
    PrimaryActorTick.bCanEverTick = true;
    RootComp = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
    SetRootComponent(RootComp);
    BodyMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("BodyMesh"));
    BodyMesh->SetupAttachment(RootComp);
    Fan0 = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Fan0"));
    Fan0->SetupAttachment(BodyMesh, TEXT("Drone_Fan_002"));
    Fan1 = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Fan1"));
    Fan1->SetupAttachment(BodyMesh, TEXT("Drone_Fan1_002"));
    Fan2 = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Fan2"));
    Fan2->SetupAttachment(BodyMesh, TEXT("Drone_Fan2_002"));
    Fan3 = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Fan3"));
    Fan3->SetupAttachment(BodyMesh, TEXT("Drone_Fan3_002"));
    FPVCamera = CreateDefaultSubobject<UCameraComponent>(TEXT("FPVCamera"));
    FPVCamera->SetupAttachment(BodyMesh, TEXT("Camera_Yaw_002"));
    MovementComp = CreateDefaultSubobject<UDroneMovementComponent>(TEXT("Movement"));
}

/** @brief 游戏开始时初始化 */
void ADronePawn::BeginPlay()
{
    Super::BeginPlay();
    if (MovementComp)
    {
        MovementComp->SetParameters(Parameters);
        FDroneState InitState;
        FVector Pos = GetActorLocation() / 100.0f;
        InitState.SetPosition(Pos);
        MovementComp->SetInitialState(InitState);
        MovementComp->SetControlMode(ControlMode);
    }
    Api = NewObject<UDroneApi>(this);
    Api->Initialize(this);
    UAgentManager::GetInstance()->RegisterAgent(DroneId, this);
    UE_LOG(LogTemp, Log, TEXT("[DronePawn] BeginPlay: %s at %s"),
        *DroneId, *GetActorLocation().ToString());
}

/**
 * @brief 每帧更新
 * @param DeltaTime 帧间隔时间
 */
void ADronePawn::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
    if (MovementComp) CurrentState = MovementComp->GetCurrentState();
    ApplyStateToActor(CurrentState);
    UpdatePropellerAnimation(DeltaTime);
}

/**
 * @brief 将仿真状态同步到 UE Actor Transform
 * @param State 无人机当前状态
 */
void ADronePawn::ApplyStateToActor(const FDroneState& State)
{
    FVector UEPos = State.GetPosition() * 100.0f;
    FRotator UERot = State.GetRotator();
    SetActorLocationAndRotation(UEPos, UERot);
}

/**
 * @brief 更新螺旋桨旋转动画
 * @param DeltaTime 帧间隔（秒）
 * 将电机转速 (rad/s) 转换为每帧旋转角度：
 *   RPM = ω * 60 / (2π)
 *   ΔAngle = RPM * 360 / 60 * DeltaTime
 */
void ADronePawn::UpdatePropellerAnimation(float DeltaTime)
{
    if (CurrentState.MotorSpeeds.Num() < 4) return;
    for (int32 i = 0; i < 4; i++)
    {
        UStaticMeshComponent* Fan = GetFanMesh(i);
        if (Fan)
        {
            float RPM = CurrentState.MotorSpeeds[i] * 60.0f / (2.0f * PI);
            float DeltaAngle = RPM * 360.0f / 60.0f * DeltaTime;
            float Direction = (i % 2 == 0) ? 1.0f : -1.0f;
            Fan->AddLocalRotation(FRotator(0.0f, DeltaAngle * Direction, 0.0f));
        }
    }
}

/**
 * @brief 根据索引获取风扇网格组件
 * @param Index 风扇索引 (0-3)
 * @return 对应的 UStaticMeshComponent
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

/**
 * @brief 设置目标位置
 * @param TargetPos 目标位置
 */
void ADronePawn::SetTargetPosition(const FVector& TargetPos)
{
    if (MovementComp)
    {
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
 * @param TargetVel 目标速度
 */
void ADronePawn::SetTargetVelocity(const FVector& TargetVel)
{
    if (MovementComp)
    {
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
 * @param Altitude 目标高度
 */
void ADronePawn::Takeoff(float Altitude)
{
    FVector CurrentPos = CurrentState.GetPosition();
    FVector TargetPos(CurrentPos.X, CurrentPos.Y, Altitude);
    SetTargetPosition(TargetPos);
    UE_LOG(LogTemp, Log, TEXT("[DronePawn] Takeoff to %.1f m"), Altitude);
}

/** @brief 降落到地面 */
void ADronePawn::Land()
{
    FVector CurrentPos = CurrentState.GetPosition();
    FVector TargetPos(CurrentPos.X, CurrentPos.Y, 0.0f);
    SetTargetPosition(TargetPos);
    UE_LOG(LogTemp, Log, TEXT("[DronePawn] Landing"));
}

/** @brief 在当前位置悬停 */
void ADronePawn::Hover()
{
    FVector CurrentPos = CurrentState.GetPosition();
    SetTargetPosition(CurrentPos);
    UE_LOG(LogTemp, Log, TEXT("[DronePawn] Hover at %s"), *CurrentPos.ToString());
}

/** @brief 获取当前位置 */
FVector ADronePawn::GetCurrentPosition() const
{
    return CurrentState.GetPosition();
}

/** @brief 获取当前速度 */
FVector ADronePawn::GetCurrentVelocity() const
{
    return CurrentState.GetVelocity();
}

/**
 * @brief 重置无人机到指定位置和姿态
 * @param NewLocation 目标位置
 * @param NewRotation 目标姿态
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

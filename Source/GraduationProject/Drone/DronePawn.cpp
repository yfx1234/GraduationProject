#include "DronePawn.h"
#include "DroneMovementComponent.h"
#include "DroneApi.h"
#include "GraduationProject/Core/Manager/AgentManager.h"
#include "Camera/CameraComponent.h"

ADronePawn::ADronePawn()
{
    PrimaryActorTick.bCanEverTick = true;

    // 根组件
    RootComp = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
    SetRootComponent(RootComp);

    // 机身静态网格（Blueprint 中配置具体 StaticMesh）
    BodyMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("BodyMesh"));
    BodyMesh->SetupAttachment(RootComp);

    // 4 个螺旋桨 — 挂载到 BodyMesh 的插槽（Socket）
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

    // 运动组件
    MovementComp = CreateDefaultSubobject<UDroneMovementComponent>(TEXT("Movement"));
}

void ADronePawn::BeginPlay()
{
    Super::BeginPlay();

    // 初始化运动组件
    if (MovementComp)
    {
        MovementComp->SetParameters(Parameters);

        FDroneState InitState;
        FVector Pos = GetActorLocation() / 100.0f; // UE cm -> SI m
        InitState.SetPosition(Pos);
        MovementComp->SetInitialState(InitState);
        MovementComp->SetControlMode(ControlMode);
    }

    // 创建 API
    Api = NewObject<UDroneApi>(this);
    Api->Initialize(this);

    // 注册到 AgentManager
    UAgentManager::GetInstance()->RegisterAgent(DroneId, this);

    UE_LOG(LogTemp, Log, TEXT("[DronePawn] BeginPlay: %s at %s"),
        *DroneId, *GetActorLocation().ToString());
}

void ADronePawn::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    // MovementComp 会在自己的 TickComponent 中更新物理状态
    if (MovementComp)
    {
        CurrentState = MovementComp->GetCurrentState();
    }

    // 将物理状态同步到 Actor
    ApplyStateToActor(CurrentState);

    // 螺旋桨动画
    UpdatePropellerAnimation(DeltaTime);
}

void ADronePawn::ApplyStateToActor(const FDroneState& State)
{
    // SI (m) -> UE (cm)
    FVector UEPos = State.GetPosition() * 100.0f;
    FRotator UERot = State.GetRotator();
    SetActorLocationAndRotation(UEPos, UERot);
}

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

// ---- 控制接口 ----

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

void ADronePawn::Takeoff(float Altitude)
{
    FVector CurrentPos = CurrentState.GetPosition();
    FVector TargetPos(CurrentPos.X, CurrentPos.Y, Altitude);
    SetTargetPosition(TargetPos);
    UE_LOG(LogTemp, Log, TEXT("[DronePawn] Takeoff to %.1f m"), Altitude);
}

void ADronePawn::Land()
{
    FVector CurrentPos = CurrentState.GetPosition();
    FVector TargetPos(CurrentPos.X, CurrentPos.Y, 0.0f);
    SetTargetPosition(TargetPos);
    UE_LOG(LogTemp, Log, TEXT("[DronePawn] Landing"));
}

void ADronePawn::Hover()
{
    // 保持当前位置
    FVector CurrentPos = CurrentState.GetPosition();
    SetTargetPosition(CurrentPos);
    UE_LOG(LogTemp, Log, TEXT("[DronePawn] Hover at %s"), *CurrentPos.ToString());
}

FVector ADronePawn::GetCurrentPosition() const
{
    return CurrentState.GetPosition();
}

FVector ADronePawn::GetCurrentVelocity() const
{
    return CurrentState.GetVelocity();
}

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

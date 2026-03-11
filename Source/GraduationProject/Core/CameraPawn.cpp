#include "CameraPawn.h"

#include "Components/InputComponent.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"
#include "GraduationProject/Core/Manager/AgentManager.h"
#include "GraduationProject/Drone/DronePawn.h"
#include "GraduationProject/UI/SimHUD.h"
#include "InputCoreTypes.h"

/**
 * @brief 构造自由相机 Pawn
 * 创建根节点、弹簧臂和主相机，并默认接管 Player0 输入。
 */
ACameraPawn::ACameraPawn()
{
    PrimaryActorTick.bCanEverTick = true;

    RootComp = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
    SetRootComponent(RootComp);

    SpringArm = CreateDefaultSubobject<USpringArmComponent>(TEXT("SpringArm"));
    SpringArm->SetupAttachment(RootComp);
    SpringArm->TargetArmLength = 0.0f;
    SpringArm->bDoCollisionTest = false;

    Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera"));
    Camera->SetupAttachment(SpringArm);

    // Keep camera controls available even if a placed CameraPawn is used in map.
    AutoPossessPlayer = EAutoReceiveInput::Player0;
}

/**
 * @brief 初始化观察相机输入模式
 * 进入游戏后默认隐藏鼠标，并切换到纯游戏输入。
 */
void ACameraPawn::BeginPlay()
{
    Super::BeginPlay();

    APlayerController* PC = Cast<APlayerController>(GetController());
    if (!PC && GetWorld())
    {
        PC = GetWorld()->GetFirstPlayerController();
    }

    if (PC)
    {
        PC->bShowMouseCursor = false;
        PC->SetInputMode(FInputModeGameOnly());
    }
}

/**
 * @brief 每帧更新观察相机位置
 * @param DeltaTime 帧间隔（秒）
 * 根据当前模式在目标环绕、俯视、第一人称和自由移动之间切换。
 */
void ACameraPawn::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    const bool bTopDownMode = (!bIsTracking && !ActiveDroneId.IsEmpty() && DroneViewMode == EDroneViewCycleMode::TopDown);
    const bool bFPVMode = (!bIsTracking && !ActiveDroneId.IsEmpty() && DroneViewMode == EDroneViewCycleMode::FPV);

    if (bIsTracking && TrackingTarget)
    {
        UpdateTrackingCamera(DeltaTime);
    }
    else if (bTopDownMode)
    {
        UpdateTopDownCamera();
    }
    else if (bFPVMode)
    {
        UpdateFPVCamera();
    }
    else
    {
        if (!InputMoveDirection.IsNearlyZero())
        {
            const FRotator CameraRotation = GetActorRotation();
            const FVector Forward = CameraRotation.Vector();
            const FVector Right = FRotationMatrix(CameraRotation).GetScaledAxis(EAxis::Y);
            const FVector Up = FVector::UpVector;
            const FVector Movement =
                Forward * InputMoveDirection.X +
                Right * InputMoveDirection.Y +
                Up * InputMoveDirection.Z;

            AddActorWorldOffset(Movement * MoveSpeed * DeltaTime);
        }
    }

    InputMoveDirection = FVector::ZeroVector;
}

/**
 * @brief 绑定输入轴和快捷键
 * @param PlayerInputComponent 输入组件
 */
void ACameraPawn::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
    Super::SetupPlayerInputComponent(PlayerInputComponent);

    PlayerInputComponent->BindAxis("MoveForward", this, &ACameraPawn::MoveForward);
    PlayerInputComponent->BindAxis("MoveRight", this, &ACameraPawn::MoveRight);
    PlayerInputComponent->BindAxis("MoveUp", this, &ACameraPawn::MoveUp);
    PlayerInputComponent->BindAxis("CameraPitch", this, &ACameraPawn::CameraPitch);
    PlayerInputComponent->BindAxis("CameraYaw", this, &ACameraPawn::CameraYaw);

    PlayerInputComponent->BindAction("ZoomIn", IE_Pressed, this, &ACameraPawn::OnZoomIn);
    PlayerInputComponent->BindAction("ZoomOut", IE_Pressed, this, &ACameraPawn::OnZoomOut);

    // AirSim-like view hotkeys. PIP 1/2/3 are handled by ASimHUD.
    PlayerInputComponent->BindKey(EKeys::F, IE_Pressed, this, &ACameraPawn::OnCycleDroneView);
    PlayerInputComponent->BindKey(EKeys::M, IE_Pressed, this, &ACameraPawn::OnSwitchFreeView);
}

/** @brief 记录前后移动输入 */
void ACameraPawn::MoveForward(float Value)
{
    InputMoveDirection.X += Value;
}

/** @brief 记录左右平移输入 */
void ACameraPawn::MoveRight(float Value)
{
    InputMoveDirection.Y += Value;
}

/** @brief 记录上下移动输入 */
void ACameraPawn::MoveUp(float Value)
{
    InputMoveDirection.Z += Value;
}

/**
 * @brief 处理俯仰输入
 * @param Value 输入量
 * 跟踪模式下调整环绕俯仰角，自由模式下直接修改相机朝向。
 */
void ACameraPawn::CameraPitch(float Value)
{
    if (FMath::Abs(Value) <= 0.01f)
    {
        return;
    }

    if (bIsTracking)
    {
        OrbitPitch = FMath::Clamp(OrbitPitch + Value * MouseSensitivity, -89.0f, 89.0f);
        return;
    }

    if (!ActiveDroneId.IsEmpty() && DroneViewMode != EDroneViewCycleMode::Chase)
    {
        return;
    }

    FRotator CurrentRot = GetActorRotation();
    CurrentRot.Pitch = FMath::Clamp(CurrentRot.Pitch + Value * MouseSensitivity, -89.0f, 89.0f);
    SetActorRotation(CurrentRot);
}

/**
 * @brief 处理偏航输入
 * @param Value 输入量
 * 跟踪模式下仅累积轨道偏航角，其余模式下直接旋转相机。
 */
void ACameraPawn::CameraYaw(float Value)
{
    if (FMath::Abs(Value) <= 0.01f)
    {
        return;
    }

    if (bIsTracking)
    {
        OrbitYaw += Value * MouseSensitivity;
        return;
    }

    if (!ActiveDroneId.IsEmpty() && DroneViewMode != EDroneViewCycleMode::Chase)
    {
        return;
    }

    AddActorWorldRotation(FRotator(0.0f, Value * MouseSensitivity, 0.0f));
}

/** @brief 缩小当前跟踪或俯视距离 */
void ACameraPawn::OnZoomIn()
{
    if (bIsTracking || (!ActiveDroneId.IsEmpty() && DroneViewMode == EDroneViewCycleMode::TopDown))
    {
        TrackingDistance = FMath::Clamp(TrackingDistance - ZoomSpeed, MinTrackingDistance, MaxTrackingDistance);
    }
}

/** @brief 放大当前跟踪或俯视距离 */
void ACameraPawn::OnZoomOut()
{
    if (bIsTracking || (!ActiveDroneId.IsEmpty() && DroneViewMode == EDroneViewCycleMode::TopDown))
    {
        TrackingDistance = FMath::Clamp(TrackingDistance + ZoomSpeed, MinTrackingDistance, MaxTrackingDistance);
    }
}

/**
 * @brief 开始跟踪指定目标
 * @param Target 目标 Actor
 * 会重置环绕角和距离，并在目标是无人机时同步激活无人机视角模式。
 */
void ACameraPawn::StartTracking(AActor* Target)
{
    if (!Target)
    {
        return;
    }

    TrackingTarget = Target;
    bIsTracking = true;
    OrbitYaw = 0.0f;
    OrbitPitch = -30.0f;
    TrackingDistance = 500.0f;

    if (const ADronePawn* Drone = Cast<ADronePawn>(Target))
    {
        ActiveDroneId = Drone->DroneId;
        DroneViewMode = EDroneViewCycleMode::Chase;
    }

    UE_LOG(LogTemp, Log, TEXT("[Camera] Start tracking: %s"), *Target->GetName());
}

/** @brief 退出跟踪并回到自由视角 */
void ACameraPawn::StopTracking()
{
    bIsTracking = false;
    TrackingTarget = nullptr;
    UE_LOG(LogTemp, Log, TEXT("[Camera] Stop tracking, free mode"));
}

/**
 * @brief 处理 Agent 列表点击结果
 * @param AgentId 被点击的 Agent ID
 * @param Actor 对应 Actor
 * 选中有效目标时切换到跟踪模式，空选择时恢复自由视角。
 */
void ACameraPawn::OnItemClicked(const FString& AgentId, AActor* Actor)
{
    if (Actor)
    {
        if (const ADronePawn* Drone = Cast<ADronePawn>(Actor))
        {
            ActiveDroneId = Drone->DroneId;
            DroneViewMode = EDroneViewCycleMode::Chase;
        }
        StartTracking(Actor);
    }
    else
    {
        ActiveDroneId.Empty();
        StopTracking();
    }
}

/** @brief 转发快捷键，切换第一个 PIP 窗口 */
void ACameraPawn::OnTogglePip1()
{
    if (ASimHUD* SimHUD = ResolveSimHUD())
    {
        SimHUD->TogglePipSlot(0);
    }
}

/** @brief 转发快捷键，切换第二个 PIP 窗口 */
void ACameraPawn::OnTogglePip2()
{
    if (ASimHUD* SimHUD = ResolveSimHUD())
    {
        SimHUD->TogglePipSlot(1);
    }
}

/** @brief 转发快捷键，切换第三个 PIP 窗口 */
void ACameraPawn::OnTogglePip3()
{
    if (ASimHUD* SimHUD = ResolveSimHUD())
    {
        SimHUD->TogglePipSlot(2);
    }
}

/**
 * @brief 在无人机的追踪/俯视/FPV 视角之间循环切换
 * 若当前没有可用无人机，则仅输出警告日志。
 */
void ACameraPawn::OnCycleDroneView()
{
    ADronePawn* Drone = ResolveActiveDrone(true);
    if (!Drone)
    {
        UE_LOG(LogTemp, Warning, TEXT("[Camera] No drone available for view cycle"));
        return;
    }

    switch (DroneViewMode)
    {
    case EDroneViewCycleMode::Chase:
        DroneViewMode = EDroneViewCycleMode::TopDown;
        bIsTracking = false;
        TrackingTarget = Drone;
        UE_LOG(LogTemp, Log, TEXT("[Camera] Drone view -> TopDown (%s)"), *ActiveDroneId);
        break;
    case EDroneViewCycleMode::TopDown:
        DroneViewMode = EDroneViewCycleMode::FPV;
        bIsTracking = false;
        TrackingTarget = Drone;
        UE_LOG(LogTemp, Log, TEXT("[Camera] Drone view -> FPV (%s)"), *ActiveDroneId);
        break;
    case EDroneViewCycleMode::FPV:
    default:
        DroneViewMode = EDroneViewCycleMode::Chase;
        StartTracking(Drone);
        UE_LOG(LogTemp, Log, TEXT("[Camera] Drone view -> Chase (%s)"), *ActiveDroneId);
        break;
    }
}

/** @brief 切回自由视角并清除当前无人机选择 */
void ACameraPawn::OnSwitchFreeView()
{
    DroneViewMode = EDroneViewCycleMode::Chase;
    ActiveDroneId.Empty();
    StopTracking();
    UE_LOG(LogTemp, Log, TEXT("[Camera] Switch to free view (M)"));
}

/**
 * @brief 更新环绕跟踪相机
 * @param DeltaTime 帧间隔（秒）
 * 基于 `OrbitYaw/OrbitPitch/TrackingDistance` 计算相机相对目标的球面偏移。
 */
void ACameraPawn::UpdateTrackingCamera(float DeltaTime)
{
    if (!TrackingTarget)
    {
        return;
    }

    const FVector TargetLocation = TrackingTarget->GetActorLocation();
    const FRotator OrbitRotation(OrbitPitch, OrbitYaw, 0.0f);
    const FVector Offset = OrbitRotation.Vector() * (-TrackingDistance);
    const FVector CameraLocation = TargetLocation + Offset;

    SetActorLocation(CameraLocation);

    const FRotator LookAtRotation = (TargetLocation - CameraLocation).Rotation();
    SetActorRotation(LookAtRotation);
}

/**
 * @brief 更新俯视相机
 * 将相机放置在无人机正上方，并保持朝下观察。
 */
void ACameraPawn::UpdateTopDownCamera()
{
    ADronePawn* Drone = ResolveActiveDrone(false);
    if (!Drone)
    {
        return;
    }

    const FVector TargetLocation = Drone->GetActorLocation();
    const FRotator TargetRot = Drone->GetActorRotation();

    SetActorLocation(TargetLocation + FVector(0.0f, 0.0f, TrackingDistance));
    SetActorRotation(FRotator(-90.0f, TargetRot.Yaw, 0.0f));
}

/**
 * @brief 更新第一人称视角
 * 优先复用无人机的 `SceneCapture` 位姿，否则退化为无人机自身 Actor 位姿。
 */
void ACameraPawn::UpdateFPVCamera()
{
    ADronePawn* Drone = ResolveActiveDrone(false);
    if (!Drone)
    {
        return;
    }

    if (Drone->DroneSceneCapture)
    {
        SetActorLocationAndRotation(
            Drone->DroneSceneCapture->GetComponentLocation(),
            Drone->DroneSceneCapture->GetComponentRotation());
    }
    else
    {
        SetActorLocationAndRotation(Drone->GetActorLocation(), Drone->GetActorRotation());
    }
}

/**
 * @brief 获取当前玩家控制器上的 HUD
 * @return `ASimHUD` 指针；失败时返回空指针
 */
ASimHUD* ACameraPawn::ResolveSimHUD() const
{
    APlayerController* PC = Cast<APlayerController>(GetController());
    if (!PC && GetWorld())
    {
        PC = GetWorld()->GetFirstPlayerController();
    }

    if (!PC)
    {
        return nullptr;
    }

    return Cast<ASimHUD>(PC->GetHUD());
}

/**
 * @brief 解析当前激活的无人机
 * @param bAutoSelect 是否允许自动选择场景中第一架可用无人机
 * @return 无人机实例；若没有可用目标则返回空指针
 */
ADronePawn* ACameraPawn::ResolveActiveDrone(bool bAutoSelect)
{
    UAgentManager* Manager = UAgentManager::GetInstance();
    if (!Manager)
    {
        return nullptr;
    }

    if (!ActiveDroneId.IsEmpty())
    {
        if (ADronePawn* Drone = Cast<ADronePawn>(Manager->GetAgent(ActiveDroneId)))
        {
            return Drone;
        }

        ActiveDroneId.Empty();
    }

    if (!bAutoSelect)
    {
        return nullptr;
    }

    if (ADronePawn* PreferredDrone = Cast<ADronePawn>(Manager->GetAgent(TEXT("drone_0"))))
    {
        ActiveDroneId = PreferredDrone->DroneId;
        return PreferredDrone;
    }

    const TArray<FString> Ids = Manager->GetAllAgentIds();
    for (const FString& Id : Ids)
    {
        if (ADronePawn* Drone = Cast<ADronePawn>(Manager->GetAgent(Id)))
        {
            ActiveDroneId = Drone->DroneId;
            return Drone;
        }
    }

    return nullptr;
}

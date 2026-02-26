/**
 * @file CameraPawn.cpp
 * @brief 自由相机 Pawn 的实现文件
 *
 * 实现了 ACameraPawn 类的自由漫游和轨道跟踪两种相机模式的具体逻辑，
 * 包括输入绑定、移动插值、轨道位置计算等功能。
 */

#include "CameraPawn.h"

/**
 * @brief 构造函数，创建并初始化相机的组件层级
 *
 * 组件层级：RootComp → SpringArm → Camera
 * 自由模式下弹簧臂长度为 0，相机直接跟随 Pawn 位置。
 */
ACameraPawn::ACameraPawn()
{
    PrimaryActorTick.bCanEverTick = true;

    // 创建根场景组件
    RootComp = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
    SetRootComponent(RootComp);

    // 创建弹簧臂组件，自由模式下不使用弹簧臂功能
    SpringArm = CreateDefaultSubobject<USpringArmComponent>(TEXT("SpringArm"));
    SpringArm->SetupAttachment(RootComp);
    SpringArm->TargetArmLength = 0.0f; // 自由模式下弹簧臂长度为0
    SpringArm->bDoCollisionTest = false; // 禁用碰撞检测

    // 创建相机组件，挂载在弹簧臂末端
    Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera"));
    Camera->SetupAttachment(SpringArm);
}

/**
 * @brief 游戏开始时调用
 *
 * 获取玩家控制器并隐藏鼠标光标，使鼠标直接控制相机旋转。
 */
void ACameraPawn::BeginPlay()
{
    Super::BeginPlay();

    // 隐藏鼠标光标，开启相机旋转输入
    if (APlayerController* PC = Cast<APlayerController>(GetController()))
    {
        PC->bShowMouseCursor = false;
    }
}

/**
 * @brief 每帧更新逻辑
 * @param DeltaTime 帧间隔时间（秒）
 *
 * 根据当前模式执行不同逻辑：
 * - 轨道跟踪模式：调用 UpdateTrackingCamera() 更新轨道位置
 * - 自由漫游模式：根据输入方向和速度计算世界位移
 *
 * 每帧结束时重置输入缓冲。
 */
void ACameraPawn::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    if (bIsTracking && TrackingTarget)
    {
        // 轨道跟踪模式：更新轨道相机位置
        UpdateTrackingCamera(DeltaTime);
    }
    else
    {
        // 自由模式：根据输入方向应用世界空间移动
        if (!InputMoveDirection.IsNearlyZero())
        {
            // 获取当前相机朝向的前、右、上方向向量
            FRotator CameraRotation = GetActorRotation();
            FVector Forward = CameraRotation.Vector();
            FVector Right = FRotationMatrix(CameraRotation).GetScaledAxis(EAxis::Y);
            FVector Up = FVector::UpVector;

            // 合成最终移动向量（前后 + 左右 + 上下）
            FVector Movement = Forward * InputMoveDirection.X
                             + Right * InputMoveDirection.Y
                             + Up * InputMoveDirection.Z;

            // 应用移动（速度 × 时间 = 位移）
            AddActorWorldOffset(Movement * MoveSpeed * DeltaTime);
        }
    }

    // 每帧结束时重置输入方向缓冲，等待下一帧输入
    InputMoveDirection = FVector::ZeroVector;
}

/**
 * @brief 设置玩家输入绑定
 * @param PlayerInputComponent 玩家输入组件
 *
 * 将轴绑定（移动、旋转）和动作绑定（缩放）映射到对应的处理函数。
 * 要求在项目设置中配置对应的输入映射名称。
 */
void ACameraPawn::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
    Super::SetupPlayerInputComponent(PlayerInputComponent);

    // 轴绑定 — 持续输入
    PlayerInputComponent->BindAxis("MoveForward", this, &ACameraPawn::MoveForward);
    PlayerInputComponent->BindAxis("MoveRight", this, &ACameraPawn::MoveRight);
    PlayerInputComponent->BindAxis("MoveUp", this, &ACameraPawn::MoveUp);
    PlayerInputComponent->BindAxis("CameraPitch", this, &ACameraPawn::CameraPitch);
    PlayerInputComponent->BindAxis("CameraYaw", this, &ACameraPawn::CameraYaw);

    // 动作绑定 — 按键触发
    PlayerInputComponent->BindAction("ZoomIn", IE_Pressed, this, &ACameraPawn::OnZoomIn);
    PlayerInputComponent->BindAction("ZoomOut", IE_Pressed, this, &ACameraPawn::OnZoomOut);
}

/**
 * @brief 前后移动输入处理，累加到输入缓冲的 X 分量
 * @param Value 输入轴值（正=前进，负=后退）
 */
void ACameraPawn::MoveForward(float Value)
{
    InputMoveDirection.X += Value;
}

/**
 * @brief 左右移动输入处理，累加到输入缓冲的 Y 分量
 * @param Value 输入轴值（正=右移，负=左移）
 */
void ACameraPawn::MoveRight(float Value)
{
    InputMoveDirection.Y += Value;
}

/**
 * @brief 上下移动输入处理，累加到输入缓冲的 Z 分量
 * @param Value 输入轴值（正=上升，负=下降）
 */
void ACameraPawn::MoveUp(float Value)
{
    InputMoveDirection.Z += Value;
}

/**
 * @brief 相机俯仰（Pitch）旋转处理
 * @param Value 鼠标 Y 轴偏移值
 *
 * 自由模式：直接旋转 Pawn 的 Pitch，范围 [-89°, 89°]
 * 轨道模式：调整轨道俯仰角 OrbitPitch，范围 [-89°, 89°]
 * 忽略过小的输入值（死区 0.01）以避免微小抖动。
 */
void ACameraPawn::CameraPitch(float Value)
{
    if (FMath::Abs(Value) > 0.01f)
    {
        if (bIsTracking)
        {
            // 轨道模式：旋转轨道 Pitch 角度
            OrbitPitch = FMath::Clamp(OrbitPitch + Value * MouseSensitivity, -89.0f, 89.0f);
        }
        else
        {
            // 自由模式：直接旋转 Pawn 的 Pitch
            FRotator CurrentRot = GetActorRotation();
            CurrentRot.Pitch = FMath::Clamp(CurrentRot.Pitch + Value * MouseSensitivity, -89.0f, 89.0f);
            SetActorRotation(CurrentRot);
        }
    }
}

/**
 * @brief 相机偏航（Yaw）旋转处理
 * @param Value 鼠标 X 轴偏移值
 *
 * 自由模式：直接旋转 Pawn 的 Yaw（无范围限制，可 360° 旋转）
 * 轨道模式：调整轨道偏航角 OrbitYaw
 */
void ACameraPawn::CameraYaw(float Value)
{
    if (FMath::Abs(Value) > 0.01f)
    {
        if (bIsTracking)
        {
            // 轨道模式：旋转轨道 Yaw 角度
            OrbitYaw += Value * MouseSensitivity;
        }
        else
        {
            // 自由模式：直接旋转 Pawn 的 Yaw
            AddActorWorldRotation(FRotator(0.0f, Value * MouseSensitivity, 0.0f));
        }
    }
}

/**
 * @brief 缩放拉近回调（仅在轨道跟踪模式下生效）
 *
 * 减小 TrackingDistance，使相机靠近目标，受最小/最大距离限制。
 */
void ACameraPawn::OnZoomIn()
{
    if (bIsTracking)
    {
        TrackingDistance = FMath::Clamp(TrackingDistance - ZoomSpeed, MinTrackingDistance, MaxTrackingDistance);
    }
}

/**
 * @brief 缩放拉远回调（仅在轨道跟踪模式下生效）
 *
 * 增大 TrackingDistance，使相机远离目标，受最小/最大距离限制。
 */
void ACameraPawn::OnZoomOut()
{
    if (bIsTracking)
    {
        TrackingDistance = FMath::Clamp(TrackingDistance + ZoomSpeed, MinTrackingDistance, MaxTrackingDistance);
    }
}

/**
 * @brief 开始跟踪指定 Actor，切换到轨道跟踪模式
 * @param Target 要跟踪的目标 Actor
 *
 * 重置轨道角度为默认值（Yaw=0, Pitch=-30°, Distance=500cm），
 * 切换到轨道跟踪模式。
 */
void ACameraPawn::StartTracking(AActor* Target)
{
    if (Target)
    {
        TrackingTarget = Target;
        bIsTracking = true;
        OrbitYaw = 0.0f;           // 重置轨道偏航角
        OrbitPitch = -30.0f;       // 从上方俯视
        TrackingDistance = 500.0f;  // 默认跟踪距离 500cm
        UE_LOG(LogTemp, Log, TEXT("[Camera] Start tracking: %s"), *Target->GetName());
    }
}

/**
 * @brief 停止跟踪，回到自由漫游模式
 *
 * 清除跟踪目标引用，恢复自由相机操控。
 */
void ACameraPawn::StopTracking()
{
    bIsTracking = false;
    TrackingTarget = nullptr;
    UE_LOG(LogTemp, Log, TEXT("[Camera] Stop tracking, free mode"));
}

/**
 * @brief UI 列表点击回调
 * @param AgentId 被点击的智能体 ID
 * @param Actor 对应的 Actor 指针
 *
 * Actor 非空时开始跟踪，否则停止跟踪。
 */
void ACameraPawn::OnItemClicked(const FString& AgentId, AActor* Actor)
{
    if (Actor)
    {
        StartTracking(Actor);
    }
    else
    {
        StopTracking();
    }
}

/**
 * @brief 更新轨道跟踪模式下的相机位置和朝向
 * @param DeltaTime 帧间隔时间（秒，当前未使用）
 *
 * 算法步骤：
 * 1. 获取目标 Actor 的世界位置
 * 2. 根据 OrbitPitch/OrbitYaw 构造旋转，计算相机在目标周围的轨道偏移
 * 3. 将相机放置在 目标位置 + 轨道偏移 处
 * 4. 让相机朝向目标位置
 */
void ACameraPawn::UpdateTrackingCamera(float DeltaTime)
{
    if (!TrackingTarget) return;

    // 获取目标世界位置
    FVector TargetLocation = TrackingTarget->GetActorLocation();

    // 根据轨道角度计算相机偏移位置
    // OrbitRotation 表示从目标看向相机的方向
    FRotator OrbitRotation(OrbitPitch, OrbitYaw, 0.0f);
    // 沿反方向偏移 TrackingDistance，得到相机位置
    FVector Offset = OrbitRotation.Vector() * (-TrackingDistance);
    FVector CameraLocation = TargetLocation + Offset;

    // 设置相机位置
    SetActorLocation(CameraLocation);

    // 计算从相机位置指向目标的旋转，并应用
    FRotator LookAtRotation = (TargetLocation - CameraLocation).Rotation();
    SetActorRotation(LookAtRotation);
}

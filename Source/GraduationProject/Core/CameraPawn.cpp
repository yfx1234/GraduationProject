#include "CameraPawn.h"

/** @brief 创建并初始化相机的组件层级 */
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
}

/** @brief 获取玩家控制器并隐藏鼠标光标，鼠标直接控制相机旋转 */
void ACameraPawn::BeginPlay()
{
    Super::BeginPlay();
    if (APlayerController* PC = Cast<APlayerController>(GetController())) PC->bShowMouseCursor = false;
}

/** 
 * @brief 每帧更新逻辑
 * @param DeltaTime 帧间隔时间（秒）
 */
void ACameraPawn::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
    if (bIsTracking && TrackingTarget) UpdateTrackingCamera(DeltaTime);
    else
    {
        if (!InputMoveDirection.IsNearlyZero())
        {
            FRotator CameraRotation = GetActorRotation();
            FVector Forward = CameraRotation.Vector();
            FVector Right = FRotationMatrix(CameraRotation).GetScaledAxis(EAxis::Y);
            FVector Up = FVector::UpVector;
            FVector Movement = Forward * InputMoveDirection.X
                             + Right * InputMoveDirection.Y
                             + Up * InputMoveDirection.Z;
            AddActorWorldOffset(Movement * MoveSpeed * DeltaTime);
        }
    }
    InputMoveDirection = FVector::ZeroVector;
}

/**
 * @brief 设置玩家输入绑定
 * @param PlayerInputComponent 玩家输入组件
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
}

/**
 * @brief 前后移动输入处理，累加到输入缓冲的 X 分量
 * @param Value 输入轴值
 */
void ACameraPawn::MoveForward(float Value)
{
    InputMoveDirection.X += Value;
}

/**
 * @brief 左右移动输入处理，累加到输入缓冲的 Y 分量
 * @param Value 输入轴值
 */
void ACameraPawn::MoveRight(float Value)
{
    InputMoveDirection.Y += Value;
}

/**
 * @brief 上下移动输入处理，累加到输入缓冲的 Z 分量
 * @param Value 输入轴值
 */
void ACameraPawn::MoveUp(float Value)
{
    InputMoveDirection.Z += Value;
}

/**
 * @brief 相机俯仰旋转处理
 * @param Value 鼠标 Y 轴偏移值
 */
void ACameraPawn::CameraPitch(float Value)
{
    if (FMath::Abs(Value) > 0.01f)
    {
        if (bIsTracking) OrbitPitch = FMath::Clamp(OrbitPitch + Value * MouseSensitivity, -89.0f, 89.0f);
        else
        {
            FRotator CurrentRot = GetActorRotation();
            CurrentRot.Pitch = FMath::Clamp(CurrentRot.Pitch + Value * MouseSensitivity, -89.0f, 89.0f);
            SetActorRotation(CurrentRot);
        }
    }
}

/**
 * @brief 相机偏航旋转处理
 * @param Value 鼠标 X 轴偏移值
 */
void ACameraPawn::CameraYaw(float Value)
{
    if (FMath::Abs(Value) > 0.01f)
    {
        if (bIsTracking) OrbitYaw += Value * MouseSensitivity;
        else AddActorWorldRotation(FRotator(0.0f, Value * MouseSensitivity, 0.0f));
    }
}

/** @brief 缩放拉近回调 */
void ACameraPawn::OnZoomIn()
{
    if (bIsTracking) TrackingDistance = FMath::Clamp(TrackingDistance - ZoomSpeed, MinTrackingDistance, MaxTrackingDistance);
}

/** @brief 缩放拉远回调 */
void ACameraPawn::OnZoomOut()
{
    if (bIsTracking) TrackingDistance = FMath::Clamp(TrackingDistance + ZoomSpeed, MinTrackingDistance, MaxTrackingDistance);
}

/**
 * @brief 开始跟踪指定 Actor，切换到轨道跟踪模式
 * @param Target 要跟踪的目标 Actor
 */
void ACameraPawn::StartTracking(AActor* Target)
{
    if (Target)
    {
        TrackingTarget = Target;
        bIsTracking = true;
        OrbitYaw = 0.0f;          
        OrbitPitch = -30.0f;       
        TrackingDistance = 500.0f;  
        UE_LOG(LogTemp, Log, TEXT("[Camera] Start tracking: %s"), *Target->GetName());
    }
}

/** @brief 停止跟踪，回到自由模式 */
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
 */
void ACameraPawn::OnItemClicked(const FString& AgentId, AActor* Actor)
{
    if (Actor) StartTracking(Actor);
    else StopTracking();
}

/**
 * @brief 更新轨道跟踪模式下的相机位置和朝向
 * @param DeltaTime 帧间隔时间
 */
void ACameraPawn::UpdateTrackingCamera(float DeltaTime)
{
    if (!TrackingTarget) return;
    FVector TargetLocation = TrackingTarget->GetActorLocation();
    FRotator OrbitRotation(OrbitPitch, OrbitYaw, 0.0f);
    FVector Offset = OrbitRotation.Vector() * (-TrackingDistance);
    FVector CameraLocation = TargetLocation + Offset;
    SetActorLocation(CameraLocation);
    FRotator LookAtRotation = (TargetLocation - CameraLocation).Rotation();
    SetActorRotation(LookAtRotation);
}

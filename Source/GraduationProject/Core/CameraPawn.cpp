#include "CameraPawn.h"

ACameraPawn::ACameraPawn()
{
    PrimaryActorTick.bCanEverTick = true;

    // 根组件
    RootComp = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
    SetRootComponent(RootComp);

    // 弹簧臂
    SpringArm = CreateDefaultSubobject<USpringArmComponent>(TEXT("SpringArm"));
    SpringArm->SetupAttachment(RootComp);
    SpringArm->TargetArmLength = 0.0f; // 自由模式不用弹簧臂
    SpringArm->bDoCollisionTest = false;

    // 相机
    Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera"));
    Camera->SetupAttachment(SpringArm);
}

void ACameraPawn::BeginPlay()
{
    Super::BeginPlay();

    // 开启鼠标输入
    if (APlayerController* PC = Cast<APlayerController>(GetController()))
    {
        PC->bShowMouseCursor = false;
    }
}

void ACameraPawn::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    if (bIsTracking && TrackingTarget)
    {
        UpdateTrackingCamera(DeltaTime);
    }
    else
    {
        // 自由模式：应用移动
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

    // 重置输入
    InputMoveDirection = FVector::ZeroVector;
}

void ACameraPawn::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
    Super::SetupPlayerInputComponent(PlayerInputComponent);

    // 轴绑定
    PlayerInputComponent->BindAxis("MoveForward", this, &ACameraPawn::MoveForward);
    PlayerInputComponent->BindAxis("MoveRight", this, &ACameraPawn::MoveRight);
    PlayerInputComponent->BindAxis("MoveUp", this, &ACameraPawn::MoveUp);
    PlayerInputComponent->BindAxis("CameraPitch", this, &ACameraPawn::CameraPitch);
    PlayerInputComponent->BindAxis("CameraYaw", this, &ACameraPawn::CameraYaw);

    // 动作绑定
    PlayerInputComponent->BindAction("ZoomIn", IE_Pressed, this, &ACameraPawn::OnZoomIn);
    PlayerInputComponent->BindAction("ZoomOut", IE_Pressed, this, &ACameraPawn::OnZoomOut);
}

void ACameraPawn::MoveForward(float Value)
{
    InputMoveDirection.X += Value;
}

void ACameraPawn::MoveRight(float Value)
{
    InputMoveDirection.Y += Value;
}

void ACameraPawn::MoveUp(float Value)
{
    InputMoveDirection.Z += Value;
}

void ACameraPawn::CameraPitch(float Value)
{
    if (FMath::Abs(Value) > 0.01f)
    {
        if (bIsTracking)
        {
            // 轨道模式：旋转轨道Pitch
            OrbitPitch = FMath::Clamp(OrbitPitch + Value * MouseSensitivity, -89.0f, 89.0f);
        }
        else
        {
            // 自由模式：旋转Pawn
            FRotator CurrentRot = GetActorRotation();
            CurrentRot.Pitch = FMath::Clamp(CurrentRot.Pitch + Value * MouseSensitivity, -89.0f, 89.0f);
            SetActorRotation(CurrentRot);
        }
    }
}

void ACameraPawn::CameraYaw(float Value)
{
    if (FMath::Abs(Value) > 0.01f)
    {
        if (bIsTracking)
        {
            OrbitYaw += Value * MouseSensitivity;
        }
        else
        {
            AddActorWorldRotation(FRotator(0.0f, Value * MouseSensitivity, 0.0f));
        }
    }
}

void ACameraPawn::OnZoomIn()
{
    if (bIsTracking)
    {
        TrackingDistance = FMath::Clamp(TrackingDistance - ZoomSpeed, MinTrackingDistance, MaxTrackingDistance);
    }
}

void ACameraPawn::OnZoomOut()
{
    if (bIsTracking)
    {
        TrackingDistance = FMath::Clamp(TrackingDistance + ZoomSpeed, MinTrackingDistance, MaxTrackingDistance);
    }
}

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

void ACameraPawn::StopTracking()
{
    bIsTracking = false;
    TrackingTarget = nullptr;
    UE_LOG(LogTemp, Log, TEXT("[Camera] Stop tracking, free mode"));
}

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

void ACameraPawn::UpdateTrackingCamera(float DeltaTime)
{
    if (!TrackingTarget) return;

    FVector TargetLocation = TrackingTarget->GetActorLocation();

    // 计算轨道位置
    FRotator OrbitRotation(OrbitPitch, OrbitYaw, 0.0f);
    FVector Offset = OrbitRotation.Vector() * (-TrackingDistance);
    FVector CameraLocation = TargetLocation + Offset;

    SetActorLocation(CameraLocation);

    // 朝向目标
    FRotator LookAtRotation = (TargetLocation - CameraLocation).Rotation();
    SetActorRotation(LookAtRotation);
}

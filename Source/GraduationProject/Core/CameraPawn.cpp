#include "CameraPawn.h"

#include "Components/InputComponent.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"
#include "GraduationProject/Core/Manager/AgentManager.h"
#include "GraduationProject/Drone/DronePawn.h"
#include "GraduationProject/UI/SimHUD.h"
#include "InputCoreTypes.h"

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

void ACameraPawn::OnZoomIn()
{
    if (bIsTracking || (!ActiveDroneId.IsEmpty() && DroneViewMode == EDroneViewCycleMode::TopDown))
    {
        TrackingDistance = FMath::Clamp(TrackingDistance - ZoomSpeed, MinTrackingDistance, MaxTrackingDistance);
    }
}

void ACameraPawn::OnZoomOut()
{
    if (bIsTracking || (!ActiveDroneId.IsEmpty() && DroneViewMode == EDroneViewCycleMode::TopDown))
    {
        TrackingDistance = FMath::Clamp(TrackingDistance + ZoomSpeed, MinTrackingDistance, MaxTrackingDistance);
    }
}

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

void ACameraPawn::OnTogglePip1()
{
    if (ASimHUD* SimHUD = ResolveSimHUD())
    {
        SimHUD->TogglePipSlot(0);
    }
}

void ACameraPawn::OnTogglePip2()
{
    if (ASimHUD* SimHUD = ResolveSimHUD())
    {
        SimHUD->TogglePipSlot(1);
    }
}

void ACameraPawn::OnTogglePip3()
{
    if (ASimHUD* SimHUD = ResolveSimHUD())
    {
        SimHUD->TogglePipSlot(2);
    }
}

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

void ACameraPawn::OnSwitchFreeView()
{
    DroneViewMode = EDroneViewCycleMode::Chase;
    ActiveDroneId.Empty();
    StopTracking();
    UE_LOG(LogTemp, Log, TEXT("[Camera] Switch to free view (M)"));
}

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

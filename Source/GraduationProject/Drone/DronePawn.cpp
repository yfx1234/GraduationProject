
#include "DronePawn.h"
#include "DroneApi.h"
#include "DroneMovementComponent.h"
#include "GraduationProject/Core/Manager/AgentManager.h"
#include "GraduationProject/Vision/CameraCaptureUtils.h"
namespace
{
    enum class EDroneFrame : uint8
    {
        UE,
        NED,
    };
    FString JsonEscape(const FString& In)
    {
        FString Out = In;
        Out.ReplaceInline(TEXT("\\"), TEXT("\\\\"));
        Out.ReplaceInline(TEXT("\""), TEXT("\\\""));
        return Out;
    }
    EDroneFrame ParseFrame(const FString& FrameText)
    {
        FString Frame = FrameText;
        Frame.TrimStartAndEndInline();
        Frame.ToLowerInline();
        return (Frame == TEXT("ned")) ? EDroneFrame::NED : EDroneFrame::UE;
    }
    FString FrameToString(EDroneFrame Frame)
    {
        return (Frame == EDroneFrame::NED) ? TEXT("ned") : TEXT("ue");
    }
    FVector ConvertInputToUE(const FVector& Value, EDroneFrame Frame)
    {
        if (Frame == EDroneFrame::NED)
        {
            return FVector(Value.X, Value.Y, -Value.Z);
        }
        return Value;
    }
    FVector ConvertUEToOutput(const FVector& Value, EDroneFrame Frame)
    {
        if (Frame == EDroneFrame::NED)
        {
            return FVector(Value.X, Value.Y, -Value.Z);
        }
        return Value;
    }
    FRotator ConvertUEToFrameRotator(const FRotator& RotUE, EDroneFrame Frame)
    {
        if (Frame == EDroneFrame::NED)
        {
            return FRotator(-RotUE.Pitch, RotUE.Yaw, -RotUE.Roll);
        }
        return RotUE;
    }
    FRotator ConvertFrameToUERotator(const FRotator& RotFrame, EDroneFrame Frame)
    {
        if (Frame == EDroneFrame::NED)
        {
            return FRotator(-RotFrame.Pitch, RotFrame.Yaw, -RotFrame.Roll);
        }
        return RotFrame;
    }
    FString YawModeToString(EDroneYawMode Mode)
    {
        switch (Mode)
        {
        case EDroneYawMode::Hold:
            return TEXT("hold");
        case EDroneYawMode::Angle:
            return TEXT("angle");
        case EDroneYawMode::Rate:
            return TEXT("rate");
        case EDroneYawMode::Auto:
        default:
            return TEXT("auto");
        }
    }
    FString DrivetrainToString(EDroneDrivetrainMode Mode)
    {
        return (Mode == EDroneDrivetrainMode::MaxDegreeOfFreedom)
            ? TEXT("max_degree_of_freedom")
            : TEXT("forward_only");
    }
    FString RoleToString(EDroneMissionRole Role)
    {
        switch (Role)
        {
        case EDroneMissionRole::Target:
            return TEXT("target");
        case EDroneMissionRole::Interceptor:
            return TEXT("interceptor");
        case EDroneMissionRole::Unknown:
        default:
            return TEXT("unknown");
        }
    }
    FString ControlModeToString(EDroneControlMode Mode)
    {
        switch (Mode)
        {
        case EDroneControlMode::Idle:
            return TEXT("idle");
        case EDroneControlMode::Position:
            return TEXT("position");
        case EDroneControlMode::Velocity:
            return TEXT("velocity");
        case EDroneControlMode::AttitudeThrust:
            return TEXT("attitude");
        case EDroneControlMode::MotorSpeed:
            return TEXT("motor_speed");
        case EDroneControlMode::TorqueThrust:
            return TEXT("torque_thrust");
        default:
            return TEXT("unknown");
        }
    }
}
    static bool IsNearlyLegacyValue(double Value, double LegacyValue, double Tolerance)
    {
        return FMath::Abs(Value - LegacyValue) <= Tolerance;
    }
    static void UpgradeLegacyDroneParameters(FDroneParameters& Params)
    {
        bool bUpdated = false;
        auto UpgradeIfLegacy = [&bUpdated](double& Value, double LegacyValue, double StableValue, double Tolerance)
        {
            if (IsNearlyLegacyValue(Value, LegacyValue, Tolerance))
            {
                Value = StableValue;
                bUpdated = true;
            }
        };
        UpgradeIfLegacy(Params.Mass, 1.0, 1.0230, 1e-3);
        UpgradeIfLegacy(Params.Jx, 0.0023, 0.0095, 1e-4);
        UpgradeIfLegacy(Params.Jy, 0.0023, 0.0095, 1e-4);
        UpgradeIfLegacy(Params.Jz, 0.0040, 0.0186, 1e-4);
        UpgradeIfLegacy(Params.ArmLength, 0.2275, 0.2223, 1e-3);
        UpgradeIfLegacy(Params.C_T, 0.109919, 0.1667725, 1e-5);
        UpgradeIfLegacy(Params.C_P, 0.040164, 0.0901964, 1e-5);
        UpgradeIfLegacy(Params.MaxRPM, 6396.667, 12000.0, 1e-2);
        UpgradeIfLegacy(Params.MotorFilterTC, 0.005, 0.076, 1e-4);
        UpgradeIfLegacy(Params.AirDensity, 1.225, 1.175, 1e-3);
        if (bUpdated)
        {
            Params.InitializeComputed();
        }
    }
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
    CameraYawMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("CameraYawMesh"));
    CameraYawMesh->SetupAttachment(BodyMesh, TEXT("Camera_Yaw_002"));
    CameraPitchMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("CameraPitchMesh"));
    CameraPitchMesh->SetupAttachment(CameraYawMesh, TEXT("Camera_Pitch_002"));
    DroneSceneCapture = CreateDefaultSubobject<USceneCaptureComponent2D>(TEXT("DroneSceneCapture"));
    DroneSceneCapture->SetupAttachment(CameraPitchMesh);
    DroneSceneCapture->bCaptureEveryFrame = true;
    DroneSceneCapture->bCaptureOnMovement = false;
    DroneSceneCapture->bAlwaysPersistRenderingState = false;
    DroneSceneCapture->PostProcessSettings.bOverride_MotionBlurAmount = true;
    DroneSceneCapture->PostProcessSettings.MotionBlurAmount = 0.0f;
    DroneSceneCapture->PostProcessSettings.bOverride_MotionBlurMax = true;
    DroneSceneCapture->PostProcessSettings.MotionBlurMax = 0.0f;
    DroneSceneCapture->PostProcessSettings.bOverride_MotionBlurPerObjectSize = true;
    DroneSceneCapture->PostProcessSettings.MotionBlurPerObjectSize = 0.0f;
    DroneSceneCapture->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
    DroneCineCamera = CreateDefaultSubobject<UCineCameraComponent>(TEXT("DroneCineCamera"));
    DroneCineCamera->SetupAttachment(CameraPitchMesh);
    DroneCineCamera->SetActive(false);
    MovementComp = CreateDefaultSubobject<UDroneMovementComponent>(TEXT("Movement"));
}
void ADronePawn::BeginPlay()
{
    Super::BeginPlay();
    if (MovementComp)
    {
        UpgradeLegacyDroneParameters(Parameters);
        MovementComp->SetParameters(Parameters);
        FDroneState InitState;
        InitState.SetPosition(GetActorLocation() / 100.0f);
        MovementComp->SetInitialState(InitState);
        MovementComp->SetControlMode(ControlMode);
    }
    if (UTextureRenderTarget2D* RT = CameraCaptureUtils::CreateColorRenderTarget(this, CameraWidth, CameraHeight))
    {
        DroneSceneCapture->TextureTarget = RT;
    }
    DroneSceneCapture->FOVAngle = CameraFOV;
    CameraCaptureUtils::ApplySegmentationStencil(this, SegmentationId);
    Api = NewObject<UDroneApi>(this);
    Api->Initialize(this);
    if (UAgentManager* Manager = UAgentManager::GetInstance())
    {
        const FString ResolvedId = Manager->RegisterOrResolveAgent(DroneId, this);
        if (!ResolvedId.IsEmpty())
        {
            DroneId = ResolvedId;
        }
    }
}
void ADronePawn::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
    if (MovementComp)
    {
        CurrentState = MovementComp->GetCurrentState();
    }
    ApplyStateToActor(CurrentState);
    UpdatePropellerAnimation(DeltaTime);
    UpdateCameraRotation(DeltaTime);
    CameraCaptureUtils::SyncPostProcessToCapture(DroneCineCamera, DroneSceneCapture, ExposureBias, true);
}
void ADronePawn::ApplyStateToActor(const FDroneState& State)
{
    SetActorLocationAndRotation(State.GetPosition() * 100.0f, State.GetRotator());
}
void ADronePawn::UpdatePropellerAnimation(float DeltaTime)
{
    if (CurrentState.MotorSpeeds.Num() < 4)
    {
        return;
    }
    for (int32 Index = 0; Index < 4; ++Index)
    {
        UStaticMeshComponent* Fan = GetFanMesh(Index);
        if (!Fan)
        {
            continue;
        }
        const float RPM = CurrentState.MotorSpeeds[Index] * 60.0f / (2.0f * PI);
        const float DeltaAngle = RPM * 360.0f / 60.0f * DeltaTime;
        const float Direction = (Index % 2 == 0) ? 1.0f : -1.0f;
        Fan->AddLocalRotation(FRotator(0.0f, DeltaAngle * Direction, 0.0f));
    }
}
UStaticMeshComponent* ADronePawn::GetFanMesh(int32 Index) const
{
    switch (Index)
    {
    case 0:
        return Fan0;
    case 1:
        return Fan1;
    case 2:
        return Fan2;
    case 3:
        return Fan3;
    default:
        return nullptr;
    }
}
void ADronePawn::SetTargetPosition(const FVector& NewTargetPosition, float Speed, FString Frame)
{
    if (!MovementComp)
    {
        return;
    }
    if (ControlMode != EDroneControlMode::Position)
    {
        MovementComp->SetControlMode(EDroneControlMode::Position);
        ControlMode = EDroneControlMode::Position;
    }
    const FVector TargetUE = ConvertInputToUE(NewTargetPosition, ParseFrame(Frame));
    MovementComp->SetTargetPosition(TargetUE, Speed);
}
void ADronePawn::SetTargetVelocity(const FVector& NewTargetVelocity, FString Frame)
{
    if (!MovementComp)
    {
        return;
    }
    if (ControlMode != EDroneControlMode::Velocity)
    {
        MovementComp->SetControlMode(EDroneControlMode::Velocity);
        ControlMode = EDroneControlMode::Velocity;
    }
    const FVector VelocityUE = ConvertInputToUE(NewTargetVelocity, ParseFrame(Frame));
    MovementComp->SetTargetVelocity(VelocityUE);
}
void ADronePawn::MoveByVelocity(float Vx, float Vy, float Vz, FString Frame)
{
    SetTargetVelocity(FVector(Vx, Vy, Vz), MoveTemp(Frame));
}
void ADronePawn::SetHeadingControl(EDroneYawMode NewYawMode, EDroneDrivetrainMode NewDrivetrain, float YawDeg)
{
    if (MovementComp)
    {
        MovementComp->SetHeadingControl(NewYawMode, NewDrivetrain, YawDeg);
    }
}
void ADronePawn::Hover()
{
    SetTargetPosition(CurrentState.GetPosition());
}
void ADronePawn::Takeoff(float Altitude)
{
    const FVector CurrentPos = CurrentState.GetPosition();
    SetTargetPosition(FVector(CurrentPos.X, CurrentPos.Y, Altitude));
}
void ADronePawn::Land()
{
    const FVector CurrentPos = CurrentState.GetPosition();
    SetTargetPosition(FVector(CurrentPos.X, CurrentPos.Y, 0.0f));
}
void ADronePawn::EnableApiControl(bool bEnable)
{
    bApiControlEnabled = bEnable;
}
void ADronePawn::ResetDrone(const FVector& NewLocation, const FRotator& NewRotation, FString Frame)
{
    FDroneState NewState;
    NewState.SetPosition(ConvertInputToUE(NewLocation, ParseFrame(Frame)));
    NewState.SetQuaternion(ConvertFrameToUERotator(NewRotation, ParseFrame(Frame)).Quaternion());
    if (MovementComp)
    {
        MovementComp->ResetState(NewState);
        MovementComp->SetControlMode(EDroneControlMode::Idle);
    }
    CurrentState = NewState;
    ControlMode = EDroneControlMode::Idle;
    ApplyStateToActor(NewState);
}
void ADronePawn::ResetActorState()
{
    if (Api)
    {
        Api->Reset();
    }
}
void ADronePawn::SetTargetAttitude(float RollDeg, float PitchDeg, float YawDeg, float Thrust, FString Frame)
{
    if (!Api)
    {
        return;
    }
    const FRotator UERot = ConvertFrameToUERotator(FRotator(PitchDeg, YawDeg, RollDeg), ParseFrame(Frame));
    Api->SetTargetAttitude(UERot.Roll, UERot.Pitch, UERot.Yaw, Thrust);
    ControlMode = EDroneControlMode::AttitudeThrust;
}
void ADronePawn::SetMotorSpeeds(float M0, float M1, float M2, float M3)
{
    if (!Api)
    {
        return;
    }
    Api->SetMotorSpeeds(M0, M1, M2, M3);
    ControlMode = EDroneControlMode::MotorSpeed;
}
void ADronePawn::SetPositionControllerGains(float Kp, float Kd)
{
    if (Api)
    {
        Api->SetPositionControllerGains(Kp, Kd);
    }
}
void ADronePawn::SetVelocityControllerGains(float Kp, float Ki, float Kd)
{
    if (Api)
    {
        Api->SetVelocityControllerGains(Kp, Ki, Kd);
    }
}
void ADronePawn::SetAttitudeControllerGains(float Kp, float Kd)
{
    if (Api)
    {
        Api->SetAttitudeControllerGains(Kp, Kd);
    }
}
void ADronePawn::SetAngleRateControllerGains(float Kp)
{
    if (Api)
    {
        Api->SetAngleRateControllerGains(Kp);
    }
}
FVector ADronePawn::GetCurrentPosition() const
{
    return CurrentState.GetPosition();
}
FVector ADronePawn::GetCurrentVelocity() const
{
    return CurrentState.GetVelocity();
}
void ADronePawn::SetCameraAngles(float TargetPitch, float TargetYaw)
{
    CameraTargetPitch = TargetPitch;
    CameraTargetYaw = TargetYaw;
}
void ADronePawn::SetCameraFOV(float NewFOV)
{
    CameraFOV = FMath::Clamp(NewFOV, 30.0f, 160.0f);
    if (DroneSceneCapture)
    {
        DroneSceneCapture->FOVAngle = CameraFOV;
    }
    if (DroneCineCamera)
    {
        DroneCineCamera->SetFieldOfView(CameraFOV);
    }
}
void ADronePawn::SetSegmentationId(int32 NewSegmentationId)
{
    SegmentationId = FMath::Clamp(NewSegmentationId, 0, 255);
    CameraCaptureUtils::ApplySegmentationStencil(this, SegmentationId);
}
FString ADronePawn::GetState(FString Frame)
{
    const EDroneFrame OutputFrame = ParseFrame(Frame);
    const FVector Pos = ConvertUEToOutput(GetCurrentPosition(), OutputFrame);
    const FVector Vel = ConvertUEToOutput(GetCurrentVelocity(), OutputFrame);
    const FRotator Rot = ConvertUEToFrameRotator(CurrentState.GetRotator(), OutputFrame);
    EDroneYawMode YawMode = EDroneYawMode::Auto;
    EDroneDrivetrainMode Drivetrain = EDroneDrivetrainMode::MaxDegreeOfFreedom;
    if (MovementComp)
    {
        YawMode = MovementComp->GetYawMode();
        Drivetrain = MovementComp->GetDrivetrainMode();
    }
    const TArray<float> Motors = Api ? Api->GetMotorSpeeds() : TArray<float>();
    return FString::Printf(
        TEXT("{\"status\":\"ok\",\"id\":\"%s\",\"frame\":\"%s\",\"orientation_frame\":\"%s\",\"position\":[%.4f,%.4f,%.4f],\"velocity\":[%.4f,%.4f,%.4f],\"orientation\":{\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f},\"motor_speeds\":[%.1f,%.1f,%.1f,%.1f],\"yaw_mode\":\"%s\",\"drivetrain\":\"%s\",\"role\":\"%s\",\"camera_pitch\":%.2f,\"camera_yaw\":%.2f,\"control_mode\":\"%s\",\"api_control\":%s}"),
        *JsonEscape(DroneId),
        *FrameToString(OutputFrame),
        *FrameToString(OutputFrame),
        Pos.X,
        Pos.Y,
        Pos.Z,
        Vel.X,
        Vel.Y,
        Vel.Z,
        Rot.Roll,
        Rot.Pitch,
        Rot.Yaw,
        Motors.Num() >= 4 ? Motors[0] : 0.0f,
        Motors.Num() >= 4 ? Motors[1] : 0.0f,
        Motors.Num() >= 4 ? Motors[2] : 0.0f,
        Motors.Num() >= 4 ? Motors[3] : 0.0f,
        *YawModeToString(YawMode),
        *DrivetrainToString(Drivetrain),
        *RoleToString(MissionRole),
        GetCameraCurrentPitch(),
        GetCameraCurrentYaw(),
        *ControlModeToString(ControlMode),
        bApiControlEnabled ? TEXT("true") : TEXT("false"));
}
FString ADronePawn::GetImage(FString ImageType, int32 Quality, float MaxDepthMeters)
{
    return CameraCaptureUtils::CaptureImageJson(
        DroneSceneCapture,
        DroneId,
        CameraWidth,
        CameraHeight,
        CameraFOV,
        JpegQuality,
        ImageType,
        Quality,
        MaxDepthMeters);
}
FString ADronePawn::CaptureImageBase64(int32 Quality)
{
    if (Quality <= 0)
    {
        Quality = JpegQuality;
    }
    return CameraCaptureUtils::CaptureColorJpegBase64(DroneSceneCapture, CameraWidth, CameraHeight, Quality);
}
void ADronePawn::UpdateCameraRotation(float DeltaTime)
{
    const float YawDiff = FRotator::NormalizeAxis(CameraTargetYaw - CameraCurrentYaw);
    const float AdjustedTargetYaw = CameraCurrentYaw + YawDiff;
    CameraCurrentYaw = FMath::FInterpTo(CameraCurrentYaw, AdjustedTargetYaw, DeltaTime, CameraRotationSpeed);
    CameraCurrentPitch = FMath::FInterpTo(CameraCurrentPitch, CameraTargetPitch, DeltaTime, CameraRotationSpeed);
    if (CameraYawMesh)
    {
        CameraYawMesh->SetRelativeRotation(FRotator(0.0f, CameraCurrentYaw, 0.0f));
    }
    if (CameraPitchMesh)
    {
        CameraPitchMesh->SetRelativeRotation(FRotator(0.0f, 0.0f, -CameraCurrentPitch));
    }
}

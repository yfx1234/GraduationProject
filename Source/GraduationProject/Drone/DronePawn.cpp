#include "DronePawn.h"
#include "DroneMovementComponent.h"
#include "DroneApi.h"
#include "GraduationProject/Core/Manager/AgentManager.h"
#include "Misc/Base64.h"
#include "IImageWrapperModule.h"
#include "IImageWrapper.h"
#include "GraduationProject/Vision/CameraCaptureUtils.h"
#include "Components/PrimitiveComponent.h"
#include "HAL/IConsoleManager.h"

/**
 * @brief 构造无人机 Pawn
 * 创建机体、旋翼、云台、相机和飞行动力学组件，并建立附着层级。
 */
ADronePawn::ADronePawn()
{
    PrimaryActorTick.bCanEverTick = true;

    // 创建根节点和机体网格。
    RootComp = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
    SetRootComponent(RootComp);
    BodyMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("BodyMesh"));
    BodyMesh->SetupAttachment(RootComp);

    // 创建四个旋翼网格，并挂到机体对应插槽上。
    Fan0 = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Fan0"));
    Fan0->SetupAttachment(BodyMesh, TEXT("Drone_Fan_002"));
    Fan1 = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Fan1"));
    Fan1->SetupAttachment(BodyMesh, TEXT("Drone_Fan1_002"));
    Fan2 = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Fan2"));
    Fan2->SetupAttachment(BodyMesh, TEXT("Drone_Fan2_002"));
    Fan3 = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Fan3"));
    Fan3->SetupAttachment(BodyMesh, TEXT("Drone_Fan3_002"));

    // 创建云台偏航轴和俯仰轴网格。
    CameraYawMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("CameraYawMesh"));
    CameraYawMesh->SetupAttachment(BodyMesh, TEXT("Camera_Yaw_002"));
    CameraPitchMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("CameraPitchMesh"));
    CameraPitchMesh->SetupAttachment(CameraYawMesh, TEXT("Camera_Pitch_002"));

    // 创建导出图像用的 SceneCapture，并禁用运动模糊。
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

    // 创建 CineCamera，用于继承场景 PostProcess，再同步给 SceneCapture。
    DroneCineCamera = CreateDefaultSubobject<UCineCameraComponent>(TEXT("DroneCineCamera"));
    DroneCineCamera->SetupAttachment(CameraPitchMesh);
    DroneCineCamera->SetActive(false);

    // 创建飞行控制与状态积分组件。
    MovementComp = CreateDefaultSubobject<UDroneMovementComponent>(TEXT("Movement"));
}

/**
 * @brief 初始化无人机运行时状态
 * 将地图初始位置写入飞控状态，创建 RenderTarget，并向 `AgentManager` 注册自身。
 */
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

    // 创建相机渲染目标并挂到 SceneCapture。
    UTextureRenderTarget2D* RT = NewObject<UTextureRenderTarget2D>(this);
    RT->InitCustomFormat(CameraWidth, CameraHeight, PF_B8G8R8A8, false);
    RT->ClearColor = FLinearColor::Black;
    DroneSceneCapture->TextureTarget = RT;
    DroneSceneCapture->FOVAngle = CameraFOV;
    ApplySegmentationStencil();

    Api = NewObject<UDroneApi>(this);
    Api->Initialize(this);
    UAgentManager* Manager = UAgentManager::GetInstance();
    if (Manager)
    {
        FString ExistingId;
        const TArray<FString> ExistingIds = Manager->GetAllAgentIds();
        for (const FString& Id : ExistingIds)
        {
            if (Manager->GetAgent(Id) == this)
            {
                ExistingId = Id;
                break;
            }
        }

        if (!ExistingId.IsEmpty() && ExistingId != DroneId)
        {
            UE_LOG(LogTemp, Warning, TEXT("[DronePawn] Adopt pre-registered ID '%s' (old DroneId='%s')"), *ExistingId, *DroneId);
            DroneId = ExistingId;
        }

        if (Manager->GetAgent(DroneId) != this)
        {
            Manager->RegisterAgent(DroneId, this);
        }
    }
    UE_LOG(LogTemp, Log, TEXT("[DronePawn] BeginPlay: %s at %s, Camera %dx%d FOV=%.0f"),
        *DroneId, *GetActorLocation().ToString(), CameraWidth, CameraHeight, CameraFOV);
}

/**
 * @brief 每帧同步无人机状态
 * @param DeltaTime 帧间隔（秒）
 * 从飞控组件读取当前状态，并同步到 Actor、旋翼动画和云台姿态。
 */
void ADronePawn::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
    if (MovementComp) CurrentState = MovementComp->GetCurrentState();
    ApplyStateToActor(CurrentState);
    UpdatePropellerAnimation(DeltaTime);
    UpdateCameraRotation(DeltaTime);
    SyncPostProcessToCapture();
}

/**
 * @brief 将飞控状态应用到场景 Actor
 * @param State 需要同步的无人机状态
 */
void ADronePawn::ApplyStateToActor(const FDroneState& State)
{
    FVector UEPos = State.GetPosition() * 100.0f;
    FRotator UERot = State.GetRotator();
    SetActorLocationAndRotation(UEPos, UERot);
}

/**
 * @brief 更新旋翼动画
 * @param DeltaTime 帧间隔（秒）
 * 电机角速度以 rad/s 存储，先换算为 RPM，再计算每帧旋转角度。
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
 * @brief 获取指定旋翼网格组件
 * @param Index 旋翼索引（0-3）
 * @return 对应的旋翼网格组件；越界时返回空指针
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
 * @brief 设置位置控制目标
 * @param TargetPos 目标位置（米）
 * @param Speed 期望飞行速度
 */
void ADronePawn::SetTargetPosition(const FVector& TargetPos, float Speed)
{
    if (MovementComp)
    {
        if (ControlMode != EDroneControlMode::Position)
        {
            MovementComp->SetControlMode(EDroneControlMode::Position);
            ControlMode = EDroneControlMode::Position;
        }
        MovementComp->SetTargetPosition(TargetPos, Speed);
    }
}

/**
 * @brief 设置速度控制目标
 * @param TargetVel 目标速度（米/秒）
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
 * @brief 以速度模式移动
 * @param Vx X 方向速度（米/秒）
 * @param Vy Y 方向速度（米/秒）
 * @param Vz Z 方向速度（米/秒）
 */
void ADronePawn::MoveByVelocity(float Vx, float Vy, float Vz)
{
    SetTargetVelocity(FVector(Vx, Vy, Vz));
}

/**
 * @brief 设置航向控制模式
 * @param YawMode 偏航控制模式
 * @param Drivetrain 运动学约束模式
 * @param YawDeg 目标偏航角
 */
void ADronePawn::SetHeadingControl(EDroneYawMode YawMode, EDroneDrivetrainMode Drivetrain, float YawDeg)
{
    if (MovementComp)
    {
        MovementComp->SetHeadingControl(YawMode, Drivetrain, YawDeg);
    }
}

/**
 * @brief 起飞到指定高度
 * @param Altitude 目标高度（米）
 */
void ADronePawn::Takeoff(float Altitude)
{
    FVector CurrentPos = CurrentState.GetPosition();
    FVector TargetPos(CurrentPos.X, CurrentPos.Y, Altitude);
    SetTargetPosition(TargetPos);
    UE_LOG(LogTemp, Log, TEXT("[DronePawn] Takeoff to %.1f m"), Altitude);
}

/** @brief 降落到地面高度 */
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
 * @brief 重置无人机状态
 * @param NewLocation 新位置（米）
 * @param NewRotation 新姿态
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

/**
 * @brief 设置云台目标角度
 * @param TargetPitch 目标俯仰角（度）
 * @param TargetYaw 目标偏航角（度）
 */
void ADronePawn::SetCameraAngles(float TargetPitch, float TargetYaw)
{
    CameraTargetPitch = TargetPitch;
    CameraTargetYaw = TargetYaw;
}

/**
 * @brief 更新云台角度插值
 * @param DeltaTime 帧间隔（秒）
 * 偏航轴使用归一化角差避免跨越 `-180~180` 时跳变。
 */
void ADronePawn::UpdateCameraRotation(float DeltaTime)
{
    float YawDiff = FRotator::NormalizeAxis(CameraTargetYaw - CameraCurrentYaw);
    float AdjustedTargetYaw = CameraCurrentYaw + YawDiff;
    CameraCurrentYaw = FMath::FInterpTo(CameraCurrentYaw, AdjustedTargetYaw, DeltaTime, CameraRotationSpeed);

    CameraCurrentPitch = FMath::FInterpTo(CameraCurrentPitch, CameraTargetPitch, DeltaTime, CameraRotationSpeed);

    if (CameraYawMesh) CameraYawMesh->SetRelativeRotation(FRotator(0.0f, CameraCurrentYaw, 0.0f));
    if (CameraPitchMesh) CameraPitchMesh->SetRelativeRotation(FRotator(0.0f, 0.0f, -CameraCurrentPitch));
}

/** @brief 将可见网格写入 CustomDepth/Stencil，供分割图像使用 */
void ADronePawn::ApplySegmentationStencil()
{
    IConsoleVariable* CustomDepthVar = IConsoleManager::Get().FindConsoleVariable(TEXT("r.CustomDepth"));
    if (CustomDepthVar && CustomDepthVar->GetInt() < 3)
    {
        CustomDepthVar->Set(3, ECVF_SetByCode);
    }

    const int32 ClampedId = FMath::Clamp(SegmentationId, 0, 255);

    TArray<UPrimitiveComponent*> PrimitiveComponents;
    GetComponents<UPrimitiveComponent>(PrimitiveComponents);
    for (UPrimitiveComponent* Primitive : PrimitiveComponents)
    {
        if (!Primitive)
        {
            continue;
        }

        Primitive->SetRenderCustomDepth(true);
        Primitive->SetCustomDepthStencilValue(ClampedId);
    }
}

/**
 * @brief 将 CineCamera 的后处理设置同步给 SceneCapture
 * 额外强制关闭运动模糊，以减少远程图像串帧和拖影。
 */
void ADronePawn::SyncPostProcessToCapture()
{
    if (!DroneCineCamera || !DroneSceneCapture) return;
    FMinimalViewInfo ViewInfo;
    DroneCineCamera->GetCameraView(0.0f, ViewInfo);
    FWeightedBlendables SavedBlendables = DroneSceneCapture->PostProcessSettings.WeightedBlendables;
    DroneSceneCapture->PostProcessSettings = ViewInfo.PostProcessSettings;
    DroneSceneCapture->PostProcessSettings.WeightedBlendables = SavedBlendables;
    DroneSceneCapture->PostProcessSettings.bOverride_AutoExposureBias = true;
    DroneSceneCapture->PostProcessSettings.AutoExposureBias += ExposureBias;

    // Avoid ghosting/stacked-frame artifacts in remote viewer captures.
    DroneSceneCapture->PostProcessSettings.bOverride_MotionBlurAmount = true;
    DroneSceneCapture->PostProcessSettings.MotionBlurAmount = 0.0f;
    DroneSceneCapture->PostProcessSettings.bOverride_MotionBlurMax = true;
    DroneSceneCapture->PostProcessSettings.MotionBlurMax = 0.0f;
    DroneSceneCapture->PostProcessSettings.bOverride_MotionBlurPerObjectSize = true;
    DroneSceneCapture->PostProcessSettings.MotionBlurPerObjectSize = 0.0f;
}

/**
 * @brief 捕获当前相机画面并返回 Base64 JPEG
 * @param Quality JPEG 压缩质量；小于等于 0 时使用 `JpegQuality`
 * @return Base64 编码后的 JPEG 字符串
 */
FString ADronePawn::CaptureImageBase64(int32 Quality)
{
    if (Quality <= 0)
    {
        Quality = JpegQuality;
    }

    return CameraCaptureUtils::CaptureColorJpegBase64(DroneSceneCapture, CameraWidth, CameraHeight, Quality);
}

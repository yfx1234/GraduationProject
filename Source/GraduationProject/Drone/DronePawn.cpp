#include "DronePawn.h"
#include "DroneMovementComponent.h"
#include "DroneApi.h"
#include "GraduationProject/Core/Manager/AgentManager.h"
#include "Misc/Base64.h"
#include "IImageWrapperModule.h"
#include "IImageWrapper.h"

/**
 * @brief 构造函数
 * 创建机身网格、四个旋翼（通过插槽附着）、
 * 摄像头 Yaw/Pitch 云台（通过插槽附着）、
 * SceneCaptureComponent2D 和 CineCameraComponent（附着到 Pitch 云台）
 */
ADronePawn::ADronePawn()
{
    PrimaryActorTick.bCanEverTick = true;

    // ── 根组件和机身 ──
    RootComp = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
    SetRootComponent(RootComp);
    BodyMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("BodyMesh"));
    BodyMesh->SetupAttachment(RootComp);

    // ── 四个旋翼 — 通过 BodyMesh 上的插槽附着 ──
    Fan0 = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Fan0"));
    Fan0->SetupAttachment(BodyMesh, TEXT("Drone_Fan_002"));
    Fan1 = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Fan1"));
    Fan1->SetupAttachment(BodyMesh, TEXT("Drone_Fan1_002"));
    Fan2 = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Fan2"));
    Fan2->SetupAttachment(BodyMesh, TEXT("Drone_Fan2_002"));
    Fan3 = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Fan3"));
    Fan3->SetupAttachment(BodyMesh, TEXT("Drone_Fan3_002"));

    // ── 摄像头云台 — Yaw 附着到 BodyMesh 插槽，Pitch 附着到 Yaw 插槽 ──
    CameraYawMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("CameraYawMesh"));
    CameraYawMesh->SetupAttachment(BodyMesh, TEXT("Camera_Yaw_002"));
    CameraPitchMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("CameraPitchMesh"));
    CameraPitchMesh->SetupAttachment(CameraYawMesh, TEXT("Camera_Pitch_002"));

    // ── 场景采集组件 — 附着到 Pitch 云台 ──
    DroneSceneCapture = CreateDefaultSubobject<USceneCaptureComponent2D>(TEXT("DroneSceneCapture"));
    DroneSceneCapture->SetupAttachment(CameraPitchMesh);
    DroneSceneCapture->bCaptureEveryFrame = true;
    DroneSceneCapture->bCaptureOnMovement = false;
    DroneSceneCapture->bAlwaysPersistRenderingState = true;
    DroneSceneCapture->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;

    // ── CineCamera — 用于继承场景 PostProcess 设置 ──
    DroneCineCamera = CreateDefaultSubobject<UCineCameraComponent>(TEXT("DroneCineCamera"));
    DroneCineCamera->SetupAttachment(CameraPitchMesh);
    DroneCineCamera->SetActive(false);

    // ── 运动组件 ──
    MovementComp = CreateDefaultSubobject<UDroneMovementComponent>(TEXT("Movement"));
}

/**
 * @brief 游戏开始时初始化
 * 初始化运动仿真参数、创建 RenderTarget 并配置 SceneCapture、注册 Agent
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

    // 创建 RenderTarget 并配置 SceneCapture
    UTextureRenderTarget2D* RT = NewObject<UTextureRenderTarget2D>(this);
    RT->InitCustomFormat(CameraWidth, CameraHeight, PF_B8G8R8A8, false);
    RT->ClearColor = FLinearColor::Black;
    DroneSceneCapture->TextureTarget = RT;
    DroneSceneCapture->FOVAngle = CameraFOV;

    Api = NewObject<UDroneApi>(this);
    Api->Initialize(this);
    UAgentManager::GetInstance()->RegisterAgent(DroneId, this);
    UE_LOG(LogTemp, Log, TEXT("[DronePawn] BeginPlay: %s at %s, Camera %dx%d FOV=%.0f"),
        *DroneId, *GetActorLocation().ToString(), CameraWidth, CameraHeight, CameraFOV);
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
    UpdateCameraRotation(DeltaTime);
    SyncPostProcessToCapture();
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

// ── 摄像头云台控制 ──────────────────────────────────────

/**
 * @brief 设置摄像头云台目标角度
 * @param TargetPitch 目标俯仰角
 * @param TargetYaw 目标偏航角
 */
void ADronePawn::SetCameraAngles(float TargetPitch, float TargetYaw)
{
    CameraTargetPitch = TargetPitch;
    CameraTargetYaw = TargetYaw;
}

/**
 * @brief 更新摄像头 Yaw/Pitch 旋转插值
 * @param DeltaTime 帧间隔时间
 * Yaw 旋转应用到 CameraYawMesh，Pitch 旋转应用到 CameraPitchMesh
 */
void ADronePawn::UpdateCameraRotation(float DeltaTime)
{
    // Yaw 插值（处理 -180~180 绕圈问题）
    float YawDiff = FRotator::NormalizeAxis(CameraTargetYaw - CameraCurrentYaw);
    float AdjustedTargetYaw = CameraCurrentYaw + YawDiff;
    CameraCurrentYaw = FMath::FInterpTo(CameraCurrentYaw, AdjustedTargetYaw, DeltaTime, CameraRotationSpeed);

    // Pitch 插值
    CameraCurrentPitch = FMath::FInterpTo(CameraCurrentPitch, CameraTargetPitch, DeltaTime, CameraRotationSpeed);

    // 应用旋转到云台网格
    if (CameraYawMesh) CameraYawMesh->SetRelativeRotation(FRotator(0.0f, CameraCurrentYaw, 0.0f));
    if (CameraPitchMesh) CameraPitchMesh->SetRelativeRotation(FRotator(0.0f, 0.0f, -CameraCurrentPitch));
}

/** @brief 将 CineCamera 的 PostProcess 设置同步到 SceneCapture */
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
}

/**
 * @brief 采集一帧图像并返回 Base64 编码的 JPEG 字符串
 * @param Quality JPEG 压缩质量
 * @return Base64 编码的 JPEG 数据
 * 手动触发 SceneCapture 采集一帧 → 读取像素 → JPEG 编码 → Base64
 */
FString ADronePawn::CaptureImageBase64(int32 Quality)
{
    if (Quality <= 0) Quality = JpegQuality;
    if (!DroneSceneCapture || !DroneSceneCapture->TextureTarget) return TEXT("");
    DroneSceneCapture->CaptureScene();
    FTextureRenderTargetResource* Resource = DroneSceneCapture->TextureTarget->GameThread_GetRenderTargetResource();
    if (!Resource) return TEXT("");
    TArray<FColor> Pixels;
    int32 W = CameraWidth;
    int32 H = CameraHeight;
    Pixels.SetNum(W * H);
    FReadSurfaceDataFlags ReadFlags(RCM_UNorm, CubeFace_MAX);
    ReadFlags.SetLinearToGamma(false);
    if (!Resource->ReadPixels(Pixels, ReadFlags)) return TEXT("");
    IImageWrapperModule& ImgModule = FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));
    TSharedPtr<IImageWrapper> Wrapper = ImgModule.CreateImageWrapper(EImageFormat::JPEG);
    TArray<uint8> RawData;
    RawData.SetNum(Pixels.Num() * 4);
    for (int32 i = 0; i < Pixels.Num(); i++)
    {
        RawData[i * 4 + 0] = Pixels[i].B;
        RawData[i * 4 + 1] = Pixels[i].G;
        RawData[i * 4 + 2] = Pixels[i].R;
        RawData[i * 4 + 3] = Pixels[i].A;
    }
    if (!Wrapper.IsValid()) return TEXT("");
    if (!Wrapper->SetRaw(RawData.GetData(), RawData.Num(), W, H, ERGBFormat::BGRA, 8)) return TEXT("");
    auto JpegData = Wrapper->GetCompressed(Quality);
    return FBase64::Encode(JpegData.GetData(), JpegData.Num());
}

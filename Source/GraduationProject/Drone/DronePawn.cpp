#include "DronePawn.h"

#include "DroneApi.h"
#include "DroneMovementComponent.h"
#include "GraduationProject/Core/Manager/AgentManager.h"
#include "GraduationProject/Vision/CameraCaptureUtils.h"

namespace
{
    /**
     * @brief 坐标/姿态参数所使用的参考坐标系
     *
     * - `UE`：直接采用 Unreal 世界坐标约定；
     * - `NED`：采用 North-East-Down 约定，需要对竖直轴和相关姿态分量做符号转换。
     */
    enum class EDroneFrame : uint8
    {
        UE,
        NED,
    };

    /** @brief 对 JSON 字符串中的反斜杠和双引号做转义 */
    FString JsonEscape(const FString& In)
    {
        FString Out = In;
        Out.ReplaceInline(TEXT("\\"), TEXT("\\\\"));
        Out.ReplaceInline(TEXT("\""), TEXT("\\\""));
        return Out;
    }

    /** @brief 解析坐标系字符串，无法识别时默认回落到 `UE` */
    EDroneFrame ParseFrame(const FString& FrameText)
    {
        FString Frame = FrameText;
        Frame.TrimStartAndEndInline();
        Frame.ToLowerInline();
        return (Frame == TEXT("ned")) ? EDroneFrame::NED : EDroneFrame::UE;
    }

    /** @brief 坐标系枚举转字符串，便于状态 JSON 输出 */
    FString FrameToString(EDroneFrame Frame)
    {
        return (Frame == EDroneFrame::NED) ? TEXT("ned") : TEXT("ue");
    }

    /**
     * @brief 将外部输入向量转换为 UE 坐标
     * @param Value 原始向量
     * @param Frame 输入坐标系
     * @return UE 坐标下的向量
     *
     * 当前 `NED -> UE` 转换的核心为：
     * $z_{ue} = -z_{ned}$。
     */
    FVector ConvertInputToUE(const FVector& Value, EDroneFrame Frame)
    {
        if (Frame == EDroneFrame::NED)
        {
            return FVector(Value.X, Value.Y, -Value.Z);
        }
        return Value;
    }

    /** @brief 将 UE 内部向量转换为外部请求的输出坐标系 */
    FVector ConvertUEToOutput(const FVector& Value, EDroneFrame Frame)
    {
        if (Frame == EDroneFrame::NED)
        {
            return FVector(Value.X, Value.Y, -Value.Z);
        }
        return Value;
    }

    /**
     * @brief 将 UE 姿态角转换为目标坐标系表示
     * @param RotUE UE 坐标下的姿态
     * @param Frame 目标坐标系
     * @return 转换后的欧拉角
     *
     * 在当前约定下，`NED` 模式对俯仰与横滚取反，
     * 以匹配常见飞控/航空坐标定义。
     */
    FRotator ConvertUEToFrameRotator(const FRotator& RotUE, EDroneFrame Frame)
    {
        if (Frame == EDroneFrame::NED)
        {
            return FRotator(-RotUE.Pitch, RotUE.Yaw, -RotUE.Roll);
        }
        return RotUE;
    }

    /** @brief 将外部姿态角转换回 UE 旋转约定 */
    FRotator ConvertFrameToUERotator(const FRotator& RotFrame, EDroneFrame Frame)
    {
        if (Frame == EDroneFrame::NED)
        {
            return FRotator(-RotFrame.Pitch, RotFrame.Yaw, -RotFrame.Roll);
        }
        return RotFrame;
    }

    /** @brief 偏航控制模式转字符串，便于状态序列化 */
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

    /** @brief 驱动模式转字符串 */
    FString DrivetrainToString(EDroneDrivetrainMode Mode)
    {
        return (Mode == EDroneDrivetrainMode::MaxDegreeOfFreedom)
            ? TEXT("max_degree_of_freedom")
            : TEXT("forward_only");
    }

    /** @brief 任务角色转字符串 */
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

    /** @brief 控制模式转字符串 */
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

/**
 * @brief 构造无人机 Pawn 并创建全部可视化/控制子组件
 *
 * 这里只完成组件装配和默认属性设置，
 * 不做运行时注册、纹理创建或动力学状态初始化。
 */
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

/**
 * @brief 初始化动力学、图像采集与 API 组件
 *
 * 主要完成三件事：
 * 1. 将当前 Actor 初始位置写入动力学状态，注意 `cm -> m` 的单位换算；
 * 2. 为机载相机创建 RenderTarget，并同步分割模板值；
 * 3. 创建高层 API 对象并向 AgentManager 注册当前无人机。
 */
void ADronePawn::BeginPlay()
{
    Super::BeginPlay();

    if (MovementComp)
    {
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

/**
 * @brief 每帧同步飞行状态、旋翼动画和云台姿态
 * @param DeltaTime 帧间隔（s）
 */
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

/**
 * @brief 将动力学状态写回 Actor 世界变换
 * @param State 当前飞行状态
 *
 * 动力学内部以米为单位，UE 世界位置以厘米为单位，故有：
 * $p_{ue} = 100 \cdot p_{sim}$。
 */
void ADronePawn::ApplyStateToActor(const FDroneState& State)
{
    SetActorLocationAndRotation(State.GetPosition() * 100.0f, State.GetRotator());
}

/**
 * @brief 根据电机角速度刷新旋翼可视化动画
 * @param DeltaTime 帧间隔（s）
 *
 * 若电机角速度为 $\omega$（rad/s），则：
 * - `RPM = \omega * 60 / (2\pi)`
 * - $\Delta\theta = RPM * 360 / 60 * \Delta t = \omega * 180 / \pi * \Delta t$
 *
 * 同时通过奇偶号电机的方向符号差异模拟顺/逆时针旋转。
 */
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

/** @brief 按索引获取对应旋翼网格组件 */
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

/**
 * @brief 切换到位置控制模式并设置目标位置
 * @param NewTargetPosition 目标位置（m）
 * @param Speed 期望速度上限（m/s）
 * @param Frame 输入坐标系
 */
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

/**
 * @brief 切换到速度控制模式并设置目标速度
 * @param NewTargetVelocity 目标速度（m/s）
 * @param Frame 输入坐标系
 */
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

/** @brief 标量形式的速度控制便捷封装 */
void ADronePawn::MoveByVelocity(float Vx, float Vy, float Vz, FString Frame)
{
    SetTargetVelocity(FVector(Vx, Vy, Vz), MoveTemp(Frame));
}

/** @brief 仅更新偏航控制策略，不直接修改平移控制目标 */
void ADronePawn::SetHeadingControl(EDroneYawMode NewYawMode, EDroneDrivetrainMode NewDrivetrain, float YawDeg)
{
    if (MovementComp)
    {
        MovementComp->SetHeadingControl(NewYawMode, NewDrivetrain, YawDeg);
    }
}

/** @brief 当前位置悬停，本质上是把当前位置重新设为位置控制目标 */
void ADronePawn::Hover()
{
    SetTargetPosition(CurrentState.GetPosition());
}

/** @brief 在当前平面位置上抬升到指定高度 */
void ADronePawn::Takeoff(float Altitude)
{
    const FVector CurrentPos = CurrentState.GetPosition();
    SetTargetPosition(FVector(CurrentPos.X, CurrentPos.Y, Altitude));
}

/** @brief 在当前平面位置下降到地面高度 `z=0` */
void ADronePawn::Land()
{
    const FVector CurrentPos = CurrentState.GetPosition();
    SetTargetPosition(FVector(CurrentPos.X, CurrentPos.Y, 0.0f));
}

/** @brief 更新远程 API 控制标志位 */
void ADronePawn::EnableApiControl(bool bEnable)
{
    bApiControlEnabled = bEnable;
}

/**
 * @brief 强制把无人机重置到指定位置和姿态
 * @param NewLocation 目标位置（m）
 * @param NewRotation 目标姿态
 * @param Frame 输入坐标系
 *
 * 该接口会同时：
 * - 重置动力学组件内部状态；
 * - 将控制模式恢复到 `Idle`；
 * - 立即把新状态同步到 Actor 变换。
 */
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

/** @brief 重置 API 层内部控制状态，但不直接改动当前位置 */
void ADronePawn::ResetActorState()
{
    if (Api)
    {
        Api->Reset();
    }
}

/**
 * @brief 设置姿态角与总推力控制目标
 * @param RollDeg 横滚角（deg）
 * @param PitchDeg 俯仰角（deg）
 * @param YawDeg 偏航角（deg）
 * @param Thrust 总推力（N）
 * @param Frame 输入坐标系
 *
 * 外部常用 `NED` 姿态表达，因此这里先完成坐标系转换，
 * 再交给 `UDroneApi` 写入姿态控制链路。
 */
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

/** @brief 直接下发四路电机角速度控制命令 */
void ADronePawn::SetMotorSpeeds(float M0, float M1, float M2, float M3)
{
    if (!Api)
    {
        return;
    }

    Api->SetMotorSpeeds(M0, M1, M2, M3);
    ControlMode = EDroneControlMode::MotorSpeed;
}

/** @brief 设置位置环控制器增益 */
void ADronePawn::SetPositionControllerGains(float Kp, float Kd)
{
    if (Api)
    {
        Api->SetPositionControllerGains(Kp, Kd);
    }
}

/** @brief 设置速度环控制器增益 */
void ADronePawn::SetVelocityControllerGains(float Kp, float Ki, float Kd)
{
    if (Api)
    {
        Api->SetVelocityControllerGains(Kp, Ki, Kd);
    }
}

/** @brief 设置姿态环控制器增益 */
void ADronePawn::SetAttitudeControllerGains(float Kp, float Kd)
{
    if (Api)
    {
        Api->SetAttitudeControllerGains(Kp, Kd);
    }
}

/** @brief 设置角速度环比例增益 */
void ADronePawn::SetAngleRateControllerGains(float Kp)
{
    if (Api)
    {
        Api->SetAngleRateControllerGains(Kp);
    }
}

/** @brief 返回当前状态中的位置分量 */
FVector ADronePawn::GetCurrentPosition() const
{
    return CurrentState.GetPosition();
}

/** @brief 返回当前状态中的速度分量 */
FVector ADronePawn::GetCurrentVelocity() const
{
    return CurrentState.GetVelocity();
}

/** @brief 写入云台目标角，实际转动由 Tick 中插值完成 */
void ADronePawn::SetCameraAngles(float TargetPitch, float TargetYaw)
{
    CameraTargetPitch = TargetPitch;
    CameraTargetYaw = TargetYaw;
}

/** @brief 更新该机体的语义分割模板值并同步到场景捕获对象 */
void ADronePawn::SetSegmentationId(int32 NewSegmentationId)
{
    SegmentationId = FMath::Clamp(NewSegmentationId, 0, 255);
    CameraCaptureUtils::ApplySegmentationStencil(this, SegmentationId);
}

/**
 * @brief 以 JSON 形式导出当前无人机状态
 * @param Frame 输出坐标系
 * @return 状态快照 JSON
 *
 * 该接口会按请求坐标系转换位置、速度与姿态，
 * 并附带电机转速、偏航模式、任务角色、云台角和控制模式等信息。
 */
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

/** @brief 调用图像工具抓取当前机载相机画面，并按 AirSim 风格封装成 JSON */
FString ADronePawn::GetImage(FString ImageType, int32 Quality, float MaxDepthMeters)
{
    return CameraCaptureUtils::CaptureAirSimImageJson(
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

/** @brief 抓取彩色图像并返回 Base64 JPEG */
FString ADronePawn::CaptureImageBase64(int32 Quality)
{
    if (Quality <= 0)
    {
        Quality = JpegQuality;
    }

    return CameraCaptureUtils::CaptureColorJpegBase64(DroneSceneCapture, CameraWidth, CameraHeight, Quality);
}

/**
 * @brief 以一阶插值方式平滑更新云台偏航/俯仰角
 * @param DeltaTime 帧间隔（s）
 *
 * 偏航先通过 `NormalizeAxis` 求最短角差，再做插值，
 * 避免跨越 `-180/180` 度边界时出现跳变。
 */
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
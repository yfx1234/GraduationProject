#include "VisualInterceptController.h"

#include "Dom/JsonObject.h"
#include "GraduationProject/Core/Controller/PIDController.h"
#include "GraduationProject/Core/Manager/AgentManager.h"
#include "GraduationProject/Drone/DronePawn.h"
#include "KalmanPredictor.h"

namespace
{
    /** @brief 视觉拦截默认时间步长（s） */
    constexpr float kDefaultDt = 0.08f;

    /**
     * @brief 从 JSON 字段中读取数值并转换为目标类型
     * @param Obj JSON 对象
     * @param Field 字段名
     * @param OutValue 输出值
     * @return 字段存在且读取成功时返回 true
     */
    template <typename T>
    bool TryGetNumberAs(const TSharedPtr<FJsonObject>& Obj, const TCHAR* Field, T& OutValue)
    {
        if (!Obj.IsValid() || !Obj->HasField(Field))
        {
            return false;
        }

        OutValue = static_cast<T>(Obj->GetNumberField(Field));
        return true;
    }

    /**
     * @brief 按任务角色查找无人机
     * @param Manager AgentManager 实例
     * @param Role 目标角色
     * @param ExcludeId 需要排除的无人机 ID
     */
    ADronePawn* FindDroneByRole(UAgentManager* Manager, EDroneMissionRole Role, const FString& ExcludeId = TEXT(""))
    {
        if (!Manager)
        {
            return nullptr;
        }

        const TArray<FString> AgentIds = Manager->GetAllAgentIds();
        for (const FString& AgentId : AgentIds)
        {
            if (!ExcludeId.IsEmpty() && AgentId == ExcludeId)
            {
                continue;
            }

            ADronePawn* Drone = Cast<ADronePawn>(Manager->GetAgent(AgentId));
            if (Drone && Drone->MissionRole == Role)
            {
                return Drone;
            }
        }

        return nullptr;
    }
}

/**
 * @brief 惰性初始化预测器与三个 PID 控制器
 *
 * - `FeaturePredictor`：对 `(cx, cy, area)` 做 Kalman 平滑与短时预测；
 * - `YawPid`：控制偏航纠偏；
 * - `VerticalPid`：控制垂向速度；
 * - `ForwardPid`：根据目标面积控制前冲速度。
 */
void UVisualInterceptController::EnsureInitialized()
{
    if (!FeaturePredictor)
    {
        FeaturePredictor = NewObject<UKalmanPredictor>(this);
        FeaturePredictor->Initialize(25.0f, 40.0f);
    }

    if (!YawPid)
    {
        YawPid = NewObject<UPIDController>(this);
        YawPid->Initialize(42.0, 0.0, 12.0, 0.03, Params.MaxYawRateDeg, 0.02, -200.0, 200.0);
    }

    if (!VerticalPid)
    {
        VerticalPid = NewObject<UPIDController>(this);
        VerticalPid->Initialize(2.2, 0.0, 0.7, 0.05, Params.MaxVerticalSpeed, 0.02, -5.0, 5.0);
    }

    if (!ForwardPid)
    {
        ForwardPid = NewObject<UPIDController>(this);
        ForwardPid->Initialize(80.0, 0.0, 22.0, 0.05, Params.MaxForwardSpeed, 0.02, -5.0, 5.0);
    }
}

/** @brief 仅重置会话期运行数据，不触碰 PID 与预测器参数 */
void UVisualInterceptController::ResetRuntimeOnly()
{
    Runtime = FVisualRuntime();
}

/** @brief 清空三个 PID 控制器的积分项和历史误差 */
void UVisualInterceptController::ResetPidControllers()
{
    if (YawPid)
    {
        YawPid->Reset();
    }
    if (VerticalPid)
    {
        VerticalPid->Reset();
    }
    if (ForwardPid)
    {
        ForwardPid->Reset();
    }
}

/**
 * @brief 完全重置视觉拦截控制器
 *
 * 该接口会重置：
 * - 特征 Kalman 预测器；
 * - 三个 PID 控制器；
 * - 当前运行时状态机与统计量。
 */
void UVisualInterceptController::Reset()
{
    EnsureInitialized();

    if (FeaturePredictor)
    {
        FeaturePredictor->Reset();
        FeaturePredictor->Initialize(25.0f, 40.0f);
    }

    ResetPidControllers();
    ResetRuntimeOnly();
}

/**
 * @brief 启动新的视觉拦截会话并初始化运行时状态
 * @param Interceptor 当前选中的拦截无人机
 */
void UVisualInterceptController::BeginSession(const ADronePawn* Interceptor)
{
    Runtime.bEnabled = true;
    Runtime.bCaptured = false;
    Runtime.State = EVisualState::Search;
    Runtime.FrameCount = 0;
    Runtime.DetectionCount = 0;
    Runtime.LostCount = 0;
    Runtime.CaptureCount = 0;
    Runtime.LastEx = 0.0f;
    Runtime.LastEy = 0.0f;
    Runtime.LastAreaNorm = 0.0f;
    Runtime.LastConf = 0.0f;
    Runtime.LastYawCmdDeg = Interceptor ? Interceptor->CurrentState.GetRotator().Yaw : 0.0f;
    Runtime.LastYawRateDeg = 0.0f;
    Runtime.LastCmdVel = FVector::ZeroVector;
    Runtime.LastTargetDistance = -1.0f;
    Runtime.SearchCamYawDeg = 0.0f;
    Runtime.SearchDir = 1.0f;
    Runtime.SearchTime = 0.0f;
}

/** @brief 布尔值转 JSON 字面量文本 */
FString UVisualInterceptController::BoolLiteral(bool bValue)
{
    return bValue ? TEXT("true") : TEXT("false");
}

/** @brief 将偏航角归一化到 `[-180, 180)` 区间 */
float UVisualInterceptController::NormalizeYawDeg(float YawDeg)
{
    return FRotator::NormalizeAxis(YawDeg);
}

/** @brief 视觉状态机枚举转字符串 */
FString UVisualInterceptController::StateToString(EVisualState State)
{
    switch (State)
    {
    case EVisualState::Search:
        return TEXT("SEARCH");
    case EVisualState::Track:
        return TEXT("TRACK");
    case EVisualState::Approach:
        return TEXT("APPROACH");
    case EVisualState::Captured:
        return TEXT("CAPTURED");
    case EVisualState::Idle:
    default:
        return TEXT("IDLE");
    }
}

/**
 * @brief 规范化视觉拦截方法名
 * @param InMethod 原始方法名
 * @param OutCanonical 输出的标准方法名
 * @return 是否为支持的方法
 */
bool UVisualInterceptController::NormalizeMethod(const FString& InMethod, FString& OutCanonical)
{
    FString Method = InMethod;
    Method.TrimStartAndEndInline();
    Method.ToLowerInline();

    if (Method.IsEmpty() || Method == TEXT("auto"))
    {
        OutCanonical = TEXT("vision_pid_kalman");
        return true;
    }

    if (Method == TEXT("vision_pid_kalman") || Method == TEXT("visual_pid_kalman") || Method == TEXT("pid_kalman"))
    {
        OutCanonical = TEXT("vision_pid_kalman");
        return true;
    }

    if (Method == TEXT("vision_pid") || Method == TEXT("visual_pid") || Method == TEXT("pid"))
    {
        OutCanonical = TEXT("vision_pid");
        return true;
    }

    if (Method == TEXT("vision_pd_kalman") || Method == TEXT("visual_pd_kalman") || Method == TEXT("pd_kalman"))
    {
        OutCanonical = TEXT("vision_pd_kalman");
        return true;
    }

    if (Method == TEXT("vision_pd") || Method == TEXT("visual_pd") || Method == TEXT("pd"))
    {
        OutCanonical = TEXT("vision_pd");
        return true;
    }

    return false;
}

/** @brief 生成统一错误返回 JSON */
FString UVisualInterceptController::MakeError(const FString& Msg) const
{
    return FString::Printf(TEXT("{\"status\":\"error\",\"message\":\"%s\"}"), *Msg.ReplaceCharWithEscapedChar());
}

/** @brief 生成统一成功返回 JSON */
FString UVisualInterceptController::MakeOk(const FString& Msg) const
{
    return FString::Printf(TEXT("{\"status\":\"ok\",\"message\":\"%s\"}"), *Msg.ReplaceCharWithEscapedChar());
}

/**
 * @brief 对时间步长做有效性检查与限幅
 * @param Dt 原始时间步长（s）
 * @return 裁剪后的时间步长，范围 `[0.01, 0.25]`
 */
float UVisualInterceptController::SafeDt(float Dt) const
{
    if (!FMath::IsFinite(Dt))
    {
        Dt = kDefaultDt;
    }
    return FMath::Clamp(Dt, 0.01f, 0.25f);
}

/** @brief 将当前时间步长同步到三个 PID 控制器 */
void UVisualInterceptController::SyncPidTimeStep(float Dt)
{
    if (YawPid)
    {
        YawPid->SetTimeStep(Dt);
    }
    if (VerticalPid)
    {
        VerticalPid->SetTimeStep(Dt);
    }
    if (ForwardPid)
    {
        ForwardPid->SetTimeStep(Dt);
    }
}

/**
 * @brief 从 JSON 命令中读取并校验全部视觉拦截参数
 * @param CmdObj JSON 命令对象
 *
 * 读入后会统一执行限幅，并把新的限幅值同步回 PID 输出上限。
 */
void UVisualInterceptController::ApplyParamsFromJson(const TSharedPtr<FJsonObject>& CmdObj)
{
    if (!CmdObj.IsValid())
    {
        return;
    }

    TryGetNumberAs(CmdObj, TEXT("desired_area"), Params.DesiredArea);
    TryGetNumberAs(CmdObj, TEXT("capture_area"), Params.CaptureArea);
    TryGetNumberAs(CmdObj, TEXT("center_tol_x"), Params.CenterTolX);
    TryGetNumberAs(CmdObj, TEXT("center_tol_y"), Params.CenterTolY);
    TryGetNumberAs(CmdObj, TEXT("capture_hold_frames"), Params.CaptureHoldFrames);
    TryGetNumberAs(CmdObj, TEXT("lost_to_search_frames"), Params.LostToSearchFrames);

    TryGetNumberAs(CmdObj, TEXT("max_forward_speed"), Params.MaxForwardSpeed);
    TryGetNumberAs(CmdObj, TEXT("max_reverse_speed"), Params.MaxReverseSpeed);
    TryGetNumberAs(CmdObj, TEXT("max_vertical_speed"), Params.MaxVerticalSpeed);
    TryGetNumberAs(CmdObj, TEXT("max_yaw_rate_deg"), Params.MaxYawRateDeg);
    TryGetNumberAs(CmdObj, TEXT("ram_area_target"), Params.RamAreaTarget);
    TryGetNumberAs(CmdObj, TEXT("min_ram_speed"), Params.MinRamSpeed);
    TryGetNumberAs(CmdObj, TEXT("intercept_distance"), Params.InterceptDistance);

    TryGetNumberAs(CmdObj, TEXT("search_cam_yaw_limit_deg"), Params.SearchCamYawLimitDeg);
    TryGetNumberAs(CmdObj, TEXT("search_cam_rate_deg"), Params.SearchCamRateDeg);
    TryGetNumberAs(CmdObj, TEXT("search_body_yaw_rate_deg"), Params.SearchBodyYawRateDeg);
    TryGetNumberAs(CmdObj, TEXT("search_cam_pitch_deg"), Params.SearchCamPitchDeg);
    TryGetNumberAs(CmdObj, TEXT("search_vz_amp"), Params.SearchVzAmp);

    if (CmdObj->HasField(TEXT("stop_on_capture")))
    {
        Params.bStopOnCapture = CmdObj->GetBoolField(TEXT("stop_on_capture"));
    }
    if (CmdObj->HasField(TEXT("use_kalman")))
    {
        Params.bUseKalman = CmdObj->GetBoolField(TEXT("use_kalman"));
    }

    Params.DesiredArea = FMath::Clamp(Params.DesiredArea, 0.0001f, 0.95f);
    Params.CaptureArea = FMath::Clamp(Params.CaptureArea, Params.DesiredArea, 0.98f);
    Params.CenterTolX = FMath::Clamp(Params.CenterTolX, 0.001f, 1.0f);
    Params.CenterTolY = FMath::Clamp(Params.CenterTolY, 0.001f, 1.0f);
    Params.CaptureHoldFrames = FMath::Max(1, Params.CaptureHoldFrames);
    Params.LostToSearchFrames = FMath::Max(1, Params.LostToSearchFrames);

    Params.MaxForwardSpeed = FMath::Max(0.1f, Params.MaxForwardSpeed);
    Params.MaxReverseSpeed = FMath::Max(0.0f, Params.MaxReverseSpeed);
    Params.MaxVerticalSpeed = FMath::Max(0.1f, Params.MaxVerticalSpeed);
    Params.MaxYawRateDeg = FMath::Max(1.0f, Params.MaxYawRateDeg);
    Params.RamAreaTarget = FMath::Clamp(FMath::Max(Params.RamAreaTarget, Params.CaptureArea), Params.CaptureArea, 0.65f);
    Params.MinRamSpeed = FMath::Clamp(Params.MinRamSpeed, 0.0f, Params.MaxForwardSpeed);
    Params.InterceptDistance = FMath::Clamp(Params.InterceptDistance, 0.2f, 20.0f);

    Params.SearchCamYawLimitDeg = FMath::Clamp(Params.SearchCamYawLimitDeg, 10.0f, 179.0f);
    Params.SearchCamRateDeg = FMath::Clamp(Params.SearchCamRateDeg, 1.0f, 180.0f);
    Params.SearchBodyYawRateDeg = FMath::Clamp(Params.SearchBodyYawRateDeg, 0.0f, 120.0f);
    Params.SearchCamPitchDeg = FMath::Clamp(Params.SearchCamPitchDeg, -80.0f, 45.0f);
    Params.SearchVzAmp = FMath::Clamp(Params.SearchVzAmp, 0.0f, 5.0f);

    if (YawPid)
    {
        YawPid->SetParameters(YawPid->Kp, YawPid->Ki, YawPid->Kd, YawPid->DiffFilterTau, Params.MaxYawRateDeg);
    }
    if (VerticalPid)
    {
        VerticalPid->SetParameters(VerticalPid->Kp, VerticalPid->Ki, VerticalPid->Kd, VerticalPid->DiffFilterTau, Params.MaxVerticalSpeed);
    }
    if (ForwardPid)
    {
        ForwardPid->SetParameters(ForwardPid->Kp, ForwardPid->Ki, ForwardPid->Kd, ForwardPid->DiffFilterTau, Params.MaxForwardSpeed);
    }
}

/**
 * @brief 解析并锁定拦截无人机
 * @param CmdObj 当前命令对象
 * @return 成功解析出的拦截无人机
 *
 * 查找优先级：
 * 1. JSON 中给定的 `interceptor_id`；
 * 2. `MissionRole == Interceptor`；
 * 3. 兜底 `drone_1`；
 * 4. 最后尝试 `drone_0`。
 */
ADronePawn* UVisualInterceptController::ResolveInterceptor(const TSharedPtr<FJsonObject>& CmdObj)
{
    UAgentManager* Manager = UAgentManager::GetInstance();
    if (!Manager)
    {
        return nullptr;
    }

    FString DesiredId = Runtime.InterceptorId;
    if (CmdObj.IsValid() && CmdObj->HasField(TEXT("interceptor_id")))
    {
        DesiredId = CmdObj->GetStringField(TEXT("interceptor_id"));
    }

    ADronePawn* Drone = nullptr;
    if (!DesiredId.IsEmpty())
    {
        Drone = Cast<ADronePawn>(Manager->GetAgent(DesiredId));
    }
    if (!Drone)
    {
        Drone = FindDroneByRole(Manager, EDroneMissionRole::Interceptor);
    }
    if (!Drone)
    {
        Drone = Cast<ADronePawn>(Manager->GetAgent(TEXT("drone_1")));
    }
    if (!Drone)
    {
        Drone = Cast<ADronePawn>(Manager->GetAgent(TEXT("drone_0")));
    }

    if (Drone)
    {
        Runtime.InterceptorId = Drone->DroneId;
    }
    return Drone;
}

/**
 * @brief 解析并锁定目标无人机
 * @param CmdObj 当前命令对象
 * @return 成功解析出的目标无人机
 */
ADronePawn* UVisualInterceptController::ResolveTarget(const TSharedPtr<FJsonObject>& CmdObj)
{
    UAgentManager* Manager = UAgentManager::GetInstance();
    if (!Manager)
    {
        return nullptr;
    }

    FString DesiredId = Runtime.TargetId;
    if (CmdObj.IsValid() && CmdObj->HasField(TEXT("target_id")))
    {
        DesiredId = CmdObj->GetStringField(TEXT("target_id"));
    }

    ADronePawn* Drone = nullptr;
    if (!DesiredId.IsEmpty())
    {
        Drone = Cast<ADronePawn>(Manager->GetAgent(DesiredId));
    }
    if (!Drone)
    {
        Drone = FindDroneByRole(Manager, EDroneMissionRole::Target, Runtime.InterceptorId);
    }
    if (!Drone)
    {
        const TCHAR* FallbackIds[] = {TEXT("drone_0"), TEXT("drone_1")};
        for (const TCHAR* FallbackId : FallbackIds)
        {
            if (!Runtime.InterceptorId.IsEmpty() && Runtime.InterceptorId == FallbackId)
            {
                continue;
            }
            Drone = Cast<ADronePawn>(Manager->GetAgent(FallbackId));
            if (Drone)
            {
                break;
            }
        }
    }

    if (Drone)
    {
        Runtime.TargetId = Drone->DroneId;
    }
    return Drone;
}

/** @brief 构造启动响应 JSON，返回当前参数快照与会话标识 */
FString UVisualInterceptController::BuildStartJson() const
{
    return FString::Printf(
        TEXT("{\"status\":\"ok\",\"mode\":\"visual_intercept\",\"enabled\":%s,\"state\":\"%s\",\"interceptor_id\":\"%s\",\"target_id\":\"%s\",\"method\":\"%s\",\"params\":{\"desired_area\":%.5f,\"capture_area\":%.5f,\"center_tol_x\":%.4f,\"center_tol_y\":%.4f,\"capture_hold_frames\":%d,\"lost_to_search_frames\":%d,\"max_forward_speed\":%.3f,\"max_reverse_speed\":%.3f,\"max_vertical_speed\":%.3f,\"max_yaw_rate_deg\":%.3f,\"ram_area_target\":%.5f,\"min_ram_speed\":%.3f,\"intercept_distance\":%.3f,\"search_cam_yaw_limit_deg\":%.3f,\"search_cam_rate_deg\":%.3f,\"search_body_yaw_rate_deg\":%.3f,\"search_cam_pitch_deg\":%.3f,\"search_vz_amp\":%.3f,\"stop_on_capture\":%s,\"use_kalman\":%s}}"),
        *BoolLiteral(Runtime.bEnabled),
        *StateToString(Runtime.State),
        *Runtime.InterceptorId,
        *Runtime.TargetId,
        *Runtime.Method,
        Params.DesiredArea,
        Params.CaptureArea,
        Params.CenterTolX,
        Params.CenterTolY,
        Params.CaptureHoldFrames,
        Params.LostToSearchFrames,
        Params.MaxForwardSpeed,
        Params.MaxReverseSpeed,
        Params.MaxVerticalSpeed,
        Params.MaxYawRateDeg,
        Params.RamAreaTarget,
        Params.MinRamSpeed,
        Params.InterceptDistance,
        Params.SearchCamYawLimitDeg,
        Params.SearchCamRateDeg,
        Params.SearchBodyYawRateDeg,
        Params.SearchCamPitchDeg,
        Params.SearchVzAmp,
        *BoolLiteral(Params.bStopOnCapture),
        *BoolLiteral(Params.bUseKalman));
}

/** @brief 构造每帧更新响应 JSON，返回控制量与统计状态 */
FString UVisualInterceptController::BuildUpdateJson(bool bValidControl) const
{
    return FString::Printf(
        TEXT("{\"status\":\"ok\",\"mode\":\"visual_intercept\",\"enabled\":%s,\"state\":\"%s\",\"captured\":%s,\"valid\":%s,\"interceptor_id\":\"%s\",\"target_id\":\"%s\",\"method\":\"%s\",\"frame\":%d,\"detections\":%d,\"lost_count\":%d,\"capture_count\":%d,\"target_distance\":%.4f,\"cmd_velocity\":[%.4f,%.4f,%.4f],\"control\":{\"ex\":%.5f,\"ey\":%.5f,\"area_norm\":%.6f,\"conf\":%.4f,\"yaw_cmd_deg\":%.4f,\"yaw_rate_deg\":%.4f}}"),
        *BoolLiteral(Runtime.bEnabled),
        *StateToString(Runtime.State),
        *BoolLiteral(Runtime.bCaptured),
        *BoolLiteral(bValidControl),
        *Runtime.InterceptorId,
        *Runtime.TargetId,
        *Runtime.Method,
        Runtime.FrameCount,
        Runtime.DetectionCount,
        Runtime.LostCount,
        Runtime.CaptureCount,
        Runtime.LastTargetDistance,
        Runtime.LastCmdVel.X,
        Runtime.LastCmdVel.Y,
        Runtime.LastCmdVel.Z,
        Runtime.LastEx,
        Runtime.LastEy,
        Runtime.LastAreaNorm,
        Runtime.LastConf,
        Runtime.LastYawCmdDeg,
        Runtime.LastYawRateDeg);
}

/**
 * @brief 根据视觉特征计算跟踪/逼近控制量
 * @param Interceptor 拦截无人机
 * @param Feature 视觉特征 `(cx, cy, area)`
 * @param FrameW 图像宽度（px）
 * @param FrameH 图像高度（px）
 * @param Confidence 当前检测置信度
 * @param Dt 时间步长（s）
 * @param bDetectionReal 本帧特征是否来自真实检测，false 表示来自 Kalman 预测
 * @param bOutCaptured [out] 本函数输出的捕获标志，当前实现固定由外层距离判定
 * @return 是否成功生成控制量
 *
 * 主要计算流程如下：
 * 1. 像面误差归一化：
 *    - $e_x = (cx - W/2) / (W/2)$
 *    - $e_y = (cy - H/2) / (H/2)$
 *    - $a = area / (W \cdot H)$
 * 2. 用相机 FOV 将水平误差映射为视线偏角：
 *    - $yaw_{img} = e_x \cdot FOV_x / 2$
 *    - $yaw_{los} = yaw_{body} + yaw_{cam} + yaw_{img}$
 * 3. 用垂向误差驱动垂直速度，用面积误差驱动前冲速度；
 * 4. 机体偏航采用受限角步进 `YawCmdDeg`，而 `YawRateCmd` 主要用于状态输出与诊断。
 */
bool UVisualInterceptController::ComputeTrackControl(
    ADronePawn* Interceptor,
    const FVector& Feature,
    float FrameW,
    float FrameH,
    float Confidence,
    float Dt,
    bool bDetectionReal,
    bool& bOutCaptured)
{
    bOutCaptured = false;

    if (!Interceptor || FrameW <= 1.0f || FrameH <= 1.0f)
    {
        return false;
    }

    const float Cx = Feature.X;
    const float Cy = Feature.Y;
    const float Area = FMath::Max(1.0f, Feature.Z);

    const float Ex = (Cx - 0.5f * FrameW) / FMath::Max(1.0f, 0.5f * FrameW);
    const float Ey = (Cy - 0.5f * FrameH) / FMath::Max(1.0f, 0.5f * FrameH);
    const float AreaNorm = Area / FMath::Max(1.0f, FrameW * FrameH);

    const float HorizontalFovDeg = FMath::Max(30.0f, Interceptor->CameraFOV);
    const float HalfHorizontalFovDeg = 0.5f * HorizontalFovDeg;
    const float Aspect = FMath::Max(0.2f, FrameW / FMath::Max(1.0f, FrameH));
    const float HalfVerticalFovDeg = FMath::RadiansToDegrees(
        FMath::Atan(FMath::Tan(FMath::DegreesToRadians(HalfHorizontalFovDeg)) / Aspect));

    const float CurrentYawDeg = Interceptor->CurrentState.GetRotator().Yaw;
    const float CurrentCameraYawDeg = Interceptor->GetCameraCurrentYaw();
    const float CurrentCameraPitchDeg = Interceptor->GetCameraCurrentPitch();
    const float ImageYawOffsetDeg = Ex * HalfHorizontalFovDeg;
    const float LineOfSightYawDeg = NormalizeYawDeg(CurrentYawDeg + CurrentCameraYawDeg + ImageYawOffsetDeg);
    const float YawErrorDeg = NormalizeYawDeg(LineOfSightYawDeg - CurrentYawDeg);
    const float TrackYawError = FMath::Clamp(
        YawErrorDeg / FMath::Max(1.0f, HalfHorizontalFovDeg), -2.0f, 2.0f);

    float YawRateCmd = static_cast<float>(YawPid ? YawPid->Update(TrackYawError, 0.0) : 0.0);
    YawRateCmd = FMath::Clamp(YawRateCmd, -Params.MaxYawRateDeg, Params.MaxYawRateDeg);

    const float DirectYawStepDeg = FMath::Clamp(YawErrorDeg, -Params.MaxYawRateDeg * Dt, Params.MaxYawRateDeg * Dt);
    const float YawCmdDeg = NormalizeYawDeg(CurrentYawDeg + DirectYawStepDeg);
    const float VelocityYawDeg = LineOfSightYawDeg;

    // 真实检测时更积极跟随，预测模式下则稍微保守一些，降低镜头抖动。
    const float CameraHoldFactor = bDetectionReal ? 0.75f : 0.92f;
    const float CameraPitchLimitDeg = 55.0f;
    const float CameraYawTargetDeg = FMath::Clamp(
        CurrentCameraYawDeg * CameraHoldFactor + ImageYawOffsetDeg,
        -Params.SearchCamYawLimitDeg,
        Params.SearchCamYawLimitDeg);
    const float CameraPitchTargetDeg = FMath::Clamp(
        CurrentCameraPitchDeg * CameraHoldFactor - Ey * HalfVerticalFovDeg,
        -CameraPitchLimitDeg,
        CameraPitchLimitDeg);

    float VzCmd = static_cast<float>(VerticalPid ? -VerticalPid->Update(Ey, 0.0) : 0.0);
    VzCmd = FMath::Clamp(VzCmd, -Params.MaxVerticalSpeed, Params.MaxVerticalSpeed);

    const float HeadingAlignment = FMath::Clamp(1.0f - 0.35f * FMath::Abs(TrackYawError), 0.25f, 1.0f);
    const float VerticalAlignment = FMath::Clamp(1.0f - 0.30f * FMath::Abs(Ey), 0.35f, 1.0f);
    const float AlignmentScale = HeadingAlignment * VerticalAlignment;

    // 面积越接近期望值，说明距离越近；因此以前向 PID 推进逼近过程。
    float ForwardCmd = static_cast<float>(
        ForwardPid ? ForwardPid->Update(Params.RamAreaTarget, AreaNorm)
                   : (Params.RamAreaTarget - AreaNorm) * Params.MaxForwardSpeed / FMath::Max(Params.RamAreaTarget, 0.001f));
    ForwardCmd = FMath::Max(0.0f, ForwardCmd);
    ForwardCmd *= AlignmentScale;
    if (bDetectionReal)
    {
        ForwardCmd = FMath::Max(ForwardCmd, Params.MinRamSpeed * HeadingAlignment);
    }
    ForwardCmd = FMath::Clamp(ForwardCmd, 0.0f, Params.MaxForwardSpeed);

    const float VelocityYawRad = FMath::DegreesToRadians(VelocityYawDeg);
    const FVector CmdVel(
        ForwardCmd * FMath::Cos(VelocityYawRad),
        ForwardCmd * FMath::Sin(VelocityYawRad),
        VzCmd);

    Interceptor->SetCameraAngles(CameraPitchTargetDeg, CameraYawTargetDeg);
    Interceptor->SetHeadingControl(EDroneYawMode::Angle, EDroneDrivetrainMode::ForwardOnly, YawCmdDeg);
    Interceptor->SetTargetVelocity(CmdVel);

    Runtime.LastEx = Ex;
    Runtime.LastEy = Ey;
    Runtime.LastAreaNorm = AreaNorm;
    Runtime.LastConf = Confidence;
    Runtime.LastYawCmdDeg = YawCmdDeg;
    Runtime.LastYawRateDeg = YawRateCmd;
    Runtime.LastCmdVel = CmdVel;
    Runtime.bCaptured = false;
    Runtime.State = (AreaNorm >= Params.CaptureArea) ? EVisualState::Approach : EVisualState::Track;

    return true;
}

/**
 * @brief 在搜索模式下生成控制量
 * @param Interceptor 拦截无人机
 * @param Dt 时间步长（s）
 *
 * 搜索模式包含三部分动作：
 * 1. 云台左右摆扫；
 * 2. 机体缓慢转向；
 * 3. 小幅垂向正弦振荡，增加重新发现目标的机会。
 */
void UVisualInterceptController::ComputeSearchControl(ADronePawn* Interceptor, float Dt)
{
    if (!Interceptor)
    {
        return;
    }

    Runtime.bCaptured = false;
    Runtime.SearchTime += Dt;

    Runtime.SearchCamYawDeg += Runtime.SearchDir * Params.SearchCamRateDeg * Dt;
    if (FMath::Abs(Runtime.SearchCamYawDeg) >= Params.SearchCamYawLimitDeg)
    {
        Runtime.SearchCamYawDeg = FMath::Clamp(Runtime.SearchCamYawDeg, -Params.SearchCamYawLimitDeg, Params.SearchCamYawLimitDeg);
        Runtime.SearchDir *= -1.0f;
    }

    Interceptor->SetCameraAngles(Params.SearchCamPitchDeg, Runtime.SearchCamYawDeg);

    const float CurrentYawDeg = Interceptor->CurrentState.GetRotator().Yaw;
    const float YawCmdDeg = NormalizeYawDeg(CurrentYawDeg + Runtime.SearchDir * Params.SearchBodyYawRateDeg * Dt);
    const float VzCmd = Params.SearchVzAmp * FMath::Sin(Runtime.SearchTime * 0.8f);
    const FVector CmdVel(0.0f, 0.0f, VzCmd);

    Interceptor->SetHeadingControl(EDroneYawMode::Angle, EDroneDrivetrainMode::ForwardOnly, YawCmdDeg);
    Interceptor->SetTargetVelocity(CmdVel);

    Runtime.State = EVisualState::Search;
    Runtime.LastEx = 0.0f;
    Runtime.LastEy = 0.0f;
    Runtime.LastAreaNorm = 0.0f;
    Runtime.LastConf = 0.0f;
    Runtime.LastYawCmdDeg = YawCmdDeg;
    Runtime.LastYawRateDeg = Runtime.SearchDir * Params.SearchBodyYawRateDeg;
    Runtime.LastCmdVel = CmdVel;
}

/**
 * @brief 启动视觉拦截会话
 * @param CmdObj 启动命令 JSON
 * @param World 当前世界对象
 * @return 启动结果 JSON
 */
FString UVisualInterceptController::HandleStart(const TSharedPtr<FJsonObject>& CmdObj, UWorld* World)
{
    EnsureInitialized();

    if (!World)
    {
        return MakeError(TEXT("No World"));
    }

    ApplyParamsFromJson(CmdObj);

    FString RequestedMethod = Runtime.Method;
    if (CmdObj.IsValid() && CmdObj->HasField(TEXT("method")))
    {
        RequestedMethod = CmdObj->GetStringField(TEXT("method"));
    }

    FString CanonicalMethod;
    if (!NormalizeMethod(RequestedMethod, CanonicalMethod))
    {
        return MakeError(FString::Printf(TEXT("unsupported visual intercept method: %s"), *RequestedMethod));
    }
    Runtime.Method = CanonicalMethod;

    if (CmdObj.IsValid() && !CmdObj->HasField(TEXT("use_kalman")))
    {
        Params.bUseKalman = Runtime.Method.Contains(TEXT("kalman"));
    }

    ADronePawn* Interceptor = ResolveInterceptor(CmdObj);
    if (!Interceptor)
    {
        return MakeError(TEXT("interceptor drone not found"));
    }

    ResolveTarget(CmdObj);

    if (FeaturePredictor)
    {
        FeaturePredictor->Reset();
    }
    ResetPidControllers();
    BeginSession(Interceptor);

    Interceptor->SetCameraAngles(Params.SearchCamPitchDeg, 0.0f);
    return BuildStartJson();
}

/**
 * @brief 处理一帧视觉拦截更新
 * @param CmdObj 更新命令 JSON
 * @param World 当前世界对象（当前实现未使用）
 * @return 本帧控制结果 JSON
 *
 * 运行逻辑分三支：
 * 1. 有检测：直接使用检测框，必要时先经过 Kalman 平滑；
 * 2. 无检测但预测器仍可用：用短时预测结果维持跟踪；
 * 3. 长时间丢失：回退到搜索模式。
 *
 * 最终捕获判定以真实三维距离为准：
 * $d = \|p_{target} - p_{interceptor}\| \le InterceptDistance$。
 */
FString UVisualInterceptController::HandleUpdate(const TSharedPtr<FJsonObject>& CmdObj, UWorld* World)
{
    (void)World;
    EnsureInitialized();

    if (!Runtime.bEnabled)
    {
        return MakeError(TEXT("visual intercept is not started"));
    }
    if (!CmdObj.IsValid())
    {
        return MakeError(TEXT("missing visual intercept update payload"));
    }

    ADronePawn* Interceptor = ResolveInterceptor(CmdObj);
    if (!Interceptor)
    {
        return MakeError(TEXT("interceptor drone not found"));
    }

    ADronePawn* Target = ResolveTarget(CmdObj);

    float Dt = kDefaultDt;
    TryGetNumberAs(CmdObj, TEXT("dt"), Dt);
    Dt = SafeDt(Dt);
    SyncPidTimeStep(Dt);

    float FrameW = Runtime.LastFrameW;
    float FrameH = Runtime.LastFrameH;
    TryGetNumberAs(CmdObj, TEXT("image_w"), FrameW);
    TryGetNumberAs(CmdObj, TEXT("image_h"), FrameH);
    Runtime.LastFrameW = FMath::Max(1.0f, FrameW);
    Runtime.LastFrameH = FMath::Max(1.0f, FrameH);

    Runtime.FrameCount++;

    const bool bHasDetection = CmdObj->HasField(TEXT("has_detection"))
        ? CmdObj->GetBoolField(TEXT("has_detection"))
        : (CmdObj->HasField(TEXT("cx")) && CmdObj->HasField(TEXT("cy")));

    bool bValidControl = true;
    bool bCapturedThisFrame = false;

    if (bHasDetection)
    {
        float Cx = 0.0f;
        float Cy = 0.0f;
        float Area = 0.0f;
        float AreaRatio = -1.0f;
        float Confidence = 1.0f;

        TryGetNumberAs(CmdObj, TEXT("cx"), Cx);
        TryGetNumberAs(CmdObj, TEXT("cy"), Cy);
        TryGetNumberAs(CmdObj, TEXT("area"), Area);
        TryGetNumberAs(CmdObj, TEXT("area_ratio"), AreaRatio);
        TryGetNumberAs(CmdObj, TEXT("conf"), Confidence);

        if (Area <= 0.0f && AreaRatio > 0.0f)
        {
            Area = AreaRatio * Runtime.LastFrameW * Runtime.LastFrameH;
        }
        Area = FMath::Max(1.0f, Area);

        Runtime.DetectionCount++;
        Runtime.LostCount = 0;

        FVector Feature(Cx, Cy, Area);
        if (Params.bUseKalman && FeaturePredictor)
        {
            FeaturePredictor->Update(Feature, Dt);
            Feature = FeaturePredictor->GetEstimatedPosition();
            Feature.Z = FMath::Max(1.0f, Feature.Z);
        }

        bValidControl = ComputeTrackControl(
            Interceptor,
            Feature,
            Runtime.LastFrameW,
            Runtime.LastFrameH,
            Confidence,
            Dt,
            true,
            bCapturedThisFrame);
    }
    else
    {
        Runtime.LostCount++;

        bool bUsedPrediction = false;
        if (Params.bUseKalman && FeaturePredictor && FeaturePredictor->IsInitialized() && Runtime.LostCount < Params.LostToSearchFrames)
        {
            FVector Pred = FeaturePredictor->PredictPosition(Dt);
            Pred.Z = FMath::Max(1.0f, Pred.Z);
            bUsedPrediction = true;
            bValidControl = ComputeTrackControl(
                Interceptor,
                Pred,
                Runtime.LastFrameW,
                Runtime.LastFrameH,
                0.0f,
                Dt,
                false,
                bCapturedThisFrame);
        }

        if (!bUsedPrediction)
        {
            ComputeSearchControl(Interceptor, Dt);
        }
    }

    Runtime.LastTargetDistance = -1.0f;
    if (Target)
    {
        Runtime.LastTargetDistance = FVector::Dist(Interceptor->GetCurrentPosition(), Target->GetCurrentPosition());
        if (Runtime.LastTargetDistance <= Params.InterceptDistance)
        {
            Runtime.CaptureCount++;
            Runtime.State = EVisualState::Captured;
            Runtime.bCaptured = true;
            bCapturedThisFrame = true;
        }
        else
        {
            Runtime.CaptureCount = 0;
        }
    }
    else
    {
        Runtime.CaptureCount = 0;
    }

    if (bCapturedThisFrame && Params.bStopOnCapture)
    {
        Interceptor->Hover();
        Runtime.bEnabled = false;
    }

    return BuildUpdateJson(bValidControl);
}

/**
 * @brief 停止视觉拦截，并令拦截机悬停
 * @param CmdObj 停止命令 JSON
 * @param World 当前世界对象（当前实现未使用）
 * @return 停止结果 JSON
 */
FString UVisualInterceptController::HandleStop(const TSharedPtr<FJsonObject>& CmdObj, UWorld* World)
{
    (void)World;
    EnsureInitialized();

    ADronePawn* Interceptor = ResolveInterceptor(CmdObj);
    if (Interceptor)
    {
        Interceptor->Hover();
    }

    Runtime.bEnabled = false;
    Runtime.State = Runtime.bCaptured ? EVisualState::Captured : EVisualState::Idle;
    return MakeOk(TEXT("visual intercept stopped"));
}

/** @brief 查询当前视觉拦截运行状态 */
FString UVisualInterceptController::HandleState() const
{
    return FString::Printf(
        TEXT("{\"status\":\"ok\",\"mode\":\"visual_intercept\",\"enabled\":%s,\"state\":\"%s\",\"captured\":%s,\"interceptor_id\":\"%s\",\"target_id\":\"%s\",\"method\":\"%s\",\"frame\":%d,\"detections\":%d,\"lost_count\":%d,\"capture_count\":%d,\"target_distance\":%.4f,\"cmd_velocity\":[%.4f,%.4f,%.4f],\"control\":{\"ex\":%.5f,\"ey\":%.5f,\"area_norm\":%.6f,\"conf\":%.4f,\"yaw_cmd_deg\":%.4f,\"yaw_rate_deg\":%.4f}}"),
        *BoolLiteral(Runtime.bEnabled),
        *StateToString(Runtime.State),
        *BoolLiteral(Runtime.bCaptured),
        *Runtime.InterceptorId,
        *Runtime.TargetId,
        *Runtime.Method,
        Runtime.FrameCount,
        Runtime.DetectionCount,
        Runtime.LostCount,
        Runtime.CaptureCount,
        Runtime.LastTargetDistance,
        Runtime.LastCmdVel.X,
        Runtime.LastCmdVel.Y,
        Runtime.LastCmdVel.Z,
        Runtime.LastEx,
        Runtime.LastEy,
        Runtime.LastAreaNorm,
        Runtime.LastConf,
        Runtime.LastYawCmdDeg,
        Runtime.LastYawRateDeg);
}
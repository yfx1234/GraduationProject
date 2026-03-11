#include "GuidanceCommandHandler.h"

#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"
#include "KalmanPredictor.h"
#include "GuidanceMethods.h"
#include "VisualInterceptController.h"
#include "GraduationProject/Core/Manager/AgentManager.h"
#include "GraduationProject/Turret/TurretPawn.h"

namespace
{
    FString BoolLiteral(bool bValue)
    {
        return bValue ? TEXT("true") : TEXT("false");
    }

    FString StringArrayToJson(const TArray<FString>& Values)
    {
        FString Out = TEXT("[");
        for (int32 Index = 0; Index < Values.Num(); ++Index)
        {
            Out += FString::Printf(TEXT("\"%s\""), *Values[Index]);
            if (Index + 1 < Values.Num())
            {
                Out += TEXT(",");
            }
        }
        Out += TEXT("]");
        return Out;
    }

    FString NormalizeInterceptMethod(const FString& Method)
    {
        FString Normalized = Method;
        Normalized.ToLowerInline();
        if (Normalized == TEXT("pn") || Normalized == TEXT("proportional") || Normalized == TEXT("proportional_navigation"))
        {
            return TEXT("proportional_nav");
        }
        if (Normalized == TEXT("smc") || Normalized == TEXT("sliding_mode") || Normalized == TEXT("slidingmode"))
        {
            return TEXT("smc");
        }
        return (Normalized == TEXT("pure_pursuit")) ? Normalized : TEXT("pure_pursuit");
    }
}

/**
 * @brief 构造错误响应 JSON
 * @param Msg 错误信息
 * @return 错误 JSON 字符串
 */
FString UGuidanceCommandHandler::MakeError(const FString& Msg)
{
    return FString::Printf(TEXT("{\"status\":\"error\",\"message\":\"%s\"}"), *Msg);
}

/**
 * @brief 构造成功响应 JSON
 * @param Msg 成功说明
 * @return 成功 JSON 字符串
 */
FString UGuidanceCommandHandler::MakeOk(const FString& Msg)
{
    return FString::Printf(TEXT("{\"status\":\"ok\",\"message\":\"%s\"}"), *Msg);
}

/** @brief 构造制导命令处理器并初始化默认状态 */
UGuidanceCommandHandler::UGuidanceCommandHandler()
    : CurrentMethod(nullptr)
    , CurrentMethodName(TEXT("predictive"))
{
}

/** @brief 析构处理器，释放非 UObject 算法对象 */
UGuidanceCommandHandler::~UGuidanceCommandHandler()
{
    if (CurrentMethod)
    {
        delete CurrentMethod;
        CurrentMethod = nullptr;
    }
}

/**
 * @brief 延迟初始化预测器、制导算法和视觉拦截控制器
 * 保证首次收到命令时才分配相关运行资源。
 */
void UGuidanceCommandHandler::EnsureInitialized()
{
    if (!Predictor)
    {
        Predictor = NewObject<UKalmanPredictor>(this);
        Predictor->Initialize(100.0f, 0.01f);
    }

    if (!VisualInterceptController)
    {
        VisualInterceptController = NewObject<UVisualInterceptController>(this);
        VisualInterceptController->EnsureInitialized();
    }

    if (!CurrentMethod)
    {
        CurrentMethod = new FPredictiveGuidance(Predictor, 3);
        CurrentMethodName = TEXT("predictive");
    }
}

/**
 * @brief 按任务角色查找无人机
 * @param Manager Agent 管理器
 * @param Role 目标角色
 * @param ExcludeId 需要排除的 ID
 * @return 匹配到的无人机实例
 */
ADronePawn* UGuidanceCommandHandler::FindDroneByRole(UAgentManager* Manager, EDroneMissionRole Role, const FString& ExcludeId) const
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

/**
 * @brief 执行制导相关命令
 * @param JsonObject 请求 JSON
 * @param World 当前场景 World
 * @return 命令执行结果 JSON
 * 该入口统一处理算法切换、炮台制导、无人机拦截和视觉闭环控制。
 */
FString UGuidanceCommandHandler::HandleCallGuidance(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World)
{
    const TSharedPtr<FJsonObject>* CmdObj = nullptr;
    if (!JsonObject->TryGetObjectField(TEXT("call_guidance"), CmdObj))
    {
        return MakeError(TEXT("Missing call_guidance field"));
    }

    FString Function;
    (*CmdObj)->TryGetStringField(TEXT("function"), Function);

    EnsureInitialized();

    if (Function == TEXT("visual_intercept_start"))
    {
        return VisualInterceptController ? VisualInterceptController->HandleStart(*CmdObj, World) : MakeError(TEXT("visual controller unavailable"));
    }

    if (Function == TEXT("visual_intercept_update"))
    {
        return VisualInterceptController ? VisualInterceptController->HandleUpdate(*CmdObj, World) : MakeError(TEXT("visual controller unavailable"));
    }

    if (Function == TEXT("visual_intercept_stop"))
    {
        return VisualInterceptController ? VisualInterceptController->HandleStop(*CmdObj, World) : MakeError(TEXT("visual controller unavailable"));
    }

    if (Function == TEXT("visual_intercept_state"))
    {
        return VisualInterceptController ? VisualInterceptController->HandleState() : MakeError(TEXT("visual controller unavailable"));
    }

    if (Function == TEXT("set_method"))
    {
        FString Method;
        if (!(*CmdObj)->TryGetStringField(TEXT("method"), Method))
        {
            return MakeError(TEXT("Missing method name"));
        }

        if (CurrentMethod)
        {
            delete CurrentMethod;
            CurrentMethod = nullptr;
        }

        if (Method == TEXT("direct"))
        {
            CurrentMethod = new FDirectAiming();
        }
        else if (Method == TEXT("proportional"))
        {
            float N = 4.0f;
            if ((*CmdObj)->HasField(TEXT("nav_constant")))
            {
                N = static_cast<float>((*CmdObj)->GetNumberField(TEXT("nav_constant")));
            }
            CurrentMethod = new FProportionalNavigation(N);
        }
        else if (Method == TEXT("predictive"))
        {
            int32 Iters = 3;
            if ((*CmdObj)->HasField(TEXT("iterations")))
            {
                Iters = static_cast<int32>((*CmdObj)->GetNumberField(TEXT("iterations")));
            }
            CurrentMethod = new FPredictiveGuidance(Predictor, Iters);
        }
        else
        {
            return MakeError(FString::Printf(TEXT("Unknown method: %s"), *Method));
        }

        CurrentMethodName = Method;
        return MakeOk(FString::Printf(TEXT("Method set to %s"), *Method));
    }

    if (Function == TEXT("set_intercept_method"))
    {
        FString Method = CurrentInterceptMethod;
        if ((*CmdObj)->HasField(TEXT("method")))
        {
            Method = NormalizeInterceptMethod((*CmdObj)->GetStringField(TEXT("method")));
        }

        CurrentInterceptMethod = Method;

        if ((*CmdObj)->HasField(TEXT("speed")))
        {
            InterceptorSpeed = FMath::Max(0.1f, static_cast<float>((*CmdObj)->GetNumberField(TEXT("speed"))));
        }

        if ((*CmdObj)->HasField(TEXT("nav_gain")))
        {
            InterceptNavGain = FMath::Max(0.1f, static_cast<float>((*CmdObj)->GetNumberField(TEXT("nav_gain"))));
        }

        if ((*CmdObj)->HasField(TEXT("lead_time")))
        {
            InterceptLeadTime = FMath::Max(0.0f, static_cast<float>((*CmdObj)->GetNumberField(TEXT("lead_time"))));
        }

        if ((*CmdObj)->HasField(TEXT("capture_radius")))
        {
            CaptureRadius = FMath::Max(0.1f, static_cast<float>((*CmdObj)->GetNumberField(TEXT("capture_radius"))));
        }

        return FString::Printf(
            TEXT("{\"status\":\"ok\",\"intercept_method\":\"%s\",\"speed\":%.2f,\"nav_gain\":%.2f,\"lead_time\":%.2f,\"capture_radius\":%.2f}"),
            *CurrentInterceptMethod, InterceptorSpeed, InterceptNavGain, InterceptLeadTime, CaptureRadius);
    }

    if (Function == TEXT("list_intercept_agents"))
    {
        UAgentManager* Manager = UAgentManager::GetInstance();
        TArray<FString> Targets;
        TArray<FString> Interceptors;

        const TArray<FString> AgentIds = Manager->GetAllAgentIds();
        for (const FString& AgentId : AgentIds)
        {
            ADronePawn* Drone = Cast<ADronePawn>(Manager->GetAgent(AgentId));
            if (!Drone)
            {
                continue;
            }

            if (Drone->MissionRole == EDroneMissionRole::Target)
            {
                Targets.Add(Drone->DroneId);
            }
            else if (Drone->MissionRole == EDroneMissionRole::Interceptor)
            {
                Interceptors.Add(Drone->DroneId);
            }
        }

        return FString::Printf(
            TEXT("{\"status\":\"ok\",\"targets\":%s,\"interceptors\":%s,\"target_count\":%d,\"interceptor_count\":%d}"),
            *StringArrayToJson(Targets),
            *StringArrayToJson(Interceptors),
            Targets.Num(),
            Interceptors.Num());
    }

    if (Function == TEXT("auto_intercept"))
    {
        UAgentManager* Manager = UAgentManager::GetInstance();

        FString TargetId;
        FString InterceptorId;
        (*CmdObj)->TryGetStringField(TEXT("target_id"), TargetId);
        (*CmdObj)->TryGetStringField(TEXT("interceptor_id"), InterceptorId);

        ADronePawn* TargetDrone = nullptr;
        ADronePawn* InterceptorDrone = nullptr;

        if (!TargetId.IsEmpty())
        {
            TargetDrone = Cast<ADronePawn>(Manager->GetAgent(TargetId));
        }
        else
        {
            TargetDrone = FindDroneByRole(Manager, EDroneMissionRole::Target);
            if (TargetDrone)
            {
                TargetId = TargetDrone->DroneId;
            }
        }

        if (!InterceptorId.IsEmpty())
        {
            InterceptorDrone = Cast<ADronePawn>(Manager->GetAgent(InterceptorId));
        }
        else
        {
            const FString ExcludeTarget = TargetDrone ? TargetDrone->DroneId : TEXT("");
            InterceptorDrone = FindDroneByRole(Manager, EDroneMissionRole::Interceptor, ExcludeTarget);
            if (InterceptorDrone)
            {
                InterceptorId = InterceptorDrone->DroneId;
            }
        }

        if (!TargetDrone)
        {
            return MakeError(TEXT("Target drone not found. Provide target_id or set MissionRole=Target."));
        }

        if (!InterceptorDrone)
        {
            return MakeError(TEXT("Interceptor drone not found. Provide interceptor_id or set MissionRole=Interceptor."));
        }

        if (TargetDrone == InterceptorDrone)
        {
            return MakeError(TEXT("target_id and interceptor_id must be different"));
        }

        FString Method = CurrentInterceptMethod;
        if ((*CmdObj)->HasField(TEXT("method")))
        {
            Method = NormalizeInterceptMethod((*CmdObj)->GetStringField(TEXT("method")));
        }

        float Speed = InterceptorSpeed;
        if ((*CmdObj)->HasField(TEXT("speed")))
        {
            Speed = FMath::Max(0.1f, static_cast<float>((*CmdObj)->GetNumberField(TEXT("speed"))));
        }

        float NavGain = InterceptNavGain;
        if ((*CmdObj)->HasField(TEXT("nav_gain")))
        {
            NavGain = FMath::Max(0.1f, static_cast<float>((*CmdObj)->GetNumberField(TEXT("nav_gain"))));
        }

        float LeadTime = InterceptLeadTime;
        if ((*CmdObj)->HasField(TEXT("lead_time")))
        {
            LeadTime = FMath::Max(0.0f, static_cast<float>((*CmdObj)->GetNumberField(TEXT("lead_time"))));
        }

        float LocalCaptureRadius = CaptureRadius;
        if ((*CmdObj)->HasField(TEXT("capture_radius")))
        {
            LocalCaptureRadius = FMath::Max(0.1f, static_cast<float>((*CmdObj)->GetNumberField(TEXT("capture_radius"))));
        }

        bool bStopOnCapture = true;
        if ((*CmdObj)->HasField(TEXT("stop_on_capture")))
        {
            bStopOnCapture = (*CmdObj)->GetBoolField(TEXT("stop_on_capture"));
        }

        const FVector InterceptorPos = InterceptorDrone->GetCurrentPosition();
        const FVector InterceptorVel = InterceptorDrone->GetCurrentVelocity();
        const FVector TargetPos = TargetDrone->GetCurrentPosition();
        const FVector TargetVel = TargetDrone->GetCurrentVelocity();

        const FVector RelativePos = TargetPos - InterceptorPos;
        const FVector RelativeVel = TargetVel - InterceptorVel;
        const float Distance = RelativePos.Size();
        const FVector LOS = RelativePos.GetSafeNormal();
        const float ClosingSpeed = -FVector::DotProduct(RelativeVel, LOS);

        FVector CommandVelocity = FVector::ZeroVector;
        bool bCaptured = Distance <= LocalCaptureRadius;
        bool bValidCmd = !RelativePos.IsNearlyZero();

        if (bCaptured)
        {
            if (bStopOnCapture)
            {
                InterceptorDrone->Hover();
            }
        }
        else
        {
            if (Method == TEXT("proportional_nav"))
            {
                const FVector LateralRelVel = RelativeVel - FVector::DotProduct(RelativeVel, LOS) * LOS;
                CommandVelocity = TargetVel + NavGain * LateralRelVel;

                if (CommandVelocity.IsNearlyZero())
                {
                    CommandVelocity = LOS * Speed;
                }
            }
            else
            {
                const FVector PredictedTarget = TargetPos + TargetVel * LeadTime;
                CommandVelocity = (PredictedTarget - InterceptorPos).GetSafeNormal() * Speed;
            }

            if (!CommandVelocity.IsNearlyZero())
            {
                CommandVelocity = CommandVelocity.GetClampedToMaxSize(Speed);
                InterceptorDrone->SetHeadingControl(EDroneYawMode::Auto, EDroneDrivetrainMode::ForwardOnly);
                InterceptorDrone->SetTargetVelocity(CommandVelocity);
            }
            else
            {
                bValidCmd = false;
            }
        }

        LastInterceptorId = InterceptorDrone->DroneId;
        LastTargetId = TargetDrone->DroneId;
        LastDistanceToTarget = Distance;
        LastClosingSpeed = ClosingSpeed;
        LastInterceptorCmdVel = CommandVelocity;
        bLastInterceptValid = bValidCmd;
        bLastCaptured = bCaptured;

        return FString::Printf(
            TEXT("{\"status\":\"ok\",\"mode\":\"auto_intercept\",\"method\":\"%s\",\"target_id\":\"%s\",\"interceptor_id\":\"%s\",\"distance\":%.3f,\"closing_speed\":%.3f,\"captured\":%s,\"valid\":%s,\"cmd_velocity\":[%.3f,%.3f,%.3f]}"),
            *Method,
            *LastTargetId,
            *LastInterceptorId,
            LastDistanceToTarget,
            LastClosingSpeed,
            *BoolLiteral(bCaptured),
            *BoolLiteral(bValidCmd),
            LastInterceptorCmdVel.X,
            LastInterceptorCmdVel.Y,
            LastInterceptorCmdVel.Z);
    }

    if (Function == TEXT("update_target"))
    {
        float X = static_cast<float>((*CmdObj)->GetNumberField(TEXT("x")));
        float Y = static_cast<float>((*CmdObj)->GetNumberField(TEXT("y")));
        float Z = static_cast<float>((*CmdObj)->GetNumberField(TEXT("z")));
        float Dt = 0.1f;
        if ((*CmdObj)->HasField(TEXT("dt")))
        {
            Dt = static_cast<float>((*CmdObj)->GetNumberField(TEXT("dt")));
        }

        Predictor->Update(FVector(X, Y, Z), Dt);
        const FVector EstPos = Predictor->GetEstimatedPosition();
        const FVector EstVel = Predictor->GetEstimatedVelocity();
        const FVector EstAcc = Predictor->GetEstimatedAcceleration();
        const float AdaptiveQ = Predictor->GetAdaptiveProcessNoise();
        return FString::Printf(
            TEXT("{\"status\":\"ok\",\"est_pos\":[%.2f,%.2f,%.2f],\"est_vel\":[%.2f,%.2f,%.2f],\"est_acc\":[%.2f,%.2f,%.2f],\"adaptive_q\":%.4f}"),
            EstPos.X, EstPos.Y, EstPos.Z,
            EstVel.X, EstVel.Y, EstVel.Z,
            EstAcc.X, EstAcc.Y, EstAcc.Z,
            AdaptiveQ);
    }

    if (Function == TEXT("compute_aim"))
    {
        FString TurretId;
        if (!(*CmdObj)->TryGetStringField(TEXT("turret_id"), TurretId))
        {
            TurretId = TEXT("turret_0");
        }

        float MuzzleSpeed = 400.0f;
        if ((*CmdObj)->HasField(TEXT("muzzle_speed")))
        {
            MuzzleSpeed = static_cast<float>((*CmdObj)->GetNumberField(TEXT("muzzle_speed")));
        }

        UAgentManager* Manager = UAgentManager::GetInstance();
        ATurretPawn* Turret = Cast<ATurretPawn>(Manager->GetAgent(TurretId));
        if (!Turret)
        {
            return MakeError(FString::Printf(TEXT("Turret '%s' not found"), *TurretId));
        }

        FGuidanceInput Input;
        Input.TurretPos = Turret->GetActorLocation();
        Input.MuzzlePos = Turret->GunMesh
            ? (Turret->GunMesh->GetComponentLocation() + Turret->GunMesh->GetComponentRotation().RotateVector(Turret->MuzzleOffset))
            : Input.TurretPos;
        Input.TargetPos = Predictor->GetEstimatedPosition();
        Input.TargetVel = Predictor->GetEstimatedVelocity();
        Input.PredictedPos = Predictor->PredictPosition(0.5f);
        Input.MuzzleSpeed = MuzzleSpeed;
        Input.DeltaTime = 0.1f;

        const FGuidanceOutput Output = CurrentMethod->ComputeAim(Input);
        LastPitch = Output.Pitch;
        LastYaw = Output.Yaw;
        LastAimPoint = Output.AimPoint;
        LastFlightTime = Output.EstFlightTime;

        return FString::Printf(
            TEXT("{\"status\":\"ok\",\"pitch\":%.2f,\"yaw\":%.2f,\"aim_point\":[%.1f,%.1f,%.1f],\"flight_time\":%.4f,\"method\":\"%s\"}"),
            Output.Pitch, Output.Yaw,
            Output.AimPoint.X, Output.AimPoint.Y, Output.AimPoint.Z,
            Output.EstFlightTime,
            *CurrentMethodName);
    }

    if (Function == TEXT("auto_engage"))
    {
        FString TurretId;
        if (!(*CmdObj)->TryGetStringField(TEXT("turret_id"), TurretId))
        {
            TurretId = TEXT("turret_0");
        }

        FString TargetId;
        if (!(*CmdObj)->TryGetStringField(TEXT("target_id"), TargetId))
        {
            TargetId = TEXT("drone_0");
        }

        float MuzzleSpeed = 400.0f;
        if ((*CmdObj)->HasField(TEXT("muzzle_speed")))
        {
            MuzzleSpeed = static_cast<float>((*CmdObj)->GetNumberField(TEXT("muzzle_speed")));
        }

        UAgentManager* Manager = UAgentManager::GetInstance();
        AActor* Target = Manager->GetAgent(TargetId);
        ATurretPawn* Turret = Cast<ATurretPawn>(Manager->GetAgent(TurretId));
        if (!Target)
        {
            return MakeError(FString::Printf(TEXT("Target '%s' not found"), *TargetId));
        }
        if (!Turret)
        {
            return MakeError(FString::Printf(TEXT("Turret '%s' not found"), *TurretId));
        }

        float Dt = 0.05f;
        if ((*CmdObj)->HasField(TEXT("dt")))
        {
            Dt = static_cast<float>((*CmdObj)->GetNumberField(TEXT("dt")));
        }

        float Latency = DefaultVisionLatency;
        if ((*CmdObj)->HasField(TEXT("latency")))
        {
            Latency = static_cast<float>((*CmdObj)->GetNumberField(TEXT("latency")));
        }
        Latency = FMath::Clamp(Latency, 0.0f, 1.0f);
        LastLatencyCompensation = Latency;

        const FVector TargetPos = Target->GetActorLocation();
        Predictor->Update(TargetPos, Dt);
        const FVector CompensatedTargetPos = Predictor->PredictPosition(Latency);

        FGuidanceInput Input;
        Input.TurretPos = Turret->GetActorLocation();
        Input.MuzzlePos = Turret->GunMesh
            ? (Turret->GunMesh->GetComponentLocation() + Turret->GunMesh->GetComponentRotation().RotateVector(Turret->MuzzleOffset))
            : Input.TurretPos;
        Input.TargetPos = CompensatedTargetPos;
        Input.TargetVel = Predictor->GetEstimatedVelocity();
        Input.PredictedPos = Predictor->PredictPosition(Latency + 0.5f);
        Input.MuzzleSpeed = MuzzleSpeed;
        Input.DeltaTime = Dt;

        const FGuidanceOutput Output = CurrentMethod->ComputeAim(Input);
        if (Output.bValid)
        {
            Turret->SetTargetAngles(Output.Pitch, Output.Yaw);
            LastPitch = Output.Pitch;
            LastYaw = Output.Yaw;
            LastAimPoint = Output.AimPoint;
            LastFlightTime = Output.EstFlightTime;
        }

        bool bFire = false;
        if ((*CmdObj)->HasField(TEXT("fire")) && (*CmdObj)->GetBoolField(TEXT("fire")))
        {
            Turret->FireX(MuzzleSpeed);
            bFire = true;
        }

        return FString::Printf(
            TEXT("{\"status\":\"ok\",\"pitch\":%.2f,\"yaw\":%.2f,\"fired\":%s,\"flight_time\":%.4f,\"latency\":%.4f,\"method\":\"%s\"}"),
            Output.Pitch,
            Output.Yaw,
            *BoolLiteral(bFire),
            Output.EstFlightTime,
            LastLatencyCompensation,
            *CurrentMethodName);
    }
    if (Function == TEXT("reset"))
    {
        Predictor->Reset();
        Predictor->Initialize(100.0f, 0.01f);

        if (CurrentMethod)
        {
            CurrentMethod->Reset();
        }

        LastPitch = 0.0f;
        LastYaw = 0.0f;
        LastAimPoint = FVector::ZeroVector;
        LastFlightTime = 0.0f;
        LastLatencyCompensation = DefaultVisionLatency;

        LastInterceptorId.Empty();
        LastTargetId.Empty();
        LastInterceptorCmdVel = FVector::ZeroVector;
        LastDistanceToTarget = 0.0f;
        LastClosingSpeed = 0.0f;
        bLastInterceptValid = false;
        bLastCaptured = false;

        if (VisualInterceptController)
        {
            VisualInterceptController->Reset();
        }

        return MakeOk(TEXT("guidance reset"));
    }

    if (Function == TEXT("set_kalman_params"))
    {
        float Q = 1.0f;
        float R = 0.5f;

        if ((*CmdObj)->HasField(TEXT("process_noise")))
        {
            Q = static_cast<float>((*CmdObj)->GetNumberField(TEXT("process_noise")));
        }
        if ((*CmdObj)->HasField(TEXT("measurement_noise")))
        {
            R = static_cast<float>((*CmdObj)->GetNumberField(TEXT("measurement_noise")));
        }

        Predictor->Initialize(Q, R);
        return MakeOk(FString::Printf(TEXT("kalman Q=%.2f R=%.2f"), Q, R));
    }

    return MakeError(FString::Printf(TEXT("Unknown function: %s"), *Function));
}

/**
 * @brief 获取当前制导状态快照
 * @param JsonObject 请求 JSON
 * @param World 当前场景 World
 * @return 制导状态 JSON
 */
FString UGuidanceCommandHandler::HandleGetGuidanceState(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World)
{
    EnsureInitialized();

    const FVector EstPos = Predictor->GetEstimatedPosition();
    const FVector EstVel = Predictor->GetEstimatedVelocity();
    const FVector EstAcc = Predictor->GetEstimatedAcceleration();
    const float Uncertainty = Predictor->GetPositionUncertainty();
    const float AdaptiveQ = Predictor->GetAdaptiveProcessNoise();

    return FString::Printf(
        TEXT("{\"status\":\"ok\",\"method\":\"%s\",\"initialized\":%s,")
        TEXT("\"est_pos\":[%.2f,%.2f,%.2f],\"est_vel\":[%.2f,%.2f,%.2f],\"est_acc\":[%.2f,%.2f,%.2f],")
        TEXT("\"uncertainty\":%.2f,\"adaptive_q\":%.4f,\"aim_pitch\":%.2f,\"aim_yaw\":%.2f,")
        TEXT("\"aim_point\":[%.1f,%.1f,%.1f],\"flight_time\":%.4f,\"latency\":%.4f,")
        TEXT("\"intercept\":{\"method\":\"%s\",\"target_id\":\"%s\",\"interceptor_id\":\"%s\",\"distance\":%.3f,\"closing_speed\":%.3f,\"captured\":%s,\"valid\":%s,\"cmd_velocity\":[%.3f,%.3f,%.3f]}}"),
        *CurrentMethodName,
        *BoolLiteral(Predictor->IsInitialized()),
        EstPos.X, EstPos.Y, EstPos.Z,
        EstVel.X, EstVel.Y, EstVel.Z,
        EstAcc.X, EstAcc.Y, EstAcc.Z,
        Uncertainty,
        AdaptiveQ,
        LastPitch, LastYaw,
        LastAimPoint.X, LastAimPoint.Y, LastAimPoint.Z,
        LastFlightTime,
        LastLatencyCompensation,
        *CurrentInterceptMethod,
        *LastTargetId,
        *LastInterceptorId,
        LastDistanceToTarget,
        LastClosingSpeed,
        *BoolLiteral(bLastCaptured),
        *BoolLiteral(bLastInterceptValid),
        LastInterceptorCmdVel.X,
        LastInterceptorCmdVel.Y,
        LastInterceptorCmdVel.Z);
}



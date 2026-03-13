#include "GuidanceActor.h"

#include "Dom/JsonObject.h"
#include "GuidanceMethods.h"
#include "KalmanPredictor.h"
#include "VisualInterceptController.h"
#include "GraduationProject/Core/Manager/AgentManager.h"
#include "GraduationProject/Turret/TurretPawn.h"

namespace
{
    /** @brief 布尔值转 JSON 字面量文本 */
    FString BoolLiteral(bool bValue)
    {
        return bValue ? TEXT("true") : TEXT("false");
    }

    /** @brief 将字符串数组序列化为 JSON 数组文本 */
    FString StringArrayToJson(const TArray<FString>& Values)
    {
        FString Out = TEXT("[");
        for (int32 Index = 0; Index < Values.Num(); ++Index)
        {
            Out += FString::Printf(TEXT("\"%s\""), *Values[Index].ReplaceCharWithEscapedChar());
            if (Index + 1 < Values.Num())
            {
                Out += TEXT(",");
            }
        }
        Out += TEXT("]");
        return Out;
    }

    /**
     * @brief 规范化无人机自动拦截方法名称
     * @param Method 原始方法名
     * @return 归一化后的方法名
     *
     * 目前统一收敛为：
     * - `pure_pursuit`
     * - `proportional_nav`
     * - `smc`
     */
    FString NormalizeInterceptMethodName(const FString& Method)
    {
        FString Normalized = Method;
        Normalized.TrimStartAndEndInline();
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

    /**
     * @brief 估计炮塔炮口世界坐标
     * @param Turret 炮塔实例
     * @return 炮口位置；若炮管网格不存在，则退化为 Actor 位置
     */
    FVector GetTurretMuzzlePosition(const ATurretPawn* Turret)
    {
        if (!Turret)
        {
            return FVector::ZeroVector;
        }
        if (!Turret->GunMesh)
        {
            return Turret->GetActorLocation();
        }

        return Turret->GunMesh->GetComponentLocation() +
            Turret->GunMesh->GetComponentRotation().RotateVector(Turret->MuzzleOffset);
    }

    /** @brief 当字符串非空时写入 JSON 字段 */
    void SetStringIfNotEmpty(const TSharedPtr<FJsonObject>& Object, const TCHAR* FieldName, const FString& Value)
    {
        if (Object.IsValid() && !Value.IsEmpty())
        {
            Object->SetStringField(FieldName, Value);
        }
    }

    /** @brief 当数值不是哨兵值时写入 JSON 字段 */
    void SetNumberIfValid(const TSharedPtr<FJsonObject>& Object, const TCHAR* FieldName, float Value, float InvalidSentinel = -1.0f)
    {
        if (Object.IsValid() && !FMath::IsNearlyEqual(Value, InvalidSentinel))
        {
            Object->SetNumberField(FieldName, Value);
        }
    }

    /** @brief 当整型参数有效时写入 JSON 字段 */
    void SetIntIfValid(const TSharedPtr<FJsonObject>& Object, const TCHAR* FieldName, int32 Value)
    {
        if (Object.IsValid() && Value >= 0)
        {
            Object->SetNumberField(FieldName, Value);
        }
    }
}

/** @brief 构造 GuidanceActor，并将其设为隐藏的后台协调 Actor */
AGuidanceActor::AGuidanceActor()
{
    PrimaryActorTick.bCanEverTick = false;
    SetActorHiddenInGame(true);
    SetActorEnableCollision(false);
}

/**
 * @brief 启动时注册到 AgentManager
 *
 * 如果场景中已有指向本对象的注册项，则优先复用其 ID，
 * 避免同一个 GuidanceActor 被重复注册为多个名称。
 */
void AGuidanceActor::BeginPlay()
{
    Super::BeginPlay();

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

        if (!ExistingId.IsEmpty() && ExistingId != GuidanceId)
        {
            GuidanceId = ExistingId;
        }

        if (Manager->GetAgent(GuidanceId) != this)
        {
            Manager->RegisterAgent(GuidanceId, this);
        }
    }
}

/**
 * @brief 结束时释放内部资源并注销自身
 * @param EndPlayReason Actor 结束原因
 */
void AGuidanceActor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    if (CurrentMethod)
    {
        delete CurrentMethod;
        CurrentMethod = nullptr;
    }

    UAgentManager* Manager = UAgentManager::GetInstance();
    if (Manager && !GuidanceId.IsEmpty() && Manager->GetAgent(GuidanceId) == this)
    {
        Manager->UnregisterAgent(GuidanceId);
    }

    Super::EndPlay(EndPlayReason);
}

/**
 * @brief 惰性初始化核心子模块
 *
 * - `Predictor`：目标状态估计与延迟补偿；
 * - `VisualInterceptController`：视觉拦截闭环控制；
 * - `CurrentMethod`：默认采用预测制导算法。
 */
void AGuidanceActor::EnsureInitialized()
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
 * @brief 按任务角色查找首个匹配的无人机
 * @param Manager AgentManager 实例
 * @param DesiredRole 目标角色
 * @param ExcludeId 需要跳过的 ID
 * @return 找到的无人机指针，失败返回 nullptr
 */
ADronePawn* AGuidanceActor::FindDroneByRole(UAgentManager* Manager, EDroneMissionRole DesiredRole, const FString& ExcludeId) const
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
        if (Drone && Drone->MissionRole == DesiredRole)
        {
            return Drone;
        }
    }

    return nullptr;
}

/** @brief 生成统一错误返回 JSON */
FString AGuidanceActor::MakeError(const FString& Msg) const
{
    return FString::Printf(TEXT("{\"status\":\"error\",\"message\":\"%s\"}"), *Msg.ReplaceCharWithEscapedChar());
}

/** @brief 生成统一成功返回 JSON */
FString AGuidanceActor::MakeOk(const FString& Msg) const
{
    return FString::Printf(TEXT("{\"status\":\"ok\",\"message\":\"%s\"}"), *Msg.ReplaceCharWithEscapedChar());
}

/**
 * @brief 切换常规瞄准/制导算法
 * @param Method 算法名称
 * @param NavConstant 比例导引导航常数
 * @param Iterations 预测制导迭代次数
 */
FString AGuidanceActor::SetMethod(FString Method, float NavConstant, int32 Iterations)
{
    EnsureInitialized();

    Method.TrimStartAndEndInline();
    Method.ToLowerInline();

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
        CurrentMethod = new FProportionalNavigation(NavConstant <= 0.0f ? 4.0f : NavConstant);
    }
    else if (Method == TEXT("predictive"))
    {
        CurrentMethod = new FPredictiveGuidance(Predictor, Iterations > 0 ? Iterations : 3);
    }
    else
    {
        return MakeError(FString::Printf(TEXT("Unknown method: %s"), *Method));
    }

    CurrentMethodName = Method;
    return MakeOk(FString::Printf(TEXT("Method set to %s"), *Method));
}

/**
 * @brief 更新无人机自动拦截的默认方法与参数
 * @param Method 拦截方法名
 * @param Speed 拦截速度上限（m/s）
 * @param NavGain 导航增益
 * @param LeadTime 目标前置预测时间（s）
 * @param CaptureRadiusValue 捕获半径（m）
 */
FString AGuidanceActor::SetInterceptMethod(FString Method, float Speed, float NavGain, float LeadTime, float CaptureRadiusValue)
{
    EnsureInitialized();

    if (!Method.IsEmpty())
    {
        CurrentInterceptMethod = NormalizeInterceptMethodName(Method);
    }
    if (Speed > 0.0f)
    {
        InterceptorSpeed = Speed;
    }
    if (NavGain > 0.0f)
    {
        InterceptNavGain = NavGain;
    }
    if (LeadTime >= 0.0f)
    {
        InterceptLeadTime = LeadTime;
    }
    if (CaptureRadiusValue > 0.0f)
    {
        CaptureRadius = CaptureRadiusValue;
    }

    return FString::Printf(
        TEXT("{\"status\":\"ok\",\"intercept_method\":\"%s\",\"speed\":%.2f,\"nav_gain\":%.2f,\"lead_time\":%.2f,\"capture_radius\":%.2f}"),
        *CurrentInterceptMethod,
        InterceptorSpeed,
        InterceptNavGain,
        InterceptLeadTime,
        CaptureRadius);
}

/**
 * @brief 统计当前场景中可参与拦截任务的无人机
 * @return 目标机与拦截机 ID 列表 JSON
 */
FString AGuidanceActor::ListInterceptAgents()
{
    EnsureInitialized();

    UAgentManager* Manager = UAgentManager::GetInstance();
    if (!Manager)
    {
        return MakeError(TEXT("Agent manager unavailable"));
    }

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

/**
 * @brief 计算一步无人机自动拦截控制并直接写入拦截机
 * @param InterceptorId 拦截机 ID
 * @param TargetId 目标机 ID
 * @param Method 临时覆盖的拦截方法
 * @param Speed 临时覆盖的速度上限（m/s）
 * @param NavGain 临时覆盖的导航增益 N
 * @param LeadTime 临时覆盖的预测时间（s）
 * @param CaptureRadiusValue 临时覆盖的捕获半径（m）
 * @param bStopOnCapture 捕获后是否悬停
 * @return 当前一步拦截状态 JSON
 *
 * 当前实现采用两类速度指令模型：
 * 1. 纯追踪/前置法：
 *    - $p_{pred} = p_t + v_t \cdot t_{lead}$
 *    - $v_{cmd} = speed \cdot \mathrm{normalize}(p_{pred} - p_i)$
 * 2. 简化比例导航：
 *    - $v_{rel} = v_t - v_i$
 *    - $LOS = \mathrm{normalize}(p_t - p_i)$
 *    - $v_{rel}^{lat} = v_{rel} - (v_{rel}\cdot LOS)LOS$
 *    - $v_{cmd} = v_t + N \cdot v_{rel}^{lat}$
 *
 * 捕获判据为：
 * $\|p_t - p_i\| \le R_{capture}$。
 */
FString AGuidanceActor::AutoIntercept(
    FString InterceptorId,
    FString TargetId,
    FString Method,
    float Speed,
    float NavGain,
    float LeadTime,
    float CaptureRadiusValue,
    bool bStopOnCapture)
{
    EnsureInitialized();

    UAgentManager* Manager = UAgentManager::GetInstance();
    if (!Manager)
    {
        return MakeError(TEXT("Agent manager unavailable"));
    }

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

    FString EffectiveMethod = CurrentInterceptMethod;
    if (!Method.IsEmpty())
    {
        EffectiveMethod = NormalizeInterceptMethodName(Method);
    }

    const float EffectiveSpeed = (Speed > 0.0f) ? Speed : InterceptorSpeed;
    const float EffectiveNavGain = (NavGain > 0.0f) ? NavGain : InterceptNavGain;
    const float EffectiveLeadTime = (LeadTime >= 0.0f) ? LeadTime : InterceptLeadTime;
    const float EffectiveCaptureRadius = (CaptureRadiusValue > 0.0f) ? CaptureRadiusValue : CaptureRadius;

    const FVector InterceptorPos = InterceptorDrone->GetCurrentPosition();
    const FVector InterceptorVel = InterceptorDrone->GetCurrentVelocity();
    const FVector TargetPos = TargetDrone->GetCurrentPosition();
    const FVector TargetVel = TargetDrone->GetCurrentVelocity();

    const FVector RelativePos = TargetPos - InterceptorPos;
    const FVector RelativeVel = TargetVel - InterceptorVel;
    const float Distance = RelativePos.Size();
    const FVector LOS = RelativePos.GetSafeNormal();

    // 闭合速度采用视线方向上的相对速度投影：closing = -v_rel·LOS。
    const float ClosingSpeed = -FVector::DotProduct(RelativeVel, LOS);

    FVector CommandVelocity = FVector::ZeroVector;
    const bool bCaptured = Distance <= EffectiveCaptureRadius;
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
        if (EffectiveMethod == TEXT("proportional_nav"))
        {
            const FVector LateralRelVel = RelativeVel - FVector::DotProduct(RelativeVel, LOS) * LOS;
            CommandVelocity = TargetVel + EffectiveNavGain * LateralRelVel;
            if (CommandVelocity.IsNearlyZero())
            {
                CommandVelocity = LOS * EffectiveSpeed;
            }
        }
        else
        {
            const FVector PredictedTarget = TargetPos + TargetVel * EffectiveLeadTime;
            CommandVelocity = (PredictedTarget - InterceptorPos).GetSafeNormal() * EffectiveSpeed;
        }

        if (!CommandVelocity.IsNearlyZero())
        {
            CommandVelocity = CommandVelocity.GetClampedToMaxSize(EffectiveSpeed);
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
        *EffectiveMethod,
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

/**
 * @brief 向 Kalman 预测器输入一帧目标位置观测
 * @param X 观测位置 X
 * @param Y 观测位置 Y
 * @param Z 观测位置 Z
 * @param Dt 采样时间间隔（s）
 * @return 当前估计状态 JSON
 */
FString AGuidanceActor::UpdateTarget(float X, float Y, float Z, float Dt)
{
    EnsureInitialized();

    Predictor->Update(FVector(X, Y, Z), Dt);
    const FVector EstPos = Predictor->GetEstimatedPosition();
    const FVector EstVel = Predictor->GetEstimatedVelocity();
    const FVector EstAcc = Predictor->GetEstimatedAcceleration();
    const float AdaptiveQ = Predictor->GetAdaptiveProcessNoise();

    return FString::Printf(
        TEXT("{\"status\":\"ok\",\"est_pos\":[%.2f,%.2f,%.2f],\"est_vel\":[%.2f,%.2f,%.2f],\"est_acc\":[%.2f,%.2f,%.2f],\"adaptive_q\":%.4f}"),
        EstPos.X,
        EstPos.Y,
        EstPos.Z,
        EstVel.X,
        EstVel.Y,
        EstVel.Z,
        EstAcc.X,
        EstAcc.Y,
        EstAcc.Z,
        AdaptiveQ);
}

/**
 * @brief 基于当前目标预测状态计算炮塔瞄准解
 * @param TurretId 炮塔 ID
 * @param MuzzleSpeed 弹丸初速度（m/s）
 * @return 瞄准角与预计飞行时间 JSON
 */
FString AGuidanceActor::ComputeAim(FString TurretId, float MuzzleSpeed)
{
    EnsureInitialized();

    UAgentManager* Manager = UAgentManager::GetInstance();
    if (!Manager)
    {
        return MakeError(TEXT("Agent manager unavailable"));
    }

    const FString EffectiveTurretId = TurretId.IsEmpty() ? TEXT("turret_0") : TurretId;
    ATurretPawn* Turret = Cast<ATurretPawn>(Manager->GetAgent(EffectiveTurretId));
    if (!Turret)
    {
        return MakeError(FString::Printf(TEXT("Turret '%s' not found"), *EffectiveTurretId));
    }

    FGuidanceInput Input;
    Input.TurretPos = Turret->GetActorLocation();
    Input.MuzzlePos = GetTurretMuzzlePosition(Turret);
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
        Output.Pitch,
        Output.Yaw,
        Output.AimPoint.X,
        Output.AimPoint.Y,
        Output.AimPoint.Z,
        Output.EstFlightTime,
        *CurrentMethodName);
}

/**
 * @brief 自动完成一次“观测 -> 预测 -> 瞄准 -> 可选开火”流程
 * @param TurretId 炮塔 ID
 * @param TargetId 目标 ID
 * @param MuzzleSpeed 弹丸初速度（m/s）
 * @param Dt 观测周期（s）
 * @param Latency 延迟补偿时间（s）
 * @param bFire 是否触发开火
 * @return 本次自动交战结果 JSON
 *
 * 延迟补偿的核心思想为：
 * $p_{comp} = \hat{p}(t + latency)$，
 * 即先用预测器把目标状态外推到射击生效时刻，再交由制导算法求解瞄准角。
 */
FString AGuidanceActor::AutoEngage(FString TurretId, FString TargetId, float MuzzleSpeed, float Dt, float Latency, bool bFire)
{
    EnsureInitialized();

    UAgentManager* Manager = UAgentManager::GetInstance();
    if (!Manager)
    {
        return MakeError(TEXT("Agent manager unavailable"));
    }

    const FString EffectiveTargetId = TargetId.IsEmpty() ? TEXT("drone_0") : TargetId;
    const FString EffectiveTurretId = TurretId.IsEmpty() ? TEXT("turret_0") : TurretId;
    AActor* Target = Manager->GetAgent(EffectiveTargetId);
    ATurretPawn* Turret = Cast<ATurretPawn>(Manager->GetAgent(EffectiveTurretId));
    if (!Target)
    {
        return MakeError(FString::Printf(TEXT("Target '%s' not found"), *EffectiveTargetId));
    }
    if (!Turret)
    {
        return MakeError(FString::Printf(TEXT("Turret '%s' not found"), *EffectiveTurretId));
    }

    const float EffectiveLatency = (Latency >= 0.0f) ? FMath::Clamp(Latency, 0.0f, 1.0f) : DefaultVisionLatency;
    LastLatencyCompensation = EffectiveLatency;

    Predictor->Update(Target->GetActorLocation(), Dt);
    const FVector CompensatedTargetPos = Predictor->PredictPosition(EffectiveLatency);

    FGuidanceInput Input;
    Input.TurretPos = Turret->GetActorLocation();
    Input.MuzzlePos = GetTurretMuzzlePosition(Turret);
    Input.TargetPos = CompensatedTargetPos;
    Input.TargetVel = Predictor->GetEstimatedVelocity();
    Input.PredictedPos = Predictor->PredictPosition(EffectiveLatency + 0.5f);
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

    if (bFire)
    {
        Turret->FireX(MuzzleSpeed);
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

/**
 * @brief 重新设置 Kalman 预测器噪声参数
 * @param ProcessNoise 过程噪声
 * @param MeasurementNoise 观测噪声
 */
FString AGuidanceActor::SetKalmanParams(float ProcessNoise, float MeasurementNoise)
{
    EnsureInitialized();
    Predictor->Initialize(ProcessNoise, MeasurementNoise);
    return MakeOk(FString::Printf(TEXT("kalman Q=%.2f R=%.2f"), ProcessNoise, MeasurementNoise));
}

/**
 * @brief 重置 GuidanceActor 的内部运行状态
 *
 * 该接口会清空：
 * - Kalman 预测器历史状态；
 * - 当前制导算法的内部记忆量；
 * - 最近一次瞄准/拦截结果缓存；
 * - 视觉拦截控制器运行状态。
 */
FString AGuidanceActor::ResetGuidance()
{
    EnsureInitialized();

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

/**
 * @brief 汇总当前制导、预测与拦截状态
 * @return 完整状态 JSON
 */
FString AGuidanceActor::GetState()
{
    EnsureInitialized();

    const FVector EstPos = Predictor->GetEstimatedPosition();
    const FVector EstVel = Predictor->GetEstimatedVelocity();
    const FVector EstAcc = Predictor->GetEstimatedAcceleration();
    const float Uncertainty = Predictor->GetPositionUncertainty();
    const float AdaptiveQ = Predictor->GetAdaptiveProcessNoise();

    return FString::Printf(
        TEXT("{\"status\":\"ok\",\"id\":\"%s\",\"method\":\"%s\",\"initialized\":%s,\"est_pos\":[%.2f,%.2f,%.2f],\"est_vel\":[%.2f,%.2f,%.2f],\"est_acc\":[%.2f,%.2f,%.2f],\"uncertainty\":%.2f,\"adaptive_q\":%.4f,\"aim_pitch\":%.2f,\"aim_yaw\":%.2f,\"aim_point\":[%.1f,%.1f,%.1f],\"flight_time\":%.4f,\"latency\":%.4f,\"intercept\":{\"method\":\"%s\",\"target_id\":\"%s\",\"interceptor_id\":\"%s\",\"distance\":%.3f,\"closing_speed\":%.3f,\"captured\":%s,\"valid\":%s,\"cmd_velocity\":[%.3f,%.3f,%.3f]}}"),
        *GuidanceId.ReplaceCharWithEscapedChar(),
        *CurrentMethodName,
        *BoolLiteral(Predictor->IsInitialized()),
        EstPos.X,
        EstPos.Y,
        EstPos.Z,
        EstVel.X,
        EstVel.Y,
        EstVel.Z,
        EstAcc.X,
        EstAcc.Y,
        EstAcc.Z,
        Uncertainty,
        AdaptiveQ,
        LastPitch,
        LastYaw,
        LastAimPoint.X,
        LastAimPoint.Y,
        LastAimPoint.Z,
        LastFlightTime,
        LastLatencyCompensation,
        *CurrentInterceptMethod,
        *LastTargetId.ReplaceCharWithEscapedChar(),
        *LastInterceptorId.ReplaceCharWithEscapedChar(),
        LastDistanceToTarget,
        LastClosingSpeed,
        *BoolLiteral(bLastCaptured),
        *BoolLiteral(bLastInterceptValid),
        LastInterceptorCmdVel.X,
        LastInterceptorCmdVel.Y,
        LastInterceptorCmdVel.Z);
}

/**
 * @brief 将 Blueprint/TCP 参数打包成启动命令并转发给视觉拦截控制器
 * @return 视觉拦截启动结果 JSON
 */
FString AGuidanceActor::VisualInterceptStart(
    FString InterceptorId,
    FString TargetId,
    FString Method,
    float DesiredArea,
    float CaptureArea,
    float CenterTolX,
    float CenterTolY,
    int32 CaptureHoldFrames,
    int32 LostToSearchFrames,
    float MaxForwardSpeed,
    float MaxReverseSpeed,
    float MaxVerticalSpeed,
    float MaxYawRateDeg,
    float SearchCamYawLimitDeg,
    float SearchCamRateDeg,
    float SearchBodyYawRateDeg,
    float SearchCamPitchDeg,
    float SearchVzAmp,
    int32 StopOnCaptureFlag,
    int32 UseKalmanFlag)
{
    EnsureInitialized();

    const TSharedPtr<FJsonObject> Cmd = MakeShareable(new FJsonObject);
    SetStringIfNotEmpty(Cmd, TEXT("interceptor_id"), InterceptorId);
    SetStringIfNotEmpty(Cmd, TEXT("target_id"), TargetId);
    SetStringIfNotEmpty(Cmd, TEXT("method"), Method);
    SetNumberIfValid(Cmd, TEXT("desired_area"), DesiredArea);
    SetNumberIfValid(Cmd, TEXT("capture_area"), CaptureArea);
    SetNumberIfValid(Cmd, TEXT("center_tol_x"), CenterTolX);
    SetNumberIfValid(Cmd, TEXT("center_tol_y"), CenterTolY);
    SetIntIfValid(Cmd, TEXT("capture_hold_frames"), CaptureHoldFrames);
    SetIntIfValid(Cmd, TEXT("lost_to_search_frames"), LostToSearchFrames);
    SetNumberIfValid(Cmd, TEXT("max_forward_speed"), MaxForwardSpeed);
    SetNumberIfValid(Cmd, TEXT("max_reverse_speed"), MaxReverseSpeed);
    SetNumberIfValid(Cmd, TEXT("max_vertical_speed"), MaxVerticalSpeed);
    SetNumberIfValid(Cmd, TEXT("max_yaw_rate_deg"), MaxYawRateDeg);
    SetNumberIfValid(Cmd, TEXT("search_cam_yaw_limit_deg"), SearchCamYawLimitDeg);
    SetNumberIfValid(Cmd, TEXT("search_cam_rate_deg"), SearchCamRateDeg);
    SetNumberIfValid(Cmd, TEXT("search_body_yaw_rate_deg"), SearchBodyYawRateDeg);
    SetNumberIfValid(Cmd, TEXT("search_vz_amp"), SearchVzAmp);
    if (!FMath::IsNearlyEqual(SearchCamPitchDeg, -1000.0f))
    {
        Cmd->SetNumberField(TEXT("search_cam_pitch_deg"), SearchCamPitchDeg);
    }
    if (StopOnCaptureFlag >= 0)
    {
        Cmd->SetBoolField(TEXT("stop_on_capture"), StopOnCaptureFlag != 0);
    }
    if (UseKalmanFlag >= 0)
    {
        Cmd->SetBoolField(TEXT("use_kalman"), UseKalmanFlag != 0);
    }

    return VisualInterceptController->HandleStart(Cmd, GetWorld());
}

/**
 * @brief 提交一帧视觉检测结果给视觉拦截控制器
 * @return 本帧视觉拦截状态 JSON
 */
FString AGuidanceActor::VisualInterceptUpdate(
    int32 HasDetection,
    float Cx,
    float Cy,
    float Area,
    float AreaRatio,
    float Conf,
    float Dt,
    float ImageW,
    float ImageH,
    FString InterceptorId,
    FString TargetId)
{
    EnsureInitialized();

    const TSharedPtr<FJsonObject> Cmd = MakeShareable(new FJsonObject);
    Cmd->SetBoolField(TEXT("has_detection"), HasDetection != 0);
    Cmd->SetNumberField(TEXT("cx"), Cx);
    Cmd->SetNumberField(TEXT("cy"), Cy);
    Cmd->SetNumberField(TEXT("area"), Area);
    Cmd->SetNumberField(TEXT("area_ratio"), AreaRatio);
    Cmd->SetNumberField(TEXT("conf"), Conf);
    Cmd->SetNumberField(TEXT("dt"), Dt);
    Cmd->SetNumberField(TEXT("image_w"), ImageW);
    Cmd->SetNumberField(TEXT("image_h"), ImageH);
    SetStringIfNotEmpty(Cmd, TEXT("interceptor_id"), InterceptorId);
    SetStringIfNotEmpty(Cmd, TEXT("target_id"), TargetId);

    return VisualInterceptController->HandleUpdate(Cmd, GetWorld());
}

/**
 * @brief 停止视觉拦截会话
 * @return 停止结果 JSON
 */
FString AGuidanceActor::VisualInterceptStop(FString InterceptorId, FString TargetId)
{
    EnsureInitialized();

    const TSharedPtr<FJsonObject> Cmd = MakeShareable(new FJsonObject);
    SetStringIfNotEmpty(Cmd, TEXT("interceptor_id"), InterceptorId);
    SetStringIfNotEmpty(Cmd, TEXT("target_id"), TargetId);
    return VisualInterceptController->HandleStop(Cmd, GetWorld());
}

/** @brief 查询视觉拦截控制器当前状态 */
FString AGuidanceActor::VisualInterceptState()
{
    EnsureInitialized();
    return VisualInterceptController->HandleState();
}
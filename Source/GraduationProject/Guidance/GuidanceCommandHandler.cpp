#include "GuidanceCommandHandler.h"
#include "KalmanPredictor.h"
#include "GuidanceMethods.h"
#include "GraduationProject/Core/Manager/AgentManager.h"
#include "GraduationProject/Turret/TurretPawn.h"
#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"

/**
 * @brief 生成错误响应 JSON
 * @param Msg 错误描述信息
 * @return {"status":"error","message":"<Msg>"}
 */
FString UGuidanceCommandHandler::MakeError(const FString& Msg)
{
    return FString::Printf(TEXT("{\"status\":\"error\",\"message\":\"%s\"}"), *Msg);
}

/**
 * @brief 生成成功响应 JSON
 * @param Msg 成功描述信息
 * @return {"status":"ok","message":"<Msg>"}
 */
FString UGuidanceCommandHandler::MakeOk(const FString& Msg)
{
    return FString::Printf(TEXT("{\"status\":\"ok\",\"message\":\"%s\"}"), *Msg);
}

/** @brief 构造函数 */
UGuidanceCommandHandler::UGuidanceCommandHandler()
    : CurrentMethod(nullptr)
    , CurrentMethodName(TEXT("predictive"))
{
}

/** @brief 析构函数 */
UGuidanceCommandHandler::~UGuidanceCommandHandler()
{
    if (CurrentMethod)
    {
        delete CurrentMethod;
        CurrentMethod = nullptr;
    }
}

/** @brief 初始化卡尔曼预测器和默认制导方法 */
void UGuidanceCommandHandler::EnsureInitialized()
{
    if (!Predictor)
    {
        Predictor = NewObject<UKalmanPredictor>(this);           
        Predictor->Initialize(100.0f, 0.01f);
    }
    if (!CurrentMethod)
    {
        CurrentMethod = new FPredictiveGuidance(Predictor, 3);
        CurrentMethodName = TEXT("predictive");
    }
}

/**
 * @brief 处理 call_guidance 命令
 * @param JsonObject 完整的 JSON 请求对象
 * @param World 当前 UWorld 指针
 * @return JSON 格式的响应字符串
 */
FString UGuidanceCommandHandler::HandleCallGuidance(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World)
{
    const TSharedPtr<FJsonObject>* CmdObj = nullptr;
    if (!JsonObject->TryGetObjectField(TEXT("call_guidance"), CmdObj)) return MakeError(TEXT("Missing call_guidance field"));
    FString Function;
    (*CmdObj)->TryGetStringField(TEXT("function"), Function);
    EnsureInitialized();
    if (Function == TEXT("set_method"))
    {
        FString Method;
        if (!(*CmdObj)->TryGetStringField(TEXT("method"), Method)) return MakeError(TEXT("Missing method name"));
        if (CurrentMethod) { delete CurrentMethod; CurrentMethod = nullptr; }
        if (Method == TEXT("direct")) CurrentMethod = new FDirectAiming();
        else if (Method == TEXT("proportional"))
        {
            float N = 4.0f;  // 导引比例常数
            if ((*CmdObj)->HasField(TEXT("nav_constant")))
                N = (*CmdObj)->GetNumberField(TEXT("nav_constant"));
            CurrentMethod = new FProportionalNavigation(N);
        }
        else if (Method == TEXT("predictive"))
        {
            int32 Iters = 3;  // 迭代次数
            if ((*CmdObj)->HasField(TEXT("iterations")))
                Iters = (int32)(*CmdObj)->GetNumberField(TEXT("iterations"));
            CurrentMethod = new FPredictiveGuidance(Predictor, Iters);
        }
        else return MakeError(FString::Printf(TEXT("Unknown method: %s"), *Method));
        CurrentMethodName = Method;
        return MakeOk(FString::Printf(TEXT("Method set to %s"), *Method));
    }
    if (Function == TEXT("update_target"))
    {
        float X = (*CmdObj)->GetNumberField(TEXT("x"));
        float Y = (*CmdObj)->GetNumberField(TEXT("y"));
        float Z = (*CmdObj)->GetNumberField(TEXT("z"));
        float Dt = 0.1f;
        if ((*CmdObj)->HasField(TEXT("dt")))
            Dt = (*CmdObj)->GetNumberField(TEXT("dt"));
        Predictor->Update(FVector(X, Y, Z), Dt);
        FVector EstPos = Predictor->GetEstimatedPosition();
        FVector EstVel = Predictor->GetEstimatedVelocity();
        return FString::Printf(
            TEXT("{\"status\":\"ok\",\"est_pos\":[%.2f,%.2f,%.2f],\"est_vel\":[%.2f,%.2f,%.2f]}"),
            EstPos.X, EstPos.Y, EstPos.Z,
            EstVel.X, EstVel.Y, EstVel.Z);
    }
    if (Function == TEXT("compute_aim"))
    {
        FString TurretId;
        if (!(*CmdObj)->TryGetStringField(TEXT("turret_id"), TurretId)) TurretId = TEXT("turret_0");
        float MuzzleSpeed = 400.0f;
        if ((*CmdObj)->HasField(TEXT("muzzle_speed"))) MuzzleSpeed = (*CmdObj)->GetNumberField(TEXT("muzzle_speed"));
        UAgentManager* Manager = UAgentManager::GetInstance();
        ATurretPawn* Turret = Cast<ATurretPawn>(Manager->GetAgent(TurretId));
        if (!Turret) return MakeError(FString::Printf(TEXT("Turret '%s' not found"), *TurretId));
        FGuidanceInput Input;
        Input.TurretPos = Turret->GetActorLocation();
        Input.MuzzlePos = Turret->GunMesh ?
            (Turret->GunMesh->GetComponentLocation() +
             Turret->GunMesh->GetComponentRotation().RotateVector(Turret->MuzzleOffset))
            : Input.TurretPos;
        Input.TargetPos = Predictor->GetEstimatedPosition();
        Input.TargetVel = Predictor->GetEstimatedVelocity();
        Input.PredictedPos = Predictor->PredictPosition(0.5f);
        Input.MuzzleSpeed = MuzzleSpeed;
        Input.DeltaTime = 0.1f;
        FGuidanceOutput Output = CurrentMethod->ComputeAim(Input);
        LastPitch = Output.Pitch;
        LastYaw = Output.Yaw;
        LastAimPoint = Output.AimPoint;
        LastFlightTime = Output.EstFlightTime;
        return FString::Printf(
            TEXT("{\"status\":\"ok\",\"pitch\":%.2f,\"yaw\":%.2f,\"aim_point\":[%.1f,%.1f,%.1f],\"flight_time\":%.4f,\"method\":\"%s\"}"),
            Output.Pitch, Output.Yaw,
            Output.AimPoint.X, Output.AimPoint.Y, Output.AimPoint.Z,
            Output.EstFlightTime, *CurrentMethodName);
    }
    if (Function == TEXT("auto_engage"))
    {
        FString TurretId;
        if (!(*CmdObj)->TryGetStringField(TEXT("turret_id"), TurretId)) TurretId = TEXT("turret_0");
        FString TargetId;
        if (!(*CmdObj)->TryGetStringField(TEXT("target_id"), TargetId)) TargetId = TEXT("drone_0");
        float MuzzleSpeed = 400.0f;
        if ((*CmdObj)->HasField(TEXT("muzzle_speed"))) MuzzleSpeed = (*CmdObj)->GetNumberField(TEXT("muzzle_speed"));
        UAgentManager* Manager = UAgentManager::GetInstance();
        AActor* Target = Manager->GetAgent(TargetId);
        ATurretPawn* Turret = Cast<ATurretPawn>(Manager->GetAgent(TurretId));
        if (!Target) return MakeError(FString::Printf(TEXT("Target '%s' not found"), *TargetId));
        if (!Turret) return MakeError(FString::Printf(TEXT("Turret '%s' not found"), *TurretId));
        float Dt = 0.05f;
        if ((*CmdObj)->HasField(TEXT("dt"))) Dt = (*CmdObj)->GetNumberField(TEXT("dt"));
        FVector TargetPos = Target->GetActorLocation();
        Predictor->Update(TargetPos, Dt);
        FGuidanceInput Input;
        Input.TurretPos = Turret->GetActorLocation();
        Input.MuzzlePos = Turret->GunMesh ?
            (Turret->GunMesh->GetComponentLocation() +
             Turret->GunMesh->GetComponentRotation().RotateVector(Turret->MuzzleOffset))
            : Input.TurretPos;
        Input.TargetPos = TargetPos;
        Input.TargetVel = Predictor->GetEstimatedVelocity();
        Input.PredictedPos = Predictor->PredictPosition(0.5f);
        Input.MuzzleSpeed = MuzzleSpeed;
        Input.DeltaTime = Dt;
        FGuidanceOutput Output = CurrentMethod->ComputeAim(Input);
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
            TEXT("{\"status\":\"ok\",\"pitch\":%.2f,\"yaw\":%.2f,\"fired\":%s,\"flight_time\":%.4f,\"method\":\"%s\"}"),
            Output.Pitch, Output.Yaw,
            bFire ? TEXT("true") : TEXT("false"),
            Output.EstFlightTime, *CurrentMethodName);
    }
    if (Function == TEXT("reset"))
    {
        Predictor->Reset();
        Predictor->Initialize(100.0f, 0.01f);
        if (CurrentMethod) CurrentMethod->Reset();
        LastPitch = 0; LastYaw = 0;
        LastAimPoint = FVector::ZeroVector;
        LastFlightTime = 0;
        return MakeOk(TEXT("guidance reset"));
    }
    if (Function == TEXT("set_kalman_params"))
    {
        float Q = 1.0f, R = 0.5f;
        if ((*CmdObj)->HasField(TEXT("process_noise"))) Q = (*CmdObj)->GetNumberField(TEXT("process_noise"));
        if ((*CmdObj)->HasField(TEXT("measurement_noise"))) R = (*CmdObj)->GetNumberField(TEXT("measurement_noise"));
        Predictor->Initialize(Q, R);
        return MakeOk(FString::Printf(TEXT("kalman Q=%.2f R=%.2f"), Q, R));
    }
    return MakeError(FString::Printf(TEXT("Unknown function: %s"), *Function));
}

/**
 * @brief 处理 get_guidance_state 命令
 * @param JsonObject 完整的 JSON 请求对象
 * @param World 当前 UWorld 指针
 * @return JSON 格式的制导系统状态快照
 * 返回字段：
 * - method: 当前制导方法名
 * - initialized: 卡尔曼滤波器是否已初始化
 * - est_pos: 卡尔曼估计位置 [X, Y, Z]
 * - est_vel: 卡尔曼估计速度 [VX, VY, VZ]
 * - uncertainty: 位置不确定性
 * - aim_pitch/aim_yaw: 最近一次计算的瞄准角度
 * - aim_point: 最近一次计算的瞄准点世界坐标
 * - flight_time: 最近一次估算的弹丸飞行时间
 */
FString UGuidanceCommandHandler::HandleGetGuidanceState(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World)
{
    EnsureInitialized();
    FVector EstPos = Predictor->GetEstimatedPosition();
    FVector EstVel = Predictor->GetEstimatedVelocity();
    float Uncertainty = Predictor->GetPositionUncertainty();
    return FString::Printf(
        TEXT("{\"status\":\"ok\",\"method\":\"%s\",\"initialized\":%s,")
        TEXT("\"est_pos\":[%.2f,%.2f,%.2f],\"est_vel\":[%.2f,%.2f,%.2f],")
        TEXT("\"uncertainty\":%.2f,\"aim_pitch\":%.2f,\"aim_yaw\":%.2f,")
        TEXT("\"aim_point\":[%.1f,%.1f,%.1f],\"flight_time\":%.4f}"),
        *CurrentMethodName,
        Predictor->IsInitialized() ? TEXT("true") : TEXT("false"),
        EstPos.X, EstPos.Y, EstPos.Z,
        EstVel.X, EstVel.Y, EstVel.Z,
        Uncertainty,
        LastPitch, LastYaw,
        LastAimPoint.X, LastAimPoint.Y, LastAimPoint.Z,
        LastFlightTime);
}

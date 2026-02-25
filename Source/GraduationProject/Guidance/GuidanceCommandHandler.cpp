#include "GuidanceCommandHandler.h"
#include "KalmanPredictor.h"
#include "GuidanceMethods.h"
#include "GraduationProject/Core/Manager/AgentManager.h"
#include "GraduationProject/Turret/TurretPawn.h"
#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"

// ---- 工具方法 ----

FString UGuidanceCommandHandler::MakeError(const FString& Msg)
{
    return FString::Printf(TEXT("{\"status\":\"error\",\"message\":\"%s\"}"), *Msg);
}

FString UGuidanceCommandHandler::MakeOk(const FString& Msg)
{
    return FString::Printf(TEXT("{\"status\":\"ok\",\"message\":\"%s\"}"), *Msg);
}

UGuidanceCommandHandler::UGuidanceCommandHandler()
    : CurrentMethod(nullptr)
    , CurrentMethodName(TEXT("predictive"))
{
}

UGuidanceCommandHandler::~UGuidanceCommandHandler()
{
    if (CurrentMethod)
    {
        delete CurrentMethod;
        CurrentMethod = nullptr;
    }
}

void UGuidanceCommandHandler::EnsureInitialized()
{
    if (!Predictor)
    {
        Predictor = NewObject<UKalmanPredictor>(this);
        // 修改为高追踪灵敏度 (Q=100.0过程噪声高, R=0.01测量极其相信传感器)
        Predictor->Initialize(100.0f, 0.01f);
    }

    if (!CurrentMethod)
    {
        // 默认使用卡尔曼预测制导
        CurrentMethod = new FPredictiveGuidance(Predictor, 3);
        CurrentMethodName = TEXT("predictive");
    }
}

// ---- call_guidance ----

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

    // ---- set_method ----
    if (Function == TEXT("set_method"))
    {
        FString Method;
        if (!(*CmdObj)->TryGetStringField(TEXT("method"), Method))
        {
            return MakeError(TEXT("Missing method name"));
        }

        // 释放旧方法
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
                N = (*CmdObj)->GetNumberField(TEXT("nav_constant"));
            CurrentMethod = new FProportionalNavigation(N);
        }
        else if (Method == TEXT("predictive"))
        {
            int32 Iters = 3;
            if ((*CmdObj)->HasField(TEXT("iterations")))
                Iters = (int32)(*CmdObj)->GetNumberField(TEXT("iterations"));
            CurrentMethod = new FPredictiveGuidance(Predictor, Iters);
        }
        else
        {
            return MakeError(FString::Printf(TEXT("Unknown method: %s"), *Method));
        }

        CurrentMethodName = Method;
        return MakeOk(FString::Printf(TEXT("method set to %s"), *Method));
    }

    // ---- update_target ----
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

    // ---- compute_aim ----
    if (Function == TEXT("compute_aim"))
    {
        FString TurretId;
        if (!(*CmdObj)->TryGetStringField(TEXT("turret_id"), TurretId))
            TurretId = TEXT("turret_0");

        float MuzzleSpeed = 400.0f;
        if ((*CmdObj)->HasField(TEXT("muzzle_speed")))
            MuzzleSpeed = (*CmdObj)->GetNumberField(TEXT("muzzle_speed"));

        // 查找转台
        UAgentManager* Manager = UAgentManager::GetInstance();
        ATurretPawn* Turret = Cast<ATurretPawn>(Manager->GetAgent(TurretId));
        if (!Turret)
        {
            return MakeError(FString::Printf(TEXT("Turret '%s' not found"), *TurretId));
        }

        // 构建制导输入
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

    // ---- auto_engage ----
    if (Function == TEXT("auto_engage"))
    {
        FString TurretId;
        if (!(*CmdObj)->TryGetStringField(TEXT("turret_id"), TurretId))
            TurretId = TEXT("turret_0");

        FString TargetId;
        if (!(*CmdObj)->TryGetStringField(TEXT("target_id"), TargetId))
            TargetId = TEXT("drone_0");

        float MuzzleSpeed = 400.0f;
        if ((*CmdObj)->HasField(TEXT("muzzle_speed")))
            MuzzleSpeed = (*CmdObj)->GetNumberField(TEXT("muzzle_speed"));

        // 查找目标和转台
        UAgentManager* Manager = UAgentManager::GetInstance();
        AActor* Target = Manager->GetAgent(TargetId);
        ATurretPawn* Turret = Cast<ATurretPawn>(Manager->GetAgent(TurretId));

        if (!Target) return MakeError(FString::Printf(TEXT("Target '%s' not found"), *TargetId));
        if (!Turret) return MakeError(FString::Printf(TEXT("Turret '%s' not found"), *TurretId));

        float Dt = 0.05f;
        if ((*CmdObj)->HasField(TEXT("dt")))
        {
            Dt = (*CmdObj)->GetNumberField(TEXT("dt"));
        }

        // 1. 更新卡尔曼
        FVector TargetPos = Target->GetActorLocation();
        Predictor->Update(TargetPos, Dt);

        // 2. 计算瞄准
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
            // 3. 设置转台角度
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

    // ---- reset ----
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

    // ---- set_kalman_params ----
    if (Function == TEXT("set_kalman_params"))
    {
        float Q = 1.0f, R = 0.5f;
        if ((*CmdObj)->HasField(TEXT("process_noise")))
            Q = (*CmdObj)->GetNumberField(TEXT("process_noise"));
        if ((*CmdObj)->HasField(TEXT("measurement_noise")))
            R = (*CmdObj)->GetNumberField(TEXT("measurement_noise"));
        Predictor->Initialize(Q, R);
        return MakeOk(FString::Printf(TEXT("kalman Q=%.2f R=%.2f"), Q, R));
    }

    return MakeError(FString::Printf(TEXT("Unknown function: %s"), *Function));
}

// ---- get_guidance_state ----

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

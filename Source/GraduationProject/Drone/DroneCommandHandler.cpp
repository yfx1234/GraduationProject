#include "DroneCommandHandler.h"
#include "DronePawn.h"
#include "DroneApi.h"
#include "DroneMovementComponent.h"
#include "GraduationProject/Core/Manager/AgentManager.h"
#include "GraduationProject/Core/Manager/CommandExecutionManager.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"

namespace
{
    enum class ECommandFrame : uint8
    {
        UE,
        NED
    };

    ECommandFrame ParseFrame(const TSharedPtr<FJsonObject>& Obj)
    {
        FString Frame;
        if (!Obj.IsValid() || !Obj->TryGetStringField(TEXT("frame"), Frame))
        {
            return ECommandFrame::UE;
        }

        Frame.ToLowerInline();
        return (Frame == TEXT("ned")) ? ECommandFrame::NED : ECommandFrame::UE;
    }

    FString FrameToString(ECommandFrame Frame)
    {
        return (Frame == ECommandFrame::NED) ? TEXT("ned") : TEXT("ue");
    }
    FVector ConvertInputToUE(const FVector& Value, ECommandFrame Frame)
    {
        if (Frame == ECommandFrame::NED)
        {
            // NED: +Z Down, UE sim: +Z Up
            return FVector(Value.X, Value.Y, -Value.Z);
        }
        return Value;
    }

    FVector ConvertUEToOutput(const FVector& Value, ECommandFrame Frame)
    {
        if (Frame == ECommandFrame::NED)
        {
            return FVector(Value.X, Value.Y, -Value.Z);
        }
        return Value;
    }

    FRotator ConvertUEToFrameRotator(const FRotator& RotUE, ECommandFrame Frame)
    {
        if (Frame == ECommandFrame::NED)
        {
            return FRotator(-RotUE.Pitch, RotUE.Yaw, -RotUE.Roll);
        }
        return RotUE;
    }

    FRotator ConvertFrameToUERotator(const FRotator& RotFrame, ECommandFrame Frame)
    {
        if (Frame == ECommandFrame::NED)
        {
            return FRotator(-RotFrame.Pitch, RotFrame.Yaw, -RotFrame.Roll);
        }
        return RotFrame;
    }

    bool HasHeadingConfig(const TSharedPtr<FJsonObject>& Obj)
    {
        return Obj.IsValid() && (
            Obj->HasField(TEXT("yaw_mode")) ||
            Obj->HasField(TEXT("yaw")) ||
            Obj->HasField(TEXT("drivetrain")));
    }

    EDroneDrivetrainMode ParseDrivetrain(const TSharedPtr<FJsonObject>& Obj)
    {
        FString Drive;
        if (!Obj.IsValid() || !Obj->TryGetStringField(TEXT("drivetrain"), Drive))
        {
            return EDroneDrivetrainMode::MaxDegreeOfFreedom;
        }

        Drive.ToLowerInline();
        if (Drive == TEXT("max_degree_of_freedom") || Drive == TEXT("maxdof") || Drive == TEXT("max_dof"))
        {
            return EDroneDrivetrainMode::MaxDegreeOfFreedom;
        }
        return EDroneDrivetrainMode::ForwardOnly;
    }

    FString DrivetrainToString(EDroneDrivetrainMode Mode)
    {
        return (Mode == EDroneDrivetrainMode::MaxDegreeOfFreedom)
            ? TEXT("max_degree_of_freedom")
            : TEXT("forward_only");
    }

    EDroneYawMode ParseYawMode(const TSharedPtr<FJsonObject>& Obj, float& OutYawDeg)
    {
        OutYawDeg = 0.0f;

        if (!Obj.IsValid())
        {
            return EDroneYawMode::Auto;
        }

        // AirSim-style: "yaw_mode": {"is_rate": false, "yaw_or_rate": 90}
        const TSharedPtr<FJsonObject>* YawObj = nullptr;
        if (Obj->TryGetObjectField(TEXT("yaw_mode"), YawObj) && YawObj && YawObj->IsValid())
        {
            bool bIsRate = false;
            (*YawObj)->TryGetBoolField(TEXT("is_rate"), bIsRate);

            double YawOrRate = 0.0;
            if ((*YawObj)->TryGetNumberField(TEXT("yaw_or_rate"), YawOrRate))
            {
                OutYawDeg = static_cast<float>(YawOrRate);
            }

            // AirSim: is_rate=true -> Rate, is_rate=false -> Angle
            return bIsRate ? EDroneYawMode::Rate : EDroneYawMode::Angle;
        }

        FString YawMode;
        if (Obj->TryGetStringField(TEXT("yaw_mode"), YawMode))
        {
            YawMode.ToLowerInline();
            if (YawMode == TEXT("hold"))
            {
                return EDroneYawMode::Hold;
            }
            if (YawMode == TEXT("angle"))
            {
                if (Obj->HasField(TEXT("yaw")))
                {
                    OutYawDeg = static_cast<float>(Obj->GetNumberField(TEXT("yaw")));
                }
                return EDroneYawMode::Angle;
            }
            if (YawMode == TEXT("rate"))
            {
                if (Obj->HasField(TEXT("yaw")))
                {
                    OutYawDeg = static_cast<float>(Obj->GetNumberField(TEXT("yaw")));
                }
                else if (Obj->HasField(TEXT("yaw_or_rate")))
                {
                    OutYawDeg = static_cast<float>(Obj->GetNumberField(TEXT("yaw_or_rate")));
                }
                return EDroneYawMode::Rate;
            }
            return EDroneYawMode::Auto;
        }

        if (Obj->HasField(TEXT("yaw")))
        {
            OutYawDeg = static_cast<float>(Obj->GetNumberField(TEXT("yaw")));
            return EDroneYawMode::Angle;
        }

        return EDroneYawMode::Auto;
    }

    FString YawModeToString(EDroneYawMode Mode)
    {
        switch (Mode)
        {
        case EDroneYawMode::Hold: return TEXT("hold");
        case EDroneYawMode::Angle: return TEXT("angle");
        case EDroneYawMode::Rate: return TEXT("rate");
        case EDroneYawMode::Auto:
        default: return TEXT("auto");
        }
    }

    FString MissionRoleToString(EDroneMissionRole Role)
    {
        switch (Role)
        {
        case EDroneMissionRole::Target: return TEXT("target");
        case EDroneMissionRole::Interceptor: return TEXT("interceptor");
        case EDroneMissionRole::Unknown:
        default: return TEXT("unknown");
        }
    }
    FString MakeOkWithCommand(const FString& Msg, const FString& CommandId)
    {
        return FString::Printf(TEXT("{\"status\":\"ok\",\"message\":\"%s\",\"command_id\":\"%s\"}"), *Msg, *CommandId);
    }

    FString MakeErrorWithCommand(const FString& Msg, const FString& CommandId)
    {
        return FString::Printf(TEXT("{\"status\":\"error\",\"message\":\"%s\",\"command_id\":\"%s\"}"), *Msg, *CommandId);
    }
}

/** @brief 构造错误响�?JSON */
FString UDroneCommandHandler::MakeError(const FString& Msg)
{
    return FString::Printf(TEXT("{\"status\":\"error\",\"message\":\"%s\"}"), *Msg);
}

/** @brief 构造成功响�?JSON */
FString UDroneCommandHandler::MakeOk(const FString& Msg)
{
    return FString::Printf(TEXT("{\"status\":\"ok\",\"message\":\"%s\"}"), *Msg);
}

/**
 * @brief 通过 Agent ID 查找 DroneApi 实例
 * @param DroneId 无人�?ID
 * @param World 当前 World
 * @return DroneApi 指针
 */
UDroneApi* UDroneCommandHandler::FindDroneApi(const FString& DroneId, UWorld* World)
{
    UAgentManager* Manager = UAgentManager::GetInstance();
    AActor* Agent = Manager->GetAgent(DroneId);
    ADronePawn* Drone = Cast<ADronePawn>(Agent);
    if (Drone && Drone->Api) return Drone->Api;
    return nullptr;
}

/**
 * @brief 处理 call_drone 命令
 * @param JsonObject 已解析的完整 JSON
 * @param World 当前 World
 * @return JSON 响应
 */
FString UDroneCommandHandler::HandleCallDrone(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World)
{
    const TSharedPtr<FJsonObject>* CmdObj = nullptr;
    if (!JsonObject->TryGetObjectField(TEXT("call_drone"), CmdObj)) return MakeError(TEXT("Missing call_drone field"));

    FString Function;
    (*CmdObj)->TryGetStringField(TEXT("function"), Function);

    FString DroneId;
    if (!(*CmdObj)->TryGetStringField(TEXT("id"), DroneId)) DroneId = TEXT("drone_0");

    UDroneApi* Api = FindDroneApi(DroneId, World);
    if (!Api) return MakeError(FString::Printf(TEXT("Drone '%s' not found"), *DroneId));

    const FString CommandId = UCommandExecutionManager::GetInstance()->StartCommand(DroneId, Function);

    if (Function == TEXT("move_to_position"))
    {
        float X = (*CmdObj)->GetNumberField(TEXT("x"));
        float Y = (*CmdObj)->GetNumberField(TEXT("y"));
        float Z = (*CmdObj)->GetNumberField(TEXT("z"));
        float Speed = 2.0f;
        if ((*CmdObj)->HasField(TEXT("speed"))) Speed = static_cast<float>((*CmdObj)->GetNumberField(TEXT("speed")));

        ECommandFrame InputFrame = ParseFrame(*CmdObj);
        FVector TargetUE = ConvertInputToUE(FVector(X, Y, Z), InputFrame);

        if (HasHeadingConfig(*CmdObj))
        {
            float YawDeg = 0.0f;
            EDroneYawMode YawMode = ParseYawMode(*CmdObj, YawDeg);
            EDroneDrivetrainMode DriveMode = ParseDrivetrain(*CmdObj);
            Api->SetHeadingControl(YawMode, DriveMode, YawDeg);
        }

        Api->MoveToPosition(TargetUE.X, TargetUE.Y, TargetUE.Z, Speed);
        const FString Msg = FString::Printf(TEXT("move_to_position(%.1f, %.1f, %.1f), speed=%.2f, frame=%s"),
            X, Y, Z, Speed, *FrameToString(InputFrame));
        UCommandExecutionManager::GetInstance()->CompleteCommand(CommandId, true, Msg);
        return MakeOkWithCommand(Msg, CommandId);
    }

    if (Function == TEXT("hover"))
    {
        Api->Hover();
        UCommandExecutionManager::GetInstance()->CompleteCommand(CommandId, true, TEXT("hovering"));
        return MakeOkWithCommand(TEXT("hovering"), CommandId);
    }

    if (Function == TEXT("takeoff"))
    {
        float Altitude = 3.0f;
        if ((*CmdObj)->HasField(TEXT("altitude"))) Altitude = static_cast<float>((*CmdObj)->GetNumberField(TEXT("altitude")));
        Api->Takeoff(Altitude);
        const FString Msg = FString::Printf(TEXT("takeoff to %.1f m"), Altitude);
        UCommandExecutionManager::GetInstance()->CompleteCommand(CommandId, true, Msg);
        return MakeOkWithCommand(Msg, CommandId);
    }

    if (Function == TEXT("land"))
    {
        Api->Land();
        UCommandExecutionManager::GetInstance()->CompleteCommand(CommandId, true, TEXT("landing"));
        return MakeOkWithCommand(TEXT("landing"), CommandId);
    }

    if (Function == TEXT("move_by_velocity"))
    {
        float Vx = static_cast<float>((*CmdObj)->GetNumberField(TEXT("vx")));
        float Vy = static_cast<float>((*CmdObj)->GetNumberField(TEXT("vy")));
        float Vz = static_cast<float>((*CmdObj)->GetNumberField(TEXT("vz")));

        ECommandFrame InputFrame = ParseFrame(*CmdObj);
        FVector VelUE = ConvertInputToUE(FVector(Vx, Vy, Vz), InputFrame);

        if (HasHeadingConfig(*CmdObj))
        {
            float YawDeg = 0.0f;
            EDroneYawMode YawMode = ParseYawMode(*CmdObj, YawDeg);
            EDroneDrivetrainMode DriveMode = ParseDrivetrain(*CmdObj);
            Api->SetHeadingControl(YawMode, DriveMode, YawDeg);
        }

        Api->MoveByVelocity(VelUE.X, VelUE.Y, VelUE.Z);
        const FString Msg = FString::Printf(TEXT("move_by_velocity(%.1f, %.1f, %.1f), frame=%s"),
            Vx, Vy, Vz, *FrameToString(InputFrame));
        UCommandExecutionManager::GetInstance()->CompleteCommand(CommandId, true, Msg);
        return MakeOkWithCommand(Msg, CommandId);
    }

    if (Function == TEXT("set_pid_position"))
    {
        float Kp = static_cast<float>((*CmdObj)->GetNumberField(TEXT("kp")));
        float Kd = (*CmdObj)->HasField(TEXT("kd")) ? static_cast<float>((*CmdObj)->GetNumberField(TEXT("kd"))) : 0.0f;
        Api->SetPositionControllerGains(Kp, Kd);
        const FString Msg = FString::Printf(TEXT("position PD: Kp=%.2f Kd=%.2f"), Kp, Kd);
        UCommandExecutionManager::GetInstance()->CompleteCommand(CommandId, true, Msg);
        return MakeOkWithCommand(Msg, CommandId);
    }

    if (Function == TEXT("set_pid_velocity"))
    {
        float Kp = static_cast<float>((*CmdObj)->GetNumberField(TEXT("kp")));
        float Ki = (*CmdObj)->HasField(TEXT("ki")) ? static_cast<float>((*CmdObj)->GetNumberField(TEXT("ki"))) : 0.0f;
        float Kd = (*CmdObj)->HasField(TEXT("kd")) ? static_cast<float>((*CmdObj)->GetNumberField(TEXT("kd"))) : 0.0f;
        Api->SetVelocityControllerGains(Kp, Ki, Kd);
        const FString Msg = FString::Printf(TEXT("velocity PID: Kp=%.2f Ki=%.2f Kd=%.2f"), Kp, Ki, Kd);
        UCommandExecutionManager::GetInstance()->CompleteCommand(CommandId, true, Msg);
        return MakeOkWithCommand(Msg, CommandId);
    }

    if (Function == TEXT("set_pid_attitude"))
    {
        float Kp = static_cast<float>((*CmdObj)->GetNumberField(TEXT("kp")));
        float Kd = (*CmdObj)->HasField(TEXT("kd")) ? static_cast<float>((*CmdObj)->GetNumberField(TEXT("kd"))) : 0.0f;
        Api->SetAttitudeControllerGains(Kp, Kd);
        const FString Msg = FString::Printf(TEXT("attitude PD: Kp=%.2f Kd=%.2f"), Kp, Kd);
        UCommandExecutionManager::GetInstance()->CompleteCommand(CommandId, true, Msg);
        return MakeOkWithCommand(Msg, CommandId);
    }

    if (Function == TEXT("set_pid_angle_rate"))
    {
        float Kp = static_cast<float>((*CmdObj)->GetNumberField(TEXT("kp")));
        Api->SetAngleRateControllerGains(Kp);
        const FString Msg = FString::Printf(TEXT("angle_rate P: Kp=%.2f"), Kp);
        UCommandExecutionManager::GetInstance()->CompleteCommand(CommandId, true, Msg);
        return MakeOkWithCommand(Msg, CommandId);
    }

    if (Function == TEXT("set_camera_angles"))
    {
        float Pitch = static_cast<float>((*CmdObj)->GetNumberField(TEXT("pitch")));
        float Yaw = static_cast<float>((*CmdObj)->GetNumberField(TEXT("yaw")));
        ADronePawn* Drone = Cast<ADronePawn>(UAgentManager::GetInstance()->GetAgent(DroneId));
        if (Drone) Drone->SetCameraAngles(Pitch, Yaw);
        const FString Msg = FString::Printf(TEXT("set_camera_angles pitch=%.1f yaw=%.1f"), Pitch, Yaw);
        UCommandExecutionManager::GetInstance()->CompleteCommand(CommandId, true, Msg);
        return MakeOkWithCommand(Msg, CommandId);
    }

    if (Function == TEXT("set_target_attitude"))
    {
        float Roll = (*CmdObj)->HasField(TEXT("roll")) ? static_cast<float>((*CmdObj)->GetNumberField(TEXT("roll"))) : 0.0f;
        float Pitch = (*CmdObj)->HasField(TEXT("pitch")) ? static_cast<float>((*CmdObj)->GetNumberField(TEXT("pitch"))) : 0.0f;
        float Yaw = (*CmdObj)->HasField(TEXT("yaw")) ? static_cast<float>((*CmdObj)->GetNumberField(TEXT("yaw"))) : 0.0f;
        float Thrust = (*CmdObj)->HasField(TEXT("thrust")) ? static_cast<float>((*CmdObj)->GetNumberField(TEXT("thrust"))) : 9.81f;
        ECommandFrame AttitudeFrame = ECommandFrame::NED;
        if ((*CmdObj)->HasField(TEXT("frame")))
        {
            AttitudeFrame = ParseFrame(*CmdObj);
        }
        const FRotator InputRot(Pitch, Yaw, Roll);
        const FRotator UERot = ConvertFrameToUERotator(InputRot, AttitudeFrame);
        Roll = UERot.Roll;
        Pitch = UERot.Pitch;
        Yaw = UERot.Yaw;
        Api->SetTargetAttitude(Roll, Pitch, Yaw, Thrust);
        const FString Msg = FString::Printf(TEXT("set_target_attitude roll=%.1f pitch=%.1f yaw=%.1f thrust=%.2f"), Roll, Pitch, Yaw, Thrust);
        UCommandExecutionManager::GetInstance()->CompleteCommand(CommandId, true, Msg);
        return MakeOkWithCommand(Msg, CommandId);
    }

    if (Function == TEXT("set_motor_speeds"))
    {
        float M0 = (*CmdObj)->HasField(TEXT("m0")) ? static_cast<float>((*CmdObj)->GetNumberField(TEXT("m0"))) : 0.0f;
        float M1 = (*CmdObj)->HasField(TEXT("m1")) ? static_cast<float>((*CmdObj)->GetNumberField(TEXT("m1"))) : 0.0f;
        float M2 = (*CmdObj)->HasField(TEXT("m2")) ? static_cast<float>((*CmdObj)->GetNumberField(TEXT("m2"))) : 0.0f;
        float M3 = (*CmdObj)->HasField(TEXT("m3")) ? static_cast<float>((*CmdObj)->GetNumberField(TEXT("m3"))) : 0.0f;
        Api->SetMotorSpeeds(M0, M1, M2, M3);
        const FString Msg = FString::Printf(TEXT("set_motor_speeds [%.1f, %.1f, %.1f, %.1f]"), M0, M1, M2, M3);
        UCommandExecutionManager::GetInstance()->CompleteCommand(CommandId, true, Msg);
        return MakeOkWithCommand(Msg, CommandId);
    }

    if (Function == TEXT("reset"))
    {
        Api->Reset();
        UCommandExecutionManager::GetInstance()->CompleteCommand(CommandId, true, TEXT("drone reset"));
        return MakeOkWithCommand(TEXT("drone reset"), CommandId);
    }

    const FString Err = FString::Printf(TEXT("Unknown function: %s"), *Function);
    UCommandExecutionManager::GetInstance()->CompleteCommand(CommandId, false, Err);
    return MakeErrorWithCommand(Err, CommandId);
}

/**
 * @brief 处理 get_drone_state 命令
 * @param JsonObject 已解析的完整 JSON
 * @param World 当前 World
 * @return JSON 响应
 */
FString UDroneCommandHandler::HandleGetDroneState(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World)
{
    const TSharedPtr<FJsonObject>* StateObj = nullptr;
    if (!JsonObject->TryGetObjectField(TEXT("get_drone_state"), StateObj)) return MakeError(TEXT("Missing get_drone_state field"));

    FString DroneId;
    if (!(*StateObj)->TryGetStringField(TEXT("id"), DroneId)) DroneId = TEXT("drone_0");

    UAgentManager* Manager = UAgentManager::GetInstance();
    ADronePawn* Drone = Cast<ADronePawn>(Manager->GetAgent(DroneId));
    if (!Drone) return MakeError(FString::Printf(TEXT("Drone '%s' not found"), *DroneId));

    ECommandFrame OutputFrame = ParseFrame(*StateObj);

    FVector PosUE = Drone->GetCurrentPosition();
    FVector VelUE = Drone->GetCurrentVelocity();
    FVector Pos = ConvertUEToOutput(PosUE, OutputFrame);
    FVector Vel = ConvertUEToOutput(VelUE, OutputFrame);

    FRotator Rot = Drone->CurrentState.GetRotator();
    Rot = ConvertUEToFrameRotator(Rot, OutputFrame);

    EDroneYawMode YawMode = EDroneYawMode::Rate;
    EDroneDrivetrainMode DriveMode = EDroneDrivetrainMode::MaxDegreeOfFreedom;
    if (Drone->MovementComp)
    {
        YawMode = Drone->MovementComp->GetYawMode();
        DriveMode = Drone->MovementComp->GetDrivetrainMode();
    }

    TArray<float> Motors;
    if (Drone->Api) Motors = Drone->Api->GetMotorSpeeds();

    FString ModeName;
    switch (Drone->ControlMode)
    {
    case EDroneControlMode::Idle: ModeName = TEXT("idle"); break;
    case EDroneControlMode::Position: ModeName = TEXT("position"); break;
    case EDroneControlMode::Velocity: ModeName = TEXT("velocity"); break;
    case EDroneControlMode::AttitudeThrust: ModeName = TEXT("attitude"); break;
    case EDroneControlMode::MotorSpeed: ModeName = TEXT("motor_speed"); break;
    case EDroneControlMode::TorqueThrust: ModeName = TEXT("torque_thrust"); break;
    default: ModeName = TEXT("unknown"); break;
    }

    return FString::Printf(
        TEXT("{\"status\":\"ok\",\"id\":\"%s\",")
        TEXT("\"frame\":\"%s\",\"orientation_frame\":\"%s\",")
        TEXT("\"position\":[%.4f,%.4f,%.4f],")
        TEXT("\"velocity\":[%.4f,%.4f,%.4f],")
        TEXT("\"orientation\":{\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f},")
        TEXT("\"motor_speeds\":[%.1f,%.1f,%.1f,%.1f],")
        TEXT("\"yaw_mode\":\"%s\",\"drivetrain\":\"%s\",")
        TEXT("\"role\":\"%s\",")
        TEXT("\"camera_pitch\":%.2f,\"camera_yaw\":%.2f,")
        TEXT("\"control_mode\":\"%s\"}"),
        *DroneId,
        *FrameToString(OutputFrame),
        *FrameToString(OutputFrame),
        Pos.X, Pos.Y, Pos.Z,
        Vel.X, Vel.Y, Vel.Z,
        Rot.Roll, Rot.Pitch, Rot.Yaw,
        Motors.Num() >= 4 ? Motors[0] : 0.0f,
        Motors.Num() >= 4 ? Motors[1] : 0.0f,
        Motors.Num() >= 4 ? Motors[2] : 0.0f,
        Motors.Num() >= 4 ? Motors[3] : 0.0f,
        *YawModeToString(YawMode), *DrivetrainToString(DriveMode),
        *MissionRoleToString(Drone->MissionRole),
        Drone->GetCameraCurrentPitch(), Drone->GetCameraCurrentYaw(),
        *ModeName);
}


























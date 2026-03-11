#include "SensorManager.h"

#include "GraduationProject/Core/Manager/AgentManager.h"
#include "GraduationProject/Drone/DronePawn.h"

namespace
{
    FVector ConvertUEToFrame(const FVector& ValueUE, const FString& Frame)
    {
        FString Lower = Frame;
        Lower.ToLowerInline();
        if (Lower == TEXT("ned"))
        {
            return FVector(ValueUE.X, ValueUE.Y, -ValueUE.Z);
        }

        return ValueUE;
    }

    FRotator ConvertUEToFrameRotator(const FRotator& RotUE, const FString& Frame)
    {
        FString Lower = Frame;
        Lower.ToLowerInline();
        if (Lower == TEXT("ned"))
        {
            return FRotator(-RotUE.Pitch, RotUE.Yaw, -RotUE.Roll);
        }

        return RotUE;
    }
    double EstimatePressurePa(double AltitudeMeters)
    {
        const double H = FMath::Max(0.0, static_cast<double>(AltitudeMeters));
        return 101325.0 * FMath::Pow(1.0 - 2.25577e-5 * H, 5.25588);
    }
}

USensorManager* USensorManager::Instance = nullptr;

USensorManager* USensorManager::GetInstance()
{
    if (!Instance)
    {
        Instance = NewObject<USensorManager>();
        Instance->AddToRoot();
    }
    return Instance;
}

void USensorManager::Cleanup()
{
    if (Instance)
    {
        Instance->DroneCache.Empty();
        Instance->RemoveFromRoot();
        Instance = nullptr;
    }
}

FString USensorManager::BuildDroneSensorJson(const FString& DroneId, UWorld* World, const FString& Frame)
{
    if (!World)
    {
        return TEXT("{\"status\":\"error\",\"message\":\"No World\"}");
    }

    UAgentManager* AgentManager = UAgentManager::GetInstance();
    ADronePawn* Drone = Cast<ADronePawn>(AgentManager->GetAgent(DroneId));
    if (!Drone)
    {
        return FString::Printf(TEXT("{\"status\":\"error\",\"message\":\"Drone '%s' not found\"}"), *DroneId);
    }

    const FVector PosUE = Drone->GetCurrentPosition();
    const FVector VelUE = Drone->GetCurrentVelocity();
    const double Now = FPlatformTime::Seconds();

    FVector AccUE = FVector::ZeroVector;
    FSensorKinematicCache& Cache = DroneCache.FindOrAdd(DroneId);
    if (Cache.bValid)
    {
        const double Dt = FMath::Max(1e-4, Now - Cache.LastTimeSec);
        AccUE = (VelUE - Cache.LastVelocity) / Dt;
    }

    Cache.LastVelocity = VelUE;
    Cache.LastTimeSec = Now;
    Cache.bValid = true;

    const FVector Pos = ConvertUEToFrame(PosUE, Frame);
    const FVector Vel = ConvertUEToFrame(VelUE, Frame);
    const FVector Acc = ConvertUEToFrame(AccUE, Frame);

    const FRotator RotUE = Drone->CurrentState.GetRotator();
    const FRotator Rot = ConvertUEToFrameRotator(RotUE, Frame);

    const double HomeLat = 31.3020;
    const double HomeLon = 120.5853;
    const double Lat = HomeLat + static_cast<double>(PosUE.Y) / 111111.0;
    const double Lon = HomeLon + static_cast<double>(PosUE.X) / (111111.0 * FMath::Max(0.1, FMath::Cos(FMath::DegreesToRadians(HomeLat))));
    const double Alt = FMath::Max(0.0, static_cast<double>(PosUE.Z));

    const double PressurePa = EstimatePressurePa(Alt);

    return FString::Printf(
        TEXT("{\"status\":\"ok\",\"id\":\"%s\",\"frame\":\"%s\",\"timestamp\":%.6f,\"imu\":{\"linear_accel\":[%.4f,%.4f,%.4f],\"angular_velocity\":[0.0,0.0,0.0],\"orientation\":{\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f}},\"gps\":{\"lat\":%.8f,\"lon\":%.8f,\"alt\":%.3f},\"barometer\":{\"altitude_m\":%.3f,\"pressure_pa\":%.2f},\"kinematics\":{\"position\":[%.4f,%.4f,%.4f],\"velocity\":[%.4f,%.4f,%.4f]}}"),
        *DroneId,
        *Frame,
        Now,
        Acc.X, Acc.Y, Acc.Z,
        Rot.Roll, Rot.Pitch, Rot.Yaw,
        Lat, Lon, Alt,
        Alt, PressurePa,
        Pos.X, Pos.Y, Pos.Z,
        Vel.X, Vel.Y, Vel.Z);
}

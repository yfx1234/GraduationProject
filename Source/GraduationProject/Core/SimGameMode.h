
#pragma once
#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "../Drone/DronePawn.h"
#include "SimGameMode.generated.h"
USTRUCT(BlueprintType)
struct FDroneSpawnConfig
{
    GENERATED_BODY()
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Drone")
    FString DroneId = TEXT("drone_0");
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Drone")
    FVector SpawnLocation = FVector(0.0f, 0.0f, 0.5f);
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Drone")
    bool bAutoTakeoff = false;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Drone")
    float TakeoffAltitude = 3.0f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Drone")
    EDroneMissionRole MissionRole = EDroneMissionRole::Unknown;
};
UCLASS()
class GRADUATIONPROJECT_API ASimGameMode : public AGameModeBase
{
    GENERATED_BODY()
public:
    ASimGameMode();
    virtual void BeginPlay() override;
    UFUNCTION(BlueprintCallable, Category = "Simulation")
    void PauseSimulation();
    UFUNCTION(BlueprintCallable, Category = "Simulation")
    void ResumeSimulation();
    UFUNCTION(BlueprintCallable, Category = "Simulation")
    void ResetSimulation();
protected:
    void SpawnDefaultDrone();
    void SpawnConfiguredDrones();
    ADronePawn* SpawnDrone(const FString& DroneId, const FVector& SpawnLocationMeters, EDroneMissionRole InMissionRole = EDroneMissionRole::Unknown);
    void CaptureInitialAgentStates();
    void DelayedTakeoff();
public:
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Drone")
    bool bAutoSpawnDrone = false;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Drone")
    bool bEnableBootstrapSpawn = false;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Drone")
    TSubclassOf<APawn> DefaultDroneClass;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Drone")
    FVector DroneSpawnLocation = FVector(0.0f, 0.0f, 0.5f);
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Drone")
    FString DefaultDroneId = TEXT("drone_0");
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Drone")
    bool bAutoTakeoff = true;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Drone")
    float TakeoffAltitude = 3.0f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Drone")
    EDroneMissionRole DefaultDroneRole = EDroneMissionRole::Target;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Drone")
    bool bUseDroneSpawnList = false;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Drone")
    TArray<FDroneSpawnConfig> DroneSpawnList;
private:
    UPROPERTY()
    APawn* SpawnedDrone = nullptr;
    UPROPERTY()
    TMap<FString, FTransform> InitialAgentTransforms;
};

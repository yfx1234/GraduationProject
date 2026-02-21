#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "TurretCommandHandler.generated.h"

class ATurretPawn;

/**
 * 转台 TCP 命令处理器
 * 解析 call_turret / get_turret_state JSON 命令
 */
UCLASS()
class GRADUATIONPROJECT_API UTurretCommandHandler : public UObject
{
    GENERATED_BODY()

public:
    /** 处理 call_turret 命令 */
    FString HandleCallTurret(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World);

    /** 处理 get_turret_state 命令 */
    FString HandleGetTurretState(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World);

private:
    ATurretPawn* FindTurretPawn(const FString& TurretId, UWorld* World);
    FString MakeError(const FString& Msg);
    FString MakeOk(const FString& Msg = TEXT("ok"));
};


#pragma once
#include "CoreMinimal.h"
#include "Dom/JsonObject.h"
#include "FCommandHandle.h"
#include "UObject/NoExportTypes.h"
#include "CommandRouter.generated.h"
UCLASS()
class GRADUATIONPROJECT_API UCommandRouter : public UObject
{
    GENERATED_BODY()
public:
    FString HandleCommand(const FString& JsonString, UWorld* World);
private:
    FString HandlePing();
    FString HandleSimPause(UWorld* World);
    FString HandleSimResume(UWorld* World);
    FString HandleSimReset(UWorld* World);
    FString HandleSimGetTime(UWorld* World);
    FString HandleSimSetTimeScale(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World);
    FString HandleSimStep(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World);
    FString HandleGetAgentList();
    FString HandleGetCommandStatus(const TSharedPtr<FJsonObject>& JsonObject);
    FString HandleCancelCommand(const TSharedPtr<FJsonObject>& JsonObject);
    FString HandleGetSensorData(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World);
    FString HandleRecorderStart(const TSharedPtr<FJsonObject>& JsonObject);
    FString HandleRecorderStop();
    FString HandleRecorderStatus();
    FString HandleRecorderRecordState(UWorld* World);
    FString MakeErrorResponse(const FString& Error);
    FString MakeOkResponse(const FString& Message = TEXT("ok"));
    TUniquePtr<FCommandHandle> CommandHandle;
};

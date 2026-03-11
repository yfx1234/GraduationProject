#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "CommandExecutionManager.generated.h"

USTRUCT(BlueprintType)
struct FCommandExecutionRecord
{
    GENERATED_BODY()

    UPROPERTY(BlueprintReadOnly)
    FString CommandId;

    UPROPERTY(BlueprintReadOnly)
    FString AgentId;

    UPROPERTY(BlueprintReadOnly)
    FString FunctionName;

    UPROPERTY(BlueprintReadOnly)
    FString Status = TEXT("queued");

    UPROPERTY(BlueprintReadOnly)
    FString Message;

    UPROPERTY(BlueprintReadOnly)
    double StartTimeSec = 0.0;

    UPROPERTY(BlueprintReadOnly)
    double EndTimeSec = 0.0;
};

UCLASS()
class GRADUATIONPROJECT_API UCommandExecutionManager : public UObject
{
    GENERATED_BODY()

public:
    static UCommandExecutionManager* GetInstance();
    static void Cleanup();

    FString StartCommand(const FString& AgentId, const FString& FunctionName);
    void CompleteCommand(const FString& CommandId, bool bSuccess, const FString& Message);
    bool GetCommand(const FString& CommandId, FCommandExecutionRecord& OutRecord) const;
    bool CancelCommand(const FString& CommandId, const FString& Reason = TEXT("canceled"));
    bool GetCommandWithTimeout(const FString& CommandId, double TimeoutSec, FCommandExecutionRecord& OutRecord);

private:
    static UCommandExecutionManager* Instance;

    UPROPERTY()
    TMap<FString, FCommandExecutionRecord> Records;

    int64 Counter = 0;
};

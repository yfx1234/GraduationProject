#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "SimulationRecorder.generated.h"

UCLASS()
class GRADUATIONPROJECT_API USimulationRecorder : public UObject
{
    GENERATED_BODY()

public:
    static USimulationRecorder* GetInstance();
    static void Cleanup();

    bool Start(const FString& InPath = TEXT(""));
    void Stop();

    bool IsRecording() const { return bRecording; }
    FString GetPath() const { return RecordPath; }
    int64 GetRecordCount() const { return RecordCount; }

    void RecordJsonLine(const FString& Type, const FString& JsonPayload);

private:
    static USimulationRecorder* Instance;

    bool bRecording = false;
    FString RecordPath;
    int64 RecordCount = 0;
};

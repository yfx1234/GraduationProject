#include "SimulationRecorder.h"

#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"

USimulationRecorder* USimulationRecorder::Instance = nullptr;

USimulationRecorder* USimulationRecorder::GetInstance()
{
    if (!Instance)
    {
        Instance = NewObject<USimulationRecorder>();
        Instance->AddToRoot();
    }
    return Instance;
}

void USimulationRecorder::Cleanup()
{
    if (Instance)
    {
        Instance->Stop();
        Instance->RemoveFromRoot();
        Instance = nullptr;
    }
}

bool USimulationRecorder::Start(const FString& InPath)
{
    FString FinalPath = InPath;
    if (FinalPath.IsEmpty())
    {
        const FString Dir = FPaths::ProjectSavedDir() / TEXT("SimRecords");
        IFileManager::Get().MakeDirectory(*Dir, true);
        FinalPath = Dir / FString::Printf(TEXT("record_%s.jsonl"), *FDateTime::Now().ToString(TEXT("%Y%m%d_%H%M%S")));
    }

    const FString Header = FString::Printf(TEXT("{\"type\":\"meta\",\"started_at\":\"%s\"}\n"), *FDateTime::UtcNow().ToIso8601());
    if (!FFileHelper::SaveStringToFile(Header, *FinalPath, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM, &IFileManager::Get(), EFileWrite::FILEWRITE_None))
    {
        return false;
    }

    bRecording = true;
    RecordPath = FinalPath;
    RecordCount = 0;
    return true;
}

void USimulationRecorder::Stop()
{
    bRecording = false;
}

void USimulationRecorder::RecordJsonLine(const FString& Type, const FString& JsonPayload)
{
    if (!bRecording || RecordPath.IsEmpty())
    {
        return;
    }

    const FString Line = FString::Printf(
        TEXT("{\"type\":\"%s\",\"time\":\"%s\",\"payload\":%s}\n"),
        *Type,
        *FDateTime::UtcNow().ToIso8601(),
        *JsonPayload);

    FFileHelper::SaveStringToFile(Line, *RecordPath, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM, &IFileManager::Get(), EFileWrite::FILEWRITE_Append);
    ++RecordCount;
}


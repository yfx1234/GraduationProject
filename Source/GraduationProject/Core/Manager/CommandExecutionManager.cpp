#include "CommandExecutionManager.h"
#include "GraduationProject/Core/Simulation/SimulationRecorder.h"

UCommandExecutionManager* UCommandExecutionManager::Instance = nullptr;

UCommandExecutionManager* UCommandExecutionManager::GetInstance()
{
    if (!Instance)
    {
        Instance = NewObject<UCommandExecutionManager>();
        Instance->AddToRoot();
    }
    return Instance;
}

void UCommandExecutionManager::Cleanup()
{
    if (Instance)
    {
        Instance->Records.Empty();
        Instance->RemoveFromRoot();
        Instance = nullptr;
    }
}

FString UCommandExecutionManager::StartCommand(const FString& AgentId, const FString& FunctionName)
{
    ++Counter;
    const FString CommandId = FString::Printf(TEXT("cmd_%lld"), Counter);

    FCommandExecutionRecord Record;
    Record.CommandId = CommandId;
    Record.AgentId = AgentId;
    Record.FunctionName = FunctionName;
    Record.Status = TEXT("running");
    Record.StartTimeSec = FPlatformTime::Seconds();

    Records.Add(CommandId, Record);

    USimulationRecorder::GetInstance()->RecordJsonLine(TEXT("command_start"),
        FString::Printf(TEXT("{\"command_id\":\"%s\",\"agent_id\":\"%s\",\"function\":\"%s\"}"), *CommandId, *AgentId, *FunctionName));

    return CommandId;
}

void UCommandExecutionManager::CompleteCommand(const FString& CommandId, bool bSuccess, const FString& Message)
{
    if (FCommandExecutionRecord* Record = Records.Find(CommandId))
    {
        if (Record->Status == TEXT("canceled"))
        {
            return;
        }

        Record->Status = bSuccess ? TEXT("completed") : TEXT("failed");
        Record->Message = Message;
        Record->EndTimeSec = FPlatformTime::Seconds();

        USimulationRecorder::GetInstance()->RecordJsonLine(TEXT("command_done"),
            FString::Printf(TEXT("{\"command_id\":\"%s\",\"state\":\"%s\",\"message\":\"%s\",\"duration\":%.6f}"),
                *Record->CommandId,
                *Record->Status,
                *Record->Message.ReplaceCharWithEscapedChar(),
                (Record->EndTimeSec > Record->StartTimeSec) ? (Record->EndTimeSec - Record->StartTimeSec) : 0.0));
    }
}

bool UCommandExecutionManager::CancelCommand(const FString& CommandId, const FString& Reason)
{
    if (FCommandExecutionRecord* Record = Records.Find(CommandId))
    {
        if (Record->Status == TEXT("completed") || Record->Status == TEXT("failed"))
        {
            return false;
        }

        Record->Status = TEXT("canceled");
        Record->Message = Reason;
        Record->EndTimeSec = FPlatformTime::Seconds();

        USimulationRecorder::GetInstance()->RecordJsonLine(TEXT("command_cancel"),
            FString::Printf(TEXT("{\"command_id\":\"%s\",\"reason\":\"%s\"}"),
                *Record->CommandId,
                *Reason.ReplaceCharWithEscapedChar()));
        return true;
    }

    return false;
}

bool UCommandExecutionManager::GetCommand(const FString& CommandId, FCommandExecutionRecord& OutRecord) const
{
    if (const FCommandExecutionRecord* Found = Records.Find(CommandId))
    {
        OutRecord = *Found;
        return true;
    }
    return false;
}

bool UCommandExecutionManager::GetCommandWithTimeout(const FString& CommandId, double TimeoutSec, FCommandExecutionRecord& OutRecord)
{
    FCommandExecutionRecord* Record = Records.Find(CommandId);
    if (!Record)
    {
        return false;
    }

    if (TimeoutSec > 0.0 && Record->Status == TEXT("running"))
    {
        const double Now = FPlatformTime::Seconds();
        if ((Now - Record->StartTimeSec) > TimeoutSec)
        {
            Record->Status = TEXT("failed");
            Record->Message = FString::Printf(TEXT("timeout(%.2fs)"), TimeoutSec);
            Record->EndTimeSec = Now;
        }
    }

    OutRecord = *Record;
    return true;
}



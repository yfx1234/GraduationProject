#include "CommandExecutionManager.h"
#include "GraduationProject/Core/Simulation/SimulationRecorder.h"

UCommandExecutionManager* UCommandExecutionManager::Instance = nullptr;

/**
 * @brief 获取命令执行管理器单例
 * 若单例不存在，则创建对象并加入 Root，避免被 GC 回收。
 */
UCommandExecutionManager* UCommandExecutionManager::GetInstance()
{
    if (!Instance)
    {
        Instance = NewObject<UCommandExecutionManager>();
        Instance->AddToRoot();
    }
    return Instance;
}

/**
 * @brief 释放单例并清空所有命令记录
 * 通常在游戏退出、网络服务关闭或测试清理阶段调用。
 */
void UCommandExecutionManager::Cleanup()
{
    if (Instance)
    {
        Instance->Records.Empty();
        Instance->RemoveFromRoot();
        Instance = nullptr;
    }
}

/**
 * @brief 启动一条新的异步命令记录
 * @param AgentId 发起命令的智能体 ID
 * @param FunctionName 被调用函数名
 * @return 新生成的命令 ID
 * 命令创建后状态立即置为 `running`，并写入一条 `command_start` 录制事件。
 */
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

/**
 * @brief 将命令标记为完成或失败
 * @param CommandId 命令 ID
 * @param bSuccess 是否执行成功
 * @param Message 结果说明
 * 若命令此前已被取消，则保持取消状态，不再覆盖其结束信息。
 */
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

/**
 * @brief 取消尚未结束的命令
 * @param CommandId 命令 ID
 * @param Reason 取消原因
 * @return 成功取消时返回 `true`
 */
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

/**
 * @brief 查询命令记录
 * @param CommandId 命令 ID
 * @param OutRecord 输出记录
 * @return 找到命令时返回 `true`
 */
bool UCommandExecutionManager::GetCommand(const FString& CommandId, FCommandExecutionRecord& OutRecord) const
{
    if (const FCommandExecutionRecord* Found = Records.Find(CommandId))
    {
        OutRecord = *Found;
        return true;
    }
    return false;
}

/**
 * @brief 查询命令状态，并在需要时自动标记超时
 * @param CommandId 命令 ID
 * @param TimeoutSec 超时时间（秒）
 * @param OutRecord 输出记录
 * @return 找到命令时返回 `true`
 * 若命令仍处于 `running` 且已超过超时阈值，则会直接改写为 `failed`。
 */
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
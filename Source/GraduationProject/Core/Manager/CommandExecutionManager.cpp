// 解释：引入当前实现文件对应的头文件 `CommandExecutionManager.h`，使实现部分能够看到类和函数声明。
#include "CommandExecutionManager.h"
// 解释：引入 `SimulationRecorder.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "GraduationProject/Core/Simulation/SimulationRecorder.h"

// 解释：这一行把右侧表达式的结果写入 `UCommandExecutionManager* UCommandExecutionManager::Instance`，完成 ucommand执行管理器ucommand执行管理器instance 的更新。
UCommandExecutionManager* UCommandExecutionManager::Instance = nullptr;

/**
 * @brief 获取命令执行管理器单例
 * 若单例不存在，则创建对象并加入 Root，避免被 GC 回收
 */
// 解释：这一行定义函数 `GetInstance`，开始实现getinstance的具体逻辑。
UCommandExecutionManager* UCommandExecutionManager::GetInstance()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!Instance)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `Instance`，完成 instance 的更新。
        Instance = NewObject<UCommandExecutionManager>();
        // 解释：调用 `AddToRoot` 执行当前步骤需要的功能逻辑。
        Instance->AddToRoot();
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return Instance;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 释放单例并清空所有命令记录 */
// 解释：这一行定义函数 `Cleanup`，开始实现cleanup的具体逻辑。
void UCommandExecutionManager::Cleanup()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (Instance)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `Empty` 执行当前步骤需要的功能逻辑。
        Instance->Records.Empty();
        // 解释：调用 `RemoveFromRoot` 执行当前步骤需要的功能逻辑。
        Instance->RemoveFromRoot();
        // 解释：这一行把右侧表达式的结果写入 `Instance`，完成 instance 的更新。
        Instance = nullptr;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 启动一条新的异步命令记录
 * @param AgentId 发起命令的智能体 ID
 * @param FunctionName 被调用函数名
 * @return 新生成的命令 ID
 */
// 解释：这一行定义函数 `StartCommand`，开始实现start命令的具体逻辑。
FString UCommandExecutionManager::StartCommand(const FString& AgentId, const FString& FunctionName)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    ++Counter;
    // 解释：这一行把右侧表达式的结果写入 `const FString CommandId`，完成 constfstring命令id 的更新。
    const FString CommandId = FString::Printf(TEXT("cmd_%lld"), Counter);
    // 解释：这一行声明成员或局部变量 `Record`，用于保存记录。
    FCommandExecutionRecord Record;
    // 解释：这一行把右侧表达式的结果写入 `Record.CommandId`，完成 命令id 的更新。
    Record.CommandId = CommandId;
    // 解释：这一行把右侧表达式的结果写入 `Record.AgentId`，完成 agentid 的更新。
    Record.AgentId = AgentId;
    // 解释：这一行把右侧表达式的结果写入 `Record.FunctionName`，完成 functionname 的更新。
    Record.FunctionName = FunctionName;
    // 解释：这一行把右侧表达式的结果写入 `Record.Status`，完成 status 的更新。
    Record.Status = TEXT("running");
    // 解释：这一行把右侧表达式的结果写入 `Record.StartTimeSec`，完成 starttimesec 的更新。
    Record.StartTimeSec = FPlatformTime::Seconds();
    // 解释：调用 `Add` 执行当前步骤需要的功能逻辑。
    Records.Add(CommandId, Record);
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    USimulationRecorder::GetInstance()->RecordJsonLine(TEXT("command_start"),
        // 解释：调用 `Printf` 执行当前步骤需要的功能逻辑。
        FString::Printf(TEXT("{\"command_id\":\"%s\",\"agent_id\":\"%s\",\"function\":\"%s\"}"), *CommandId, *AgentId, *FunctionName));
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return CommandId;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 将命令标记为完成或失败
 * @param CommandId 命令 ID
 * @param bSuccess 是否执行成功
 * @param Message 结果说明
 */
// 解释：这一行定义函数 `CompleteCommand`，开始实现complete命令的具体逻辑。
void UCommandExecutionManager::CompleteCommand(const FString& CommandId, bool bSuccess, const FString& Message)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (FCommandExecutionRecord* Record = Records.Find(CommandId))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (Record->Status == TEXT("canceled")) return;
        // 解释：调用 `TEXT` 执行当前步骤需要的功能逻辑。
        Record->Status = bSuccess ? TEXT("completed") : TEXT("failed");
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Record->Message = Message;
        // 解释：调用 `Seconds` 执行当前步骤需要的功能逻辑。
        Record->EndTimeSec = FPlatformTime::Seconds();
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        USimulationRecorder::GetInstance()->RecordJsonLine(TEXT("command_done"),
            // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
            FString::Printf(TEXT("{\"command_id\":\"%s\",\"state\":\"%s\",\"message\":\"%s\",\"duration\":%.6f}"),
                *Record->CommandId,
                *Record->Status,
                *Record->Message.ReplaceCharWithEscapedChar(),
                // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                (Record->EndTimeSec > Record->StartTimeSec) ? (Record->EndTimeSec - Record->StartTimeSec) : 0.0));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 取消尚未结束的命令
 * @param CommandId 命令 ID
 * @param Reason 取消原因
 * @return 成功取消时返回 true
 */
// 解释：这一行定义函数 `CancelCommand`，开始实现cancel命令的具体逻辑。
bool UCommandExecutionManager::CancelCommand(const FString& CommandId, const FString& Reason)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (FCommandExecutionRecord* Record = Records.Find(CommandId))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (Record->Status == TEXT("completed") || Record->Status == TEXT("failed")) return false;
        // 解释：调用 `TEXT` 执行当前步骤需要的功能逻辑。
        Record->Status = TEXT("canceled");
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Record->Message = Reason;
        // 解释：调用 `Seconds` 执行当前步骤需要的功能逻辑。
        Record->EndTimeSec = FPlatformTime::Seconds();
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        USimulationRecorder::GetInstance()->RecordJsonLine(TEXT("command_cancel"),
            // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
            FString::Printf(TEXT("{\"command_id\":\"%s\",\"reason\":\"%s\"}"),
                *Record->CommandId,
                *Reason.ReplaceCharWithEscapedChar()));
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return true;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return false;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 查询命令记录
 * @param CommandId 命令 ID
 * @param OutRecord 输出记录
 * @return 找到命令时返回 true
 */
// 解释：这一行定义函数 `GetCommand`，开始实现get命令的具体逻辑。
bool UCommandExecutionManager::GetCommand(const FString& CommandId, FCommandExecutionRecord& OutRecord) const
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (const FCommandExecutionRecord* Found = Records.Find(CommandId))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `OutRecord`，完成 out记录 的更新。
        OutRecord = *Found;
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return true;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return false;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 查询命令状态，并在需要时自动标记超时
 * @param CommandId 命令 ID
 * @param TimeoutSec 超时时间（秒）
 * @param OutRecord 输出记录
 * @return 找到命令时返回 true
 */
// 解释：这一行定义函数 `GetCommandWithTimeout`，开始实现get命令withtimeout的具体逻辑。
bool UCommandExecutionManager::GetCommandWithTimeout(const FString& CommandId, double TimeoutSec, FCommandExecutionRecord& OutRecord)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `FCommandExecutionRecord* Record`，完成 fcommand执行记录记录 的更新。
    FCommandExecutionRecord* Record = Records.Find(CommandId);
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!Record) return false;
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (TimeoutSec > 0.0 && Record->Status == TEXT("running"))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `const double Now`，完成 constdoublenow 的更新。
        const double Now = FPlatformTime::Seconds();
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if ((Now - Record->StartTimeSec) > TimeoutSec)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：调用 `TEXT` 执行当前步骤需要的功能逻辑。
            Record->Status = TEXT("failed");
            // 解释：调用 `Printf` 执行当前步骤需要的功能逻辑。
            Record->Message = FString::Printf(TEXT("timeout(%.2fs)"), TimeoutSec);
            // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
            Record->EndTimeSec = Now;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行把右侧表达式的结果写入 `OutRecord`，完成 out记录 的更新。
    OutRecord = *Record;
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return true;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

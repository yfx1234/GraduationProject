// 解释：引入当前实现文件对应的头文件 `AgentManager.h`，使实现部分能够看到类和函数声明。
#include "AgentManager.h"

/** @brief AgentManager 全局单例实例 */
// 解释：这一行把右侧表达式的结果写入 `UAgentManager* UAgentManager::Instance`，完成 uagent管理器uagent管理器instance 的更新。
UAgentManager* UAgentManager::Instance = nullptr;

/**
 * @brief 获取全局单例
 * @return 管理器实例
 */
// 解释：这一行定义函数 `GetInstance`，开始实现getinstance的具体逻辑。
UAgentManager* UAgentManager::GetInstance()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!Instance)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `Instance`，完成 instance 的更新。
        Instance = NewObject<UAgentManager>();
        // 解释：调用 `AddToRoot` 执行当前步骤需要的功能逻辑。
        Instance->AddToRoot();
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return Instance;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 注册一个智能体
 * @param AgentId 智能体唯一 ID
 * @param Agent   对应 Actor
 */
// 解释：这一行定义函数 `RegisterAgent`，开始实现registeragent的具体逻辑。
void UAgentManager::RegisterAgent(const FString& AgentId, AActor* Agent)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!Agent || AgentId.IsEmpty())
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (AgentMap.Contains(AgentId))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `UE_LOG` 输出调试日志，便于运行时观察状态和排查问题。
        UE_LOG(LogTemp, Warning, TEXT("[AgentManager] Agent '%s' already registered, updating..."), *AgentId);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：调用 `Add` 执行当前步骤需要的功能逻辑。
    AgentMap.Add(AgentId, Agent);
    // 解释：调用 `Broadcast` 执行当前步骤需要的功能逻辑。
    OnAgentListChanged.Broadcast();
    // 解释：调用 `UE_LOG` 输出调试日志，便于运行时观察状态和排查问题。
    UE_LOG(LogTemp, Log, TEXT("[AgentManager] Registered: %s (%s)"), *AgentId, *Agent->GetClass()->GetName());
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 注册或复用某个 Actor 的 ID
 * @param PreferredAgentId 期望 ID
 * @param Agent            对应 Actor
 * @return 最终使用的 ID
 */
// 解释：这一行定义函数 `RegisterOrResolveAgent`，开始实现registerorresolveagent的具体逻辑。
FString UAgentManager::RegisterOrResolveAgent(const FString& PreferredAgentId, AActor* Agent)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!Agent)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return PreferredAgentId;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行把右侧表达式的结果写入 `FString ResolvedAgentId`，完成 fstringresolvedagentid 的更新。
    FString ResolvedAgentId = FindAgentId(Agent);
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (ResolvedAgentId.IsEmpty())
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `ResolvedAgentId`，完成 resolvedagentid 的更新。
        ResolvedAgentId = PreferredAgentId;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (ResolvedAgentId.IsEmpty())
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return FString();
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (GetAgent(ResolvedAgentId) != Agent)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `RegisterAgent` 执行当前步骤需要的功能逻辑。
        RegisterAgent(ResolvedAgentId, Agent);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return ResolvedAgentId;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 注销一个智能体
 * @param AgentId 目标 ID
 */
// 解释：这一行定义函数 `UnregisterAgent`，开始实现unregisteragent的具体逻辑。
void UAgentManager::UnregisterAgent(const FString& AgentId)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (AgentMap.Remove(AgentId) > 0)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `Broadcast` 执行当前步骤需要的功能逻辑。
        OnAgentListChanged.Broadcast();
        // 解释：调用 `UE_LOG` 输出调试日志，便于运行时观察状态和排查问题。
        UE_LOG(LogTemp, Log, TEXT("[AgentManager] Unregistered: %s"), *AgentId);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 按 ID 获取智能体
 * @param AgentId 智能体 ID
 * @return 对应 Actor；未命中时返回 nullptr
 */
// 解释：这一行定义函数 `GetAgent`，开始实现getagent的具体逻辑。
AActor* UAgentManager::GetAgent(const FString& AgentId) const
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `AActor* const* Found`，完成 aactorconstfound 的更新。
    AActor* const* Found = AgentMap.Find(AgentId);
    // 解释：这一行使用三目表达式选择返回值，在正常路径和兜底路径之间做快速切换。
    return Found ? *Found : nullptr;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 反向查找 Actor 对应的 ID
 * @param Agent 待查 Actor
 * @return 找到的 ID；未找到时返回空串
 */
// 解释：这一行定义函数 `FindAgentId`，开始实现findagentid的具体逻辑。
FString UAgentManager::FindAgentId(AActor* Agent) const
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!Agent)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return FString();
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
    for (const TPair<FString, AActor*>& Pair : AgentMap)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (Pair.Value == Agent)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return Pair.Key;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return FString();
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 获取所有已注册 Actor
 * @return Actor 数组
 */
// 解释：这一行定义函数 `GetAllAgents`，开始实现getallagents的具体逻辑。
TArray<AActor*> UAgentManager::GetAllAgents() const
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行声明成员或局部变量 `Result`，用于保存result。
    TArray<AActor*> Result;
    // 解释：调用 `GenerateValueArray` 执行当前步骤需要的功能逻辑。
    AgentMap.GenerateValueArray(Result);
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return Result;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 获取所有已注册 ID
 * @return ID 数组
 */
// 解释：这一行定义函数 `GetAllAgentIds`，开始实现getallagentids的具体逻辑。
TArray<FString> UAgentManager::GetAllAgentIds() const
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行声明成员或局部变量 `Result`，用于保存result。
    TArray<FString> Result;
    // 解释：调用 `GenerateKeyArray` 执行当前步骤需要的功能逻辑。
    AgentMap.GenerateKeyArray(Result);
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return Result;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 清理单例
 *
 * 仅清空映射并释放管理器自身，不回收场景中的 Actor。
 */
// 解释：这一行定义函数 `Cleanup`，开始实现cleanup的具体逻辑。
void UAgentManager::Cleanup()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (Instance)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `Empty` 执行当前步骤需要的功能逻辑。
        Instance->AgentMap.Empty();
        // 解释：调用 `RemoveFromRoot` 执行当前步骤需要的功能逻辑。
        Instance->RemoveFromRoot();
        // 解释：这一行把右侧表达式的结果写入 `Instance`，完成 instance 的更新。
        Instance = nullptr;
        // 解释：调用 `UE_LOG` 输出调试日志，便于运行时观察状态和排查问题。
        UE_LOG(LogTemp, Log, TEXT("[AgentManager] Instance cleanup completed"));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

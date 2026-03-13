#include "AgentManager.h"

/** @brief AgentManager 全局单例实例 */
UAgentManager* UAgentManager::Instance = nullptr;

/**
 * @brief 获取全局单例
 * @return 管理器实例
 */
UAgentManager* UAgentManager::GetInstance()
{
    if (!Instance)
    {
        Instance = NewObject<UAgentManager>();
        Instance->AddToRoot();
    }
    return Instance;
}

/**
 * @brief 注册一个智能体
 * @param AgentId 智能体唯一 ID
 * @param Agent   对应 Actor
 */
void UAgentManager::RegisterAgent(const FString& AgentId, AActor* Agent)
{
    if (!Agent || AgentId.IsEmpty())
    {
        return;
    }

    if (AgentMap.Contains(AgentId))
    {
        UE_LOG(LogTemp, Warning, TEXT("[AgentManager] Agent '%s' already registered, updating..."), *AgentId);
    }

    AgentMap.Add(AgentId, Agent);
    OnAgentListChanged.Broadcast();
    UE_LOG(LogTemp, Log, TEXT("[AgentManager] Registered: %s (%s)"), *AgentId, *Agent->GetClass()->GetName());
}

/**
 * @brief 注册或复用某个 Actor 的 ID
 * @param PreferredAgentId 期望 ID
 * @param Agent            对应 Actor
 * @return 最终使用的 ID
 */
FString UAgentManager::RegisterOrResolveAgent(const FString& PreferredAgentId, AActor* Agent)
{
    if (!Agent)
    {
        return PreferredAgentId;
    }

    FString ResolvedAgentId = FindAgentId(Agent);
    if (ResolvedAgentId.IsEmpty())
    {
        ResolvedAgentId = PreferredAgentId;
    }

    if (ResolvedAgentId.IsEmpty())
    {
        return FString();
    }

    if (GetAgent(ResolvedAgentId) != Agent)
    {
        RegisterAgent(ResolvedAgentId, Agent);
    }

    return ResolvedAgentId;
}

/**
 * @brief 注销一个智能体
 * @param AgentId 目标 ID
 */
void UAgentManager::UnregisterAgent(const FString& AgentId)
{
    if (AgentMap.Remove(AgentId) > 0)
    {
        OnAgentListChanged.Broadcast();
        UE_LOG(LogTemp, Log, TEXT("[AgentManager] Unregistered: %s"), *AgentId);
    }
}

/**
 * @brief 按 ID 获取智能体
 * @param AgentId 智能体 ID
 * @return 对应 Actor；未命中时返回 nullptr
 */
AActor* UAgentManager::GetAgent(const FString& AgentId) const
{
    AActor* const* Found = AgentMap.Find(AgentId);
    return Found ? *Found : nullptr;
}

/**
 * @brief 反向查找 Actor 对应的 ID
 * @param Agent 待查 Actor
 * @return 找到的 ID；未找到时返回空串
 */
FString UAgentManager::FindAgentId(AActor* Agent) const
{
    if (!Agent)
    {
        return FString();
    }

    for (const TPair<FString, AActor*>& Pair : AgentMap)
    {
        if (Pair.Value == Agent)
        {
            return Pair.Key;
        }
    }

    return FString();
}

/**
 * @brief 获取所有已注册 Actor
 * @return Actor 数组
 */
TArray<AActor*> UAgentManager::GetAllAgents() const
{
    TArray<AActor*> Result;
    AgentMap.GenerateValueArray(Result);
    return Result;
}

/**
 * @brief 获取所有已注册 ID
 * @return ID 数组
 */
TArray<FString> UAgentManager::GetAllAgentIds() const
{
    TArray<FString> Result;
    AgentMap.GenerateKeyArray(Result);
    return Result;
}

/**
 * @brief 清理单例
 *
 * 仅清空映射并释放管理器自身，不回收场景中的 Actor。
 */
void UAgentManager::Cleanup()
{
    if (Instance)
    {
        Instance->AgentMap.Empty();
        Instance->RemoveFromRoot();
        Instance = nullptr;
        UE_LOG(LogTemp, Log, TEXT("[AgentManager] Instance cleanup completed"));
    }
}
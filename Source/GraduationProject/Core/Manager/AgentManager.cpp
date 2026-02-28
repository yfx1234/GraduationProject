#include "AgentManager.h"

/** 定义UAgentManage单例实例静态成员，初始化为空指针 */
UAgentManager* UAgentManager::Instance = nullptr;

/**
 * 获取单例实例
 * @return AgentManager 实例
 * 首次调用时通过 NewObject 创建实例，并调用 AddToRoot() 防止被 UE 垃圾回收器回收。
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
 * 注册智能体到管理器
 * @param AgentId 智能体唯一标识符
 * @param Agent 对应的 Actor 指针
 * 空指针或空 ID 会被直接忽略
 * 注册成功后广播 OnAgentListChanged
 */
void UAgentManager::RegisterAgent(const FString& AgentId, AActor* Agent)
{
    if (!Agent || AgentId.IsEmpty()) return;
    if (AgentMap.Contains(AgentId)) UE_LOG(LogTemp, Warning, TEXT("[AgentManager] Agent '%s' already registered, updating... "), *AgentId);
    AgentMap.Add(AgentId, Agent);       // 添加到映射表
    OnAgentListChanged.Broadcast();     // 广播列表变更通知
    UE_LOG(LogTemp, Log, TEXT("[AgentManager] Registered: %s (%s)"), *AgentId, *Agent->GetClass()->GetName());
}

/**
 * 注销智能体
 * @param AgentId 要注销的智能体 ID
 * 从映射表中移除指定 ID 的智能体
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
 * 按 ID 查找智能体
 * @param AgentId 智能体 ID
 * @return 对应的 Actor 指针，未找到返回 nullptr
 */
AActor* UAgentManager::GetAgent(const FString& AgentId) const
{
    AActor* const* Found = AgentMap.Find(AgentId);
    return Found ? *Found : nullptr;
}

/**
 * 获取所有已注册智能体的 Actor 数组
 * @return Actor 指针数组
 */
TArray<AActor*> UAgentManager::GetAllAgents() const
{
    TArray<AActor*> Result;
    AgentMap.GenerateValueArray(Result);
    return Result;
}

/**
 * 获取所有已注册智能体的 ID 数组
 * @return ID 字符串数组
 */
TArray<FString> UAgentManager::GetAllAgentIds() const
{
    TArray<FString> Result;
    AgentMap.GenerateKeyArray(Result);
    return Result;
}

/**
 * 清理单例实例
 * 清空映射表，从 Root 集合中移除，允许 GC 回收
 * 将 Instance 置 nullptr
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

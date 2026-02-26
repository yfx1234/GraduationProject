/**
 * @file AgentManager.cpp
 * @brief 全局智能体管理器的实现文件
 *
 * 实现单例模式的智能体管理，包括懒加载实例创建、
 * 智能体的注册/注销/查找，以及单例清理。
 */

#include "AgentManager.h"

/** @brief 单例实例静态成员初始化为空指针 */
UAgentManager* UAgentManager::Instance = nullptr;

/**
 * @brief 获取单例实例（懒加载）
 * @return 全局唯一的 AgentManager 实例
 *
 * 首次调用时通过 NewObject 创建实例，
 * 并调用 AddToRoot() 防止被 UE 垃圾回收器回收。
 */
UAgentManager* UAgentManager::GetInstance()
{
    if (!Instance)
    {
        Instance = NewObject<UAgentManager>();
        Instance->AddToRoot(); // 防止GC回收
    }
    return Instance;
}

/**
 * @brief 注册智能体到管理器
 * @param AgentId 智能体唯一标识符
 * @param Agent 对应的 Actor 指针
 *
 * 空指针或空 ID 会被直接忽略。
 * 已存在的 ID 会被更新并打印警告日志。
 * 注册成功后广播 OnAgentListChanged 通知 UI 刷新。
 */
void UAgentManager::RegisterAgent(const FString& AgentId, AActor* Agent)
{
    // 参数校验
    if (!Agent || AgentId.IsEmpty())
    {
        return;
    }

    // 检查是否已存在同名智能体
    if (AgentMap.Contains(AgentId))
    {
        UE_LOG(LogTemp, Warning, TEXT("[AgentManager] Agent '%s' already registered, updating..."), *AgentId);
    }

    // 添加到映射表（存在时自动覆盖）
    AgentMap.Add(AgentId, Agent);
    // 广播列表变更通知
    OnAgentListChanged.Broadcast();
    UE_LOG(LogTemp, Log, TEXT("[AgentManager] Registered: %s (%s)"), *AgentId, *Agent->GetClass()->GetName());
}

/**
 * @brief 注销智能体
 * @param AgentId 要注销的智能体 ID
 *
 * 从映射表中移除指定 ID 的智能体。
 * 成功移除后广播通知。
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
 * @brief 按 ID 查找智能体
 * @param AgentId 智能体 ID
 * @return 对应的 Actor 指针，未找到返回 nullptr
 */
AActor* UAgentManager::GetAgent(const FString& AgentId) const
{
    AActor* const* Found = AgentMap.Find(AgentId);
    return Found ? *Found : nullptr;
}

/**
 * @brief 获取所有已注册智能体的 Actor 数组
 * @return Actor 指针数组（无序）
 */
TArray<AActor*> UAgentManager::GetAllAgents() const
{
    TArray<AActor*> Result;
    AgentMap.GenerateValueArray(Result);
    return Result;
}

/**
 * @brief 获取所有已注册智能体的 ID 数组
 * @return ID 字符串数组（无序）
 */
TArray<FString> UAgentManager::GetAllAgentIds() const
{
    TArray<FString> Result;
    AgentMap.GenerateKeyArray(Result);
    return Result;
}

/**
 * @brief 清理单例实例
 *
 * 清空映射表，从 Root 集合中移除（允许 GC 回收），
 * 将 Instance 置 nullptr。
 * 应在 GameInstance::Shutdown() 等退出流程中调用。
 */
void UAgentManager::Cleanup()
{
    if (Instance)
    {
        Instance->AgentMap.Empty();
        Instance->RemoveFromRoot(); // 允许GC回收
        Instance = nullptr;
        UE_LOG(LogTemp, Log, TEXT("[AgentManager] Singleton cleanup completed"));
    }
}

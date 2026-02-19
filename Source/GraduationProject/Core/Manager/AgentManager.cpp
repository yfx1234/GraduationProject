#include "AgentManager.h"

UAgentManager* UAgentManager::Instance = nullptr;

UAgentManager* UAgentManager::GetInstance()
{
    if (!Instance)
    {
        Instance = NewObject<UAgentManager>();
        Instance->AddToRoot(); // 防止GC回收
    }
    return Instance;
}

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

void UAgentManager::UnregisterAgent(const FString& AgentId)
{
    if (AgentMap.Remove(AgentId) > 0)
    {
        OnAgentListChanged.Broadcast();
        UE_LOG(LogTemp, Log, TEXT("[AgentManager] Unregistered: %s"), *AgentId);
    }
}

AActor* UAgentManager::GetAgent(const FString& AgentId) const
{
    AActor* const* Found = AgentMap.Find(AgentId);
    return Found ? *Found : nullptr;
}

TArray<AActor*> UAgentManager::GetAllAgents() const
{
    TArray<AActor*> Result;
    AgentMap.GenerateValueArray(Result);
    return Result;
}

TArray<FString> UAgentManager::GetAllAgentIds() const
{
    TArray<FString> Result;
    AgentMap.GenerateKeyArray(Result);
    return Result;
}

void UAgentManager::Cleanup()
{
    if (Instance)
    {
        Instance->AgentMap.Empty();
        Instance->RemoveFromRoot();
        Instance = nullptr;
        UE_LOG(LogTemp, Log, TEXT("[AgentManager] Singleton cleanup completed"));
    }
}

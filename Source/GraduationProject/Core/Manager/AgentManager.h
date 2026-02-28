#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "AgentManager.generated.h"

/** 智能体列表变更委托，当智能体注册或注销时广播此委托 */
DECLARE_MULTICAST_DELEGATE(FOnAgentListChanged);

/**
 * 全局智能体管理器
 * 维护所有智能体的 ID → Actor 映射表，提供：
 * 注册/注销智能体
 * 按 ID 查找智能体
 * 获取所有智能体列表
 * 列表变更通知
 */
UCLASS()
class GRADUATIONPROJECT_API UAgentManager : public UObject
{
    GENERATED_BODY()

public:
    /**
     * 获取单例实例
     * @return AgentManager 实例指针
     */
    static UAgentManager* GetInstance();

    /**
     * 注册一个智能体到管理器
     * @param AgentId 智能体的唯一标识符
     * @param Agent 智能体对应的 Actor 指针
     */
    UFUNCTION(BlueprintCallable, Category = "AgentManager")
    void RegisterAgent(const FString& AgentId, AActor* Agent);

    /**
     * 注销一个智能体
     * @param AgentId 要注销的智能体 ID
     */
    UFUNCTION(BlueprintCallable, Category = "AgentManager")
    void UnregisterAgent(const FString& AgentId);

    /**
     * 按 ID 查找智能体
     * @param AgentId 智能体 ID
     * @return 对应的 Actor 指针
     */
    UFUNCTION(BlueprintPure, Category = "AgentManager")
    AActor* GetAgent(const FString& AgentId) const;

    /**
     * 获取所有已注册智能体的 Actor 数组
     * @return Actor 指针数组
     */
    UFUNCTION(BlueprintPure, Category = "AgentManager")
    TArray<AActor*> GetAllAgents() const;

    /**
     * 获取所有已注册智能体的 ID 数组
     * @return ID 字符串数组
     */
    UFUNCTION(BlueprintPure, Category = "AgentManager")
    TArray<FString> GetAllAgentIds() const;

    /**
     * 获取当前已注册智能体的数量
     * @return 智能体数量
     */
    UFUNCTION(BlueprintPure, Category = "AgentManager")
    int32 GetAgentCount() const { return AgentMap.Num(); }

    /** 清理单例实例，释放资源 */
    static void Cleanup();

    /** 列表变更委托，当智能体注册或注销时广播 */
    FOnAgentListChanged OnAgentListChanged;

private:
    /** 单例实例指针 */
    static UAgentManager* Instance;

    /** 智能体 ID → Actor 的映射表 */
    UPROPERTY()
    TMap<FString, AActor*> AgentMap;
};

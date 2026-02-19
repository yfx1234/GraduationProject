#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "AgentManager.generated.h"

// 智能体列表变更委托
DECLARE_MULTICAST_DELEGATE(FOnAgentListChanged);

/**
 * 全局智能体管理器（单例）
 * 维护所有智能体列表（无人机、转台等），供TCP命令查找和UI显示使用
 * 参考AirSim的Vehicle管理思想
 */
UCLASS()
class GRADUATIONPROJECT_API UAgentManager : public UObject
{
    GENERATED_BODY()

public:
    /** 获取单例实例 */
    static UAgentManager* GetInstance();

    /** 注册智能体 */
    UFUNCTION(BlueprintCallable, Category = "AgentManager")
    void RegisterAgent(const FString& AgentId, AActor* Agent);

    /** 注销智能体 */
    UFUNCTION(BlueprintCallable, Category = "AgentManager")
    void UnregisterAgent(const FString& AgentId);

    /** 获取智能体 */
    UFUNCTION(BlueprintPure, Category = "AgentManager")
    AActor* GetAgent(const FString& AgentId) const;

    /** 获取所有智能体 */
    UFUNCTION(BlueprintPure, Category = "AgentManager")
    TArray<AActor*> GetAllAgents() const;

    /** 获取所有智能体ID */
    UFUNCTION(BlueprintPure, Category = "AgentManager")
    TArray<FString> GetAllAgentIds() const;

    /** 智能体数量 */
    UFUNCTION(BlueprintPure, Category = "AgentManager")
    int32 GetAgentCount() const { return AgentMap.Num(); }

    /** 清理单例实例 */
    static void Cleanup();

    /** 列表变更委托（UI监听用） */
    FOnAgentListChanged OnAgentListChanged;

private:
    static UAgentManager* Instance;

    UPROPERTY()
    TMap<FString, AActor*> AgentMap;
};

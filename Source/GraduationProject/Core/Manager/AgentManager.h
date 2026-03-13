#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "AgentManager.generated.h"

class AActor;

/** @brief 智能体列表变更委托，在注册/注销后广播 */
DECLARE_MULTICAST_DELEGATE(FOnAgentListChanged);

/**
 * @brief 全局智能体注册管理器
 *
 * 维护“智能体 ID -> Actor 指针”的映射关系，负责：
 * 1. 为 TCP / Blueprint / UI 层提供统一查询入口；
 * 2. 在运行时注册、更新和注销动态生成的 Actor；
 * 3. 广播列表变更事件，驱动 HUD 与列表控件刷新。
 */
UCLASS()
class GRADUATIONPROJECT_API UAgentManager : public UObject
{
    GENERATED_BODY()

public:
    /**
     * @brief 获取全局单例
     * @return 管理器实例；首次调用时自动创建并加入 Root 防止被 GC 回收
     */
    static UAgentManager* GetInstance();

    /**
     * @brief 注册一个智能体
     * @param AgentId 智能体唯一 ID
     * @param Agent   对应的 Actor 指针
     *
     * 如果同名 ID 已存在，则新指针会覆盖旧映射，并广播列表变更事件。
     */
    UFUNCTION(BlueprintCallable, Category = "AgentManager")
    void RegisterAgent(const FString& AgentId, AActor* Agent);

    /**
     * @brief 注册或复用一个 Actor 的现有 ID
     * @param PreferredAgentId 期望使用的 ID
     * @param Agent            待注册 Actor
     * @return 最终解析出的 ID；若 Actor 无效且无法推断，则返回空串
     *
     * 该接口用于避免同一个 Actor 被重复注册为多个 ID：
     * - 如果 Actor 已在表中，则直接返回已有 ID；
     * - 否则优先使用 PreferredAgentId 完成注册。
     */
    UFUNCTION(BlueprintCallable, Category = "AgentManager")
    FString RegisterOrResolveAgent(const FString& PreferredAgentId, AActor* Agent);

    /**
     * @brief 注销一个智能体
     * @param AgentId 要移除的智能体 ID
     */
    UFUNCTION(BlueprintCallable, Category = "AgentManager")
    void UnregisterAgent(const FString& AgentId);

    /**
     * @brief 按 ID 查询智能体
     * @param AgentId 智能体 ID
     * @return 对应的 Actor；不存在时返回 nullptr
     */
    UFUNCTION(BlueprintPure, Category = "AgentManager")
    AActor* GetAgent(const FString& AgentId) const;

    /**
     * @brief 反向查找某个 Actor 对应的 ID
     * @param Agent 待查询 Actor
     * @return 匹配到的 ID；未找到时返回空串
     */
    UFUNCTION(BlueprintPure, Category = "AgentManager")
    FString FindAgentId(AActor* Agent) const;

    /**
     * @brief 获取所有已注册的 Actor
     * @return Actor 指针数组
     */
    UFUNCTION(BlueprintPure, Category = "AgentManager")
    TArray<AActor*> GetAllAgents() const;

    /**
     * @brief 获取所有已注册的 ID
     * @return 字符串数组
     */
    UFUNCTION(BlueprintPure, Category = "AgentManager")
    TArray<FString> GetAllAgentIds() const;

    /** @brief 当前注册数量 */
    UFUNCTION(BlueprintPure, Category = "AgentManager")
    int32 GetAgentCount() const { return AgentMap.Num(); }

    /**
     * @brief 清理单例与内部映射
     *
     * 仅释放管理器自身，不会销毁已注册 Actor 的生命周期。
     */
    static void Cleanup();

    /** @brief 列表变更广播，在注册或注销成功后触发 */
    FOnAgentListChanged OnAgentListChanged;

private:
    /** @brief 全局单例指针 */
    static UAgentManager* Instance;

    /** @brief 智能体 ID 到 Actor 的映射表 */
    UPROPERTY()
    TMap<FString, AActor*> AgentMap;
};
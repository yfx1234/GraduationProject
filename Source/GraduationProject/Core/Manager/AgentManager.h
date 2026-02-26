/**
 * @file AgentManager.h
 * @brief 全局智能体管理器（单例）的头文件
 *
 * 本文件定义了 UAgentManager 类，采用单例模式管理所有仿真中的智能体（无人机、转台等）。
 * 提供注册、注销、查找智能体的功能，供 TCP 命令路由和 UI 显示使用。
 * 参考 AirSim 的 Vehicle 管理思想。
 */

#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "AgentManager.generated.h"

/**
 * @brief 智能体列表变更委托
 *
 * 当智能体注册或注销时广播此委托，UI 可监听此事件刷新显示。
 */
DECLARE_MULTICAST_DELEGATE(FOnAgentListChanged);

/**
 * 全局智能体管理器（单例模式）
 *
 * 维护所有智能体的 ID → Actor 映射表，提供：
 * - 注册/注销智能体
 * - 按 ID 查找智能体
 * - 获取所有智能体列表
 * - 列表变更通知（委托）
 *
 * 生命周期：通过 GetInstance() 懒加载创建，通过 Cleanup() 销毁。
 * 使用 AddToRoot() 防止 GC 回收。
 */
UCLASS()
class GRADUATIONPROJECT_API UAgentManager : public UObject
{
    GENERATED_BODY()

public:
    /**
     * @brief 获取单例实例（懒加载）
     * @return 全局唯一的 AgentManager 实例指针
     *
     * 首次调用时创建实例并 AddToRoot() 防止 GC 回收。
     */
    static UAgentManager* GetInstance();

    /**
     * @brief 注册一个智能体到管理器
     * @param AgentId 智能体的唯一标识符（如 "drone_0"、"turret_0"）
     * @param Agent 智能体对应的 Actor 指针
     *
     * 如果 AgentId 已存在，会打印警告并更新 Actor 引用。
     * 注册成功后广播 OnAgentListChanged 委托。
     */
    UFUNCTION(BlueprintCallable, Category = "AgentManager")
    void RegisterAgent(const FString& AgentId, AActor* Agent);

    /**
     * @brief 注销一个智能体
     * @param AgentId 要注销的智能体 ID
     *
     * 从映射表中移除，成功注销后广播 OnAgentListChanged 委托。
     */
    UFUNCTION(BlueprintCallable, Category = "AgentManager")
    void UnregisterAgent(const FString& AgentId);

    /**
     * @brief 按 ID 查找智能体
     * @param AgentId 智能体 ID
     * @return 对应的 Actor 指针，未找到返回 nullptr
     */
    UFUNCTION(BlueprintPure, Category = "AgentManager")
    AActor* GetAgent(const FString& AgentId) const;

    /**
     * @brief 获取所有已注册智能体的 Actor 数组
     * @return Actor 指针数组
     */
    UFUNCTION(BlueprintPure, Category = "AgentManager")
    TArray<AActor*> GetAllAgents() const;

    /**
     * @brief 获取所有已注册智能体的 ID 数组
     * @return ID 字符串数组
     */
    UFUNCTION(BlueprintPure, Category = "AgentManager")
    TArray<FString> GetAllAgentIds() const;

    /**
     * @brief 获取当前已注册智能体的数量
     * @return 智能体数量
     */
    UFUNCTION(BlueprintPure, Category = "AgentManager")
    int32 GetAgentCount() const { return AgentMap.Num(); }

    /**
     * @brief 清理单例实例，释放资源
     *
     * 清空映射表，调用 RemoveFromRoot() 允许 GC 回收，
     * 将单例指针置 nullptr。通常在 GameInstance::Shutdown() 时调用。
     */
    static void Cleanup();

    /**
     * @brief 列表变更委托
     *
     * 当智能体注册或注销时广播。UI 系统可绑定此委托来自动刷新显示。
     */
    FOnAgentListChanged OnAgentListChanged;

private:
    /** @brief 单例实例指针 */
    static UAgentManager* Instance;

    /** @brief 智能体 ID → Actor 的映射表 */
    UPROPERTY()
    TMap<FString, AActor*> AgentMap;
};

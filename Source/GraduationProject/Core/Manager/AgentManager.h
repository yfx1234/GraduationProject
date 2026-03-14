// 解释：使用 `#pragma once` 防止该头文件在编译过程中被重复包含。
#pragma once

// 解释：引入 Unreal 的核心基础头文件，提供常用容器、数学类型和日志宏。
#include "CoreMinimal.h"
// 解释：引入 `NoExportTypes.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "UObject/NoExportTypes.h"
// 解释：引入 `AgentManager.generated.h`，让 Unreal Header Tool 生成的反射代码在本文件中可见。
#include "AgentManager.generated.h"

// 解释：这一行声明 类 `AActor`，用于封装aactor相关的数据与行为。
class AActor;

/** @brief 智能体列表变更委托，在注册/注销后广播 */
// 解释：调用 `DECLARE_MULTICAST_DELEGATE` 执行当前步骤需要的功能逻辑。
DECLARE_MULTICAST_DELEGATE(FOnAgentListChanged);

/**
 * @brief 全局智能体注册管理器
 *
 * 维护“智能体 ID -> Actor 指针”的映射关系，负责：
 * 1. 为 TCP / Blueprint / UI 层提供统一查询入口；
 * 2. 在运行时注册、更新和注销动态生成的 Actor；
 * 3. 广播列表变更事件，驱动 HUD 与列表控件刷新。
 */
// 解释：使用 `UCLASS` 宏把当前类型注册到 Unreal 反射系统中，便于蓝图、序列化和编辑器识别。
UCLASS()
// 解释：这一行声明 类 `UAgentManager`，用于封装uagent管理器相关的数据与行为。
class GRADUATIONPROJECT_API UAgentManager : public UObject
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：展开 Unreal Header Tool 生成的样板代码，这是 UObject、Actor、组件等类型正常工作的基础。
    GENERATED_BODY()

// 解释：从这一行开始进入 `public` 访问区，下面的成员会对外部模块公开。
public:
    /**
     * @brief 获取全局单例
     * @return 管理器实例；首次调用时自动创建并加入 Root 防止被 GC 回收
     */
    // 解释：调用 `GetInstance` 执行当前步骤需要的功能逻辑。
    static UAgentManager* GetInstance();

    /**
     * @brief 注册一个智能体
     * @param AgentId 智能体唯一 ID
     * @param Agent   对应的 Actor 指针
     *
     * 如果同名 ID 已存在，则新指针会覆盖旧映射，并广播列表变更事件。
     */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "AgentManager")
    // 解释：调用 `RegisterAgent` 执行当前步骤需要的功能逻辑。
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
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "AgentManager")
    // 解释：调用 `RegisterOrResolveAgent` 执行当前步骤需要的功能逻辑。
    FString RegisterOrResolveAgent(const FString& PreferredAgentId, AActor* Agent);

    /**
     * @brief 注销一个智能体
     * @param AgentId 要移除的智能体 ID
     */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "AgentManager")
    // 解释：调用 `UnregisterAgent` 执行当前步骤需要的功能逻辑。
    void UnregisterAgent(const FString& AgentId);

    /**
     * @brief 按 ID 查询智能体
     * @param AgentId 智能体 ID
     * @return 对应的 Actor；不存在时返回 nullptr
     */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintPure, Category = "AgentManager")
    // 解释：调用 `GetAgent` 执行当前步骤需要的功能逻辑。
    AActor* GetAgent(const FString& AgentId) const;

    /**
     * @brief 反向查找某个 Actor 对应的 ID
     * @param Agent 待查询 Actor
     * @return 匹配到的 ID；未找到时返回空串
     */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintPure, Category = "AgentManager")
    // 解释：调用 `FindAgentId` 执行当前步骤需要的功能逻辑。
    FString FindAgentId(AActor* Agent) const;

    /**
     * @brief 获取所有已注册的 Actor
     * @return Actor 指针数组
     */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintPure, Category = "AgentManager")
    // 解释：调用 `GetAllAgents` 执行当前步骤需要的功能逻辑。
    TArray<AActor*> GetAllAgents() const;

    /**
     * @brief 获取所有已注册的 ID
     * @return 字符串数组
     */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintPure, Category = "AgentManager")
    // 解释：调用 `GetAllAgentIds` 执行当前步骤需要的功能逻辑。
    TArray<FString> GetAllAgentIds() const;

    /** @brief 当前注册数量 */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintPure, Category = "AgentManager")
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    int32 GetAgentCount() const { return AgentMap.Num(); }

    /**
     * @brief 清理单例与内部映射
     *
     * 仅释放管理器自身，不会销毁已注册 Actor 的生命周期。
     */
    // 解释：调用 `Cleanup` 执行当前步骤需要的功能逻辑。
    static void Cleanup();

    /** @brief 列表变更广播，在注册或注销成功后触发 */
    // 解释：这一行声明成员或局部变量 `OnAgentListChanged`，用于保存onagent列表changed。
    FOnAgentListChanged OnAgentListChanged;

// 解释：从这一行开始进入 `private` 访问区，下面的成员只允许类内部访问。
private:
    /** @brief 全局单例指针 */
    // 解释：这一行声明成员或局部变量 `Instance`，用于保存instance。
    static UAgentManager* Instance;

    /** @brief 智能体 ID 到 Actor 的映射表 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY()
    // 解释：这一行声明成员或局部变量 `AgentMap`，用于保存agentmap。
    TMap<FString, AActor*> AgentMap;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
};

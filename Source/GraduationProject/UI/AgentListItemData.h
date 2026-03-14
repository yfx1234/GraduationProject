// 解释：使用 `#pragma once` 防止该头文件在编译过程中被重复包含。
#pragma once

// 解释：引入 Unreal 的核心基础头文件，提供常用容器、数学类型和日志宏。
#include "CoreMinimal.h"
// 解释：引入 `NoExportTypes.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "UObject/NoExportTypes.h"
// 解释：引入 `AgentListItemData.generated.h`，让 Unreal Header Tool 生成的反射代码在本文件中可见。
#include "AgentListItemData.generated.h"

/**
 * @brief Agent 列表单项数据模型
 * 作为 Slate 列表中的轻量数据载体，保存智能体 ID、类型和对象弱引用，
 * 自身不拥有 Actor 生命周期，只负责给 UI 层提供可绑定的数据视图。
 */
// 解释：使用 `UCLASS` 宏把当前类型注册到 Unreal 反射系统中，便于蓝图、序列化和编辑器识别。
UCLASS()
// 解释：这一行声明 类 `UAgentListItemData`，用于封装uagent列表项data相关的数据与行为。
class GRADUATIONPROJECT_API UAgentListItemData : public UObject
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：展开 Unreal Header Tool 生成的样板代码，这是 UObject、Actor、组件等类型正常工作的基础。
    GENERATED_BODY()

// 解释：从这一行开始进入 `public` 访问区，下面的成员会对外部模块公开。
public:
    /** @brief 智能体 ID */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY()
    // 解释：这一行声明成员或局部变量 `AgentId`，用于保存agentid。
    FString AgentId;

    /** @brief 智能体类型，例如 `Drone` 或 `Turret` */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY()
    // 解释：这一行声明成员或局部变量 `AgentType`，用于保存agenttype。
    FString AgentType;

    /** @brief 智能体对应的 Actor 弱引用 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY()
    // 解释：这一行声明成员或局部变量 `AgentActor`，用于保存agentActor。
    TWeakObjectPtr<AActor> AgentActor;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
};

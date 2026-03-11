#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "AgentListItemData.generated.h"

/**
 * @brief Agent 列表单项数据模型
 * 作为 Slate 列表中的轻量数据载体，保存智能体 ID、类型和对象弱引用。
 */
UCLASS()
class GRADUATIONPROJECT_API UAgentListItemData : public UObject
{
    GENERATED_BODY()

public:
    /** @brief 智能体 ID */
    UPROPERTY()
    FString AgentId;

    /** @brief 智能体类型，例如 `Drone` 或 `Turret` */
    UPROPERTY()
    FString AgentType;

    /** @brief 智能体对应的 Actor 弱引用 */
    UPROPERTY()
    TWeakObjectPtr<AActor> AgentActor;
};

#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "AgentListItemData.generated.h"

UCLASS()
class GRADUATIONPROJECT_API UAgentListItemData : public UObject
{
    GENERATED_BODY()

public:
    UPROPERTY()
    FString AgentId;

    UPROPERTY()
    FString AgentType;

    UPROPERTY()
    TWeakObjectPtr<AActor> AgentActor;
};

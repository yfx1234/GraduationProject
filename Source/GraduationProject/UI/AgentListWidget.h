#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "Widgets/Views/SListView.h"
#include "AgentListWidget.generated.h"

class ACameraPawn;

UCLASS()
class GRADUATIONPROJECT_API UAgentListWidget : public UUserWidget
{
    GENERATED_BODY()

public:
    void SetCameraPawn(ACameraPawn* InCameraPawn);

protected:
    virtual TSharedRef<SWidget> RebuildWidget() override;
    virtual void NativeConstruct() override;
    virtual void NativeDestruct() override;

private:
    void RefreshAgentList();
    void SortItems();
    TSharedRef<ITableRow> HandleGenerateRow(UObject* Item, const TSharedRef<STableViewBase>& OwnerTable);
    void HandleSelectionChanged(UObject* Item, ESelectInfo::Type SelectInfo);

private:
    UPROPERTY(Transient)
    TArray<UObject*> ListDataItems;

    TSharedPtr<SListView<UObject*>> AgentListView;
    TWeakObjectPtr<ACameraPawn> CameraPawn;
};

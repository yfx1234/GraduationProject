#include "AgentListWidget.h"

#include "AgentListItemData.h"
#include "Styling/CoreStyle.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/SOverlay.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Views/STableRow.h"
#include "GraduationProject/Core/CameraPawn.h"
#include "GraduationProject/Core/Manager/AgentManager.h"
#include "GraduationProject/Drone/DronePawn.h"
#include "GraduationProject/Turret/TurretPawn.h"

namespace
{
    FString DetectAgentType(AActor* Agent)
    {
        if (Cast<ADronePawn>(Agent))
        {
            return TEXT("drone");
        }
        if (Cast<ATurretPawn>(Agent))
        {
            return TEXT("turret");
        }
        return TEXT("actor");
    }

    int32 GetAgentPriority(const UAgentListItemData* Data)
    {
        if (!Data)
        {
            return 100;
        }

        if (Data->AgentId.Equals(TEXT("drone_0"), ESearchCase::IgnoreCase))
        {
            return 0;
        }
        if (Data->AgentType == TEXT("drone"))
        {
            return 10;
        }
        if (Data->AgentId.Equals(TEXT("turret_0"), ESearchCase::IgnoreCase))
        {
            return 20;
        }
        if (Data->AgentType == TEXT("turret"))
        {
            return 30;
        }
        return 50;
    }
}

void UAgentListWidget::SetCameraPawn(ACameraPawn* InCameraPawn)
{
    CameraPawn = InCameraPawn;
}

TSharedRef<SWidget> UAgentListWidget::RebuildWidget()
{
    return SNew(SOverlay)
        + SOverlay::Slot()
        .HAlign(HAlign_Right)
        .VAlign(VAlign_Top)
        .Padding(FMargin(0.0f, 20.0f, 20.0f, 0.0f))
        [
            SNew(SBox)
            .WidthOverride(360.0f)
            .HeightOverride(320.0f)
            [
                SNew(SBorder)
                .BorderBackgroundColor(FLinearColor(0.08f, 0.10f, 0.12f, 0.88f))
                .Padding(10.0f)
                [
                    SNew(SVerticalBox)
                    + SVerticalBox::Slot()
                    .AutoHeight()
                    .Padding(5.0f)
                    [
                        SNew(STextBlock)
                        .Text(FText::FromString(TEXT("Agent Manager")))
                        .Font(FCoreStyle::GetDefaultFontStyle("Bold", 14))
                    ]
                    + SVerticalBox::Slot()
                    .AutoHeight()
                    .Padding(FMargin(5.0f, 0.0f, 5.0f, 8.0f))
                    [
                        SNew(STextBlock)
                        .Text(FText::FromString(TEXT("Select an agent to focus camera")))
                        .ColorAndOpacity(FSlateColor(FLinearColor(0.75f, 0.78f, 0.82f, 1.0f)))
                    ]
                    + SVerticalBox::Slot()
                    .FillHeight(1.0f)
                    [
                        SAssignNew(AgentListView, SListView<UObject*>)
                        .ListItemsSource(&ListDataItems)
                        .OnGenerateRow_UObject(this, &UAgentListWidget::HandleGenerateRow)
                        .OnSelectionChanged_UObject(this, &UAgentListWidget::HandleSelectionChanged)
                        .SelectionMode(ESelectionMode::Single)
                    ]
                ]
            ]
        ];
}

void UAgentListWidget::NativeConstruct()
{
    Super::NativeConstruct();

    if (UAgentManager* Manager = UAgentManager::GetInstance())
    {
        Manager->OnAgentListChanged.AddUObject(this, &UAgentListWidget::RefreshAgentList);
    }

    RefreshAgentList();
}

void UAgentListWidget::NativeDestruct()
{
    if (UAgentManager* Manager = UAgentManager::GetInstance())
    {
        Manager->OnAgentListChanged.RemoveAll(this);
    }

    Super::NativeDestruct();
}

void UAgentListWidget::SortItems()
{
    ListDataItems.Sort([](const UObject& A, const UObject& B)
    {
        const UAgentListItemData* ItemA = Cast<UAgentListItemData>(&A);
        const UAgentListItemData* ItemB = Cast<UAgentListItemData>(&B);

        const int32 PriorityA = GetAgentPriority(ItemA);
        const int32 PriorityB = GetAgentPriority(ItemB);
        if (PriorityA != PriorityB)
        {
            return PriorityA < PriorityB;
        }

        const FString IdA = ItemA ? ItemA->AgentId : TEXT("");
        const FString IdB = ItemB ? ItemB->AgentId : TEXT("");
        return IdA < IdB;
    });
}

void UAgentListWidget::RefreshAgentList()
{
    UAgentManager* Manager = UAgentManager::GetInstance();
    if (!Manager)
    {
        return;
    }

    const TArray<FString> LatestIds = Manager->GetAllAgentIds();
    TMap<AActor*, UAgentListItemData*> ExistingByActor;
    ExistingByActor.Reserve(ListDataItems.Num());

    for (UObject* Item : ListDataItems)
    {
        UAgentListItemData* Data = Cast<UAgentListItemData>(Item);
        if (!Data)
        {
            continue;
        }

        if (AActor* Actor = Data->AgentActor.Get())
        {
            ExistingByActor.Add(Actor, Data);
        }
    }

    TSet<AActor*> LatestActors;
    bool bChanged = false;

    for (const FString& Id : LatestIds)
    {
        AActor* Agent = Manager->GetAgent(Id);
        if (!IsValid(Agent))
        {
            continue;
        }

        LatestActors.Add(Agent);

        if (UAgentListItemData** FoundData = ExistingByActor.Find(Agent))
        {
            UAgentListItemData* Data = *FoundData;
            const FString NewType = DetectAgentType(Agent);
            if (Data->AgentId != Id || Data->AgentType != NewType)
            {
                Data->AgentId = Id;
                Data->AgentType = NewType;
                bChanged = true;
            }
            continue;
        }

        UAgentListItemData* NewData = NewObject<UAgentListItemData>(this);
        NewData->AgentId = Id;
        NewData->AgentType = DetectAgentType(Agent);
        NewData->AgentActor = Agent;
        ListDataItems.Add(NewData);
        bChanged = true;
    }

    for (int32 Index = ListDataItems.Num() - 1; Index >= 0; --Index)
    {
        UAgentListItemData* Data = Cast<UAgentListItemData>(ListDataItems[Index]);
        if (!Data)
        {
            ListDataItems.RemoveAt(Index);
            bChanged = true;
            continue;
        }

        AActor* Actor = Data->AgentActor.Get();
        if (!IsValid(Actor) || !LatestActors.Contains(Actor))
        {
            ListDataItems.RemoveAt(Index);
            bChanged = true;
        }
    }

    if (bChanged)
    {
        SortItems();
        if (AgentListView.IsValid())
        {
            AgentListView->RequestListRefresh();
        }
    }
}

TSharedRef<ITableRow> UAgentListWidget::HandleGenerateRow(UObject* Item, const TSharedRef<STableViewBase>& OwnerTable)
{
    const UAgentListItemData* Data = Cast<UAgentListItemData>(Item);
    const FString Label = Data
        ? FString::Printf(TEXT("[%s] %s"), *Data->AgentType, *Data->AgentId)
        : TEXT("<invalid>");

    return SNew(STableRow<UObject*>, OwnerTable)
        [
            SNew(STextBlock)
            .Text(FText::FromString(Label))
        ];
}

void UAgentListWidget::HandleSelectionChanged(UObject* Item, ESelectInfo::Type SelectInfo)
{
    ACameraPawn* Pawn = CameraPawn.Get();
    if (!Pawn)
    {
        return;
    }

    UAgentListItemData* Data = Cast<UAgentListItemData>(Item);
    if (!Data)
    {
        Pawn->OnItemClicked(TEXT(""), nullptr);
        return;
    }

    Pawn->OnItemClicked(Data->AgentId, Data->AgentActor.Get());
}




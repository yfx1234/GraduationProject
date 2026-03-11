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
    /**
     * @brief 根据 Actor 实际类型生成列表显示类型
     * @param Agent 智能体 Actor
     * @return `drone`、`turret` 或 `actor`
     */
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

    /**
     * @brief 计算列表排序优先级
     * @param Data 列表项数据
     * @return 数值越小，排序越靠前
     * 默认将 `drone_0`、其他无人机、`turret_0`、其他炮台依次排前。
     */
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

/**
 * @brief 绑定用于联动的相机 Pawn
 * @param InCameraPawn 相机实例
 */
void UAgentListWidget::SetCameraPawn(ACameraPawn* InCameraPawn)
{
    CameraPawn = InCameraPawn;
}

/**
 * @brief 构建 Agent 列表 Slate 控件树
 * @return 根 Slate 控件
 * 列表固定停靠在右上角，包含标题、提示文本和单选列表视图。
 */
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

/**
 * @brief 初始化列表控件
 * 注册 Agent 列表变化回调，并立即执行一次初始刷新。
 */
void UAgentListWidget::NativeConstruct()
{
    Super::NativeConstruct();

    if (UAgentManager* Manager = UAgentManager::GetInstance())
    {
        Manager->OnAgentListChanged.AddUObject(this, &UAgentListWidget::RefreshAgentList);
    }

    RefreshAgentList();
}

/**
 * @brief 销毁列表控件
 * 解除对 `AgentManager` 事件的绑定，避免悬挂回调。
 */
void UAgentListWidget::NativeDestruct()
{
    if (UAgentManager* Manager = UAgentManager::GetInstance())
    {
        Manager->OnAgentListChanged.RemoveAll(this);
    }

    Super::NativeDestruct();
}

/**
 * @brief 对当前列表项执行稳定排序
 * 先按类型优先级排序，再按字符串 ID 排序。
 */
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

/**
 * @brief 根据 `AgentManager` 当前状态刷新列表内容
 * 复用已存在的数据项，只有在新增、删除或类型变化时才触发列表重建。
 */
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

/**
 * @brief 生成单行列表项视图
 * @param Item 列表数据对象
 * @param OwnerTable 所属表格
 * @return Slate 行控件
 */
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

/**
 * @brief 处理列表选中变化
 * @param Item 当前选中的数据项
 * @param SelectInfo 选中来源
 * 若数据无效，则通知相机切回自由视角。
 */
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

#include "AgentListWidget.h"

#include "AgentListItemData.h"
#include "Styling/CoreStyle.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/SOverlay.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Views/STableRow.h"
#include "GraduationProject/Core/CameraPawn.h"
#include "GraduationProject/Core/Manager/AgentManager.h"
#include "GraduationProject/Drone/DronePawn.h"
#include "GraduationProject/Turret/TurretPawn.h"

// ──── 匿名工具函数 ────
namespace
{
    /** @brief 根据 Actor 类型返回字符串标签 */
    FString GetAgentType(AActor* Agent)
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

    /** @brief Agent 排序优先级：drone_0 最高，其次 drone，再 turret，最后 actor */
    int32 GetAgentPriority(const UAgentListItemData* ItemData)
    {
        if (!ItemData)
        {
            return 100;
        }

        if (ItemData->AgentId.Equals(TEXT("drone_0"), ESearchCase::IgnoreCase))
        {
            return 0;
        }
        if (ItemData->AgentType == TEXT("drone"))
        {
            return 10;
        }
        if (ItemData->AgentId.Equals(TEXT("turret_0"), ESearchCase::IgnoreCase))
        {
            return 20;
        }
        if (ItemData->AgentType == TEXT("turret"))
        {
            return 30;
        }
        return 50;
    }
}

// ──── 公开接口 ────

/** @brief 注入摄像机 Pawn 引用 */
void UAgentListWidget::SetCameraPawn(ACameraPawn* InCameraPawn)
{
    CameraPawn = InCameraPawn;
}

// ──── Slate 构建 ────

/** @brief 构建控件树：半透明背景面板 → 标题 + 副标题 + Agent 列表 */
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
                        .ListItemsSource(&AgentItems)
                        .OnGenerateRow_UObject(this, &UAgentListWidget::BuildAgentRow)
                        .OnSelectionChanged_UObject(this, &UAgentListWidget::HandleAgentSelected)
                        .SelectionMode(ESelectionMode::Single)
                    ]
                ]
            ]
        ];
}

// ──── 生命周期 ────

/** @brief 订阅 AgentManager 列表变更事件，并立即加载一次 */
void UAgentListWidget::NativeConstruct()
{
    Super::NativeConstruct();

    if (UAgentManager* Manager = UAgentManager::GetInstance())
    {
        Manager->OnAgentListChanged.AddUObject(this, &UAgentListWidget::ReloadAgentItems);  // 监听 Agent 增删
    }

    ReloadAgentItems();
}

/** @brief 取消事件订阅，避免悬挂引用 */
void UAgentListWidget::NativeDestruct()
{
    if (UAgentManager* Manager = UAgentManager::GetInstance())
    {
        Manager->OnAgentListChanged.RemoveAll(this);
    }

    Super::NativeDestruct();
}

// ──── 数据管理 ────

/** @brief 按 GetAgentPriority 排序列表项，同优先级按 ID 字典序 */
void UAgentListWidget::SortAgentItems()
{
    AgentItems.Sort([](const UObject& LeftItem, const UObject& RightItem)
    {
        const UAgentListItemData* LeftData = Cast<UAgentListItemData>(&LeftItem);
        const UAgentListItemData* RightData = Cast<UAgentListItemData>(&RightItem);

        const int32 LeftPriority = GetAgentPriority(LeftData);
        const int32 RightPriority = GetAgentPriority(RightData);
        if (LeftPriority != RightPriority)
        {
            return LeftPriority < RightPriority;
        }

        const FString LeftId = LeftData ? LeftData->AgentId : TEXT("");
        const FString RightId = RightData ? RightData->AgentId : TEXT("");
        return LeftId < RightId;
    });
}

/**
 * @brief 增量刷新 Agent 列表
 *
 * 1) 用 Actor 指针做映射索引已有条目
 * 2) 遍历最新 ID，复用或新建条目
 * 3) 移除已不存在的条目
 * 4) 有变更时重新排序并刷新 UI
 */
void UAgentListWidget::ReloadAgentItems()
{
    UAgentManager* Manager = UAgentManager::GetInstance();
    if (!Manager)
    {
        return;
    }

    const TArray<FString> LatestIds = Manager->GetAllAgentIds();  // 当前所有 Agent ID
    TMap<AActor*, UAgentListItemData*> ExistingItemsByActor;       // Actor→已有条目 映射
    ExistingItemsByActor.Reserve(AgentItems.Num());

    for (UObject* Item : AgentItems)
    {
        UAgentListItemData* ItemData = Cast<UAgentListItemData>(Item);
        if (!ItemData)
        {
            continue;
        }

        if (AActor* Agent = ItemData->AgentActor.Get())
        {
            ExistingItemsByActor.Add(Agent, ItemData);
        }
    }

    TSet<AActor*> LatestActors;   // 当前帧存在的 Actor 集合
    bool bChanged = false;         // 是否有增删改

    // ── 第二步：遍历最新 Agent，复用或创建条目 ──
    for (const FString& AgentId : LatestIds)
    {
        AActor* Agent = Manager->GetAgent(AgentId);
        if (!IsValid(Agent))
        {
            continue;
        }

        LatestActors.Add(Agent);

        if (UAgentListItemData** FoundItem = ExistingItemsByActor.Find(Agent))
        {
            UAgentListItemData* ItemData = *FoundItem;
            const FString NewType = GetAgentType(Agent);
            if (ItemData->AgentId != AgentId || ItemData->AgentType != NewType)
            {
                ItemData->AgentId = AgentId;
                ItemData->AgentType = NewType;
                bChanged = true;
            }
            continue;
        }

        UAgentListItemData* NewItem = NewObject<UAgentListItemData>(this);
        NewItem->AgentId = AgentId;
        NewItem->AgentType = GetAgentType(Agent);
        NewItem->AgentActor = Agent;
        AgentItems.Add(NewItem);
        bChanged = true;
    }

    // ── 第三步：反向遍历移除已失效条目 ──
    for (int32 Index = AgentItems.Num() - 1; Index >= 0; --Index)
    {
        UAgentListItemData* ItemData = Cast<UAgentListItemData>(AgentItems[Index]);
        if (!ItemData)
        {
            AgentItems.RemoveAt(Index);
            bChanged = true;
            continue;
        }

        AActor* Agent = ItemData->AgentActor.Get();
        if (!IsValid(Agent) || !LatestActors.Contains(Agent))
        {
            AgentItems.RemoveAt(Index);
            bChanged = true;
        }
    }

    // ── 第四步：有变更则排序并通知列表刷新 ──
    if (bChanged)
    {
        SortAgentItems();
        if (AgentListView.IsValid())
        {
            AgentListView->RequestListRefresh();
        }
    }
}

// ──── 行构建与选择回调 ────

/** @brief 为每行创建 "[type] agent_id" 格式的文本控件 */
TSharedRef<ITableRow> UAgentListWidget::BuildAgentRow(UObject* Item, const TSharedRef<STableViewBase>& OwnerTable)
{
    const UAgentListItemData* ItemData = Cast<UAgentListItemData>(Item);
    const FString Label = ItemData
        ? FString::Printf(TEXT("[%s] %s"), *ItemData->AgentType, *ItemData->AgentId)
        : TEXT("<invalid>");

    return SNew(STableRow<UObject*>, OwnerTable)
        [
            SNew(STextBlock)
            .Text(FText::FromString(Label))
        ];
}

/** @brief 用户选中行时通知 CameraPawn 跟随该 Agent */
void UAgentListWidget::HandleAgentSelected(UObject* Item, ESelectInfo::Type SelectInfo)
{
    ACameraPawn* Pawn = CameraPawn.Get();
    if (!Pawn)
    {
        return;
    }

    UAgentListItemData* ItemData = Cast<UAgentListItemData>(Item);
    if (!ItemData)
    {
        Pawn->OnItemClicked(TEXT(""), nullptr);
        return;
    }

    Pawn->OnItemClicked(ItemData->AgentId, ItemData->AgentActor.Get());
}

// 解释：引入当前实现文件对应的头文件 `AgentListWidget.h`，使实现部分能够看到类和函数声明。
#include "AgentListWidget.h"

// 解释：引入 `AgentListItemData.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "AgentListItemData.h"
// 解释：引入 `CoreStyle.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "Styling/CoreStyle.h"
// 解释：引入 `SBorder.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "Widgets/Layout/SBorder.h"
// 解释：引入 `SBox.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "Widgets/Layout/SBox.h"
// 解释：引入 `SBoxPanel.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "Widgets/SBoxPanel.h"
// 解释：引入 `SOverlay.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "Widgets/SOverlay.h"
// 解释：引入 `STextBlock.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "Widgets/Text/STextBlock.h"
// 解释：引入 `STableRow.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "Widgets/Views/STableRow.h"
// 解释：引入 `CameraPawn.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "GraduationProject/Core/CameraPawn.h"
// 解释：引入 `AgentManager.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "GraduationProject/Core/Manager/AgentManager.h"
// 解释：引入 `DronePawn.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "GraduationProject/Drone/DronePawn.h"
// 解释：引入 `TurretPawn.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "GraduationProject/Turret/TurretPawn.h"

// ──── 匿名工具函数 ────
// 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
namespace
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    /** @brief 根据 Actor 类型返回字符串标签 */
    // 解释：这一行定义函数 `GetAgentType`，开始实现getagenttype的具体逻辑。
    FString GetAgentType(AActor* Agent)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (Cast<ADronePawn>(Agent))
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return TEXT("drone");
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (Cast<ATurretPawn>(Agent))
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return TEXT("turret");
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return TEXT("actor");
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    /** @brief Agent 排序优先级：drone_0 最高，其次 drone，再 turret，最后 actor */
    // 解释：这一行定义函数 `GetAgentPriority`，开始实现getagentpriority的具体逻辑。
    int32 GetAgentPriority(const UAgentListItemData* ItemData)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (!ItemData)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return 100;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (ItemData->AgentId.Equals(TEXT("drone_0"), ESearchCase::IgnoreCase))
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return 0;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (ItemData->AgentType == TEXT("drone"))
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return 10;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (ItemData->AgentId.Equals(TEXT("turret_0"), ESearchCase::IgnoreCase))
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return 20;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (ItemData->AgentType == TEXT("turret"))
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return 30;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return 50;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

// ──── 公开接口 ────

/** @brief 注入摄像机 Pawn 引用 */
// 解释：这一行定义函数 `SetCameraPawn`，开始实现set相机Pawn的具体逻辑。
void UAgentListWidget::SetCameraPawn(ACameraPawn* InCameraPawn)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `CameraPawn`，完成 相机Pawn 的更新。
    CameraPawn = InCameraPawn;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

// ──── Slate 构建 ────

/** @brief 构建控件树：半透明背景面板 → 标题 + 副标题 + Agent 列表 */
// 解释：这一行定义函数 `RebuildWidget`，开始实现rebuild控件的具体逻辑。
TSharedRef<SWidget> UAgentListWidget::RebuildWidget()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return SNew(SOverlay)
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        + SOverlay::Slot()
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        .HAlign(HAlign_Right)
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        .VAlign(VAlign_Top)
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        .Padding(FMargin(0.0f, 20.0f, 20.0f, 0.0f))
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        [
            // 解释：这一行位于构造函数初始化列表中，把 `SNew` 直接初始化为 `SBox`，减少进入函数体后的额外赋值开销。
            SNew(SBox)
            // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
            .WidthOverride(360.0f)
            // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
            .HeightOverride(320.0f)
            // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
            [
                // 解释：这一行位于构造函数初始化列表中，把 `SNew` 直接初始化为 `SBorder`，减少进入函数体后的额外赋值开销。
                SNew(SBorder)
                // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                .BorderBackgroundColor(FLinearColor(0.08f, 0.10f, 0.12f, 0.88f))
                // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                .Padding(10.0f)
                // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                [
                    // 解释：这一行位于构造函数初始化列表中，把 `SNew` 直接初始化为 `SVerticalBox`，减少进入函数体后的额外赋值开销。
                    SNew(SVerticalBox)
                    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                    + SVerticalBox::Slot()
                    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                    .AutoHeight()
                    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                    .Padding(5.0f)
                    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                    [
                        // 解释：这一行位于构造函数初始化列表中，把 `SNew` 直接初始化为 `STextBlock`，减少进入函数体后的额外赋值开销。
                        SNew(STextBlock)
                        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                        .Text(FText::FromString(TEXT("Agent Manager")))
                        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                        .Font(FCoreStyle::GetDefaultFontStyle("Bold", 14))
                    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                    ]
                    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                    + SVerticalBox::Slot()
                    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                    .AutoHeight()
                    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                    .Padding(FMargin(5.0f, 0.0f, 5.0f, 8.0f))
                    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                    [
                        // 解释：这一行位于构造函数初始化列表中，把 `SNew` 直接初始化为 `STextBlock`，减少进入函数体后的额外赋值开销。
                        SNew(STextBlock)
                        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                        .Text(FText::FromString(TEXT("Select an agent to focus camera")))
                        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                        .ColorAndOpacity(FSlateColor(FLinearColor(0.75f, 0.78f, 0.82f, 1.0f)))
                    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                    ]
                    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                    + SVerticalBox::Slot()
                    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                    .FillHeight(1.0f)
                    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                    [
                        // 解释：这一行位于构造函数初始化列表中，把 `SAssignNew` 直接初始化为 `AgentListView, SListView<UObject*>`，减少进入函数体后的额外赋值开销。
                        SAssignNew(AgentListView, SListView<UObject*>)
                        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                        .ListItemsSource(&AgentItems)
                        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                        .OnGenerateRow_UObject(this, &UAgentListWidget::BuildAgentRow)
                        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                        .OnSelectionChanged_UObject(this, &UAgentListWidget::HandleAgentSelected)
                        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                        .SelectionMode(ESelectionMode::Single)
                    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                    ]
                // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                ]
            // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
            ]
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        ];
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

// ──── 生命周期 ────

/** @brief 订阅 AgentManager 列表变更事件，并立即加载一次 */
// 解释：这一行定义函数 `NativeConstruct`，开始实现nativeconstruct的具体逻辑。
void UAgentListWidget::NativeConstruct()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：调用 `NativeConstruct` 执行当前步骤需要的功能逻辑。
    Super::NativeConstruct();

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (UAgentManager* Manager = UAgentManager::GetInstance())
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `AddUObject` 执行当前步骤需要的功能逻辑。
        Manager->OnAgentListChanged.AddUObject(this, &UAgentListWidget::ReloadAgentItems);  // 监听 Agent 增删
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：调用 `ReloadAgentItems` 执行当前步骤需要的功能逻辑。
    ReloadAgentItems();
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 取消事件订阅，避免悬挂引用 */
// 解释：这一行定义函数 `NativeDestruct`，开始实现nativedestruct的具体逻辑。
void UAgentListWidget::NativeDestruct()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (UAgentManager* Manager = UAgentManager::GetInstance())
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `RemoveAll` 执行当前步骤需要的功能逻辑。
        Manager->OnAgentListChanged.RemoveAll(this);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：调用 `NativeDestruct` 执行当前步骤需要的功能逻辑。
    Super::NativeDestruct();
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

// ──── 数据管理 ────

/** @brief 按 GetAgentPriority 排序列表项，同优先级按 ID 字典序 */
// 解释：这一行定义函数 `SortAgentItems`，开始实现sortagentitems的具体逻辑。
void UAgentListWidget::SortAgentItems()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    AgentItems.Sort([](const UObject& LeftItem, const UObject& RightItem)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `const UAgentListItemData* LeftData`，完成 constuagent列表项dataleftdata 的更新。
        const UAgentListItemData* LeftData = Cast<UAgentListItemData>(&LeftItem);
        // 解释：这一行把右侧表达式的结果写入 `const UAgentListItemData* RightData`，完成 constuagent列表项datarightdata 的更新。
        const UAgentListItemData* RightData = Cast<UAgentListItemData>(&RightItem);

        // 解释：这一行把右侧表达式的结果写入 `const int32 LeftPriority`，完成 constint32leftpriority 的更新。
        const int32 LeftPriority = GetAgentPriority(LeftData);
        // 解释：这一行把右侧表达式的结果写入 `const int32 RightPriority`，完成 constint32rightpriority 的更新。
        const int32 RightPriority = GetAgentPriority(RightData);
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (LeftPriority != RightPriority)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return LeftPriority < RightPriority;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行把右侧表达式的结果写入 `const FString LeftId`，完成 constfstringleftid 的更新。
        const FString LeftId = LeftData ? LeftData->AgentId : TEXT("");
        // 解释：这一行把右侧表达式的结果写入 `const FString RightId`，完成 constfstringrightid 的更新。
        const FString RightId = RightData ? RightData->AgentId : TEXT("");
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return LeftId < RightId;
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    });
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 增量刷新 Agent 列表
 *
 * 1) 用 Actor 指针做映射索引已有条目
 * 2) 遍历最新 ID，复用或新建条目
 * 3) 移除已不存在的条目
 * 4) 有变更时重新排序并刷新 UI
 */
// 解释：这一行定义函数 `ReloadAgentItems`，开始实现reloadagentitems的具体逻辑。
void UAgentListWidget::ReloadAgentItems()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `UAgentManager* Manager`，完成 uagent管理器管理器 的更新。
    UAgentManager* Manager = UAgentManager::GetInstance();
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!Manager)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：调用 `GetAllAgentIds` 执行当前步骤需要的功能逻辑。
    const TArray<FString> LatestIds = Manager->GetAllAgentIds();  // 当前所有 Agent ID
    // 解释：这一行声明成员或局部变量 `ExistingItemsByActor`，用于保存existingitemsbyActor。
    TMap<AActor*, UAgentListItemData*> ExistingItemsByActor;       // Actor→已有条目 映射
    // 解释：调用 `Reserve` 执行当前步骤需要的功能逻辑。
    ExistingItemsByActor.Reserve(AgentItems.Num());

    // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
    for (UObject* Item : AgentItems)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `UAgentListItemData* ItemData`，完成 uagent列表项data项data 的更新。
        UAgentListItemData* ItemData = Cast<UAgentListItemData>(Item);
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (!ItemData)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行跳过本轮循环剩余语句，直接进入下一轮迭代。
            continue;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (AActor* Agent = ItemData->AgentActor.Get())
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：调用 `Add` 执行当前步骤需要的功能逻辑。
            ExistingItemsByActor.Add(Agent, ItemData);
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行声明成员或局部变量 `LatestActors`，用于保存latestactors。
    TSet<AActor*> LatestActors;   // 当前帧存在的 Actor 集合
    // 解释：这一行声明成员或局部变量 `bChanged`，用于保存布尔标志 changed。
    bool bChanged = false;         // 是否有增删改

    // ── 第二步：遍历最新 Agent，复用或创建条目 ──
    // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
    for (const FString& AgentId : LatestIds)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `AActor* Agent`，完成 aactoragent 的更新。
        AActor* Agent = Manager->GetAgent(AgentId);
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (!IsValid(Agent))
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行跳过本轮循环剩余语句，直接进入下一轮迭代。
            continue;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：调用 `Add` 执行当前步骤需要的功能逻辑。
        LatestActors.Add(Agent);

        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (UAgentListItemData** FoundItem = ExistingItemsByActor.Find(Agent))
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行声明成员或局部变量 `ItemData`，用于保存项data。
            UAgentListItemData* ItemData = *FoundItem;
            // 解释：这一行把右侧表达式的结果写入 `const FString NewType`，完成 constfstringnewtype 的更新。
            const FString NewType = GetAgentType(Agent);
            // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
            if (ItemData->AgentId != AgentId || ItemData->AgentType != NewType)
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            {
                // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                ItemData->AgentId = AgentId;
                // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                ItemData->AgentType = NewType;
                // 解释：这一行把右侧表达式的结果写入 `bChanged`，完成 布尔标志 changed 的更新。
                bChanged = true;
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            }
            // 解释：这一行跳过本轮循环剩余语句，直接进入下一轮迭代。
            continue;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行把右侧表达式的结果写入 `UAgentListItemData* NewItem`，完成 uagent列表项datanew项 的更新。
        UAgentListItemData* NewItem = NewObject<UAgentListItemData>(this);
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        NewItem->AgentId = AgentId;
        // 解释：调用 `GetAgentType` 执行当前步骤需要的功能逻辑。
        NewItem->AgentType = GetAgentType(Agent);
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        NewItem->AgentActor = Agent;
        // 解释：调用 `Add` 执行当前步骤需要的功能逻辑。
        AgentItems.Add(NewItem);
        // 解释：这一行把右侧表达式的结果写入 `bChanged`，完成 布尔标志 changed 的更新。
        bChanged = true;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // ── 第三步：反向遍历移除已失效条目 ──
    // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
    for (int32 Index = AgentItems.Num() - 1; Index >= 0; --Index)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `UAgentListItemData* ItemData`，完成 uagent列表项data项data 的更新。
        UAgentListItemData* ItemData = Cast<UAgentListItemData>(AgentItems[Index]);
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (!ItemData)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：调用 `RemoveAt` 执行当前步骤需要的功能逻辑。
            AgentItems.RemoveAt(Index);
            // 解释：这一行把右侧表达式的结果写入 `bChanged`，完成 布尔标志 changed 的更新。
            bChanged = true;
            // 解释：这一行跳过本轮循环剩余语句，直接进入下一轮迭代。
            continue;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行把右侧表达式的结果写入 `AActor* Agent`，完成 aactoragent 的更新。
        AActor* Agent = ItemData->AgentActor.Get();
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (!IsValid(Agent) || !LatestActors.Contains(Agent))
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：调用 `RemoveAt` 执行当前步骤需要的功能逻辑。
            AgentItems.RemoveAt(Index);
            // 解释：这一行把右侧表达式的结果写入 `bChanged`，完成 布尔标志 changed 的更新。
            bChanged = true;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // ── 第四步：有变更则排序并通知列表刷新 ──
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (bChanged)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `SortAgentItems` 执行当前步骤需要的功能逻辑。
        SortAgentItems();
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (AgentListView.IsValid())
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：调用 `RequestListRefresh` 执行当前步骤需要的功能逻辑。
            AgentListView->RequestListRefresh();
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

// ──── 行构建与选择回调 ────

/** @brief 为每行创建 "[type] agent_id" 格式的文本控件 */
// 解释：这一行定义函数 `BuildAgentRow`，开始实现buildagentrow的具体逻辑。
TSharedRef<ITableRow> UAgentListWidget::BuildAgentRow(UObject* Item, const TSharedRef<STableViewBase>& OwnerTable)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `const UAgentListItemData* ItemData`，完成 constuagent列表项data项data 的更新。
    const UAgentListItemData* ItemData = Cast<UAgentListItemData>(Item);
    // 解释：这一行声明成员或局部变量 `Label`，用于保存label。
    const FString Label = ItemData
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        ? FString::Printf(TEXT("[%s] %s"), *ItemData->AgentType, *ItemData->AgentId)
        // 解释：这一行开始构造函数初始化列表，下面会依次初始化成员变量。
        : TEXT("<invalid>");

    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return SNew(STableRow<UObject*>, OwnerTable)
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        [
            // 解释：这一行位于构造函数初始化列表中，把 `SNew` 直接初始化为 `STextBlock`，减少进入函数体后的额外赋值开销。
            SNew(STextBlock)
            // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
            .Text(FText::FromString(Label))
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        ];
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 用户选中行时通知 CameraPawn 跟随该 Agent */
// 解释：这一行定义函数 `HandleAgentSelected`，开始实现handleagentselected的具体逻辑。
void UAgentListWidget::HandleAgentSelected(UObject* Item, ESelectInfo::Type SelectInfo)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `ACameraPawn* Pawn`，完成 acameraPawnPawn 的更新。
    ACameraPawn* Pawn = CameraPawn.Get();
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!Pawn)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行把右侧表达式的结果写入 `UAgentListItemData* ItemData`，完成 uagent列表项data项data 的更新。
    UAgentListItemData* ItemData = Cast<UAgentListItemData>(Item);
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!ItemData)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `OnItemClicked` 执行当前步骤需要的功能逻辑。
        Pawn->OnItemClicked(TEXT(""), nullptr);
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：调用 `OnItemClicked` 执行当前步骤需要的功能逻辑。
    Pawn->OnItemClicked(ItemData->AgentId, ItemData->AgentActor.Get());
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

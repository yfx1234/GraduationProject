#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "Widgets/Views/SListView.h"
#include "AgentListWidget.generated.h"

class ACameraPawn;

/**
 * @brief Agent 列表 Slate 控件
 *
 * 在屏幕右上角显示所有已注册的 Agent，
 * 点击某行后驱动 CameraPawn 跟随所选 Agent。
 */
UCLASS()
class GRADUATIONPROJECT_API UAgentListWidget : public UUserWidget
{
    GENERATED_BODY()

public:
    /** @brief 设置摄像机 Pawn 引用（用于点击后跟随） */
    void SetCameraPawn(ACameraPawn* InCameraPawn);

protected:
    /** @brief 构建 Slate 控件树 */
    virtual TSharedRef<SWidget> RebuildWidget() override;
    /** @brief 控件初始化：订阅 AgentManager 变更事件 */
    virtual void NativeConstruct() override;
    /** @brief 控件销毁：取消事件订阅 */
    virtual void NativeDestruct() override;

private:
    /** @brief 从 AgentManager 重新加载 Agent 列表（增量更新） */
    void ReloadAgentItems();
    /** @brief 按类型优先级排序 Agent 列表项 */
    void SortAgentItems();
    /** @brief 为列表中每一行创建 Slate 控件 */
    TSharedRef<ITableRow> BuildAgentRow(UObject* Item, const TSharedRef<STableViewBase>& OwnerTable);
    /** @brief 用户选中某行时的回调，通知 CameraPawn 跟随 */
    void HandleAgentSelected(UObject* Item, ESelectInfo::Type SelectInfo);

private:
    UPROPERTY(Transient)
    TArray<UObject*> AgentItems;                        ///< 列表数据源

    TSharedPtr<SListView<UObject*>> AgentListView;      ///< Slate 列表控件
    TWeakObjectPtr<ACameraPawn> CameraPawn;              ///< 关联的摄像机 Pawn
};

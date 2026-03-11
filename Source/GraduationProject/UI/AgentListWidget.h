#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "Widgets/Views/SListView.h"
#include "AgentListWidget.generated.h"

class ACameraPawn;

/**
 * @brief 智能体列表控件
 * 以 Slate 列表方式展示当前注册的 Agent，并将选中结果同步给观察相机。
 */
UCLASS()
class GRADUATIONPROJECT_API UAgentListWidget : public UUserWidget
{
    GENERATED_BODY()

public:
    /**
     * @brief 绑定相机 Pawn
     * @param InCameraPawn 用于响应列表点击的相机实例
     */
    void SetCameraPawn(ACameraPawn* InCameraPawn);

protected:
    /** @brief 重建底层 Slate 控件树 */
    virtual TSharedRef<SWidget> RebuildWidget() override;

    /** @brief 注册数据刷新回调并初始化列表内容 */
    virtual void NativeConstruct() override;

    /** @brief 解除回调绑定并清理控件状态 */
    virtual void NativeDestruct() override;

private:
    /** @brief 刷新列表数据源 */
    void RefreshAgentList();

    /** @brief 对列表项按类型和 ID 排序 */
    void SortItems();

    /** @brief 为列表项生成一行 Slate 视图 */
    TSharedRef<ITableRow> HandleGenerateRow(UObject* Item, const TSharedRef<STableViewBase>& OwnerTable);

    /** @brief 处理用户选中某个列表项后的相机切换 */
    void HandleSelectionChanged(UObject* Item, ESelectInfo::Type SelectInfo);

private:
    /** @brief 列表项数据缓存 */
    UPROPERTY(Transient)
    TArray<UObject*> ListDataItems;

    /** @brief Slate 列表视图实例 */
    TSharedPtr<SListView<UObject*>> AgentListView;

    /** @brief 与列表联动的相机 Pawn */
    TWeakObjectPtr<ACameraPawn> CameraPawn;
};

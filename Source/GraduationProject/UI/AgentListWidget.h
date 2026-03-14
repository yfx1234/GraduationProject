// 解释：使用 `#pragma once` 防止该头文件在编译过程中被重复包含。
#pragma once

// 解释：引入 Unreal 的核心基础头文件，提供常用容器、数学类型和日志宏。
#include "CoreMinimal.h"
// 解释：引入 `UserWidget.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "Blueprint/UserWidget.h"
// 解释：引入 `SListView.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "Widgets/Views/SListView.h"
// 解释：引入 `AgentListWidget.generated.h`，让 Unreal Header Tool 生成的反射代码在本文件中可见。
#include "AgentListWidget.generated.h"

// 解释：这一行声明 类 `ACameraPawn`，用于封装acameraPawn相关的数据与行为。
class ACameraPawn;

/**
 * @brief Agent 列表 Slate 控件
 *
 * 在屏幕右上角显示所有已注册的 Agent，
 * 点击某行后驱动 CameraPawn 跟随所选 Agent。
 */
// 解释：使用 `UCLASS` 宏把当前类型注册到 Unreal 反射系统中，便于蓝图、序列化和编辑器识别。
UCLASS()
// 解释：这一行声明 类 `UAgentListWidget`，用于封装uagent列表控件相关的数据与行为。
class GRADUATIONPROJECT_API UAgentListWidget : public UUserWidget
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：展开 Unreal Header Tool 生成的样板代码，这是 UObject、Actor、组件等类型正常工作的基础。
    GENERATED_BODY()

// 解释：从这一行开始进入 `public` 访问区，下面的成员会对外部模块公开。
public:
    /** @brief 设置摄像机 Pawn 引用（用于点击后跟随） */
    // 解释：调用 `SetCameraPawn` 执行当前步骤需要的功能逻辑。
    void SetCameraPawn(ACameraPawn* InCameraPawn);

// 解释：从这一行开始进入 `protected` 访问区，下面的成员只对本类及其派生类可见。
protected:
    /** @brief 构建 Slate 控件树 */
    // 解释：调用 `RebuildWidget` 执行当前步骤需要的功能逻辑。
    virtual TSharedRef<SWidget> RebuildWidget() override;
    /** @brief 控件初始化：订阅 AgentManager 变更事件 */
    // 解释：调用 `NativeConstruct` 执行当前步骤需要的功能逻辑。
    virtual void NativeConstruct() override;
    /** @brief 控件销毁：取消事件订阅 */
    // 解释：调用 `NativeDestruct` 执行当前步骤需要的功能逻辑。
    virtual void NativeDestruct() override;

// 解释：从这一行开始进入 `private` 访问区，下面的成员只允许类内部访问。
private:
    /** @brief 从 AgentManager 重新加载 Agent 列表（增量更新） */
    // 解释：调用 `ReloadAgentItems` 执行当前步骤需要的功能逻辑。
    void ReloadAgentItems();
    /** @brief 按类型优先级排序 Agent 列表项 */
    // 解释：调用 `SortAgentItems` 执行当前步骤需要的功能逻辑。
    void SortAgentItems();
    /** @brief 为列表中每一行创建 Slate 控件 */
    // 解释：调用 `BuildAgentRow` 执行当前步骤需要的功能逻辑。
    TSharedRef<ITableRow> BuildAgentRow(UObject* Item, const TSharedRef<STableViewBase>& OwnerTable);
    /** @brief 用户选中某行时的回调，通知 CameraPawn 跟随 */
    // 解释：调用 `HandleAgentSelected` 执行当前步骤需要的功能逻辑。
    void HandleAgentSelected(UObject* Item, ESelectInfo::Type SelectInfo);

// 解释：从这一行开始进入 `private` 访问区，下面的成员只允许类内部访问。
private:
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(Transient)
    // 解释：这一行声明成员或局部变量 `AgentItems`，用于保存agentitems。
    TArray<UObject*> AgentItems;                        ///< 列表数据源

    // 解释：这一行声明成员或局部变量 `AgentListView`，用于保存agent列表view。
    TSharedPtr<SListView<UObject*>> AgentListView;      ///< Slate 列表控件
    // 解释：这一行声明成员或局部变量 `CameraPawn`，用于保存相机Pawn。
    TWeakObjectPtr<ACameraPawn> CameraPawn;              ///< 关联的摄像机 Pawn
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
};

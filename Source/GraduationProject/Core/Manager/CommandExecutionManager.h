// 解释：使用 `#pragma once` 防止该头文件在编译过程中被重复包含。
#pragma once

// 解释：引入 Unreal 的核心基础头文件，提供常用容器、数学类型和日志宏。
#include "CoreMinimal.h"
// 解释：引入 `NoExportTypes.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "UObject/NoExportTypes.h"
// 解释：引入 `CommandExecutionManager.generated.h`，让 Unreal Header Tool 生成的反射代码在本文件中可见。
#include "CommandExecutionManager.generated.h"

/**
 * @brief 单条异步命令执行记录
 * 记录命令所属智能体、调用函数、开始/结束时间以及当前状态
 */
// 解释：使用 `USTRUCT` 宏声明可被 Unreal 反射系统识别的结构体类型。
USTRUCT(BlueprintType)
// 解释：这一行声明 结构体 `FCommandExecutionRecord`，用于封装fcommand执行记录相关的数据与行为。
struct FCommandExecutionRecord
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：展开 Unreal Header Tool 生成的样板代码，这是 UObject、Actor、组件等类型正常工作的基础。
    GENERATED_BODY()

    /** @brief 命令唯一 ID */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(BlueprintReadOnly)
    // 解释：这一行声明成员或局部变量 `CommandId`，用于保存命令id。
    FString CommandId;

    /** @brief 发起命令的智能体 ID */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(BlueprintReadOnly)
    // 解释：这一行声明成员或局部变量 `AgentId`，用于保存agentid。
    FString AgentId;

    /** @brief 被调用的函数名 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(BlueprintReadOnly)
    // 解释：这一行声明成员或局部变量 `FunctionName`，用于保存functionname。
    FString FunctionName;

    /** @brief 当前状态 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(BlueprintReadOnly)
    // 解释：这一行把右侧表达式的结果写入 `FString Status`，完成 fstringstatus 的更新。
    FString Status = TEXT("queued");

    /** @brief 附加说明信息 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(BlueprintReadOnly)
    // 解释：这一行声明成员或局部变量 `Message`，用于保存message。
    FString Message;

    /** @brief 命令开始执行时的时间（秒） */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(BlueprintReadOnly)
    // 解释：这一行声明成员或局部变量 `StartTimeSec`，用于保存starttimesec。
    double StartTimeSec = 0.0;

    /** @brief 命令结束执行时的时间（秒） */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(BlueprintReadOnly)
    // 解释：这一行声明成员或局部变量 `EndTimeSec`，用于保存endtimesec。
    double EndTimeSec = 0.0;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
};

/** @brief 命令执行生命周期管理器 */
// 解释：使用 `UCLASS` 宏把当前类型注册到 Unreal 反射系统中，便于蓝图、序列化和编辑器识别。
UCLASS()
// 解释：这一行声明 类 `UCommandExecutionManager`，用于封装ucommand执行管理器相关的数据与行为。
class GRADUATIONPROJECT_API UCommandExecutionManager : public UObject
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：展开 Unreal Header Tool 生成的样板代码，这是 UObject、Actor、组件等类型正常工作的基础。
    GENERATED_BODY()

// 解释：从这一行开始进入 `public` 访问区，下面的成员会对外部模块公开。
public:
    /** @brief 获取全局单例 */
    // 解释：调用 `GetInstance` 执行当前步骤需要的功能逻辑。
    static UCommandExecutionManager* GetInstance();

    /** @brief 释放全局单例并清空命令记录 */
    // 解释：调用 `Cleanup` 执行当前步骤需要的功能逻辑。
    static void Cleanup();

    /**
     * @brief 注册一条新命令并进入运行中状态
     * @param AgentId 发起命令的智能体 ID
     * @param FunctionName 被调用函数名
     * @return 新生成的命令 ID
     */
    // 解释：调用 `StartCommand` 执行当前步骤需要的功能逻辑。
    FString StartCommand(const FString& AgentId, const FString& FunctionName);

    /**
     * @brief 标记命令执行完成
     * @param CommandId 命令 ID
     * @param bSuccess 是否执行成功
     * @param Message 附加说明或错误信息
     */
    // 解释：调用 `CompleteCommand` 执行当前步骤需要的功能逻辑。
    void CompleteCommand(const FString& CommandId, bool bSuccess, const FString& Message);

    /**
     * @brief 查询指定命令记录
     * @param CommandId 命令 ID
     * @param OutRecord 输出命令记录
     * @return 找到命令时返回 true
     */
    // 解释：调用 `GetCommand` 执行当前步骤需要的功能逻辑。
    bool GetCommand(const FString& CommandId, FCommandExecutionRecord& OutRecord) const;

    /**
     * @brief 取消尚未完成的命令
     * @param CommandId 命令 ID
     * @param Reason 取消原因
     * @return 命令可取消并已更新状态时返回 true
     */
    // 解释：这一行把右侧表达式的结果写入 `bool CancelCommand(const FString& CommandId, const FString& Reason`，完成 boolcancel命令constfstring命令idconstfstringreason 的更新。
    bool CancelCommand(const FString& CommandId, const FString& Reason = TEXT("canceled"));

    /**
     * @brief 查询命令状态，并在需要时执行超时判定
     * @param CommandId 命令 ID
     * @param TimeoutSec 超时时间阈值，非正数表示不检查
     * @param OutRecord 输出命令记录
     * @return 找到命令时返回 true
     */
    // 解释：调用 `GetCommandWithTimeout` 执行当前步骤需要的功能逻辑。
    bool GetCommandWithTimeout(const FString& CommandId, double TimeoutSec, FCommandExecutionRecord& OutRecord);

// 解释：从这一行开始进入 `private` 访问区，下面的成员只允许类内部访问。
private:
    /** @brief 全局单例指针 */
    // 解释：这一行声明成员或局部变量 `Instance`，用于保存instance。
    static UCommandExecutionManager* Instance;

    /** @brief 以命令 ID 为键的执行记录表 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY()
    // 解释：这一行声明成员或局部变量 `Records`，用于保存records。
    TMap<FString, FCommandExecutionRecord> Records;

    /** @brief 单调递增的命令序号生成器 */
    // 解释：这一行声明成员或局部变量 `Counter`，用于保存counter。
    int64 Counter = 0;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
};

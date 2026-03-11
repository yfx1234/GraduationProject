#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "CommandExecutionManager.generated.h"

/**
 * @brief 单条异步命令执行记录
 * 记录命令所属智能体、调用函数、开始/结束时间以及当前状态，
 * 供状态查询、超时检测和录制系统复用。
 */
USTRUCT(BlueprintType)
struct FCommandExecutionRecord
{
    GENERATED_BODY()

    /** @brief 命令唯一 ID，例如 `cmd_1` */
    UPROPERTY(BlueprintReadOnly)
    FString CommandId;

    /** @brief 发起命令的智能体 ID */
    UPROPERTY(BlueprintReadOnly)
    FString AgentId;

    /** @brief 被调用的函数名 */
    UPROPERTY(BlueprintReadOnly)
    FString FunctionName;

    /** @brief 当前状态：`queued`、`running`、`completed`、`failed`、`canceled` */
    UPROPERTY(BlueprintReadOnly)
    FString Status = TEXT("queued");

    /** @brief 附加说明信息，例如错误原因或完成提示 */
    UPROPERTY(BlueprintReadOnly)
    FString Message;

    /** @brief 命令开始执行时的墙钟时间（秒） */
    UPROPERTY(BlueprintReadOnly)
    double StartTimeSec = 0.0;

    /** @brief 命令结束执行时的墙钟时间（秒） */
    UPROPERTY(BlueprintReadOnly)
    double EndTimeSec = 0.0;
};

/**
 * @brief 命令执行生命周期管理器
 * 负责为异步命令生成 ID、维护运行状态、处理取消/超时逻辑，
 * 并将关键事件写入 `SimulationRecorder` 便于回放与调试。
 */
UCLASS()
class GRADUATIONPROJECT_API UCommandExecutionManager : public UObject
{
    GENERATED_BODY()

public:
    /** @brief 获取全局单例 */
    static UCommandExecutionManager* GetInstance();

    /** @brief 释放全局单例并清空命令记录 */
    static void Cleanup();

    /**
     * @brief 注册一条新命令并进入运行中状态
     * @param AgentId 发起命令的智能体 ID
     * @param FunctionName 被调用函数名
     * @return 新生成的命令 ID
     */
    FString StartCommand(const FString& AgentId, const FString& FunctionName);

    /**
     * @brief 标记命令执行完成
     * @param CommandId 命令 ID
     * @param bSuccess 是否执行成功
     * @param Message 附加说明或错误信息
     */
    void CompleteCommand(const FString& CommandId, bool bSuccess, const FString& Message);

    /**
     * @brief 查询指定命令记录
     * @param CommandId 命令 ID
     * @param OutRecord 输出命令记录
     * @return 找到命令时返回 `true`
     */
    bool GetCommand(const FString& CommandId, FCommandExecutionRecord& OutRecord) const;

    /**
     * @brief 取消尚未完成的命令
     * @param CommandId 命令 ID
     * @param Reason 取消原因
     * @return 命令可取消并已更新状态时返回 `true`
     */
    bool CancelCommand(const FString& CommandId, const FString& Reason = TEXT("canceled"));

    /**
     * @brief 查询命令状态，并在需要时执行超时判定
     * @param CommandId 命令 ID
     * @param TimeoutSec 超时时间阈值，非正数表示不检查
     * @param OutRecord 输出命令记录
     * @return 找到命令时返回 `true`
     */
    bool GetCommandWithTimeout(const FString& CommandId, double TimeoutSec, FCommandExecutionRecord& OutRecord);

private:
    /** @brief 全局单例指针 */
    static UCommandExecutionManager* Instance;

    /** @brief 以命令 ID 为键的执行记录表 */
    UPROPERTY()
    TMap<FString, FCommandExecutionRecord> Records;

    /** @brief 单调递增的命令序号生成器 */
    int64 Counter = 0;
};
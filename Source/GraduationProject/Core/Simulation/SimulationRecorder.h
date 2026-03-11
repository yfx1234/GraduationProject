#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "SimulationRecorder.generated.h"

/**
 * @brief 仿真 JSONL 录制器
 * 按行写入结构化事件，方便后处理、回放分析和实验留档。
 */
UCLASS()
class GRADUATIONPROJECT_API USimulationRecorder : public UObject
{
    GENERATED_BODY()

public:
    /** @brief 获取全局单例 */
    static USimulationRecorder* GetInstance();

    /** @brief 释放全局单例并停止录制 */
    static void Cleanup();

    /**
     * @brief 开始录制
     * @param InPath 输出文件路径；为空时自动生成到 `Saved/SimRecords`
     * @return 文件创建成功时返回 `true`
     */
    bool Start(const FString& InPath = TEXT(""));

    /** @brief 停止录制，不删除已写入文件 */
    void Stop();

    /** @brief 当前是否正在录制 */
    bool IsRecording() const { return bRecording; }

    /** @brief 获取当前录制文件路径 */
    FString GetPath() const { return RecordPath; }

    /** @brief 获取当前已追加的事件数量 */
    int64 GetRecordCount() const { return RecordCount; }

    /**
     * @brief 追加一条 JSONL 事件
     * @param Type 事件类型
     * @param JsonPayload 事件负载 JSON，要求本身是合法 JSON 片段
     */
    void RecordJsonLine(const FString& Type, const FString& JsonPayload);

private:
    /** @brief 全局单例指针 */
    static USimulationRecorder* Instance;

    /** @brief 录制状态标记 */
    bool bRecording = false;

    /** @brief 当前录制文件路径 */
    FString RecordPath;

    /** @brief 已写入的事件条数 */
    int64 RecordCount = 0;
};
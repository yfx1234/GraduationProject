#include "SimulationRecorder.h"

#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"

USimulationRecorder* USimulationRecorder::Instance = nullptr;

/**
 * @brief 获取录制器单例
 * 若不存在则创建对象并加入 Root，保证录制期间不会被垃圾回收。
 */
USimulationRecorder* USimulationRecorder::GetInstance()
{
    if (!Instance)
    {
        Instance = NewObject<USimulationRecorder>();
        Instance->AddToRoot();
    }
    return Instance;
}

/**
 * @brief 释放录制器单例
 * 销毁前会先调用 Stop，确保状态复位。
 */
void USimulationRecorder::Cleanup()
{
    if (Instance)
    {
        Instance->Stop();
        Instance->RemoveFromRoot();
        Instance = nullptr;
    }
}

/**
 * @brief 开始录制 JSONL 文件
 * @param InPath 输出文件路径
 * @return 文件创建成功时返回 `true`
 * 若未指定路径，则自动在 `Saved/SimRecords` 下生成带时间戳的文件名。
 */
bool USimulationRecorder::Start(const FString& InPath)
{
    FString FinalPath = InPath;
    if (FinalPath.IsEmpty())
    {
        const FString Dir = FPaths::ProjectSavedDir() / TEXT("SimRecords");
        IFileManager::Get().MakeDirectory(*Dir, true);
        FinalPath = Dir / FString::Printf(TEXT("record_%s.jsonl"), *FDateTime::Now().ToString(TEXT("%Y%m%d_%H%M%S")));
    }

    // 先写入一条元信息头，保证文件一创建就可以作为合法 JSONL 流被后处理脚本消费。
    const FString Header = FString::Printf(TEXT("{\"type\":\"meta\",\"started_at\":\"%s\"}\n"), *FDateTime::UtcNow().ToIso8601());
    if (!FFileHelper::SaveStringToFile(Header, *FinalPath, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM, &IFileManager::Get(), EFileWrite::FILEWRITE_None))
    {
        return false;
    }

    bRecording = true;
    RecordPath = FinalPath;
    RecordCount = 0;
    return true;
}

/** @brief 停止录制，仅更新内部状态，不删除现有文件 */
void USimulationRecorder::Stop()
{
    bRecording = false;
}

/**
 * @brief 追加一条事件到 JSONL 文件
 * @param Type 事件类型
 * @param JsonPayload 事件负载 JSON
 * 每条记录都会补充 UTC 时间戳，方便回放时按事件顺序重建过程。
 */
void USimulationRecorder::RecordJsonLine(const FString& Type, const FString& JsonPayload)
{
    if (!bRecording || RecordPath.IsEmpty())
    {
        return;
    }

    const FString Line = FString::Printf(
        TEXT("{\"type\":\"%s\",\"time\":\"%s\",\"payload\":%s}\n"),
        *Type,
        *FDateTime::UtcNow().ToIso8601(),
        *JsonPayload);

    FFileHelper::SaveStringToFile(Line, *RecordPath, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM, &IFileManager::Get(), EFileWrite::FILEWRITE_Append);
    ++RecordCount;
}
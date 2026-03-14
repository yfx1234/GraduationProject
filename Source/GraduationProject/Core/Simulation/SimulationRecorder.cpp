// 解释：引入当前实现文件对应的头文件 `SimulationRecorder.h`，使实现部分能够看到类和函数声明。
#include "SimulationRecorder.h"

// 解释：引入 `FileManager.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "HAL/FileManager.h"
// 解释：引入 `FileHelper.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "Misc/FileHelper.h"
// 解释：引入 `Paths.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "Misc/Paths.h"

// 解释：这一行把右侧表达式的结果写入 `USimulationRecorder* USimulationRecorder::Instance`，完成 usimulation记录器usimulation记录器instance 的更新。
USimulationRecorder* USimulationRecorder::Instance = nullptr;

/**
 * @brief 获取录制器单例
 * 若不存在则创建对象并加入 Root，保证录制期间不会被垃圾回收。
 */
// 解释：这一行定义函数 `GetInstance`，开始实现getinstance的具体逻辑。
USimulationRecorder* USimulationRecorder::GetInstance()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!Instance)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `Instance`，完成 instance 的更新。
        Instance = NewObject<USimulationRecorder>();
        // 解释：调用 `AddToRoot` 执行当前步骤需要的功能逻辑。
        Instance->AddToRoot();
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return Instance;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 释放录制器单例
 * 销毁前会先调用 Stop，确保状态复位。
 */
// 解释：这一行定义函数 `Cleanup`，开始实现cleanup的具体逻辑。
void USimulationRecorder::Cleanup()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (Instance)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `Stop` 执行当前步骤需要的功能逻辑。
        Instance->Stop();
        // 解释：调用 `RemoveFromRoot` 执行当前步骤需要的功能逻辑。
        Instance->RemoveFromRoot();
        // 解释：这一行把右侧表达式的结果写入 `Instance`，完成 instance 的更新。
        Instance = nullptr;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 开始录制 JSONL 文件
 * @param InPath 输出文件路径
 * @return 文件创建成功时返回 `true`
 * 若未指定路径，则自动在 `Saved/SimRecords` 下生成带时间戳的文件名。
 */
// 解释：这一行定义函数 `Start`，开始实现start的具体逻辑。
bool USimulationRecorder::Start(const FString& InPath)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行声明成员或局部变量 `FinalPath`，用于保存finalpath。
    FString FinalPath = InPath;
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (FinalPath.IsEmpty())
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `const FString Dir`，完成 constfstringdir 的更新。
        const FString Dir = FPaths::ProjectSavedDir() / TEXT("SimRecords");
        // 解释：调用 `Get` 执行当前步骤需要的功能逻辑。
        IFileManager::Get().MakeDirectory(*Dir, true);
        // 解释：这一行把右侧表达式的结果写入 `FinalPath`，完成 finalpath 的更新。
        FinalPath = Dir / FString::Printf(TEXT("record_%s.jsonl"), *FDateTime::Now().ToString(TEXT("%Y%m%d_%H%M%S")));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 先写入一条元信息头，保证文件一创建就可以作为合法 JSONL 流被后处理脚本消费。
    // 解释：这一行把右侧表达式的结果写入 `const FString Header`，完成 constfstringheader 的更新。
    const FString Header = FString::Printf(TEXT("{\"type\":\"meta\",\"started_at\":\"%s\"}\n"), *FDateTime::UtcNow().ToIso8601());
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!FFileHelper::SaveStringToFile(Header, *FinalPath, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM, &IFileManager::Get(), EFileWrite::FILEWRITE_None))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return false;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行把右侧表达式的结果写入 `bRecording`，完成 布尔标志 recording 的更新。
    bRecording = true;
    // 解释：这一行把右侧表达式的结果写入 `RecordPath`，完成 记录path 的更新。
    RecordPath = FinalPath;
    // 解释：这一行把右侧表达式的结果写入 `RecordCount`，完成 记录count 的更新。
    RecordCount = 0;
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return true;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 停止录制，仅更新内部状态，不删除现有文件 */
// 解释：这一行定义函数 `Stop`，开始实现stop的具体逻辑。
void USimulationRecorder::Stop()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `bRecording`，完成 布尔标志 recording 的更新。
    bRecording = false;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 追加一条事件到 JSONL 文件
 * @param Type 事件类型
 * @param JsonPayload 事件负载 JSON
 * 每条记录都会补充 UTC 时间戳，方便回放时按事件顺序重建过程。
 */
// 解释：这一行定义函数 `RecordJsonLine`，开始实现记录jsonline的具体逻辑。
void USimulationRecorder::RecordJsonLine(const FString& Type, const FString& JsonPayload)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!bRecording || RecordPath.IsEmpty())
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    const FString Line = FString::Printf(
        // 解释：这一行位于构造函数初始化列表中，把 `TEXT` 直接初始化为 `"{\"type\":\"%s\",\"time\":\"%s\",\"payload\":%s}\n"`，减少进入函数体后的额外赋值开销。
        TEXT("{\"type\":\"%s\",\"time\":\"%s\",\"payload\":%s}\n"),
        *Type,
        *FDateTime::UtcNow().ToIso8601(),
        *JsonPayload);

    // 解释：调用 `SaveStringToFile` 执行当前步骤需要的功能逻辑。
    FFileHelper::SaveStringToFile(Line, *RecordPath, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM, &IFileManager::Get(), EFileWrite::FILEWRITE_Append);
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    ++RecordCount;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

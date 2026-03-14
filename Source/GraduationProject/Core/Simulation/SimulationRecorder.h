// 解释：使用 `#pragma once` 防止该头文件在编译过程中被重复包含。
#pragma once

// 解释：引入 Unreal 的核心基础头文件，提供常用容器、数学类型和日志宏。
#include "CoreMinimal.h"
// 解释：引入 `NoExportTypes.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "UObject/NoExportTypes.h"
// 解释：引入 `SimulationRecorder.generated.h`，让 Unreal Header Tool 生成的反射代码在本文件中可见。
#include "SimulationRecorder.generated.h"

/**
 * @brief 仿真 JSONL 录制器
 * 按行写入结构化事件，方便后处理、回放分析和实验留档。
 */
// 解释：使用 `UCLASS` 宏把当前类型注册到 Unreal 反射系统中，便于蓝图、序列化和编辑器识别。
UCLASS()
// 解释：这一行声明 类 `USimulationRecorder`，用于封装usimulation记录器相关的数据与行为。
class GRADUATIONPROJECT_API USimulationRecorder : public UObject
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：展开 Unreal Header Tool 生成的样板代码，这是 UObject、Actor、组件等类型正常工作的基础。
    GENERATED_BODY()

// 解释：从这一行开始进入 `public` 访问区，下面的成员会对外部模块公开。
public:
    /** @brief 获取全局单例 */
    // 解释：调用 `GetInstance` 执行当前步骤需要的功能逻辑。
    static USimulationRecorder* GetInstance();

    /** @brief 释放全局单例并停止录制 */
    // 解释：调用 `Cleanup` 执行当前步骤需要的功能逻辑。
    static void Cleanup();

    /**
     * @brief 开始录制
     * @param InPath 输出文件路径；为空时自动生成到 `Saved/SimRecords`
     * @return 文件创建成功时返回 `true`
     */
    // 解释：这一行把右侧表达式的结果写入 `bool Start(const FString& InPath`，完成 boolstartconstfstringinpath 的更新。
    bool Start(const FString& InPath = TEXT(""));

    /** @brief 停止录制，不删除已写入文件 */
    // 解释：调用 `Stop` 执行当前步骤需要的功能逻辑。
    void Stop();

    /** @brief 当前是否正在录制 */
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    bool IsRecording() const { return bRecording; }

    /** @brief 获取当前录制文件路径 */
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    FString GetPath() const { return RecordPath; }

    /** @brief 获取当前已追加的事件数量 */
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    int64 GetRecordCount() const { return RecordCount; }

    /**
     * @brief 追加一条 JSONL 事件
     * @param Type 事件类型
     * @param JsonPayload 事件负载 JSON，要求本身是合法 JSON 片段
     */
    // 解释：调用 `RecordJsonLine` 执行当前步骤需要的功能逻辑。
    void RecordJsonLine(const FString& Type, const FString& JsonPayload);

// 解释：从这一行开始进入 `private` 访问区，下面的成员只允许类内部访问。
private:
    /** @brief 全局单例指针 */
    // 解释：这一行声明成员或局部变量 `Instance`，用于保存instance。
    static USimulationRecorder* Instance;

    /** @brief 录制状态标记 */
    // 解释：这一行声明成员或局部变量 `bRecording`，用于保存布尔标志 recording。
    bool bRecording = false;

    /** @brief 当前录制文件路径 */
    // 解释：这一行声明成员或局部变量 `RecordPath`，用于保存记录path。
    FString RecordPath;

    /** @brief 已写入的事件条数 */
    // 解释：这一行声明成员或局部变量 `RecordCount`，用于保存记录count。
    int64 RecordCount = 0;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
};

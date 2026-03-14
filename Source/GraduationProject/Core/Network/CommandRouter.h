// 解释：使用 `#pragma once` 防止该头文件在编译过程中被重复包含。
#pragma once

// 解释：引入 Unreal 的核心基础头文件，提供常用容器、数学类型和日志宏。
#include "CoreMinimal.h"
// 解释：引入 `JsonObject.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "Dom/JsonObject.h"
// 解释：引入 `FCommandHandle.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "FCommandHandle.h"
// 解释：引入 `NoExportTypes.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "UObject/NoExportTypes.h"
// 解释：引入 `CommandRouter.generated.h`，让 Unreal Header Tool 生成的反射代码在本文件中可见。
#include "CommandRouter.generated.h"

/**
 * @brief TCP 命令路由器
 *
 * 接收 JSON 字符串，根据字段名分派到对应的处理方法。
 * 支持仿真控制、智能体查询、命令执行、传感器、录制等接口。
 * 通用 Actor 命令（add_actor / remove_actor / call_actor）委托给 FCommandHandle。
 */
// 解释：使用 `UCLASS` 宏把当前类型注册到 Unreal 反射系统中，便于蓝图、序列化和编辑器识别。
UCLASS()
// 解释：这一行声明 类 `UCommandRouter`，用于封装ucommand路由器相关的数据与行为。
class GRADUATIONPROJECT_API UCommandRouter : public UObject
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：展开 Unreal Header Tool 生成的样板代码，这是 UObject、Actor、组件等类型正常工作的基础。
    GENERATED_BODY()

// 解释：从这一行开始进入 `public` 访问区，下面的成员会对外部模块公开。
public:
    /**
     * @brief 命令总入口——解析 JSON 并分派到各处理函数
     * @param JsonString 原始 JSON 字符串
     * @param World      当前 UWorld
     * @return JSON 响应字符串
     */
    // 解释：调用 `HandleCommand` 执行当前步骤需要的功能逻辑。
    FString HandleCommand(const FString& JsonString, UWorld* World);

// 解释：从这一行开始进入 `private` 访问区，下面的成员只允许类内部访问。
private:
    // ──── 仿真控制 ────

    /** @brief 心跳检测，返回 pong */
    // 解释：调用 `HandlePing` 执行当前步骤需要的功能逻辑。
    FString HandlePing();

    /** @brief 暂停仿真 */
    // 解释：调用 `HandleSimPause` 执行当前步骤需要的功能逻辑。
    FString HandleSimPause(UWorld* World);

    /** @brief 恢复仿真 */
    // 解释：调用 `HandleSimResume` 执行当前步骤需要的功能逻辑。
    FString HandleSimResume(UWorld* World);

    /** @brief 重置仿真 */
    // 解释：调用 `HandleSimReset` 执行当前步骤需要的功能逻辑。
    FString HandleSimReset(UWorld* World);

    /** @brief 获取仿真时间 / 实时时间 / 时间缩放倍率 */
    // 解释：调用 `HandleSimGetTime` 执行当前步骤需要的功能逻辑。
    FString HandleSimGetTime(UWorld* World);

    /** @brief 设置时间缩放倍率 */
    // 解释：调用 `HandleSimSetTimeScale` 执行当前步骤需要的功能逻辑。
    FString HandleSimSetTimeScale(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World);

    /** @brief 单步推进仿真（指定步数与 dt） */
    // 解释：调用 `HandleSimStep` 执行当前步骤需要的功能逻辑。
    FString HandleSimStep(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World);

    // ──── 智能体查询 ────

    /** @brief 获取所有已注册智能体列表（ID、类型、角色） */
    // 解释：调用 `HandleGetAgentList` 执行当前步骤需要的功能逻辑。
    FString HandleGetAgentList();

    // ──── 命令执行管理 ────

    /** @brief 查询异步命令的执行状态 */
    // 解释：调用 `HandleGetCommandStatus` 执行当前步骤需要的功能逻辑。
    FString HandleGetCommandStatus(const TSharedPtr<FJsonObject>& JsonObject);

    /** @brief 取消正在执行的异步命令 */
    // 解释：调用 `HandleCancelCommand` 执行当前步骤需要的功能逻辑。
    FString HandleCancelCommand(const TSharedPtr<FJsonObject>& JsonObject);

    // ──── 传感器数据 ────

    /** @brief 获取指定无人机的传感器状态（位置、速度、姿态等） */
    // 解释：调用 `HandleGetSensorData` 执行当前步骤需要的功能逻辑。
    FString HandleGetSensorData(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World);

    // ──── 录制 ────

    /** @brief 开始录制 */
    // 解释：调用 `HandleRecorderStart` 执行当前步骤需要的功能逻辑。
    FString HandleRecorderStart(const TSharedPtr<FJsonObject>& JsonObject);

    /** @brief 停止录制 */
    // 解释：调用 `HandleRecorderStop` 执行当前步骤需要的功能逻辑。
    FString HandleRecorderStop();

    /** @brief 查询录制状态 */
    // 解释：调用 `HandleRecorderStatus` 执行当前步骤需要的功能逻辑。
    FString HandleRecorderStatus();

    /** @brief 立即记录一帧全部智能体状态到录制文件 */
    // 解释：调用 `HandleRecorderRecordState` 执行当前步骤需要的功能逻辑。
    FString HandleRecorderRecordState(UWorld* World);

    // ──── 工具 ────

    /** @brief 生成错误 JSON 响应 */
    // 解释：调用 `MakeErrorResponse` 执行当前步骤需要的功能逻辑。
    FString MakeErrorResponse(const FString& Error);

    /** @brief 生成成功 JSON 响应 */
    // 解释：这一行把右侧表达式的结果写入 `FString MakeOkResponse(const FString& Message`，完成 fstringmakeokresponseconstfstringmessage 的更新。
    FString MakeOkResponse(const FString& Message = TEXT("ok"));

    /** @brief 通用 Actor 命令处理器（动态生成 / 销毁 / 函数调用） */
    // 解释：这一行声明成员或局部变量 `CommandHandle`，用于保存命令handle。
    TUniquePtr<FCommandHandle> CommandHandle;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
};

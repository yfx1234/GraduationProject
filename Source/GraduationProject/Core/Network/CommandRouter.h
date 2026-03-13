#pragma once

#include "CoreMinimal.h"
#include "Dom/JsonObject.h"
#include "FCommandHandle.h"
#include "UObject/NoExportTypes.h"
#include "CommandRouter.generated.h"

/**
 * @brief TCP 命令路由器
 *
 * 接收 JSON 字符串，根据字段名分派到对应的处理方法。
 * 支持仿真控制、智能体查询、命令执行、传感器、录制等接口。
 * 通用 Actor 命令（add_actor / remove_actor / call_actor）委托给 FCommandHandle。
 */
UCLASS()
class GRADUATIONPROJECT_API UCommandRouter : public UObject
{
    GENERATED_BODY()

public:
    /**
     * @brief 命令总入口——解析 JSON 并分派到各处理函数
     * @param JsonString 原始 JSON 字符串
     * @param World      当前 UWorld
     * @return JSON 响应字符串
     */
    FString HandleCommand(const FString& JsonString, UWorld* World);

private:
    // ──── 仿真控制 ────

    /** @brief 心跳检测，返回 pong */
    FString HandlePing();

    /** @brief 暂停仿真 */
    FString HandleSimPause(UWorld* World);

    /** @brief 恢复仿真 */
    FString HandleSimResume(UWorld* World);

    /** @brief 重置仿真 */
    FString HandleSimReset(UWorld* World);

    /** @brief 获取仿真时间 / 实时时间 / 时间缩放倍率 */
    FString HandleSimGetTime(UWorld* World);

    /** @brief 设置时间缩放倍率 */
    FString HandleSimSetTimeScale(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World);

    /** @brief 单步推进仿真（指定步数与 dt） */
    FString HandleSimStep(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World);

    // ──── 智能体查询 ────

    /** @brief 获取所有已注册智能体列表（ID、类型、角色） */
    FString HandleGetAgentList();

    // ──── 命令执行管理 ────

    /** @brief 查询异步命令的执行状态 */
    FString HandleGetCommandStatus(const TSharedPtr<FJsonObject>& JsonObject);

    /** @brief 取消正在执行的异步命令 */
    FString HandleCancelCommand(const TSharedPtr<FJsonObject>& JsonObject);

    // ──── 传感器数据 ────

    /** @brief 获取指定无人机的传感器状态（位置、速度、姿态等） */
    FString HandleGetSensorData(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World);

    // ──── 录制 ────

    /** @brief 开始录制 */
    FString HandleRecorderStart(const TSharedPtr<FJsonObject>& JsonObject);

    /** @brief 停止录制 */
    FString HandleRecorderStop();

    /** @brief 查询录制状态 */
    FString HandleRecorderStatus();

    /** @brief 立即记录一帧全部智能体状态到录制文件 */
    FString HandleRecorderRecordState(UWorld* World);

    // ──── 工具 ────

    /** @brief 生成错误 JSON 响应 */
    FString MakeErrorResponse(const FString& Error);

    /** @brief 生成成功 JSON 响应 */
    FString MakeOkResponse(const FString& Message = TEXT("ok"));

    /** @brief 通用 Actor 命令处理器（动态生成 / 销毁 / 函数调用） */
    TUniquePtr<FCommandHandle> CommandHandle;
};

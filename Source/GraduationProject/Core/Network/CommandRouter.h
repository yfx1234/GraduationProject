#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "Dom/JsonObject.h"
#include "FCommandHandle.h"
#include "CommandRouter.generated.h"

class UDroneCommandHandler;
class UTurretCommandHandler;
class UGuidanceCommandHandler;

/**
 * @brief TCP 命令总路由器
 * 负责解析外部传入的 JSON 请求，根据字段名将命令分发到仿真控制、
 * 智能体控制、传感器查询、图像获取和录制管理等子模块。
 */
UCLASS()
class GRADUATIONPROJECT_API UCommandRouter : public UObject
{
    GENERATED_BODY()

public:
    /**
     * @brief 处理一条 TCP JSON 命令
     * @param JsonString 原始 JSON 字符串
     * @param World 当前场景 World
     * @return JSON 格式响应，包含 `status` 与具体业务字段
     */
    FString HandleCommand(const FString& JsonString, UWorld* World);

private:
    /** @brief 处理 `ping` 命令，返回连通性检查结果 */
    FString HandlePing();

    /** @brief 处理 `sim_pause` 命令，暂停仿真 */
    FString HandleSimPause(UWorld* World);

    /** @brief 处理 `sim_resume` 命令，恢复仿真 */
    FString HandleSimResume(UWorld* World);

    /** @brief 处理 `sim_reset` 命令，重置当前仿真场景 */
    FString HandleSimReset(UWorld* World);

    /** @brief 处理 `sim_get_time` 命令，返回仿真时间与墙钟时间 */
    FString HandleSimGetTime(UWorld* World);

    /** @brief 处理 `sim_set_time_scale` 命令，设置时间倍率 */
    FString HandleSimSetTimeScale(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World);

    /** @brief 处理 `sim_step` 命令，执行固定步长推进 */
    FString HandleSimStep(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World);

    /** @brief 处理 `get_agent_list` 命令，返回已注册智能体清单 */
    FString HandleGetAgentList();

    /** @brief 处理 `get_command_status` 命令，查询异步命令状态 */
    FString HandleGetCommandStatus(const TSharedPtr<FJsonObject>& JsonObject);

    /** @brief 处理 `cancel_command` 命令，取消异步命令 */
    FString HandleCancelCommand(const TSharedPtr<FJsonObject>& JsonObject);

    /** @brief 处理 `get_sensor_data` 命令，返回无人机传感器数据 */
    FString HandleGetSensorData(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World);

    /** @brief 处理 `recorder_start` 命令，启动 JSONL 录制 */
    FString HandleRecorderStart(const TSharedPtr<FJsonObject>& JsonObject);

    /** @brief 处理 `recorder_stop` 命令，停止录制 */
    FString HandleRecorderStop();

    /** @brief 处理 `recorder_status` 命令，查询录制状态 */
    FString HandleRecorderStatus();

    /** @brief 处理 `recorder_record_state` 命令，主动写入一帧状态快照 */
    FString HandleRecorderRecordState(UWorld* World);

    /**
     * @brief 处理 `get_image` 命令
     * @param JsonObject 已解析的 JSON 对象
     * @param World 当前场景 World
     * @return 包含 Base64 图像数据和相机元数据的 JSON 响应
     */
    FString HandleGetImage(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World);

    /**
     * @brief 构造统一错误响应
     * @param Error 错误信息
     * @return `{"status":"error","message":"..."}`
     */
    FString MakeErrorResponse(const FString& Error);

    /**
     * @brief 构造统一成功响应
     * @param Message 成功说明，默认值为 `ok`
     * @return `{"status":"ok","message":"..."}`
     */
    FString MakeOkResponse(const FString& Message = TEXT("ok"));

    /** @brief 通用 Actor 命令处理器，负责 add/remove/call_actor */
    TUniquePtr<FCommandHandle> CommandHandle;

    /** @brief 无人机命令处理器 */
    UPROPERTY()
    UDroneCommandHandler* DroneHandler = nullptr;

    /** @brief 炮台命令处理器 */
    UPROPERTY()
    UTurretCommandHandler* TurretHandler = nullptr;

    /** @brief 制导命令处理器 */
    UPROPERTY()
    UGuidanceCommandHandler* GuidanceHandler = nullptr;
};

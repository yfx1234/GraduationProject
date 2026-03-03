#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "DroneCommandHandler.generated.h"

class UDroneApi;

/**
 * 无人机 TCP 命令处理器
 * 处理JSON 命令：
 * call_drone — 控制命令
 * get_drone_state — 状态查询
 */
UCLASS()
class GRADUATIONPROJECT_API UDroneCommandHandler : public UObject
{
    GENERATED_BODY()

public:
    /**
     * @brief 处理 call_drone 命令
     * @param JsonObject 已解析的 JSON 对象
     * @param World 当前 UWorld 指针
     * @return JSON 响应字符串
     * 支持的 function 字段：
     * - move_to_position: 移动到指定位置
     * - hover: 悬停
     * - takeoff: 起飞
     * - land: 降落
     * - move_by_velocity: 按速度飞行
     * - set_pid_position: 设置位置控制器增益
     * - set_pid_velocity: 设置速度控制器增益
     * - set_pid_attitude: 设置姿态控制器增益
     * - set_pid_angle_rate: 设置角速率控制器增益
     * - reset: 重置无人机
     */
    FString HandleCallDrone(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World);

    /**
     * @brief 处理 get_drone_state 命令
     * @param JsonObject 已解析的 JSON 对象
     * @param World 当前 UWorld 指针
     * @return JSON 响应字符串
     */
    FString HandleGetDroneState(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World);

private:
    /**
     * @brief 通过 Agent ID 查找对应的 DroneApi 实例
     * @param DroneId 无人机 ID
     * @param World 当前 UWorld 指针
     * @return DroneApi 指针
     */
    UDroneApi* FindDroneApi(const FString& DroneId, UWorld* World);

    /**
     * @brief 构造错误响应 JSON
     * @param Msg 错误信息
     * @return 格式化的 JSON 字符串
     */
    FString MakeError(const FString& Msg);

    /**
     * @brief 构造成功响应 JSON
     * @param Msg 成功信息
     * @return 格式化的 JSON 字符串
     */
    FString MakeOk(const FString& Msg = TEXT("ok"));
};

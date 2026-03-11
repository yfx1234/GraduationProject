#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "DroneCommandHandler.generated.h"

class UDroneApi;

/**
 * @brief 无人机 TCP 命令处理器
 * 负责解析 `call_drone` 和 `get_drone_state` 请求，
 * 并把 JSON 字段分发为对 `UDroneApi` 的调用。
 */
UCLASS()
class GRADUATIONPROJECT_API UDroneCommandHandler : public UObject
{
    GENERATED_BODY()

public:
    /**
     * @brief 处理 `call_drone` 请求
     * @param JsonObject 已解析的 JSON 对象
     * @param World 当前 `UWorld`
     * @return JSON 响应字符串
     */
    FString HandleCallDrone(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World);

    /**
     * @brief 处理 `get_drone_state` 请求
     * @param JsonObject 已解析的 JSON 对象
     * @param World 当前 `UWorld`
     * @return JSON 响应字符串
     */
    FString HandleGetDroneState(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World);

private:
    /**
     * @brief 通过 Agent ID 查找对应的无人机接口
     * @param DroneId 无人机 ID
     * @param World 当前 `UWorld`
     * @return 对应的 `UDroneApi`；未找到时返回空指针
     */
    UDroneApi* FindDroneApi(const FString& DroneId, UWorld* World);

    /**
     * @brief 构造错误响应 JSON
     * @param Msg 错误信息
     * @return 格式化后的 JSON 字符串
     */
    FString MakeError(const FString& Msg);

    /**
     * @brief 构造成功响应 JSON
     * @param Msg 成功信息
     * @return 格式化后的 JSON 字符串
     */
    FString MakeOk(const FString& Msg = TEXT("ok"));
};
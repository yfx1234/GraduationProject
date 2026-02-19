#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "DroneCommandHandler.generated.h"

class UDroneApi;

/**
 * Drone 命令处理器
 * 解析 call_drone / get_drone_state JSON → 调用 DroneApi
 */
UCLASS()
class GRADUATIONPROJECT_API UDroneCommandHandler : public UObject
{
    GENERATED_BODY()

public:
    /**
     * 处理 call_drone 命令
     * @param JsonObject 已解析的 JSON 对象
     * @param World 当前 World
     * @return 响应 JSON
     */
    FString HandleCallDrone(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World);

    /**
     * 处理 get_drone_state 命令
     * @param JsonObject 已解析的 JSON 对象
     * @param World 当前 World
     * @return 响应 JSON
     */
    FString HandleGetDroneState(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World);

private:
    /** 根据 AgentId 找到对应的 DroneApi */
    UDroneApi* FindDroneApi(const FString& DroneId, UWorld* World);

    FString MakeOk(const FString& Msg = TEXT("ok"));
    FString MakeError(const FString& Msg);
};

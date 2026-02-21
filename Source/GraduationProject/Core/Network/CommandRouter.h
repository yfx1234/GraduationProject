#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "CommandRouter.generated.h"

class UDroneCommandHandler;
class UTurretCommandHandler;
class UGuidanceCommandHandler;

/**
 * TCP 命令路由器
 * 接收 JSON 字符串，根据字段路由到对应的 Handler
 * 参考旧项目 DroneCommandHandle（拆分为路由 + 各 Phase Handler）
 */
UCLASS()
class GRADUATIONPROJECT_API UCommandRouter : public UObject
{
    GENERATED_BODY()

public:
    /**
     * 处理一条 TCP 命令（JSON 字符串）
     * @param JsonString 收到的 JSON
     * @param World 当前 World（用于获取 GameMode 等）
     * @return 响应 JSON 字符串
     */
    FString HandleCommand(const FString& JsonString, UWorld* World);

private:
    // ---- 内置命令处理 ----
    FString HandlePing();
    FString HandleSimPause(UWorld* World);
    FString HandleSimResume(UWorld* World);
    FString HandleSimReset(UWorld* World);
    FString HandleGetAgentList();
    FString HandleGetImage(UWorld* World);

    // ---- 工具方法 ----
    FString MakeErrorResponse(const FString& Error);
    FString MakeOkResponse(const FString& Message = TEXT("ok"));

    UPROPERTY()
    UDroneCommandHandler* DroneHandler = nullptr;

    UPROPERTY()
    UTurretCommandHandler* TurretHandler = nullptr;

    UPROPERTY()
    UGuidanceCommandHandler* GuidanceHandler = nullptr;
};

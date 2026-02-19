#pragma once

#include "CoreMinimal.h"
#include "Engine/GameInstance.h"
#include "Sockets.h"
#include "SocketSubsystem.h"
#include "Containers/Ticker.h"
#include "SimGameInstance.generated.h"

class UCommandRouter;

/**
 * 仿真 GameInstance — TCP 服务器
 * 使用 FTSTicker（不依赖 World）驱动 TCP 轮询
 */
UCLASS()
class GRADUATIONPROJECT_API USimGameInstance : public UGameInstance
{
    GENERATED_BODY()

public:
    virtual void Init() override;
    virtual void Shutdown() override;

    /** TCP 监听端口 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Network")
    int32 ListenPort = 9000;

    /** 发送响应 */
    void SendResponse(const FString& Response);

protected:
    /** 核心 Tick 回调（FTSTicker 驱动，不依赖 World） */
    bool HandleTick(float DeltaTime);

    /** 处理接收到的 JSON 字符串 */
    void ProcessReceivedData(const FString& Data);

    /** 创建监听 Socket */
    void CreateListenerSocket();

    /** 检查新连接 */
    void CheckForConnections();

    /** 接收数据 */
    void ReceiveData();

private:
    FSocket* ListenerSocket = nullptr;
    FSocket* ClientSocket = nullptr;

    UPROPERTY()
    UCommandRouter* CommandRouter = nullptr;

    FTSTicker::FDelegateHandle TickDelegateHandle;

    // 接收缓冲
    FString ReceiveBuffer;
};

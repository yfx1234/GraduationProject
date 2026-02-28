#pragma once

#include "CoreMinimal.h"
#include "Engine/GameInstance.h"
#include "Sockets.h"
#include "SocketSubsystem.h"
#include "Containers/Ticker.h"
#include "SimGameInstance.generated.h"

class UCommandRouter;

/**
 * GameInstance — TCP 服务器
 * 作用：
 * 1.  Init()   创建 TCP 监听 Socket 和命令路由器
 * 2. FTSTicker 每帧轮询连接和接收数据
 * 3. 将完整的 JSON 消息交给 CommandRouter 处理
 * 4. 将响应通过 TCP 发送回客户端
 * 5.  Shutdown()   清理所有 Socket 资源
 */
UCLASS()
class GRADUATIONPROJECT_API USimGameInstance : public UGameInstance
{
    GENERATED_BODY()

public:
    /** 初始化 GameInstance */
    virtual void Init() override;

    /** 关闭 GameInstance */
    virtual void Shutdown() override;

    /** TCP 监听端口，默认 9000 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Network")
    int32 ListenPort = 9000;

    /**
     * 向当前连接的客户端发送响应
     * @param Response 响应 JSON 字符串
     */
    void SendResponse(const FString& Response);

protected:
    /**
     * 核心 Tick 回调
     * @param DeltaTime 帧间隔时间（秒）
     * @return 返回 true 保持 Ticker 继续运行
     */
    bool HandleTick(float DeltaTime);

    /**
     * 处理接收到的完整 JSON 字符串
     * @param Data 一条完整的 JSON 命令
     * 将 JSON 字符串传给 CommandRouter 处理，并将响应发送回客户端。
     */
    void ProcessReceivedData(const FString& Data);

    /** 创建 TCP 监听 Socket */
    void CreateListenerSocket();

    /** 检查并接受新的客户端连接 */
    void CheckForConnections();

    /** 从客户端 Socket 接收数据 */
    void ReceiveData();

private:
    /** TCP 监听 Socket */
    FSocket* ListenerSocket = nullptr;

    /** 当前连接的客户端 Socket */
    FSocket* ClientSocket = nullptr;

    /** TCP 命令路由器 */
    UPROPERTY()
    UCommandRouter* CommandRouter = nullptr;

    /** 用于注销 Ticker */
    FTSTicker::FDelegateHandle TickDelegateHandle;

    /** TCP 接收数据缓冲区 */
    FString ReceiveBuffer;
};

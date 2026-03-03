#pragma once

#include "CoreMinimal.h"
#include "Engine/GameInstance.h"
#include "Sockets.h"
#include "SocketSubsystem.h"
#include "Containers/Ticker.h"
#include "SimGameInstance.generated.h"

class UCommandRouter;

UCLASS()
class GRADUATIONPROJECT_API USimGameInstance : public UGameInstance
{
    GENERATED_BODY()

public:
    /** @brief 初始化 GameInstance */
    virtual void Init() override;

    /** @brief 关闭 GameInstance */
    virtual void Shutdown() override;

    /** @brief TCP 监听端口，默认 9000 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Network")
    int32 ListenPort = 9000;

    /**
     * @brief 向当前连接的客户端发送响应
     * @param Response 响应 JSON 字符串
     */
    void SendResponse(const FString& Response);

protected:
    /**
     * @brief 核心 Tick 回调
     * @param DeltaTime 帧间隔时间（秒）
     * @return 返回 true 保持 Ticker 继续运行
     */
    bool HandleTick(float DeltaTime);

    /**
     * @brief 处理接收到的完整 JSON 字符串
     * @param Data 一条完整的 JSON 命令
     * 将 JSON 字符串传给 CommandRouter 处理，并将响应发送回客户端。
     */
    void ProcessReceivedData(const FString& Data);

    /** @brief 创建 TCP 监听 Socket */
    void CreateListenerSocket();

    /** @brief 检查并接受新的客户端连接 */
    void CheckForConnections();

    /** @brief 从客户端 Socket 接收数据 */
    void ReceiveData();

private:
    /** @brief TCP 监听 Socket */
    FSocket* ListenerSocket = nullptr;

    /** @brief 当前连接的客户端 Socket */
    FSocket* ClientSocket = nullptr;

    /** @brief TCP 命令路由器 */
    UPROPERTY()
    UCommandRouter* CommandRouter = nullptr;

    /** @brief 用于注销 Ticker */
    FTSTicker::FDelegateHandle TickDelegateHandle;

    /** @brief TCP 接收数据缓冲区 */
    FString ReceiveBuffer;
};

/**
 * @file SimGameInstance.h
 * @brief 仿真 GameInstance 的头文件 — TCP 服务器实现
 *
 * 本文件定义了 USimGameInstance 类，继承自 UGameInstance，
 * 负责在引擎最底层建立 TCP 服务器，接收来自 Python 客户端的 JSON 命令，
 * 并将命令转发给 CommandRouter 处理后返回响应。
 * 使用 FTSTicker 驱动 TCP 轮询，不依赖 World 的 Tick。
 */

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
 *
 * 核心职责：
 * 1. 在 Init() 阶段创建 TCP 监听 Socket 和命令路由器
 * 2. 通过 FTSTicker（不依赖 World Tick）每帧轮询连接和接收数据
 * 3. 将完整的 JSON 消息交给 CommandRouter 处理
 * 4. 将响应通过 TCP 发送回客户端
 * 5. 在 Shutdown() 阶段清理所有 Socket 资源
 *
 * 消息协议：JSON 字符串以换行符 '\n' 或花括号匹配作为消息分隔符。
 */
UCLASS()
class GRADUATIONPROJECT_API USimGameInstance : public UGameInstance
{
    GENERATED_BODY()

public:
    /**
     * @brief 初始化 GameInstance
     *
     * 创建 CommandRouter、TCP 监听 Socket，注册 FTSTicker 回调。
     */
    virtual void Init() override;

    /**
     * @brief 关闭 GameInstance
     *
     * 移除 Ticker，关闭并销毁所有 Socket 资源。
     */
    virtual void Shutdown() override;

    /** @brief TCP 监听端口，默认 9000 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Network")
    int32 ListenPort = 9000;

    /**
     * @brief 向当前连接的客户端发送响应
     * @param Response 响应 JSON 字符串（会自动追加换行符）
     */
    void SendResponse(const FString& Response);

protected:
    /**
     * @brief 核心 Tick 回调（FTSTicker 驱动）
     * @param DeltaTime 帧间隔时间（秒）
     * @return 返回 true 保持 Ticker 继续运行
     *
     * 每帧检查新连接并接收数据。
     * 不依赖 World 的 Tick，在 Init 阶段就能开始工作。
     */
    bool HandleTick(float DeltaTime);

    /**
     * @brief 处理接收到的完整 JSON 字符串
     * @param Data 一条完整的 JSON 命令
     *
     * 将 JSON 字符串传给 CommandRouter 处理，并将响应发送回客户端。
     */
    void ProcessReceivedData(const FString& Data);

    /**
     * @brief 创建 TCP 监听 Socket
     *
     * 使用 FTcpSocketBuilder 创建可重用的监听 Socket，
     * 绑定到 0.0.0.0:ListenPort，最大等待连接数 8。
     */
    void CreateListenerSocket();

    /**
     * @brief 检查并接受新的客户端连接
     *
     * 如果已有连接则跳过。接受连接后设置为非阻塞模式。
     */
    void CheckForConnections();

    /**
     * @brief 从客户端 Socket 接收数据
     *
     * 将接收到的字节流追加到 ReceiveBuffer，
     * 然后按换行符或花括号匹配分割出完整的 JSON 消息，
     * 逐条调用 ProcessReceivedData() 处理。
     * 如果检测到客户端断开连接，则清理 ClientSocket。
     */
    void ReceiveData();

private:
    /** @brief TCP 监听 Socket（等待客户端连接） */
    FSocket* ListenerSocket = nullptr;

    /** @brief 当前连接的客户端 Socket（同时只支持一个客户端） */
    FSocket* ClientSocket = nullptr;

    /** @brief TCP 命令路由器 */
    UPROPERTY()
    UCommandRouter* CommandRouter = nullptr;

    /** @brief FTSTicker 委托句柄，用于注销 Ticker */
    FTSTicker::FDelegateHandle TickDelegateHandle;

    /** @brief TCP 接收数据缓冲区（可能包含不完整的 JSON 片段） */
    FString ReceiveBuffer;
};

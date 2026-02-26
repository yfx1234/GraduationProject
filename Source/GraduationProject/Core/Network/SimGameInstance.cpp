/**
 * @file SimGameInstance.cpp
 * @brief 仿真 GameInstance 的实现文件 — TCP 服务器逻辑
 *
 * 实现 TCP 监听、客户端连接管理、数据接收与分帧解析、响应发送等网络通信功能。
 */

#include "SimGameInstance.h"
#include "CommandRouter.h"
#include "Common/TcpSocketBuilder.h"

/**
 * @brief 初始化 GameInstance
 *
 * 1. 创建命令路由器
 * 2. 创建 TCP 监听 Socket
 * 3. 注册 FTSTicker 回调（每 10ms 轮询一次）
 */
void USimGameInstance::Init()
{
    Super::Init();

    // 创建命令路由器实例
    CommandRouter = NewObject<UCommandRouter>(this);

    // 创建 TCP 监听 Socket
    CreateListenerSocket();

    // 注册核心 Ticker（不依赖 World，在 Init 阶段就能工作）
    // 第二个参数 0.01f 表示期望的 Tick 间隔（10ms）
    TickDelegateHandle = FTSTicker::GetCoreTicker().AddTicker(
        FTickerDelegate::CreateUObject(this, &USimGameInstance::HandleTick), 0.01f);

    UE_LOG(LogTemp, Warning, TEXT("=== [TCP] SimGameInstance initialized, port=%d ==="), ListenPort);
}

/**
 * @brief 关闭 GameInstance，释放所有网络资源
 *
 * 1. 移除 Ticker 回调
 * 2. 关闭并销毁客户端 Socket
 * 3. 关闭并销毁监听 Socket
 */
void USimGameInstance::Shutdown()
{
    // 移除 Ticker 回调
    FTSTicker::GetCoreTicker().RemoveTicker(TickDelegateHandle);

    // 关闭客户端 Socket
    if (ClientSocket)
    {
        ClientSocket->Close();
        ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(ClientSocket);
        ClientSocket = nullptr;
    }

    // 关闭监听 Socket
    if (ListenerSocket)
    {
        ListenerSocket->Close();
        ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(ListenerSocket);
        ListenerSocket = nullptr;
    }

    UE_LOG(LogTemp, Warning, TEXT("=== [TCP] SimGameInstance shutdown ==="));

    Super::Shutdown();
}

/**
 * @brief 创建 TCP 监听 Socket
 *
 * 使用 FTcpSocketBuilder 创建可重用的 TCP 监听 Socket，
 * 绑定到 0.0.0.0:ListenPort（所有网络接口），最大等待连接数为 8。
 */
void USimGameInstance::CreateListenerSocket()
{
    ListenerSocket = FTcpSocketBuilder(TEXT("SimTcpListener"))
        .AsReusable()               // 允许端口复用
        .BoundToPort(ListenPort)    // 绑定到指定端口
        .Listening(8)               // 最大等待连接队列长度
        .Build();

    if (!ListenerSocket)
    {
        UE_LOG(LogTemp, Error, TEXT("[TCP] Failed to create listener socket on port %d!"), ListenPort);
        return;
    }

    UE_LOG(LogTemp, Warning, TEXT("=== [TCP] Server started on 0.0.0.0:%d ==="), ListenPort);
}

/**
 * @brief 核心 Tick 回调
 * @param DeltaTime 帧间隔时间
 * @return true 保持 Ticker 继续运行
 *
 * 每帧执行：检查新连接 → 接收数据并处理。
 */
bool USimGameInstance::HandleTick(float DeltaTime)
{
    // 检查是否有新的客户端连接请求
    CheckForConnections();

    // 如果有已连接的客户端，接收数据
    if (ClientSocket)
    {
        ReceiveData();
    }

    return true; // 返回 true 保持 Ticker 继续运行
}

/**
 * @brief 检查并接受新的客户端连接
 *
 * 如果已有连接，则跳过（同时只支持一个客户端）。
 * 接受连接后设置 Socket 为非阻塞模式。
 */
void USimGameInstance::CheckForConnections()
{
    if (!ListenerSocket) return;
    if (ClientSocket) return; // 已有连接，跳过

    bool bPending = false;
    if (ListenerSocket->HasPendingConnection(bPending) && bPending)
    {
        // 接受新连接
        TSharedRef<FInternetAddr> RemoteAddr = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();
        ClientSocket = ListenerSocket->Accept(*RemoteAddr, TEXT("SimTcpClient"));

        if (ClientSocket)
        {
            ClientSocket->SetNonBlocking(true); // 非阻塞模式，避免 Recv() 阻塞引擎
            UE_LOG(LogTemp, Warning, TEXT("=== [TCP] Client connected: %s ==="), *RemoteAddr->ToString(true));
        }
    }
}

/**
 * @brief 从客户端接收数据并解析消息
 *
 * 数据接收流程：
 * 1. 循环读取 Socket 缓冲区数据，追加到 ReceiveBuffer
 * 2. 检测连接是否断开
 * 3. 分割完整消息：
 *    - 优先按换行符 '\n' 分割
 *    - 没有换行符时使用花括号匹配法（统计 { } 配对）
 * 4. 将每条完整 JSON 消息交给 ProcessReceivedData() 处理
 */
void USimGameInstance::ReceiveData()
{
    if (!ClientSocket) return;

    // 64KB 接收缓冲区
    uint8 Buffer[65536];
    int32 BytesRead = 0;

    // 循环读取所有可用数据
    while (ClientSocket->Recv(Buffer, sizeof(Buffer) - 1, BytesRead))
    {
        if (BytesRead <= 0) break;

        // 将字节数据转换为 FString 并追加到缓冲区
        Buffer[BytesRead] = '\0';
        FString Received = FString(UTF8_TO_TCHAR(reinterpret_cast<const char*>(Buffer)));
        ReceiveBuffer += Received;
    }

    // 检查连接是否断开
    ESocketConnectionState ConnectionState = ClientSocket->GetConnectionState();
    if (ConnectionState != ESocketConnectionState::SCS_Connected && ReceiveBuffer.IsEmpty())
    {
        UE_LOG(LogTemp, Warning, TEXT("[TCP] Client disconnected"));
        ClientSocket->Close();
        ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(ClientSocket);
        ClientSocket = nullptr;
        ReceiveBuffer.Empty();
        return;
    }

    // 解析完整 JSON 消息
    while (!ReceiveBuffer.IsEmpty())
    {
        int32 DelimiterIndex = INDEX_NONE;

        // 方式1：按换行符分割
        if (ReceiveBuffer.FindChar('\n', DelimiterIndex) || ReceiveBuffer.FindChar('\0', DelimiterIndex))
        {
            // 截取换行符前的内容作为一条消息
            FString Message = ReceiveBuffer.Left(DelimiterIndex).TrimStartAndEnd();
            ReceiveBuffer = ReceiveBuffer.Mid(DelimiterIndex + 1);

            if (!Message.IsEmpty())
            {
                ProcessReceivedData(Message);
            }
        }
        else
        {
            // 方式2：花括号匹配法 — 统计 { } 配对来确定完整 JSON
            FString Trimmed = ReceiveBuffer.TrimStartAndEnd();
            if (!Trimmed.IsEmpty())
            {
                int32 BraceCount = 0;
                bool bComplete = false;
                for (int32 i = 0; i < Trimmed.Len(); i++)
                {
                    if (Trimmed[i] == '{') BraceCount++;
                    else if (Trimmed[i] == '}') BraceCount--;

                    // 花括号归零表示找到一个完整的 JSON 对象
                    if (BraceCount == 0 && i > 0)
                    {
                        bComplete = true;
                        FString CompleteMsg = Trimmed.Left(i + 1);
                        ReceiveBuffer = Trimmed.Mid(i + 1);
                        ProcessReceivedData(CompleteMsg);
                        break;
                    }
                }
                if (!bComplete) break; // 消息不完整，等待更多数据
            }
            else
            {
                ReceiveBuffer.Empty();
                break;
            }
        }
    }
}

/**
 * @brief 处理一条完整的 JSON 命令
 * @param Data JSON 字符串
 *
 * 将 JSON 交给 CommandRouter 处理，获取响应后通过 TCP 发送回客户端。
 */
void USimGameInstance::ProcessReceivedData(const FString& Data)
{
    UE_LOG(LogTemp, Log, TEXT("[TCP] Received: %s"), *Data);

    FString Response;
    if (CommandRouter)
    {
        // 通过命令路由器处理并获取响应
        Response = CommandRouter->HandleCommand(Data, GetWorld());
    }
    else
    {
        Response = TEXT("{\"error\": \"CommandRouter not initialized\"}");
    }

    // 发送响应给客户端
    SendResponse(Response);
}

/**
 * @brief 向客户端发送响应字符串
 * @param Response 响应内容（会自动追加换行符作为消息分隔符）
 *
 * 将 FString 转换为 UTF-8 字节流后通过 TCP Socket 发送。
 */
void USimGameInstance::SendResponse(const FString& Response)
{
    if (!ClientSocket) return;

    // 追加换行符作为消息分隔符
    FString ResponseWithNewline = Response + TEXT("\n");

    // FString → UTF-8 字节流
    FTCHARToUTF8 Converter(*ResponseWithNewline);
    int32 BytesSent = 0;
    ClientSocket->Send(
        reinterpret_cast<const uint8*>(Converter.Get()),
        Converter.Length(),
        BytesSent);

    UE_LOG(LogTemp, Log, TEXT("[TCP] Sent: %s"), *Response);
}

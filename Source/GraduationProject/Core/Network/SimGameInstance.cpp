#include "SimGameInstance.h"
#include "CommandRouter.h"
#include "Common/TcpSocketBuilder.h"

/**
 * 初始化 GameInstance
 * 创建命令路由器
 * 创建 TCP 监听 Socket
 * 注册 FTSTicker 回调
 */
void USimGameInstance::Init()
{
    Super::Init();
    CommandRouter = NewObject<UCommandRouter>(this);
    CreateListenerSocket();
    TickDelegateHandle = FTSTicker::GetCoreTicker().AddTicker(
        FTickerDelegate::CreateUObject(this, &USimGameInstance::HandleTick), 0.01f);
    UE_LOG(LogTemp, Warning, TEXT("[TCP] SimGameInstance initialized, port=%d"), ListenPort);
}

/**
 * 关闭 GameInstance
 * 移除 Ticker 回调
 * 关闭并销毁客户端 Socket
 * 关闭并销毁监听 Socket
 */
void USimGameInstance::Shutdown()
{
    FTSTicker::GetCoreTicker().RemoveTicker(TickDelegateHandle);
    if (ClientSocket)
    {
        ClientSocket->Close();
        ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(ClientSocket);
        ClientSocket = nullptr;
    }
    if (ListenerSocket)
    {
        ListenerSocket->Close();
        ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(ListenerSocket);
        ListenerSocket = nullptr;
    }
    UE_LOG(LogTemp, Warning, TEXT("[TCP] SimGameInstance shutdown"));
    Super::Shutdown();
}

/** 创建 TCP 监听 Socket */
void USimGameInstance::CreateListenerSocket()
{
    ListenerSocket = FTcpSocketBuilder(TEXT("SimTcpListener"))
        .AsReusable()               // 允许端口复用
        .BoundToPort(ListenPort)    // 绑定到指定端口
        .Listening(8)               // 最大等待连接队列数
        .Build();

    if (!ListenerSocket)
    {
        UE_LOG(LogTemp, Error, TEXT("[TCP] Failed to create listener socket on port %d!"), ListenPort);
        return;
    }
    UE_LOG(LogTemp, Warning, TEXT("[TCP] Server started on 0.0.0.0:%d"), ListenPort);
}

/**
 * 核心 Tick 回调
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
 * 检查并接受新的客户端连接
 * 如果已有连接，则跳过（同时只支持一个客户端）
 * 接受连接后设置 Socket 为非阻塞模式
 */
void USimGameInstance::CheckForConnections()
{
    if (!ListenerSocket) return;
    if (ClientSocket) return;
    bool bPending = false;
    if (ListenerSocket->HasPendingConnection(bPending) && bPending)
    {
        TSharedRef<FInternetAddr> RemoteAddr = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();
        ClientSocket = ListenerSocket->Accept(*RemoteAddr, TEXT("SimTcpClient"));
        if (ClientSocket)
        {
            ClientSocket->SetNonBlocking(true);     // 非阻塞模式
            UE_LOG(LogTemp, Warning, TEXT("[TCP] Client connected: %s"), *RemoteAddr->ToString(true));
        }
    }
}

/** 从客户端接收数据并解析消息 */
void USimGameInstance::ReceiveData()
{
    if (!ClientSocket) return;
    uint8 Buffer[65536];
    int32 BytesRead = 0;
    while (ClientSocket->Recv(Buffer, sizeof(Buffer) - 1, BytesRead))
    {
        if (BytesRead <= 0) break;
        Buffer[BytesRead] = '\0';
        FString Received = FString(UTF8_TO_TCHAR(reinterpret_cast<const char*>(Buffer)));
        ReceiveBuffer += Received;
    }
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
    while (!ReceiveBuffer.IsEmpty())
    {
        int32 DelimiterIndex = INDEX_NONE;
        if (ReceiveBuffer.FindChar('\n', DelimiterIndex) || ReceiveBuffer.FindChar('\0', DelimiterIndex))
        {
            FString Message = ReceiveBuffer.Left(DelimiterIndex).TrimStartAndEnd();
            ReceiveBuffer = ReceiveBuffer.Mid(DelimiterIndex + 1);
            if (!Message.IsEmpty()) ProcessReceivedData(Message);
        }
        else
        {
            FString Trimmed = ReceiveBuffer.TrimStartAndEnd();
            if (!Trimmed.IsEmpty())
            {
                int32 BraceCount = 0;
                bool bComplete = false;
                for (int32 i = 0; i < Trimmed.Len(); i++)
                {
                    if (Trimmed[i] == '{') BraceCount++;
                    else if (Trimmed[i] == '}') BraceCount--;
                    if (BraceCount == 0 && i > 0)   // 计数器归零，'{'和'}'完成闭合
                    {
                        bComplete = true;
                        FString CompleteMsg = Trimmed.Left(i + 1);
                        ReceiveBuffer = Trimmed.Mid(i + 1);
                        ProcessReceivedData(CompleteMsg);
                        break;
                    }
                }
                if (!bComplete) break;
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
 * 处理一条完整的 JSON 命令
 * @param Data JSON 字符串
 * 将 JSON 交给 CommandRouter 处理，获取响应后通过 TCP 发送回客户端。
 */
void USimGameInstance::ProcessReceivedData(const FString& Data)
{
    UE_LOG(LogTemp, Log, TEXT("[TCP] Received: %s"), *Data);
    FString Response;
    if (CommandRouter) Response = CommandRouter->HandleCommand(Data, GetWorld());
    else Response = TEXT("{\"error\": \"CommandRouter not initialized\"}");
    SendResponse(Response);
}

/**
 * 向客户端发送响应字符串
 * @param Response 响应内容
 * 将 FString 转换为 UTF-8 字节后通过 TCP Socket 发送。
 */
void USimGameInstance::SendResponse(const FString& Response)
{
    if (!ClientSocket) return;
    FString ResponseWithNewline = Response + TEXT("\n");
    FTCHARToUTF8 Converter(*ResponseWithNewline);
    int32 BytesSent = 0;
    ClientSocket->Send(
        reinterpret_cast<const uint8*>(Converter.Get()),
        Converter.Length(),
        BytesSent);
    UE_LOG(LogTemp, Log, TEXT("[TCP] Sent: %s"), *Response);
}

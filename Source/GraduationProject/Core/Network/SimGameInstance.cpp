#include "SimGameInstance.h"
#include "CommandRouter.h"
#include "Common/TcpSocketBuilder.h"

void USimGameInstance::Init()
{
    Super::Init();

    // 创建命令路由器
    CommandRouter = NewObject<UCommandRouter>(this);

    // 创建 TCP 监听 Socket
    CreateListenerSocket();

    // 注册核心 Ticker（不依赖 World，在 Init 阶段就能工作）
    TickDelegateHandle = FTSTicker::GetCoreTicker().AddTicker(
        FTickerDelegate::CreateUObject(this, &USimGameInstance::HandleTick), 0.01f);

    UE_LOG(LogTemp, Warning, TEXT("=== [TCP] SimGameInstance initialized, port=%d ==="), ListenPort);
}

void USimGameInstance::Shutdown()
{
    // 移除 Ticker
    FTSTicker::GetCoreTicker().RemoveTicker(TickDelegateHandle);

    // 关闭 socket
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

    UE_LOG(LogTemp, Warning, TEXT("=== [TCP] SimGameInstance shutdown ==="));

    Super::Shutdown();
}

void USimGameInstance::CreateListenerSocket()
{
    ListenerSocket = FTcpSocketBuilder(TEXT("SimTcpListener"))
        .AsReusable()
        .BoundToPort(ListenPort)
        .Listening(8)
        .Build();

    if (!ListenerSocket)
    {
        UE_LOG(LogTemp, Error, TEXT("[TCP] Failed to create listener socket on port %d!"), ListenPort);
        return;
    }

    UE_LOG(LogTemp, Warning, TEXT("=== [TCP] Server started on 0.0.0.0:%d ==="), ListenPort);
}

bool USimGameInstance::HandleTick(float DeltaTime)
{
    // 每帧检查连接和接收数据
    CheckForConnections();

    if (ClientSocket)
    {
        ReceiveData();
    }

    return true; // 返回 true 保持 Ticker 继续运行
}

void USimGameInstance::CheckForConnections()
{
    if (!ListenerSocket) return;
    if (ClientSocket) return; // 已有连接

    bool bPending = false;
    if (ListenerSocket->HasPendingConnection(bPending) && bPending)
    {
        TSharedRef<FInternetAddr> RemoteAddr = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();
        ClientSocket = ListenerSocket->Accept(*RemoteAddr, TEXT("SimTcpClient"));

        if (ClientSocket)
        {
            ClientSocket->SetNonBlocking(true);
            UE_LOG(LogTemp, Warning, TEXT("=== [TCP] Client connected: %s ==="), *RemoteAddr->ToString(true));
        }
    }
}

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

    // 解析完整 JSON 消息（按换行符或花括号匹配分割）
    while (!ReceiveBuffer.IsEmpty())
    {
        int32 DelimiterIndex = INDEX_NONE;

        if (ReceiveBuffer.FindChar('\n', DelimiterIndex) || ReceiveBuffer.FindChar('\0', DelimiterIndex))
        {
            FString Message = ReceiveBuffer.Left(DelimiterIndex).TrimStartAndEnd();
            ReceiveBuffer = ReceiveBuffer.Mid(DelimiterIndex + 1);

            if (!Message.IsEmpty())
            {
                ProcessReceivedData(Message);
            }
        }
        else
        {
            // 尝试花括号匹配
            FString Trimmed = ReceiveBuffer.TrimStartAndEnd();
            if (!Trimmed.IsEmpty())
            {
                int32 BraceCount = 0;
                bool bComplete = false;
                for (int32 i = 0; i < Trimmed.Len(); i++)
                {
                    if (Trimmed[i] == '{') BraceCount++;
                    else if (Trimmed[i] == '}') BraceCount--;
                    if (BraceCount == 0 && i > 0)
                    {
                        bComplete = true;
                        FString CompleteMsg = Trimmed.Left(i + 1);
                        ReceiveBuffer = Trimmed.Mid(i + 1);
                        ProcessReceivedData(CompleteMsg);
                        break;
                    }
                }
                if (!bComplete) break; // 等待更多数据
            }
            else
            {
                ReceiveBuffer.Empty();
                break;
            }
        }
    }
}

void USimGameInstance::ProcessReceivedData(const FString& Data)
{
    UE_LOG(LogTemp, Log, TEXT("[TCP] Received: %s"), *Data);

    FString Response;
    if (CommandRouter)
    {
        Response = CommandRouter->HandleCommand(Data, GetWorld());
    }
    else
    {
        Response = TEXT("{\"error\": \"CommandRouter not initialized\"}");
    }

    SendResponse(Response);
}

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

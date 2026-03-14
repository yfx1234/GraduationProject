#include "SimGameInstance.h"

#include "CommandRouter.h"
#include "Common/TcpSocketBuilder.h"
#include "SocketSubsystem.h"

namespace
{
void DestroySocket(FSocket*& Socket)
{
    if (!Socket)
    {
        return;
    }

    Socket->Close();
    ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(Socket);
    Socket = nullptr;
}

FString Utf8BytesToString(const TArray<uint8>& Bytes)
{
    if (Bytes.IsEmpty())
    {
        return FString();
    }

    TArray<uint8> NullTerminated(Bytes);
    NullTerminated.Add(0);
    return FString(UTF8_TO_TCHAR(reinterpret_cast<const char*>(NullTerminated.GetData())));
}
}

void USimGameInstance::Init()
{
    Super::Init();

    CommandRouter = NewObject<UCommandRouter>(this);
    CreateListenerSocket();
    TickDelegateHandle = FTSTicker::GetCoreTicker().AddTicker(
        FTickerDelegate::CreateUObject(this, &USimGameInstance::HandleTick),
        0.01f);

    UE_LOG(LogTemp, Log, TEXT("[TCP] SimGameInstance initialized on port %d"), ListenPort);
}

void USimGameInstance::Shutdown()
{
    if (TickDelegateHandle.IsValid())
    {
        FTSTicker::GetCoreTicker().RemoveTicker(TickDelegateHandle);
    }

    ReceiveBuffer.Reset();
    DestroySocket(ClientSocket);
    DestroySocket(ListenerSocket);

    UE_LOG(LogTemp, Log, TEXT("[TCP] SimGameInstance shutdown"));
    Super::Shutdown();
}

void USimGameInstance::CreateListenerSocket()
{
    ListenerSocket = FTcpSocketBuilder(TEXT("GradSimListener"))
        .AsReusable()
        .BoundToPort(ListenPort)
        .Listening(8);

    if (!ListenerSocket)
    {
        UE_LOG(LogTemp, Error, TEXT("[TCP] Failed to create listener socket on port %d"), ListenPort);
        return;
    }

    ListenerSocket->SetNonBlocking(true);
    UE_LOG(LogTemp, Log, TEXT("[TCP] Listening on 0.0.0.0:%d"), ListenPort);
}

bool USimGameInstance::HandleTick(float DeltaTime)
{
    CheckForConnections();
    if (ClientSocket)
    {
        ReceiveData();
    }

    return true;
}

void USimGameInstance::CheckForConnections()
{
    if (!ListenerSocket || ClientSocket)
    {
        return;
    }

    bool bPending = false;
    if (!ListenerSocket->HasPendingConnection(bPending) || !bPending)
    {
        return;
    }

    TSharedRef<FInternetAddr> RemoteAddr = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();
    ClientSocket = ListenerSocket->Accept(*RemoteAddr, TEXT("GradSimClient"));
    if (!ClientSocket)
    {
        return;
    }

    ClientSocket->SetNonBlocking(true);
    ReceiveBuffer.Reset();
    UE_LOG(LogTemp, Log, TEXT("[TCP] Client connected: %s"), *RemoteAddr->ToString(true));
}

void USimGameInstance::ReceiveData()
{
    if (!ClientSocket)
    {
        return;
    }

    uint32 PendingSize = 0;
    while (ClientSocket->HasPendingData(PendingSize))
    {
        const int32 BufferSize = FMath::Max(1, static_cast<int32>(FMath::Min<uint32>(PendingSize, 65536u)));
        TArray<uint8> Buffer;
        Buffer.SetNumUninitialized(BufferSize);

        int32 BytesRead = 0;
        if (!ClientSocket->Recv(Buffer.GetData(), Buffer.Num(), BytesRead) || BytesRead <= 0)
        {
            break;
        }

        ReceiveBuffer.Append(Buffer.GetData(), BytesRead);
    }

    while (true)
    {
        const int32 NewlineIndex = ReceiveBuffer.Find(static_cast<uint8>('\n'));
        if (NewlineIndex == INDEX_NONE)
        {
            break;
        }

        TArray<uint8> MessageBytes;
        if (NewlineIndex > 0)
        {
            MessageBytes.Append(ReceiveBuffer.GetData(), NewlineIndex);
        }

        ReceiveBuffer.RemoveAt(0, NewlineIndex + 1, EAllowShrinking::No);
        const FString Message = Utf8BytesToString(MessageBytes).TrimStartAndEnd();
        if (!Message.IsEmpty())
        {
            ProcessReceivedData(Message);
        }
    }

    if (ClientSocket->GetConnectionState() == ESocketConnectionState::SCS_Connected)
    {
        return;
    }

    const FString PartialMessage = Utf8BytesToString(ReceiveBuffer).TrimStartAndEnd();
    if (!PartialMessage.IsEmpty())
    {
        UE_LOG(LogTemp, Warning, TEXT("[TCP] Discarding partial message on disconnect: %s"), *PartialMessage);
    }

    ReceiveBuffer.Reset();
    DestroySocket(ClientSocket);
    UE_LOG(LogTemp, Log, TEXT("[TCP] Client disconnected"));
}

void USimGameInstance::ProcessReceivedData(const FString& Data)
{
    FString Response = TEXT("{\"status\":\"error\",\"message\":\"CommandRouter not initialized\"}");
    if (CommandRouter)
    {
        Response = CommandRouter->HandleCommand(Data, GetWorld());
    }

    SendResponse(Response);
}

void USimGameInstance::SendResponse(const FString& Response)
{
    if (!ClientSocket)
    {
        return;
    }

    const FString Payload = Response + TEXT("\n");
    FTCHARToUTF8 Converter(*Payload);
    int32 BytesSent = 0;
    if (!ClientSocket->Send(reinterpret_cast<const uint8*>(Converter.Get()), Converter.Length(), BytesSent))
    {
        UE_LOG(LogTemp, Warning, TEXT("[TCP] Failed to send response"));
    }
}
#pragma once

#include "CoreMinimal.h"
#include "Engine/GameInstance.h"
#include "Sockets.h"
#include "Containers/Ticker.h"
#include "SimGameInstance.generated.h"

class UCommandRouter;

UCLASS()
class GRADUATIONPROJECT_API USimGameInstance : public UGameInstance
{
    GENERATED_BODY()

public:
    virtual void Init() override;
    virtual void Shutdown() override;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Network")
    int32 ListenPort = 9000;

    void SendResponse(const FString& Response);

protected:
    bool HandleTick(float DeltaTime);
    void CreateListenerSocket();
    void CheckForConnections();
    void ReceiveData();
    void ProcessReceivedData(const FString& Data);

private:
    FSocket* ListenerSocket = nullptr;
    FSocket* ClientSocket = nullptr;

    UPROPERTY()
    UCommandRouter* CommandRouter = nullptr;

    FTSTicker::FDelegateHandle TickDelegateHandle;

    // 直接保存 UTF-8 原始字节，避免中文在分包时被截断。
    TArray<uint8> ReceiveBuffer;
};
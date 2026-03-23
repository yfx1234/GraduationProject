
#pragma once
#include "CoreMinimal.h"
class FJsonObject;
class UAgentManager;
class GRADUATIONPROJECT_API FCommandHandle
{
public:
    explicit FCommandHandle(UGameInstance* InGameInstance);
    ~FCommandHandle() {}
    FString HandleCommand(const TSharedPtr<FJsonObject>& RootJson);
    FString HandleAddActor(const TSharedPtr<FJsonObject>& CommandJson);
    FString HandleRemoveActor(const TSharedPtr<FJsonObject>& CommandJson);
    FString HandleCallActor(const TSharedPtr<FJsonObject>& CommandJson);
private:
    struct FCallParameters
    {
        bool bUseNamedParameters = false;
        TArray<FString> PositionalParameters;
        TMap<FString, FString> NamedParameters;
    };
    static UClass* ResolveActorClass(const FString& ClassNameOrPath);
    static void ReadSpawnPose(const TSharedPtr<FJsonObject>& CommandJson, FVector& OutSpawnPos, FRotator& OutSpawnRot);
    static FString ResolveActorId(const TSharedPtr<FJsonObject>& CommandJson, UAgentManager* Manager);
    static bool ParseCallParameters(
        const TSharedPtr<FJsonObject>& CommandJson,
        FCallParameters& OutCallParameters,
        FString& OutError);
    static bool CallActorFunction(
        AActor* TargetActor,
        const FString& FunctionName,
        const FCallParameters& Parameters,
        FString& OutReturnValue);
    static void SetPropertyValue(FProperty* Property, void* Container, const FString& Value);
    static FString ConvertPropertyToString(FProperty* Prop, void* ValuePtr);
    static FString MakeAddActorResponse(const FString& ActorID, bool bSuccess, const FString& Message);
    static FString MakeRemoveActorResponse(bool bSuccess, const FString& Message);
    static FString MakeCallActorResponse(bool bSuccess, const FString& Message, const FString& ReturnValue);
    static FString MakeErrorResponse(const FString& ReturnType, const FString& Message);
    UGameInstance* GameInstance;
};

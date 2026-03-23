
#include "FCommandHandle.h"
#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"
#include "Engine/GameInstance.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "GraduationProject/Core/Manager/AgentManager.h"
#include "GraduationProject/Drone/DronePawn.h"
#include "Kismet/GameplayStatics.h"
#include "Policies/CondensedJsonPrintPolicy.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "UObject/UObjectGlobals.h"
#include "UObject/UnrealType.h"
namespace
{
    bool TryParseOrderedKey(const FString& Key, int32& OutIndex)
    {
        FString Work = Key;
        Work.ToLowerInline();
        if (Work.StartsWith(TEXT("arg")))
        {
            Work = Work.RightChop(3);
        }
        else if (Work.StartsWith(TEXT("p")))
        {
            Work = Work.RightChop(1);
        }
        if (!Work.IsNumeric())
        {
            return false;
        }
        OutIndex = FCString::Atoi(*Work);
        return true;
    }
    FString SerializeJsonValue(const TSharedPtr<FJsonValue>& Value)
    {
        if (!Value.IsValid())
        {
            return TEXT("");
        }
        FString Output;
        const TSharedRef<TJsonWriter<>> Writer = TJsonWriterFactory<>::Create(&Output);
        FJsonSerializer::Serialize(Value.ToSharedRef(), TEXT(""), Writer);
        return Output;
    }
    FString JsonValueToParamString(const TSharedPtr<FJsonValue>& Value)
    {
        if (!Value.IsValid())
        {
            return TEXT("");
        }
        switch (Value->Type)
        {
        case EJson::String:
            return Value->AsString();
        case EJson::Number:
            return FString::SanitizeFloat(Value->AsNumber());
        case EJson::Boolean:
            return Value->AsBool() ? TEXT("true") : TEXT("false");
        case EJson::Array:
        {
            const TArray<TSharedPtr<FJsonValue>>& ArrayValues = Value->AsArray();
            if (ArrayValues.Num() == 3 &&
                ArrayValues[0].IsValid() && ArrayValues[1].IsValid() && ArrayValues[2].IsValid() &&
                ArrayValues[0]->Type == EJson::Number &&
                ArrayValues[1]->Type == EJson::Number &&
                ArrayValues[2]->Type == EJson::Number)
            {
                const float X = static_cast<float>(ArrayValues[0]->AsNumber());
                const float Y = static_cast<float>(ArrayValues[1]->AsNumber());
                const float Z = static_cast<float>(ArrayValues[2]->AsNumber());
                return FString::Printf(TEXT("FVector:%f,%f,%f"), X, Y, Z);
            }
            return SerializeJsonValue(Value);
        }
        case EJson::Object:
        {
            const TSharedPtr<FJsonObject> Obj = Value->AsObject();
            if (!Obj.IsValid())
            {
                return TEXT("");
            }
            if (Obj->HasTypedField<EJson::Number>(TEXT("x")) &&
                Obj->HasTypedField<EJson::Number>(TEXT("y")) &&
                Obj->HasTypedField<EJson::Number>(TEXT("z")))
            {
                const float X = static_cast<float>(Obj->GetNumberField(TEXT("x")));
                const float Y = static_cast<float>(Obj->GetNumberField(TEXT("y")));
                const float Z = static_cast<float>(Obj->GetNumberField(TEXT("z")));
                return FString::Printf(TEXT("FVector:%f,%f,%f"), X, Y, Z);
            }
            if (Obj->HasTypedField<EJson::Number>(TEXT("roll")) &&
                Obj->HasTypedField<EJson::Number>(TEXT("pitch")) &&
                Obj->HasTypedField<EJson::Number>(TEXT("yaw")))
            {
                const float Roll = static_cast<float>(Obj->GetNumberField(TEXT("roll")));
                const float Pitch = static_cast<float>(Obj->GetNumberField(TEXT("pitch")));
                const float Yaw = static_cast<float>(Obj->GetNumberField(TEXT("yaw")));
                return FString::Printf(TEXT("FRotator:%f,%f,%f"), Roll, Pitch, Yaw);
            }
            return SerializeJsonValue(Value);
        }
        case EJson::Null:
            return TEXT("");
        default:
            return SerializeJsonValue(Value);
        }
    }
    EDroneMissionRole ParseMissionRole(const TSharedPtr<FJsonObject>& CommandJson)
    {
        if (!CommandJson.IsValid())
        {
            return EDroneMissionRole::Unknown;
        }
        FString Role;
        if (CommandJson->HasField(TEXT("mission_role")))
        {
            Role = CommandJson->GetStringField(TEXT("mission_role"));
        }
        else if (CommandJson->HasField(TEXT("role")))
        {
            Role = CommandJson->GetStringField(TEXT("role"));
        }
        Role.ToLowerInline();
        if (Role == TEXT("target"))
        {
            return EDroneMissionRole::Target;
        }
        if (Role == TEXT("interceptor"))
        {
            return EDroneMissionRole::Interceptor;
        }
        return EDroneMissionRole::Unknown;
    }
    bool ShouldUseMetersForSpawn(const TSharedPtr<FJsonObject>& CommandJson, UClass* ActorClass)
    {
        if (CommandJson.IsValid())
        {
            FString Unit;
            if (CommandJson->TryGetStringField(TEXT("unit"), Unit) ||
                CommandJson->TryGetStringField(TEXT("units"), Unit) ||
                CommandJson->TryGetStringField(TEXT("frame"), Unit))
            {
                Unit.ToLowerInline();
                if (Unit == TEXT("cm") || Unit == TEXT("centimeter") || Unit == TEXT("centimeters") || Unit == TEXT("ue_cm"))
                {
                    return false;
                }
                if (Unit == TEXT("m") || Unit == TEXT("meter") || Unit == TEXT("meters") || Unit == TEXT("ue") || Unit == TEXT("world"))
                {
                    return true;
                }
            }
        }
        return ActorClass &&
            (ActorClass->IsChildOf(ADronePawn::StaticClass()));
    }
    bool TrySetStringProperty(AActor* TargetActor, const TCHAR* PropertyName, const FString& Value)
    {
        if (!TargetActor)
        {
            return false;
        }
        if (FStrProperty* Property = FindFProperty<FStrProperty>(TargetActor->GetClass(), PropertyName))
        {
            Property->SetPropertyValue_InContainer(TargetActor, Value);
            return true;
        }
        return false;
    }
    void AssignActorId(AActor* TargetActor, const FString& ActorId)
    {
        if (!TargetActor || ActorId.IsEmpty())
        {
            return;
        }
        TrySetStringProperty(TargetActor, TEXT("DroneId"), ActorId);
        TrySetStringProperty(TargetActor, TEXT("GuidanceId"), ActorId);
        TrySetStringProperty(TargetActor, TEXT("ActorId"), ActorId);
        TrySetStringProperty(TargetActor, TEXT("AgentId"), ActorId);
        if (ADronePawn* Drone = Cast<ADronePawn>(TargetActor))
        {
            Drone->DroneId = ActorId;
        }
    }
    FString MakeRandomActorId()
    {
        return FString::Printf(TEXT("actor_%d"), FMath::RandRange(1000, 9999));
    }
    void ApplySpawnMetadata(AActor* TargetActor, const FString& ActorId, EDroneMissionRole MissionRole)
    {
        AssignActorId(TargetActor, ActorId);
        if (ADronePawn* SpawnedDrone = Cast<ADronePawn>(TargetActor))
        {
            SpawnedDrone->MissionRole = MissionRole;
        }
    }
    const FString* FindNamedParameterValue(
        const TMap<FString, FString>& NamedParameters,
        const FString& PropertyName,
        int32 ParamIndex)
    {
        for (const TPair<FString, FString>& Pair : NamedParameters)
        {
            if (Pair.Key.Equals(PropertyName, ESearchCase::IgnoreCase))
            {
                return &Pair.Value;
            }
        }
        for (const TPair<FString, FString>& Pair : NamedParameters)
        {
            if (!Pair.Key.EndsWith(PropertyName, ESearchCase::IgnoreCase))
            {
                continue;
            }
            if (Pair.Key.Len() == PropertyName.Len())
            {
                return &Pair.Value;
            }
            const int32 PrefixIndex = Pair.Key.Len() - PropertyName.Len() - 1;
            if (PrefixIndex >= 0 && Pair.Key[PrefixIndex] == TCHAR('_'))
            {
                return &Pair.Value;
            }
        }
        const FString ArgKey = FString::Printf(TEXT("arg%d"), ParamIndex);
        const FString PKey = FString::Printf(TEXT("p%d"), ParamIndex);
        for (const TPair<FString, FString>& Pair : NamedParameters)
        {
            if (Pair.Key.Equals(ArgKey, ESearchCase::IgnoreCase) || Pair.Key.Equals(PKey, ESearchCase::IgnoreCase))
            {
                return &Pair.Value;
            }
        }
        return nullptr;
    }
    TSharedPtr<FJsonObject> MakeStatusResponseObject(bool bSuccess, const FString& Message)
    {
        const TSharedPtr<FJsonObject> ResponseJson = MakeShareable(new FJsonObject);
        ResponseJson->SetBoolField(TEXT("status"), bSuccess);
        ResponseJson->SetStringField(TEXT("message"), Message);
        return ResponseJson;
    }
    FString SerializeWrappedResponse(const FString& FieldName, const TSharedPtr<FJsonObject>& ResponseJson)
    {
        const TSharedPtr<FJsonObject> FullResponse = MakeShareable(new FJsonObject);
        FullResponse->SetObjectField(FieldName, ResponseJson);
        FString OutputString;
        const TSharedRef<TJsonWriter<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>> Writer =
            TJsonWriterFactory<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>::Create(&OutputString);
        FJsonSerializer::Serialize(FullResponse.ToSharedRef(), Writer);
        return OutputString;
    }
}
FCommandHandle::FCommandHandle(UGameInstance* InGameInstance)
    : GameInstance(InGameInstance)
{
}
UClass* FCommandHandle::ResolveActorClass(const FString& ClassNameOrPath)
{
    if (ClassNameOrPath.StartsWith(TEXT("Blueprint'/")) || ClassNameOrPath.StartsWith(TEXT("/Game/")))
    {
        return LoadClass<AActor>(nullptr, *ClassNameOrPath);
    }
    if (ClassNameOrPath.StartsWith(TEXT("/Script/")))
    {
        return StaticLoadClass(AActor::StaticClass(), nullptr, *ClassNameOrPath);
    }
    FString FullClassName = FString::Printf(TEXT("/Script/GraduationProject.%s"), *ClassNameOrPath);
    UClass* ActorClass = StaticLoadClass(AActor::StaticClass(), nullptr, *FullClassName);
    if (!ActorClass)
    {
        FullClassName = FString::Printf(TEXT("/Script/Engine.%s"), *ClassNameOrPath);
        ActorClass = StaticLoadClass(AActor::StaticClass(), nullptr, *FullClassName);
    }
    return ActorClass;
}
void FCommandHandle::ReadSpawnPose(const TSharedPtr<FJsonObject>& CommandJson, FVector& OutSpawnPos, FRotator& OutSpawnRot)
{
    OutSpawnPos = FVector::ZeroVector;
    OutSpawnRot = FRotator::ZeroRotator;
    const TSharedPtr<FJsonObject>* PoseJsonPtr = nullptr;
    if (!CommandJson->TryGetObjectField(TEXT("pose"), PoseJsonPtr) || !PoseJsonPtr || !PoseJsonPtr->IsValid())
    {
        return;
    }
    const TSharedPtr<FJsonObject>& PoseJson = *PoseJsonPtr;
    double Value = 0.0;
    if (PoseJson->TryGetNumberField(TEXT("x"), Value))
    {
        OutSpawnPos.X = static_cast<float>(Value);
    }
    if (PoseJson->TryGetNumberField(TEXT("y"), Value))
    {
        OutSpawnPos.Y = static_cast<float>(Value);
    }
    if (PoseJson->TryGetNumberField(TEXT("z"), Value))
    {
        OutSpawnPos.Z = static_cast<float>(Value);
    }
    double Roll = 0.0;
    double Pitch = 0.0;
    double Yaw = 0.0;
    PoseJson->TryGetNumberField(TEXT("roll"), Roll);
    PoseJson->TryGetNumberField(TEXT("pitch"), Pitch);
    PoseJson->TryGetNumberField(TEXT("yaw"), Yaw);
    OutSpawnRot = FRotator(static_cast<float>(Pitch), static_cast<float>(Yaw), static_cast<float>(Roll));
}
FString FCommandHandle::ResolveActorId(const TSharedPtr<FJsonObject>& CommandJson, UAgentManager* Manager)
{
    if (CommandJson->HasField(TEXT("expected_id")) && !CommandJson->GetStringField(TEXT("expected_id")).IsEmpty())
    {
        return CommandJson->GetStringField(TEXT("expected_id"));
    }
    if (!Manager)
    {
        return MakeRandomActorId();
    }
    FString ActorId = MakeRandomActorId();
    while (Manager->GetAgent(ActorId) != nullptr)
    {
        ActorId = MakeRandomActorId();
    }
    return ActorId;
}
bool FCommandHandle::ParseCallParameters(
    const TSharedPtr<FJsonObject>& CommandJson,
    FCallParameters& OutCallParameters,
    FString& OutError)
{
    OutCallParameters = FCallParameters{};
    OutError.Empty();
    if (!CommandJson.IsValid() || !CommandJson->HasField(TEXT("parameters")))
    {
        return true;
    }
    const TSharedPtr<FJsonValue> ParametersValue = CommandJson->TryGetField(TEXT("parameters"));
    if (!ParametersValue.IsValid())
    {
        return true;
    }
    if (ParametersValue->Type == EJson::Array)
    {
        const TArray<TSharedPtr<FJsonValue>>& Values = ParametersValue->AsArray();
        OutCallParameters.PositionalParameters.Reserve(Values.Num());
        for (const TSharedPtr<FJsonValue>& Value : Values)
        {
            OutCallParameters.PositionalParameters.Add(JsonValueToParamString(Value));
        }
        return true;
    }
    if (ParametersValue->Type == EJson::Object)
    {
        OutCallParameters.bUseNamedParameters = true;
        const TSharedPtr<FJsonObject> ParametersObject = ParametersValue->AsObject();
        if (!ParametersObject.IsValid())
        {
            return true;
        }
        for (const TPair<FString, TSharedPtr<FJsonValue>>& Pair : ParametersObject->Values)
        {
            OutCallParameters.NamedParameters.Add(Pair.Key, JsonValueToParamString(Pair.Value));
        }
        return true;
    }
    OutError = TEXT("parameters must be array or object");
    return false;
}
FString FCommandHandle::HandleCommand(const TSharedPtr<FJsonObject>& RootJson)
{
    if (!RootJson.IsValid())
    {
        return TEXT("");
    }
    if (RootJson->HasField(TEXT("add_actor")))
    {
        return HandleAddActor(RootJson->GetObjectField(TEXT("add_actor")));
    }
    if (RootJson->HasField(TEXT("remove_actor")))
    {
        return HandleRemoveActor(RootJson->GetObjectField(TEXT("remove_actor")));
    }
    if (RootJson->HasField(TEXT("call_actor")))
    {
        return HandleCallActor(RootJson->GetObjectField(TEXT("call_actor")));
    }
    return TEXT("");
}
FString FCommandHandle::HandleAddActor(const TSharedPtr<FJsonObject>& CommandJson)
{
    if (!CommandJson.IsValid() || !CommandJson->HasField(TEXT("classname")))
    {
        return MakeAddActorResponse(TEXT(""), false, TEXT("Missing classname"));
    }
    const FString ClassNameOrPath = CommandJson->GetStringField(TEXT("classname"));
    UClass* ActorClass = ResolveActorClass(ClassNameOrPath);
    if (!ActorClass)
    {
        UE_LOG(LogTemp, Error, TEXT("[FCommandHandle] Class not found: %s"), *ClassNameOrPath);
        return MakeAddActorResponse(TEXT(""), false, FString::Printf(TEXT("Class not found: %s"), *ClassNameOrPath));
    }
    FVector SpawnPos = FVector::ZeroVector;
    FRotator SpawnRot = FRotator::ZeroRotator;
    ReadSpawnPose(CommandJson, SpawnPos, SpawnRot);
    if (ShouldUseMetersForSpawn(CommandJson, ActorClass))
    {
        SpawnPos *= 100.0f;
    }
    UAgentManager* Manager = UAgentManager::GetInstance();
    if (!Manager)
    {
        return MakeAddActorResponse(TEXT(""), false, TEXT("Agent manager unavailable"));
    }
    const FString ActorId = ResolveActorId(CommandJson, Manager);
    if (Manager->GetAgent(ActorId) != nullptr)
    {
        UE_LOG(LogTemp, Warning, TEXT("[FCommandHandle] Actor ID already exists: %s"), *ActorId);
        return MakeAddActorResponse(ActorId, false, FString::Printf(TEXT("Actor ID already exists: %s"), *ActorId));
    }
    UWorld* World = GameInstance ? GameInstance->GetWorld() : nullptr;
    if (!World)
    {
        return MakeAddActorResponse(ActorId, false, TEXT("World is null"));
    }
    const FTransform SpawnTransform(SpawnRot, SpawnPos);
    AActor* NewActor = World->SpawnActorDeferred<AActor>(
        ActorClass,
        SpawnTransform,
        nullptr,
        nullptr,
        ESpawnActorCollisionHandlingMethod::AlwaysSpawn);
    if (!NewActor)
    {
        UE_LOG(LogTemp, Error, TEXT("[FCommandHandle] Failed to spawn actor"));
        return MakeAddActorResponse(ActorId, false, TEXT("Failed to spawn actor"));
    }
    const EDroneMissionRole MissionRole = ParseMissionRole(CommandJson);
    ApplySpawnMetadata(NewActor, ActorId, MissionRole);
    UGameplayStatics::FinishSpawningActor(NewActor, SpawnTransform);
    ApplySpawnMetadata(NewActor, ActorId, MissionRole);
    FString Label = CommandJson->HasField(TEXT("label")) ? CommandJson->GetStringField(TEXT("label")) : TEXT("Vehicle");
    if (Label.IsEmpty())
    {
        Label = TEXT("Vehicle");
    }
    NewActor->SetActorLabel(FString::Printf(TEXT("%s_%s"), *Label, *ActorId));
    if (Manager->GetAgent(ActorId) != NewActor)
    {
        Manager->RegisterAgent(ActorId, NewActor);
    }
    UE_LOG(LogTemp, Log, TEXT("[FCommandHandle] Actor created successfully, ID: %s"), *ActorId);
    return MakeAddActorResponse(ActorId, true, TEXT("Actor created successfully"));
}
FString FCommandHandle::HandleRemoveActor(const TSharedPtr<FJsonObject>& CommandJson)
{
    FString ActorID;
    if (!CommandJson->TryGetStringField(TEXT("actor_id"), ActorID))
    {
        return MakeRemoveActorResponse(false, TEXT("Missing actor_id"));
    }
    UAgentManager* Manager = UAgentManager::GetInstance();
    if (!Manager)
    {
        return MakeRemoveActorResponse(false, TEXT("Agent manager unavailable"));
    }
    AActor* ActorToRemove = Manager->GetAgent(ActorID);
    if (!ActorToRemove)
    {
        UE_LOG(LogTemp, Warning, TEXT("[FCommandHandle] Actor not found: %s"), *ActorID);
        return MakeRemoveActorResponse(false, FString::Printf(TEXT("Actor not found: %s"), *ActorID));
    }
    TArray<FString> AliasIds;
    const TArray<FString> ExistingIds = Manager->GetAllAgentIds();
    for (const FString& ExistingId : ExistingIds)
    {
        if (Manager->GetAgent(ExistingId) == ActorToRemove)
        {
            AliasIds.Add(ExistingId);
        }
    }
    if (IsValid(ActorToRemove))
    {
        ActorToRemove->Destroy();
    }
    if (!AliasIds.Contains(ActorID))
    {
        AliasIds.Add(ActorID);
    }
    for (const FString& Id : AliasIds)
    {
        Manager->UnregisterAgent(Id);
    }
    UE_LOG(LogTemp, Log, TEXT("[FCommandHandle] Actor removed: %s (aliases removed=%d)"), *ActorID, AliasIds.Num());
    return MakeRemoveActorResponse(true, FString::Printf(TEXT("Actor removed successfully (aliases=%d)"), AliasIds.Num()));
}
FString FCommandHandle::HandleCallActor(const TSharedPtr<FJsonObject>& CommandJson)
{
    FString FunctionName;
    if (!CommandJson->TryGetStringField(TEXT("function"), FunctionName))
    {
        return MakeCallActorResponse(false, TEXT("Missing function field"), TEXT(""));
    }
    FString ActorID;
    if (!CommandJson->TryGetStringField(TEXT("actor_id"), ActorID))
    {
        return MakeCallActorResponse(false, TEXT("Missing actor_id"), TEXT(""));
    }
    UAgentManager* Manager = UAgentManager::GetInstance();
    if (!Manager)
    {
        return MakeCallActorResponse(false, TEXT("Agent manager unavailable"), TEXT(""));
    }
    AActor* TargetActor = Manager->GetAgent(ActorID);
    if (!TargetActor)
    {
        return MakeCallActorResponse(false, FString::Printf(TEXT("Actor not found: %s"), *ActorID), TEXT(""));
    }
    FCallParameters CallParameters;
    FString ParameterError;
    if (!ParseCallParameters(CommandJson, CallParameters, ParameterError))
    {
        return MakeCallActorResponse(false, ParameterError, TEXT(""));
    }
    FString ReturnValue;
    if (CallActorFunction(TargetActor, FunctionName, CallParameters, ReturnValue))
    {
        bool bNeedReturn = false;
        CommandJson->TryGetBoolField(TEXT("return"), bNeedReturn);
        return MakeCallActorResponse(
            true,
            FString::Printf(TEXT("Function %s called successfully"), *FunctionName),
            bNeedReturn ? ReturnValue : TEXT(""));
    }
    return MakeCallActorResponse(false, FString::Printf(TEXT("Failed to call %s on %s"), *FunctionName, *ActorID), TEXT(""));
}
bool FCommandHandle::CallActorFunction(
    AActor* TargetActor,
    const FString& FunctionName,
    const FCallParameters& Parameters,
    FString& OutReturnValue)
{
    if (!TargetActor)
    {
        return false;
    }
    UFunction* Function = TargetActor->GetClass()->FindFunctionByName(*FunctionName);
    if (!Function)
    {
        UE_LOG(LogTemp, Error, TEXT("[FCommandHandle] Function not found: %s on %s"), *FunctionName, *TargetActor->GetName());
        return false;
    }
    TArray<FProperty*> FunctionParams;
    FProperty* ReturnProp = nullptr;
    for (TFieldIterator<FProperty> PropIt(Function); PropIt; ++PropIt)
    {
        if (!(PropIt->PropertyFlags & CPF_Parm))
        {
            continue;
        }
        if (PropIt->PropertyFlags & CPF_ReturnParm)
        {
            ReturnProp = *PropIt;
        }
        else
        {
            FunctionParams.Add(*PropIt);
        }
    }
    if (!Parameters.bUseNamedParameters && FunctionParams.Num() != Parameters.PositionalParameters.Num())
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT("[FCommandHandle] Param count mismatch on %s. Expected %d, got %d"),
            *FunctionName,
            FunctionParams.Num(),
            Parameters.PositionalParameters.Num());
        return false;
    }

    void* ParamsBuffer = FMemory_Alloca(Function->ParmsSize);
    FMemory::Memzero(ParamsBuffer, Function->ParmsSize);

    TArray<FProperty*> InitializedProperties = FunctionParams;
    if (ReturnProp)
    {
        InitializedProperties.Add(ReturnProp);
    }
    for (FProperty* Property : InitializedProperties)
    {
        Property->InitializeValue_InContainer(ParamsBuffer);
    }

    for (int32 ParamIndex = 0; ParamIndex < FunctionParams.Num(); ++ParamIndex)
    {
        FProperty* Param = FunctionParams[ParamIndex];
        const FString* Value = nullptr;
        if (Parameters.bUseNamedParameters)
        {
            Value = FindNamedParameterValue(Parameters.NamedParameters, Param->GetName(), ParamIndex);
            if (!Value)
            {
                continue;
            }
        }
        else
        {
            Value = &Parameters.PositionalParameters[ParamIndex];
        }
        SetPropertyValue(Param, ParamsBuffer, *Value);
    }

    TargetActor->ProcessEvent(Function, ParamsBuffer);
    if (ReturnProp)
    {
        void* ReturnValuePtr = ReturnProp->ContainerPtrToValuePtr<void>(ParamsBuffer);
        OutReturnValue = ConvertPropertyToString(ReturnProp, ReturnValuePtr);
    }
    else
    {
        OutReturnValue = TEXT("");
    }

    for (const FProperty* Property : InitializedProperties)
    {
        Property->DestroyValue_InContainer(ParamsBuffer);
    }
    return true;
}
void FCommandHandle::SetPropertyValue(FProperty* Property, void* Container, const FString& Value)
{
    auto ParseTriple = [](const FString& InValue, const FString& Prefix, float& OutA, float& OutB, float& OutC) -> bool
    {
        FString Work = InValue;
        if (!Prefix.IsEmpty() && Work.StartsWith(Prefix))
        {
            Work = Work.RightChop(Prefix.Len());
        }
        TArray<FString> Components;
        Work.ParseIntoArray(Components, TEXT(","));
        if (Components.Num() != 3)
        {
            return false;
        }
        OutA = FCString::Atof(*Components[0]);
        OutB = FCString::Atof(*Components[1]);
        OutC = FCString::Atof(*Components[2]);
        return true;
    };
    if (FBoolProperty* BoolProp = CastField<FBoolProperty>(Property))
    {
        BoolProp->SetPropertyValue_InContainer(Container, Value.ToBool());
    }
    else if (FEnumProperty* EnumProp = CastField<FEnumProperty>(Property))
    {
        int64 EnumValue = 0;
        if (Value.IsNumeric())
        {
            EnumValue = FCString::Atoi64(*Value);
        }
        else if (UEnum* EnumDef = EnumProp->GetEnum())
        {
            int64 FoundValue = EnumDef->GetValueByNameString(Value);
            if (FoundValue == INDEX_NONE)
            {
                const FString ScopedName = FString::Printf(TEXT("%s::%s"), *EnumDef->GetName(), *Value);
                FoundValue = EnumDef->GetValueByNameString(ScopedName);
            }
            if (FoundValue != INDEX_NONE)
            {
                EnumValue = FoundValue;
            }
        }
        if (FNumericProperty* Underlying = EnumProp->GetUnderlyingProperty())
        {
            Underlying->SetIntPropertyValue(EnumProp->ContainerPtrToValuePtr<void>(Container), EnumValue);
        }
    }
    else if (FByteProperty* ByteProp = CastField<FByteProperty>(Property))
    {
        if (ByteProp->Enum && !Value.IsNumeric())
        {
            int64 FoundValue = ByteProp->Enum->GetValueByNameString(Value);
            if (FoundValue == INDEX_NONE)
            {
                const FString ScopedName = FString::Printf(TEXT("%s::%s"), *ByteProp->Enum->GetName(), *Value);
                FoundValue = ByteProp->Enum->GetValueByNameString(ScopedName);
            }
            if (FoundValue != INDEX_NONE)
            {
                ByteProp->SetPropertyValue_InContainer(Container, static_cast<uint8>(FoundValue));
                return;
            }
        }
        ByteProp->SetPropertyValue_InContainer(Container, static_cast<uint8>(FCString::Atoi(*Value)));
    }
    else if (FIntProperty* IntProp = CastField<FIntProperty>(Property))
    {
        IntProp->SetPropertyValue_InContainer(Container, FCString::Atoi(*Value));
    }
    else if (FInt64Property* Int64Prop = CastField<FInt64Property>(Property))
    {
        Int64Prop->SetPropertyValue_InContainer(Container, FCString::Atoi64(*Value));
    }
    else if (FFloatProperty* FloatProp = CastField<FFloatProperty>(Property))
    {
        FloatProp->SetPropertyValue_InContainer(Container, FCString::Atof(*Value));
    }
    else if (FDoubleProperty* DoubleProp = CastField<FDoubleProperty>(Property))
    {
        DoubleProp->SetPropertyValue_InContainer(Container, FCString::Atof(*Value));
    }
    else if (FStrProperty* StrProp = CastField<FStrProperty>(Property))
    {
        StrProp->SetPropertyValue_InContainer(Container, Value);
    }
    else if (FNameProperty* NameProp = CastField<FNameProperty>(Property))
    {
        NameProp->SetPropertyValue_InContainer(Container, FName(*Value));
    }
    else if (FTextProperty* TextProp = CastField<FTextProperty>(Property))
    {
        TextProp->SetPropertyValue_InContainer(Container, FText::FromString(Value));
    }
    else if (FStructProperty* StructProp = CastField<FStructProperty>(Property))
    {
        if (StructProp->Struct == TBaseStructure<FVector>::Get())
        {
            float X = 0.0f;
            float Y = 0.0f;
            float Z = 0.0f;
            if (ParseTriple(Value, TEXT("FVector:"), X, Y, Z) || ParseTriple(Value, TEXT(""), X, Y, Z))
            {
                const FVector VectorValue(X, Y, Z);
                StructProp->CopyCompleteValue(StructProp->ContainerPtrToValuePtr<void>(Container), &VectorValue);
            }
        }
        else if (StructProp->Struct == TBaseStructure<FRotator>::Get())
        {
            float Roll = 0.0f;
            float Pitch = 0.0f;
            float Yaw = 0.0f;
            if (ParseTriple(Value, TEXT("FRotator:"), Roll, Pitch, Yaw) ||
                ParseTriple(Value, TEXT("FVector:"), Roll, Pitch, Yaw) ||
                ParseTriple(Value, TEXT(""), Roll, Pitch, Yaw))
            {
                const FRotator RotatorValue(Pitch, Yaw, Roll);
                StructProp->CopyCompleteValue(StructProp->ContainerPtrToValuePtr<void>(Container), &RotatorValue);
            }
        }
    }
}
FString FCommandHandle::ConvertPropertyToString(FProperty* Prop, void* ValuePtr)
{
    if (FStrProperty* StrProp = CastField<FStrProperty>(Prop))
    {
        return StrProp->GetPropertyValue(ValuePtr);
    }
    if (FBoolProperty* BoolProp = CastField<FBoolProperty>(Prop))
    {
        return BoolProp->GetPropertyValue(ValuePtr) ? TEXT("true") : TEXT("false");
    }
    if (FFloatProperty* FloatProp = CastField<FFloatProperty>(Prop))
    {
        return FString::SanitizeFloat(FloatProp->GetPropertyValue(ValuePtr));
    }
    if (FDoubleProperty* DoubleProp = CastField<FDoubleProperty>(Prop))
    {
        return FString::SanitizeFloat(DoubleProp->GetPropertyValue(ValuePtr));
    }
    if (FIntProperty* IntProp = CastField<FIntProperty>(Prop))
    {
        return FString::FromInt(IntProp->GetPropertyValue(ValuePtr));
    }
    if (FInt64Property* Int64Prop = CastField<FInt64Property>(Prop))
    {
        return LexToString(Int64Prop->GetPropertyValue(ValuePtr));
    }
    if (FNameProperty* NameProp = CastField<FNameProperty>(Prop))
    {
        return NameProp->GetPropertyValue(ValuePtr).ToString();
    }
    if (FTextProperty* TextProp = CastField<FTextProperty>(Prop))
    {
        return TextProp->GetPropertyValue(ValuePtr).ToString();
    }
    if (FStructProperty* StructProp = CastField<FStructProperty>(Prop))
    {
        if (StructProp->Struct == TBaseStructure<FVector>::Get())
        {
            const FVector& VectorValue = *reinterpret_cast<const FVector*>(ValuePtr);
            return FString::Printf(TEXT("{\"x\":%.6f,\"y\":%.6f,\"z\":%.6f}"), VectorValue.X, VectorValue.Y, VectorValue.Z);
        }
        if (StructProp->Struct == TBaseStructure<FRotator>::Get())
        {
            const FRotator& RotatorValue = *reinterpret_cast<const FRotator*>(ValuePtr);
            return FString::Printf(TEXT("{\"roll\":%.6f,\"pitch\":%.6f,\"yaw\":%.6f}"), RotatorValue.Roll, RotatorValue.Pitch, RotatorValue.Yaw);
        }
    }
    return TEXT("unsupported_type");
}
FString FCommandHandle::MakeAddActorResponse(const FString& ActorID, bool bSuccess, const FString& Message)
{
    const TSharedPtr<FJsonObject> ResponseJson = MakeStatusResponseObject(bSuccess, Message);
    if (!ActorID.IsEmpty())
    {
        ResponseJson->SetStringField(TEXT("actor_id"), ActorID);
    }
    return SerializeWrappedResponse(TEXT("add_actor_return"), ResponseJson);
}
FString FCommandHandle::MakeRemoveActorResponse(bool bSuccess, const FString& Message)
{
    return SerializeWrappedResponse(TEXT("remove_actor_return"), MakeStatusResponseObject(bSuccess, Message));
}
FString FCommandHandle::MakeCallActorResponse(bool bSuccess, const FString& Message, const FString& ReturnValue)
{
    const TSharedPtr<FJsonObject> ResponseJson = MakeStatusResponseObject(bSuccess, Message);
    if (!ReturnValue.IsEmpty())
    {
        ResponseJson->SetStringField(TEXT("return"), ReturnValue);
    }
    return SerializeWrappedResponse(TEXT("call_actor_return"), ResponseJson);
}
FString FCommandHandle::MakeErrorResponse(const FString& ReturnType, const FString& Message)
{
    return SerializeWrappedResponse(ReturnType, MakeStatusResponseObject(false, Message));
}

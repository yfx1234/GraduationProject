#include "FCommandHandle.h"
#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Policies/CondensedJsonPrintPolicy.h"
#include "Engine/GameInstance.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "GraduationProject/Core/Manager/AgentManager.h"
#include "GraduationProject/Drone/DronePawn.h"
#include "UObject/UObjectGlobals.h"
#include "Kismet/GameplayStatics.h"

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
        TSharedRef<TJsonWriter<>> Writer = TJsonWriterFactory<>::Create(&Output);
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

        return ActorClass && ActorClass->IsChildOf(ADronePawn::StaticClass());
    }
    bool CollectCallParameters(const TSharedPtr<FJsonObject>& CommandJson, TArray<FString>& OutParameters, FString& OutError)
    {
        OutParameters.Reset();
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
            OutParameters.Reserve(Values.Num());
            for (const TSharedPtr<FJsonValue>& Value : Values)
            {
                OutParameters.Add(JsonValueToParamString(Value));
            }
            return true;
        }

        if (ParametersValue->Type == EJson::Object)
        {
            const TSharedPtr<FJsonObject> ParametersObject = ParametersValue->AsObject();
            if (!ParametersObject.IsValid())
            {
                return true;
            }

            struct FParamPair
            {
                FString Key;
                TSharedPtr<FJsonValue> Value;
                bool bHasIndex = false;
                int32 Index = 0;
            };

            TArray<FParamPair> Pairs;
            Pairs.Reserve(ParametersObject->Values.Num());
            for (const TPair<FString, TSharedPtr<FJsonValue>>& Pair : ParametersObject->Values)
            {
                FParamPair Item;
                Item.Key = Pair.Key;
                Item.Value = Pair.Value;
                Item.bHasIndex = TryParseOrderedKey(Pair.Key, Item.Index);
                Pairs.Add(MoveTemp(Item));
            }

            Pairs.Sort([](const FParamPair& A, const FParamPair& B)
            {
                if (A.bHasIndex != B.bHasIndex)
                {
                    return A.bHasIndex;
                }
                if (A.bHasIndex && B.bHasIndex && A.Index != B.Index)
                {
                    return A.Index < B.Index;
                }
                return A.Key < B.Key;
            });

            OutParameters.Reserve(Pairs.Num());
            for (const FParamPair& Pair : Pairs)
            {
                OutParameters.Add(JsonValueToParamString(Pair.Value));
            }
            return true;
        }

        OutError = TEXT("parameters must be array or object");
        return false;
    }
}
FCommandHandle::FCommandHandle(UGameInstance* InGameInstance)
    : GameInstance(InGameInstance)
{
}

FString FCommandHandle::HandleAddActor(const TSharedPtr<FJsonObject>& CommandJson)
{
    if (!CommandJson.IsValid() || !CommandJson->HasField(TEXT("classname")))
    {
        return MakeAddActorResponse(TEXT(""), false, TEXT("Missing classname"));
    }

    FString ClassNameOrPath = CommandJson->GetStringField(TEXT("classname"));
    UClass* ActorClass = nullptr;

    if (ClassNameOrPath.StartsWith(TEXT("Blueprint'/")) || ClassNameOrPath.StartsWith(TEXT("/Game/")))
    {
        ActorClass = LoadClass<AActor>(nullptr, *ClassNameOrPath);
    }
    else if (ClassNameOrPath.StartsWith(TEXT("/Script/")))
    {
        ActorClass = StaticLoadClass(UObject::StaticClass(), nullptr, *ClassNameOrPath);
    }
    else
    {
        FString FullClassName = FString::Printf(TEXT("/Script/GraduationProject.%s"), *ClassNameOrPath);
        ActorClass = StaticLoadClass(UObject::StaticClass(), nullptr, *FullClassName);
        if (!ActorClass)
        {
            FullClassName = FString::Printf(TEXT("/Script/Engine.%s"), *ClassNameOrPath);
            ActorClass = StaticLoadClass(UObject::StaticClass(), nullptr, *FullClassName);
        }
    }

    if (!ActorClass)
    {
        UE_LOG(LogTemp, Error, TEXT("[FCommandHandle] Class not found: %s"), *ClassNameOrPath);
        return MakeAddActorResponse(TEXT(""), false, FString::Printf(TEXT("Class not found: %s"), *ClassNameOrPath));
    }

    FVector SpawnPos = FVector::ZeroVector;
    FRotator SpawnRot = FRotator::ZeroRotator;

    const TSharedPtr<FJsonObject>* PoseJsonPtr = nullptr;
    if (CommandJson->TryGetObjectField(TEXT("pose"), PoseJsonPtr) && PoseJsonPtr && PoseJsonPtr->IsValid())
    {
        const TSharedPtr<FJsonObject>& PoseJson = *PoseJsonPtr;

        double Value = 0.0;
        if (PoseJson->TryGetNumberField(TEXT("x"), Value))
        {
            SpawnPos.X = static_cast<float>(Value);
        }
        if (PoseJson->TryGetNumberField(TEXT("y"), Value))
        {
            SpawnPos.Y = static_cast<float>(Value);
        }
        if (PoseJson->TryGetNumberField(TEXT("z"), Value))
        {
            SpawnPos.Z = static_cast<float>(Value);
        }

        double Roll = 0.0;
        double Pitch = 0.0;
        double Yaw = 0.0;
        PoseJson->TryGetNumberField(TEXT("roll"), Roll);
        PoseJson->TryGetNumberField(TEXT("pitch"), Pitch);
        PoseJson->TryGetNumberField(TEXT("yaw"), Yaw);
        SpawnRot = FRotator(static_cast<float>(Pitch), static_cast<float>(Yaw), static_cast<float>(Roll));
    }

    if (ShouldUseMetersForSpawn(CommandJson, ActorClass))
    {
        SpawnPos *= 100.0f;
    }

    UAgentManager* Manager = UAgentManager::GetInstance();

    FString ActorId;
    if (CommandJson->HasField(TEXT("expected_id")) && !CommandJson->GetStringField(TEXT("expected_id")).IsEmpty())
    {
        ActorId = CommandJson->GetStringField(TEXT("expected_id"));
    }
    else
    {
        ActorId = FString::Printf(TEXT("actor_%d"), FMath::RandRange(1000, 9999));
        while (Manager->GetAgent(ActorId) != nullptr)
        {
            ActorId = FString::Printf(TEXT("actor_%d"), FMath::RandRange(1000, 9999));
        }
    }

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

    AActor* NewActor = nullptr;

    if (ActorClass->IsChildOf(ADronePawn::StaticClass()))
    {
        const FTransform SpawnTransform(SpawnRot, SpawnPos);
        const TSubclassOf<ADronePawn> DroneClass = ActorClass;
        ADronePawn* SpawnedDrone = World->SpawnActorDeferred<ADronePawn>(
            DroneClass,
            SpawnTransform,
            nullptr,
            nullptr,
            ESpawnActorCollisionHandlingMethod::AlwaysSpawn);

        if (!SpawnedDrone)
        {
            UE_LOG(LogTemp, Error, TEXT("[FCommandHandle] Failed to spawn drone actor"));
            return MakeAddActorResponse(ActorId, false, TEXT("Failed to spawn actor"));
        }

        const EDroneMissionRole RequestedRole = ParseMissionRole(CommandJson);
        SpawnedDrone->DroneId = ActorId;
        SpawnedDrone->MissionRole = RequestedRole;
        UGameplayStatics::FinishSpawningActor(SpawnedDrone, SpawnTransform);

        // Guard against Blueprint construction defaults overwriting preassigned id/role.
        SpawnedDrone->DroneId = ActorId;
        SpawnedDrone->MissionRole = RequestedRole;
        NewActor = SpawnedDrone;
    }
    else
    {
        FActorSpawnParameters SpawnParams;
        SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
        NewActor = World->SpawnActor<AActor>(ActorClass, SpawnPos, SpawnRot, SpawnParams);
    }

    if (!NewActor)
    {
        UE_LOG(LogTemp, Error, TEXT("[FCommandHandle] Failed to spawn actor"));
        return MakeAddActorResponse(ActorId, false, TEXT("Failed to spawn actor"));
    }

    FString Label = CommandJson->HasField(TEXT("label"))
        ? CommandJson->GetStringField(TEXT("label"))
        : TEXT("Vehicle");
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

    AActor* TargetActor = UAgentManager::GetInstance()->GetAgent(ActorID);
    if (!TargetActor)
    {
        return MakeCallActorResponse(false, FString::Printf(TEXT("Actor not found: %s"), *ActorID), TEXT(""));
    }

    TArray<FString> Parameters;
    FString ParameterError;
    if (!CollectCallParameters(CommandJson, Parameters, ParameterError))
    {
        return MakeCallActorResponse(false, ParameterError, TEXT(""));
    }

    FString ReturnValue;
    if (CallActorFunction(TargetActor, FunctionName, Parameters, ReturnValue))
    {
        bool bNeedReturn = false;
        CommandJson->TryGetBoolField(TEXT("return"), bNeedReturn);
        
        if (bNeedReturn)
        {
            return MakeCallActorResponse(true, FString::Printf(TEXT("Function %s called successfully"), *FunctionName), ReturnValue);
        }
        return MakeCallActorResponse(true, FString::Printf(TEXT("Function %s called successfully"), *FunctionName), TEXT(""));
    }

    return MakeCallActorResponse(false, FString::Printf(TEXT("Failed to call %s on %s"), *FunctionName, *ActorID), TEXT(""));
}

bool FCommandHandle::CallActorFunction(AActor* TargetActor, const FString& FunctionName,
                                       const TArray<FString>& Parameters, FString& OutReturnValue)
{
    if (!TargetActor) return false;

    UClass* ActorClass = TargetActor->GetClass();
    UFunction* Function = ActorClass->FindFunctionByName(*FunctionName);

    if (!Function)
    {
        UE_LOG(LogTemp, Error, TEXT("[FCommandHandle] Function not found: %s on %s"), *FunctionName, *TargetActor->GetName());
        return false;
    }

    TArray<FProperty*> FunctionParams;
    for (TFieldIterator<FProperty> PropIt(Function); PropIt; ++PropIt)
    {
        if (PropIt->PropertyFlags & CPF_Parm)
        {
            FunctionParams.Add(*PropIt);
        }
    }

    // Remove return property from param processing
    if (FunctionParams.Num() > 0 && FunctionParams.Last()->PropertyFlags & CPF_OutParm &&
        FunctionParams.Last()->PropertyFlags & CPF_ReturnParm)
    {
        FunctionParams.Pop();
    }

    if (FunctionParams.Num() != Parameters.Num())
    {
        UE_LOG(LogTemp, Error, TEXT("[FCommandHandle] Param count mismatch on %s. Expected %d, got %d"),
               *FunctionName, FunctionParams.Num(), Parameters.Num());
        return false;
    }

    void* ParamsBuffer = FMemory_Alloca(Function->ParmsSize);
    FMemory::Memzero(ParamsBuffer, Function->ParmsSize);

    int32 ParamIndex = 0;
    for (FProperty* Param : FunctionParams)
    {
        SetPropertyValue(Param, ParamsBuffer, Parameters[ParamIndex]);
        ParamIndex++;
    }

    TargetActor->ProcessEvent(Function, ParamsBuffer);

    if (FProperty* ReturnProp = Function->GetReturnProperty())
    {
        void* ReturnValuePtr = ReturnProp->ContainerPtrToValuePtr<void>(ParamsBuffer);
        OutReturnValue = ConvertPropertyToString(ReturnProp, ReturnValuePtr);
    }
    else
    {
        OutReturnValue = TEXT("");
    }

    for (const FProperty* Param : FunctionParams)
    {
        if (!(Param->PropertyFlags & CPF_OutParm))
        {
            Param->DestroyValue_InContainer(ParamsBuffer);
        }
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
            if (ParseTriple(Value, TEXT("FRotator:"), Roll, Pitch, Yaw) || ParseTriple(Value, TEXT("FVector:"), Roll, Pitch, Yaw) || ParseTriple(Value, TEXT(""), Roll, Pitch, Yaw))
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
    if (FIntProperty* IntProp = CastField<FIntProperty>(Prop))
    {
        return FString::FromInt(IntProp->GetPropertyValue(ValuePtr));
    }
    return TEXT("unsupported_type");
}

FString FCommandHandle::MakeAddActorResponse(const FString& ActorID, bool bSuccess, const FString& Message)
{
    TSharedPtr<FJsonObject> ResponseJson = MakeShareable(new FJsonObject);
    ResponseJson->SetStringField(TEXT("actor_id"), ActorID);
    ResponseJson->SetBoolField(TEXT("status"), bSuccess);
    ResponseJson->SetStringField(TEXT("message"), Message);

    TSharedPtr<FJsonObject> FullResponse = MakeShareable(new FJsonObject);
    FullResponse->SetObjectField(TEXT("add_actor_return"), ResponseJson);

    FString OutputString;
    TSharedRef<TJsonWriter<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>> Writer = TJsonWriterFactory<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>::Create(&OutputString);
    FJsonSerializer::Serialize(FullResponse.ToSharedRef(), Writer);
    return OutputString;
}

FString FCommandHandle::MakeRemoveActorResponse(bool bSuccess, const FString& Message)
{
    TSharedPtr<FJsonObject> ResponseJson = MakeShareable(new FJsonObject);
    ResponseJson->SetBoolField(TEXT("status"), bSuccess);
    ResponseJson->SetStringField(TEXT("message"), Message);

    TSharedPtr<FJsonObject> FullResponse = MakeShareable(new FJsonObject);
    FullResponse->SetObjectField(TEXT("remove_actor_return"), ResponseJson);

    FString OutputString;
    TSharedRef<TJsonWriter<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>> Writer = TJsonWriterFactory<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>::Create(&OutputString);
    FJsonSerializer::Serialize(FullResponse.ToSharedRef(), Writer);
    return OutputString;
}

FString FCommandHandle::MakeCallActorResponse(bool bSuccess, const FString& Message, const FString& ReturnValue)
{
    TSharedPtr<FJsonObject> ResponseJson = MakeShareable(new FJsonObject);
    ResponseJson->SetBoolField(TEXT("status"), bSuccess);
    ResponseJson->SetStringField(TEXT("message"), Message);
    if (!ReturnValue.IsEmpty())
    {
        ResponseJson->SetStringField(TEXT("return"), ReturnValue);
    }

    TSharedPtr<FJsonObject> FullResponse = MakeShareable(new FJsonObject);
    FullResponse->SetObjectField(TEXT("call_actor_return"), ResponseJson);

    FString OutputString;
    TSharedRef<TJsonWriter<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>> Writer = TJsonWriterFactory<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>::Create(&OutputString);
    FJsonSerializer::Serialize(FullResponse.ToSharedRef(), Writer);
    return OutputString;
}

FString FCommandHandle::MakeErrorResponse(const FString& ReturnType, const FString& Message)
{
    TSharedPtr<FJsonObject> ResponseJson = MakeShareable(new FJsonObject);
    ResponseJson->SetBoolField(TEXT("status"), false);
    ResponseJson->SetStringField(TEXT("message"), Message);

    TSharedPtr<FJsonObject> FullResponse = MakeShareable(new FJsonObject);
    FullResponse->SetObjectField(ReturnType, ResponseJson);

    FString OutputString;
    TSharedRef<TJsonWriter<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>> Writer = TJsonWriterFactory<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>::Create(&OutputString);
    FJsonSerializer::Serialize(FullResponse.ToSharedRef(), Writer);
    return OutputString;
}









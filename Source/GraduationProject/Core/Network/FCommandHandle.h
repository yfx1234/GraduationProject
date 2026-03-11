#pragma once

#include "CoreMinimal.h"

/**
 * 通用命令处理器 (不依赖 UObject 反射宏)
 * 支持三种泛型指令:
 *   add_actor    — 在场景中动态生成 Actor
 *   remove_actor — 从场景中移除 Actor
 *   call_actor   — 通过 UE 反射调用 Actor 上的 UFUNCTION
 *
 * 所有指令均使用 JSON 格式, 由 CommandRouter 路由到此处理器
 */
class GRADUATIONPROJECT_API FCommandHandle
{
public:
	explicit FCommandHandle(UGameInstance* InGameInstance);
	~FCommandHandle() {}

	/** 处理 add_actor 指令 */
	FString HandleAddActor(const TSharedPtr<FJsonObject>& CommandJson);

	/** 处理 remove_actor 指令 */
	FString HandleRemoveActor(const TSharedPtr<FJsonObject>& CommandJson);

	/** 处理 call_actor 指令 */
	FString HandleCallActor(const TSharedPtr<FJsonObject>& CommandJson);

private:
	// ---- 反射调用核心 ----
	static bool CallActorFunction(AActor* TargetActor, const FString& FunctionName,
	                               const TArray<FString>& Parameters, FString& OutReturnValue);
	static void SetPropertyValue(FProperty* Property, void* Container, const FString& Value);
	static FString ConvertPropertyToString(FProperty* Prop, void* ValuePtr);

	// ---- JSON 响应构建 ----
	static FString MakeAddActorResponse(const FString& ActorID, bool bSuccess, const FString& Message);
	static FString MakeRemoveActorResponse(bool bSuccess, const FString& Message);
	static FString MakeCallActorResponse(bool bSuccess, const FString& Message, const FString& ReturnValue);
	static FString MakeErrorResponse(const FString& ReturnType, const FString& Message);

	/** Actor ID → Actor* 映射表 */
	TMap<FString, AActor*> ActorMap;

	/** GameInstance 指针, 用于获取 World */
	UGameInstance* GameInstance;
};

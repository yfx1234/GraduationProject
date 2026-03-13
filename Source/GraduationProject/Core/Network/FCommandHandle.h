#pragma once

#include "CoreMinimal.h"

class FJsonObject;
class UAgentManager;

/**
 * @brief 通用 Actor 命令处理器
 *
 * 支持三类操作：
 * - add_actor  ：在场景中动态生成指定类型的 Actor 并注册到 AgentManager
 * - remove_actor：销毁已注册的 Actor
 * - call_actor ：通过 UE 反射调用 Actor 的 UFUNCTION 方法
 *
 * 通过 JSON 参数实现位置参数 / 命名参数的自动绑定，
 * 支持 bool / int / float / FString / FVector / FRotator / 枚举等类型自动转换。
 */
class GRADUATIONPROJECT_API FCommandHandle
{
public:
    explicit FCommandHandle(UGameInstance* InGameInstance);
    ~FCommandHandle() {}

    /**
     * @brief 命令分派入口——根据 JSON 字段转发到 Add / Remove / Call
     * @return 非空表示已处理，空字符串表示未匹配
     */
    FString HandleCommand(const TSharedPtr<FJsonObject>& RootJson);

    /** @brief 动态生成 Actor：解析类名、位姿、ID，生成并注册 */
    FString HandleAddActor(const TSharedPtr<FJsonObject>& CommandJson);

    /** @brief 销毁已注册 Actor并清理所有别名绑定 */
    FString HandleRemoveActor(const TSharedPtr<FJsonObject>& CommandJson);

    /** @brief 通过 UE 反射调用 Actor 的指定 UFUNCTION */
    FString HandleCallActor(const TSharedPtr<FJsonObject>& CommandJson);

private:
    /** @brief 调用参数包装：支持位置或命名两种方式 */
    struct FCallParameters
    {
        bool bUseNamedParameters = false;                ///< 是否使用命名参数模式
        TArray<FString> PositionalParameters;            ///< 位置参数列表
        TMap<FString, FString> NamedParameters;          ///< 命名参数映射
    };

    // ──── 类型解析 ────

    /** @brief 解析 Actor 类：支持简名 / 蓝图路径 / 完整 Script 路径 */
    static UClass* ResolveActorClass(const FString& ClassNameOrPath);

    /** @brief 从 JSON 读取生成位置与旋转 */
    static void ReadSpawnPose(const TSharedPtr<FJsonObject>& CommandJson, FVector& OutSpawnPos, FRotator& OutSpawnRot);

    /** @brief 解析 Actor ID：使用 expected_id 或自动生成唯一 ID */
    static FString ResolveActorId(const TSharedPtr<FJsonObject>& CommandJson, UAgentManager* Manager);

    // ──── 参数解析 ────

    /** @brief 从 JSON 解析位置/命名参数 */
    static bool ParseCallParameters(
        const TSharedPtr<FJsonObject>& CommandJson,
        FCallParameters& OutCallParameters,
        FString& OutError);

    // ──── 反射调用 ────

    /**
     * @brief 通过 UE 反射机制调用 Actor 的 UFUNCTION
     *        自动绑定参数并获取返回值
     */
    static bool CallActorFunction(
        AActor* TargetActor,
        const FString& FunctionName,
        const FCallParameters& Parameters,
        FString& OutReturnValue);

    /** @brief 根据属性类型设置属性值（支持 bool/int/float/FString/FVector/FRotator/枚举） */
    static void SetPropertyValue(FProperty* Property, void* Container, const FString& Value);

    /** @brief 将属性值转换为字符串（用于返回值） */
    static FString ConvertPropertyToString(FProperty* Prop, void* ValuePtr);

    // ──── JSON 响应构建 ────

    static FString MakeAddActorResponse(const FString& ActorID, bool bSuccess, const FString& Message);
    static FString MakeRemoveActorResponse(bool bSuccess, const FString& Message);
    static FString MakeCallActorResponse(bool bSuccess, const FString& Message, const FString& ReturnValue);
    static FString MakeErrorResponse(const FString& ReturnType, const FString& Message);

    UGameInstance* GameInstance;  ///< UE GameInstance 引用，用于获取 World
};

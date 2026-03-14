// 解释：使用 `#pragma once` 防止该头文件在编译过程中被重复包含。
#pragma once

// 解释：引入 Unreal 的核心基础头文件，提供常用容器、数学类型和日志宏。
#include "CoreMinimal.h"

// 解释：这一行声明 类 `FJsonObject`，用于封装fjsonobject相关的数据与行为。
class FJsonObject;
// 解释：这一行声明 类 `UAgentManager`，用于封装uagent管理器相关的数据与行为。
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
// 解释：这一行声明 类 `FCommandHandle`，用于封装fcommandhandle相关的数据与行为。
class GRADUATIONPROJECT_API FCommandHandle
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
// 解释：从这一行开始进入 `public` 访问区，下面的成员会对外部模块公开。
public:
    // 解释：调用 `FCommandHandle` 执行当前步骤需要的功能逻辑。
    explicit FCommandHandle(UGameInstance* InGameInstance);
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    ~FCommandHandle() {}

    /**
     * @brief 命令分派入口——根据 JSON 字段转发到 Add / Remove / Call
     * @return 非空表示已处理，空字符串表示未匹配
     */
    // 解释：调用 `HandleCommand` 执行当前步骤需要的功能逻辑。
    FString HandleCommand(const TSharedPtr<FJsonObject>& RootJson);

    /** @brief 动态生成 Actor：解析类名、位姿、ID，生成并注册 */
    // 解释：调用 `HandleAddActor` 执行当前步骤需要的功能逻辑。
    FString HandleAddActor(const TSharedPtr<FJsonObject>& CommandJson);

    /** @brief 销毁已注册 Actor并清理所有别名绑定 */
    // 解释：调用 `HandleRemoveActor` 执行当前步骤需要的功能逻辑。
    FString HandleRemoveActor(const TSharedPtr<FJsonObject>& CommandJson);

    /** @brief 通过 UE 反射调用 Actor 的指定 UFUNCTION */
    // 解释：调用 `HandleCallActor` 执行当前步骤需要的功能逻辑。
    FString HandleCallActor(const TSharedPtr<FJsonObject>& CommandJson);

// 解释：从这一行开始进入 `private` 访问区，下面的成员只允许类内部访问。
private:
    /** @brief 调用参数包装：支持位置或命名两种方式 */
    // 解释：这一行声明 结构体 `FCallParameters`，用于封装fcall参数相关的数据与行为。
    struct FCallParameters
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行声明成员或局部变量 `bUseNamedParameters`，用于保存布尔标志 usenamed参数。
        bool bUseNamedParameters = false;                ///< 是否使用命名参数模式
        // 解释：这一行声明成员或局部变量 `PositionalParameters`，用于保存positional参数。
        TArray<FString> PositionalParameters;            ///< 位置参数列表
        // 解释：这一行声明成员或局部变量 `NamedParameters`，用于保存named参数。
        TMap<FString, FString> NamedParameters;          ///< 命名参数映射
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    };

    // ──── 类型解析 ────

    /** @brief 解析 Actor 类：支持简名 / 蓝图路径 / 完整 Script 路径 */
    // 解释：调用 `ResolveActorClass` 执行当前步骤需要的功能逻辑。
    static UClass* ResolveActorClass(const FString& ClassNameOrPath);

    /** @brief 从 JSON 读取生成位置与旋转 */
    // 解释：调用 `ReadSpawnPose` 执行当前步骤需要的功能逻辑。
    static void ReadSpawnPose(const TSharedPtr<FJsonObject>& CommandJson, FVector& OutSpawnPos, FRotator& OutSpawnRot);

    /** @brief 解析 Actor ID：使用 expected_id 或自动生成唯一 ID */
    // 解释：调用 `ResolveActorId` 执行当前步骤需要的功能逻辑。
    static FString ResolveActorId(const TSharedPtr<FJsonObject>& CommandJson, UAgentManager* Manager);

    // ──── 参数解析 ────

    /** @brief 从 JSON 解析位置/命名参数 */
    // 解释：这一行定义函数 `ParseCallParameters`，开始实现parsecall参数的具体逻辑。
    static bool ParseCallParameters(
        // 解释：这一行继续展开 `ParseCallParameters` 的参数列表，声明参数 `CommandJson` 用于传入命令json。
        const TSharedPtr<FJsonObject>& CommandJson,
        // 解释：这一行继续展开 `ParseCallParameters` 的参数列表，声明参数 `OutCallParameters` 用于传入outcall参数。
        FCallParameters& OutCallParameters,
        // 解释：这一行继续补充函数 `ParseCallParameters` 的参数列表、限定符或返回类型说明。
        FString& OutError);

    // ──── 反射调用 ────

    /**
     * @brief 通过 UE 反射机制调用 Actor 的 UFUNCTION
     *        自动绑定参数并获取返回值
     */
    // 解释：这一行定义函数 `CallActorFunction`，开始实现callActorfunction的具体逻辑。
    static bool CallActorFunction(
        // 解释：这一行继续展开 `CallActorFunction` 的参数列表，声明参数 `TargetActor` 用于传入targetActor。
        AActor* TargetActor,
        // 解释：这一行继续展开 `CallActorFunction` 的参数列表，声明参数 `FunctionName` 用于传入functionname。
        const FString& FunctionName,
        // 解释：这一行继续展开 `CallActorFunction` 的参数列表，声明参数 `Parameters` 用于传入参数。
        const FCallParameters& Parameters,
        // 解释：这一行继续补充函数 `CallActorFunction` 的参数列表、限定符或返回类型说明。
        FString& OutReturnValue);

    /** @brief 根据属性类型设置属性值（支持 bool/int/float/FString/FVector/FRotator/枚举） */
    // 解释：调用 `SetPropertyValue` 执行当前步骤需要的功能逻辑。
    static void SetPropertyValue(FProperty* Property, void* Container, const FString& Value);

    /** @brief 将属性值转换为字符串（用于返回值） */
    // 解释：调用 `ConvertPropertyToString` 执行当前步骤需要的功能逻辑。
    static FString ConvertPropertyToString(FProperty* Prop, void* ValuePtr);

    // ──── JSON 响应构建 ────

    // 解释：调用 `MakeAddActorResponse` 执行当前步骤需要的功能逻辑。
    static FString MakeAddActorResponse(const FString& ActorID, bool bSuccess, const FString& Message);
    // 解释：调用 `MakeRemoveActorResponse` 执行当前步骤需要的功能逻辑。
    static FString MakeRemoveActorResponse(bool bSuccess, const FString& Message);
    // 解释：调用 `MakeCallActorResponse` 执行当前步骤需要的功能逻辑。
    static FString MakeCallActorResponse(bool bSuccess, const FString& Message, const FString& ReturnValue);
    // 解释：调用 `MakeErrorResponse` 执行当前步骤需要的功能逻辑。
    static FString MakeErrorResponse(const FString& ReturnType, const FString& Message);

    // 解释：这一行声明成员或局部变量 `GameInstance`，用于保存gameinstance。
    UGameInstance* GameInstance;  ///< UE GameInstance 引用，用于获取 World
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
};

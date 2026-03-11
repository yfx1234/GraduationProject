#pragma once

#include "CoreMinimal.h"

/**
 * @brief 通用 Actor 命令处理器
 * 不依赖 UObject 反射生命周期，专门负责解析 `add_actor`、`remove_actor`、`call_actor`
 * 三类 JSON 命令，并把网络层请求转换为场景对象创建、销毁与反射调用。
 */
class GRADUATIONPROJECT_API FCommandHandle
{
public:
    /**
     * @brief 构造命令处理器
     * @param InGameInstance 当前游戏实例，用于获取 World 与场景上下文
     */
    explicit FCommandHandle(UGameInstance* InGameInstance);

    /** @brief 析构函数，当前无额外资源需要主动释放 */
    ~FCommandHandle() {}

    /**
     * @brief 处理 `add_actor` 命令
     * @param CommandJson `add_actor` 对应的 JSON 对象
     * @return JSON 格式执行结果
     */
    FString HandleAddActor(const TSharedPtr<FJsonObject>& CommandJson);

    /**
     * @brief 处理 `remove_actor` 命令
     * @param CommandJson `remove_actor` 对应的 JSON 对象
     * @return JSON 格式执行结果
     */
    FString HandleRemoveActor(const TSharedPtr<FJsonObject>& CommandJson);

    /**
     * @brief 处理 `call_actor` 命令
     * @param CommandJson `call_actor` 对应的 JSON 对象
     * @return JSON 格式执行结果
     */
    FString HandleCallActor(const TSharedPtr<FJsonObject>& CommandJson);

private:
    /**
     * @brief 通过 Unreal 反射调用 Actor 成员函数
     * @param TargetActor 目标 Actor
     * @param FunctionName 函数名
     * @param Parameters 字符串化参数列表
     * @param OutReturnValue 输出返回值字符串
     * @return 调用成功时返回 `true`
     */
    static bool CallActorFunction(AActor* TargetActor, const FString& FunctionName,
                                  const TArray<FString>& Parameters, FString& OutReturnValue);

    /**
     * @brief 将字符串参数写入指定属性
     * @param Property 目标属性描述
     * @param Container 参数缓冲区或对象实例
     * @param Value 字符串形式的输入值
     */
    static void SetPropertyValue(FProperty* Property, void* Container, const FString& Value);

    /**
     * @brief 将属性值转换成字符串形式
     * @param Prop 属性描述
     * @param ValuePtr 指向属性值的内存地址
     * @return 序列化后的字符串表示
     */
    static FString ConvertPropertyToString(FProperty* Prop, void* ValuePtr);

    /**
     * @brief 构造 `add_actor` 成功/失败响应
     * @param ActorID 新建 Actor 的 ID
     * @param bSuccess 是否成功
     * @param Message 附加说明
     * @return 紧凑 JSON 响应字符串
     */
    static FString MakeAddActorResponse(const FString& ActorID, bool bSuccess, const FString& Message);

    /**
     * @brief 构造 `remove_actor` 响应
     * @param bSuccess 是否成功
     * @param Message 附加说明
     * @return 紧凑 JSON 响应字符串
     */
    static FString MakeRemoveActorResponse(bool bSuccess, const FString& Message);

    /**
     * @brief 构造 `call_actor` 响应
     * @param bSuccess 是否成功
     * @param Message 附加说明
     * @param ReturnValue 函数返回值字符串
     * @return 紧凑 JSON 响应字符串
     */
    static FString MakeCallActorResponse(bool bSuccess, const FString& Message, const FString& ReturnValue);

    /**
     * @brief 构造统一错误响应
     * @param ReturnType 返回值类型说明
     * @param Message 错误信息
     * @return 紧凑 JSON 响应字符串
     */
    static FString MakeErrorResponse(const FString& ReturnType, const FString& Message);

    /** @brief 维护 Agent ID 到 Actor 实例的临时映射 */
    TMap<FString, AActor*> ActorMap;

    /** @brief 游戏实例指针，用于访问 World 和生成对象 */
    UGameInstance* GameInstance;
};

#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "GuidanceCommandHandler.generated.h"

class UKalmanPredictor;
class IGuidanceMethod;

/**
 * 制导算法 TCP 命令处理器
 * 管理 IGuidanceMethod
 * 处理 call_guidance 命令
 * 处理 get_guidance_state 命令
 */
UCLASS()
class GRADUATIONPROJECT_API UGuidanceCommandHandler : public UObject
{
    GENERATED_BODY()

public:
    UGuidanceCommandHandler();
    ~UGuidanceCommandHandler();

    /**
     * @brief 处理 call_guidance 命令
     * @param JsonObject 完整的 JSON 请求对象
     * @param World 当前 UWorld 指针
     * @return JSON 格式的响应字符串
     */
    FString HandleCallGuidance(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World);

    /**
     * @brief 处理 get_guidance_state 命令
     * @param JsonObject 完整的 JSON 请求对象
     * @param World 当前 UWorld 指针
     * @return JSON 格式的制导状态数据
     */
    FString HandleGetGuidanceState(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World);

private:
    /** @brief 初始化 */
    void EnsureInitialized();

    /** @brief 卡尔曼滤波目标预测器 */
    UPROPERTY()
    UKalmanPredictor* Predictor = nullptr;

    /** @brief 当前制导方法 */
    IGuidanceMethod* CurrentMethod = nullptr;

    /** @brief 当前制导方法名称 */
    FString CurrentMethodName;

    /** @brief 最近一次制导输出参数 */
    float LastPitch = 0.0f;       
    float LastYaw = 0.0f;         
    FVector LastAimPoint = FVector::ZeroVector;  
    float LastFlightTime = 0.0f;  

    /**
     * @brief 生成错误响应 JSON
     * @param Msg 错误信息
     * @return {"status":"error","message":"<Msg>"}
     */
    FString MakeError(const FString& Msg);

    /**
     * @brief 生成成功响应 JSON
     * @param Msg 成功信息，默认 "ok"
     * @return {"status":"ok","message":"<Msg>"}
     */
    FString MakeOk(const FString& Msg = TEXT("ok"));
};

/**
 * @file GuidanceCommandHandler.h
 * @brief 制导 TCP 命令处理器的头文件
 *
 * 定义 UGuidanceCommandHandler 类，处理：
 * - call_guidance: 设置制导方法、弹速、是否射击等
 * - get_guidance_state: 查询当前制导输出和目标状态
 */

#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "GuidanceCommandHandler.generated.h"

class UKalmanPredictor;
class IGuidanceMethod;

/**
 * 制导算法 TCP 命令处理器
 * 管理卡尔曼预测器和制导方法，处理 call_guidance / get_guidance_state 命令
 */
UCLASS()
class GRADUATIONPROJECT_API UGuidanceCommandHandler : public UObject
{
    GENERATED_BODY()

public:
    UGuidanceCommandHandler();
    ~UGuidanceCommandHandler();

    /** 处理 call_guidance 命令 */
    FString HandleCallGuidance(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World);

    /** 处理 get_guidance_state 命令 */
    FString HandleGetGuidanceState(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World);

private:
    void EnsureInitialized();

    // 卡尔曼预测器
    UPROPERTY()
    UKalmanPredictor* Predictor = nullptr;

    // 当前制导方法（非 UObject，手动管理生命周期）
    IGuidanceMethod* CurrentMethod = nullptr;
    FString CurrentMethodName;

    // 最近一次制导输出（缓存用于查询）
    float LastPitch = 0.0f;
    float LastYaw = 0.0f;
    FVector LastAimPoint = FVector::ZeroVector;
    float LastFlightTime = 0.0f;

    FString MakeError(const FString& Msg);
    FString MakeOk(const FString& Msg = TEXT("ok"));
};

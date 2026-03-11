#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "../Drone/DronePawn.h"
#include "GuidanceCommandHandler.generated.h"

class UKalmanPredictor;
class IGuidanceMethod;
class UAgentManager;
class ADronePawn;
class UVisualInterceptController;

/**
 * @brief 制导命令处理器
 * 负责接收网络层的制导相关 JSON 请求，并驱动炮台瞄准、卡尔曼预测、
 * 拦截无人机速度规划以及视觉拦截控制器等上层逻辑。
 */
UCLASS()
class GRADUATIONPROJECT_API UGuidanceCommandHandler : public UObject
{
    GENERATED_BODY()

public:
    /** @brief 构造制导命令处理器 */
    UGuidanceCommandHandler();

    /** @brief 析构处理器，释放非 UObject 指针等临时资源 */
    ~UGuidanceCommandHandler();

    /**
     * @brief 处理 `call_guidance` 命令
     * @param JsonObject 已解析的命令 JSON
     * @param World 当前场景 World
     * @return JSON 格式执行结果
     */
    FString HandleCallGuidance(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World);

    /**
     * @brief 处理 `get_guidance_state` 命令
     * @param JsonObject 已解析的命令 JSON
     * @param World 当前场景 World
     * @return 当前制导状态 JSON
     */
    FString HandleGetGuidanceState(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World);

private:
    /** @brief 延迟创建预测器、控制器和默认算法对象 */
    void EnsureInitialized();

    /**
     * @brief 按任务角色查找无人机
     * @param Manager Agent 管理器
     * @param Role 需要匹配的任务角色
     * @param ExcludeId 需要排除的无人机 ID
     * @return 匹配到的无人机实例
     */
    ADronePawn* FindDroneByRole(UAgentManager* Manager, EDroneMissionRole Role, const FString& ExcludeId = TEXT("")) const;

    /** @brief 卡尔曼目标预测器 */
    UPROPERTY()
    UKalmanPredictor* Predictor = nullptr;

    /** @brief 视觉拦截控制器 */
    UPROPERTY()
    UVisualInterceptController* VisualInterceptController = nullptr;

    /** @brief 当前制导算法实例 */
    IGuidanceMethod* CurrentMethod = nullptr;

    /** @brief 当前制导算法名称 */
    FString CurrentMethodName;

    /** @brief 上一次输出的俯仰角命令 */
    float LastPitch = 0.0f;

    /** @brief 上一次输出的偏航角命令 */
    float LastYaw = 0.0f;

    /** @brief 上一次瞄准点 */
    FVector LastAimPoint = FVector::ZeroVector;

    /** @brief 上一次估算弹丸飞行时间 */
    float LastFlightTime = 0.0f;

    /** @brief 默认视觉链路时延补偿 */
    float DefaultVisionLatency = 0.08f;

    /** @brief 上一次实际使用的时延补偿 */
    float LastLatencyCompensation = 0.08f;

    /** @brief 当前无人机拦截算法名称 */
    FString CurrentInterceptMethod = TEXT("pure_pursuit");

    /** @brief 拦截无人机期望速度 */
    float InterceptorSpeed = 8.0f;

    /** @brief 拦截导航增益 */
    float InterceptNavGain = 3.0f;

    /** @brief 目标超前时间 */
    float InterceptLeadTime = 0.6f;

    /** @brief 判定捕获成功的距离阈值 */
    float CaptureRadius = 1.5f;

    /** @brief 最近一次参与拦截的拦截机 ID */
    FString LastInterceptorId;

    /** @brief 最近一次参与拦截的目标机 ID */
    FString LastTargetId;

    /** @brief 最近一次输出给拦截机的速度命令 */
    FVector LastInterceptorCmdVel = FVector::ZeroVector;

    /** @brief 最近一次拦截距离 */
    float LastDistanceToTarget = 0.0f;

    /** @brief 最近一次闭合速度 */
    float LastClosingSpeed = 0.0f;

    /** @brief 最近一次拦截计算是否有效 */
    bool bLastInterceptValid = false;

    /** @brief 最近一次是否判定捕获成功 */
    bool bLastCaptured = false;

    /** @brief 构造错误响应 JSON */
    FString MakeError(const FString& Msg);

    /** @brief 构造成功响应 JSON */
    FString MakeOk(const FString& Msg = TEXT("ok"));
};

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "GraduationProject/Drone/DronePawn.h"
#include "GuidanceActor.generated.h"

class IGuidanceMethod;
class UKalmanPredictor;
class UVisualInterceptController;
class UAgentManager;

/**
 * @brief 制导与拦截协调 Actor
 *
 * 该类统一封装三类能力：
 * 1. 常规弹道/瞄准制导算法的管理与调用；
 * 2. 基于 Kalman 预测的目标状态估计；
 * 3. 无人机自动拦截与视觉拦截控制接口。
 *
 * 因此它既可以服务炮塔自动瞄准，也可以为无人机追踪/拦截任务提供统一的 Blueprint 与 TCP 调用入口。
 */
UCLASS()
class GRADUATIONPROJECT_API AGuidanceActor : public AActor
{
    GENERATED_BODY()

public:
    /** @brief 构造函数：创建一个隐藏且无碰撞的后台协调 Actor */
    AGuidanceActor();

    /** @brief 注册自身到 AgentManager，便于外部通过 GuidanceId 查找 */
    virtual void BeginPlay() override;

    /** @brief 结束时释放制导算法对象，并从 AgentManager 注销自身 */
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

    /** @brief 重置预测器、制导器和历史输出状态 */
    UFUNCTION(BlueprintCallable, Category = "Guidance")
    FString ResetGuidance();

    /** @brief 获取当前 GuidanceActor 的状态 JSON */
    UFUNCTION(BlueprintCallable, Category = "Guidance")
    FString GetState();

    /**
     * @brief 切换常规制导算法
     * @param Method 制导方法名，支持 `direct`、`proportional`、`predictive`
     * @param NavConstant 比例导引导航常数，仅 `proportional` 模式使用
     * @param Iterations 预测制导迭代次数，仅 `predictive` 模式使用
     */
    UFUNCTION(BlueprintCallable, Category = "Guidance")
    FString SetMethod(FString Method, float NavConstant = 4.0f, int32 Iterations = 3);

    /**
     * @brief 设置无人机自动拦截算法的默认参数
     * @param Method 拦截方法名，支持 `pure_pursuit` / `proportional_nav` 及其别名
     * @param Speed 拦截机速度上限（m/s）
     * @param NavGain 比例导航增益 N
     * @param LeadTime 目标前置预测时间（s）
     * @param CaptureRadiusValue 捕获半径（m）
     */
    UFUNCTION(BlueprintCallable, Category = "Guidance")
    FString SetInterceptMethod(FString Method = TEXT("pure_pursuit"), float Speed = -1.0f, float NavGain = -1.0f, float LeadTime = -1.0f, float CaptureRadiusValue = -1.0f);

    /** @brief 列出当前场景中已注册的目标机和拦截机 ID */
    UFUNCTION(BlueprintCallable, Category = "Guidance")
    FString ListInterceptAgents();

    /**
     * @brief 执行一步自动拦截控制并直接下发指令
     * @param InterceptorId 拦截无人机 ID，可留空自动查找
     * @param TargetId 目标无人机 ID，可留空自动查找
     * @param Method 本次调用临时覆盖的拦截方法
     * @param Speed 本次调用临时覆盖的速度上限（m/s）
     * @param NavGain 本次调用临时覆盖的导航增益
     * @param LeadTime 本次调用临时覆盖的前置预测时间（s）
     * @param CaptureRadiusValue 本次调用临时覆盖的捕获半径（m）
     * @param bStopOnCapture 捕获后是否自动悬停
     */
    UFUNCTION(BlueprintCallable, Category = "Guidance")
    FString AutoIntercept(
        FString InterceptorId = TEXT(""),
        FString TargetId = TEXT(""),
        FString Method = TEXT(""),
        float Speed = -1.0f,
        float NavGain = -1.0f,
        float LeadTime = -1.0f,
        float CaptureRadiusValue = -1.0f,
        bool bStopOnCapture = true);

    /**
     * @brief 向 Kalman 预测器输入一帧目标位置观测
     * @param X 观测位置 X
     * @param Y 观测位置 Y
     * @param Z 观测位置 Z
     * @param Dt 采样时间间隔（s）
     */
    UFUNCTION(BlueprintCallable, Category = "Guidance")
    FString UpdateTarget(float X, float Y, float Z, float Dt = 0.1f);

    /**
     * @brief 根据当前目标预测状态计算炮塔瞄准角
     * @param TurretId 炮塔 ID
     * @param MuzzleSpeed 弹丸初速度（m/s）
     */
    UFUNCTION(BlueprintCallable, Category = "Guidance")
    FString ComputeAim(FString TurretId = TEXT("turret_0"), float MuzzleSpeed = 400.0f);

    /**
     * @brief 一步式自动瞄准/射击接口
     * @param TurretId 炮塔 ID
     * @param TargetId 目标 Actor ID
     * @param MuzzleSpeed 弹丸初速度（m/s）
     * @param Dt 观测刷新周期（s）
     * @param Latency 视觉/通信延迟补偿（s），小于 0 时使用默认值
     * @param bFire 是否在完成瞄准后触发开火
     */
    UFUNCTION(BlueprintCallable, Category = "Guidance")
    FString AutoEngage(FString TurretId = TEXT("turret_0"), FString TargetId = TEXT("drone_0"), float MuzzleSpeed = 400.0f, float Dt = 0.05f, float Latency = -1.0f, bool bFire = false);

    /**
     * @brief 设置 Kalman 预测器参数
     * @param ProcessNoise 过程噪声协方差系数
     * @param MeasurementNoise 观测噪声协方差系数
     */
    UFUNCTION(BlueprintCallable, Category = "Guidance")
    FString SetKalmanParams(float ProcessNoise = 1.0f, float MeasurementNoise = 0.5f);

    /**
     * @brief 启动视觉拦截会话
     *
     * 该接口把 Blueprint/TCP 参数整理为 JSON 命令，再转交给 `UVisualInterceptController`。
     */
    UFUNCTION(BlueprintCallable, Category = "Guidance|Visual")
    FString VisualInterceptStart(
        FString InterceptorId = TEXT(""),
        FString TargetId = TEXT(""),
        FString Method = TEXT("vision_pid_kalman"),
        float DesiredArea = -1.0f,
        float CaptureArea = -1.0f,
        float CenterTolX = -1.0f,
        float CenterTolY = -1.0f,
        int32 CaptureHoldFrames = -1,
        int32 LostToSearchFrames = -1,
        float MaxForwardSpeed = -1.0f,
        float MaxReverseSpeed = -1.0f,
        float MaxVerticalSpeed = -1.0f,
        float MaxYawRateDeg = -1.0f,
        float SearchCamYawLimitDeg = -1.0f,
        float SearchCamRateDeg = -1.0f,
        float SearchBodyYawRateDeg = -1.0f,
        float SearchCamPitchDeg = -1000.0f,
        float SearchVzAmp = -1.0f,
        int32 StopOnCaptureFlag = -1,
        int32 UseKalmanFlag = -1);

    /**
     * @brief 向视觉拦截控制器提交一帧检测结果
     *
     * 输入检测框中心、面积、置信度以及图像尺寸，由视觉控制器输出机体速度和云台控制量。
     */
    UFUNCTION(BlueprintCallable, Category = "Guidance|Visual")
    FString VisualInterceptUpdate(
        int32 HasDetection = 0,
        float Cx = 0.0f,
        float Cy = 0.0f,
        float Area = 0.0f,
        float AreaRatio = -1.0f,
        float Conf = 1.0f,
        float Dt = 0.08f,
        float ImageW = 640.0f,
        float ImageH = 480.0f,
        FString InterceptorId = TEXT(""),
        FString TargetId = TEXT(""));

    /** @brief 停止视觉拦截，并请求拦截无人机悬停 */
    UFUNCTION(BlueprintCallable, Category = "Guidance|Visual")
    FString VisualInterceptStop(FString InterceptorId = TEXT(""), FString TargetId = TEXT(""));

    /** @brief 查询视觉拦截控制器当前状态 */
    UFUNCTION(BlueprintCallable, Category = "Guidance|Visual")
    FString VisualInterceptState();

    /** @brief GuidanceActor 的唯一注册 ID */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Guidance")
    FString GuidanceId = TEXT("guidance_0");

private:
    /** @brief 惰性初始化预测器、视觉拦截控制器与默认制导算法 */
    void EnsureInitialized();

    /**
     * @brief 按任务角色查找无人机
     * @param Manager AgentManager 实例
     * @param DesiredRole 目标任务角色
     * @param ExcludeId 需要排除的无人机 ID
     */
    ADronePawn* FindDroneByRole(UAgentManager* Manager, EDroneMissionRole DesiredRole, const FString& ExcludeId = TEXT("")) const;

    /** @brief 构造错误 JSON 字符串 */
    FString MakeError(const FString& Msg) const;

    /** @brief 构造成功 JSON 字符串 */
    FString MakeOk(const FString& Msg = TEXT("ok")) const;

    /** @brief 目标状态 Kalman 预测器 */
    UPROPERTY()
    UKalmanPredictor* Predictor = nullptr;

    /** @brief 视觉拦截控制器 */
    UPROPERTY()
    UVisualInterceptController* VisualInterceptController = nullptr;

    /** @brief 当前启用的常规制导算法对象，生命周期由本类管理 */
    IGuidanceMethod* CurrentMethod = nullptr;

    /** @brief 当前制导算法名称 */
    FString CurrentMethodName = TEXT("predictive");

    /** @brief 最近一次输出的俯仰瞄准角（deg） */
    float LastPitch = 0.0f;

    /** @brief 最近一次输出的偏航瞄准角（deg） */
    float LastYaw = 0.0f;

    /** @brief 最近一次计算得到的瞄准点 */
    FVector LastAimPoint = FVector::ZeroVector;

    /** @brief 最近一次估计的弹丸飞行时间（s） */
    float LastFlightTime = 0.0f;

    /** @brief 默认视觉延迟补偿值（s） */
    float DefaultVisionLatency = 0.08f;

    /** @brief 最近一次参与计算的延迟补偿值（s） */
    float LastLatencyCompensation = 0.08f;

    /** @brief 当前默认的无人机拦截方法名 */
    FString CurrentInterceptMethod = TEXT("pure_pursuit");

    /** @brief 拦截机默认速度上限（m/s） */
    float InterceptorSpeed = 8.0f;

    /** @brief 比例导航增益 N */
    float InterceptNavGain = 3.0f;

    /** @brief 纯追踪/前置法使用的预测时间（s） */
    float InterceptLeadTime = 0.6f;

    /** @brief 判定捕获成功的距离阈值（m） */
    float CaptureRadius = 1.5f;

    /** @brief 最近一次自动拦截使用的拦截机 ID */
    FString LastInterceptorId;

    /** @brief 最近一次自动拦截使用的目标机 ID */
    FString LastTargetId;

    /** @brief 最近一次下发给拦截机的速度指令（m/s） */
    FVector LastInterceptorCmdVel = FVector::ZeroVector;

    /** @brief 最近一次自动拦截时的目标距离（m） */
    float LastDistanceToTarget = 0.0f;

    /** @brief 最近一次自动拦截时的闭合速度（m/s） */
    float LastClosingSpeed = 0.0f;

    /** @brief 最近一次自动拦截是否生成了有效控制量 */
    bool bLastInterceptValid = false;

    /** @brief 最近一次自动拦截是否已判定捕获 */
    bool bLastCaptured = false;
};
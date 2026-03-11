#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "VisualInterceptController.generated.h"

class FJsonObject;
class ADronePawn;
class UKalmanPredictor;
class UPIDController;

/**
 * @brief 视觉闭环拦截控制器
 * 基于图像目标框中心偏差与面积误差，输出偏航、升降和前进速度命令，
 * 支持搜索、跟踪、逼近、捕获等状态切换，并可结合卡尔曼预测做丢检补偿。
 */
UCLASS()
class GRADUATIONPROJECT_API UVisualInterceptController : public UObject
{
    GENERATED_BODY()

public:
    /** @brief 延迟创建 PID 与预测器，并完成默认参数初始化 */
    void EnsureInitialized();

    /** @brief 重置控制器参数和运行时状态 */
    void Reset();

    /** @brief 处理视觉拦截启动命令 */
    FString HandleStart(const TSharedPtr<FJsonObject>& CmdObj, UWorld* World);

    /** @brief 处理视觉拦截逐帧更新命令 */
    FString HandleUpdate(const TSharedPtr<FJsonObject>& CmdObj, UWorld* World);

    /** @brief 处理视觉拦截停止命令 */
    FString HandleStop(const TSharedPtr<FJsonObject>& CmdObj, UWorld* World);

    /** @brief 输出当前视觉拦截状态 JSON */
    FString HandleState() const;

private:
    /** @brief 视觉拦截有限状态机 */
    enum class EVisualState : uint8
    {
        Idle,
        Search,
        Track,
        Approach,
        Captured,
    };

    /**
     * @brief 视觉拦截参数集合
     * 包含目标面积阈值、搜索策略、最大速度和是否启用卡尔曼预测等配置。
     */
    struct FVisualParams
    {
        float DesiredArea = 0.05f;
        float CaptureArea = 0.09f;
        float CenterTolX = 0.08f;
        float CenterTolY = 0.10f;
        int32 CaptureHoldFrames = 12;
        int32 LostToSearchFrames = 8;

        float MaxForwardSpeed = 7.5f;
        float MaxReverseSpeed = 1.5f;
        float MaxVerticalSpeed = 2.5f;
        float MaxYawRateDeg = 60.0f;

        float SearchCamYawLimitDeg = 165.0f;
        float SearchCamRateDeg = 70.0f;
        float SearchBodyYawRateDeg = 22.0f;
        float SearchCamPitchDeg = -5.0f;
        float SearchVzAmp = 0.4f;

        bool bStopOnCapture = true;
        bool bUseKalman = true;
    };

    /**
     * @brief 视觉拦截运行时状态
     * 保存当前状态机状态、最近一次检测结果、搜索相位和控制输出等中间量。
     */
    struct FVisualRuntime
    {
        bool bEnabled = false;
        bool bCaptured = false;

        FString InterceptorId;
        FString TargetId;
        FString Method = TEXT("vision_pid_kalman");

        EVisualState State = EVisualState::Idle;

        int32 FrameCount = 0;
        int32 DetectionCount = 0;
        int32 LostCount = 0;
        int32 CaptureCount = 0;

        float LastEx = 0.0f;
        float LastEy = 0.0f;
        float LastAreaNorm = 0.0f;
        float LastConf = 0.0f;
        float LastYawCmdDeg = 0.0f;
        float LastYawRateDeg = 0.0f;
        FVector LastCmdVel = FVector::ZeroVector;

        float SearchCamYawDeg = 0.0f;
        float SearchDir = 1.0f;
        float SearchTime = 0.0f;

        float LastFrameW = 640.0f;
        float LastFrameH = 480.0f;
    };

    /** @brief 从 JSON 读取并更新控制参数 */
    void ApplyParamsFromJson(const TSharedPtr<FJsonObject>& CmdObj);

    /** @brief 仅重置运行时状态，不改动参数配置 */
    void ResetRuntimeOnly();

    /** @brief 解析拦截机对象 */
    ADronePawn* ResolveInterceptor(UWorld* World, const TSharedPtr<FJsonObject>& CmdObj);

    /** @brief 解析目标机对象 */
    ADronePawn* ResolveTarget(UWorld* World, const TSharedPtr<FJsonObject>& CmdObj);

    /** @brief 规范化帧间隔，避免 PID 和预测器收到异常 dt */
    float SafeDt(float Dt) const;

    /** @brief 将最新 dt 同步给各 PID 控制器 */
    void SyncPidTimeStep(float Dt);

    /** @brief 构造启动响应 JSON */
    FString BuildStartJson() const;

    /** @brief 构造逐帧更新响应 JSON */
    FString BuildUpdateJson(bool bValidControl) const;

    /** @brief 构造错误响应 JSON */
    FString MakeError(const FString& Msg) const;

    /** @brief 构造成功响应 JSON */
    FString MakeOk(const FString& Msg) const;

    /** @brief 将布尔值转换为 JSON 字面量 */
    static FString BoolLiteral(bool bValue);

    /** @brief 规范化偏航角到 Unreal 标准范围 */
    static float NormalizeYawDeg(float YawDeg);

    /** @brief 将状态枚举转换为字符串 */
    static FString StateToString(EVisualState State);

    /** @brief 将外部请求的方法名规范化为内部标准名 */
    static bool NormalizeMethod(const FString& InMethod, FString& OutCanonical);

    /**
     * @brief 基于单帧检测结果计算跟踪控制量
     * @param Interceptor 拦截机
     * @param Feature 目标特征，约定为 `(cx, cy, area)`
     * @param FrameW 图像宽度
     * @param FrameH 图像高度
     * @param Confidence 检测置信度
     * @param Dt 帧间隔
     * @param bDetectionReal 本帧是否来自真实检测
     * @param bOutCaptured 输出是否达成捕获条件
     * @return 控制量有效时返回 `true`
     */
    bool ComputeTrackControl(
        ADronePawn* Interceptor,
        const FVector& Feature,
        float FrameW,
        float FrameH,
        float Confidence,
        float Dt,
        bool bDetectionReal,
        bool& bOutCaptured);

    /** @brief 计算搜索状态下的相机扫描与机体搜索控制 */
    void ComputeSearchControl(ADronePawn* Interceptor, float Dt);

private:
    /** @brief 图像特征卡尔曼预测器 */
    UPROPERTY()
    UKalmanPredictor* FeaturePredictor = nullptr;

    /** @brief 水平方向 PID */
    UPROPERTY()
    UPIDController* YawPid = nullptr;

    /** @brief 垂直方向 PID */
    UPROPERTY()
    UPIDController* VerticalPid = nullptr;

    /** @brief 前进速度 PID */
    UPROPERTY()
    UPIDController* ForwardPid = nullptr;

    /** @brief 参数配置集合 */
    FVisualParams Params;

    /** @brief 运行时状态集合 */
    FVisualRuntime Runtime;
};

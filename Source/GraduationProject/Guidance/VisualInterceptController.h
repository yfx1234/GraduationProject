#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "VisualInterceptController.generated.h"

class FJsonObject;
class ADronePawn;
class UKalmanPredictor;
class UPIDController;

/**
 * @brief 基于视觉的无人机拦截控制器
 *
 * 通过图像帧中目标检测结果（中心坐标 + 面积）驱动拦截无人机，
 * 实现 搜索 → 跟踪 → 逼近 → 捕获 的四阶段视觉伺服闭环。
 * 支持 PID / PD 控制，可选 Kalman 滤波平滑特征预测。
 * 所有参数均可通过 JSON 命令在线调整。
 */
UCLASS()
class GRADUATIONPROJECT_API UVisualInterceptController : public UObject
{
    GENERATED_BODY()

public:
    /** @brief 惰性初始化：首次使用时创建 Kalman 预测器和三个 PID 控制器 */
    void EnsureInitialized();

    /** @brief 完全重置控制器——重建预测器、PID、运行时状态 */
    void Reset();

    /**
     * @brief 处理 "visual_intercept_start" 命令
     * @param CmdObj JSON 命令对象（含 method / interceptor_id / target_id / 参数覆盖等）
     * @param World  当前 UWorld
     * @return JSON 响应字符串（包含已应用的参数快照）
     */
    FString HandleStart(const TSharedPtr<FJsonObject>& CmdObj, UWorld* World);

    /**
     * @brief 处理 "visual_intercept_update" 命令——每帧调用
     *        接收检测框信息（cx, cy, area, conf），输出速度 / 偏航 / 云台角指令
     * @param CmdObj JSON 命令对象（含 has_detection / cx / cy / area / dt 等）
     * @param World  当前 UWorld
     * @return JSON 响应字符串（含控制量与状态信息）
     */
    FString HandleUpdate(const TSharedPtr<FJsonObject>& CmdObj, UWorld* World);

    /**
     * @brief 处理 "visual_intercept_stop" 命令——悬停并停止控制
     */
    FString HandleStop(const TSharedPtr<FJsonObject>& CmdObj, UWorld* World);

    /** @brief 查询当前拦截状态（JSON 格式） */
    FString HandleState() const;

private:
    // ──── 状态枚举 ────

    /** @brief 视觉拦截状态机的五个阶段 */
    enum class EVisualState : uint8
    {
        Idle,       ///< 空闲——未启动或已停止
        Search,     ///< 搜索——云台扫描 + 机体缓转，寻找目标
        Track,      ///< 跟踪——PID 纠偏，保持目标在画面中心
        Approach,   ///< 逼近——面积已超过 CaptureArea，持续前冲
        Captured,   ///< 捕获——距离小于 InterceptDistance，任务完成
    };

    // ──── 可调参数 ────

    /** @brief 视觉拦截算法的全部可调参数（可通过 JSON 在线覆盖） */
    struct FVisualParams
    {
        float DesiredArea = 0.05f;            ///< 期望目标面积占比（归一化）
        float CaptureArea = 0.09f;            ///< 判定为 Approach 阶段的面积阈值
        float CenterTolX = 0.08f;             ///< 水平居中容差（归一化坐标）
        float CenterTolY = 0.10f;             ///< 垂直居中容差
        int32 CaptureHoldFrames = 12;         ///< 连续满足捕获条件多少帧才确认
        int32 LostToSearchFrames = 8;         ///< 连续丢失超过该帧数后退回 Search

        float MaxForwardSpeed = 7.5f;         ///< 最大前进速度 (m/s)
        float MaxReverseSpeed = 1.5f;         ///< 最大后退速度 (m/s)
        float MaxVerticalSpeed = 2.5f;        ///< 最大垂直速度 (m/s)
        float MaxYawRateDeg = 60.0f;          ///< 最大偏航角速度 (°/s)
        float RamAreaTarget = 0.28f;          ///< 冲刺阶段目标面积占比阈值
        float MinRamSpeed = 1.8f;             ///< 最小冲刺速度 (m/s)
        float InterceptDistance = 1.5f;       ///< 距离判定捕获的阈值 (m)

        float SearchCamYawLimitDeg = 165.0f;  ///< 搜索时云台偏航极限 (°)
        float SearchCamRateDeg = 70.0f;       ///< 搜索时云台扫描速度 (°/s)
        float SearchBodyYawRateDeg = 22.0f;   ///< 搜索时机体偏航旋转速度 (°/s)
        float SearchCamPitchDeg = -5.0f;      ///< 搜索时云台俯仰角 (°)
        float SearchVzAmp = 0.4f;             ///< 搜索时垂直振荡幅度 (m/s)

        bool bStopOnCapture = false;          ///< 捕获后是否自动悬停
        bool bUseKalman = true;               ///< 是否启用 Kalman 预测
    };

    // ──── 运行时状态 ────

    /** @brief 控制器运行时数据，每次 BeginSession 时重置 */
    struct FVisualRuntime
    {
        bool bEnabled = false;                           ///< 是否已启动
        bool bCaptured = false;                          ///< 是否已完成捕获

        FString InterceptorId;                           ///< 拦截无人机 ID
        FString TargetId;                                ///< 目标无人机 ID
        FString Method = TEXT("vision_pid_kalman");       ///< 当前使用的控制方法

        EVisualState State = EVisualState::Idle;         ///< 当前状态机阶段

        int32 FrameCount = 0;                            ///< 已处理帧数
        int32 DetectionCount = 0;                        ///< 累计检测到目标帧数
        int32 LostCount = 0;                             ///< 连续丢失帧计数
        int32 CaptureCount = 0;                          ///< 连续满足捕获条件帧数

        float LastEx = 0.0f;                             ///< 上一帧水平归一化误差
        float LastEy = 0.0f;                             ///< 上一帧垂直归一化误差
        float LastAreaNorm = 0.0f;                       ///< 上一帧目标面积归一化值
        float LastConf = 0.0f;                           ///< 上一帧检测置信度
        float LastYawCmdDeg = 0.0f;                      ///< 上一帧偏航指令 (°)
        float LastYawRateDeg = 0.0f;                     ///< 上一帧偏航角速度 (°/s)
        FVector LastCmdVel = FVector::ZeroVector;         ///< 上一帧速度指令 (m/s)
        float LastTargetDistance = -1.0f;                ///< 上一帧到目标的距离 (m)

        float SearchCamYawDeg = 0.0f;                    ///< 搜索模式下云台当前偏航角
        float SearchDir = 1.0f;                          ///< 搜索扫描方向 (+1 / -1)
        float SearchTime = 0.0f;                         ///< 搜索持续时间累计

        float LastFrameW = 640.0f;                       ///< 上一帧图像宽度 (px)
        float LastFrameH = 480.0f;                       ///< 上一帧图像高度 (px)
    };

    // ──── 内部辅助方法 ────

    /** @brief 从 JSON 对象读取并校验视觉拦截参数，更新 Params */
    void ApplyParamsFromJson(const TSharedPtr<FJsonObject>& CmdObj);

    /** @brief 仅重置运行时状态，不影响 PID / 预测器 */
    void ResetRuntimeOnly();

    /** @brief 重置所有 PID 控制器的积分项与历史误差 */
    void ResetPidControllers();

    /** @brief 开启新拦截会话，初始化运行时数据 */
    void BeginSession(const ADronePawn* Interceptor);

    /**
     * @brief 查找拦截无人机——依次按 interceptor_id → MissionRole::Interceptor → drone_1 → drone_0
     */
    ADronePawn* ResolveInterceptor(const TSharedPtr<FJsonObject>& CmdObj);

    /** @brief 查找目标无人机——逻辑与 ResolveInterceptor 类似 */
    ADronePawn* ResolveTarget(const TSharedPtr<FJsonObject>& CmdObj);

    /** @brief 对 Dt 执行有效性检查并限幅到 [0.01, 0.25] 秒 */
    float SafeDt(float Dt) const;

    /** @brief 将当前 Dt 同步到三个 PID 控制器的时间步长 */
    void SyncPidTimeStep(float Dt);

    // ──── JSON 响应构建 ────

    /** @brief 构建 HandleStart 的 JSON 响应（含完整参数快照） */
    FString BuildStartJson() const;

    /** @brief 构建 HandleUpdate 的 JSON 响应（含控制量 + 统计信息） */
    FString BuildUpdateJson(bool bValidControl) const;

    /** @brief 构建错误 JSON：{"status":"error","message":"..."} */
    FString MakeError(const FString& Msg) const;

    /** @brief 构建成功 JSON：{"status":"ok","message":"..."} */
    FString MakeOk(const FString& Msg) const;

    // ──── 静态工具函数 ────

    /** @brief 布尔值转 JSON 字面量 ("true" / "false") */
    static FString BoolLiteral(bool bValue);

    /** @brief 偏航角归一化到 [-180, 180) */
    static float NormalizeYawDeg(float YawDeg);

    /** @brief 状态枚举转字符串 */
    static FString StateToString(EVisualState State);

    /**
     * @brief 规范化方法名称（如 "pid" → "vision_pid"）
     * @return 是否为支持的方法名
     */
    static bool NormalizeMethod(const FString& InMethod, FString& OutCanonical);

    // ──── 核心控制计算 ────

    /**
     * @brief 跟踪/逼近模式的控制计算
     *
     * 根据检测特征（中心 + 面积）计算偏航角速度、垂直速度、前进速度、
     * 云台角指令，并发送到拦截无人机。
     *
     * @param Interceptor    拦截无人机
     * @param Feature        (Cx, Cy, Area) 像素坐标与面积
     * @param FrameW         图像宽度 (px)
     * @param FrameH         图像高度 (px)
     * @param Confidence     检测置信度
     * @param Dt             时间步长 (s)
     * @param bDetectionReal 是否为真实检测（否则为 Kalman 预测）
     * @param bOutCaptured   [out] 本帧是否判定为捕获
     * @return 是否成功计算
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

    /**
     * @brief 搜索模式的控制计算——云台左右扫描 + 机体缓慢旋转 + 垂直振荡
     */
    void ComputeSearchControl(ADronePawn* Interceptor, float Dt);

    // ──── 成员变量 ────

private:
    UPROPERTY()
    UKalmanPredictor* FeaturePredictor = nullptr;  ///< Kalman 滤波器——平滑特征并预测丢失帧

    UPROPERTY()
    UPIDController* YawPid = nullptr;              ///< 偏航角速度 PID

    UPROPERTY()
    UPIDController* VerticalPid = nullptr;         ///< 垂直速度 PID

    UPROPERTY()
    UPIDController* ForwardPid = nullptr;          ///< 前进速度 PID

    FVisualParams Params;                          ///< 算法参数（可在线调整）
    FVisualRuntime Runtime;                        ///< 运行时状态
};

// 解释：使用 `#pragma once` 防止该头文件在编译过程中被重复包含。
#pragma once

// 解释：引入 Unreal 的核心基础头文件，提供常用容器、数学类型和日志宏。
#include "CoreMinimal.h"
// 解释：引入 `NoExportTypes.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "UObject/NoExportTypes.h"
// 解释：引入 `VisualInterceptController.generated.h`，让 Unreal Header Tool 生成的反射代码在本文件中可见。
#include "VisualInterceptController.generated.h"

// 解释：这一行声明 类 `FJsonObject`，用于封装fjsonobject相关的数据与行为。
class FJsonObject;
// 解释：这一行声明 类 `ADronePawn`，用于封装adronePawn相关的数据与行为。
class ADronePawn;
// 解释：这一行声明 类 `UKalmanPredictor`，用于封装ukalman预测器相关的数据与行为。
class UKalmanPredictor;
// 解释：这一行声明 类 `UPIDController`，用于封装upidcontroller相关的数据与行为。
class UPIDController;

/**
 * @brief 基于视觉的无人机拦截控制器
 *
 * 通过图像帧中目标检测结果（中心坐标 + 面积）驱动拦截无人机，
 * 实现 搜索 → 跟踪 → 逼近 → 捕获 的四阶段视觉伺服闭环。
 * 支持 PID / PD 控制，可选 Kalman 滤波平滑特征预测。
 * 所有参数均可通过 JSON 命令在线调整。
 */
// 解释：使用 `UCLASS` 宏把当前类型注册到 Unreal 反射系统中，便于蓝图、序列化和编辑器识别。
UCLASS()
// 解释：这一行声明 类 `UVisualInterceptController`，用于封装uvisual拦截控制器相关的数据与行为。
class GRADUATIONPROJECT_API UVisualInterceptController : public UObject
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：展开 Unreal Header Tool 生成的样板代码，这是 UObject、Actor、组件等类型正常工作的基础。
    GENERATED_BODY()

// 解释：从这一行开始进入 `public` 访问区，下面的成员会对外部模块公开。
public:
    /** @brief 惰性初始化：首次使用时创建 Kalman 预测器和三个 PID 控制器 */
    // 解释：调用 `EnsureInitialized` 执行当前步骤需要的功能逻辑。
    void EnsureInitialized();

    /** @brief 完全重置控制器——重建预测器、PID、运行时状态 */
    // 解释：调用 `Reset` 执行当前步骤需要的功能逻辑。
    void Reset();

    /**
     * @brief 处理 "visual_intercept_start" 命令
     * @param CmdObj JSON 命令对象（含 method / interceptor_id / target_id / 参数覆盖等）
     * @param World  当前 UWorld
     * @return JSON 响应字符串（包含已应用的参数快照）
     */
    // 解释：调用 `HandleStart` 执行当前步骤需要的功能逻辑。
    FString HandleStart(const TSharedPtr<FJsonObject>& CmdObj, UWorld* World);

    /**
     * @brief 处理 "visual_intercept_update" 命令——每帧调用
     *        接收检测框信息（cx, cy, area, conf），输出速度 / 偏航 / 云台角指令
     * @param CmdObj JSON 命令对象（含 has_detection / cx / cy / area / dt 等）
     * @param World  当前 UWorld
     * @return JSON 响应字符串（含控制量与状态信息）
     */
    // 解释：调用 `HandleUpdate` 执行当前步骤需要的功能逻辑。
    FString HandleUpdate(const TSharedPtr<FJsonObject>& CmdObj, UWorld* World);

    /**
     * @brief 处理 "visual_intercept_stop" 命令——悬停并停止控制
     */
    // 解释：调用 `HandleStop` 执行当前步骤需要的功能逻辑。
    FString HandleStop(const TSharedPtr<FJsonObject>& CmdObj, UWorld* World);

    /** @brief 查询当前拦截状态（JSON 格式） */
    // 解释：调用 `HandleState` 执行当前步骤需要的功能逻辑。
    FString HandleState() const;

// 解释：从这一行开始进入 `private` 访问区，下面的成员只允许类内部访问。
private:
    // ──── 状态枚举 ────

    /** @brief 视觉拦截状态机的五个阶段 */
    // 解释：这一行声明枚举 `EVisualState`，用于约束一组有限的状态或模式取值。
    enum class EVisualState : uint8
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Idle,       ///< 空闲——未启动或已停止
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Search,     ///< 搜索——云台扫描 + 机体缓转，寻找目标
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Track,      ///< 跟踪——PID 纠偏，保持目标在画面中心
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Approach,   ///< 逼近——面积已超过 CaptureArea，持续前冲
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Captured,   ///< 捕获——距离小于 InterceptDistance，任务完成
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    };

    // ──── 可调参数 ────

    /** @brief 视觉拦截算法的全部可调参数（可通过 JSON 在线覆盖） */
    // 解释：这一行声明 结构体 `FVisualParams`，用于封装fvisualparams相关的数据与行为。
    struct FVisualParams
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行声明成员或局部变量 `DesiredArea`，用于保存desiredarea。
        float DesiredArea = 0.05f;            ///< 期望目标面积占比（归一化）
        // 解释：这一行声明成员或局部变量 `CaptureArea`，用于保存采集area。
        float CaptureArea = 0.09f;            ///< 判定为 Approach 阶段的面积阈值
        // 解释：这一行声明成员或局部变量 `CenterTolX`，用于保存centertolX。
        float CenterTolX = 0.08f;             ///< 水平居中容差（归一化坐标）
        // 解释：这一行声明成员或局部变量 `CenterTolY`，用于保存centertolY。
        float CenterTolY = 0.10f;             ///< 垂直居中容差
        // 解释：这一行声明成员或局部变量 `CaptureHoldFrames`，用于保存采集holdframes。
        int32 CaptureHoldFrames = 12;         ///< 连续满足捕获条件多少帧才确认
        // 解释：这一行声明成员或局部变量 `LostToSearchFrames`，用于保存losttosearchframes。
        int32 LostToSearchFrames = 8;         ///< 连续丢失超过该帧数后退回 Search

        // 解释：这一行声明成员或局部变量 `MaxForwardSpeed`，用于保存maxforwardspeed。
        float MaxForwardSpeed = 7.5f;         ///< 最大前进速度 (m/s)
        // 解释：这一行声明成员或局部变量 `MaxReverseSpeed`，用于保存maxreversespeed。
        float MaxReverseSpeed = 1.5f;         ///< 最大后退速度 (m/s)
        // 解释：这一行声明成员或局部变量 `MaxVerticalSpeed`，用于保存maxverticalspeed。
        float MaxVerticalSpeed = 2.5f;        ///< 最大垂直速度 (m/s)
        // 解释：这一行声明成员或局部变量 `MaxYawRateDeg`，用于保存maxyawratedeg。
        float MaxYawRateDeg = 60.0f;          ///< 最大偏航角速度 (°/s)
        // 解释：这一行声明成员或局部变量 `RamAreaTarget`，用于保存ramareatarget。
        float RamAreaTarget = 0.28f;          ///< 冲刺阶段目标面积占比阈值
        // 解释：这一行声明成员或局部变量 `MinRamSpeed`，用于保存minramspeed。
        float MinRamSpeed = 1.8f;             ///< 最小冲刺速度 (m/s)
        // 解释：这一行声明成员或局部变量 `InterceptDistance`，用于保存拦截distance。
        float InterceptDistance = 1.5f;       ///< 距离判定捕获的阈值 (m)

        // 解释：这一行声明成员或局部变量 `SearchCamYawLimitDeg`，用于保存searchcamyawlimitdeg。
        float SearchCamYawLimitDeg = 165.0f;  ///< 搜索时云台偏航极限 (°)
        // 解释：这一行声明成员或局部变量 `SearchCamRateDeg`，用于保存searchcamratedeg。
        float SearchCamRateDeg = 70.0f;       ///< 搜索时云台扫描速度 (°/s)
        // 解释：这一行声明成员或局部变量 `SearchBodyYawRateDeg`，用于保存searchbodyyawratedeg。
        float SearchBodyYawRateDeg = 22.0f;   ///< 搜索时机体偏航旋转速度 (°/s)
        // 解释：这一行声明成员或局部变量 `SearchCamPitchDeg`，用于保存searchcampitchdeg。
        float SearchCamPitchDeg = -5.0f;      ///< 搜索时云台俯仰角 (°)
        // 解释：这一行声明成员或局部变量 `SearchVzAmp`，用于保存searchvzamp。
        float SearchVzAmp = 0.4f;             ///< 搜索时垂直振荡幅度 (m/s)

        // 解释：这一行声明成员或局部变量 `bStopOnCapture`，用于保存布尔标志 stopon采集。
        bool bStopOnCapture = false;          ///< 捕获后是否自动悬停
        // 解释：这一行声明成员或局部变量 `bUseKalman`，用于保存布尔标志 use卡尔曼。
        bool bUseKalman = true;               ///< 是否启用 Kalman 预测
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    };

    // ──── 运行时状态 ────

    /** @brief 控制器运行时数据，每次 BeginSession 时重置 */
    // 解释：这一行声明 结构体 `FVisualRuntime`，用于封装fvisualruntime相关的数据与行为。
    struct FVisualRuntime
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行声明成员或局部变量 `bEnabled`，用于保存布尔标志 enabled。
        bool bEnabled = false;                           ///< 是否已启动
        // 解释：这一行声明成员或局部变量 `bCaptured`，用于保存布尔标志 captured。
        bool bCaptured = false;                          ///< 是否已完成捕获

        // 解释：这一行声明成员或局部变量 `InterceptorId`，用于保存interceptorid。
        FString InterceptorId;                           ///< 拦截无人机 ID
        // 解释：这一行声明成员或局部变量 `TargetId`，用于保存targetid。
        FString TargetId;                                ///< 目标无人机 ID
        // 解释：这一行把右侧表达式的结果写入 `FString Method`，完成 fstringmethod 的更新。
        FString Method = TEXT("vision_pid_kalman");       ///< 当前使用的控制方法

        // 解释：这一行声明成员或局部变量 `State`，用于保存状态。
        EVisualState State = EVisualState::Idle;         ///< 当前状态机阶段

        // 解释：这一行声明成员或局部变量 `FrameCount`，用于保存framecount。
        int32 FrameCount = 0;                            ///< 已处理帧数
        // 解释：这一行声明成员或局部变量 `DetectionCount`，用于保存detectioncount。
        int32 DetectionCount = 0;                        ///< 累计检测到目标帧数
        // 解释：这一行声明成员或局部变量 `LostCount`，用于保存lostcount。
        int32 LostCount = 0;                             ///< 连续丢失帧计数
        // 解释：这一行声明成员或局部变量 `CaptureCount`，用于保存采集count。
        int32 CaptureCount = 0;                          ///< 连续满足捕获条件帧数

        // 解释：这一行声明成员或局部变量 `LastEx`，用于保存lastex。
        float LastEx = 0.0f;                             ///< 上一帧水平归一化误差
        // 解释：这一行声明成员或局部变量 `LastEy`，用于保存lastey。
        float LastEy = 0.0f;                             ///< 上一帧垂直归一化误差
        // 解释：这一行声明成员或局部变量 `LastAreaNorm`，用于保存lastareanorm。
        float LastAreaNorm = 0.0f;                       ///< 上一帧目标面积归一化值
        // 解释：这一行声明成员或局部变量 `LastConf`，用于保存lastconf。
        float LastConf = 0.0f;                           ///< 上一帧检测置信度
        // 解释：这一行声明成员或局部变量 `LastYawCmdDeg`，用于保存lastyawcmddeg。
        float LastYawCmdDeg = 0.0f;                      ///< 上一帧偏航指令 (°)
        // 解释：这一行声明成员或局部变量 `LastYawRateDeg`，用于保存lastyawratedeg。
        float LastYawRateDeg = 0.0f;                     ///< 上一帧偏航角速度 (°/s)
        // 解释：这一行声明成员或局部变量 `LastCmdVel`，用于保存lastcmdvel。
        FVector LastCmdVel = FVector::ZeroVector;         ///< 上一帧速度指令 (m/s)
        // 解释：这一行声明成员或局部变量 `LastTargetDistance`，用于保存lasttargetdistance。
        float LastTargetDistance = -1.0f;                ///< 上一帧到目标的距离 (m)

        // 解释：这一行声明成员或局部变量 `SearchCamYawDeg`，用于保存searchcamyawdeg。
        float SearchCamYawDeg = 0.0f;                    ///< 搜索模式下云台当前偏航角
        // 解释：这一行声明成员或局部变量 `SearchDir`，用于保存searchdir。
        float SearchDir = 1.0f;                          ///< 搜索扫描方向 (+1 / -1)
        // 解释：这一行声明成员或局部变量 `SearchTime`，用于保存searchtime。
        float SearchTime = 0.0f;                         ///< 搜索持续时间累计

        // 解释：这一行声明成员或局部变量 `LastFrameW`，用于保存lastframeW。
        float LastFrameW = 640.0f;                       ///< 上一帧图像宽度 (px)
        // 解释：这一行声明成员或局部变量 `LastFrameH`，用于保存lastframeH。
        float LastFrameH = 480.0f;                       ///< 上一帧图像高度 (px)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    };

    // ──── 内部辅助方法 ────

    /** @brief 从 JSON 对象读取并校验视觉拦截参数，更新 Params */
    // 解释：调用 `ApplyParamsFromJson` 执行当前步骤需要的功能逻辑。
    void ApplyParamsFromJson(const TSharedPtr<FJsonObject>& CmdObj);

    /** @brief 仅重置运行时状态，不影响 PID / 预测器 */
    // 解释：调用 `ResetRuntimeOnly` 执行当前步骤需要的功能逻辑。
    void ResetRuntimeOnly();

    /** @brief 重置所有 PID 控制器的积分项与历史误差 */
    // 解释：调用 `ResetPidControllers` 执行当前步骤需要的功能逻辑。
    void ResetPidControllers();

    /** @brief 开启新拦截会话，初始化运行时数据 */
    // 解释：调用 `BeginSession` 执行当前步骤需要的功能逻辑。
    void BeginSession(const ADronePawn* Interceptor);

    /**
     * @brief 查找拦截无人机——依次按 interceptor_id → MissionRole::Interceptor → drone_1 → drone_0
     */
    // 解释：调用 `ResolveInterceptor` 执行当前步骤需要的功能逻辑。
    ADronePawn* ResolveInterceptor(const TSharedPtr<FJsonObject>& CmdObj);

    /** @brief 查找目标无人机——逻辑与 ResolveInterceptor 类似 */
    // 解释：调用 `ResolveTarget` 执行当前步骤需要的功能逻辑。
    ADronePawn* ResolveTarget(const TSharedPtr<FJsonObject>& CmdObj);

    /** @brief 对 Dt 执行有效性检查并限幅到 [0.01, 0.25] 秒 */
    // 解释：调用 `SafeDt` 执行当前步骤需要的功能逻辑。
    float SafeDt(float Dt) const;

    /** @brief 将当前 Dt 同步到三个 PID 控制器的时间步长 */
    // 解释：调用 `SyncPidTimeStep` 执行当前步骤需要的功能逻辑。
    void SyncPidTimeStep(float Dt);

    // ──── JSON 响应构建 ────

    /** @brief 构建 HandleStart 的 JSON 响应（含完整参数快照） */
    // 解释：调用 `BuildStartJson` 执行当前步骤需要的功能逻辑。
    FString BuildStartJson() const;

    /** @brief 构建 HandleUpdate 的 JSON 响应（含控制量 + 统计信息） */
    // 解释：调用 `BuildUpdateJson` 执行当前步骤需要的功能逻辑。
    FString BuildUpdateJson(bool bValidControl) const;

    /** @brief 构建错误 JSON：{"status":"error","message":"..."} */
    // 解释：调用 `MakeError` 执行当前步骤需要的功能逻辑。
    FString MakeError(const FString& Msg) const;

    /** @brief 构建成功 JSON：{"status":"ok","message":"..."} */
    // 解释：调用 `MakeOk` 执行当前步骤需要的功能逻辑。
    FString MakeOk(const FString& Msg) const;

    // ──── 静态工具函数 ────

    /** @brief 布尔值转 JSON 字面量 ("true" / "false") */
    // 解释：调用 `BoolLiteral` 执行当前步骤需要的功能逻辑。
    static FString BoolLiteral(bool bValue);

    /** @brief 偏航角归一化到 [-180, 180) */
    // 解释：调用 `NormalizeYawDeg` 执行当前步骤需要的功能逻辑。
    static float NormalizeYawDeg(float YawDeg);

    /** @brief 状态枚举转字符串 */
    // 解释：调用 `StateToString` 执行当前步骤需要的功能逻辑。
    static FString StateToString(EVisualState State);

    /**
     * @brief 规范化方法名称（如 "pid" → "vision_pid"）
     * @return 是否为支持的方法名
     */
    // 解释：调用 `NormalizeMethod` 执行当前步骤需要的功能逻辑。
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
    // 解释：这一行定义函数 `ComputeTrackControl`，开始实现computetrackcontrol的具体逻辑。
    bool ComputeTrackControl(
        // 解释：这一行继续展开 `ComputeTrackControl` 的参数列表，声明参数 `Interceptor` 用于传入interceptor。
        ADronePawn* Interceptor,
        // 解释：这一行继续展开 `ComputeTrackControl` 的参数列表，声明参数 `Feature` 用于传入feature。
        const FVector& Feature,
        // 解释：这一行继续展开 `ComputeTrackControl` 的参数列表，声明参数 `FrameW` 用于传入frameW。
        float FrameW,
        // 解释：这一行继续展开 `ComputeTrackControl` 的参数列表，声明参数 `FrameH` 用于传入frameH。
        float FrameH,
        // 解释：这一行继续展开 `ComputeTrackControl` 的参数列表，声明参数 `Confidence` 用于传入confidence。
        float Confidence,
        // 解释：这一行继续展开 `ComputeTrackControl` 的参数列表，声明参数 `Dt` 用于传入dt。
        float Dt,
        // 解释：这一行继续展开 `ComputeTrackControl` 的参数列表，声明参数 `bDetectionReal` 用于传入布尔标志 detectionreal。
        bool bDetectionReal,
        // 解释：这一行继续补充函数 `ComputeTrackControl` 的参数列表、限定符或返回类型说明。
        bool& bOutCaptured);

    /**
     * @brief 搜索模式的控制计算——云台左右扫描 + 机体缓慢旋转 + 垂直振荡
     */
    // 解释：调用 `ComputeSearchControl` 执行当前步骤需要的功能逻辑。
    void ComputeSearchControl(ADronePawn* Interceptor, float Dt);

    // ──── 成员变量 ────

// 解释：从这一行开始进入 `private` 访问区，下面的成员只允许类内部访问。
private:
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY()
    // 解释：这一行声明成员或局部变量 `FeaturePredictor`，用于保存feature预测器。
    UKalmanPredictor* FeaturePredictor = nullptr;  ///< Kalman 滤波器——平滑特征并预测丢失帧

    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY()
    // 解释：这一行声明成员或局部变量 `YawPid`，用于保存yawPID。
    UPIDController* YawPid = nullptr;              ///< 偏航角速度 PID

    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY()
    // 解释：这一行声明成员或局部变量 `VerticalPid`，用于保存verticalPID。
    UPIDController* VerticalPid = nullptr;         ///< 垂直速度 PID

    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY()
    // 解释：这一行声明成员或局部变量 `ForwardPid`，用于保存forwardPID。
    UPIDController* ForwardPid = nullptr;          ///< 前进速度 PID

    // 解释：这一行声明成员或局部变量 `Params`，用于保存params。
    FVisualParams Params;                          ///< 算法参数（可在线调整）
    // 解释：这一行声明成员或局部变量 `Runtime`，用于保存runtime。
    FVisualRuntime Runtime;                        ///< 运行时状态
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
};

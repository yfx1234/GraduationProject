// 解释：使用 `#pragma once` 防止该头文件在编译过程中被重复包含。
#pragma once

// 解释：引入 Unreal 的核心基础头文件，提供常用容器、数学类型和日志宏。
#include "CoreMinimal.h"
// 解释：引入 `Actor.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "GameFramework/Actor.h"
// 解释：引入 `DronePawn.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "GraduationProject/Drone/DronePawn.h"
// 解释：引入 `GuidanceActor.generated.h`，让 Unreal Header Tool 生成的反射代码在本文件中可见。
#include "GuidanceActor.generated.h"

// 解释：这一行声明 类 `IGuidanceMethod`，用于封装iguidancemethod相关的数据与行为。
class IGuidanceMethod;
// 解释：这一行声明 类 `UKalmanPredictor`，用于封装ukalman预测器相关的数据与行为。
class UKalmanPredictor;
// 解释：这一行声明 类 `UVisualInterceptController`，用于封装uvisual拦截控制器相关的数据与行为。
class UVisualInterceptController;
// 解释：这一行声明 类 `UAgentManager`，用于封装uagent管理器相关的数据与行为。
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
// 解释：使用 `UCLASS` 宏把当前类型注册到 Unreal 反射系统中，便于蓝图、序列化和编辑器识别。
UCLASS()
// 解释：这一行声明 类 `AGuidanceActor`，用于封装aguidanceActor相关的数据与行为。
class GRADUATIONPROJECT_API AGuidanceActor : public AActor
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：展开 Unreal Header Tool 生成的样板代码，这是 UObject、Actor、组件等类型正常工作的基础。
    GENERATED_BODY()

// 解释：从这一行开始进入 `public` 访问区，下面的成员会对外部模块公开。
public:
    /** @brief 构造函数：创建一个隐藏且无碰撞的后台协调 Actor */
    // 解释：调用 `AGuidanceActor` 执行当前步骤需要的功能逻辑。
    AGuidanceActor();

    /** @brief 注册自身到 AgentManager，便于外部通过 GuidanceId 查找 */
    // 解释：调用 `BeginPlay` 执行当前步骤需要的功能逻辑。
    virtual void BeginPlay() override;

    /** @brief 结束时释放制导算法对象，并从 AgentManager 注销自身 */
    // 解释：调用 `EndPlay` 执行当前步骤需要的功能逻辑。
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

    /** @brief 重置预测器、制导器和历史输出状态 */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "Guidance")
    // 解释：调用 `ResetGuidance` 执行当前步骤需要的功能逻辑。
    FString ResetGuidance();

    /** @brief 获取当前 GuidanceActor 的状态 JSON */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "Guidance")
    // 解释：调用 `GetState` 执行当前步骤需要的功能逻辑。
    FString GetState();

    /**
     * @brief 切换常规制导算法
     * @param Method 制导方法名，支持 `direct`、`proportional`、`predictive`
     * @param NavConstant 比例导引导航常数，仅 `proportional` 模式使用
     * @param Iterations 预测制导迭代次数，仅 `predictive` 模式使用
     */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "Guidance")
    // 解释：这一行把右侧表达式的结果写入 `FString SetMethod(FString Method, float NavConstant`，完成 fstringsetmethodfstringmethodfloatnavconstant 的更新。
    FString SetMethod(FString Method, float NavConstant = 4.0f, int32 Iterations = 3);

    /**
     * @brief 设置无人机自动拦截算法的默认参数
     * @param Method 拦截方法名，支持 `pure_pursuit` / `proportional_nav` 及其别名
     * @param Speed 拦截机速度上限（m/s）
     * @param NavGain 比例导航增益 N
     * @param LeadTime 目标前置预测时间（s）
     * @param CaptureRadiusValue 捕获半径（m）
     */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "Guidance")
    // 解释：这一行把右侧表达式的结果写入 `FString SetInterceptMethod(FString Method`，完成 fstringset拦截methodfstringmethod 的更新。
    FString SetInterceptMethod(FString Method = TEXT("pure_pursuit"), float Speed = -1.0f, float NavGain = -1.0f, float LeadTime = -1.0f, float CaptureRadiusValue = -1.0f);

    /** @brief 列出当前场景中已注册的目标机和拦截机 ID */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "Guidance")
    // 解释：调用 `ListInterceptAgents` 执行当前步骤需要的功能逻辑。
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
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "Guidance")
    // 解释：这一行定义函数 `AutoIntercept`，开始实现auto拦截的具体逻辑。
    FString AutoIntercept(
        // 解释：这一行继续展开 `AutoIntercept` 的参数列表，声明参数 `InterceptorId` 用于传入interceptorid。
        FString InterceptorId = TEXT(""),
        // 解释：这一行继续展开 `AutoIntercept` 的参数列表，声明参数 `TargetId` 用于传入targetid。
        FString TargetId = TEXT(""),
        // 解释：这一行继续展开 `AutoIntercept` 的参数列表，声明参数 `Method` 用于传入method。
        FString Method = TEXT(""),
        // 解释：这一行声明成员或局部变量 `Speed`，用于保存speed。
        float Speed = -1.0f,
        // 解释：这一行声明成员或局部变量 `NavGain`，用于保存navgain。
        float NavGain = -1.0f,
        // 解释：这一行声明成员或局部变量 `LeadTime`，用于保存leadtime。
        float LeadTime = -1.0f,
        // 解释：这一行声明成员或局部变量 `CaptureRadiusValue`，用于保存采集radiusvalue。
        float CaptureRadiusValue = -1.0f,
        // 解释：这一行声明成员或局部变量 `bStopOnCapture`，用于保存布尔标志 stopon采集。
        bool bStopOnCapture = true);

    /**
     * @brief 向 Kalman 预测器输入一帧目标位置观测
     * @param X 观测位置 X
     * @param Y 观测位置 Y
     * @param Z 观测位置 Z
     * @param Dt 采样时间间隔（s）
     */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "Guidance")
    // 解释：这一行把右侧表达式的结果写入 `FString UpdateTarget(float X, float Y, float Z, float Dt`，完成 fstringupdatetargetfloatXfloatYfloatZfloatdt 的更新。
    FString UpdateTarget(float X, float Y, float Z, float Dt = 0.1f);

    /**
     * @brief 根据当前目标预测状态计算炮塔瞄准角
     * @param TurretId 炮塔 ID
     * @param MuzzleSpeed 弹丸初速度（m/s）
     */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "Guidance")
    // 解释：这一行把右侧表达式的结果写入 `FString ComputeAim(FString TurretId`，完成 fstringcomputeaimfstringturretid 的更新。
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
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "Guidance")
    // 解释：这一行把右侧表达式的结果写入 `FString AutoEngage(FString TurretId`，完成 fstringautoengagefstringturretid 的更新。
    FString AutoEngage(FString TurretId = TEXT("turret_0"), FString TargetId = TEXT("drone_0"), float MuzzleSpeed = 400.0f, float Dt = 0.05f, float Latency = -1.0f, bool bFire = false);

    /**
     * @brief 设置 Kalman 预测器参数
     * @param ProcessNoise 过程噪声协方差系数
     * @param MeasurementNoise 观测噪声协方差系数
     */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "Guidance")
    // 解释：这一行把右侧表达式的结果写入 `FString SetKalmanParams(float ProcessNoise`，完成 fstringset卡尔曼paramsfloatprocessnoise 的更新。
    FString SetKalmanParams(float ProcessNoise = 1.0f, float MeasurementNoise = 0.5f);

    /**
     * @brief 启动视觉拦截会话
     *
     * 该接口把 Blueprint/TCP 参数整理为 JSON 命令，再转交给 `UVisualInterceptController`。
     */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "Guidance|Visual")
    // 解释：这一行定义函数 `VisualInterceptStart`，开始实现视觉拦截start的具体逻辑。
    FString VisualInterceptStart(
        // 解释：这一行继续展开 `VisualInterceptStart` 的参数列表，声明参数 `InterceptorId` 用于传入interceptorid。
        FString InterceptorId = TEXT(""),
        // 解释：这一行继续展开 `VisualInterceptStart` 的参数列表，声明参数 `TargetId` 用于传入targetid。
        FString TargetId = TEXT(""),
        // 解释：这一行继续展开 `VisualInterceptStart` 的参数列表，声明参数 `Method` 用于传入method。
        FString Method = TEXT("vision_pid_kalman"),
        // 解释：这一行声明成员或局部变量 `DesiredArea`，用于保存desiredarea。
        float DesiredArea = -1.0f,
        // 解释：这一行声明成员或局部变量 `CaptureArea`，用于保存采集area。
        float CaptureArea = -1.0f,
        // 解释：这一行声明成员或局部变量 `CenterTolX`，用于保存centertolX。
        float CenterTolX = -1.0f,
        // 解释：这一行声明成员或局部变量 `CenterTolY`，用于保存centertolY。
        float CenterTolY = -1.0f,
        // 解释：这一行声明成员或局部变量 `CaptureHoldFrames`，用于保存采集holdframes。
        int32 CaptureHoldFrames = -1,
        // 解释：这一行声明成员或局部变量 `LostToSearchFrames`，用于保存losttosearchframes。
        int32 LostToSearchFrames = -1,
        // 解释：这一行声明成员或局部变量 `MaxForwardSpeed`，用于保存maxforwardspeed。
        float MaxForwardSpeed = -1.0f,
        // 解释：这一行声明成员或局部变量 `MaxReverseSpeed`，用于保存maxreversespeed。
        float MaxReverseSpeed = -1.0f,
        // 解释：这一行声明成员或局部变量 `MaxVerticalSpeed`，用于保存maxverticalspeed。
        float MaxVerticalSpeed = -1.0f,
        // 解释：这一行声明成员或局部变量 `MaxYawRateDeg`，用于保存maxyawratedeg。
        float MaxYawRateDeg = -1.0f,
        // 解释：这一行声明成员或局部变量 `SearchCamYawLimitDeg`，用于保存searchcamyawlimitdeg。
        float SearchCamYawLimitDeg = -1.0f,
        // 解释：这一行声明成员或局部变量 `SearchCamRateDeg`，用于保存searchcamratedeg。
        float SearchCamRateDeg = -1.0f,
        // 解释：这一行声明成员或局部变量 `SearchBodyYawRateDeg`，用于保存searchbodyyawratedeg。
        float SearchBodyYawRateDeg = -1.0f,
        // 解释：这一行声明成员或局部变量 `SearchCamPitchDeg`，用于保存searchcampitchdeg。
        float SearchCamPitchDeg = -1000.0f,
        // 解释：这一行声明成员或局部变量 `SearchVzAmp`，用于保存searchvzamp。
        float SearchVzAmp = -1.0f,
        // 解释：这一行声明成员或局部变量 `StopOnCaptureFlag`，用于保存stopon采集flag。
        int32 StopOnCaptureFlag = -1,
        // 解释：这一行声明成员或局部变量 `UseKalmanFlag`，用于保存use卡尔曼flag。
        int32 UseKalmanFlag = -1);

    /**
     * @brief 向视觉拦截控制器提交一帧检测结果
     *
     * 输入检测框中心、面积、置信度以及图像尺寸，由视觉控制器输出机体速度和云台控制量。
     */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "Guidance|Visual")
    // 解释：这一行定义函数 `VisualInterceptUpdate`，开始实现视觉拦截update的具体逻辑。
    FString VisualInterceptUpdate(
        // 解释：这一行声明成员或局部变量 `HasDetection`，用于保存hasdetection。
        int32 HasDetection = 0,
        // 解释：这一行声明成员或局部变量 `Cx`，用于保存cx。
        float Cx = 0.0f,
        // 解释：这一行声明成员或局部变量 `Cy`，用于保存cy。
        float Cy = 0.0f,
        // 解释：这一行声明成员或局部变量 `Area`，用于保存area。
        float Area = 0.0f,
        // 解释：这一行声明成员或局部变量 `AreaRatio`，用于保存arearatio。
        float AreaRatio = -1.0f,
        // 解释：这一行声明成员或局部变量 `Conf`，用于保存conf。
        float Conf = 1.0f,
        // 解释：这一行声明成员或局部变量 `Dt`，用于保存dt。
        float Dt = 0.08f,
        // 解释：这一行声明成员或局部变量 `ImageW`，用于保存图像W。
        float ImageW = 640.0f,
        // 解释：这一行声明成员或局部变量 `ImageH`，用于保存图像H。
        float ImageH = 480.0f,
        // 解释：这一行继续展开 `VisualInterceptUpdate` 的参数列表，声明参数 `InterceptorId` 用于传入interceptorid。
        FString InterceptorId = TEXT(""),
        // 解释：这一行把右侧表达式的结果写入 `FString TargetId`，完成 fstringtargetid 的更新。
        FString TargetId = TEXT(""));

    /** @brief 停止视觉拦截，并请求拦截无人机悬停 */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "Guidance|Visual")
    // 解释：这一行把右侧表达式的结果写入 `FString VisualInterceptStop(FString InterceptorId`，完成 fstring视觉拦截stopfstringinterceptorid 的更新。
    FString VisualInterceptStop(FString InterceptorId = TEXT(""), FString TargetId = TEXT(""));

    /** @brief 查询视觉拦截控制器当前状态 */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "Guidance|Visual")
    // 解释：调用 `VisualInterceptState` 执行当前步骤需要的功能逻辑。
    FString VisualInterceptState();

    /** @brief GuidanceActor 的唯一注册 ID */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Guidance")
    // 解释：这一行把右侧表达式的结果写入 `FString GuidanceId`，完成 fstring制导id 的更新。
    FString GuidanceId = TEXT("guidance_0");

// 解释：从这一行开始进入 `private` 访问区，下面的成员只允许类内部访问。
private:
    /** @brief 惰性初始化预测器、视觉拦截控制器与默认制导算法 */
    // 解释：调用 `EnsureInitialized` 执行当前步骤需要的功能逻辑。
    void EnsureInitialized();

    /**
     * @brief 按任务角色查找无人机
     * @param Manager AgentManager 实例
     * @param DesiredRole 目标任务角色
     * @param ExcludeId 需要排除的无人机 ID
     */
    // 解释：这一行把右侧表达式的结果写入 `ADronePawn* FindDroneByRole(UAgentManager* Manager, EDroneMissionRole DesiredRole, const FString& ExcludeId`，完成 adronePawnfind无人机byroleuagent管理器管理器edronemissionroledesiredroleconstfstringexcludeid 的更新。
    ADronePawn* FindDroneByRole(UAgentManager* Manager, EDroneMissionRole DesiredRole, const FString& ExcludeId = TEXT("")) const;

    /** @brief 构造错误 JSON 字符串 */
    // 解释：调用 `MakeError` 执行当前步骤需要的功能逻辑。
    FString MakeError(const FString& Msg) const;

    /** @brief 构造成功 JSON 字符串 */
    // 解释：这一行把右侧表达式的结果写入 `FString MakeOk(const FString& Msg`，完成 fstringmakeokconstfstringmsg 的更新。
    FString MakeOk(const FString& Msg = TEXT("ok")) const;

    /** @brief 目标状态 Kalman 预测器 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY()
    // 解释：这一行声明成员或局部变量 `Predictor`，用于保存预测器。
    UKalmanPredictor* Predictor = nullptr;

    /** @brief 视觉拦截控制器 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY()
    // 解释：这一行声明成员或局部变量 `VisualInterceptController`，用于保存视觉拦截控制器。
    UVisualInterceptController* VisualInterceptController = nullptr;

    /** @brief 当前启用的常规制导算法对象，生命周期由本类管理 */
    // 解释：这一行声明成员或局部变量 `CurrentMethod`，用于保存currentmethod。
    IGuidanceMethod* CurrentMethod = nullptr;

    /** @brief 当前制导算法名称 */
    // 解释：这一行把右侧表达式的结果写入 `FString CurrentMethodName`，完成 fstringcurrentmethodname 的更新。
    FString CurrentMethodName = TEXT("predictive");

    /** @brief 最近一次输出的俯仰瞄准角（deg） */
    // 解释：这一行声明成员或局部变量 `LastPitch`，用于保存lastpitch。
    float LastPitch = 0.0f;

    /** @brief 最近一次输出的偏航瞄准角（deg） */
    // 解释：这一行声明成员或局部变量 `LastYaw`，用于保存lastyaw。
    float LastYaw = 0.0f;

    /** @brief 最近一次计算得到的瞄准点 */
    // 解释：这一行声明成员或局部变量 `LastAimPoint`，用于保存lastaimpoint。
    FVector LastAimPoint = FVector::ZeroVector;

    /** @brief 最近一次估计的弹丸飞行时间（s） */
    // 解释：这一行声明成员或局部变量 `LastFlightTime`，用于保存lastflighttime。
    float LastFlightTime = 0.0f;

    /** @brief 默认视觉延迟补偿值（s） */
    // 解释：这一行声明成员或局部变量 `DefaultVisionLatency`，用于保存defaultvisionlatency。
    float DefaultVisionLatency = 0.08f;

    /** @brief 最近一次参与计算的延迟补偿值（s） */
    // 解释：这一行声明成员或局部变量 `LastLatencyCompensation`，用于保存lastlatencycompensation。
    float LastLatencyCompensation = 0.08f;

    /** @brief 当前默认的无人机拦截方法名 */
    // 解释：这一行把右侧表达式的结果写入 `FString CurrentInterceptMethod`，完成 fstringcurrent拦截method 的更新。
    FString CurrentInterceptMethod = TEXT("pure_pursuit");

    /** @brief 拦截机默认速度上限（m/s） */
    // 解释：这一行声明成员或局部变量 `InterceptorSpeed`，用于保存interceptorspeed。
    float InterceptorSpeed = 8.0f;

    /** @brief 比例导航增益 N */
    // 解释：这一行声明成员或局部变量 `InterceptNavGain`，用于保存拦截navgain。
    float InterceptNavGain = 3.0f;

    /** @brief 纯追踪/前置法使用的预测时间（s） */
    // 解释：这一行声明成员或局部变量 `InterceptLeadTime`，用于保存拦截leadtime。
    float InterceptLeadTime = 0.6f;

    /** @brief 判定捕获成功的距离阈值（m） */
    // 解释：这一行声明成员或局部变量 `CaptureRadius`，用于保存采集radius。
    float CaptureRadius = 1.5f;

    /** @brief 最近一次自动拦截使用的拦截机 ID */
    // 解释：这一行声明成员或局部变量 `LastInterceptorId`，用于保存lastinterceptorid。
    FString LastInterceptorId;

    /** @brief 最近一次自动拦截使用的目标机 ID */
    // 解释：这一行声明成员或局部变量 `LastTargetId`，用于保存lasttargetid。
    FString LastTargetId;

    /** @brief 最近一次下发给拦截机的速度指令（m/s） */
    // 解释：这一行声明成员或局部变量 `LastInterceptorCmdVel`，用于保存lastinterceptorcmdvel。
    FVector LastInterceptorCmdVel = FVector::ZeroVector;

    /** @brief 最近一次自动拦截时的目标距离（m） */
    // 解释：这一行声明成员或局部变量 `LastDistanceToTarget`，用于保存lastdistancetotarget。
    float LastDistanceToTarget = 0.0f;

    /** @brief 最近一次自动拦截时的闭合速度（m/s） */
    // 解释：这一行声明成员或局部变量 `LastClosingSpeed`，用于保存lastclosingspeed。
    float LastClosingSpeed = 0.0f;

    /** @brief 最近一次自动拦截是否生成了有效控制量 */
    // 解释：这一行声明成员或局部变量 `bLastInterceptValid`，用于保存布尔标志 last拦截valid。
    bool bLastInterceptValid = false;

    /** @brief 最近一次自动拦截是否已判定捕获 */
    // 解释：这一行声明成员或局部变量 `bLastCaptured`，用于保存布尔标志 lastcaptured。
    bool bLastCaptured = false;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
};

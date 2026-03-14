// 解释：引入当前实现文件对应的头文件 `VisualInterceptController.h`，使实现部分能够看到类和函数声明。
#include "VisualInterceptController.h"

// 解释：引入 `JsonObject.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "Dom/JsonObject.h"
// 解释：引入 `PIDController.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "GraduationProject/Core/Controller/PIDController.h"
// 解释：引入 `AgentManager.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "GraduationProject/Core/Manager/AgentManager.h"
// 解释：引入 `DronePawn.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "GraduationProject/Drone/DronePawn.h"
// 解释：引入 `KalmanPredictor.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "KalmanPredictor.h"

// 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
namespace
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    /** @brief 视觉拦截默认时间步长（s） */
    // 解释：这一行声明成员或局部变量 `kDefaultDt`，用于保存Kdefaultdt。
    constexpr float kDefaultDt = 0.08f;

    /**
     * @brief 从 JSON 字段中读取数值并转换为目标类型
     * @param Obj JSON 对象
     * @param Field 字段名
     * @param OutValue 输出值
     * @return 字段存在且读取成功时返回 true
     */
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    template <typename T>
    // 解释：这一行定义函数 `TryGetNumberAs`，开始实现trygetnumberas的具体逻辑。
    bool TryGetNumberAs(const TSharedPtr<FJsonObject>& Obj, const TCHAR* Field, T& OutValue)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (!Obj.IsValid() || !Obj->HasField(Field))
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return false;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行把右侧表达式的结果写入 `OutValue`，完成 outvalue 的更新。
        OutValue = static_cast<T>(Obj->GetNumberField(Field));
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return true;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    /**
     * @brief 按任务角色查找无人机
     * @param Manager AgentManager 实例
     * @param Role 目标角色
     * @param ExcludeId 需要排除的无人机 ID
     */
    // 解释：这一行定义函数 `FindDroneByRole`，开始实现find无人机byrole的具体逻辑。
    ADronePawn* FindDroneByRole(UAgentManager* Manager, EDroneMissionRole Role, const FString& ExcludeId = TEXT(""))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (!Manager)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return nullptr;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：调用 `GetAllAgentIds` 执行当前步骤需要的功能逻辑。
        const TArray<FString> AgentIds = Manager->GetAllAgentIds();
        // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
        for (const FString& AgentId : AgentIds)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
            if (!ExcludeId.IsEmpty() && AgentId == ExcludeId)
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            {
                // 解释：这一行跳过本轮循环剩余语句，直接进入下一轮迭代。
                continue;
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            }

            // 解释：这一行把右侧表达式的结果写入 `ADronePawn* Drone`，完成 adronePawn无人机 的更新。
            ADronePawn* Drone = Cast<ADronePawn>(Manager->GetAgent(AgentId));
            // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
            if (Drone && Drone->MissionRole == Role)
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            {
                // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
                return Drone;
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            }
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return nullptr;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 惰性初始化预测器与三个 PID 控制器
 *
 * - `FeaturePredictor`：对 `(cx, cy, area)` 做 Kalman 平滑与短时预测；
 * - `YawPid`：控制偏航纠偏；
 * - `VerticalPid`：控制垂向速度；
 * - `ForwardPid`：根据目标面积控制前冲速度。
 */
// 解释：这一行定义函数 `EnsureInitialized`，开始实现ensureinitialized的具体逻辑。
void UVisualInterceptController::EnsureInitialized()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!FeaturePredictor)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `FeaturePredictor`，完成 feature预测器 的更新。
        FeaturePredictor = NewObject<UKalmanPredictor>(this);
        // 解释：调用 `Initialize` 执行当前步骤需要的功能逻辑。
        FeaturePredictor->Initialize(25.0f, 40.0f);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!YawPid)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `YawPid`，完成 yawPID 的更新。
        YawPid = NewObject<UPIDController>(this);
        // 解释：调用 `Initialize` 执行当前步骤需要的功能逻辑。
        YawPid->Initialize(42.0, 0.0, 12.0, 0.03, Params.MaxYawRateDeg, 0.02, -200.0, 200.0);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!VerticalPid)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `VerticalPid`，完成 verticalPID 的更新。
        VerticalPid = NewObject<UPIDController>(this);
        // 解释：调用 `Initialize` 执行当前步骤需要的功能逻辑。
        VerticalPid->Initialize(2.2, 0.0, 0.7, 0.05, Params.MaxVerticalSpeed, 0.02, -5.0, 5.0);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!ForwardPid)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `ForwardPid`，完成 forwardPID 的更新。
        ForwardPid = NewObject<UPIDController>(this);
        // 解释：调用 `Initialize` 执行当前步骤需要的功能逻辑。
        ForwardPid->Initialize(80.0, 0.0, 22.0, 0.05, Params.MaxForwardSpeed, 0.02, -5.0, 5.0);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 仅重置会话期运行数据，不触碰 PID 与预测器参数 */
// 解释：这一行定义函数 `ResetRuntimeOnly`，开始实现resetruntimeonly的具体逻辑。
void UVisualInterceptController::ResetRuntimeOnly()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `Runtime`，完成 runtime 的更新。
    Runtime = FVisualRuntime();
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 清空三个 PID 控制器的积分项和历史误差 */
// 解释：这一行定义函数 `ResetPidControllers`，开始实现resetPIDcontrollers的具体逻辑。
void UVisualInterceptController::ResetPidControllers()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (YawPid)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `Reset` 执行当前步骤需要的功能逻辑。
        YawPid->Reset();
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (VerticalPid)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `Reset` 执行当前步骤需要的功能逻辑。
        VerticalPid->Reset();
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (ForwardPid)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `Reset` 执行当前步骤需要的功能逻辑。
        ForwardPid->Reset();
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 完全重置视觉拦截控制器
 *
 * 该接口会重置：
 * - 特征 Kalman 预测器；
 * - 三个 PID 控制器；
 * - 当前运行时状态机与统计量。
 */
// 解释：这一行定义函数 `Reset`，开始实现reset的具体逻辑。
void UVisualInterceptController::Reset()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：调用 `EnsureInitialized` 执行当前步骤需要的功能逻辑。
    EnsureInitialized();

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (FeaturePredictor)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `Reset` 执行当前步骤需要的功能逻辑。
        FeaturePredictor->Reset();
        // 解释：调用 `Initialize` 执行当前步骤需要的功能逻辑。
        FeaturePredictor->Initialize(25.0f, 40.0f);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：调用 `ResetPidControllers` 执行当前步骤需要的功能逻辑。
    ResetPidControllers();
    // 解释：调用 `ResetRuntimeOnly` 执行当前步骤需要的功能逻辑。
    ResetRuntimeOnly();
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 启动新的视觉拦截会话并初始化运行时状态
 * @param Interceptor 当前选中的拦截无人机
 */
// 解释：这一行定义函数 `BeginSession`，开始实现beginsession的具体逻辑。
void UVisualInterceptController::BeginSession(const ADronePawn* Interceptor)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `Runtime.bEnabled`，完成 布尔标志 enabled 的更新。
    Runtime.bEnabled = true;
    // 解释：这一行把右侧表达式的结果写入 `Runtime.bCaptured`，完成 布尔标志 captured 的更新。
    Runtime.bCaptured = false;
    // 解释：这一行把右侧表达式的结果写入 `Runtime.State`，完成 状态 的更新。
    Runtime.State = EVisualState::Search;
    // 解释：这一行把右侧表达式的结果写入 `Runtime.FrameCount`，完成 framecount 的更新。
    Runtime.FrameCount = 0;
    // 解释：这一行把右侧表达式的结果写入 `Runtime.DetectionCount`，完成 detectioncount 的更新。
    Runtime.DetectionCount = 0;
    // 解释：这一行把右侧表达式的结果写入 `Runtime.LostCount`，完成 lostcount 的更新。
    Runtime.LostCount = 0;
    // 解释：这一行把右侧表达式的结果写入 `Runtime.CaptureCount`，完成 采集count 的更新。
    Runtime.CaptureCount = 0;
    // 解释：这一行把右侧表达式的结果写入 `Runtime.LastEx`，完成 lastex 的更新。
    Runtime.LastEx = 0.0f;
    // 解释：这一行把右侧表达式的结果写入 `Runtime.LastEy`，完成 lastey 的更新。
    Runtime.LastEy = 0.0f;
    // 解释：这一行把右侧表达式的结果写入 `Runtime.LastAreaNorm`，完成 lastareanorm 的更新。
    Runtime.LastAreaNorm = 0.0f;
    // 解释：这一行把右侧表达式的结果写入 `Runtime.LastConf`，完成 lastconf 的更新。
    Runtime.LastConf = 0.0f;
    // 解释：这一行把右侧表达式的结果写入 `Runtime.LastYawCmdDeg`，完成 lastyawcmddeg 的更新。
    Runtime.LastYawCmdDeg = Interceptor ? Interceptor->CurrentState.GetRotator().Yaw : 0.0f;
    // 解释：这一行把右侧表达式的结果写入 `Runtime.LastYawRateDeg`，完成 lastyawratedeg 的更新。
    Runtime.LastYawRateDeg = 0.0f;
    // 解释：这一行把右侧表达式的结果写入 `Runtime.LastCmdVel`，完成 lastcmdvel 的更新。
    Runtime.LastCmdVel = FVector::ZeroVector;
    // 解释：这一行把右侧表达式的结果写入 `Runtime.LastTargetDistance`，完成 lasttargetdistance 的更新。
    Runtime.LastTargetDistance = -1.0f;
    // 解释：这一行把右侧表达式的结果写入 `Runtime.SearchCamYawDeg`，完成 searchcamyawdeg 的更新。
    Runtime.SearchCamYawDeg = 0.0f;
    // 解释：这一行把右侧表达式的结果写入 `Runtime.SearchDir`，完成 searchdir 的更新。
    Runtime.SearchDir = 1.0f;
    // 解释：这一行把右侧表达式的结果写入 `Runtime.SearchTime`，完成 searchtime 的更新。
    Runtime.SearchTime = 0.0f;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 布尔值转 JSON 字面量文本 */
// 解释：这一行定义函数 `BoolLiteral`，开始实现boolliteral的具体逻辑。
FString UVisualInterceptController::BoolLiteral(bool bValue)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行使用三目表达式选择返回值，在正常路径和兜底路径之间做快速切换。
    return bValue ? TEXT("true") : TEXT("false");
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 将偏航角归一化到 `[-180, 180)` 区间 */
// 解释：这一行定义函数 `NormalizeYawDeg`，开始实现normalizeyawdeg的具体逻辑。
float UVisualInterceptController::NormalizeYawDeg(float YawDeg)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return FRotator::NormalizeAxis(YawDeg);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 视觉状态机枚举转字符串 */
// 解释：这一行定义函数 `StateToString`，开始实现状态tostring的具体逻辑。
FString UVisualInterceptController::StateToString(EVisualState State)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行进入 `switch` 分支分派结构，后续会按不同枚举或状态值执行不同逻辑。
    switch (State)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
    // 解释：这一行声明 `switch` 中的一个分支标签，对应某个具体取值的处理路径。
    case EVisualState::Search:
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return TEXT("SEARCH");
    // 解释：这一行声明 `switch` 中的一个分支标签，对应某个具体取值的处理路径。
    case EVisualState::Track:
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return TEXT("TRACK");
    // 解释：这一行声明 `switch` 中的一个分支标签，对应某个具体取值的处理路径。
    case EVisualState::Approach:
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return TEXT("APPROACH");
    // 解释：这一行声明 `switch` 中的一个分支标签，对应某个具体取值的处理路径。
    case EVisualState::Captured:
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return TEXT("CAPTURED");
    // 解释：这一行声明 `switch` 中的一个分支标签，对应某个具体取值的处理路径。
    case EVisualState::Idle:
    // 解释：这一行声明 `switch` 的默认分支，当前面所有 `case` 都不匹配时执行。
    default:
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return TEXT("IDLE");
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 规范化视觉拦截方法名
 * @param InMethod 原始方法名
 * @param OutCanonical 输出的标准方法名
 * @return 是否为支持的方法
 */
// 解释：这一行定义函数 `NormalizeMethod`，开始实现normalizemethod的具体逻辑。
bool UVisualInterceptController::NormalizeMethod(const FString& InMethod, FString& OutCanonical)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行声明成员或局部变量 `Method`，用于保存method。
    FString Method = InMethod;
    // 解释：调用 `TrimStartAndEndInline` 执行当前步骤需要的功能逻辑。
    Method.TrimStartAndEndInline();
    // 解释：调用 `ToLowerInline` 执行当前步骤需要的功能逻辑。
    Method.ToLowerInline();

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (Method.IsEmpty() || Method == TEXT("auto"))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `OutCanonical`，完成 outcanonical 的更新。
        OutCanonical = TEXT("vision_pid_kalman");
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return true;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (Method == TEXT("vision_pid_kalman") || Method == TEXT("visual_pid_kalman") || Method == TEXT("pid_kalman"))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `OutCanonical`，完成 outcanonical 的更新。
        OutCanonical = TEXT("vision_pid_kalman");
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return true;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (Method == TEXT("vision_pid") || Method == TEXT("visual_pid") || Method == TEXT("pid"))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `OutCanonical`，完成 outcanonical 的更新。
        OutCanonical = TEXT("vision_pid");
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return true;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (Method == TEXT("vision_pd_kalman") || Method == TEXT("visual_pd_kalman") || Method == TEXT("pd_kalman"))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `OutCanonical`，完成 outcanonical 的更新。
        OutCanonical = TEXT("vision_pd_kalman");
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return true;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (Method == TEXT("vision_pd") || Method == TEXT("visual_pd") || Method == TEXT("pd"))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `OutCanonical`，完成 outcanonical 的更新。
        OutCanonical = TEXT("vision_pd");
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return true;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return false;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 生成统一错误返回 JSON */
// 解释：这一行定义函数 `MakeError`，开始实现makeerror的具体逻辑。
FString UVisualInterceptController::MakeError(const FString& Msg) const
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return FString::Printf(TEXT("{\"status\":\"error\",\"message\":\"%s\"}"), *Msg.ReplaceCharWithEscapedChar());
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 生成统一成功返回 JSON */
// 解释：这一行定义函数 `MakeOk`，开始实现makeok的具体逻辑。
FString UVisualInterceptController::MakeOk(const FString& Msg) const
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return FString::Printf(TEXT("{\"status\":\"ok\",\"message\":\"%s\"}"), *Msg.ReplaceCharWithEscapedChar());
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 对时间步长做有效性检查与限幅
 * @param Dt 原始时间步长（s）
 * @return 裁剪后的时间步长，范围 `[0.01, 0.25]`
 */
// 解释：这一行定义函数 `SafeDt`，开始实现safedt的具体逻辑。
float UVisualInterceptController::SafeDt(float Dt) const
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!FMath::IsFinite(Dt))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `Dt`，完成 dt 的更新。
        Dt = kDefaultDt;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return FMath::Clamp(Dt, 0.01f, 0.25f);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 将当前时间步长同步到三个 PID 控制器 */
// 解释：这一行定义函数 `SyncPidTimeStep`，开始实现syncPIDtimestep的具体逻辑。
void UVisualInterceptController::SyncPidTimeStep(float Dt)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (YawPid)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `SetTimeStep` 执行当前步骤需要的功能逻辑。
        YawPid->SetTimeStep(Dt);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (VerticalPid)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `SetTimeStep` 执行当前步骤需要的功能逻辑。
        VerticalPid->SetTimeStep(Dt);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (ForwardPid)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `SetTimeStep` 执行当前步骤需要的功能逻辑。
        ForwardPid->SetTimeStep(Dt);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 从 JSON 命令中读取并校验全部视觉拦截参数
 * @param CmdObj JSON 命令对象
 *
 * 读入后会统一执行限幅，并把新的限幅值同步回 PID 输出上限。
 */
// 解释：这一行定义函数 `ApplyParamsFromJson`，开始实现applyparamsfromjson的具体逻辑。
void UVisualInterceptController::ApplyParamsFromJson(const TSharedPtr<FJsonObject>& CmdObj)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!CmdObj.IsValid())
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：调用 `TryGetNumberAs` 执行当前步骤需要的功能逻辑。
    TryGetNumberAs(CmdObj, TEXT("desired_area"), Params.DesiredArea);
    // 解释：调用 `TryGetNumberAs` 执行当前步骤需要的功能逻辑。
    TryGetNumberAs(CmdObj, TEXT("capture_area"), Params.CaptureArea);
    // 解释：调用 `TryGetNumberAs` 执行当前步骤需要的功能逻辑。
    TryGetNumberAs(CmdObj, TEXT("center_tol_x"), Params.CenterTolX);
    // 解释：调用 `TryGetNumberAs` 执行当前步骤需要的功能逻辑。
    TryGetNumberAs(CmdObj, TEXT("center_tol_y"), Params.CenterTolY);
    // 解释：调用 `TryGetNumberAs` 执行当前步骤需要的功能逻辑。
    TryGetNumberAs(CmdObj, TEXT("capture_hold_frames"), Params.CaptureHoldFrames);
    // 解释：调用 `TryGetNumberAs` 执行当前步骤需要的功能逻辑。
    TryGetNumberAs(CmdObj, TEXT("lost_to_search_frames"), Params.LostToSearchFrames);

    // 解释：调用 `TryGetNumberAs` 执行当前步骤需要的功能逻辑。
    TryGetNumberAs(CmdObj, TEXT("max_forward_speed"), Params.MaxForwardSpeed);
    // 解释：调用 `TryGetNumberAs` 执行当前步骤需要的功能逻辑。
    TryGetNumberAs(CmdObj, TEXT("max_reverse_speed"), Params.MaxReverseSpeed);
    // 解释：调用 `TryGetNumberAs` 执行当前步骤需要的功能逻辑。
    TryGetNumberAs(CmdObj, TEXT("max_vertical_speed"), Params.MaxVerticalSpeed);
    // 解释：调用 `TryGetNumberAs` 执行当前步骤需要的功能逻辑。
    TryGetNumberAs(CmdObj, TEXT("max_yaw_rate_deg"), Params.MaxYawRateDeg);
    // 解释：调用 `TryGetNumberAs` 执行当前步骤需要的功能逻辑。
    TryGetNumberAs(CmdObj, TEXT("ram_area_target"), Params.RamAreaTarget);
    // 解释：调用 `TryGetNumberAs` 执行当前步骤需要的功能逻辑。
    TryGetNumberAs(CmdObj, TEXT("min_ram_speed"), Params.MinRamSpeed);
    // 解释：调用 `TryGetNumberAs` 执行当前步骤需要的功能逻辑。
    TryGetNumberAs(CmdObj, TEXT("intercept_distance"), Params.InterceptDistance);

    // 解释：调用 `TryGetNumberAs` 执行当前步骤需要的功能逻辑。
    TryGetNumberAs(CmdObj, TEXT("search_cam_yaw_limit_deg"), Params.SearchCamYawLimitDeg);
    // 解释：调用 `TryGetNumberAs` 执行当前步骤需要的功能逻辑。
    TryGetNumberAs(CmdObj, TEXT("search_cam_rate_deg"), Params.SearchCamRateDeg);
    // 解释：调用 `TryGetNumberAs` 执行当前步骤需要的功能逻辑。
    TryGetNumberAs(CmdObj, TEXT("search_body_yaw_rate_deg"), Params.SearchBodyYawRateDeg);
    // 解释：调用 `TryGetNumberAs` 执行当前步骤需要的功能逻辑。
    TryGetNumberAs(CmdObj, TEXT("search_cam_pitch_deg"), Params.SearchCamPitchDeg);
    // 解释：调用 `TryGetNumberAs` 执行当前步骤需要的功能逻辑。
    TryGetNumberAs(CmdObj, TEXT("search_vz_amp"), Params.SearchVzAmp);

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (CmdObj->HasField(TEXT("stop_on_capture")))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `Params.bStopOnCapture`，完成 布尔标志 stopon采集 的更新。
        Params.bStopOnCapture = CmdObj->GetBoolField(TEXT("stop_on_capture"));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (CmdObj->HasField(TEXT("use_kalman")))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `Params.bUseKalman`，完成 布尔标志 use卡尔曼 的更新。
        Params.bUseKalman = CmdObj->GetBoolField(TEXT("use_kalman"));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行先对计算结果做限幅，再写入 `Params.DesiredArea`，防止 desiredarea 超出允许范围。
    Params.DesiredArea = FMath::Clamp(Params.DesiredArea, 0.0001f, 0.95f);
    // 解释：这一行先对计算结果做限幅，再写入 `Params.CaptureArea`，防止 采集area 超出允许范围。
    Params.CaptureArea = FMath::Clamp(Params.CaptureArea, Params.DesiredArea, 0.98f);
    // 解释：这一行先对计算结果做限幅，再写入 `Params.CenterTolX`，防止 centertolX 超出允许范围。
    Params.CenterTolX = FMath::Clamp(Params.CenterTolX, 0.001f, 1.0f);
    // 解释：这一行先对计算结果做限幅，再写入 `Params.CenterTolY`，防止 centertolY 超出允许范围。
    Params.CenterTolY = FMath::Clamp(Params.CenterTolY, 0.001f, 1.0f);
    // 解释：这一行通过 `FMath::Max` 给 `Params.CaptureHoldFrames` 施加下界约束，避免 采集holdframes 过小。
    Params.CaptureHoldFrames = FMath::Max(1, Params.CaptureHoldFrames);
    // 解释：这一行通过 `FMath::Max` 给 `Params.LostToSearchFrames` 施加下界约束，避免 losttosearchframes 过小。
    Params.LostToSearchFrames = FMath::Max(1, Params.LostToSearchFrames);

    // 解释：这一行通过 `FMath::Max` 给 `Params.MaxForwardSpeed` 施加下界约束，避免 maxforwardspeed 过小。
    Params.MaxForwardSpeed = FMath::Max(0.1f, Params.MaxForwardSpeed);
    // 解释：这一行通过 `FMath::Max` 给 `Params.MaxReverseSpeed` 施加下界约束，避免 maxreversespeed 过小。
    Params.MaxReverseSpeed = FMath::Max(0.0f, Params.MaxReverseSpeed);
    // 解释：这一行通过 `FMath::Max` 给 `Params.MaxVerticalSpeed` 施加下界约束，避免 maxverticalspeed 过小。
    Params.MaxVerticalSpeed = FMath::Max(0.1f, Params.MaxVerticalSpeed);
    // 解释：这一行通过 `FMath::Max` 给 `Params.MaxYawRateDeg` 施加下界约束，避免 maxyawratedeg 过小。
    Params.MaxYawRateDeg = FMath::Max(1.0f, Params.MaxYawRateDeg);
    // 解释：这一行先对计算结果做限幅，再写入 `Params.RamAreaTarget`，防止 ramareatarget 超出允许范围。
    Params.RamAreaTarget = FMath::Clamp(FMath::Max(Params.RamAreaTarget, Params.CaptureArea), Params.CaptureArea, 0.65f);
    // 解释：这一行先对计算结果做限幅，再写入 `Params.MinRamSpeed`，防止 minramspeed 超出允许范围。
    Params.MinRamSpeed = FMath::Clamp(Params.MinRamSpeed, 0.0f, Params.MaxForwardSpeed);
    // 解释：这一行先对计算结果做限幅，再写入 `Params.InterceptDistance`，防止 拦截distance 超出允许范围。
    Params.InterceptDistance = FMath::Clamp(Params.InterceptDistance, 0.2f, 20.0f);

    // 解释：这一行先对计算结果做限幅，再写入 `Params.SearchCamYawLimitDeg`，防止 searchcamyawlimitdeg 超出允许范围。
    Params.SearchCamYawLimitDeg = FMath::Clamp(Params.SearchCamYawLimitDeg, 10.0f, 179.0f);
    // 解释：这一行先对计算结果做限幅，再写入 `Params.SearchCamRateDeg`，防止 searchcamratedeg 超出允许范围。
    Params.SearchCamRateDeg = FMath::Clamp(Params.SearchCamRateDeg, 1.0f, 180.0f);
    // 解释：这一行先对计算结果做限幅，再写入 `Params.SearchBodyYawRateDeg`，防止 searchbodyyawratedeg 超出允许范围。
    Params.SearchBodyYawRateDeg = FMath::Clamp(Params.SearchBodyYawRateDeg, 0.0f, 120.0f);
    // 解释：这一行先对计算结果做限幅，再写入 `Params.SearchCamPitchDeg`，防止 searchcampitchdeg 超出允许范围。
    Params.SearchCamPitchDeg = FMath::Clamp(Params.SearchCamPitchDeg, -80.0f, 45.0f);
    // 解释：这一行先对计算结果做限幅，再写入 `Params.SearchVzAmp`，防止 searchvzamp 超出允许范围。
    Params.SearchVzAmp = FMath::Clamp(Params.SearchVzAmp, 0.0f, 5.0f);

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (YawPid)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `SetParameters` 执行当前步骤需要的功能逻辑。
        YawPid->SetParameters(YawPid->Kp, YawPid->Ki, YawPid->Kd, YawPid->DiffFilterTau, Params.MaxYawRateDeg);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (VerticalPid)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `SetParameters` 执行当前步骤需要的功能逻辑。
        VerticalPid->SetParameters(VerticalPid->Kp, VerticalPid->Ki, VerticalPid->Kd, VerticalPid->DiffFilterTau, Params.MaxVerticalSpeed);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (ForwardPid)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `SetParameters` 执行当前步骤需要的功能逻辑。
        ForwardPid->SetParameters(ForwardPid->Kp, ForwardPid->Ki, ForwardPid->Kd, ForwardPid->DiffFilterTau, Params.MaxForwardSpeed);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 解析并锁定拦截无人机
 * @param CmdObj 当前命令对象
 * @return 成功解析出的拦截无人机
 *
 * 查找优先级：
 * 1. JSON 中给定的 `interceptor_id`；
 * 2. `MissionRole == Interceptor`；
 * 3. 兜底 `drone_1`；
 * 4. 最后尝试 `drone_0`。
 */
// 解释：这一行定义函数 `ResolveInterceptor`，开始实现resolveinterceptor的具体逻辑。
ADronePawn* UVisualInterceptController::ResolveInterceptor(const TSharedPtr<FJsonObject>& CmdObj)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `UAgentManager* Manager`，完成 uagent管理器管理器 的更新。
    UAgentManager* Manager = UAgentManager::GetInstance();
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!Manager)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return nullptr;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行声明成员或局部变量 `DesiredId`，用于保存desiredid。
    FString DesiredId = Runtime.InterceptorId;
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (CmdObj.IsValid() && CmdObj->HasField(TEXT("interceptor_id")))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `DesiredId`，完成 desiredid 的更新。
        DesiredId = CmdObj->GetStringField(TEXT("interceptor_id"));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行声明成员或局部变量 `Drone`，用于保存无人机。
    ADronePawn* Drone = nullptr;
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!DesiredId.IsEmpty())
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `Drone`，完成 无人机 的更新。
        Drone = Cast<ADronePawn>(Manager->GetAgent(DesiredId));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!Drone)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `Drone`，完成 无人机 的更新。
        Drone = FindDroneByRole(Manager, EDroneMissionRole::Interceptor);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!Drone)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `Drone`，完成 无人机 的更新。
        Drone = Cast<ADronePawn>(Manager->GetAgent(TEXT("drone_1")));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!Drone)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `Drone`，完成 无人机 的更新。
        Drone = Cast<ADronePawn>(Manager->GetAgent(TEXT("drone_0")));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (Drone)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `Runtime.InterceptorId`，完成 interceptorid 的更新。
        Runtime.InterceptorId = Drone->DroneId;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return Drone;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 解析并锁定目标无人机
 * @param CmdObj 当前命令对象
 * @return 成功解析出的目标无人机
 */
// 解释：这一行定义函数 `ResolveTarget`，开始实现resolvetarget的具体逻辑。
ADronePawn* UVisualInterceptController::ResolveTarget(const TSharedPtr<FJsonObject>& CmdObj)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `UAgentManager* Manager`，完成 uagent管理器管理器 的更新。
    UAgentManager* Manager = UAgentManager::GetInstance();
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!Manager)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return nullptr;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行声明成员或局部变量 `DesiredId`，用于保存desiredid。
    FString DesiredId = Runtime.TargetId;
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (CmdObj.IsValid() && CmdObj->HasField(TEXT("target_id")))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `DesiredId`，完成 desiredid 的更新。
        DesiredId = CmdObj->GetStringField(TEXT("target_id"));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行声明成员或局部变量 `Drone`，用于保存无人机。
    ADronePawn* Drone = nullptr;
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!DesiredId.IsEmpty())
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `Drone`，完成 无人机 的更新。
        Drone = Cast<ADronePawn>(Manager->GetAgent(DesiredId));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!Drone)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `Drone`，完成 无人机 的更新。
        Drone = FindDroneByRole(Manager, EDroneMissionRole::Target, Runtime.InterceptorId);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!Drone)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `const TCHAR* FallbackIds[]`，完成 consttcharfallbackids 的更新。
        const TCHAR* FallbackIds[] = {TEXT("drone_0"), TEXT("drone_1")};
        // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
        for (const TCHAR* FallbackId : FallbackIds)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
            if (!Runtime.InterceptorId.IsEmpty() && Runtime.InterceptorId == FallbackId)
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            {
                // 解释：这一行跳过本轮循环剩余语句，直接进入下一轮迭代。
                continue;
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            }
            // 解释：这一行把右侧表达式的结果写入 `Drone`，完成 无人机 的更新。
            Drone = Cast<ADronePawn>(Manager->GetAgent(FallbackId));
            // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
            if (Drone)
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            {
                // 解释：这一行立即跳出当前循环或 `switch` 分支，避免继续执行后续分支。
                break;
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            }
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (Drone)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `Runtime.TargetId`，完成 targetid 的更新。
        Runtime.TargetId = Drone->DroneId;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return Drone;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 构造启动响应 JSON，返回当前参数快照与会话标识 */
// 解释：这一行定义函数 `BuildStartJson`，开始实现buildstartjson的具体逻辑。
FString UVisualInterceptController::BuildStartJson() const
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return FString::Printf(
        // 解释：这一行位于构造函数初始化列表中，把 `TEXT` 直接初始化为 `"{\"status\":\"ok\",\"mode\":\"visual_intercept\",\"enabled\":%s,\"state\":\"%s\",\"interceptor_id\":\"%s\",\"target_id\":\"%s\",\"method\":\"%s\",\"params\":{\"desired_area\":%.5f,\"capture_area\":%.5f,\"center_tol_x\":%.4f,\"center_tol_y\":%.4f,\"capture_hold_frames\":%d,\"lost_to_search_frames\":%d,\"max_forward_speed\":%.3f,\"max_reverse_speed\":%.3f,\"max_vertical_speed\":%.3f,\"max_yaw_rate_deg\":%.3f,\"ram_area_target\":%.5f,\"min_ram_speed\":%.3f,\"intercept_distance\":%.3f,\"search_cam_yaw_limit_deg\":%.3f,\"search_cam_rate_deg\":%.3f,\"search_body_yaw_rate_deg\":%.3f,\"search_cam_pitch_deg\":%.3f,\"search_vz_amp\":%.3f,\"stop_on_capture\":%s,\"use_kalman\":%s}}"`，减少进入函数体后的额外赋值开销。
        TEXT("{\"status\":\"ok\",\"mode\":\"visual_intercept\",\"enabled\":%s,\"state\":\"%s\",\"interceptor_id\":\"%s\",\"target_id\":\"%s\",\"method\":\"%s\",\"params\":{\"desired_area\":%.5f,\"capture_area\":%.5f,\"center_tol_x\":%.4f,\"center_tol_y\":%.4f,\"capture_hold_frames\":%d,\"lost_to_search_frames\":%d,\"max_forward_speed\":%.3f,\"max_reverse_speed\":%.3f,\"max_vertical_speed\":%.3f,\"max_yaw_rate_deg\":%.3f,\"ram_area_target\":%.5f,\"min_ram_speed\":%.3f,\"intercept_distance\":%.3f,\"search_cam_yaw_limit_deg\":%.3f,\"search_cam_rate_deg\":%.3f,\"search_body_yaw_rate_deg\":%.3f,\"search_cam_pitch_deg\":%.3f,\"search_vz_amp\":%.3f,\"stop_on_capture\":%s,\"use_kalman\":%s}}"),
        *BoolLiteral(Runtime.bEnabled),
        *StateToString(Runtime.State),
        *Runtime.InterceptorId,
        *Runtime.TargetId,
        *Runtime.Method,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Params.DesiredArea,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Params.CaptureArea,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Params.CenterTolX,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Params.CenterTolY,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Params.CaptureHoldFrames,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Params.LostToSearchFrames,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Params.MaxForwardSpeed,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Params.MaxReverseSpeed,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Params.MaxVerticalSpeed,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Params.MaxYawRateDeg,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Params.RamAreaTarget,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Params.MinRamSpeed,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Params.InterceptDistance,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Params.SearchCamYawLimitDeg,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Params.SearchCamRateDeg,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Params.SearchBodyYawRateDeg,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Params.SearchCamPitchDeg,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Params.SearchVzAmp,
        *BoolLiteral(Params.bStopOnCapture),
        *BoolLiteral(Params.bUseKalman));
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 构造每帧更新响应 JSON，返回控制量与统计状态 */
// 解释：这一行定义函数 `BuildUpdateJson`，开始实现buildupdatejson的具体逻辑。
FString UVisualInterceptController::BuildUpdateJson(bool bValidControl) const
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return FString::Printf(
        // 解释：这一行位于构造函数初始化列表中，把 `TEXT` 直接初始化为 `"{\"status\":\"ok\",\"mode\":\"visual_intercept\",\"enabled\":%s,\"state\":\"%s\",\"captured\":%s,\"valid\":%s,\"interceptor_id\":\"%s\",\"target_id\":\"%s\",\"method\":\"%s\",\"frame\":%d,\"detections\":%d,\"lost_count\":%d,\"capture_count\":%d,\"target_distance\":%.4f,\"cmd_velocity\":[%.4f,%.4f,%.4f],\"control\":{\"ex\":%.5f,\"ey\":%.5f,\"area_norm\":%.6f,\"conf\":%.4f,\"yaw_cmd_deg\":%.4f,\"yaw_rate_deg\":%.4f}}"`，减少进入函数体后的额外赋值开销。
        TEXT("{\"status\":\"ok\",\"mode\":\"visual_intercept\",\"enabled\":%s,\"state\":\"%s\",\"captured\":%s,\"valid\":%s,\"interceptor_id\":\"%s\",\"target_id\":\"%s\",\"method\":\"%s\",\"frame\":%d,\"detections\":%d,\"lost_count\":%d,\"capture_count\":%d,\"target_distance\":%.4f,\"cmd_velocity\":[%.4f,%.4f,%.4f],\"control\":{\"ex\":%.5f,\"ey\":%.5f,\"area_norm\":%.6f,\"conf\":%.4f,\"yaw_cmd_deg\":%.4f,\"yaw_rate_deg\":%.4f}}"),
        *BoolLiteral(Runtime.bEnabled),
        *StateToString(Runtime.State),
        *BoolLiteral(Runtime.bCaptured),
        *BoolLiteral(bValidControl),
        *Runtime.InterceptorId,
        *Runtime.TargetId,
        *Runtime.Method,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Runtime.FrameCount,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Runtime.DetectionCount,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Runtime.LostCount,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Runtime.CaptureCount,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Runtime.LastTargetDistance,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Runtime.LastCmdVel.X,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Runtime.LastCmdVel.Y,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Runtime.LastCmdVel.Z,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Runtime.LastEx,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Runtime.LastEy,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Runtime.LastAreaNorm,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Runtime.LastConf,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Runtime.LastYawCmdDeg,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Runtime.LastYawRateDeg);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 根据视觉特征计算跟踪/逼近控制量
 * @param Interceptor 拦截无人机
 * @param Feature 视觉特征 `(cx, cy, area)`
 * @param FrameW 图像宽度（px）
 * @param FrameH 图像高度（px）
 * @param Confidence 当前检测置信度
 * @param Dt 时间步长（s）
 * @param bDetectionReal 本帧特征是否来自真实检测，false 表示来自 Kalman 预测
 * @param bOutCaptured [out] 本函数输出的捕获标志，当前实现固定由外层距离判定
 * @return 是否成功生成控制量
 *
 * 主要计算流程如下：
 * 1. 像面误差归一化：
 *    - $e_x = (cx - W/2) / (W/2)$
 *    - $e_y = (cy - H/2) / (H/2)$
 *    - $a = area / (W \cdot H)$
 * 2. 用相机 FOV 将水平误差映射为视线偏角：
 *    - $yaw_{img} = e_x \cdot FOV_x / 2$
 *    - $yaw_{los} = yaw_{body} + yaw_{cam} + yaw_{img}$
 * 3. 用垂向误差驱动垂直速度，用面积误差驱动前冲速度；
 * 4. 机体偏航采用受限角步进 `YawCmdDeg`，而 `YawRateCmd` 主要用于状态输出与诊断。
 */
// 解释：这一行定义函数 `ComputeTrackControl`，开始实现computetrackcontrol的具体逻辑。
bool UVisualInterceptController::ComputeTrackControl(
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
    // 解释：这一行收束函数 `ComputeTrackControl` 的签名，后面会进入实现体或以分号结束声明。
    bool& bOutCaptured)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `bOutCaptured`，完成 布尔标志 outcaptured 的更新。
    bOutCaptured = false;

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!Interceptor || FrameW <= 1.0f || FrameH <= 1.0f)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return false;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行声明成员或局部变量 `Cx`，用于保存cx。
    const float Cx = Feature.X;
    // 解释：这一行声明成员或局部变量 `Cy`，用于保存cy。
    const float Cy = Feature.Y;
    // 解释：这一行通过 `FMath::Max` 给 `const float Area` 施加下界约束，避免 constfloatarea 过小。
    const float Area = FMath::Max(1.0f, Feature.Z);

    // 解释：这一行通过 `FMath::Max` 给 `const float Ex` 施加下界约束，避免 constfloatex 过小。
    const float Ex = (Cx - 0.5f * FrameW) / FMath::Max(1.0f, 0.5f * FrameW);
    // 解释：这一行通过 `FMath::Max` 给 `const float Ey` 施加下界约束，避免 constfloatey 过小。
    const float Ey = (Cy - 0.5f * FrameH) / FMath::Max(1.0f, 0.5f * FrameH);
    // 解释：这一行通过 `FMath::Max` 给 `const float AreaNorm` 施加下界约束，避免 constfloatareanorm 过小。
    const float AreaNorm = Area / FMath::Max(1.0f, FrameW * FrameH);

    // 解释：这一行通过 `FMath::Max` 给 `const float HorizontalFovDeg` 施加下界约束，避免 constfloathorizontalfovdeg 过小。
    const float HorizontalFovDeg = FMath::Max(30.0f, Interceptor->CameraFOV);
    // 解释：这一行声明成员或局部变量 `HalfHorizontalFovDeg`，用于保存halfhorizontalfovdeg。
    const float HalfHorizontalFovDeg = 0.5f * HorizontalFovDeg;
    // 解释：这一行通过 `FMath::Max` 给 `const float Aspect` 施加下界约束，避免 constfloataspect 过小。
    const float Aspect = FMath::Max(0.2f, FrameW / FMath::Max(1.0f, FrameH));
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    const float HalfVerticalFovDeg = FMath::RadiansToDegrees(
        // 解释：调用 `Atan` 执行当前步骤需要的功能逻辑。
        FMath::Atan(FMath::Tan(FMath::DegreesToRadians(HalfHorizontalFovDeg)) / Aspect));

    // 解释：这一行把右侧表达式的结果写入 `const float CurrentYawDeg`，完成 constfloatcurrentyawdeg 的更新。
    const float CurrentYawDeg = Interceptor->CurrentState.GetRotator().Yaw;
    // 解释：这一行把右侧表达式的结果写入 `const float CurrentCameraYawDeg`，完成 constfloatcurrent相机yawdeg 的更新。
    const float CurrentCameraYawDeg = Interceptor->GetCameraCurrentYaw();
    // 解释：这一行把右侧表达式的结果写入 `const float CurrentCameraPitchDeg`，完成 constfloatcurrent相机pitchdeg 的更新。
    const float CurrentCameraPitchDeg = Interceptor->GetCameraCurrentPitch();
    // 解释：这一行声明成员或局部变量 `ImageYawOffsetDeg`，用于保存图像yawoffsetdeg。
    const float ImageYawOffsetDeg = Ex * HalfHorizontalFovDeg;
    // 解释：这一行把右侧表达式的结果写入 `const float LineOfSightYawDeg`，完成 constfloatlineofsightyawdeg 的更新。
    const float LineOfSightYawDeg = NormalizeYawDeg(CurrentYawDeg + CurrentCameraYawDeg + ImageYawOffsetDeg);
    // 解释：这一行把右侧表达式的结果写入 `const float YawErrorDeg`，完成 constfloatyawerrordeg 的更新。
    const float YawErrorDeg = NormalizeYawDeg(LineOfSightYawDeg - CurrentYawDeg);
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    const float TrackYawError = FMath::Clamp(
        // 解释：调用 `Max` 保证结果不低于设定下界。
        YawErrorDeg / FMath::Max(1.0f, HalfHorizontalFovDeg), -2.0f, 2.0f);

    // 解释：这一行把右侧表达式的结果写入 `float YawRateCmd`，完成 floatyawratecmd 的更新。
    float YawRateCmd = static_cast<float>(YawPid ? YawPid->Update(TrackYawError, 0.0) : 0.0);
    // 解释：这一行先对计算结果做限幅，再写入 `YawRateCmd`，防止 yawratecmd 超出允许范围。
    YawRateCmd = FMath::Clamp(YawRateCmd, -Params.MaxYawRateDeg, Params.MaxYawRateDeg);

    // 解释：这一行先对计算结果做限幅，再写入 `const float DirectYawStepDeg`，防止 constfloatdirectyawstepdeg 超出允许范围。
    const float DirectYawStepDeg = FMath::Clamp(YawErrorDeg, -Params.MaxYawRateDeg * Dt, Params.MaxYawRateDeg * Dt);
    // 解释：这一行把右侧表达式的结果写入 `const float YawCmdDeg`，完成 constfloatyawcmddeg 的更新。
    const float YawCmdDeg = NormalizeYawDeg(CurrentYawDeg + DirectYawStepDeg);
    // 解释：这一行声明成员或局部变量 `VelocityYawDeg`，用于保存velocityyawdeg。
    const float VelocityYawDeg = LineOfSightYawDeg;

    // 真实检测时更积极跟随，预测模式下则稍微保守一些，降低镜头抖动。
    // 解释：这一行声明成员或局部变量 `CameraHoldFactor`，用于保存相机holdfactor。
    const float CameraHoldFactor = bDetectionReal ? 0.75f : 0.92f;
    // 解释：这一行声明成员或局部变量 `CameraPitchLimitDeg`，用于保存相机pitchlimitdeg。
    const float CameraPitchLimitDeg = 55.0f;
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    const float CameraYawTargetDeg = FMath::Clamp(
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        CurrentCameraYawDeg * CameraHoldFactor + ImageYawOffsetDeg,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        -Params.SearchCamYawLimitDeg,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Params.SearchCamYawLimitDeg);
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    const float CameraPitchTargetDeg = FMath::Clamp(
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        CurrentCameraPitchDeg * CameraHoldFactor - Ey * HalfVerticalFovDeg,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        -CameraPitchLimitDeg,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        CameraPitchLimitDeg);

    // 解释：这一行把右侧表达式的结果写入 `float VzCmd`，完成 floatvzcmd 的更新。
    float VzCmd = static_cast<float>(VerticalPid ? -VerticalPid->Update(Ey, 0.0) : 0.0);
    // 解释：这一行先对计算结果做限幅，再写入 `VzCmd`，防止 vzcmd 超出允许范围。
    VzCmd = FMath::Clamp(VzCmd, -Params.MaxVerticalSpeed, Params.MaxVerticalSpeed);

    // 解释：这一行先对计算结果做限幅，再写入 `const float HeadingAlignment`，防止 constfloatheadingalignment 超出允许范围。
    const float HeadingAlignment = FMath::Clamp(1.0f - 0.35f * FMath::Abs(TrackYawError), 0.25f, 1.0f);
    // 解释：这一行先对计算结果做限幅，再写入 `const float VerticalAlignment`，防止 constfloatverticalalignment 超出允许范围。
    const float VerticalAlignment = FMath::Clamp(1.0f - 0.30f * FMath::Abs(Ey), 0.35f, 1.0f);
    // 解释：这一行声明成员或局部变量 `AlignmentScale`，用于保存alignmentscale。
    const float AlignmentScale = HeadingAlignment * VerticalAlignment;

    // 面积越接近期望值，说明距离越近；因此以前向 PID 推进逼近过程。
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    float ForwardCmd = static_cast<float>(
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        ForwardPid ? ForwardPid->Update(Params.RamAreaTarget, AreaNorm)
                   // 解释：这一行开始构造函数初始化列表，下面会依次初始化成员变量。
                   : (Params.RamAreaTarget - AreaNorm) * Params.MaxForwardSpeed / FMath::Max(Params.RamAreaTarget, 0.001f));
    // 解释：这一行通过 `FMath::Max` 给 `ForwardCmd` 施加下界约束，避免 forwardcmd 过小。
    ForwardCmd = FMath::Max(0.0f, ForwardCmd);
    // 解释：这一行把右侧表达式的结果写入 `ForwardCmd`，完成 forwardcmd 的更新。
    ForwardCmd *= AlignmentScale;
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (bDetectionReal)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行通过 `FMath::Max` 给 `ForwardCmd` 施加下界约束，避免 forwardcmd 过小。
        ForwardCmd = FMath::Max(ForwardCmd, Params.MinRamSpeed * HeadingAlignment);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行先对计算结果做限幅，再写入 `ForwardCmd`，防止 forwardcmd 超出允许范围。
    ForwardCmd = FMath::Clamp(ForwardCmd, 0.0f, Params.MaxForwardSpeed);

    // 解释：这一行把右侧表达式的结果写入 `const float VelocityYawRad`，完成 constfloatvelocityyawrad 的更新。
    const float VelocityYawRad = FMath::DegreesToRadians(VelocityYawDeg);
    // 解释：这一行定义函数 `CmdVel`，开始实现cmdvel的具体逻辑。
    const FVector CmdVel(
        // 解释：这一行继续补充函数 `CmdVel` 的参数列表、限定符或返回类型说明。
        ForwardCmd * FMath::Cos(VelocityYawRad),
        // 解释：这一行继续补充函数 `CmdVel` 的参数列表、限定符或返回类型说明。
        ForwardCmd * FMath::Sin(VelocityYawRad),
        // 解释：这一行继续补充函数 `CmdVel` 的参数列表、限定符或返回类型说明。
        VzCmd);

    // 解释：调用 `SetCameraAngles` 执行当前步骤需要的功能逻辑。
    Interceptor->SetCameraAngles(CameraPitchTargetDeg, CameraYawTargetDeg);
    // 解释：调用 `SetHeadingControl` 执行当前步骤需要的功能逻辑。
    Interceptor->SetHeadingControl(EDroneYawMode::Angle, EDroneDrivetrainMode::ForwardOnly, YawCmdDeg);
    // 解释：调用 `SetTargetVelocity` 执行当前步骤需要的功能逻辑。
    Interceptor->SetTargetVelocity(CmdVel);

    // 解释：这一行把右侧表达式的结果写入 `Runtime.LastEx`，完成 lastex 的更新。
    Runtime.LastEx = Ex;
    // 解释：这一行把右侧表达式的结果写入 `Runtime.LastEy`，完成 lastey 的更新。
    Runtime.LastEy = Ey;
    // 解释：这一行把右侧表达式的结果写入 `Runtime.LastAreaNorm`，完成 lastareanorm 的更新。
    Runtime.LastAreaNorm = AreaNorm;
    // 解释：这一行把右侧表达式的结果写入 `Runtime.LastConf`，完成 lastconf 的更新。
    Runtime.LastConf = Confidence;
    // 解释：这一行把右侧表达式的结果写入 `Runtime.LastYawCmdDeg`，完成 lastyawcmddeg 的更新。
    Runtime.LastYawCmdDeg = YawCmdDeg;
    // 解释：这一行把右侧表达式的结果写入 `Runtime.LastYawRateDeg`，完成 lastyawratedeg 的更新。
    Runtime.LastYawRateDeg = YawRateCmd;
    // 解释：这一行把右侧表达式的结果写入 `Runtime.LastCmdVel`，完成 lastcmdvel 的更新。
    Runtime.LastCmdVel = CmdVel;
    // 解释：这一行把右侧表达式的结果写入 `Runtime.bCaptured`，完成 布尔标志 captured 的更新。
    Runtime.bCaptured = false;
    // 解释：这一行把右侧表达式的结果写入 `Runtime.State`，完成 状态 的更新。
    Runtime.State = (AreaNorm >= Params.CaptureArea) ? EVisualState::Approach : EVisualState::Track;

    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return true;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 在搜索模式下生成控制量
 * @param Interceptor 拦截无人机
 * @param Dt 时间步长（s）
 *
 * 搜索模式包含三部分动作：
 * 1. 云台左右摆扫；
 * 2. 机体缓慢转向；
 * 3. 小幅垂向正弦振荡，增加重新发现目标的机会。
 */
// 解释：这一行定义函数 `ComputeSearchControl`，开始实现computesearchcontrol的具体逻辑。
void UVisualInterceptController::ComputeSearchControl(ADronePawn* Interceptor, float Dt)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!Interceptor)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行把右侧表达式的结果写入 `Runtime.bCaptured`，完成 布尔标志 captured 的更新。
    Runtime.bCaptured = false;
    // 解释：这一行在 `Runtime.SearchTime` 的原有基础上继续累加新量，用于持续更新 searchtime。
    Runtime.SearchTime += Dt;

    // 解释：这一行在 `Runtime.SearchCamYawDeg` 的原有基础上继续累加新量，用于持续更新 searchcamyawdeg。
    Runtime.SearchCamYawDeg += Runtime.SearchDir * Params.SearchCamRateDeg * Dt;
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (FMath::Abs(Runtime.SearchCamYawDeg) >= Params.SearchCamYawLimitDeg)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行先对计算结果做限幅，再写入 `Runtime.SearchCamYawDeg`，防止 searchcamyawdeg 超出允许范围。
        Runtime.SearchCamYawDeg = FMath::Clamp(Runtime.SearchCamYawDeg, -Params.SearchCamYawLimitDeg, Params.SearchCamYawLimitDeg);
        // 解释：这一行把右侧表达式的结果写入 `Runtime.SearchDir`，完成 searchdir 的更新。
        Runtime.SearchDir *= -1.0f;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：调用 `SetCameraAngles` 执行当前步骤需要的功能逻辑。
    Interceptor->SetCameraAngles(Params.SearchCamPitchDeg, Runtime.SearchCamYawDeg);

    // 解释：这一行把右侧表达式的结果写入 `const float CurrentYawDeg`，完成 constfloatcurrentyawdeg 的更新。
    const float CurrentYawDeg = Interceptor->CurrentState.GetRotator().Yaw;
    // 解释：这一行把右侧表达式的结果写入 `const float YawCmdDeg`，完成 constfloatyawcmddeg 的更新。
    const float YawCmdDeg = NormalizeYawDeg(CurrentYawDeg + Runtime.SearchDir * Params.SearchBodyYawRateDeg * Dt);
    // 解释：这一行把右侧表达式的结果写入 `const float VzCmd`，完成 constfloatvzcmd 的更新。
    const float VzCmd = Params.SearchVzAmp * FMath::Sin(Runtime.SearchTime * 0.8f);
    // 解释：调用 `CmdVel` 执行当前步骤需要的功能逻辑。
    const FVector CmdVel(0.0f, 0.0f, VzCmd);

    // 解释：调用 `SetHeadingControl` 执行当前步骤需要的功能逻辑。
    Interceptor->SetHeadingControl(EDroneYawMode::Angle, EDroneDrivetrainMode::ForwardOnly, YawCmdDeg);
    // 解释：调用 `SetTargetVelocity` 执行当前步骤需要的功能逻辑。
    Interceptor->SetTargetVelocity(CmdVel);

    // 解释：这一行把右侧表达式的结果写入 `Runtime.State`，完成 状态 的更新。
    Runtime.State = EVisualState::Search;
    // 解释：这一行把右侧表达式的结果写入 `Runtime.LastEx`，完成 lastex 的更新。
    Runtime.LastEx = 0.0f;
    // 解释：这一行把右侧表达式的结果写入 `Runtime.LastEy`，完成 lastey 的更新。
    Runtime.LastEy = 0.0f;
    // 解释：这一行把右侧表达式的结果写入 `Runtime.LastAreaNorm`，完成 lastareanorm 的更新。
    Runtime.LastAreaNorm = 0.0f;
    // 解释：这一行把右侧表达式的结果写入 `Runtime.LastConf`，完成 lastconf 的更新。
    Runtime.LastConf = 0.0f;
    // 解释：这一行把右侧表达式的结果写入 `Runtime.LastYawCmdDeg`，完成 lastyawcmddeg 的更新。
    Runtime.LastYawCmdDeg = YawCmdDeg;
    // 解释：这一行把右侧表达式的结果写入 `Runtime.LastYawRateDeg`，完成 lastyawratedeg 的更新。
    Runtime.LastYawRateDeg = Runtime.SearchDir * Params.SearchBodyYawRateDeg;
    // 解释：这一行把右侧表达式的结果写入 `Runtime.LastCmdVel`，完成 lastcmdvel 的更新。
    Runtime.LastCmdVel = CmdVel;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 启动视觉拦截会话
 * @param CmdObj 启动命令 JSON
 * @param World 当前世界对象
 * @return 启动结果 JSON
 */
// 解释：这一行定义函数 `HandleStart`，开始实现handlestart的具体逻辑。
FString UVisualInterceptController::HandleStart(const TSharedPtr<FJsonObject>& CmdObj, UWorld* World)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：调用 `EnsureInitialized` 执行当前步骤需要的功能逻辑。
    EnsureInitialized();

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!World)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return MakeError(TEXT("No World"));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：调用 `ApplyParamsFromJson` 执行当前步骤需要的功能逻辑。
    ApplyParamsFromJson(CmdObj);

    // 解释：这一行声明成员或局部变量 `RequestedMethod`，用于保存requestedmethod。
    FString RequestedMethod = Runtime.Method;
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (CmdObj.IsValid() && CmdObj->HasField(TEXT("method")))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `RequestedMethod`，完成 requestedmethod 的更新。
        RequestedMethod = CmdObj->GetStringField(TEXT("method"));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行声明成员或局部变量 `CanonicalMethod`，用于保存canonicalmethod。
    FString CanonicalMethod;
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!NormalizeMethod(RequestedMethod, CanonicalMethod))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return MakeError(FString::Printf(TEXT("unsupported visual intercept method: %s"), *RequestedMethod));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行把右侧表达式的结果写入 `Runtime.Method`，完成 method 的更新。
    Runtime.Method = CanonicalMethod;

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (CmdObj.IsValid() && !CmdObj->HasField(TEXT("use_kalman")))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `Params.bUseKalman`，完成 布尔标志 use卡尔曼 的更新。
        Params.bUseKalman = Runtime.Method.Contains(TEXT("kalman"));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行把右侧表达式的结果写入 `ADronePawn* Interceptor`，完成 adronePawninterceptor 的更新。
    ADronePawn* Interceptor = ResolveInterceptor(CmdObj);
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!Interceptor)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return MakeError(TEXT("interceptor drone not found"));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：调用 `ResolveTarget` 执行当前步骤需要的功能逻辑。
    ResolveTarget(CmdObj);

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (FeaturePredictor)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `Reset` 执行当前步骤需要的功能逻辑。
        FeaturePredictor->Reset();
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：调用 `ResetPidControllers` 执行当前步骤需要的功能逻辑。
    ResetPidControllers();
    // 解释：调用 `BeginSession` 执行当前步骤需要的功能逻辑。
    BeginSession(Interceptor);

    // 解释：调用 `SetCameraAngles` 执行当前步骤需要的功能逻辑。
    Interceptor->SetCameraAngles(Params.SearchCamPitchDeg, 0.0f);
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return BuildStartJson();
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 处理一帧视觉拦截更新
 * @param CmdObj 更新命令 JSON
 * @param World 当前世界对象（当前实现未使用）
 * @return 本帧控制结果 JSON
 *
 * 运行逻辑分三支：
 * 1. 有检测：直接使用检测框，必要时先经过 Kalman 平滑；
 * 2. 无检测但预测器仍可用：用短时预测结果维持跟踪；
 * 3. 长时间丢失：回退到搜索模式。
 *
 * 最终捕获判定以真实三维距离为准：
 * $d = \|p_{target} - p_{interceptor}\| \le InterceptDistance$。
 */
// 解释：这一行定义函数 `HandleUpdate`，开始实现handleupdate的具体逻辑。
FString UVisualInterceptController::HandleUpdate(const TSharedPtr<FJsonObject>& CmdObj, UWorld* World)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    (void)World;
    // 解释：调用 `EnsureInitialized` 执行当前步骤需要的功能逻辑。
    EnsureInitialized();

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!Runtime.bEnabled)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return MakeError(TEXT("visual intercept is not started"));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!CmdObj.IsValid())
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return MakeError(TEXT("missing visual intercept update payload"));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行把右侧表达式的结果写入 `ADronePawn* Interceptor`，完成 adronePawninterceptor 的更新。
    ADronePawn* Interceptor = ResolveInterceptor(CmdObj);
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!Interceptor)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return MakeError(TEXT("interceptor drone not found"));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行把右侧表达式的结果写入 `ADronePawn* Target`，完成 adronePawntarget 的更新。
    ADronePawn* Target = ResolveTarget(CmdObj);

    // 解释：这一行声明成员或局部变量 `Dt`，用于保存dt。
    float Dt = kDefaultDt;
    // 解释：调用 `TryGetNumberAs` 执行当前步骤需要的功能逻辑。
    TryGetNumberAs(CmdObj, TEXT("dt"), Dt);
    // 解释：这一行把右侧表达式的结果写入 `Dt`，完成 dt 的更新。
    Dt = SafeDt(Dt);
    // 解释：调用 `SyncPidTimeStep` 执行当前步骤需要的功能逻辑。
    SyncPidTimeStep(Dt);

    // 解释：这一行声明成员或局部变量 `FrameW`，用于保存frameW。
    float FrameW = Runtime.LastFrameW;
    // 解释：这一行声明成员或局部变量 `FrameH`，用于保存frameH。
    float FrameH = Runtime.LastFrameH;
    // 解释：调用 `TryGetNumberAs` 执行当前步骤需要的功能逻辑。
    TryGetNumberAs(CmdObj, TEXT("image_w"), FrameW);
    // 解释：调用 `TryGetNumberAs` 执行当前步骤需要的功能逻辑。
    TryGetNumberAs(CmdObj, TEXT("image_h"), FrameH);
    // 解释：这一行通过 `FMath::Max` 给 `Runtime.LastFrameW` 施加下界约束，避免 lastframeW 过小。
    Runtime.LastFrameW = FMath::Max(1.0f, FrameW);
    // 解释：这一行通过 `FMath::Max` 给 `Runtime.LastFrameH` 施加下界约束，避免 lastframeH 过小。
    Runtime.LastFrameH = FMath::Max(1.0f, FrameH);

    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    Runtime.FrameCount++;

    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    const bool bHasDetection = CmdObj->HasField(TEXT("has_detection"))
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        ? CmdObj->GetBoolField(TEXT("has_detection"))
        // 解释：这一行开始构造函数初始化列表，下面会依次初始化成员变量。
        : (CmdObj->HasField(TEXT("cx")) && CmdObj->HasField(TEXT("cy")));

    // 解释：这一行声明成员或局部变量 `bValidControl`，用于保存布尔标志 validcontrol。
    bool bValidControl = true;
    // 解释：这一行声明成员或局部变量 `bCapturedThisFrame`，用于保存布尔标志 capturedthisframe。
    bool bCapturedThisFrame = false;

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (bHasDetection)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行声明成员或局部变量 `Cx`，用于保存cx。
        float Cx = 0.0f;
        // 解释：这一行声明成员或局部变量 `Cy`，用于保存cy。
        float Cy = 0.0f;
        // 解释：这一行声明成员或局部变量 `Area`，用于保存area。
        float Area = 0.0f;
        // 解释：这一行声明成员或局部变量 `AreaRatio`，用于保存arearatio。
        float AreaRatio = -1.0f;
        // 解释：这一行声明成员或局部变量 `Confidence`，用于保存confidence。
        float Confidence = 1.0f;

        // 解释：调用 `TryGetNumberAs` 执行当前步骤需要的功能逻辑。
        TryGetNumberAs(CmdObj, TEXT("cx"), Cx);
        // 解释：调用 `TryGetNumberAs` 执行当前步骤需要的功能逻辑。
        TryGetNumberAs(CmdObj, TEXT("cy"), Cy);
        // 解释：调用 `TryGetNumberAs` 执行当前步骤需要的功能逻辑。
        TryGetNumberAs(CmdObj, TEXT("area"), Area);
        // 解释：调用 `TryGetNumberAs` 执行当前步骤需要的功能逻辑。
        TryGetNumberAs(CmdObj, TEXT("area_ratio"), AreaRatio);
        // 解释：调用 `TryGetNumberAs` 执行当前步骤需要的功能逻辑。
        TryGetNumberAs(CmdObj, TEXT("conf"), Confidence);

        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (Area <= 0.0f && AreaRatio > 0.0f)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行把右侧表达式的结果写入 `Area`，完成 area 的更新。
            Area = AreaRatio * Runtime.LastFrameW * Runtime.LastFrameH;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
        // 解释：这一行通过 `FMath::Max` 给 `Area` 施加下界约束，避免 area 过小。
        Area = FMath::Max(1.0f, Area);

        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Runtime.DetectionCount++;
        // 解释：这一行把右侧表达式的结果写入 `Runtime.LostCount`，完成 lostcount 的更新。
        Runtime.LostCount = 0;

        // 解释：调用 `Feature` 执行当前步骤需要的功能逻辑。
        FVector Feature(Cx, Cy, Area);
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (Params.bUseKalman && FeaturePredictor)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：调用 `Update` 执行当前步骤需要的功能逻辑。
            FeaturePredictor->Update(Feature, Dt);
            // 解释：这一行把右侧表达式的结果写入 `Feature`，完成 feature 的更新。
            Feature = FeaturePredictor->GetEstimatedPosition();
            // 解释：这一行通过 `FMath::Max` 给 `Feature.Z` 施加下界约束，避免 Z 过小。
            Feature.Z = FMath::Max(1.0f, Feature.Z);
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        bValidControl = ComputeTrackControl(
            // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
            Interceptor,
            // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
            Feature,
            // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
            Runtime.LastFrameW,
            // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
            Runtime.LastFrameH,
            // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
            Confidence,
            // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
            Dt,
            // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
            true,
            // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
            bCapturedThisFrame);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行给出兜底分支，当上面的条件都不满足时执行。
    else
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Runtime.LostCount++;

        // 解释：这一行声明成员或局部变量 `bUsedPrediction`，用于保存布尔标志 usedprediction。
        bool bUsedPrediction = false;
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (Params.bUseKalman && FeaturePredictor && FeaturePredictor->IsInitialized() && Runtime.LostCount < Params.LostToSearchFrames)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行把右侧表达式的结果写入 `FVector Pred`，完成 fvectorpred 的更新。
            FVector Pred = FeaturePredictor->PredictPosition(Dt);
            // 解释：这一行通过 `FMath::Max` 给 `Pred.Z` 施加下界约束，避免 Z 过小。
            Pred.Z = FMath::Max(1.0f, Pred.Z);
            // 解释：这一行把右侧表达式的结果写入 `bUsedPrediction`，完成 布尔标志 usedprediction 的更新。
            bUsedPrediction = true;
            // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
            bValidControl = ComputeTrackControl(
                // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                Interceptor,
                // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                Pred,
                // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                Runtime.LastFrameW,
                // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                Runtime.LastFrameH,
                // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                0.0f,
                // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                Dt,
                // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                false,
                // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                bCapturedThisFrame);
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (!bUsedPrediction)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：调用 `ComputeSearchControl` 执行当前步骤需要的功能逻辑。
            ComputeSearchControl(Interceptor, Dt);
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行把右侧表达式的结果写入 `Runtime.LastTargetDistance`，完成 lasttargetdistance 的更新。
    Runtime.LastTargetDistance = -1.0f;
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (Target)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `Runtime.LastTargetDistance`，完成 lasttargetdistance 的更新。
        Runtime.LastTargetDistance = FVector::Dist(Interceptor->GetCurrentPosition(), Target->GetCurrentPosition());
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (Runtime.LastTargetDistance <= Params.InterceptDistance)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
            Runtime.CaptureCount++;
            // 解释：这一行把右侧表达式的结果写入 `Runtime.State`，完成 状态 的更新。
            Runtime.State = EVisualState::Captured;
            // 解释：这一行把右侧表达式的结果写入 `Runtime.bCaptured`，完成 布尔标志 captured 的更新。
            Runtime.bCaptured = true;
            // 解释：这一行把右侧表达式的结果写入 `bCapturedThisFrame`，完成 布尔标志 capturedthisframe 的更新。
            bCapturedThisFrame = true;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
        // 解释：这一行给出兜底分支，当上面的条件都不满足时执行。
        else
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行把右侧表达式的结果写入 `Runtime.CaptureCount`，完成 采集count 的更新。
            Runtime.CaptureCount = 0;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行给出兜底分支，当上面的条件都不满足时执行。
    else
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `Runtime.CaptureCount`，完成 采集count 的更新。
        Runtime.CaptureCount = 0;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (bCapturedThisFrame && Params.bStopOnCapture)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `Hover` 执行当前步骤需要的功能逻辑。
        Interceptor->Hover();
        // 解释：这一行把右侧表达式的结果写入 `Runtime.bEnabled`，完成 布尔标志 enabled 的更新。
        Runtime.bEnabled = false;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return BuildUpdateJson(bValidControl);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 停止视觉拦截，并令拦截机悬停
 * @param CmdObj 停止命令 JSON
 * @param World 当前世界对象（当前实现未使用）
 * @return 停止结果 JSON
 */
// 解释：这一行定义函数 `HandleStop`，开始实现handlestop的具体逻辑。
FString UVisualInterceptController::HandleStop(const TSharedPtr<FJsonObject>& CmdObj, UWorld* World)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    (void)World;
    // 解释：调用 `EnsureInitialized` 执行当前步骤需要的功能逻辑。
    EnsureInitialized();

    // 解释：这一行把右侧表达式的结果写入 `ADronePawn* Interceptor`，完成 adronePawninterceptor 的更新。
    ADronePawn* Interceptor = ResolveInterceptor(CmdObj);
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (Interceptor)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `Hover` 执行当前步骤需要的功能逻辑。
        Interceptor->Hover();
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行把右侧表达式的结果写入 `Runtime.bEnabled`，完成 布尔标志 enabled 的更新。
    Runtime.bEnabled = false;
    // 解释：这一行把右侧表达式的结果写入 `Runtime.State`，完成 状态 的更新。
    Runtime.State = Runtime.bCaptured ? EVisualState::Captured : EVisualState::Idle;
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return MakeOk(TEXT("visual intercept stopped"));
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 查询当前视觉拦截运行状态 */
// 解释：这一行定义函数 `HandleState`，开始实现handle状态的具体逻辑。
FString UVisualInterceptController::HandleState() const
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return FString::Printf(
        // 解释：这一行位于构造函数初始化列表中，把 `TEXT` 直接初始化为 `"{\"status\":\"ok\",\"mode\":\"visual_intercept\",\"enabled\":%s,\"state\":\"%s\",\"captured\":%s,\"interceptor_id\":\"%s\",\"target_id\":\"%s\",\"method\":\"%s\",\"frame\":%d,\"detections\":%d,\"lost_count\":%d,\"capture_count\":%d,\"target_distance\":%.4f,\"cmd_velocity\":[%.4f,%.4f,%.4f],\"control\":{\"ex\":%.5f,\"ey\":%.5f,\"area_norm\":%.6f,\"conf\":%.4f,\"yaw_cmd_deg\":%.4f,\"yaw_rate_deg\":%.4f}}"`，减少进入函数体后的额外赋值开销。
        TEXT("{\"status\":\"ok\",\"mode\":\"visual_intercept\",\"enabled\":%s,\"state\":\"%s\",\"captured\":%s,\"interceptor_id\":\"%s\",\"target_id\":\"%s\",\"method\":\"%s\",\"frame\":%d,\"detections\":%d,\"lost_count\":%d,\"capture_count\":%d,\"target_distance\":%.4f,\"cmd_velocity\":[%.4f,%.4f,%.4f],\"control\":{\"ex\":%.5f,\"ey\":%.5f,\"area_norm\":%.6f,\"conf\":%.4f,\"yaw_cmd_deg\":%.4f,\"yaw_rate_deg\":%.4f}}"),
        *BoolLiteral(Runtime.bEnabled),
        *StateToString(Runtime.State),
        *BoolLiteral(Runtime.bCaptured),
        *Runtime.InterceptorId,
        *Runtime.TargetId,
        *Runtime.Method,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Runtime.FrameCount,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Runtime.DetectionCount,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Runtime.LostCount,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Runtime.CaptureCount,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Runtime.LastTargetDistance,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Runtime.LastCmdVel.X,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Runtime.LastCmdVel.Y,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Runtime.LastCmdVel.Z,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Runtime.LastEx,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Runtime.LastEy,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Runtime.LastAreaNorm,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Runtime.LastConf,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Runtime.LastYawCmdDeg,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Runtime.LastYawRateDeg);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

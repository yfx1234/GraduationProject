// 解释：引入当前实现文件对应的头文件 `SimClockService.h`，使实现部分能够看到类和函数声明。
#include "SimClockService.h"

// 解释：引入 `World.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "Engine/World.h"
// 解释：引入 `WorldSettings.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "GameFramework/WorldSettings.h"

// 解释：这一行把右侧表达式的结果写入 `USimClockService* USimClockService::Instance`，完成 usim时钟serviceusim时钟serviceinstance 的更新。
USimClockService* USimClockService::Instance = nullptr;

/**
 * @brief 获取仿真时钟服务单例
 * 若不存在则立即创建，并加入 Root 防止 GC 回收。
 */
// 解释：这一行定义函数 `GetInstance`，开始实现getinstance的具体逻辑。
USimClockService* USimClockService::GetInstance()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!Instance)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `Instance`，完成 instance 的更新。
        Instance = NewObject<USimClockService>();
        // 解释：调用 `AddToRoot` 执行当前步骤需要的功能逻辑。
        Instance->AddToRoot();
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return Instance;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 释放仿真时钟服务单例 */
// 解释：这一行定义函数 `Cleanup`，开始实现cleanup的具体逻辑。
void USimClockService::Cleanup()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (Instance)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `RemoveFromRoot` 执行当前步骤需要的功能逻辑。
        Instance->RemoveFromRoot();
        // 解释：这一行把右侧表达式的结果写入 `Instance`，完成 instance 的更新。
        Instance = nullptr;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 初始化时钟参考点
 * @param World 当前场景 World
 * 记录墙钟与 World 时间的起点，后续所有时间查询都以此为零点。
 */
// 解释：这一行定义函数 `Initialize`，开始实现initialize的具体逻辑。
void USimClockService::Initialize(UWorld* World)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `StartWallTimeSec`，完成 startwalltimesec 的更新。
    StartWallTimeSec = FPlatformTime::Seconds();
    // 解释：这一行把右侧表达式的结果写入 `StartWorldTimeSec`，完成 startworldtimesec 的更新。
    StartWorldTimeSec = World ? World->GetTimeSeconds() : 0.0;
    // 解释：这一行把右侧表达式的结果写入 `bInitialized`，完成 布尔标志 initialized 的更新。
    bInitialized = true;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 设置仿真时间倍率
 * @param InTimeScale 目标倍率
 * @param World 当前场景 World
 * 时间倍率会同步到 WorldSettings 的 Time Dilation。
 */
// 解释：这一行定义函数 `SetTimeScale`，开始实现settimescale的具体逻辑。
void USimClockService::SetTimeScale(float InTimeScale, UWorld* World)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 夹紧倍率范围，避免时间冻结或过大倍率导致仿真数值不稳定。
    // 解释：这一行先对计算结果做限幅，再写入 `TimeScale`，防止 timescale 超出允许范围。
    TimeScale = FMath::Clamp(InTimeScale, 0.05f, 20.0f);
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (World && World->GetWorldSettings())
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `GetWorldSettings` 执行当前步骤需要的功能逻辑。
        World->GetWorldSettings()->SetTimeDilation(TimeScale);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 获取累计仿真时间
 * @param World 当前场景 World
 * @return 从 Initialize 起累计的仿真时间（秒）
 */
// 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
double USimClockService::GetSimTimeSec(UWorld* World) const
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!World)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return 0.0;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return World->GetTimeSeconds() - StartWorldTimeSec;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 获取累计墙钟时间
 * @return 从 Initialize 起累计的真实时间（秒）
 */
// 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
double USimClockService::GetWallTimeSec() const
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return FPlatformTime::Seconds() - StartWallTimeSec;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

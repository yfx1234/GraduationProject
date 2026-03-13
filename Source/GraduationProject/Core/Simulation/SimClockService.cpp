#include "SimClockService.h"

#include "Engine/World.h"
#include "GameFramework/WorldSettings.h"

USimClockService* USimClockService::Instance = nullptr;

/**
 * @brief 获取仿真时钟服务单例
 * 若不存在则立即创建，并加入 Root 防止 GC 回收。
 */
USimClockService* USimClockService::GetInstance()
{
    if (!Instance)
    {
        Instance = NewObject<USimClockService>();
        Instance->AddToRoot();
    }
    return Instance;
}

/** @brief 释放仿真时钟服务单例 */
void USimClockService::Cleanup()
{
    if (Instance)
    {
        Instance->RemoveFromRoot();
        Instance = nullptr;
    }
}

/**
 * @brief 初始化时钟参考点
 * @param World 当前场景 World
 * 记录墙钟与 World 时间的起点，后续所有时间查询都以此为零点。
 */
void USimClockService::Initialize(UWorld* World)
{
    StartWallTimeSec = FPlatformTime::Seconds();
    StartWorldTimeSec = World ? World->GetTimeSeconds() : 0.0;
    bInitialized = true;
}

/**
 * @brief 设置仿真时间倍率
 * @param InTimeScale 目标倍率
 * @param World 当前场景 World
 * 时间倍率会同步到 WorldSettings 的 Time Dilation。
 */
void USimClockService::SetTimeScale(float InTimeScale, UWorld* World)
{
    // 夹紧倍率范围，避免时间冻结或过大倍率导致仿真数值不稳定。
    TimeScale = FMath::Clamp(InTimeScale, 0.05f, 20.0f);
    if (World && World->GetWorldSettings())
    {
        World->GetWorldSettings()->SetTimeDilation(TimeScale);
    }
}

/**
 * @brief 获取累计仿真时间
 * @param World 当前场景 World
 * @return 从 Initialize 起累计的仿真时间（秒）
 */
double USimClockService::GetSimTimeSec(UWorld* World) const
{
    if (!World)
    {
        return 0.0;
    }

    return World->GetTimeSeconds() - StartWorldTimeSec;
}

/**
 * @brief 获取累计墙钟时间
 * @return 从 Initialize 起累计的真实时间（秒）
 */
double USimClockService::GetWallTimeSec() const
{
    return FPlatformTime::Seconds() - StartWallTimeSec;
}
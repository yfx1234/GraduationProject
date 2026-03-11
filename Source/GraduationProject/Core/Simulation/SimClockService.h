#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "SimClockService.generated.h"

/**
 * @brief 仿真时钟服务
 * 维护墙钟时间与 Unreal 世界时间的起点，并提供时间倍率设置，
 * 供 `sim_get_time`、`sim_set_time_scale` 等接口统一查询。
 */
UCLASS()
class GRADUATIONPROJECT_API USimClockService : public UObject
{
    GENERATED_BODY()

public:
    /** @brief 获取全局单例 */
    static USimClockService* GetInstance();

    /** @brief 释放全局单例 */
    static void Cleanup();

    /**
     * @brief 记录仿真起始时间参考
     * @param World 当前场景 World
     */
    void Initialize(UWorld* World);

    /** @brief 时钟服务是否已经初始化 */
    bool IsInitialized() const { return bInitialized; }

    /**
     * @brief 设置仿真时间倍率
     * @param InTimeScale 目标时间倍率
     * @param World 当前场景 World，用于同步 Unreal Time Dilation
     */
    void SetTimeScale(float InTimeScale, UWorld* World);

    /** @brief 获取当前时间倍率 */
    float GetTimeScale() const { return TimeScale; }

    /**
     * @brief 获取从初始化起累计的仿真时间
     * @param World 当前场景 World
     * @return 仿真时间（秒）
     */
    double GetSimTimeSec(UWorld* World) const;

    /**
     * @brief 获取从初始化起累计的墙钟时间
     * @return 墙钟时间（秒）
     */
    double GetWallTimeSec() const;

private:
    /** @brief 全局单例指针 */
    static USimClockService* Instance;

    /** @brief 当前仿真时间倍率 */
    float TimeScale = 1.0f;

    /** @brief 初始化时的墙钟时间 */
    double StartWallTimeSec = 0.0;

    /** @brief 初始化时的 World 时间 */
    double StartWorldTimeSec = 0.0;

    /** @brief 是否已记录起始时间 */
    bool bInitialized = false;
};
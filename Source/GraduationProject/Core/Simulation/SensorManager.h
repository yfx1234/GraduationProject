#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "SensorManager.generated.h"

/**
 * @brief 传感器运动学缓存
 * 保存上一帧速度和时间戳，用于通过差分近似线加速度。
 */
USTRUCT()
struct FSensorKinematicCache
{
    GENERATED_BODY()

    /** @brief 上一次记录的速度向量 */
    FVector LastVelocity = FVector::ZeroVector;

    /** @brief 上一次采样时间（秒） */
    double LastTimeSec = 0.0;

    /** @brief 缓存是否已经初始化 */
    bool bValid = false;
};

/**
 * @brief 传感器数据构建服务
 * 从场景中的无人机状态生成 IMU、GPS、气压计和运动学 JSON，
 * 供网络接口统一返回给 Python 客户端或外部仿真适配层。
 */
UCLASS()
class GRADUATIONPROJECT_API USensorManager : public UObject
{
    GENERATED_BODY()

public:
    /** @brief 获取全局单例 */
    static USensorManager* GetInstance();

    /** @brief 释放全局单例并清理缓存 */
    static void Cleanup();

    /**
     * @brief 生成指定无人机的传感器 JSON
     * @param DroneId 无人机 ID
     * @param World 当前场景 World
     * @param Frame 输出坐标系，支持 `ue` 与 `ned`
     * @return 传感器 JSON 字符串；失败时返回错误 JSON
     */
    FString BuildDroneSensorJson(const FString& DroneId, UWorld* World, const FString& Frame = TEXT("ue"));

private:
    /** @brief 全局单例指针 */
    static USensorManager* Instance;

    /** @brief 各无人机的速度缓存，用于估算加速度 */
    UPROPERTY()
    TMap<FString, FSensorKinematicCache> DroneCache;
};
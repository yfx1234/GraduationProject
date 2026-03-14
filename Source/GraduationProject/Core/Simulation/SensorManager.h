// 解释：使用 `#pragma once` 防止该头文件在编译过程中被重复包含。
#pragma once

// 解释：引入 Unreal 的核心基础头文件，提供常用容器、数学类型和日志宏。
#include "CoreMinimal.h"
// 解释：引入 `NoExportTypes.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "UObject/NoExportTypes.h"
// 解释：引入 `SensorManager.generated.h`，让 Unreal Header Tool 生成的反射代码在本文件中可见。
#include "SensorManager.generated.h"

/**
 * @brief 传感器运动学缓存
 * 保存上一帧速度和时间戳，用于通过差分近似线加速度。
 */
// 解释：使用 `USTRUCT` 宏声明可被 Unreal 反射系统识别的结构体类型。
USTRUCT()
// 解释：这一行声明 结构体 `FSensorKinematicCache`，用于封装fsensorkinematiccache相关的数据与行为。
struct FSensorKinematicCache
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：展开 Unreal Header Tool 生成的样板代码，这是 UObject、Actor、组件等类型正常工作的基础。
    GENERATED_BODY()

    /** @brief 上一次记录的速度向量 */
    // 解释：这一行声明成员或局部变量 `LastVelocity`，用于保存lastvelocity。
    FVector LastVelocity = FVector::ZeroVector;

    /** @brief 上一次采样时间（秒） */
    // 解释：这一行声明成员或局部变量 `LastTimeSec`，用于保存lasttimesec。
    double LastTimeSec = 0.0;

    /** @brief 缓存是否已经初始化 */
    // 解释：这一行声明成员或局部变量 `bValid`，用于保存布尔标志 valid。
    bool bValid = false;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
};

/**
 * @brief 传感器数据构建服务
 * 从场景中的无人机状态生成 IMU、GPS、气压计和运动学 JSON，
 * 供网络接口统一返回给 Python 客户端或外部仿真适配层。
 */
// 解释：使用 `UCLASS` 宏把当前类型注册到 Unreal 反射系统中，便于蓝图、序列化和编辑器识别。
UCLASS()
// 解释：这一行声明 类 `USensorManager`，用于封装usensor管理器相关的数据与行为。
class GRADUATIONPROJECT_API USensorManager : public UObject
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：展开 Unreal Header Tool 生成的样板代码，这是 UObject、Actor、组件等类型正常工作的基础。
    GENERATED_BODY()

// 解释：从这一行开始进入 `public` 访问区，下面的成员会对外部模块公开。
public:
    /** @brief 获取全局单例 */
    // 解释：调用 `GetInstance` 执行当前步骤需要的功能逻辑。
    static USensorManager* GetInstance();

    /** @brief 释放全局单例并清理缓存 */
    // 解释：调用 `Cleanup` 执行当前步骤需要的功能逻辑。
    static void Cleanup();

    /**
     * @brief 生成指定无人机的传感器 JSON
     * @param DroneId 无人机 ID
     * @param World 当前场景 World
     * @param Frame 输出坐标系，支持 `ue` 与 `ned`
     * @return 传感器 JSON 字符串；失败时返回错误 JSON
     */
    // 解释：这一行把右侧表达式的结果写入 `FString BuildDroneSensorJson(const FString& DroneId, UWorld* World, const FString& Frame`，完成 fstringbuild无人机传感器jsonconstfstring无人机iduworldworldconstfstringframe 的更新。
    FString BuildDroneSensorJson(const FString& DroneId, UWorld* World, const FString& Frame = TEXT("ue"));

// 解释：从这一行开始进入 `private` 访问区，下面的成员只允许类内部访问。
private:
    /** @brief 全局单例指针 */
    // 解释：这一行声明成员或局部变量 `Instance`，用于保存instance。
    static USensorManager* Instance;

    /** @brief 各无人机的速度缓存，用于估算加速度 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY()
    // 解释：这一行声明成员或局部变量 `DroneCache`，用于保存无人机cache。
    TMap<FString, FSensorKinematicCache> DroneCache;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
};

// 解释：使用 `#pragma once` 防止该头文件在编译过程中被重复包含。
#pragma once

// 解释：引入 Unreal 的核心基础头文件，提供常用容器、数学类型和日志宏。
#include "CoreMinimal.h"
// 解释：引入 `NoExportTypes.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "UObject/NoExportTypes.h"
// 解释：引入 `SimClockService.generated.h`，让 Unreal Header Tool 生成的反射代码在本文件中可见。
#include "SimClockService.generated.h"

/**
 * @brief 仿真时钟服务
 * 维护墙钟时间与 Unreal 世界时间的起点，并提供时间倍率设置，
 * 供 `sim_get_time`、`sim_set_time_scale` 等接口统一查询。
 */
// 解释：使用 `UCLASS` 宏把当前类型注册到 Unreal 反射系统中，便于蓝图、序列化和编辑器识别。
UCLASS()
// 解释：这一行声明 类 `USimClockService`，用于封装usim时钟service相关的数据与行为。
class GRADUATIONPROJECT_API USimClockService : public UObject
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：展开 Unreal Header Tool 生成的样板代码，这是 UObject、Actor、组件等类型正常工作的基础。
    GENERATED_BODY()

// 解释：从这一行开始进入 `public` 访问区，下面的成员会对外部模块公开。
public:
    /** @brief 获取全局单例 */
    // 解释：调用 `GetInstance` 执行当前步骤需要的功能逻辑。
    static USimClockService* GetInstance();

    /** @brief 释放全局单例 */
    // 解释：调用 `Cleanup` 执行当前步骤需要的功能逻辑。
    static void Cleanup();

    /**
     * @brief 记录仿真起始时间参考
     * @param World 当前场景 World
     */
    // 解释：调用 `Initialize` 执行当前步骤需要的功能逻辑。
    void Initialize(UWorld* World);

    /** @brief 时钟服务是否已经初始化 */
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    bool IsInitialized() const { return bInitialized; }

    /**
     * @brief 设置仿真时间倍率
     * @param InTimeScale 目标时间倍率
     * @param World 当前场景 World，用于同步 Unreal Time Dilation
     */
    // 解释：调用 `SetTimeScale` 执行当前步骤需要的功能逻辑。
    void SetTimeScale(float InTimeScale, UWorld* World);

    /** @brief 获取当前时间倍率 */
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    float GetTimeScale() const { return TimeScale; }

    /**
     * @brief 获取从初始化起累计的仿真时间
     * @param World 当前场景 World
     * @return 仿真时间（秒）
     */
    // 解释：调用 `GetSimTimeSec` 执行当前步骤需要的功能逻辑。
    double GetSimTimeSec(UWorld* World) const;

    /**
     * @brief 获取从初始化起累计的墙钟时间
     * @return 墙钟时间（秒）
     */
    // 解释：调用 `GetWallTimeSec` 执行当前步骤需要的功能逻辑。
    double GetWallTimeSec() const;

// 解释：从这一行开始进入 `private` 访问区，下面的成员只允许类内部访问。
private:
    /** @brief 全局单例指针 */
    // 解释：这一行声明成员或局部变量 `Instance`，用于保存instance。
    static USimClockService* Instance;

    /** @brief 当前仿真时间倍率 */
    // 解释：这一行声明成员或局部变量 `TimeScale`，用于保存timescale。
    float TimeScale = 1.0f;

    /** @brief 初始化时的墙钟时间 */
    // 解释：这一行声明成员或局部变量 `StartWallTimeSec`，用于保存startwalltimesec。
    double StartWallTimeSec = 0.0;

    /** @brief 初始化时的 World 时间 */
    // 解释：这一行声明成员或局部变量 `StartWorldTimeSec`，用于保存startworldtimesec。
    double StartWorldTimeSec = 0.0;

    /** @brief 是否已记录起始时间 */
    // 解释：这一行声明成员或局部变量 `bInitialized`，用于保存布尔标志 initialized。
    bool bInitialized = false;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
};

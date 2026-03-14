// 解释：使用 `#pragma once` 防止该头文件在编译过程中被重复包含。
#pragma once

// 解释：引入 Unreal 的核心基础头文件，提供常用容器、数学类型和日志宏。
#include "CoreMinimal.h"
// 解释：引入 `GameModeBase.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "GameFramework/GameModeBase.h"
// 解释：引入 `DronePawn.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "../Drone/DronePawn.h"
// 解释：引入 `SimGameMode.generated.h`，让 Unreal Header Tool 生成的反射代码在本文件中可见。
#include "SimGameMode.generated.h"

/** @brief 单架无人机生成配置，定义 ID、位置、角色与起飞参数 */
// 解释：使用 `USTRUCT` 宏声明可被 Unreal 反射系统识别的结构体类型。
USTRUCT(BlueprintType)
// 解释：这一行声明 结构体 `FDroneSpawnConfig`，用于封装fdronespawnconfig相关的数据与行为。
struct FDroneSpawnConfig
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：展开 Unreal Header Tool 生成的样板代码，这是 UObject、Actor、组件等类型正常工作的基础。
    GENERATED_BODY()

    /** @brief 无人机唯一 ID */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Drone")
    // 解释：这一行把右侧表达式的结果写入 `FString DroneId`，完成 fstring无人机id 的更新。
    FString DroneId = TEXT("drone_0");

    /** @brief 生成位置（仿真坐标，单位：米） */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Drone")
    // 解释：这一行把右侧表达式的结果写入 `FVector SpawnLocation`，完成 fvectorspawnlocation 的更新。
    FVector SpawnLocation = FVector(0.0f, 0.0f, 0.5f);

    /** @brief 生成后是否自动起飞 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Drone")
    // 解释：这一行声明成员或局部变量 `bAutoTakeoff`，用于保存布尔标志 autotakeoff。
    bool bAutoTakeoff = false;

    /** @brief 自动起飞目标高度（米） */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Drone")
    // 解释：这一行声明成员或局部变量 `TakeoffAltitude`，用于保存takeoffaltitude。
    float TakeoffAltitude = 3.0f;

    /** @brief 任务角色（目标机 / 拦截机 / 未知） */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Drone")
    // 解释：这一行声明成员或局部变量 `MissionRole`，用于保存missionrole。
    EDroneMissionRole MissionRole = EDroneMissionRole::Unknown;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
};

/**
 * @brief 仿真 GameMode
 * 智能体管理 — 负责生成 / 销毁无人机和转台
 * 仿真控制 — 暂停 / 恢复 / 重置仿真
 * 默认 Pawn 和 HUD 配置
 */
// 解释：使用 `UCLASS` 宏把当前类型注册到 Unreal 反射系统中，便于蓝图、序列化和编辑器识别。
UCLASS()
// 解释：这一行声明 类 `ASimGameMode`，用于封装asimgame模式相关的数据与行为。
class GRADUATIONPROJECT_API ASimGameMode : public AGameModeBase
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：展开 Unreal Header Tool 生成的样板代码，这是 UObject、Actor、组件等类型正常工作的基础。
    GENERATED_BODY()

// 解释：从这一行开始进入 `public` 访问区，下面的成员会对外部模块公开。
public:
    // 解释：调用 `ASimGameMode` 执行当前步骤需要的功能逻辑。
    ASimGameMode();
    // 解释：调用 `BeginPlay` 执行当前步骤需要的功能逻辑。
    virtual void BeginPlay() override;

    /** @brief 暂停仿真 */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "Simulation")
    // 解释：调用 `PauseSimulation` 执行当前步骤需要的功能逻辑。
    void PauseSimulation();

    /** @brief 恢复仿真 */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "Simulation")
    // 解释：调用 `ResumeSimulation` 执行当前步骤需要的功能逻辑。
    void ResumeSimulation();

    /** @brief 重置仿真 */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "Simulation")
    // 解释：调用 `ResetSimulation` 执行当前步骤需要的功能逻辑。
    void ResetSimulation();

// 解释：从这一行开始进入 `protected` 访问区，下面的成员只对本类及其派生类可见。
protected:
    /** @brief 生成默认无人机 */
    // 解释：调用 `SpawnDefaultDrone` 执行当前步骤需要的功能逻辑。
    void SpawnDefaultDrone();

    /** @brief 按配置生成多无人机 */
    // 解释：调用 `SpawnConfiguredDrones` 执行当前步骤需要的功能逻辑。
    void SpawnConfiguredDrones();

    /** @brief 生成一架无人机（位置单位：米） */
    // 解释：这一行把右侧表达式的结果写入 `ADronePawn* SpawnDrone(const FString& DroneId, const FVector& SpawnLocationMeters, EDroneMissionRole InMissionRole`，完成 adronePawnspawn无人机constfstring无人机idconstfvectorspawnlocationmetersedronemissionroleinmissionrole 的更新。
    ADronePawn* SpawnDrone(const FString& DroneId, const FVector& SpawnLocationMeters, EDroneMissionRole InMissionRole = EDroneMissionRole::Unknown);

    /** @brief 捕获当前 Agent 初始状态快照，用于 reset */
    // 解释：调用 `CaptureInitialAgentStates` 执行当前步骤需要的功能逻辑。
    void CaptureInitialAgentStates();

    /** @brief 延迟起飞回调 */
    // 解释：调用 `DelayedTakeoff` 执行当前步骤需要的功能逻辑。
    void DelayedTakeoff();

// 解释：从这一行开始进入 `public` 访问区，下面的成员会对外部模块公开。
public:
    /** @brief 是否在游戏开始时自动生成无人机 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Drone")
    // 解释：这一行声明成员或局部变量 `bAutoSpawnDrone`，用于保存布尔标志 autospawn无人机。
    bool bAutoSpawnDrone = false;

    /** @brief 启动阶段是否允许批量自动生成（双开关保护，默认关闭） */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Drone")
    // 解释：这一行声明成员或局部变量 `bEnableBootstrapSpawn`，用于保存布尔标志 enablebootstrapspawn。
    bool bEnableBootstrapSpawn = false;

    /** @brief 无人机蓝图类 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Drone")
    // 解释：这一行声明成员或局部变量 `DefaultDroneClass`，用于保存default无人机class。
    TSubclassOf<APawn> DefaultDroneClass;

    /** @brief 无人机生成位置 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Drone")
    // 解释：这一行把右侧表达式的结果写入 `FVector DroneSpawnLocation`，完成 fvector无人机spawnlocation 的更新。
    FVector DroneSpawnLocation = FVector(0.0f, 0.0f, 0.5f);

    /** @brief 无人机生成时分配的 Agent ID */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Drone")
    // 解释：这一行把右侧表达式的结果写入 `FString DefaultDroneId`，完成 fstringdefault无人机id 的更新。
    FString DefaultDroneId = TEXT("drone_0");

    /** @brief 是否在生成后自动起飞 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Drone")
    // 解释：这一行声明成员或局部变量 `bAutoTakeoff`，用于保存布尔标志 autotakeoff。
    bool bAutoTakeoff = true;

    /** @brief 自动起飞目标高度（米） */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Drone")
    // 解释：这一行声明成员或局部变量 `TakeoffAltitude`，用于保存takeoffaltitude。
    float TakeoffAltitude = 3.0f;

    /** @brief 默认单机角色（目标机/拦截机） */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Drone")
    // 解释：这一行声明成员或局部变量 `DefaultDroneRole`，用于保存default无人机role。
    EDroneMissionRole DefaultDroneRole = EDroneMissionRole::Target;

    /** @brief 是否启用多无人机配置生成 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Drone")
    // 解释：这一行声明成员或局部变量 `bUseDroneSpawnList`，用于保存布尔标志 use无人机spawn列表。
    bool bUseDroneSpawnList = false;

    /** @brief 多无人机生成配置列表（启用后优先于单机配置） */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Drone")
    // 解释：这一行声明成员或局部变量 `DroneSpawnList`，用于保存无人机spawn列表。
    TArray<FDroneSpawnConfig> DroneSpawnList;

// 解释：从这一行开始进入 `private` 访问区，下面的成员只允许类内部访问。
private:
    /** @brief 已生成的无人机 Pawn 引用 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY()
    // 解释：这一行声明成员或局部变量 `SpawnedDrone`，用于保存spawned无人机。
    APawn* SpawnedDrone = nullptr;

    /** @brief 各 Agent 初始 Transform 快照（用于 ResetSimulation） */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY()
    // 解释：这一行声明成员或局部变量 `InitialAgentTransforms`，用于保存initialagenttransforms。
    TMap<FString, FTransform> InitialAgentTransforms;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
};





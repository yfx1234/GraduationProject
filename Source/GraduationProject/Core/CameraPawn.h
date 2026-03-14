// 解释：使用 `#pragma once` 防止该头文件在编译过程中被重复包含。
#pragma once

// 解释：引入 Unreal 的核心基础头文件，提供常用容器、数学类型和日志宏。
#include "CoreMinimal.h"
// 解释：引入 `Pawn.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "GameFramework/Pawn.h"
// 解释：引入 `SpringArmComponent.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "GameFramework/SpringArmComponent.h"
// 解释：引入 `CameraComponent.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "Camera/CameraComponent.h"
// 解释：引入 `CameraPawn.generated.h`，让 Unreal Header Tool 生成的反射代码在本文件中可见。
#include "CameraPawn.generated.h"

// 解释：这一行声明 类 `ADronePawn`，用于封装adronePawn相关的数据与行为。
class ADronePawn;

/** @brief 无人机观察模式循环状态 */
// 解释：使用 `UENUM` 宏把枚举注册给 Unreal 反射系统和蓝图。
UENUM()
// 解释：这一行声明枚举 `EDroneViewCycleMode`，用于约束一组有限的状态或模式取值。
enum class EDroneViewCycleMode : uint8
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    Chase = 0,
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    TopDown,
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    FPV
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
};

/**
 * @brief 场景自由相机 Pawn
 * 支持自由移动、目标跟随、无人机追踪视角切换，以及与 HUD 中 Agent List 联动。
 */
// 解释：使用 `UCLASS` 宏把当前类型注册到 Unreal 反射系统中，便于蓝图、序列化和编辑器识别。
UCLASS()
// 解释：这一行声明 类 `ACameraPawn`，用于封装acameraPawn相关的数据与行为。
class GRADUATIONPROJECT_API ACameraPawn : public APawn
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：展开 Unreal Header Tool 生成的样板代码，这是 UObject、Actor、组件等类型正常工作的基础。
    GENERATED_BODY()

// 解释：从这一行开始进入 `public` 访问区，下面的成员会对外部模块公开。
public:
    /** @brief 构造相机 Pawn，创建根节点、弹簧臂和相机组件 */
    // 解释：调用 `ACameraPawn` 执行当前步骤需要的功能逻辑。
    ACameraPawn();

    /** @brief 初始化输入模式和鼠标状态 */
    // 解释：调用 `BeginPlay` 执行当前步骤需要的功能逻辑。
    virtual void BeginPlay() override;

    /** @brief 每帧更新自由相机或跟踪相机状态 */
    // 解释：调用 `Tick` 执行当前步骤需要的功能逻辑。
    virtual void Tick(float DeltaTime) override;

    /** @brief 绑定移动、视角和快捷键输入 */
    // 解释：调用 `SetupPlayerInputComponent` 执行当前步骤需要的功能逻辑。
    virtual void SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent) override;

    /**
     * @brief 开始跟踪指定目标
     * @param Target 需要被观察的 Actor
     */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "Camera")
    // 解释：调用 `StartTracking` 执行当前步骤需要的功能逻辑。
    void StartTracking(AActor* Target);

    /** @brief 停止跟踪并回到自由视角 */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "Camera")
    // 解释：调用 `StopTracking` 执行当前步骤需要的功能逻辑。
    void StopTracking();

    /**
     * @brief 响应 Agent 列表点击事件
     * @param AgentId 被点击的智能体 ID
     * @param Actor 对应的 Actor 实例
     */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "Camera")
    // 解释：调用 `OnItemClicked` 执行当前步骤需要的功能逻辑。
    void OnItemClicked(const FString& AgentId, AActor* Actor);

// 解释：从这一行开始进入 `protected` 访问区，下面的成员只对本类及其派生类可见。
protected:
    /** @brief 处理前后移动输入 */
    // 解释：调用 `MoveForward` 执行当前步骤需要的功能逻辑。
    void MoveForward(float Value);

    /** @brief 处理左右平移输入 */
    // 解释：调用 `MoveRight` 执行当前步骤需要的功能逻辑。
    void MoveRight(float Value);

    /** @brief 处理上下移动输入 */
    // 解释：调用 `MoveUp` 执行当前步骤需要的功能逻辑。
    void MoveUp(float Value);

    /** @brief 处理俯仰输入 */
    // 解释：调用 `CameraPitch` 执行当前步骤需要的功能逻辑。
    void CameraPitch(float Value);

    /** @brief 处理偏航输入 */
    // 解释：调用 `CameraYaw` 执行当前步骤需要的功能逻辑。
    void CameraYaw(float Value);

    /** @brief 缩小跟踪距离或俯视距离 */
    // 解释：调用 `OnZoomIn` 执行当前步骤需要的功能逻辑。
    void OnZoomIn();

    /** @brief 放大跟踪距离或俯视距离 */
    // 解释：调用 `OnZoomOut` 执行当前步骤需要的功能逻辑。
    void OnZoomOut();

    /** @brief 在追踪/俯视/FPV 视角之间循环切换 */
    // 解释：调用 `OnCycleDroneView` 执行当前步骤需要的功能逻辑。
    void OnCycleDroneView();

    /** @brief 强制切回自由视角 */
    // 解释：调用 `OnSwitchFreeView` 执行当前步骤需要的功能逻辑。
    void OnSwitchFreeView();

    /** @brief 更新绕目标的追踪相机轨道位置 */
    // 解释：调用 `UpdateTrackingCamera` 执行当前步骤需要的功能逻辑。
    void UpdateTrackingCamera(float DeltaTime);

    /** @brief 更新无人机俯视观察视角 */
    // 解释：调用 `UpdateTopDownCamera` 执行当前步骤需要的功能逻辑。
    void UpdateTopDownCamera();

    /** @brief 更新无人机第一人称视角 */
    // 解释：调用 `UpdateFPVCamera` 执行当前步骤需要的功能逻辑。
    void UpdateFPVCamera();

    /**
     * @brief 解析当前激活的无人机
     * @param bAutoSelect 为真时允许自动选择场景中可用无人机
     * @return 当前用于视角切换的无人机实例
     */
    // 解释：这一行把右侧表达式的结果写入 `ADronePawn* ResolveActiveDrone(bool bAutoSelect`，完成 adronePawnresolveactive无人机boolBautoselect 的更新。
    ADronePawn* ResolveActiveDrone(bool bAutoSelect = true);


// 解释：从这一行开始进入 `protected` 访问区，下面的成员只对本类及其派生类可见。
protected:
    /** @brief 根场景组件 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(VisibleAnywhere, Category = "Camera")
    // 解释：这一行声明成员或局部变量 `RootComp`，用于保存rootcomp。
    USceneComponent* RootComp;

    /** @brief 弹簧臂组件，负责相机偏移与轨道控制 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(VisibleAnywhere, Category = "Camera")
    // 解释：这一行声明成员或局部变量 `SpringArm`，用于保存springarm。
    USpringArmComponent* SpringArm;

    /** @brief 主观察相机 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(VisibleAnywhere, Category = "Camera")
    // 解释：这一行声明成员或局部变量 `Camera`，用于保存相机。
    UCameraComponent* Camera;

    /** @brief 自由移动速度 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Settings")
    // 解释：这一行声明成员或局部变量 `MoveSpeed`，用于保存movespeed。
    float MoveSpeed = 600.0f;

    /** @brief 鼠标灵敏度 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Settings")
    // 解释：这一行声明成员或局部变量 `MouseSensitivity`，用于保存mousesensitivity。
    float MouseSensitivity = 1.0f;

    /** @brief 缩放步长 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Settings")
    // 解释：这一行声明成员或局部变量 `ZoomSpeed`，用于保存zoomspeed。
    float ZoomSpeed = 50.0f;

    /** @brief 默认跟踪距离 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Settings")
    // 解释：这一行声明成员或局部变量 `TrackingDistance`，用于保存trackingdistance。
    float TrackingDistance = 500.0f;

    /** @brief 跟踪距离下限 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Settings")
    // 解释：这一行声明成员或局部变量 `MinTrackingDistance`，用于保存mintrackingdistance。
    float MinTrackingDistance = 100.0f;

    /** @brief 跟踪距离上限 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Settings")
    // 解释：这一行声明成员或局部变量 `MaxTrackingDistance`，用于保存maxtrackingdistance。
    float MaxTrackingDistance = 3000.0f;

// 解释：从这一行开始进入 `private` 访问区，下面的成员只允许类内部访问。
private:
    /** @brief 当前是否处于目标跟踪模式 */
    // 解释：这一行声明成员或局部变量 `bIsTracking`，用于保存布尔标志 istracking。
    bool bIsTracking = false;

    /** @brief 当前跟踪目标 */
    // 解释：这一行声明成员或局部变量 `TrackingTarget`，用于保存trackingtarget。
    AActor* TrackingTarget = nullptr;

    /** @brief 绕目标的水平轨道角 */
    // 解释：这一行声明成员或局部变量 `OrbitYaw`，用于保存orbityaw。
    float OrbitYaw = 0.0f;

    /** @brief 绕目标的俯仰轨道角 */
    // 解释：这一行声明成员或局部变量 `OrbitPitch`，用于保存orbitpitch。
    float OrbitPitch = -30.0f;

    /** @brief 累计的平移输入方向 */
    // 解释：这一行声明成员或局部变量 `InputMoveDirection`，用于保存inputmovedirection。
    FVector InputMoveDirection = FVector::ZeroVector;

    /** @brief 累计的俯仰输入 */
    // 解释：这一行声明成员或局部变量 `InputPitchValue`，用于保存inputpitchvalue。
    float InputPitchValue = 0.0f;

    /** @brief 累计的偏航输入 */
    // 解释：这一行声明成员或局部变量 `InputYawValue`，用于保存inputyawvalue。
    float InputYawValue = 0.0f;

    /** @brief 当前激活的无人机 ID */
    // 解释：这一行声明成员或局部变量 `ActiveDroneId`，用于保存active无人机id。
    FString ActiveDroneId;

    /** @brief 当前无人机观察模式 */
    // 解释：这一行声明成员或局部变量 `DroneViewMode`，用于保存无人机view模式。
    EDroneViewCycleMode DroneViewMode = EDroneViewCycleMode::Chase;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
};
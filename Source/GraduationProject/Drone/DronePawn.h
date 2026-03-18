// 解释：使用 `#pragma once` 防止该头文件在编译过程中被重复包含。
#pragma once

// 解释：引入 `CineCameraComponent.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "CineCameraComponent.h"
// 解释：引入 `SceneCaptureComponent2D.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "Components/SceneCaptureComponent2D.h"
// 解释：引入 `StaticMeshComponent.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "Components/StaticMeshComponent.h"
// 解释：引入 Unreal 的核心基础头文件，提供常用容器、数学类型和日志宏。
#include "CoreMinimal.h"
// 解释：引入 `DroneParameters.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "DroneParameters.h"
// 解释：引入 `DroneState.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "DroneState.h"
// 解释：引入 `TextureRenderTarget2D.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "Engine/TextureRenderTarget2D.h"
// 解释：引入 `Pawn.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "GameFramework/Pawn.h"
// 解释：引入 `DronePawn.generated.h`，让 Unreal Header Tool 生成的反射代码在本文件中可见。
#include "DronePawn.generated.h"

// 解释：这一行声明 类 `UDroneApi`，用于封装udrone接口相关的数据与行为。
class UDroneApi;
// 解释：这一行声明 类 `UDroneMovementComponent`，用于封装udrone运动组件相关的数据与行为。
class UDroneMovementComponent;

/**
 * @brief 无人机任务角色
 *
 * 用于区分场景中的无人机在拦截任务中的职责，便于 Guidance 模块自动查找。
 */
// 解释：使用 `UENUM` 宏把枚举注册给 Unreal 反射系统和蓝图。
UENUM(BlueprintType)
// 解释：这一行声明枚举 `EDroneMissionRole`，用于约束一组有限的状态或模式取值。
enum class EDroneMissionRole : uint8
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    /** @brief 未指定任务角色 */
    // 解释：这一行定义函数 `UMETA`，开始实现umeta的具体逻辑。
    Unknown     UMETA(DisplayName = "Unknown"),

    /** @brief 目标无人机，被拦截对象 */
    // 解释：这一行继续补充函数 `UMETA` 的参数列表、限定符或返回类型说明。
    Target      UMETA(DisplayName = "Target"),

    /** @brief 拦截无人机，主动执行追踪/捕获任务 */
    // 解释：这一行收束函数 `UMETA` 的签名，后面会进入实现体或以分号结束声明。
    Interceptor UMETA(DisplayName = "Interceptor")
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
};

/**
 * @brief 无人机主 Pawn
 *
 * 该类集成了：
 * 1. 飞行动力学组件 `UDroneMovementComponent`；
 * 2. 对外控制接口 `UDroneApi`；
 * 3. 云台相机与图像抓取组件；
 * 4. 与 AgentManager 的注册逻辑。
 *
 * 因此它既是仿真中的物理载体，也是 Python / Blueprint 侧远程控制的核心入口。
 */
// 解释：使用 `UCLASS` 宏把当前类型注册到 Unreal 反射系统中，便于蓝图、序列化和编辑器识别。
UCLASS()
// 解释：这一行声明 类 `ADronePawn`，用于封装adronePawn相关的数据与行为。
class GRADUATIONPROJECT_API ADronePawn : public APawn
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：展开 Unreal Header Tool 生成的样板代码，这是 UObject、Actor、组件等类型正常工作的基础。
    GENERATED_BODY()

// 解释：从这一行开始进入 `public` 访问区，下面的成员会对外部模块公开。
public:
    /** @brief 构造函数：创建全部子组件并设置默认参数 */
    // 解释：调用 `ADronePawn` 执行当前步骤需要的功能逻辑。
    ADronePawn();

    /** @brief 初始化动力学、相机 RenderTarget、API，并注册到 AgentManager */
    // 解释：调用 `BeginPlay` 执行当前步骤需要的功能逻辑。
    virtual void BeginPlay() override;

    /**
     * @brief 每帧更新无人机显示状态
     * @param DeltaTime 帧间隔（s）
     *
     * 主要负责：
     * - 从动力学组件同步当前状态；
     * - 把状态写回 Actor 位置/姿态；
     * - 刷新旋翼动画；
     * - 插值更新云台相机姿态；
     * - 同步电影相机后处理到捕获组件。
     */
    // 解释：调用 `Tick` 执行当前步骤需要的功能逻辑。
    virtual void Tick(float DeltaTime) override;

    /**
     * @brief 设置目标位置，切换到位置控制模式
     * @param NewTargetPosition 目标位置（m）
     * @param Speed             飞行速度上限（m/s），0 表示使用默认值
     * @param Frame             输入坐标系，支持 `ue` 与 `ned`
     */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "DroneControl")
    // 解释：这一行把右侧表达式的结果写入 `void SetTargetPosition(const FVector& NewTargetPosition, float Speed`，完成 voidsettargetpositionconstfvectornewtargetpositionfloatspeed 的更新。
    void SetTargetPosition(const FVector& NewTargetPosition, float Speed = 0.0f, FString Frame = TEXT("ue"));

    /**
     * @brief 设置目标速度，切换到速度控制模式
     * @param NewTargetVelocity 目标速度（m/s）
     * @param Frame             输入坐标系，支持 `ue` 与 `ned`
     */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "DroneControl")
    // 解释：这一行把右侧表达式的结果写入 `void SetTargetVelocity(const FVector& NewTargetVelocity, FString Frame`，完成 voidsettargetvelocityconstfvectornewtargetvelocityfstringframe 的更新。
    void SetTargetVelocity(const FVector& NewTargetVelocity, FString Frame = TEXT("ue"));

    /**
     * @brief 设置航向控制模式
     * @param NewYawMode     偏航控制模式
     * @param NewDrivetrain  驱动模式
     * @param YawDeg         目标偏航角或偏航角速度（视模式而定）
     */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "DroneControl")
    // 解释：这一行把右侧表达式的结果写入 `void SetHeadingControl(EDroneYawMode NewYawMode, EDroneDrivetrainMode NewDrivetrain, float YawDeg`，完成 voidsetheadingcontroledroneyaw模式newyaw模式edronedrivetrain模式newdrivetrainfloatyawdeg 的更新。
    void SetHeadingControl(EDroneYawMode NewYawMode, EDroneDrivetrainMode NewDrivetrain, float YawDeg = 0.0f);

    /** @brief 在当前位置悬停 */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "DroneControl")
    // 解释：调用 `Hover` 执行当前步骤需要的功能逻辑。
    void Hover();

    /**
     * @brief 以速度模式飞行的便捷封装
     * @param Vx X 方向速度（m/s）
     * @param Vy Y 方向速度（m/s）
     * @param Vz Z 方向速度（m/s）
     * @param Frame 输入坐标系
     */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "DroneControl")
    // 解释：这一行把右侧表达式的结果写入 `void MoveByVelocity(float Vx, float Vy, float Vz, FString Frame`，完成 voidmovebyvelocityfloatvxfloatvyfloatvzfstringframe 的更新。
    void MoveByVelocity(float Vx, float Vy, float Vz, FString Frame = TEXT("ue"));

    /**
     * @brief 起飞到指定高度
     * @param Altitude 目标高度（m）
     */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "Drone")
    // 解释：这一行把右侧表达式的结果写入 `void Takeoff(float Altitude`，完成 voidtakeofffloataltitude 的更新。
    void Takeoff(float Altitude = 3.0f);

    /** @brief 降落到地面高度 */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "Drone")
    // 解释：调用 `Land` 执行当前步骤需要的功能逻辑。
    void Land();

    /**
     * @brief 启用或禁用 API 远程控制标志
     * @param bEnable true 为启用，false 为禁用
     */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "Drone")
    // 解释：这一行把右侧表达式的结果写入 `void EnableApiControl(bool bEnable`，完成 voidenable接口controlboolBenable 的更新。
    void EnableApiControl(bool bEnable = true);

    /**
     * @brief 把无人机重置到指定位置和姿态
     * @param NewLocation 目标位置（m）
     * @param NewRotation 目标姿态
     * @param Frame       输入坐标系
     */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "DroneControl")
    // 解释：这一行把右侧表达式的结果写入 `void ResetDrone(const FVector& NewLocation, const FRotator& NewRotation, FString Frame`，完成 voidreset无人机constfvectornewlocationconstfrotatornewrotationfstringframe 的更新。
    void ResetDrone(const FVector& NewLocation, const FRotator& NewRotation, FString Frame = TEXT("ue"));

    /** @brief 重置 API / 控制状态，但不直接修改当前物理位置 */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "DroneControl")
    // 解释：调用 `ResetActorState` 执行当前步骤需要的功能逻辑。
    void ResetActorState();

    /**
     * @brief 设置目标姿态与总推力，进入姿态控制模式
     * @param RollDeg  横滚角（deg）
     * @param PitchDeg 俯仰角（deg）
     * @param YawDeg   偏航角（deg）
     * @param Thrust   总推力（N）
     * @param Frame    输入坐标系，默认 `ned`
     */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "DroneControl")
    // 解释：这一行把右侧表达式的结果写入 `void SetTargetAttitude(float RollDeg, float PitchDeg, float YawDeg, float Thrust`，完成 voidsettargetattitudefloatrolldegfloatpitchdegfloatyawdegfloatthrust 的更新。
    void SetTargetAttitude(float RollDeg, float PitchDeg, float YawDeg, float Thrust = 9.81f, FString Frame = TEXT("ned"));

    /**
     * @brief 直接设置四个电机角速度，进入电机转速控制模式
     * @param M0 电机 0 转速（rad/s）
     * @param M1 电机 1 转速（rad/s）
     * @param M2 电机 2 转速（rad/s）
     * @param M3 电机 3 转速（rad/s）
     */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "DroneControl")
    // 解释：调用 `SetMotorSpeeds` 执行当前步骤需要的功能逻辑。
    void SetMotorSpeeds(float M0, float M1, float M2, float M3);

    /**
     * @brief 设置位置环控制器增益
     * @param Kp 比例增益
     * @param Kd 微分增益
     */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "DroneControl")
    // 解释：这一行把右侧表达式的结果写入 `void SetPositionControllerGains(float Kp, float Kd`，完成 voidsetposition控制器gainsfloatkpfloatkd 的更新。
    void SetPositionControllerGains(float Kp, float Kd = 0.0f);

    /**
     * @brief 设置速度环控制器增益
     * @param Kp 比例增益
     * @param Ki 积分增益
     * @param Kd 微分增益
     */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "DroneControl")
    // 解释：这一行把右侧表达式的结果写入 `void SetVelocityControllerGains(float Kp, float Ki`，完成 voidsetvelocity控制器gainsfloatkpfloatki 的更新。
    void SetVelocityControllerGains(float Kp, float Ki = 0.0f, float Kd = 0.0f);

    /**
     * @brief 设置姿态环控制器增益
     * @param Kp 比例增益
     * @param Kd 微分增益
     */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "DroneControl")
    // 解释：这一行把右侧表达式的结果写入 `void SetAttitudeControllerGains(float Kp, float Kd`，完成 voidsetattitude控制器gainsfloatkpfloatkd 的更新。
    void SetAttitudeControllerGains(float Kp, float Kd = 0.0f);

    /**
     * @brief 设置角速度环比例增益
     * @param Kp 比例增益
     */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "DroneControl")
    // 解释：调用 `SetAngleRateControllerGains` 执行当前步骤需要的功能逻辑。
    void SetAngleRateControllerGains(float Kp);

    /** @brief 获取当前位置（m） */
    // 解释：调用 `GetCurrentPosition` 执行当前步骤需要的功能逻辑。
    FVector GetCurrentPosition() const;

    /** @brief 获取当前速度（m/s） */
    // 解释：调用 `GetCurrentVelocity` 执行当前步骤需要的功能逻辑。
    FVector GetCurrentVelocity() const;

    /**
     * @brief 设置云台目标角度
     * @param TargetPitch 目标俯仰角（deg）
     * @param TargetYaw   目标偏航角（deg）
     */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "Drone|Camera")
    // 解释：调用 `SetCameraAngles` 执行当前步骤需要的功能逻辑。
    void SetCameraAngles(float TargetPitch, float TargetYaw);

    UFUNCTION(BlueprintCallable, Category = "Drone|Camera")
    void SetCameraFOV(float NewFOV);

    /**
     * @brief 设置该无人机的语义分割 Stencil ID
     * @param NewSegmentationId 0-255 的分割标识
     */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "Drone|Camera")
    // 解释：调用 `SetSegmentationId` 执行当前步骤需要的功能逻辑。
    void SetSegmentationId(int32 NewSegmentationId);

    /**
     * @brief 获取无人机状态 JSON
     * @param Frame 输出坐标系，支持 `ue` 与 `ned`
     * @return JSON 格式状态快照
     */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "Drone")
    // 解释：这一行把右侧表达式的结果写入 `FString GetState(FString Frame`，完成 fstringget状态fstringframe 的更新。
    FString GetState(FString Frame = TEXT("ue"));

    /**
     * @brief 获取机载相机图像 JSON
     * @param ImageType      图像类型，如 `scene`、`depth_planar`
     * @param Quality        JPEG 质量，小于等于 0 时使用默认值
     * @param MaxDepthMeters 深度图最大映射距离（m）
     * @return JSON 格式图像结果
     */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "Drone|Camera")
    // 解释：这一行把右侧表达式的结果写入 `FString GetImage(FString ImageType`，完成 fstringget图像fstring图像type 的更新。
    FString GetImage(FString ImageType = TEXT("scene"), int32 Quality = -1, float MaxDepthMeters = 200.0f);

    /**
     * @brief 抓取彩色图像并返回 Base64 JPEG
     * @param Quality JPEG 质量，小于等于 0 时使用默认值
     * @return Base64 编码的 JPEG 字符串
     */
    // 解释：这一行把右侧表达式的结果写入 `FString CaptureImageBase64(int32 Quality`，完成 fstring采集图像base64int32quality 的更新。
    FString CaptureImageBase64(int32 Quality = -1);

    /** @brief 获取当前云台俯仰角（deg） */
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    float GetCameraCurrentPitch() const { return CameraCurrentPitch; }

    /** @brief 获取当前云台偏航角（deg） */
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    float GetCameraCurrentYaw() const { return CameraCurrentYaw; }

    /** @brief 当前飞行状态，包含位置、速度、姿态和电机转速等量 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|State")
    // 解释：这一行声明成员或局部变量 `CurrentState`，用于保存current状态。
    FDroneState CurrentState;

    /** @brief 当前控制模式 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|State")
    // 解释：这一行声明成员或局部变量 `ControlMode`，用于保存control模式。
    EDroneControlMode ControlMode = EDroneControlMode::Idle;

    /** @brief 无人机物理参数 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters")
    // 解释：这一行声明成员或局部变量 `Parameters`，用于保存参数。
    FDroneParameters Parameters;

    /** @brief 飞行动力学与级联控制组件 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    // 解释：这一行声明成员或局部变量 `MovementComp`，用于保存飞行运动组件。
    UDroneMovementComponent* MovementComp = nullptr;

    /** @brief 高层控制 API 对象 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY()
    // 解释：这一行声明成员或局部变量 `Api`，用于保存接口。
    UDroneApi* Api = nullptr;

    /** @brief 无人机唯一标识，用于注册和远程调用 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Config")
    // 解释：这一行把右侧表达式的结果写入 `FString DroneId`，完成 fstring无人机id 的更新。
    FString DroneId = TEXT("drone_0");

    /** @brief 无人机任务角色 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Config")
    // 解释：这一行声明成员或局部变量 `MissionRole`，用于保存missionrole。
    EDroneMissionRole MissionRole = EDroneMissionRole::Unknown;

    /** @brief 是否允许 API 远程控制 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Config")
    // 解释：这一行声明成员或局部变量 `bApiControlEnabled`，用于保存布尔标志 接口controlenabled。
    bool bApiControlEnabled = true;

    /** @brief 根场景组件 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    // 解释：这一行声明成员或局部变量 `RootComp`，用于保存rootcomp。
    USceneComponent* RootComp;

    /** @brief 机体静态网格 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    // 解释：这一行声明成员或局部变量 `BodyMesh`，用于保存bodymesh。
    UStaticMeshComponent* BodyMesh;

    /** @brief 旋翼 0 网格 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    // 解释：这一行声明成员或局部变量 `Fan0`，用于保存fan0。
    UStaticMeshComponent* Fan0;

    /** @brief 旋翼 1 网格 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    // 解释：这一行声明成员或局部变量 `Fan1`，用于保存fan1。
    UStaticMeshComponent* Fan1;

    /** @brief 旋翼 2 网格 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    // 解释：这一行声明成员或局部变量 `Fan2`，用于保存fan2。
    UStaticMeshComponent* Fan2;

    /** @brief 旋翼 3 网格 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    // 解释：这一行声明成员或局部变量 `Fan3`，用于保存fan3。
    UStaticMeshComponent* Fan3;

    /** @brief 云台偏航轴网格 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    // 解释：这一行声明成员或局部变量 `CameraYawMesh`，用于保存相机yawmesh。
    UStaticMeshComponent* CameraYawMesh;

    /** @brief 云台俯仰轴网格 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    // 解释：这一行声明成员或局部变量 `CameraPitchMesh`，用于保存相机pitchmesh。
    UStaticMeshComponent* CameraPitchMesh;

    /** @brief 场景捕获组件，用于生成 API 图像 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Camera")
    // 解释：这一行声明成员或局部变量 `DroneSceneCapture`，用于保存无人机scene采集。
    USceneCaptureComponent2D* DroneSceneCapture;

    /** @brief 电影相机组件，用于同步后处理参数 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Camera")
    // 解释：这一行声明成员或局部变量 `DroneCineCamera`，用于保存无人机cine相机。
    UCineCameraComponent* DroneCineCamera;

    /** @brief 相机水平视场角（deg） */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera")
    // 解释：这一行声明成员或局部变量 `CameraFOV`，用于保存相机fov。
    float CameraFOV = 90.0f;

    /** @brief 捕获图像宽度（px） */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera")
    // 解释：这一行声明成员或局部变量 `CameraWidth`，用于保存相机width。
    int32 CameraWidth = 1280;

    /** @brief 捕获图像高度（px） */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera")
    // 解释：这一行声明成员或局部变量 `CameraHeight`，用于保存相机height。
    int32 CameraHeight = 720;

    /** @brief 语义分割模板值 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera", meta = (ClampMin = "0", ClampMax = "255"))
    // 解释：这一行声明成员或局部变量 `SegmentationId`，用于保存segmentationid。
    int32 SegmentationId = 20;

    /** @brief 曝光补偿偏置 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera", meta = (ClampMin = "-10.0", ClampMax = "10.0"))
    // 解释：这一行声明成员或局部变量 `ExposureBias`，用于保存exposurebias。
    float ExposureBias = 0.0f;

    /** @brief JPEG 默认编码质量 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera", meta = (ClampMin = "1", ClampMax = "100"))
    // 解释：这一行声明成员或局部变量 `JpegQuality`，用于保存jpegquality。
    int32 JpegQuality = 90;

    /** @brief 云台角度插值速度 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera")
    // 解释：这一行声明成员或局部变量 `CameraRotationSpeed`，用于保存相机rotationspeed。
    float CameraRotationSpeed = 8.0f;

// 解释：从这一行开始进入 `private` 访问区，下面的成员只允许类内部访问。
private:
    /**
     * @brief 把飞行状态写回到 Actor 世界变换
     * @param State 当前飞行状态
     */
    // 解释：调用 `ApplyStateToActor` 执行当前步骤需要的功能逻辑。
    void ApplyStateToActor(const FDroneState& State);

    /**
     * @brief 更新四个旋翼的可视化转动动画
     * @param DeltaTime 帧间隔（s）
     */
    // 解释：调用 `UpdatePropellerAnimation` 执行当前步骤需要的功能逻辑。
    void UpdatePropellerAnimation(float DeltaTime);

    /**
     * @brief 根据索引返回对应旋翼网格
     * @param Index 旋翼索引 0-3
     * @return 对应网格指针，不存在时返回 nullptr
     */
    // 解释：调用 `GetFanMesh` 执行当前步骤需要的功能逻辑。
    UStaticMeshComponent* GetFanMesh(int32 Index) const;

    /**
     * @brief 平滑插值更新云台姿态
     * @param DeltaTime 帧间隔（s）
     */
    // 解释：调用 `UpdateCameraRotation` 执行当前步骤需要的功能逻辑。
    void UpdateCameraRotation(float DeltaTime);

    /** @brief 云台目标俯仰角（deg） */
    // 解释：这一行声明成员或局部变量 `CameraTargetPitch`，用于保存相机targetpitch。
    float CameraTargetPitch = 0.0f;

    /** @brief 云台目标偏航角（deg） */
    // 解释：这一行声明成员或局部变量 `CameraTargetYaw`，用于保存相机targetyaw。
    float CameraTargetYaw = 0.0f;

    /** @brief 云台当前俯仰角（deg） */
    // 解释：这一行声明成员或局部变量 `CameraCurrentPitch`，用于保存相机currentpitch。
    float CameraCurrentPitch = 0.0f;

    /** @brief 云台当前偏航角（deg） */
    // 解释：这一行声明成员或局部变量 `CameraCurrentYaw`，用于保存相机currentyaw。
    float CameraCurrentYaw = 0.0f;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
};

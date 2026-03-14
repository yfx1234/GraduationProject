// 解释：使用 `#pragma once` 防止该头文件在编译过程中被重复包含。
#pragma once

// 解释：引入 Unreal 的核心基础头文件，提供常用容器、数学类型和日志宏。
#include "CoreMinimal.h"
// 解释：引入 `NoExportTypes.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "UObject/NoExportTypes.h"
// 解释：引入 `DroneParameters.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "DroneParameters.h"
// 解释：引入 `DroneApi.generated.h`，让 Unreal Header Tool 生成的反射代码在本文件中可见。
#include "DroneApi.generated.h"

// 解释：这一行声明 类 `ADronePawn`，用于封装adronePawn相关的数据与行为。
class ADronePawn;

/**
 * @brief 无人机高层控制接口
 * 对外提供位置、速度、姿态和控制器参数调节等便捷调用，
 * 本质上是对 `ADronePawn` 和 `UDroneMovementComponent` 的一层轻量包装。
 */
// 解释：使用 `UCLASS` 宏把当前类型注册到 Unreal 反射系统中，便于蓝图、序列化和编辑器识别。
UCLASS()
// 解释：这一行声明 类 `UDroneApi`，用于封装udrone接口相关的数据与行为。
class GRADUATIONPROJECT_API UDroneApi : public UObject
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：展开 Unreal Header Tool 生成的样板代码，这是 UObject、Actor、组件等类型正常工作的基础。
    GENERATED_BODY()

// 解释：从这一行开始进入 `public` 访问区，下面的成员会对外部模块公开。
public:
    /**
     * @brief 初始化 API，并绑定到目标无人机
     * @param Owner 拥有该 API 的 `ADronePawn`
     */
    // 解释：调用 `Initialize` 执行当前步骤需要的功能逻辑。
    void Initialize(ADronePawn* Owner);

    /**
     * @brief 以位置模式飞往指定坐标
     * @param X 目标 X 坐标（米）
     * @param Y 目标 Y 坐标（米）
     * @param Z 目标 Z 坐标（米）
     * @param Speed 飞行速度（米/秒）
     */
    // 解释：这一行把右侧表达式的结果写入 `void MoveToPosition(float X, float Y, float Z, float Speed`，完成 voidmovetopositionfloatXfloatYfloatZfloatspeed 的更新。
    void MoveToPosition(float X, float Y, float Z, float Speed = 2.0f);

    /** @brief 在当前位置悬停 */
    // 解释：调用 `Hover` 执行当前步骤需要的功能逻辑。
    void Hover();

    /**
     * @brief 起飞到指定高度
     * @param Altitude 目标高度（米）
     */
    // 解释：调用 `Takeoff` 执行当前步骤需要的功能逻辑。
    void Takeoff(float Altitude);

    /** @brief 降落到地面 */
    // 解释：调用 `Land` 执行当前步骤需要的功能逻辑。
    void Land();

    /**
     * @brief 以速度模式飞行
     * @param Vx X 方向速度（米/秒）
     * @param Vy Y 方向速度（米/秒）
     * @param Vz Z 方向速度（米/秒）
     */
    // 解释：调用 `MoveByVelocity` 执行当前步骤需要的功能逻辑。
    void MoveByVelocity(float Vx, float Vy, float Vz);

    /**
     * @brief 设置目标姿态和总推力
     * @param RollDeg 目标横滚角（度）
     * @param PitchDeg 目标俯仰角（度）
     * @param YawDeg 目标偏航角（度）
     * @param Thrust 目标总推力
     */
    // 解释：调用 `SetTargetAttitude` 执行当前步骤需要的功能逻辑。
    void SetTargetAttitude(float RollDeg, float PitchDeg, float YawDeg, float Thrust);

    /**
     * @brief 直接设置四个电机转速命令
     * @param M0 电机 0 转速（rad/s）
     * @param M1 电机 1 转速（rad/s）
     * @param M2 电机 2 转速（rad/s）
     * @param M3 电机 3 转速（rad/s）
     */
    // 解释：调用 `SetMotorSpeeds` 执行当前步骤需要的功能逻辑。
    void SetMotorSpeeds(float M0, float M1, float M2, float M3);

    /**
     * @brief 设置航向控制模式
     * @param YawMode 偏航控制模式
     * @param Drivetrain 运动学约束模式
     * @param YawDeg 目标偏航角；仅在角度模式下生效
     */
    // 解释：这一行把右侧表达式的结果写入 `void SetHeadingControl(EDroneYawMode YawMode, EDroneDrivetrainMode Drivetrain, float YawDeg`，完成 voidsetheadingcontroledroneyaw模式yaw模式edronedrivetrain模式drivetrainfloatyawdeg 的更新。
    void SetHeadingControl(EDroneYawMode YawMode, EDroneDrivetrainMode Drivetrain, float YawDeg = 0.0f);

    /**
     * @brief 获取当前无人机位置
     * @return 位置向量（米）
     */
    // 解释：调用 `GetPosition` 执行当前步骤需要的功能逻辑。
    FVector GetPosition() const;

    /**
     * @brief 获取当前无人机速度
     * @return 速度向量（米/秒）
     */
    // 解释：调用 `GetVelocity` 执行当前步骤需要的功能逻辑。
    FVector GetVelocity() const;

    /**
     * @brief 获取当前姿态
     * @return 欧拉角姿态 `(Roll, Pitch, Yaw)`
     */
    // 解释：调用 `GetOrientation` 执行当前步骤需要的功能逻辑。
    FRotator GetOrientation() const;

    /**
     * @brief 获取当前四个电机转速
     * @return 电机转速数组（rad/s）
     */
    // 解释：调用 `GetMotorSpeeds` 执行当前步骤需要的功能逻辑。
    TArray<float> GetMotorSpeeds() const;

    /**
     * @brief 获取当前控制模式
     * @return 控制模式枚举值
     */
    // 解释：调用 `GetControlMode` 执行当前步骤需要的功能逻辑。
    EDroneControlMode GetControlMode() const;

    /**
     * @brief 设置位置控制器增益
     * @param Kp 比例增益
     * @param Kd 微分增益
     */
    // 解释：调用 `SetPositionControllerGains` 执行当前步骤需要的功能逻辑。
    void SetPositionControllerGains(float Kp, float Kd);

    /**
     * @brief 设置速度控制器增益
     * @param Kp 比例增益
     * @param Ki 积分增益
     * @param Kd 微分增益
     */
    // 解释：调用 `SetVelocityControllerGains` 执行当前步骤需要的功能逻辑。
    void SetVelocityControllerGains(float Kp, float Ki, float Kd);

    /**
     * @brief 设置姿态控制器增益
     * @param Kp 比例增益
     * @param Kd 微分增益
     */
    // 解释：调用 `SetAttitudeControllerGains` 执行当前步骤需要的功能逻辑。
    void SetAttitudeControllerGains(float Kp, float Kd);

    /**
     * @brief 设置角速度控制器比例增益
     * @param Kp 比例增益
     */
    // 解释：调用 `SetAngleRateControllerGains` 执行当前步骤需要的功能逻辑。
    void SetAngleRateControllerGains(float Kp);

    /**
     * @brief 将无人机重置到指定位置和姿态
     * @param Position 目标位置（米）
     * @param Rotation 目标姿态
     */
    // 解释：这一行把右侧表达式的结果写入 `void Reset(FVector Position`，完成 voidresetfvectorposition 的更新。
    void Reset(FVector Position = FVector::ZeroVector, FRotator Rotation = FRotator::ZeroRotator);

// 解释：从这一行开始进入 `private` 访问区，下面的成员只允许类内部访问。
private:
    /** @brief 绑定的无人机 Pawn */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY()
    // 解释：这一行声明成员或局部变量 `OwnerPawn`，用于保存所属无人机 Pawn 指针。
    ADronePawn* OwnerPawn = nullptr;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
};

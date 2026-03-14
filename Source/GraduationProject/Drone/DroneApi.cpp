// 解释：引入当前实现文件对应的头文件 `DroneApi.h`，使实现部分能够看到类和函数声明。
#include "DroneApi.h"
// 解释：引入 `DronePawn.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "DronePawn.h"
// 解释：引入 `DroneMovementComponent.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "DroneMovementComponent.h"

/**
 * @brief 初始化无人机 API
 * @param Owner 目标无人机 Pawn
 */
// 解释：这一行定义函数 `Initialize`，开始实现initialize的具体逻辑。
void UDroneApi::Initialize(ADronePawn* Owner)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `OwnerPawn`，完成 所属无人机 Pawn 指针 的更新。
    OwnerPawn = Owner;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 以位置模式飞往指定坐标
 * @param X 目标 X 坐标（米）
 * @param Y 目标 Y 坐标（米）
 * @param Z 目标 Z 坐标（米）
 * @param Speed 飞行速度（米/秒）
 */
// 解释：这一行定义函数 `MoveToPosition`，开始实现movetoposition的具体逻辑。
void UDroneApi::MoveToPosition(float X, float Y, float Z, float Speed)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (OwnerPawn)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `SetTargetPosition` 执行当前步骤需要的功能逻辑。
        OwnerPawn->SetTargetPosition(FVector(X, Y, Z), Speed);
        // 解释：调用 `UE_LOG` 输出调试日志，便于运行时观察状态和排查问题。
        UE_LOG(LogTemp, Log, TEXT("[DroneApi] MoveToPosition(%.1f, %.1f, %.1f), speed=%.2f"), X, Y, Z, Speed);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 在当前位置悬停 */
// 解释：这一行定义函数 `Hover`，开始实现hover的具体逻辑。
void UDroneApi::Hover()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (OwnerPawn) OwnerPawn->Hover();
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 起飞到指定高度
 * @param Altitude 目标飞行高度（米）
 */
// 解释：这一行定义函数 `Takeoff`，开始实现takeoff的具体逻辑。
void UDroneApi::Takeoff(float Altitude)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (OwnerPawn) OwnerPawn->Takeoff(Altitude);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 执行降落命令 */
// 解释：这一行定义函数 `Land`，开始实现land的具体逻辑。
void UDroneApi::Land()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (OwnerPawn) OwnerPawn->Land();
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 以速度模式飞行
 * @param Vx X 方向速度（米/秒）
 * @param Vy Y 方向速度（米/秒）
 * @param Vz Z 方向速度（米/秒）
 */
// 解释：这一行定义函数 `MoveByVelocity`，开始实现movebyvelocity的具体逻辑。
void UDroneApi::MoveByVelocity(float Vx, float Vy, float Vz)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (OwnerPawn)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `SetTargetVelocity` 执行当前步骤需要的功能逻辑。
        OwnerPawn->SetTargetVelocity(FVector(Vx, Vy, Vz));
        // 解释：调用 `UE_LOG` 输出调试日志，便于运行时观察状态和排查问题。
        UE_LOG(LogTemp, Log, TEXT("[DroneApi] MoveByVelocity(%.1f, %.1f, %.1f)"), Vx, Vy, Vz);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 设置目标姿态和总推力
 * @param RollDeg 目标横滚角（度）
 * @param PitchDeg 目标俯仰角（度）
 * @param YawDeg 目标偏航角（度）
 * @param Thrust 目标总推力
 */
// 解释：这一行定义函数 `SetTargetAttitude`，开始实现settargetattitude的具体逻辑。
void UDroneApi::SetTargetAttitude(float RollDeg, float PitchDeg, float YawDeg, float Thrust)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (OwnerPawn && OwnerPawn->MovementComp)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `SetControlMode` 执行当前步骤需要的功能逻辑。
        OwnerPawn->MovementComp->SetControlMode(EDroneControlMode::AttitudeThrust);
        // 解释：调用 `SetTargetAttitude` 执行当前步骤需要的功能逻辑。
        OwnerPawn->MovementComp->SetTargetAttitude(FRotator(PitchDeg, YawDeg, RollDeg), Thrust);
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        OwnerPawn->ControlMode = EDroneControlMode::AttitudeThrust;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 直接设置电机转速命令
 * @param M0 电机 0 转速（rad/s）
 * @param M1 电机 1 转速（rad/s）
 * @param M2 电机 2 转速（rad/s）
 * @param M3 电机 3 转速（rad/s）
 */
// 解释：这一行定义函数 `SetMotorSpeeds`，开始实现setmotorspeeds的具体逻辑。
void UDroneApi::SetMotorSpeeds(float M0, float M1, float M2, float M3)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (OwnerPawn && OwnerPawn->MovementComp)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `SetControlMode` 执行当前步骤需要的功能逻辑。
        OwnerPawn->MovementComp->SetControlMode(EDroneControlMode::MotorSpeed);
        // 解释：调用 `SetControlCommand` 执行当前步骤需要的功能逻辑。
        OwnerPawn->MovementComp->SetControlCommand({ M0, M1, M2, M3 });
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        OwnerPawn->ControlMode = EDroneControlMode::MotorSpeed;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 设置航向控制模式
 * @param YawMode 偏航控制模式
 * @param Drivetrain 运动学约束模式
 * @param YawDeg 目标偏航角
 */
// 解释：这一行定义函数 `SetHeadingControl`，开始实现setheadingcontrol的具体逻辑。
void UDroneApi::SetHeadingControl(EDroneYawMode YawMode, EDroneDrivetrainMode Drivetrain, float YawDeg)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (OwnerPawn)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `SetHeadingControl` 执行当前步骤需要的功能逻辑。
        OwnerPawn->SetHeadingControl(YawMode, Drivetrain, YawDeg);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 获取当前位置
 * @return 位置向量（米）
 */
// 解释：这一行定义函数 `GetPosition`，开始实现getposition的具体逻辑。
FVector UDroneApi::GetPosition() const
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行使用三目表达式选择返回值，在正常路径和兜底路径之间做快速切换。
    return OwnerPawn ? OwnerPawn->GetCurrentPosition() : FVector::ZeroVector;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 获取当前速度
 * @return 速度向量（米/秒）
 */
// 解释：这一行定义函数 `GetVelocity`，开始实现getvelocity的具体逻辑。
FVector UDroneApi::GetVelocity() const
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行使用三目表达式选择返回值，在正常路径和兜底路径之间做快速切换。
    return OwnerPawn ? OwnerPawn->GetCurrentVelocity() : FVector::ZeroVector;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 获取当前姿态
 * @return 欧拉角姿态
 */
// 解释：这一行定义函数 `GetOrientation`，开始实现getorientation的具体逻辑。
FRotator UDroneApi::GetOrientation() const
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行使用三目表达式选择返回值，在正常路径和兜底路径之间做快速切换。
    return OwnerPawn ? OwnerPawn->CurrentState.GetRotator() : FRotator::ZeroRotator;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 获取当前四个电机转速
 * @return 电机转速数组（rad/s）
 */
// 解释：这一行定义函数 `GetMotorSpeeds`，开始实现getmotorspeeds的具体逻辑。
TArray<float> UDroneApi::GetMotorSpeeds() const
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行声明成员或局部变量 `Result`，用于保存result。
    TArray<float> Result;
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (OwnerPawn && OwnerPawn->CurrentState.MotorSpeeds.Num() == 4)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
        for (double Speed : OwnerPawn->CurrentState.MotorSpeeds)
            // 解释：调用 `Add` 执行当前步骤需要的功能逻辑。
            Result.Add(static_cast<float>(Speed));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return Result;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 获取当前控制模式
 * @return 控制模式枚举值
 */
// 解释：这一行定义函数 `GetControlMode`，开始实现getcontrol模式的具体逻辑。
EDroneControlMode UDroneApi::GetControlMode() const
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行使用三目表达式选择返回值，在正常路径和兜底路径之间做快速切换。
    return OwnerPawn ? OwnerPawn->ControlMode : EDroneControlMode::Idle;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 设置位置控制器增益
 * @param Kp 比例增益
 * @param Kd 微分增益
 */
// 解释：这一行定义函数 `SetPositionControllerGains`，开始实现setposition控制器gains的具体逻辑。
void UDroneApi::SetPositionControllerGains(float Kp, float Kd)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (OwnerPawn && OwnerPawn->MovementComp)
        // 解释：调用 `SetPositionGains` 执行当前步骤需要的功能逻辑。
        OwnerPawn->MovementComp->SetPositionGains(Kp, Kd);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 设置速度控制器增益
 * @param Kp 比例增益
 * @param Ki 积分增益
 * @param Kd 微分增益
 */
// 解释：这一行定义函数 `SetVelocityControllerGains`，开始实现setvelocity控制器gains的具体逻辑。
void UDroneApi::SetVelocityControllerGains(float Kp, float Ki, float Kd)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (OwnerPawn && OwnerPawn->MovementComp)
        // 解释：调用 `SetVelocityGains` 执行当前步骤需要的功能逻辑。
        OwnerPawn->MovementComp->SetVelocityGains(Kp, Ki, Kd);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 设置姿态控制器增益
 * @param Kp 比例增益
 * @param Kd 微分增益
 */
// 解释：这一行定义函数 `SetAttitudeControllerGains`，开始实现setattitude控制器gains的具体逻辑。
void UDroneApi::SetAttitudeControllerGains(float Kp, float Kd)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (OwnerPawn && OwnerPawn->MovementComp)
        // 解释：调用 `SetAttitudeGains` 执行当前步骤需要的功能逻辑。
        OwnerPawn->MovementComp->SetAttitudeGains(Kp, Kd);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 设置角速度控制器比例增益
 * @param Kp 比例增益
 */
// 解释：这一行定义函数 `SetAngleRateControllerGains`，开始实现setanglerate控制器gains的具体逻辑。
void UDroneApi::SetAngleRateControllerGains(float Kp)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (OwnerPawn && OwnerPawn->MovementComp)
        // 解释：调用 `SetAngleRateGains` 执行当前步骤需要的功能逻辑。
        OwnerPawn->MovementComp->SetAngleRateGains(Kp);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 重置无人机位置和姿态
 * @param Position 目标位置（米）
 * @param Rotation 目标姿态
 */
// 解释：这一行定义函数 `Reset`，开始实现reset的具体逻辑。
void UDroneApi::Reset(FVector Position, FRotator Rotation)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (OwnerPawn) OwnerPawn->ResetDrone(Position, Rotation);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

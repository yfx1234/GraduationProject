// 解释：使用 `#pragma once` 防止该头文件在编译过程中被重复包含。
#pragma once

// 解释：引入 Unreal 的核心基础头文件，提供常用容器、数学类型和日志宏。
#include "CoreMinimal.h"
// 解释：引入 `DroneState.generated.h`，让 Unreal Header Tool 生成的反射代码在本文件中可见。
#include "DroneState.generated.h"

/**
 * @brief 无人机状态结构体
 *   位置 (X, Y, Z)
 *   线速度 (Vx, Vy, Vz)
 *   姿态四元数 (Qw, Qx, Qy, Qz)
 *   角速度 (AngRollRate, AngPitchRate, AngYawRate)
 *   电机转速
 */
// 解释：使用 `USTRUCT` 宏声明可被 Unreal 反射系统识别的结构体类型。
USTRUCT(BlueprintType)
// 解释：这一行声明 结构体 `FDroneState`，用于封装fdrone状态相关的数据与行为。
struct GRADUATIONPROJECT_API FDroneState
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：展开 Unreal Header Tool 生成的样板代码，这是 UObject、Actor、组件等类型正常工作的基础。
    GENERATED_BODY()

    /** @brief X 坐标 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|Position")
    // 解释：这一行声明成员或局部变量 `X`，用于保存状态向量 X。
    double X = 0.0;
    
    /** @brief Y 坐标 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|Position")
    // 解释：这一行声明成员或局部变量 `Y`，用于保存Y。
    double Y = 0.0;
    
    /** @brief Z 坐标 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|Position")
    // 解释：这一行声明成员或局部变量 `Z`，用于保存Z。
    double Z = 0.0;

    /** @brief X 方向速度 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|Velocity")
    // 解释：这一行声明成员或局部变量 `Vx`，用于保存vx。
    double Vx = 0.0;
    
    /** @brief Y 方向速度 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|Velocity")
    // 解释：这一行声明成员或局部变量 `Vy`，用于保存vy。
    double Vy = 0.0;
    
    /** @brief Z 方向速度 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|Velocity")
    // 解释：这一行声明成员或局部变量 `Vz`，用于保存vz。
    double Vz = 0.0;

    /** @brief 四元数 W 分量 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|Attitude")
    // 解释：这一行声明成员或局部变量 `Qw`，用于保存qw。
    double Qw = 1.0;
    
    /** @brief 四元数 X 分量 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|Attitude")
    // 解释：这一行声明成员或局部变量 `Qx`，用于保存qx。
    double Qx = 0.0;
    
    /** @brief 四元数 Y 分量 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|Attitude")
    // 解释：这一行声明成员或局部变量 `Qy`，用于保存qy。
    double Qy = 0.0;
    
    /** @brief 四元数 Z 分量 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|Attitude")
    // 解释：这一行声明成员或局部变量 `Qz`，用于保存qz。
    double Qz = 0.0;

    /** @brief 滚转角速率 p */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|AngularVelocity")
    // 解释：这一行声明成员或局部变量 `AngRollRate`，用于保存angrollrate。
    double AngRollRate = 0.0;

    /** @brief 俯仰角速率 q */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|AngularVelocity")
    // 解释：这一行声明成员或局部变量 `AngPitchRate`，用于保存angpitchrate。
    double AngPitchRate = 0.0;

    /** @brief 偏航角速率 r */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|AngularVelocity")
    // 解释：这一行声明成员或局部变量 `AngYawRate`，用于保存angyawrate。
    double AngYawRate = 0.0;

    /** @brief 四个电机的角速度数组 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|State|Motors")
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    TArray<double> MotorSpeeds = {0.0, 0.0, 0.0, 0.0};

    /** @brief 获取位置向量 */
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    FVector GetPosition() const { return FVector(X, Y, Z); }

    /**
     * @brief 设置位置
     * @param Pos 位置向量
     */
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    void SetPosition(const FVector& Pos) { X = Pos.X; Y = Pos.Y; Z = Pos.Z; }
    
    /** @brief 获取速度向量 */
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    FVector GetVelocity() const { return FVector(Vx, Vy, Vz); }

    /**
     * @brief 设置速度
     * @param Vel 速度向量
     */
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    void SetVelocity(const FVector& Vel) { Vx = Vel.X; Vy = Vel.Y; Vz = Vel.Z; }
    
    /**
     * @brief 获取姿态四元数
     * @return UE FQuat
     */
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    FQuat GetQuaternion() const { return FQuat(Qx, Qy, Qz, Qw); }

    /**
     * @brief 设置姿态四元数
     * @param InQuat UE FQuat
     */
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    void SetQuaternion(const FQuat& InQuat) { Qw = InQuat.W; Qx = InQuat.X; Qy = InQuat.Y; Qz = InQuat.Z; }
    
    /**
     * @brief 获取姿态欧拉角
     * @return UE FRotator
     */
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    FRotator GetRotator() const { return GetQuaternion().Rotator(); }
    
    /**
     * @brief 获取角速度向量
     * @return FVector(p, q, r)
     */
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    FVector GetAngularVelocity() const { return FVector(AngRollRate, AngPitchRate, AngYawRate); }
    
    /** @brief 归一化四元数 */
    // 解释：这一行定义函数 `NormalizeQuaternion`，开始实现normalizequaternion的具体逻辑。
    void NormalizeQuaternion()
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `double Mag`，完成 doublemag 的更新。
        double Mag = FMath::Sqrt(Qw*Qw + Qx*Qx + Qy*Qy + Qz*Qz);
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (Mag > KINDA_SMALL_NUMBER)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行把右侧表达式的结果写入 `Qw`，完成 qw 的更新。
            Qw /= Mag;
            // 解释：这一行把右侧表达式的结果写入 `Qx`，完成 qx 的更新。
            Qx /= Mag;
            // 解释：这一行把右侧表达式的结果写入 `Qy`，完成 qy 的更新。
            Qy /= Mag;
            // 解释：这一行把右侧表达式的结果写入 `Qz`，完成 qz 的更新。
            Qz /= Mag;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
        // 解释：这一行给出兜底分支，当上面的条件都不满足时执行。
        else
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行把右侧表达式的结果写入 `Qw`，完成 qw 的更新。
            Qw = 1.0; Qx = 0.0; Qy = 0.0; Qz = 0.0;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    /** @brief 检测状态中是否包含 NaN 或 Inf */
    // 解释：这一行定义函数 `HasNaN`，开始实现hasnaN的具体逻辑。
    bool HasNaN() const
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return !FMath::IsFinite(X) || !FMath::IsFinite(Y) || !FMath::IsFinite(Z)
            // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
            || !FMath::IsFinite(Vx) || !FMath::IsFinite(Vy) || !FMath::IsFinite(Vz)
            // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
            || !FMath::IsFinite(Qw) || !FMath::IsFinite(Qx) || !FMath::IsFinite(Qy) || !FMath::IsFinite(Qz)
            // 解释：调用 `IsFinite` 执行当前步骤需要的功能逻辑。
            || !FMath::IsFinite(AngRollRate) || !FMath::IsFinite(AngPitchRate) || !FMath::IsFinite(AngYawRate);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    /**
     * @brief 裁剪速度和角速度，防止数值发散
     * @param MaxLinearSpeed 最大线速度 m/s
     * @param MaxAngularSpeed 最大角速度 rad/s
     */
    // 解释：这一行定义函数 `ClampVelocities`，开始实现clampvelocities的具体逻辑。
    void ClampVelocities(double MaxLinearSpeed = 100.0, double MaxAngularSpeed = 100.0)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `Vel` 执行当前步骤需要的功能逻辑。
        FVector Vel(Vx, Vy, Vz);
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (Vel.SizeSquared() > MaxLinearSpeed * MaxLinearSpeed)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行把右侧表达式的结果写入 `Vel`，完成 vel 的更新。
            Vel = Vel.GetSafeNormal() * MaxLinearSpeed;
            // 解释：这一行把右侧表达式的结果写入 `Vx`，完成 vx 的更新。
            Vx = Vel.X; Vy = Vel.Y; Vz = Vel.Z;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
        // 解释：调用 `AngVel` 执行当前步骤需要的功能逻辑。
        FVector AngVel(AngRollRate, AngPitchRate, AngYawRate);
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (AngVel.SizeSquared() > MaxAngularSpeed * MaxAngularSpeed)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行把右侧表达式的结果写入 `AngVel`，完成 angvel 的更新。
            AngVel = AngVel.GetSafeNormal() * MaxAngularSpeed;
            // 解释：这一行把右侧表达式的结果写入 `AngRollRate`，完成 angrollrate 的更新。
            AngRollRate = AngVel.X; AngPitchRate = AngVel.Y; AngYawRate = AngVel.Z;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
};

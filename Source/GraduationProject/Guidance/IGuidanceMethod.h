// 解释：使用 `#pragma once` 防止该头文件在编译过程中被重复包含。
#pragma once

// 解释：引入 Unreal 的核心基础头文件，提供常用容器、数学类型和日志宏。
#include "CoreMinimal.h"

/**
 * @brief 制导输入数据
 * 包含制导算法计算瞄准方向所需的全部信息
 */
// 解释：这一行声明 结构体 `FGuidanceInput`，用于封装fguidanceinput相关的数据与行为。
struct FGuidanceInput
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    /** @brief 转台底座位置 */
    // 解释：这一行声明成员或局部变量 `TurretPos`，用于保存turretpos。
    FVector TurretPos;
    /** @brief 枪口位置 */
    // 解释：这一行声明成员或局部变量 `MuzzlePos`，用于保存muzzlepos。
    FVector MuzzlePos;
    /** @brief 目标当前位置 */
    // 解释：这一行声明成员或局部变量 `TargetPos`，用于保存目标位置。
    FVector TargetPos;
    /** @brief 卡尔曼估计的目标速度 */
    // 解释：这一行声明成员或局部变量 `TargetVel`，用于保存目标速度。
    FVector TargetVel;
    /** @brief 卡尔曼预测的未来位置 */
    // 解释：这一行声明成员或局部变量 `PredictedPos`，用于保存predictedpos。
    FVector PredictedPos;
    /** @brief 弹丸初速 */
    // 解释：这一行声明成员或局部变量 `MuzzleSpeed`，用于保存muzzlespeed。
    float MuzzleSpeed;
    /** @brief 帧间隔 */
    // 解释：这一行声明成员或局部变量 `DeltaTime`，用于保存时间步长。
    float DeltaTime;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
};

/**
 * @brief 制导输出数据
 * 制导算法的计算结果。
 */
// 解释：这一行声明 结构体 `FGuidanceOutput`，用于封装fguidanceoutput相关的数据与行为。
struct FGuidanceOutput
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    /** @brief 俯仰瞄准角 */
    // 解释：这一行声明成员或局部变量 `Pitch`，用于保存pitch。
    float Pitch = 0.0f;
    /** @brief 偏航瞄准角 */
    // 解释：这一行声明成员或局部变量 `Yaw`，用于保存yaw。
    float Yaw = 0.0f;
    /** @brief 瞄准点世界坐标 */
    // 解释：这一行声明成员或局部变量 `AimPoint`，用于保存aimpoint。
    FVector AimPoint;
    /** @brief 预估弹丸飞行时间 */
    // 解释：这一行声明成员或局部变量 `EstFlightTime`，用于保存estflighttime。
    float EstFlightTime = 0.0f;
    /** @brief 计算结果是否有效 */
    // 解释：这一行声明成员或局部变量 `bValid`，用于保存布尔标志 valid。
    bool bValid = false;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
};

/**
 * @brief 制导方法接口
 * 输入目标信息，输出瞄准角度。
 */
// 解释：这一行声明 类 `IGuidanceMethod`，用于封装iguidancemethod相关的数据与行为。
class IGuidanceMethod
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
// 解释：从这一行开始进入 `public` 访问区，下面的成员会对外部模块公开。
public:
    // 解释：这一行把右侧表达式的结果写入 `virtual ~IGuidanceMethod()`，完成 virtualiguidancemethod 的更新。
    virtual ~IGuidanceMethod() = default;

    /** @brief 获取制导方法名称 */
    // 解释：这一行把右侧表达式的结果写入 `virtual FString GetName() const`，完成 virtualfstringgetnameconst 的更新。
    virtual FString GetName() const = 0;

    /** @brief 计算瞄准角度 */
    // 解释：这一行把右侧表达式的结果写入 `virtual FGuidanceOutput ComputeAim(const FGuidanceInput& Input)`，完成 virtualfguidanceoutputcomputeaimconstfguidanceinputinput 的更新。
    virtual FGuidanceOutput ComputeAim(const FGuidanceInput& Input) = 0;

    /** @brief 重置内部状态 */
    // 解释：这一行定义函数 `Reset`，开始实现reset的具体逻辑。
    virtual void Reset() {}
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
};

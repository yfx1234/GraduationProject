// 解释：使用 `#pragma once` 防止该头文件在编译过程中被重复包含。
#pragma once

// 解释：引入 Unreal 的核心基础头文件，提供常用容器、数学类型和日志宏。
#include "CoreMinimal.h"
// 解释：引入 `IGuidanceMethod.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "IGuidanceMethod.h"

// 解释：这一行声明 类 `ITargetPredictor`，用于封装itarget预测器相关的数据与行为。
class ITargetPredictor;

/**
 * @brief 直接瞄准算法
 * 始终将炮口/拦截器朝向目标当前位置，不做运动预测或视线变化补偿。
 */
// 解释：这一行声明 类 `FDirectAiming`，用于封装fdirectaiming相关的数据与行为。
class FDirectAiming : public IGuidanceMethod
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
// 解释：从这一行开始进入 `public` 访问区，下面的成员会对外部模块公开。
public:
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    virtual FString GetName() const override { return TEXT("direct"); }
    // 解释：调用 `ComputeAim` 执行当前步骤需要的功能逻辑。
    virtual FGuidanceOutput ComputeAim(const FGuidanceInput& Input) override;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
};

/**
 * @brief 比例导引算法
 * 根据相邻两帧视线向量变化估计 LOS 角速度，并按导航常数放大补偿量。
 */
// 解释：这一行声明 类 `FProportionalNavigation`，用于封装fproportionalnavigation相关的数据与行为。
class FProportionalNavigation : public IGuidanceMethod
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
// 解释：从这一行开始进入 `public` 访问区，下面的成员会对外部模块公开。
public:
    /**
     * @brief 构造比例导引算法
     * @param NavConstant 导航常数 N，常见取值约为 3~5
     */
    // 解释：这一行把右侧表达式的结果写入 `FProportionalNavigation(float NavConstant`，完成 fproportionalnavigationfloatnavconstant 的更新。
    FProportionalNavigation(float NavConstant = 4.0f);

    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    virtual FString GetName() const override { return TEXT("proportional"); }
    // 解释：调用 `ComputeAim` 执行当前步骤需要的功能逻辑。
    virtual FGuidanceOutput ComputeAim(const FGuidanceInput& Input) override;
    // 解释：调用 `Reset` 执行当前步骤需要的功能逻辑。
    virtual void Reset() override;

// 解释：从这一行开始进入 `private` 访问区，下面的成员只允许类内部访问。
private:
    /** @brief 比例导引导航常数 */
    // 解释：这一行声明成员或局部变量 `N`，用于保存N。
    float N;

    /** @brief 上一帧视线向量，用于估算 LOSRate */
    // 解释：这一行声明成员或局部变量 `LastLOS`，用于保存lastlos。
    FVector LastLOS;

    /** @brief 是否已经缓存过上一帧视线向量 */
    // 解释：这一行声明成员或局部变量 `bHasLastLOS`，用于保存布尔标志 haslastlos。
    bool bHasLastLOS;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
};

/**
 * @brief 预测制导算法
 * 借助目标预测器迭代估计拦截时刻的目标位置，再输出对应瞄准角。
 */
// 解释：这一行声明 类 `FPredictiveGuidance`，用于封装fpredictive制导相关的数据与行为。
class FPredictiveGuidance : public IGuidanceMethod
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
// 解释：从这一行开始进入 `public` 访问区，下面的成员会对外部模块公开。
public:
    /**
     * @brief 构造预测制导算法
     * @param Predictor 目标位置预测器
     * @param Iterations 飞行时间/目标位置的迭代次数
     */
    // 解释：这一行把右侧表达式的结果写入 `FPredictiveGuidance(ITargetPredictor* Predictor, int32 Iterations`，完成 fpredictive制导itarget预测器预测器int32iterations 的更新。
    FPredictiveGuidance(ITargetPredictor* Predictor, int32 Iterations = 3);

    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    virtual FString GetName() const override { return TEXT("predictive"); }
    // 解释：调用 `ComputeAim` 执行当前步骤需要的功能逻辑。
    virtual FGuidanceOutput ComputeAim(const FGuidanceInput& Input) override;

    /**
     * @brief 替换内部目标预测器
     * @param Predictor 新的目标预测器实例
     */
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    void SetPredictor(ITargetPredictor* Predictor) { KalmanPredictor = Predictor; }

// 解释：从这一行开始进入 `private` 访问区，下面的成员只允许类内部访问。
private:
    /** @brief 目标预测器，不拥有其生命周期 */
    // 解释：这一行声明成员或局部变量 `KalmanPredictor`，用于保存卡尔曼预测器。
    ITargetPredictor* KalmanPredictor;

    /** @brief 预测迭代次数，数值越大越接近收敛但开销也更高 */
    // 解释：这一行声明成员或局部变量 `MaxIterations`，用于保存maxiterations。
    int32 MaxIterations;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
};

// 解释：使用 `#pragma once` 防止该头文件在编译过程中被重复包含。
#pragma once

// 解释：引入 Unreal 的核心基础头文件，提供常用容器、数学类型和日志宏。
#include "CoreMinimal.h"
// 解释：引入 `NoExportTypes.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "UObject/NoExportTypes.h"

// 解释：引入 `PIDController.generated.h`，让 Unreal Header Tool 生成的反射代码在本文件中可见。
#include "PIDController.generated.h"

/**
 * e = target - current
 * I += (T/2) * (e + e_prev)
 * α = 2 / (2τ + T), β = (2τ - T) / (2τ + T)
 * de = α * (e[k] - e[k-1]) + β * de[k-1]
 * u = Kp*e + Ki*I + Kd*de
 */
// 解释：使用 `UCLASS` 宏把当前类型注册到 Unreal 反射系统中，便于蓝图、序列化和编辑器识别。
UCLASS(Blueprintable, BlueprintType)
// 解释：这一行声明 类 `UPIDController`，用于封装upidcontroller相关的数据与行为。
class GRADUATIONPROJECT_API UPIDController : public UObject
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：展开 Unreal Header Tool 生成的样板代码，这是 UObject、Actor、组件等类型正常工作的基础。
    GENERATED_BODY()

// 解释：从这一行开始进入 `public` 访问区，下面的成员会对外部模块公开。
public:
    // 解释：调用 `UPIDController` 执行当前步骤需要的功能逻辑。
    UPIDController();

    /**
     * @brief 初始化控制器参数
     * @param InputKp 比例增益 Kp
     * @param InputKi 积分增益 Ki
     * @param InputKd 微分增益 Kd
     * @param InputDiffFilterTau 微分滤波器时间常数 τ（秒）
     * @param InputOutputLimit 输出限幅值
     * @param InputTimeStep 采样时间步长（秒）
     * @param InputIntegratorMin 积分器下限
     * @param InputIntegratorMax 积分器上限
     */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "PID Controller")
    // 解释：这一行定义函数 `Initialize`，开始实现initialize的具体逻辑。
    void Initialize(double InputKp = 1.0, double InputKi = 0.1, double InputKd = 0.05, 
                   // 解释：这一行声明成员或局部变量 `InputDiffFilterTau`，用于保存inputdifffiltertau。
                   double InputDiffFilterTau = 0.05, double InputOutputLimit = 0.0, double InputTimeStep = 0.02,
                   // 解释：这一行声明成员或局部变量 `InputIntegratorMin`，用于保存inputintegratormin。
                   double InputIntegratorMin = -100.0, double InputIntegratorMax = 100.0);

    /**
     * @brief 执行一步 PID 控制更新
     * @param TargetValue 目标值
     * @param CurrentValue 当前测量值
     * @param bReset 是否在本次计算前重置控制器内部状态
     * @return 控制输出 u = Kp*e + Ki*∫e + Kd*(de/dt)
     */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "PID Controller")
    // 解释：这一行把右侧表达式的结果写入 `double Update(double TargetValue, double CurrentValue, bool bReset`，完成 doubleupdatedoubletargetvaluedoublecurrentvalueboolBreset 的更新。
    double Update(double TargetValue, double CurrentValue, bool bReset = false);

    /**
     * @brief 运行时动态修改 PID 参数
     * @param NewKp 新的比例增益
     * @param NewKi 新的积分增益
     * @param NewKd 新的微分增益
     * @param NewDiffFilterTau 新的微分滤波时间常数 τ（秒）
     * @param NewOutputLimit 新的输出限幅值
     */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "PID Controller")
    // 解释：这一行把右侧表达式的结果写入 `void SetParameters(double NewKp, double NewKi, double NewKd, double NewDiffFilterTau`，完成 voidset参数doublenewkpdoublenewkidoublenewkddoublenewdifffiltertau 的更新。
    void SetParameters(double NewKp, double NewKi, double NewKd, double NewDiffFilterTau = 0.05, double NewOutputLimit = 0.0);

    /**
     * @brief 设置采样时间步长，并重新计算微分滤波器系数
     * @param NewTimeStep 新的时间步长（秒）
     */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "PID Controller")
    // 解释：调用 `SetTimeStep` 执行当前步骤需要的功能逻辑。
    void SetTimeStep(double NewTimeStep);

    /**
     * @brief 设置积分器限幅范围，防止积分饱和
     * @param NewMin 积分器下限
     * @param NewMax 积分器上限
     */
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "PID Controller")
    // 解释：调用 `SetIntegratorLimits` 执行当前步骤需要的功能逻辑。
    void SetIntegratorLimits(double NewMin, double NewMax);

    /** @brief 重置控制器内部状态*/
    // 解释：使用 `UFUNCTION` 宏为下面的函数声明反射元数据，使其可被蓝图、RPC 或编辑器调用。
    UFUNCTION(BlueprintCallable, Category = "PID Controller")
    // 解释：调用 `Reset` 执行当前步骤需要的功能逻辑。
    void Reset();

// 解释：从这一行开始进入 `public` 访问区，下面的成员会对外部模块公开。
public:
    /** @brief 比例增益 Kp */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID Controller|Parameters")
    // 解释：这一行声明成员或局部变量 `Kp`，用于保存比例增益 Kp。
    double Kp;

    /** @brief 积分增益 Ki */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID Controller|Parameters")
    // 解释：这一行声明成员或局部变量 `Ki`，用于保存积分增益 Ki。
    double Ki;

    /** @brief 微分增益 Kd */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID Controller|Parameters")
    // 解释：这一行声明成员或局部变量 `Kd`，用于保存微分增益 Kd。
    double Kd;

    /** @brief 微分滤波器时间常数 τ（秒），值越大滤波越强，范围 [0.001, 1.0] */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID Controller|Parameters", meta = (ClampMin = "0.001", ClampMax = "1.0"))
    // 解释：这一行声明成员或局部变量 `DiffFilterTau`，用于保存微分滤波时间常数 τ。
    double DiffFilterTau;

    /** @brief 输出限幅值 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID Controller|Parameters", meta = (ClampMin = "0.0"))
    // 解释：这一行声明成员或局部变量 `OutputLimit`，用于保存输出限幅值。
    double OutputLimit;

    /** @brief 积分器下限，防止负向积分饱和 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID Controller|Integrator")
    // 解释：这一行声明成员或局部变量 `IntegratorMin`，用于保存积分器下限。
    double IntegratorMin;

    /** @brief 积分器上限，防止正向积分饱和 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID Controller|Integrator")
    // 解释：这一行声明成员或局部变量 `IntegratorMax`，用于保存积分器上限。
    double IntegratorMax;

    /** @brief 当前采样时间步长（秒） */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "PID Controller|State")
    // 解释：这一行声明成员或局部变量 `TimeStep`，用于保存采样时间步长。
    double TimeStep;

    /** @brief 控制器是否已被初始化 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "PID Controller|State")
    // 解释：这一行声明成员或局部变量 `bInitialized`，用于保存布尔标志 initialized。
    bool bInitialized;

// 解释：从这一行开始进入 `private` 访问区，下面的成员只允许类内部访问。
private:
    /** @brief 积分器累积值 ∫e·dt */
    // 解释：这一行声明成员或局部变量 `Integrator`，用于保存积分项。
    double Integrator;

    /** @brief 微分滤波器系数 α = 2 / (2τ + T) */
    // 解释：这一行声明成员或局部变量 `DiffFilterAlpha`，用于保存微分滤波系数 α。
    double DiffFilterAlpha;

    /** @brief 微分滤波器系数 β = (2τ - T) / (2τ + T) */
    // 解释：这一行声明成员或局部变量 `DiffFilterBeta`，用于保存微分滤波系数 β。
    double DiffFilterBeta;

    /** @brief 上一步的误差值 e(k-1) */
    // 解释：这一行声明成员或局部变量 `PreviousError`，用于保存上一时刻误差。
    double PreviousError;

    /** @brief 上一步的滤波后误差变化率 de(k-1) */
    // 解释：这一行声明成员或局部变量 `PreviousErrorRate`，用于保存上一时刻误差变化率。
    double PreviousErrorRate;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
};

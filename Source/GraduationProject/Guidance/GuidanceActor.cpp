// 解释：引入当前实现文件对应的头文件 `GuidanceActor.h`，使实现部分能够看到类和函数声明。
#include "GuidanceActor.h"

// 解释：引入 `JsonObject.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "Dom/JsonObject.h"
// 解释：引入 `GuidanceMethods.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "GuidanceMethods.h"
// 解释：引入 `KalmanPredictor.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "KalmanPredictor.h"
// 解释：引入 `VisualInterceptController.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "VisualInterceptController.h"
// 解释：引入 `AgentManager.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "GraduationProject/Core/Manager/AgentManager.h"
// 解释：引入 `TurretPawn.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "GraduationProject/Turret/TurretPawn.h"

// 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
namespace
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    /** @brief 布尔值转 JSON 字面量文本 */
    // 解释：这一行定义函数 `BoolLiteral`，开始实现boolliteral的具体逻辑。
    FString BoolLiteral(bool bValue)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行使用三目表达式选择返回值，在正常路径和兜底路径之间做快速切换。
        return bValue ? TEXT("true") : TEXT("false");
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    /** @brief 将字符串数组序列化为 JSON 数组文本 */
    // 解释：这一行定义函数 `StringArrayToJson`，开始实现stringarraytojson的具体逻辑。
    FString StringArrayToJson(const TArray<FString>& Values)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `FString Out`，完成 fstringout 的更新。
        FString Out = TEXT("[");
        // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
        for (int32 Index = 0; Index < Values.Num(); ++Index)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行在 `Out` 的原有基础上继续累加新量，用于持续更新 out。
            Out += FString::Printf(TEXT("\"%s\""), *Values[Index].ReplaceCharWithEscapedChar());
            // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
            if (Index + 1 < Values.Num())
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            {
                // 解释：这一行在 `Out` 的原有基础上继续累加新量，用于持续更新 out。
                Out += TEXT(",");
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            }
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
        // 解释：这一行在 `Out` 的原有基础上继续累加新量，用于持续更新 out。
        Out += TEXT("]");
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return Out;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    /**
     * @brief 规范化无人机自动拦截方法名称
     * @param Method 原始方法名
     * @return 归一化后的方法名
     *
     * 目前统一收敛为：
     * - `pure_pursuit`
     * - `proportional_nav`
     * - `smc`
     */
    // 解释：这一行定义函数 `NormalizeInterceptMethodName`，开始实现normalize拦截methodname的具体逻辑。
    FString NormalizeInterceptMethodName(const FString& Method)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行声明成员或局部变量 `Normalized`，用于保存normalized。
        FString Normalized = Method;
        // 解释：调用 `TrimStartAndEndInline` 执行当前步骤需要的功能逻辑。
        Normalized.TrimStartAndEndInline();
        // 解释：调用 `ToLowerInline` 执行当前步骤需要的功能逻辑。
        Normalized.ToLowerInline();
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (Normalized == TEXT("pn") || Normalized == TEXT("proportional") || Normalized == TEXT("proportional_navigation"))
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return TEXT("proportional_nav");
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (Normalized == TEXT("smc") || Normalized == TEXT("sliding_mode") || Normalized == TEXT("slidingmode"))
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return TEXT("smc");
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
        // 解释：这一行使用三目表达式选择返回值，在正常路径和兜底路径之间做快速切换。
        return (Normalized == TEXT("pure_pursuit")) ? Normalized : TEXT("pure_pursuit");
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    /**
     * @brief 估计炮塔炮口世界坐标
     * @param Turret 炮塔实例
     * @return 炮口位置；若炮管网格不存在，则退化为 Actor 位置
     */
    // 解释：这一行定义函数 `GetTurretMuzzlePosition`，开始实现getturretmuzzleposition的具体逻辑。
    FVector GetTurretMuzzlePosition(const ATurretPawn* Turret)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (!Turret)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return FVector::ZeroVector;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (!Turret->GunMesh)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return Turret->GetActorLocation();
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return Turret->GunMesh->GetComponentLocation() +
            // 解释：调用 `GetComponentRotation` 执行当前步骤需要的功能逻辑。
            Turret->GunMesh->GetComponentRotation().RotateVector(Turret->MuzzleOffset);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    /** @brief 当字符串非空时写入 JSON 字段 */
    // 解释：这一行定义函数 `SetStringIfNotEmpty`，开始实现setstringifnotempty的具体逻辑。
    void SetStringIfNotEmpty(const TSharedPtr<FJsonObject>& Object, const TCHAR* FieldName, const FString& Value)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (Object.IsValid() && !Value.IsEmpty())
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：调用 `SetStringField` 执行当前步骤需要的功能逻辑。
            Object->SetStringField(FieldName, Value);
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    /** @brief 当数值不是哨兵值时写入 JSON 字段 */
    // 解释：这一行定义函数 `SetNumberIfValid`，开始实现setnumberifvalid的具体逻辑。
    void SetNumberIfValid(const TSharedPtr<FJsonObject>& Object, const TCHAR* FieldName, float Value, float InvalidSentinel = -1.0f)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (Object.IsValid() && !FMath::IsNearlyEqual(Value, InvalidSentinel))
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：调用 `SetNumberField` 执行当前步骤需要的功能逻辑。
            Object->SetNumberField(FieldName, Value);
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    /** @brief 当整型参数有效时写入 JSON 字段 */
    // 解释：这一行定义函数 `SetIntIfValid`，开始实现setintifvalid的具体逻辑。
    void SetIntIfValid(const TSharedPtr<FJsonObject>& Object, const TCHAR* FieldName, int32 Value)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (Object.IsValid() && Value >= 0)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：调用 `SetNumberField` 执行当前步骤需要的功能逻辑。
            Object->SetNumberField(FieldName, Value);
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 构造 GuidanceActor，并将其设为隐藏的后台协调 Actor */
// 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
AGuidanceActor::AGuidanceActor()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `PrimaryActorTick.bCanEverTick`，完成 布尔标志 canevertick 的更新。
    PrimaryActorTick.bCanEverTick = false;
    // 解释：调用 `SetActorHiddenInGame` 执行当前步骤需要的功能逻辑。
    SetActorHiddenInGame(true);
    // 解释：调用 `SetActorEnableCollision` 执行当前步骤需要的功能逻辑。
    SetActorEnableCollision(false);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 启动时注册到 AgentManager
 *
 * 如果场景中已有指向本对象的注册项，则优先复用其 ID，
 * 避免同一个 GuidanceActor 被重复注册为多个名称。
 */
// 解释：这一行定义函数 `BeginPlay`，开始实现beginplay的具体逻辑。
void AGuidanceActor::BeginPlay()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：调用 `BeginPlay` 执行当前步骤需要的功能逻辑。
    Super::BeginPlay();

    // 解释：这一行把右侧表达式的结果写入 `UAgentManager* Manager`，完成 uagent管理器管理器 的更新。
    UAgentManager* Manager = UAgentManager::GetInstance();
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (Manager)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行声明成员或局部变量 `ExistingId`，用于保存existingid。
        FString ExistingId;
        // 解释：调用 `GetAllAgentIds` 执行当前步骤需要的功能逻辑。
        const TArray<FString> ExistingIds = Manager->GetAllAgentIds();
        // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
        for (const FString& Id : ExistingIds)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
            if (Manager->GetAgent(Id) == this)
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            {
                // 解释：这一行把右侧表达式的结果写入 `ExistingId`，完成 existingid 的更新。
                ExistingId = Id;
                // 解释：这一行立即跳出当前循环或 `switch` 分支，避免继续执行后续分支。
                break;
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            }
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (!ExistingId.IsEmpty() && ExistingId != GuidanceId)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行把右侧表达式的结果写入 `GuidanceId`，完成 制导id 的更新。
            GuidanceId = ExistingId;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (Manager->GetAgent(GuidanceId) != this)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：调用 `RegisterAgent` 执行当前步骤需要的功能逻辑。
            Manager->RegisterAgent(GuidanceId, this);
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 结束时释放内部资源并注销自身
 * @param EndPlayReason Actor 结束原因
 */
// 解释：这一行定义函数 `EndPlay`，开始实现endplay的具体逻辑。
void AGuidanceActor::EndPlay(const EEndPlayReason::Type EndPlayReason)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (CurrentMethod)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行声明成员或局部变量 `CurrentMethod`，用于保存currentmethod。
        delete CurrentMethod;
        // 解释：这一行把右侧表达式的结果写入 `CurrentMethod`，完成 currentmethod 的更新。
        CurrentMethod = nullptr;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行把右侧表达式的结果写入 `UAgentManager* Manager`，完成 uagent管理器管理器 的更新。
    UAgentManager* Manager = UAgentManager::GetInstance();
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (Manager && !GuidanceId.IsEmpty() && Manager->GetAgent(GuidanceId) == this)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `UnregisterAgent` 执行当前步骤需要的功能逻辑。
        Manager->UnregisterAgent(GuidanceId);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：调用 `EndPlay` 执行当前步骤需要的功能逻辑。
    Super::EndPlay(EndPlayReason);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 惰性初始化核心子模块
 *
 * - `Predictor`：目标状态估计与延迟补偿；
 * - `VisualInterceptController`：视觉拦截闭环控制；
 * - `CurrentMethod`：默认采用预测制导算法。
 */
// 解释：这一行定义函数 `EnsureInitialized`，开始实现ensureinitialized的具体逻辑。
void AGuidanceActor::EnsureInitialized()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!Predictor)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `Predictor`，完成 预测器 的更新。
        Predictor = NewObject<UKalmanPredictor>(this);
        // 解释：调用 `Initialize` 执行当前步骤需要的功能逻辑。
        Predictor->Initialize(100.0f, 0.01f);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!VisualInterceptController)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `VisualInterceptController`，完成 视觉拦截控制器 的更新。
        VisualInterceptController = NewObject<UVisualInterceptController>(this);
        // 解释：调用 `EnsureInitialized` 执行当前步骤需要的功能逻辑。
        VisualInterceptController->EnsureInitialized();
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!CurrentMethod)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `CurrentMethod`，完成 currentmethod 的更新。
        CurrentMethod = new FPredictiveGuidance(Predictor, 3);
        // 解释：这一行把右侧表达式的结果写入 `CurrentMethodName`，完成 currentmethodname 的更新。
        CurrentMethodName = TEXT("predictive");
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 按任务角色查找首个匹配的无人机
 * @param Manager AgentManager 实例
 * @param DesiredRole 目标角色
 * @param ExcludeId 需要跳过的 ID
 * @return 找到的无人机指针，失败返回 nullptr
 */
// 解释：这一行定义函数 `FindDroneByRole`，开始实现find无人机byrole的具体逻辑。
ADronePawn* AGuidanceActor::FindDroneByRole(UAgentManager* Manager, EDroneMissionRole DesiredRole, const FString& ExcludeId) const
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!Manager)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return nullptr;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：调用 `GetAllAgentIds` 执行当前步骤需要的功能逻辑。
    const TArray<FString> AgentIds = Manager->GetAllAgentIds();
    // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
    for (const FString& AgentId : AgentIds)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (!ExcludeId.IsEmpty() && AgentId == ExcludeId)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行跳过本轮循环剩余语句，直接进入下一轮迭代。
            continue;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行把右侧表达式的结果写入 `ADronePawn* Drone`，完成 adronePawn无人机 的更新。
        ADronePawn* Drone = Cast<ADronePawn>(Manager->GetAgent(AgentId));
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (Drone && Drone->MissionRole == DesiredRole)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return Drone;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return nullptr;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 生成统一错误返回 JSON */
// 解释：这一行定义函数 `MakeError`，开始实现makeerror的具体逻辑。
FString AGuidanceActor::MakeError(const FString& Msg) const
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return FString::Printf(TEXT("{\"status\":\"error\",\"message\":\"%s\"}"), *Msg.ReplaceCharWithEscapedChar());
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 生成统一成功返回 JSON */
// 解释：这一行定义函数 `MakeOk`，开始实现makeok的具体逻辑。
FString AGuidanceActor::MakeOk(const FString& Msg) const
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return FString::Printf(TEXT("{\"status\":\"ok\",\"message\":\"%s\"}"), *Msg.ReplaceCharWithEscapedChar());
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 切换常规瞄准/制导算法
 * @param Method 算法名称
 * @param NavConstant 比例导引导航常数
 * @param Iterations 预测制导迭代次数
 */
// 解释：这一行定义函数 `SetMethod`，开始实现setmethod的具体逻辑。
FString AGuidanceActor::SetMethod(FString Method, float NavConstant, int32 Iterations)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：调用 `EnsureInitialized` 执行当前步骤需要的功能逻辑。
    EnsureInitialized();

    // 解释：调用 `TrimStartAndEndInline` 执行当前步骤需要的功能逻辑。
    Method.TrimStartAndEndInline();
    // 解释：调用 `ToLowerInline` 执行当前步骤需要的功能逻辑。
    Method.ToLowerInline();

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (CurrentMethod)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行声明成员或局部变量 `CurrentMethod`，用于保存currentmethod。
        delete CurrentMethod;
        // 解释：这一行把右侧表达式的结果写入 `CurrentMethod`，完成 currentmethod 的更新。
        CurrentMethod = nullptr;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (Method == TEXT("direct"))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `CurrentMethod`，完成 currentmethod 的更新。
        CurrentMethod = new FDirectAiming();
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行声明补充分支，当上一个条件不成立且当前条件成立时执行。
    else if (Method == TEXT("proportional"))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `CurrentMethod`，完成 currentmethod 的更新。
        CurrentMethod = new FProportionalNavigation(NavConstant <= 0.0f ? 4.0f : NavConstant);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行声明补充分支，当上一个条件不成立且当前条件成立时执行。
    else if (Method == TEXT("predictive"))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `CurrentMethod`，完成 currentmethod 的更新。
        CurrentMethod = new FPredictiveGuidance(Predictor, Iterations > 0 ? Iterations : 3);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行给出兜底分支，当上面的条件都不满足时执行。
    else
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return MakeError(FString::Printf(TEXT("Unknown method: %s"), *Method));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行把右侧表达式的结果写入 `CurrentMethodName`，完成 currentmethodname 的更新。
    CurrentMethodName = Method;
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return MakeOk(FString::Printf(TEXT("Method set to %s"), *Method));
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 更新无人机自动拦截的默认方法与参数
 * @param Method 拦截方法名
 * @param Speed 拦截速度上限（m/s）
 * @param NavGain 导航增益
 * @param LeadTime 目标前置预测时间（s）
 * @param CaptureRadiusValue 捕获半径（m）
 */
// 解释：这一行定义函数 `SetInterceptMethod`，开始实现set拦截method的具体逻辑。
FString AGuidanceActor::SetInterceptMethod(FString Method, float Speed, float NavGain, float LeadTime, float CaptureRadiusValue)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：调用 `EnsureInitialized` 执行当前步骤需要的功能逻辑。
    EnsureInitialized();

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!Method.IsEmpty())
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `CurrentInterceptMethod`，完成 current拦截method 的更新。
        CurrentInterceptMethod = NormalizeInterceptMethodName(Method);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (Speed > 0.0f)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `InterceptorSpeed`，完成 interceptorspeed 的更新。
        InterceptorSpeed = Speed;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (NavGain > 0.0f)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `InterceptNavGain`，完成 拦截navgain 的更新。
        InterceptNavGain = NavGain;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (LeadTime >= 0.0f)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `InterceptLeadTime`，完成 拦截leadtime 的更新。
        InterceptLeadTime = LeadTime;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (CaptureRadiusValue > 0.0f)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `CaptureRadius`，完成 采集radius 的更新。
        CaptureRadius = CaptureRadiusValue;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return FString::Printf(
        // 解释：这一行位于构造函数初始化列表中，把 `TEXT` 直接初始化为 `"{\"status\":\"ok\",\"intercept_method\":\"%s\",\"speed\":%.2f,\"nav_gain\":%.2f,\"lead_time\":%.2f,\"capture_radius\":%.2f}"`，减少进入函数体后的额外赋值开销。
        TEXT("{\"status\":\"ok\",\"intercept_method\":\"%s\",\"speed\":%.2f,\"nav_gain\":%.2f,\"lead_time\":%.2f,\"capture_radius\":%.2f}"),
        *CurrentInterceptMethod,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        InterceptorSpeed,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        InterceptNavGain,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        InterceptLeadTime,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        CaptureRadius);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 统计当前场景中可参与拦截任务的无人机
 * @return 目标机与拦截机 ID 列表 JSON
 */
// 解释：这一行定义函数 `ListInterceptAgents`，开始实现列表拦截agents的具体逻辑。
FString AGuidanceActor::ListInterceptAgents()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：调用 `EnsureInitialized` 执行当前步骤需要的功能逻辑。
    EnsureInitialized();

    // 解释：这一行把右侧表达式的结果写入 `UAgentManager* Manager`，完成 uagent管理器管理器 的更新。
    UAgentManager* Manager = UAgentManager::GetInstance();
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!Manager)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return MakeError(TEXT("Agent manager unavailable"));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行声明成员或局部变量 `Targets`，用于保存targets。
    TArray<FString> Targets;
    // 解释：这一行声明成员或局部变量 `Interceptors`，用于保存interceptors。
    TArray<FString> Interceptors;

    // 解释：调用 `GetAllAgentIds` 执行当前步骤需要的功能逻辑。
    const TArray<FString> AgentIds = Manager->GetAllAgentIds();
    // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
    for (const FString& AgentId : AgentIds)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `ADronePawn* Drone`，完成 adronePawn无人机 的更新。
        ADronePawn* Drone = Cast<ADronePawn>(Manager->GetAgent(AgentId));
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (!Drone)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行跳过本轮循环剩余语句，直接进入下一轮迭代。
            continue;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (Drone->MissionRole == EDroneMissionRole::Target)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：调用 `Add` 执行当前步骤需要的功能逻辑。
            Targets.Add(Drone->DroneId);
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
        // 解释：这一行声明补充分支，当上一个条件不成立且当前条件成立时执行。
        else if (Drone->MissionRole == EDroneMissionRole::Interceptor)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：调用 `Add` 执行当前步骤需要的功能逻辑。
            Interceptors.Add(Drone->DroneId);
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return FString::Printf(
        // 解释：这一行位于构造函数初始化列表中，把 `TEXT` 直接初始化为 `"{\"status\":\"ok\",\"targets\":%s,\"interceptors\":%s,\"target_count\":%d,\"interceptor_count\":%d}"`，减少进入函数体后的额外赋值开销。
        TEXT("{\"status\":\"ok\",\"targets\":%s,\"interceptors\":%s,\"target_count\":%d,\"interceptor_count\":%d}"),
        *StringArrayToJson(Targets),
        *StringArrayToJson(Interceptors),
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Targets.Num(),
        // 解释：调用 `Num` 执行当前步骤需要的功能逻辑。
        Interceptors.Num());
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 计算一步无人机自动拦截控制并直接写入拦截机
 * @param InterceptorId 拦截机 ID
 * @param TargetId 目标机 ID
 * @param Method 临时覆盖的拦截方法
 * @param Speed 临时覆盖的速度上限（m/s）
 * @param NavGain 临时覆盖的导航增益 N
 * @param LeadTime 临时覆盖的预测时间（s）
 * @param CaptureRadiusValue 临时覆盖的捕获半径（m）
 * @param bStopOnCapture 捕获后是否悬停
 * @return 当前一步拦截状态 JSON
 *
 * 当前实现采用两类速度指令模型：
 * 1. 纯追踪/前置法：
 *    - $p_{pred} = p_t + v_t \cdot t_{lead}$
 *    - $v_{cmd} = speed \cdot \mathrm{normalize}(p_{pred} - p_i)$
 * 2. 简化比例导航：
 *    - $v_{rel} = v_t - v_i$
 *    - $LOS = \mathrm{normalize}(p_t - p_i)$
 *    - $v_{rel}^{lat} = v_{rel} - (v_{rel}\cdot LOS)LOS$
 *    - $v_{cmd} = v_t + N \cdot v_{rel}^{lat}$
 *
 * 捕获判据为：
 * $\|p_t - p_i\| \le R_{capture}$。
 */
// 解释：这一行定义函数 `AutoIntercept`，开始实现auto拦截的具体逻辑。
FString AGuidanceActor::AutoIntercept(
    // 解释：这一行继续展开 `AutoIntercept` 的参数列表，声明参数 `InterceptorId` 用于传入interceptorid。
    FString InterceptorId,
    // 解释：这一行继续展开 `AutoIntercept` 的参数列表，声明参数 `TargetId` 用于传入targetid。
    FString TargetId,
    // 解释：这一行继续展开 `AutoIntercept` 的参数列表，声明参数 `Method` 用于传入method。
    FString Method,
    // 解释：这一行继续展开 `AutoIntercept` 的参数列表，声明参数 `Speed` 用于传入speed。
    float Speed,
    // 解释：这一行继续展开 `AutoIntercept` 的参数列表，声明参数 `NavGain` 用于传入navgain。
    float NavGain,
    // 解释：这一行继续展开 `AutoIntercept` 的参数列表，声明参数 `LeadTime` 用于传入leadtime。
    float LeadTime,
    // 解释：这一行继续展开 `AutoIntercept` 的参数列表，声明参数 `CaptureRadiusValue` 用于传入采集radiusvalue。
    float CaptureRadiusValue,
    // 解释：这一行收束函数 `AutoIntercept` 的签名，后面会进入实现体或以分号结束声明。
    bool bStopOnCapture)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：调用 `EnsureInitialized` 执行当前步骤需要的功能逻辑。
    EnsureInitialized();

    // 解释：这一行把右侧表达式的结果写入 `UAgentManager* Manager`，完成 uagent管理器管理器 的更新。
    UAgentManager* Manager = UAgentManager::GetInstance();
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!Manager)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return MakeError(TEXT("Agent manager unavailable"));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行声明成员或局部变量 `TargetDrone`，用于保存target无人机。
    ADronePawn* TargetDrone = nullptr;
    // 解释：这一行声明成员或局部变量 `InterceptorDrone`，用于保存interceptor无人机。
    ADronePawn* InterceptorDrone = nullptr;

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!TargetId.IsEmpty())
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `TargetDrone`，完成 target无人机 的更新。
        TargetDrone = Cast<ADronePawn>(Manager->GetAgent(TargetId));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行给出兜底分支，当上面的条件都不满足时执行。
    else
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `TargetDrone`，完成 target无人机 的更新。
        TargetDrone = FindDroneByRole(Manager, EDroneMissionRole::Target);
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (TargetDrone)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行把右侧表达式的结果写入 `TargetId`，完成 targetid 的更新。
            TargetId = TargetDrone->DroneId;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!InterceptorId.IsEmpty())
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `InterceptorDrone`，完成 interceptor无人机 的更新。
        InterceptorDrone = Cast<ADronePawn>(Manager->GetAgent(InterceptorId));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行给出兜底分支，当上面的条件都不满足时执行。
    else
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `const FString ExcludeTarget`，完成 constfstringexcludetarget 的更新。
        const FString ExcludeTarget = TargetDrone ? TargetDrone->DroneId : TEXT("");
        // 解释：这一行把右侧表达式的结果写入 `InterceptorDrone`，完成 interceptor无人机 的更新。
        InterceptorDrone = FindDroneByRole(Manager, EDroneMissionRole::Interceptor, ExcludeTarget);
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (InterceptorDrone)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行把右侧表达式的结果写入 `InterceptorId`，完成 interceptorid 的更新。
            InterceptorId = InterceptorDrone->DroneId;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!TargetDrone)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return MakeError(TEXT("Target drone not found. Provide target_id or set MissionRole=Target."));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!InterceptorDrone)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return MakeError(TEXT("Interceptor drone not found. Provide interceptor_id or set MissionRole=Interceptor."));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (TargetDrone == InterceptorDrone)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return MakeError(TEXT("target_id and interceptor_id must be different"));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行声明成员或局部变量 `EffectiveMethod`，用于保存effectivemethod。
    FString EffectiveMethod = CurrentInterceptMethod;
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!Method.IsEmpty())
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `EffectiveMethod`，完成 effectivemethod 的更新。
        EffectiveMethod = NormalizeInterceptMethodName(Method);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行把右侧表达式的结果写入 `const float EffectiveSpeed`，完成 constfloateffectivespeed 的更新。
    const float EffectiveSpeed = (Speed > 0.0f) ? Speed : InterceptorSpeed;
    // 解释：这一行把右侧表达式的结果写入 `const float EffectiveNavGain`，完成 constfloateffectivenavgain 的更新。
    const float EffectiveNavGain = (NavGain > 0.0f) ? NavGain : InterceptNavGain;
    // 解释：这一行把右侧表达式的结果写入 `const float EffectiveLeadTime`，完成 constfloateffectiveleadtime 的更新。
    const float EffectiveLeadTime = (LeadTime >= 0.0f) ? LeadTime : InterceptLeadTime;
    // 解释：这一行把右侧表达式的结果写入 `const float EffectiveCaptureRadius`，完成 constfloateffective采集radius 的更新。
    const float EffectiveCaptureRadius = (CaptureRadiusValue > 0.0f) ? CaptureRadiusValue : CaptureRadius;

    // 解释：这一行把右侧表达式的结果写入 `const FVector InterceptorPos`，完成 constfvectorinterceptorpos 的更新。
    const FVector InterceptorPos = InterceptorDrone->GetCurrentPosition();
    // 解释：这一行把右侧表达式的结果写入 `const FVector InterceptorVel`，完成 constfvectorinterceptorvel 的更新。
    const FVector InterceptorVel = InterceptorDrone->GetCurrentVelocity();
    // 解释：这一行把右侧表达式的结果写入 `const FVector TargetPos`，完成 constfvectortargetpos 的更新。
    const FVector TargetPos = TargetDrone->GetCurrentPosition();
    // 解释：这一行把右侧表达式的结果写入 `const FVector TargetVel`，完成 constfvectortargetvel 的更新。
    const FVector TargetVel = TargetDrone->GetCurrentVelocity();

    // 解释：这一行声明成员或局部变量 `RelativePos`，用于保存relativepos。
    const FVector RelativePos = TargetPos - InterceptorPos;
    // 解释：这一行声明成员或局部变量 `RelativeVel`，用于保存relativevel。
    const FVector RelativeVel = TargetVel - InterceptorVel;
    // 解释：这一行把向量模长写入 `const float Distance`，用于表示距离、速度大小或不确定度。
    const float Distance = RelativePos.Size();
    // 解释：这一行把右侧表达式的结果写入 `const FVector LOS`，完成 constfvectorlos 的更新。
    const FVector LOS = RelativePos.GetSafeNormal();

    // 闭合速度采用视线方向上的相对速度投影：closing = -v_rel·LOS。
    // 解释：这一行利用向量点乘结果更新 `const float ClosingSpeed`，提取某一方向上的投影量。
    const float ClosingSpeed = -FVector::DotProduct(RelativeVel, LOS);

    // 解释：这一行声明成员或局部变量 `CommandVelocity`，用于保存命令velocity。
    FVector CommandVelocity = FVector::ZeroVector;
    // 解释：这一行声明成员或局部变量 `bCaptured`，用于保存布尔标志 captured。
    const bool bCaptured = Distance <= EffectiveCaptureRadius;
    // 解释：这一行把右侧表达式的结果写入 `bool bValidCmd`，完成 boolBvalidcmd 的更新。
    bool bValidCmd = !RelativePos.IsNearlyZero();

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (bCaptured)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (bStopOnCapture)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：调用 `Hover` 执行当前步骤需要的功能逻辑。
            InterceptorDrone->Hover();
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行给出兜底分支，当上面的条件都不满足时执行。
    else
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (EffectiveMethod == TEXT("proportional_nav"))
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行利用向量点乘结果更新 `const FVector LateralRelVel`，提取某一方向上的投影量。
            const FVector LateralRelVel = RelativeVel - FVector::DotProduct(RelativeVel, LOS) * LOS;
            // 解释：这一行把右侧表达式的结果写入 `CommandVelocity`，完成 命令velocity 的更新。
            CommandVelocity = TargetVel + EffectiveNavGain * LateralRelVel;
            // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
            if (CommandVelocity.IsNearlyZero())
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            {
                // 解释：这一行把右侧表达式的结果写入 `CommandVelocity`，完成 命令velocity 的更新。
                CommandVelocity = LOS * EffectiveSpeed;
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            }
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
        // 解释：这一行给出兜底分支，当上面的条件都不满足时执行。
        else
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行声明成员或局部变量 `PredictedTarget`，用于保存predictedtarget。
            const FVector PredictedTarget = TargetPos + TargetVel * EffectiveLeadTime;
            // 解释：这一行把右侧表达式的结果写入 `CommandVelocity`，完成 命令velocity 的更新。
            CommandVelocity = (PredictedTarget - InterceptorPos).GetSafeNormal() * EffectiveSpeed;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (!CommandVelocity.IsNearlyZero())
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行把右侧表达式的结果写入 `CommandVelocity`，完成 命令velocity 的更新。
            CommandVelocity = CommandVelocity.GetClampedToMaxSize(EffectiveSpeed);
            // 解释：调用 `SetHeadingControl` 执行当前步骤需要的功能逻辑。
            InterceptorDrone->SetHeadingControl(EDroneYawMode::Auto, EDroneDrivetrainMode::ForwardOnly);
            // 解释：调用 `SetTargetVelocity` 执行当前步骤需要的功能逻辑。
            InterceptorDrone->SetTargetVelocity(CommandVelocity);
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
        // 解释：这一行给出兜底分支，当上面的条件都不满足时执行。
        else
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行把右侧表达式的结果写入 `bValidCmd`，完成 布尔标志 validcmd 的更新。
            bValidCmd = false;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行把右侧表达式的结果写入 `LastInterceptorId`，完成 lastinterceptorid 的更新。
    LastInterceptorId = InterceptorDrone->DroneId;
    // 解释：这一行把右侧表达式的结果写入 `LastTargetId`，完成 lasttargetid 的更新。
    LastTargetId = TargetDrone->DroneId;
    // 解释：这一行把右侧表达式的结果写入 `LastDistanceToTarget`，完成 lastdistancetotarget 的更新。
    LastDistanceToTarget = Distance;
    // 解释：这一行把右侧表达式的结果写入 `LastClosingSpeed`，完成 lastclosingspeed 的更新。
    LastClosingSpeed = ClosingSpeed;
    // 解释：这一行把右侧表达式的结果写入 `LastInterceptorCmdVel`，完成 lastinterceptorcmdvel 的更新。
    LastInterceptorCmdVel = CommandVelocity;
    // 解释：这一行把右侧表达式的结果写入 `bLastInterceptValid`，完成 布尔标志 last拦截valid 的更新。
    bLastInterceptValid = bValidCmd;
    // 解释：这一行把右侧表达式的结果写入 `bLastCaptured`，完成 布尔标志 lastcaptured 的更新。
    bLastCaptured = bCaptured;

    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return FString::Printf(
        // 解释：这一行位于构造函数初始化列表中，把 `TEXT` 直接初始化为 `"{\"status\":\"ok\",\"mode\":\"auto_intercept\",\"method\":\"%s\",\"target_id\":\"%s\",\"interceptor_id\":\"%s\",\"distance\":%.3f,\"closing_speed\":%.3f,\"captured\":%s,\"valid\":%s,\"cmd_velocity\":[%.3f,%.3f,%.3f]}"`，减少进入函数体后的额外赋值开销。
        TEXT("{\"status\":\"ok\",\"mode\":\"auto_intercept\",\"method\":\"%s\",\"target_id\":\"%s\",\"interceptor_id\":\"%s\",\"distance\":%.3f,\"closing_speed\":%.3f,\"captured\":%s,\"valid\":%s,\"cmd_velocity\":[%.3f,%.3f,%.3f]}"),
        *EffectiveMethod,
        *LastTargetId,
        *LastInterceptorId,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        LastDistanceToTarget,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        LastClosingSpeed,
        *BoolLiteral(bCaptured),
        *BoolLiteral(bValidCmd),
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        LastInterceptorCmdVel.X,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        LastInterceptorCmdVel.Y,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        LastInterceptorCmdVel.Z);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 向 Kalman 预测器输入一帧目标位置观测
 * @param X 观测位置 X
 * @param Y 观测位置 Y
 * @param Z 观测位置 Z
 * @param Dt 采样时间间隔（s）
 * @return 当前估计状态 JSON
 */
// 解释：这一行定义函数 `UpdateTarget`，开始实现updatetarget的具体逻辑。
FString AGuidanceActor::UpdateTarget(float X, float Y, float Z, float Dt)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：调用 `EnsureInitialized` 执行当前步骤需要的功能逻辑。
    EnsureInitialized();

    // 解释：调用 `Update` 执行当前步骤需要的功能逻辑。
    Predictor->Update(FVector(X, Y, Z), Dt);
    // 解释：这一行把右侧表达式的结果写入 `const FVector EstPos`，完成 constfvectorestpos 的更新。
    const FVector EstPos = Predictor->GetEstimatedPosition();
    // 解释：这一行把右侧表达式的结果写入 `const FVector EstVel`，完成 constfvectorestvel 的更新。
    const FVector EstVel = Predictor->GetEstimatedVelocity();
    // 解释：这一行把右侧表达式的结果写入 `const FVector EstAcc`，完成 constfvectorestacc 的更新。
    const FVector EstAcc = Predictor->GetEstimatedAcceleration();
    // 解释：这一行把右侧表达式的结果写入 `const float AdaptiveQ`，完成 constfloatadaptiveQ 的更新。
    const float AdaptiveQ = Predictor->GetAdaptiveProcessNoise();

    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return FString::Printf(
        // 解释：这一行位于构造函数初始化列表中，把 `TEXT` 直接初始化为 `"{\"status\":\"ok\",\"est_pos\":[%.2f,%.2f,%.2f],\"est_vel\":[%.2f,%.2f,%.2f],\"est_acc\":[%.2f,%.2f,%.2f],\"adaptive_q\":%.4f}"`，减少进入函数体后的额外赋值开销。
        TEXT("{\"status\":\"ok\",\"est_pos\":[%.2f,%.2f,%.2f],\"est_vel\":[%.2f,%.2f,%.2f],\"est_acc\":[%.2f,%.2f,%.2f],\"adaptive_q\":%.4f}"),
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        EstPos.X,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        EstPos.Y,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        EstPos.Z,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        EstVel.X,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        EstVel.Y,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        EstVel.Z,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        EstAcc.X,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        EstAcc.Y,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        EstAcc.Z,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        AdaptiveQ);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 基于当前目标预测状态计算炮塔瞄准解
 * @param TurretId 炮塔 ID
 * @param MuzzleSpeed 弹丸初速度（m/s）
 * @return 瞄准角与预计飞行时间 JSON
 */
// 解释：这一行定义函数 `ComputeAim`，开始实现computeaim的具体逻辑。
FString AGuidanceActor::ComputeAim(FString TurretId, float MuzzleSpeed)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：调用 `EnsureInitialized` 执行当前步骤需要的功能逻辑。
    EnsureInitialized();

    // 解释：这一行把右侧表达式的结果写入 `UAgentManager* Manager`，完成 uagent管理器管理器 的更新。
    UAgentManager* Manager = UAgentManager::GetInstance();
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!Manager)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return MakeError(TEXT("Agent manager unavailable"));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行把右侧表达式的结果写入 `const FString EffectiveTurretId`，完成 constfstringeffectiveturretid 的更新。
    const FString EffectiveTurretId = TurretId.IsEmpty() ? TEXT("turret_0") : TurretId;
    // 解释：这一行把右侧表达式的结果写入 `ATurretPawn* Turret`，完成 aturretPawnturret 的更新。
    ATurretPawn* Turret = Cast<ATurretPawn>(Manager->GetAgent(EffectiveTurretId));
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!Turret)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return MakeError(FString::Printf(TEXT("Turret '%s' not found"), *EffectiveTurretId));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行声明成员或局部变量 `Input`，用于保存input。
    FGuidanceInput Input;
    // 解释：这一行把右侧表达式的结果写入 `Input.TurretPos`，完成 turretpos 的更新。
    Input.TurretPos = Turret->GetActorLocation();
    // 解释：这一行把右侧表达式的结果写入 `Input.MuzzlePos`，完成 muzzlepos 的更新。
    Input.MuzzlePos = GetTurretMuzzlePosition(Turret);
    // 解释：这一行把右侧表达式的结果写入 `Input.TargetPos`，完成 目标位置 的更新。
    Input.TargetPos = Predictor->GetEstimatedPosition();
    // 解释：这一行把右侧表达式的结果写入 `Input.TargetVel`，完成 目标速度 的更新。
    Input.TargetVel = Predictor->GetEstimatedVelocity();
    // 解释：这一行把右侧表达式的结果写入 `Input.PredictedPos`，完成 predictedpos 的更新。
    Input.PredictedPos = Predictor->PredictPosition(0.5f);
    // 解释：这一行把右侧表达式的结果写入 `Input.MuzzleSpeed`，完成 muzzlespeed 的更新。
    Input.MuzzleSpeed = MuzzleSpeed;
    // 解释：这一行把右侧表达式的结果写入 `Input.DeltaTime`，完成 时间步长 的更新。
    Input.DeltaTime = 0.1f;

    // 解释：这一行把右侧表达式的结果写入 `const FGuidanceOutput Output`，完成 constfguidanceoutputoutput 的更新。
    const FGuidanceOutput Output = CurrentMethod->ComputeAim(Input);
    // 解释：这一行把右侧表达式的结果写入 `LastPitch`，完成 lastpitch 的更新。
    LastPitch = Output.Pitch;
    // 解释：这一行把右侧表达式的结果写入 `LastYaw`，完成 lastyaw 的更新。
    LastYaw = Output.Yaw;
    // 解释：这一行把右侧表达式的结果写入 `LastAimPoint`，完成 lastaimpoint 的更新。
    LastAimPoint = Output.AimPoint;
    // 解释：这一行把右侧表达式的结果写入 `LastFlightTime`，完成 lastflighttime 的更新。
    LastFlightTime = Output.EstFlightTime;

    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return FString::Printf(
        // 解释：这一行位于构造函数初始化列表中，把 `TEXT` 直接初始化为 `"{\"status\":\"ok\",\"pitch\":%.2f,\"yaw\":%.2f,\"aim_point\":[%.1f,%.1f,%.1f],\"flight_time\":%.4f,\"method\":\"%s\"}"`，减少进入函数体后的额外赋值开销。
        TEXT("{\"status\":\"ok\",\"pitch\":%.2f,\"yaw\":%.2f,\"aim_point\":[%.1f,%.1f,%.1f],\"flight_time\":%.4f,\"method\":\"%s\"}"),
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Output.Pitch,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Output.Yaw,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Output.AimPoint.X,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Output.AimPoint.Y,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Output.AimPoint.Z,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Output.EstFlightTime,
        *CurrentMethodName);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 自动完成一次“观测 -> 预测 -> 瞄准 -> 可选开火”流程
 * @param TurretId 炮塔 ID
 * @param TargetId 目标 ID
 * @param MuzzleSpeed 弹丸初速度（m/s）
 * @param Dt 观测周期（s）
 * @param Latency 延迟补偿时间（s）
 * @param bFire 是否触发开火
 * @return 本次自动交战结果 JSON
 *
 * 延迟补偿的核心思想为：
 * $p_{comp} = \hat{p}(t + latency)$，
 * 即先用预测器把目标状态外推到射击生效时刻，再交由制导算法求解瞄准角。
 */
// 解释：这一行定义函数 `AutoEngage`，开始实现autoengage的具体逻辑。
FString AGuidanceActor::AutoEngage(FString TurretId, FString TargetId, float MuzzleSpeed, float Dt, float Latency, bool bFire)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：调用 `EnsureInitialized` 执行当前步骤需要的功能逻辑。
    EnsureInitialized();

    // 解释：这一行把右侧表达式的结果写入 `UAgentManager* Manager`，完成 uagent管理器管理器 的更新。
    UAgentManager* Manager = UAgentManager::GetInstance();
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!Manager)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return MakeError(TEXT("Agent manager unavailable"));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行把右侧表达式的结果写入 `const FString EffectiveTargetId`，完成 constfstringeffectivetargetid 的更新。
    const FString EffectiveTargetId = TargetId.IsEmpty() ? TEXT("drone_0") : TargetId;
    // 解释：这一行把右侧表达式的结果写入 `const FString EffectiveTurretId`，完成 constfstringeffectiveturretid 的更新。
    const FString EffectiveTurretId = TurretId.IsEmpty() ? TEXT("turret_0") : TurretId;
    // 解释：这一行把右侧表达式的结果写入 `AActor* Target`，完成 aactortarget 的更新。
    AActor* Target = Manager->GetAgent(EffectiveTargetId);
    // 解释：这一行把右侧表达式的结果写入 `ATurretPawn* Turret`，完成 aturretPawnturret 的更新。
    ATurretPawn* Turret = Cast<ATurretPawn>(Manager->GetAgent(EffectiveTurretId));
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!Target)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return MakeError(FString::Printf(TEXT("Target '%s' not found"), *EffectiveTargetId));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!Turret)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return MakeError(FString::Printf(TEXT("Turret '%s' not found"), *EffectiveTurretId));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行先对计算结果做限幅，再写入 `const float EffectiveLatency`，防止 constfloateffectivelatency 超出允许范围。
    const float EffectiveLatency = (Latency >= 0.0f) ? FMath::Clamp(Latency, 0.0f, 1.0f) : DefaultVisionLatency;
    // 解释：这一行把右侧表达式的结果写入 `LastLatencyCompensation`，完成 lastlatencycompensation 的更新。
    LastLatencyCompensation = EffectiveLatency;

    // 解释：调用 `Update` 执行当前步骤需要的功能逻辑。
    Predictor->Update(Target->GetActorLocation(), Dt);
    // 解释：这一行把右侧表达式的结果写入 `const FVector CompensatedTargetPos`，完成 constfvectorcompensatedtargetpos 的更新。
    const FVector CompensatedTargetPos = Predictor->PredictPosition(EffectiveLatency);

    // 解释：这一行声明成员或局部变量 `Input`，用于保存input。
    FGuidanceInput Input;
    // 解释：这一行把右侧表达式的结果写入 `Input.TurretPos`，完成 turretpos 的更新。
    Input.TurretPos = Turret->GetActorLocation();
    // 解释：这一行把右侧表达式的结果写入 `Input.MuzzlePos`，完成 muzzlepos 的更新。
    Input.MuzzlePos = GetTurretMuzzlePosition(Turret);
    // 解释：这一行把右侧表达式的结果写入 `Input.TargetPos`，完成 目标位置 的更新。
    Input.TargetPos = CompensatedTargetPos;
    // 解释：这一行把右侧表达式的结果写入 `Input.TargetVel`，完成 目标速度 的更新。
    Input.TargetVel = Predictor->GetEstimatedVelocity();
    // 解释：这一行把右侧表达式的结果写入 `Input.PredictedPos`，完成 predictedpos 的更新。
    Input.PredictedPos = Predictor->PredictPosition(EffectiveLatency + 0.5f);
    // 解释：这一行把右侧表达式的结果写入 `Input.MuzzleSpeed`，完成 muzzlespeed 的更新。
    Input.MuzzleSpeed = MuzzleSpeed;
    // 解释：这一行把右侧表达式的结果写入 `Input.DeltaTime`，完成 时间步长 的更新。
    Input.DeltaTime = Dt;

    // 解释：这一行把右侧表达式的结果写入 `const FGuidanceOutput Output`，完成 constfguidanceoutputoutput 的更新。
    const FGuidanceOutput Output = CurrentMethod->ComputeAim(Input);
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (Output.bValid)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `SetTargetAngles` 执行当前步骤需要的功能逻辑。
        Turret->SetTargetAngles(Output.Pitch, Output.Yaw);
        // 解释：这一行把右侧表达式的结果写入 `LastPitch`，完成 lastpitch 的更新。
        LastPitch = Output.Pitch;
        // 解释：这一行把右侧表达式的结果写入 `LastYaw`，完成 lastyaw 的更新。
        LastYaw = Output.Yaw;
        // 解释：这一行把右侧表达式的结果写入 `LastAimPoint`，完成 lastaimpoint 的更新。
        LastAimPoint = Output.AimPoint;
        // 解释：这一行把右侧表达式的结果写入 `LastFlightTime`，完成 lastflighttime 的更新。
        LastFlightTime = Output.EstFlightTime;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (bFire)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `FireX` 执行当前步骤需要的功能逻辑。
        Turret->FireX(MuzzleSpeed);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return FString::Printf(
        // 解释：这一行位于构造函数初始化列表中，把 `TEXT` 直接初始化为 `"{\"status\":\"ok\",\"pitch\":%.2f,\"yaw\":%.2f,\"fired\":%s,\"flight_time\":%.4f,\"latency\":%.4f,\"method\":\"%s\"}"`，减少进入函数体后的额外赋值开销。
        TEXT("{\"status\":\"ok\",\"pitch\":%.2f,\"yaw\":%.2f,\"fired\":%s,\"flight_time\":%.4f,\"latency\":%.4f,\"method\":\"%s\"}"),
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Output.Pitch,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Output.Yaw,
        *BoolLiteral(bFire),
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Output.EstFlightTime,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        LastLatencyCompensation,
        *CurrentMethodName);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 重新设置 Kalman 预测器噪声参数
 * @param ProcessNoise 过程噪声
 * @param MeasurementNoise 观测噪声
 */
// 解释：这一行定义函数 `SetKalmanParams`，开始实现set卡尔曼params的具体逻辑。
FString AGuidanceActor::SetKalmanParams(float ProcessNoise, float MeasurementNoise)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：调用 `EnsureInitialized` 执行当前步骤需要的功能逻辑。
    EnsureInitialized();
    // 解释：调用 `Initialize` 执行当前步骤需要的功能逻辑。
    Predictor->Initialize(ProcessNoise, MeasurementNoise);
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return MakeOk(FString::Printf(TEXT("kalman Q=%.2f R=%.2f"), ProcessNoise, MeasurementNoise));
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 重置 GuidanceActor 的内部运行状态
 *
 * 该接口会清空：
 * - Kalman 预测器历史状态；
 * - 当前制导算法的内部记忆量；
 * - 最近一次瞄准/拦截结果缓存；
 * - 视觉拦截控制器运行状态。
 */
// 解释：这一行定义函数 `ResetGuidance`，开始实现reset制导的具体逻辑。
FString AGuidanceActor::ResetGuidance()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：调用 `EnsureInitialized` 执行当前步骤需要的功能逻辑。
    EnsureInitialized();

    // 解释：调用 `Reset` 执行当前步骤需要的功能逻辑。
    Predictor->Reset();
    // 解释：调用 `Initialize` 执行当前步骤需要的功能逻辑。
    Predictor->Initialize(100.0f, 0.01f);

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (CurrentMethod)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `Reset` 执行当前步骤需要的功能逻辑。
        CurrentMethod->Reset();
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行把右侧表达式的结果写入 `LastPitch`，完成 lastpitch 的更新。
    LastPitch = 0.0f;
    // 解释：这一行把右侧表达式的结果写入 `LastYaw`，完成 lastyaw 的更新。
    LastYaw = 0.0f;
    // 解释：这一行把右侧表达式的结果写入 `LastAimPoint`，完成 lastaimpoint 的更新。
    LastAimPoint = FVector::ZeroVector;
    // 解释：这一行把右侧表达式的结果写入 `LastFlightTime`，完成 lastflighttime 的更新。
    LastFlightTime = 0.0f;
    // 解释：这一行把右侧表达式的结果写入 `LastLatencyCompensation`，完成 lastlatencycompensation 的更新。
    LastLatencyCompensation = DefaultVisionLatency;

    // 解释：调用 `Empty` 执行当前步骤需要的功能逻辑。
    LastInterceptorId.Empty();
    // 解释：调用 `Empty` 执行当前步骤需要的功能逻辑。
    LastTargetId.Empty();
    // 解释：这一行把右侧表达式的结果写入 `LastInterceptorCmdVel`，完成 lastinterceptorcmdvel 的更新。
    LastInterceptorCmdVel = FVector::ZeroVector;
    // 解释：这一行把右侧表达式的结果写入 `LastDistanceToTarget`，完成 lastdistancetotarget 的更新。
    LastDistanceToTarget = 0.0f;
    // 解释：这一行把右侧表达式的结果写入 `LastClosingSpeed`，完成 lastclosingspeed 的更新。
    LastClosingSpeed = 0.0f;
    // 解释：这一行把右侧表达式的结果写入 `bLastInterceptValid`，完成 布尔标志 last拦截valid 的更新。
    bLastInterceptValid = false;
    // 解释：这一行把右侧表达式的结果写入 `bLastCaptured`，完成 布尔标志 lastcaptured 的更新。
    bLastCaptured = false;

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (VisualInterceptController)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `Reset` 执行当前步骤需要的功能逻辑。
        VisualInterceptController->Reset();
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return MakeOk(TEXT("guidance reset"));
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 汇总当前制导、预测与拦截状态
 * @return 完整状态 JSON
 */
// 解释：这一行定义函数 `GetState`，开始实现get状态的具体逻辑。
FString AGuidanceActor::GetState()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：调用 `EnsureInitialized` 执行当前步骤需要的功能逻辑。
    EnsureInitialized();

    // 解释：这一行把右侧表达式的结果写入 `const FVector EstPos`，完成 constfvectorestpos 的更新。
    const FVector EstPos = Predictor->GetEstimatedPosition();
    // 解释：这一行把右侧表达式的结果写入 `const FVector EstVel`，完成 constfvectorestvel 的更新。
    const FVector EstVel = Predictor->GetEstimatedVelocity();
    // 解释：这一行把右侧表达式的结果写入 `const FVector EstAcc`，完成 constfvectorestacc 的更新。
    const FVector EstAcc = Predictor->GetEstimatedAcceleration();
    // 解释：这一行把右侧表达式的结果写入 `const float Uncertainty`，完成 constfloatuncertainty 的更新。
    const float Uncertainty = Predictor->GetPositionUncertainty();
    // 解释：这一行把右侧表达式的结果写入 `const float AdaptiveQ`，完成 constfloatadaptiveQ 的更新。
    const float AdaptiveQ = Predictor->GetAdaptiveProcessNoise();

    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return FString::Printf(
        // 解释：这一行位于构造函数初始化列表中，把 `TEXT` 直接初始化为 `"{\"status\":\"ok\",\"id\":\"%s\",\"method\":\"%s\",\"initialized\":%s,\"est_pos\":[%.2f,%.2f,%.2f],\"est_vel\":[%.2f,%.2f,%.2f],\"est_acc\":[%.2f,%.2f,%.2f],\"uncertainty\":%.2f,\"adaptive_q\":%.4f,\"aim_pitch\":%.2f,\"aim_yaw\":%.2f,\"aim_point\":[%.1f,%.1f,%.1f],\"flight_time\":%.4f,\"latency\":%.4f,\"intercept\":{\"method\":\"%s\",\"target_id\":\"%s\",\"interceptor_id\":\"%s\",\"distance\":%.3f,\"closing_speed\":%.3f,\"captured\":%s,\"valid\":%s,\"cmd_velocity\":[%.3f,%.3f,%.3f]}}"`，减少进入函数体后的额外赋值开销。
        TEXT("{\"status\":\"ok\",\"id\":\"%s\",\"method\":\"%s\",\"initialized\":%s,\"est_pos\":[%.2f,%.2f,%.2f],\"est_vel\":[%.2f,%.2f,%.2f],\"est_acc\":[%.2f,%.2f,%.2f],\"uncertainty\":%.2f,\"adaptive_q\":%.4f,\"aim_pitch\":%.2f,\"aim_yaw\":%.2f,\"aim_point\":[%.1f,%.1f,%.1f],\"flight_time\":%.4f,\"latency\":%.4f,\"intercept\":{\"method\":\"%s\",\"target_id\":\"%s\",\"interceptor_id\":\"%s\",\"distance\":%.3f,\"closing_speed\":%.3f,\"captured\":%s,\"valid\":%s,\"cmd_velocity\":[%.3f,%.3f,%.3f]}}"),
        *GuidanceId.ReplaceCharWithEscapedChar(),
        *CurrentMethodName,
        *BoolLiteral(Predictor->IsInitialized()),
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        EstPos.X,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        EstPos.Y,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        EstPos.Z,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        EstVel.X,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        EstVel.Y,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        EstVel.Z,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        EstAcc.X,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        EstAcc.Y,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        EstAcc.Z,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Uncertainty,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        AdaptiveQ,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        LastPitch,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        LastYaw,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        LastAimPoint.X,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        LastAimPoint.Y,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        LastAimPoint.Z,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        LastFlightTime,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        LastLatencyCompensation,
        *CurrentInterceptMethod,
        *LastTargetId.ReplaceCharWithEscapedChar(),
        *LastInterceptorId.ReplaceCharWithEscapedChar(),
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        LastDistanceToTarget,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        LastClosingSpeed,
        *BoolLiteral(bLastCaptured),
        *BoolLiteral(bLastInterceptValid),
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        LastInterceptorCmdVel.X,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        LastInterceptorCmdVel.Y,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        LastInterceptorCmdVel.Z);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 将 Blueprint/TCP 参数打包成启动命令并转发给视觉拦截控制器
 * @return 视觉拦截启动结果 JSON
 */
// 解释：这一行定义函数 `VisualInterceptStart`，开始实现视觉拦截start的具体逻辑。
FString AGuidanceActor::VisualInterceptStart(
    // 解释：这一行继续展开 `VisualInterceptStart` 的参数列表，声明参数 `InterceptorId` 用于传入interceptorid。
    FString InterceptorId,
    // 解释：这一行继续展开 `VisualInterceptStart` 的参数列表，声明参数 `TargetId` 用于传入targetid。
    FString TargetId,
    // 解释：这一行继续展开 `VisualInterceptStart` 的参数列表，声明参数 `Method` 用于传入method。
    FString Method,
    // 解释：这一行继续展开 `VisualInterceptStart` 的参数列表，声明参数 `DesiredArea` 用于传入desiredarea。
    float DesiredArea,
    // 解释：这一行继续展开 `VisualInterceptStart` 的参数列表，声明参数 `CaptureArea` 用于传入采集area。
    float CaptureArea,
    // 解释：这一行继续展开 `VisualInterceptStart` 的参数列表，声明参数 `CenterTolX` 用于传入centertolX。
    float CenterTolX,
    // 解释：这一行继续展开 `VisualInterceptStart` 的参数列表，声明参数 `CenterTolY` 用于传入centertolY。
    float CenterTolY,
    // 解释：这一行继续展开 `VisualInterceptStart` 的参数列表，声明参数 `CaptureHoldFrames` 用于传入采集holdframes。
    int32 CaptureHoldFrames,
    // 解释：这一行继续展开 `VisualInterceptStart` 的参数列表，声明参数 `LostToSearchFrames` 用于传入losttosearchframes。
    int32 LostToSearchFrames,
    // 解释：这一行继续展开 `VisualInterceptStart` 的参数列表，声明参数 `MaxForwardSpeed` 用于传入maxforwardspeed。
    float MaxForwardSpeed,
    // 解释：这一行继续展开 `VisualInterceptStart` 的参数列表，声明参数 `MaxReverseSpeed` 用于传入maxreversespeed。
    float MaxReverseSpeed,
    // 解释：这一行继续展开 `VisualInterceptStart` 的参数列表，声明参数 `MaxVerticalSpeed` 用于传入maxverticalspeed。
    float MaxVerticalSpeed,
    // 解释：这一行继续展开 `VisualInterceptStart` 的参数列表，声明参数 `MaxYawRateDeg` 用于传入maxyawratedeg。
    float MaxYawRateDeg,
    // 解释：这一行继续展开 `VisualInterceptStart` 的参数列表，声明参数 `SearchCamYawLimitDeg` 用于传入searchcamyawlimitdeg。
    float SearchCamYawLimitDeg,
    // 解释：这一行继续展开 `VisualInterceptStart` 的参数列表，声明参数 `SearchCamRateDeg` 用于传入searchcamratedeg。
    float SearchCamRateDeg,
    // 解释：这一行继续展开 `VisualInterceptStart` 的参数列表，声明参数 `SearchBodyYawRateDeg` 用于传入searchbodyyawratedeg。
    float SearchBodyYawRateDeg,
    // 解释：这一行继续展开 `VisualInterceptStart` 的参数列表，声明参数 `SearchCamPitchDeg` 用于传入searchcampitchdeg。
    float SearchCamPitchDeg,
    // 解释：这一行继续展开 `VisualInterceptStart` 的参数列表，声明参数 `SearchVzAmp` 用于传入searchvzamp。
    float SearchVzAmp,
    // 解释：这一行继续展开 `VisualInterceptStart` 的参数列表，声明参数 `StopOnCaptureFlag` 用于传入stopon采集flag。
    int32 StopOnCaptureFlag,
    // 解释：这一行收束函数 `VisualInterceptStart` 的签名，后面会进入实现体或以分号结束声明。
    int32 UseKalmanFlag)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：调用 `EnsureInitialized` 执行当前步骤需要的功能逻辑。
    EnsureInitialized();

    // 解释：调用 `MakeShareable` 执行当前步骤需要的功能逻辑。
    const TSharedPtr<FJsonObject> Cmd = MakeShareable(new FJsonObject);
    // 解释：调用 `SetStringIfNotEmpty` 执行当前步骤需要的功能逻辑。
    SetStringIfNotEmpty(Cmd, TEXT("interceptor_id"), InterceptorId);
    // 解释：调用 `SetStringIfNotEmpty` 执行当前步骤需要的功能逻辑。
    SetStringIfNotEmpty(Cmd, TEXT("target_id"), TargetId);
    // 解释：调用 `SetStringIfNotEmpty` 执行当前步骤需要的功能逻辑。
    SetStringIfNotEmpty(Cmd, TEXT("method"), Method);
    // 解释：调用 `SetNumberIfValid` 执行当前步骤需要的功能逻辑。
    SetNumberIfValid(Cmd, TEXT("desired_area"), DesiredArea);
    // 解释：调用 `SetNumberIfValid` 执行当前步骤需要的功能逻辑。
    SetNumberIfValid(Cmd, TEXT("capture_area"), CaptureArea);
    // 解释：调用 `SetNumberIfValid` 执行当前步骤需要的功能逻辑。
    SetNumberIfValid(Cmd, TEXT("center_tol_x"), CenterTolX);
    // 解释：调用 `SetNumberIfValid` 执行当前步骤需要的功能逻辑。
    SetNumberIfValid(Cmd, TEXT("center_tol_y"), CenterTolY);
    // 解释：调用 `SetIntIfValid` 执行当前步骤需要的功能逻辑。
    SetIntIfValid(Cmd, TEXT("capture_hold_frames"), CaptureHoldFrames);
    // 解释：调用 `SetIntIfValid` 执行当前步骤需要的功能逻辑。
    SetIntIfValid(Cmd, TEXT("lost_to_search_frames"), LostToSearchFrames);
    // 解释：调用 `SetNumberIfValid` 执行当前步骤需要的功能逻辑。
    SetNumberIfValid(Cmd, TEXT("max_forward_speed"), MaxForwardSpeed);
    // 解释：调用 `SetNumberIfValid` 执行当前步骤需要的功能逻辑。
    SetNumberIfValid(Cmd, TEXT("max_reverse_speed"), MaxReverseSpeed);
    // 解释：调用 `SetNumberIfValid` 执行当前步骤需要的功能逻辑。
    SetNumberIfValid(Cmd, TEXT("max_vertical_speed"), MaxVerticalSpeed);
    // 解释：调用 `SetNumberIfValid` 执行当前步骤需要的功能逻辑。
    SetNumberIfValid(Cmd, TEXT("max_yaw_rate_deg"), MaxYawRateDeg);
    // 解释：调用 `SetNumberIfValid` 执行当前步骤需要的功能逻辑。
    SetNumberIfValid(Cmd, TEXT("search_cam_yaw_limit_deg"), SearchCamYawLimitDeg);
    // 解释：调用 `SetNumberIfValid` 执行当前步骤需要的功能逻辑。
    SetNumberIfValid(Cmd, TEXT("search_cam_rate_deg"), SearchCamRateDeg);
    // 解释：调用 `SetNumberIfValid` 执行当前步骤需要的功能逻辑。
    SetNumberIfValid(Cmd, TEXT("search_body_yaw_rate_deg"), SearchBodyYawRateDeg);
    // 解释：调用 `SetNumberIfValid` 执行当前步骤需要的功能逻辑。
    SetNumberIfValid(Cmd, TEXT("search_vz_amp"), SearchVzAmp);
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!FMath::IsNearlyEqual(SearchCamPitchDeg, -1000.0f))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `SetNumberField` 执行当前步骤需要的功能逻辑。
        Cmd->SetNumberField(TEXT("search_cam_pitch_deg"), SearchCamPitchDeg);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (StopOnCaptureFlag >= 0)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `SetBoolField` 执行当前步骤需要的功能逻辑。
        Cmd->SetBoolField(TEXT("stop_on_capture"), StopOnCaptureFlag != 0);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (UseKalmanFlag >= 0)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `SetBoolField` 执行当前步骤需要的功能逻辑。
        Cmd->SetBoolField(TEXT("use_kalman"), UseKalmanFlag != 0);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return VisualInterceptController->HandleStart(Cmd, GetWorld());
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 提交一帧视觉检测结果给视觉拦截控制器
 * @return 本帧视觉拦截状态 JSON
 */
// 解释：这一行定义函数 `VisualInterceptUpdate`，开始实现视觉拦截update的具体逻辑。
FString AGuidanceActor::VisualInterceptUpdate(
    // 解释：这一行继续展开 `VisualInterceptUpdate` 的参数列表，声明参数 `HasDetection` 用于传入hasdetection。
    int32 HasDetection,
    // 解释：这一行继续展开 `VisualInterceptUpdate` 的参数列表，声明参数 `Cx` 用于传入cx。
    float Cx,
    // 解释：这一行继续展开 `VisualInterceptUpdate` 的参数列表，声明参数 `Cy` 用于传入cy。
    float Cy,
    // 解释：这一行继续展开 `VisualInterceptUpdate` 的参数列表，声明参数 `Area` 用于传入area。
    float Area,
    // 解释：这一行继续展开 `VisualInterceptUpdate` 的参数列表，声明参数 `AreaRatio` 用于传入arearatio。
    float AreaRatio,
    // 解释：这一行继续展开 `VisualInterceptUpdate` 的参数列表，声明参数 `Conf` 用于传入conf。
    float Conf,
    // 解释：这一行继续展开 `VisualInterceptUpdate` 的参数列表，声明参数 `Dt` 用于传入dt。
    float Dt,
    // 解释：这一行继续展开 `VisualInterceptUpdate` 的参数列表，声明参数 `ImageW` 用于传入图像W。
    float ImageW,
    // 解释：这一行继续展开 `VisualInterceptUpdate` 的参数列表，声明参数 `ImageH` 用于传入图像H。
    float ImageH,
    // 解释：这一行继续展开 `VisualInterceptUpdate` 的参数列表，声明参数 `InterceptorId` 用于传入interceptorid。
    FString InterceptorId,
    // 解释：这一行收束函数 `VisualInterceptUpdate` 的签名，后面会进入实现体或以分号结束声明。
    FString TargetId)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：调用 `EnsureInitialized` 执行当前步骤需要的功能逻辑。
    EnsureInitialized();

    // 解释：调用 `MakeShareable` 执行当前步骤需要的功能逻辑。
    const TSharedPtr<FJsonObject> Cmd = MakeShareable(new FJsonObject);
    // 解释：调用 `SetBoolField` 执行当前步骤需要的功能逻辑。
    Cmd->SetBoolField(TEXT("has_detection"), HasDetection != 0);
    // 解释：调用 `SetNumberField` 执行当前步骤需要的功能逻辑。
    Cmd->SetNumberField(TEXT("cx"), Cx);
    // 解释：调用 `SetNumberField` 执行当前步骤需要的功能逻辑。
    Cmd->SetNumberField(TEXT("cy"), Cy);
    // 解释：调用 `SetNumberField` 执行当前步骤需要的功能逻辑。
    Cmd->SetNumberField(TEXT("area"), Area);
    // 解释：调用 `SetNumberField` 执行当前步骤需要的功能逻辑。
    Cmd->SetNumberField(TEXT("area_ratio"), AreaRatio);
    // 解释：调用 `SetNumberField` 执行当前步骤需要的功能逻辑。
    Cmd->SetNumberField(TEXT("conf"), Conf);
    // 解释：调用 `SetNumberField` 执行当前步骤需要的功能逻辑。
    Cmd->SetNumberField(TEXT("dt"), Dt);
    // 解释：调用 `SetNumberField` 执行当前步骤需要的功能逻辑。
    Cmd->SetNumberField(TEXT("image_w"), ImageW);
    // 解释：调用 `SetNumberField` 执行当前步骤需要的功能逻辑。
    Cmd->SetNumberField(TEXT("image_h"), ImageH);
    // 解释：调用 `SetStringIfNotEmpty` 执行当前步骤需要的功能逻辑。
    SetStringIfNotEmpty(Cmd, TEXT("interceptor_id"), InterceptorId);
    // 解释：调用 `SetStringIfNotEmpty` 执行当前步骤需要的功能逻辑。
    SetStringIfNotEmpty(Cmd, TEXT("target_id"), TargetId);

    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return VisualInterceptController->HandleUpdate(Cmd, GetWorld());
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 停止视觉拦截会话
 * @return 停止结果 JSON
 */
// 解释：这一行定义函数 `VisualInterceptStop`，开始实现视觉拦截stop的具体逻辑。
FString AGuidanceActor::VisualInterceptStop(FString InterceptorId, FString TargetId)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：调用 `EnsureInitialized` 执行当前步骤需要的功能逻辑。
    EnsureInitialized();

    // 解释：调用 `MakeShareable` 执行当前步骤需要的功能逻辑。
    const TSharedPtr<FJsonObject> Cmd = MakeShareable(new FJsonObject);
    // 解释：调用 `SetStringIfNotEmpty` 执行当前步骤需要的功能逻辑。
    SetStringIfNotEmpty(Cmd, TEXT("interceptor_id"), InterceptorId);
    // 解释：调用 `SetStringIfNotEmpty` 执行当前步骤需要的功能逻辑。
    SetStringIfNotEmpty(Cmd, TEXT("target_id"), TargetId);
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return VisualInterceptController->HandleStop(Cmd, GetWorld());
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 查询视觉拦截控制器当前状态 */
// 解释：这一行定义函数 `VisualInterceptState`，开始实现视觉拦截状态的具体逻辑。
FString AGuidanceActor::VisualInterceptState()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：调用 `EnsureInitialized` 执行当前步骤需要的功能逻辑。
    EnsureInitialized();
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return VisualInterceptController->HandleState();
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

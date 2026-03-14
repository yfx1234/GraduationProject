// 解释：引入当前实现文件对应的头文件 `CommandRouter.h`，使实现部分能够看到类和函数声明。
#include "CommandRouter.h"

// 解释：引入 `JsonObject.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "Dom/JsonObject.h"
// 解释：引入 `AgentManager.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "GraduationProject/Core/Manager/AgentManager.h"
// 解释：引入 `CommandExecutionManager.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "GraduationProject/Core/Manager/CommandExecutionManager.h"
// 解释：引入 `FCommandHandle.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "GraduationProject/Core/Network/FCommandHandle.h"
// 解释：引入 `SimGameMode.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "GraduationProject/Core/SimGameMode.h"
// 解释：引入 `SensorManager.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "GraduationProject/Core/Simulation/SensorManager.h"
// 解释：引入 `SimClockService.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "GraduationProject/Core/Simulation/SimClockService.h"
// 解释：引入 `SimulationRecorder.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "GraduationProject/Core/Simulation/SimulationRecorder.h"
// 解释：引入 `DronePawn.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "GraduationProject/Drone/DronePawn.h"
// 解释：引入 `GuidanceActor.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "GraduationProject/Guidance/GuidanceActor.h"
// 解释：引入 `TurretPawn.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "GraduationProject/Turret/TurretPawn.h"
// 解释：引入 `GameInstance.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "Engine/GameInstance.h"
// 解释：引入 `GameplayStatics.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "Kismet/GameplayStatics.h"
// 解释：引入 `JsonReader.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "Serialization/JsonReader.h"
// 解释：引入 `JsonSerializer.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "Serialization/JsonSerializer.h"

// ──── 匿名命名空间：工具函数 ────
// 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
namespace
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    /** @brief 无人机任务角色枚举转 JSON 字符串 */
    // 解释：这一行定义函数 `DroneRoleToString`，开始实现无人机roletostring的具体逻辑。
    FString DroneRoleToString(EDroneMissionRole Role)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行进入 `switch` 分支分派结构，后续会按不同枚举或状态值执行不同逻辑。
        switch (Role)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
        // 解释：这一行声明 `switch` 中的一个分支标签，对应某个具体取值的处理路径。
        case EDroneMissionRole::Target:
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return TEXT("target");
        // 解释：这一行声明 `switch` 中的一个分支标签，对应某个具体取值的处理路径。
        case EDroneMissionRole::Interceptor:
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return TEXT("interceptor");
        // 解释：这一行声明 `switch` 中的一个分支标签，对应某个具体取值的处理路径。
        case EDroneMissionRole::Unknown:
        // 解释：这一行声明 `switch` 的默认分支，当前面所有 `case` 都不匹配时执行。
        default:
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return TEXT("unknown");
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    /** @brief JSON 字符串转义（反斜杠 + 双引号） */
    // 解释：这一行定义函数 `JsonEscape`，开始实现jsonescape的具体逻辑。
    FString JsonEscape(const FString& In)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行声明成员或局部变量 `Out`，用于保存out。
        FString Out = In;
        // 解释：调用 `ReplaceInline` 执行当前步骤需要的功能逻辑。
        Out.ReplaceInline(TEXT("\\"), TEXT("\\\\"));
        // 解释：调用 `ReplaceInline` 执行当前步骤需要的功能逻辑。
        Out.ReplaceInline(TEXT("\""), TEXT("\\\""));
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return Out;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    /** @brief 检查 JSON 是否包含通用 Actor 命令字段 */
    // 解释：这一行定义函数 `HasGenericActorCommand`，开始实现hasgenericActor命令的具体逻辑。
    bool HasGenericActorCommand(const TSharedPtr<FJsonObject>& JsonObject)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return JsonObject.IsValid() &&
            // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
            (JsonObject->HasField(TEXT("add_actor")) ||
             // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
             JsonObject->HasField(TEXT("remove_actor")) ||
             // 解释：调用 `HasField` 执行当前步骤需要的功能逻辑。
             JsonObject->HasField(TEXT("call_actor")));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

// ──── 命令总入口 ────

/**
 * @brief 解析 JSON 命令并分派到对应处理函数
 *
 * 工作流程：
 * 1. 反序列化 JSON
 * 2. 优先检查通用 Actor 命令（add_actor / remove_actor / call_actor）
 * 3. 按字段名分派到专用处理函数
 */
// 解释：这一行定义函数 `HandleCommand`，开始实现handle命令的具体逻辑。
FString UCommandRouter::HandleCommand(const FString& JsonString, UWorld* World)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：调用 `Create` 执行当前步骤需要的功能逻辑。
    const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(JsonString);  // 创建 JSON 读取器
    // 解释：这一行声明成员或局部变量 `JsonObject`，用于保存jsonobject。
    TSharedPtr<FJsonObject> JsonObject;
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!FJsonSerializer::Deserialize(Reader, JsonObject) || !JsonObject.IsValid())  // 反序列化失败
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return MakeErrorResponse(TEXT("Invalid JSON"));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // ── 优先处理通用 Actor 命令 ──
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (HasGenericActorCommand(JsonObject))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (!CommandHandle.IsValid())  // 首次使用时惰性创建
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
            if (!World)
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            {
                // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
                return MakeErrorResponse(TEXT("No World"));
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            }

            // 解释：这一行把右侧表达式的结果写入 `UGameInstance* GameInstance`，完成 ugameinstancegameinstance 的更新。
            UGameInstance* GameInstance = World->GetGameInstance();
            // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
            if (!GameInstance)
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            {
                // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
                return MakeErrorResponse(TEXT("GameInstance not found"));
            // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
            }

            // 解释：这一行把右侧表达式的结果写入 `CommandHandle`，完成 命令handle 的更新。
            CommandHandle = MakeUnique<FCommandHandle>(GameInstance);  // 创建 FCommandHandle
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行把右侧表达式的结果写入 `const FString GenericActorResponse`，完成 constfstringgenericActorresponse 的更新。
        const FString GenericActorResponse = CommandHandle->HandleCommand(JsonObject);  // 委托处理
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (!GenericActorResponse.IsEmpty())
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return GenericActorResponse;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // ── 按字段名分派到各处理函数 ──

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (JsonObject->HasField(TEXT("ping")))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return HandlePing();
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (JsonObject->HasField(TEXT("sim_pause")))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return HandleSimPause(World);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (JsonObject->HasField(TEXT("sim_resume")))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return HandleSimResume(World);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (JsonObject->HasField(TEXT("sim_reset")))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return HandleSimReset(World);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (JsonObject->HasField(TEXT("sim_get_time")))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return HandleSimGetTime(World);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (JsonObject->HasField(TEXT("sim_set_time_scale")))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return HandleSimSetTimeScale(JsonObject, World);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (JsonObject->HasField(TEXT("sim_step")))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return HandleSimStep(JsonObject, World);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (JsonObject->HasField(TEXT("get_agent_list")))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return HandleGetAgentList();
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (JsonObject->HasField(TEXT("get_command_status")))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return HandleGetCommandStatus(JsonObject);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (JsonObject->HasField(TEXT("cancel_command")))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return HandleCancelCommand(JsonObject);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (JsonObject->HasField(TEXT("get_sensor_data")))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return HandleGetSensorData(JsonObject, World);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (JsonObject->HasField(TEXT("recorder_start")))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return HandleRecorderStart(JsonObject);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (JsonObject->HasField(TEXT("recorder_stop")))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return HandleRecorderStop();
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (JsonObject->HasField(TEXT("recorder_status")))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return HandleRecorderStatus();
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (JsonObject->HasField(TEXT("recorder_record_state")))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return HandleRecorderRecordState(World);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
return MakeErrorResponse(TEXT("Unknown command"));  // 未匹配任何命令
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

// ──── 仿真控制处理 ────

/** @brief 心跳检测，返回 "pong" */
// 解释：这一行定义函数 `HandlePing`，开始实现handleping的具体逻辑。
FString UCommandRouter::HandlePing()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return TEXT("{\"status\":\"ok\",\"message\":\"pong\"}");
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 暂停仿真——委托 GameMode */
// 解释：这一行定义函数 `HandleSimPause`，开始实现handle仿真pause的具体逻辑。
FString UCommandRouter::HandleSimPause(UWorld* World)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!World)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return MakeErrorResponse(TEXT("No World"));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行把右侧表达式的结果写入 `ASimGameMode* GameMode`，完成 asimgame模式game模式 的更新。
    ASimGameMode* GameMode = Cast<ASimGameMode>(World->GetAuthGameMode());  // 获取 GameMode
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!GameMode)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return MakeErrorResponse(TEXT("GameMode not found"));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：调用 `PauseSimulation` 执行当前步骤需要的功能逻辑。
    GameMode->PauseSimulation();
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return MakeOkResponse(TEXT("simulation paused"));
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 恢复仿真 */
// 解释：这一行定义函数 `HandleSimResume`，开始实现handle仿真resume的具体逻辑。
FString UCommandRouter::HandleSimResume(UWorld* World)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!World)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return MakeErrorResponse(TEXT("No World"));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行把右侧表达式的结果写入 `ASimGameMode* GameMode`，完成 asimgame模式game模式 的更新。
    ASimGameMode* GameMode = Cast<ASimGameMode>(World->GetAuthGameMode());
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!GameMode)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return MakeErrorResponse(TEXT("GameMode not found"));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：调用 `ResumeSimulation` 执行当前步骤需要的功能逻辑。
    GameMode->ResumeSimulation();
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return MakeOkResponse(TEXT("simulation resumed"));
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 重置仿真 */
// 解释：这一行定义函数 `HandleSimReset`，开始实现handle仿真reset的具体逻辑。
FString UCommandRouter::HandleSimReset(UWorld* World)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!World)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return MakeErrorResponse(TEXT("No World"));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行把右侧表达式的结果写入 `ASimGameMode* GameMode`，完成 asimgame模式game模式 的更新。
    ASimGameMode* GameMode = Cast<ASimGameMode>(World->GetAuthGameMode());
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!GameMode)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return MakeErrorResponse(TEXT("GameMode not found"));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：调用 `ResetSimulation` 执行当前步骤需要的功能逻辑。
    GameMode->ResetSimulation();
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return MakeOkResponse(TEXT("simulation reset"));
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 获取仿真时间、实时时间、时间缩放倍率 */
// 解释：这一行定义函数 `HandleSimGetTime`，开始实现handle仿真gettime的具体逻辑。
FString UCommandRouter::HandleSimGetTime(UWorld* World)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!World)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return MakeErrorResponse(TEXT("No World"));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行把右侧表达式的结果写入 `USimClockService* Clock`，完成 usim时钟service时钟 的更新。
    USimClockService* Clock = USimClockService::GetInstance();
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!Clock->IsInitialized())
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `Initialize` 执行当前步骤需要的功能逻辑。
        Clock->Initialize(World);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return FString::Printf(
        // 解释：这一行位于构造函数初始化列表中，把 `TEXT` 直接初始化为 `"{\"status\":\"ok\",\"sim_time\":%.6f,\"wall_time\":%.6f,\"time_scale\":%.3f}"`，减少进入函数体后的额外赋值开销。
        TEXT("{\"status\":\"ok\",\"sim_time\":%.6f,\"wall_time\":%.6f,\"time_scale\":%.3f}"),
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Clock->GetSimTimeSec(World),
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Clock->GetWallTimeSec(),
        // 解释：调用 `GetTimeScale` 执行当前步骤需要的功能逻辑。
        Clock->GetTimeScale());
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 设置时间缩放倍率，读取 sim_set_time_scale.time_scale 字段 */
// 解释：这一行定义函数 `HandleSimSetTimeScale`，开始实现handle仿真settimescale的具体逻辑。
FString UCommandRouter::HandleSimSetTimeScale(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!World)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return MakeErrorResponse(TEXT("No World"));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行声明成员或局部变量 `Obj`，用于保存obj。
    const TSharedPtr<FJsonObject>* Obj = nullptr;
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!JsonObject->TryGetObjectField(TEXT("sim_set_time_scale"), Obj) || !Obj || !Obj->IsValid())
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return MakeErrorResponse(TEXT("Missing sim_set_time_scale field"));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    const float TimeScale = (*Obj)->HasField(TEXT("time_scale"))
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        ? static_cast<float>((*Obj)->GetNumberField(TEXT("time_scale")))
        // 解释：这一行开始构造函数初始化列表，下面会依次初始化成员变量。
        : 1.0f;

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!USimClockService::GetInstance()->IsInitialized())
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `GetInstance` 执行当前步骤需要的功能逻辑。
        USimClockService::GetInstance()->Initialize(World);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：调用 `GetInstance` 执行当前步骤需要的功能逻辑。
    USimClockService::GetInstance()->SetTimeScale(TimeScale, World);
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return MakeOkResponse(FString::Printf(TEXT("time_scale=%.3f"), USimClockService::GetInstance()->GetTimeScale()));
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 单步推进仿真：暂时解除暂停 → Tick 指定次数 → 恢复暂停状态
 * @note 读取 sim_step.steps 和 sim_step.dt 字段
 */
// 解释：这一行定义函数 `HandleSimStep`，开始实现handle仿真step的具体逻辑。
FString UCommandRouter::HandleSimStep(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!World)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return MakeErrorResponse(TEXT("No World"));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行声明成员或局部变量 `Obj`，用于保存obj。
    const TSharedPtr<FJsonObject>* Obj = nullptr;
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!JsonObject->TryGetObjectField(TEXT("sim_step"), Obj) || !Obj || !Obj->IsValid())
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return MakeErrorResponse(TEXT("Missing sim_step field"));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    const int32 Steps = (*Obj)->HasField(TEXT("steps"))  // 读取步数，默认 1
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        ? FMath::Max(1, static_cast<int32>((*Obj)->GetNumberField(TEXT("steps"))))
        // 解释：这一行开始构造函数初始化列表，下面会依次初始化成员变量。
        : 1;
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    const float Dt = (*Obj)->HasField(TEXT("dt"))  // 读取 dt，默认 1/60s
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        ? FMath::Max(0.0005f, static_cast<float>((*Obj)->GetNumberField(TEXT("dt"))))
        // 解释：这一行开始构造函数初始化列表，下面会依次初始化成员变量。
        : (1.0f / 60.0f);

    // 解释：这一行把右侧表达式的结果写入 `const bool bWasPaused`，完成 constboolBwaspaused 的更新。
    const bool bWasPaused = UGameplayStatics::IsGamePaused(World);  // 记录原始暂停状态
    // 解释：调用 `SetGamePaused` 执行当前步骤需要的功能逻辑。
    UGameplayStatics::SetGamePaused(World, false);                  // 解除暂停
    // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
    for (int32 Index = 0; Index < Steps; ++Index)                   // 逻辑步进
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `Tick` 执行当前步骤需要的功能逻辑。
        World->Tick(ELevelTick::LEVELTICK_All, Dt);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：调用 `SetGamePaused` 执行当前步骤需要的功能逻辑。
    UGameplayStatics::SetGamePaused(World, bWasPaused);             // 恢复原状态

    // 解释：这一行把右侧表达式的结果写入 `USimClockService* Clock`，完成 usim时钟service时钟 的更新。
    USimClockService* Clock = USimClockService::GetInstance();
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!Clock->IsInitialized())
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `Initialize` 执行当前步骤需要的功能逻辑。
        Clock->Initialize(World);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return FString::Printf(
        // 解释：这一行位于构造函数初始化列表中，把 `TEXT` 直接初始化为 `"{\"status\":\"ok\",\"steps\":%d,\"dt\":%.6f,\"sim_time\":%.6f,\"wall_time\":%.6f,\"time_scale\":%.3f}"`，减少进入函数体后的额外赋值开销。
        TEXT("{\"status\":\"ok\",\"steps\":%d,\"dt\":%.6f,\"sim_time\":%.6f,\"wall_time\":%.6f,\"time_scale\":%.3f}"),
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Steps,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Dt,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Clock->GetSimTimeSec(World),
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Clock->GetWallTimeSec(),
        // 解释：调用 `GetTimeScale` 执行当前步骤需要的功能逻辑。
        Clock->GetTimeScale());
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

// ──── 智能体查询 ────

/** @brief 获取全部已注册智能体列表（ID + 类型 + 角色） */
// 解释：这一行定义函数 `HandleGetAgentList`，开始实现handlegetagent列表的具体逻辑。
FString UCommandRouter::HandleGetAgentList()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `UAgentManager* Manager`，完成 uagent管理器管理器 的更新。
    UAgentManager* Manager = UAgentManager::GetInstance();
    // 解释：调用 `GetAllAgentIds` 执行当前步骤需要的功能逻辑。
    const TArray<FString> Ids = Manager->GetAllAgentIds();  // 获取所有 ID

    // 解释：这一行把右侧表达式的结果写入 `FString AgentIdsJson`，完成 fstringagentidsjson 的更新。
    FString AgentIdsJson = TEXT("[");      // ID 数组
    // 解释：这一行把右侧表达式的结果写入 `FString AgentDetailJson`，完成 fstringagentdetailjson 的更新。
    FString AgentDetailJson = TEXT("[");   // 详细信息数组
    // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
    for (int32 Index = 0; Index < Ids.Num(); ++Index)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行声明成员或局部变量 `Id`，用于保存id。
        const FString& Id = Ids[Index];
        // 解释：这一行把右侧表达式的结果写入 `AActor* Agent`，完成 aactoragent 的更新。
        AActor* Agent = Manager->GetAgent(Id);  // 获取 Actor 对象

        // ​── 根据类型确定 type 和 role ──
        // 解释：这一行把右侧表达式的结果写入 `FString Type`，完成 fstringtype 的更新。
        FString Type = TEXT("actor");
        // 解释：这一行把右侧表达式的结果写入 `FString Role`，完成 fstringrole 的更新。
        FString Role = TEXT("unknown");

        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (const ADronePawn* Drone = Cast<ADronePawn>(Agent))
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行把右侧表达式的结果写入 `Type`，完成 type 的更新。
            Type = TEXT("drone");
            // 解释：这一行把右侧表达式的结果写入 `Role`，完成 role 的更新。
            Role = DroneRoleToString(Drone->MissionRole);
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
        // 解释：这一行声明补充分支，当上一个条件不成立且当前条件成立时执行。
        else if (Cast<ATurretPawn>(Agent))
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行把右侧表达式的结果写入 `Type`，完成 type 的更新。
            Type = TEXT("turret");
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
        // 解释：这一行声明补充分支，当上一个条件不成立且当前条件成立时执行。
        else if (Cast<AGuidanceActor>(Agent))
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行把右侧表达式的结果写入 `Type`，完成 type 的更新。
            Type = TEXT("guidance");
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行在 `AgentIdsJson` 的原有基础上继续累加新量，用于持续更新 agentidsjson。
        AgentIdsJson += FString::Printf(TEXT("\"%s\""), *JsonEscape(Id));
        // 解释：这一行在 `AgentDetailJson` 的原有基础上继续累加新量，用于持续更新 agentdetailjson。
        AgentDetailJson += FString::Printf(TEXT("{\"id\":\"%s\",\"type\":\"%s\",\"role\":\"%s\"}"), *JsonEscape(Id), *Type, *Role);
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (Index + 1 < Ids.Num())
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行在 `AgentIdsJson` 的原有基础上继续累加新量，用于持续更新 agentidsjson。
            AgentIdsJson += TEXT(",");
            // 解释：这一行在 `AgentDetailJson` 的原有基础上继续累加新量，用于持续更新 agentdetailjson。
            AgentDetailJson += TEXT(",");
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行在 `AgentIdsJson` 的原有基础上继续累加新量，用于持续更新 agentidsjson。
    AgentIdsJson += TEXT("]");
    // 解释：这一行在 `AgentDetailJson` 的原有基础上继续累加新量，用于持续更新 agentdetailjson。
    AgentDetailJson += TEXT("]");

    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return FString::Printf(TEXT("{\"status\":\"ok\",\"agents\":%s,\"agents_detail\":%s,\"count\":%d}"), *AgentIdsJson, *AgentDetailJson, Ids.Num());
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

// ──── 命令执行管理 ────

/** @brief 查询异步命令的执行状态（可指定超时等待） */
// 解释：这一行定义函数 `HandleGetCommandStatus`，开始实现handleget命令status的具体逻辑。
FString UCommandRouter::HandleGetCommandStatus(const TSharedPtr<FJsonObject>& JsonObject)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行声明成员或局部变量 `Obj`，用于保存obj。
    const TSharedPtr<FJsonObject>* Obj = nullptr;
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!JsonObject->TryGetObjectField(TEXT("get_command_status"), Obj) || !Obj || !Obj->IsValid())
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return MakeErrorResponse(TEXT("Missing get_command_status field"));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行声明成员或局部变量 `CommandId`，用于保存命令id。
    FString CommandId;
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!(*Obj)->TryGetStringField(TEXT("command_id"), CommandId))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return MakeErrorResponse(TEXT("Missing command_id"));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    const double TimeoutSec = (*Obj)->HasField(TEXT("timeout_sec"))
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        ? FMath::Max(0.0, (*Obj)->GetNumberField(TEXT("timeout_sec")))
        // 解释：这一行开始构造函数初始化列表，下面会依次初始化成员变量。
        : 0.0;

    // 解释：这一行声明成员或局部变量 `Record`，用于保存记录。
    FCommandExecutionRecord Record;
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!UCommandExecutionManager::GetInstance()->GetCommandWithTimeout(CommandId, TimeoutSec, Record))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return MakeErrorResponse(FString::Printf(TEXT("Command '%s' not found"), *CommandId));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    const double Duration = (Record.EndTimeSec > Record.StartTimeSec)
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        ? (Record.EndTimeSec - Record.StartTimeSec)
        // 解释：这一行开始构造函数初始化列表，下面会依次初始化成员变量。
        : 0.0;

    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return FString::Printf(
        // 解释：这一行位于构造函数初始化列表中，把 `TEXT` 直接初始化为 `"{\"status\":\"ok\",\"command_id\":\"%s\",\"agent_id\":\"%s\",\"function\":\"%s\",\"state\":\"%s\",\"message\":\"%s\",\"start_time\":%.6f,\"end_time\":%.6f,\"duration\":%.6f}"`，减少进入函数体后的额外赋值开销。
        TEXT("{\"status\":\"ok\",\"command_id\":\"%s\",\"agent_id\":\"%s\",\"function\":\"%s\",\"state\":\"%s\",\"message\":\"%s\",\"start_time\":%.6f,\"end_time\":%.6f,\"duration\":%.6f}"),
        *Record.CommandId,
        *Record.AgentId,
        *Record.FunctionName,
        *Record.Status,
        *Record.Message.ReplaceCharWithEscapedChar(),
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Record.StartTimeSec,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Record.EndTimeSec,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Duration);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 取消正在执行的异步命令 */
// 解释：这一行定义函数 `HandleCancelCommand`，开始实现handlecancel命令的具体逻辑。
FString UCommandRouter::HandleCancelCommand(const TSharedPtr<FJsonObject>& JsonObject)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行声明成员或局部变量 `Obj`，用于保存obj。
    const TSharedPtr<FJsonObject>* Obj = nullptr;
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!JsonObject->TryGetObjectField(TEXT("cancel_command"), Obj) || !Obj || !Obj->IsValid())
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return MakeErrorResponse(TEXT("Missing cancel_command field"));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行声明成员或局部变量 `CommandId`，用于保存命令id。
    FString CommandId;
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!(*Obj)->TryGetStringField(TEXT("command_id"), CommandId))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return MakeErrorResponse(TEXT("Missing command_id"));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行把右侧表达式的结果写入 `FString Reason`，完成 fstringreason 的更新。
    FString Reason = TEXT("canceled");
    // 解释：调用 `TryGetStringField` 执行当前步骤需要的功能逻辑。
    (*Obj)->TryGetStringField(TEXT("reason"), Reason);

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!UCommandExecutionManager::GetInstance()->CancelCommand(CommandId, Reason))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return MakeErrorResponse(FString::Printf(TEXT("Command '%s' cannot be canceled"), *CommandId));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return MakeOkResponse(FString::Printf(TEXT("command '%s' canceled"), *CommandId));
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

// ──── 传感器数据 ────

/** @brief 获取指定无人机的传感器状态，帧坐标系可选 "ue" / "ned" 等 */
// 解释：这一行定义函数 `HandleGetSensorData`，开始实现handleget传感器data的具体逻辑。
FString UCommandRouter::HandleGetSensorData(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行声明成员或局部变量 `Obj`，用于保存obj。
    const TSharedPtr<FJsonObject>* Obj = nullptr;
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!JsonObject->TryGetObjectField(TEXT("get_sensor_data"), Obj) || !Obj || !Obj->IsValid())
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return MakeErrorResponse(TEXT("Missing get_sensor_data field"));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行把右侧表达式的结果写入 `FString DroneId`，完成 fstring无人机id 的更新。
    FString DroneId = TEXT("drone_0");
    // 解释：调用 `TryGetStringField` 执行当前步骤需要的功能逻辑。
    (*Obj)->TryGetStringField(TEXT("id"), DroneId);

    // 解释：这一行把右侧表达式的结果写入 `FString Frame`，完成 fstringframe 的更新。
    FString Frame = TEXT("ue");
    // 解释：调用 `TryGetStringField` 执行当前步骤需要的功能逻辑。
    (*Obj)->TryGetStringField(TEXT("frame"), Frame);

    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return USensorManager::GetInstance()->BuildDroneSensorJson(DroneId, World, Frame);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

// ──── 录制 ────

/** @brief 开始录制，可指定输出路径 */
// 解释：这一行定义函数 `HandleRecorderStart`，开始实现handle记录器start的具体逻辑。
FString UCommandRouter::HandleRecorderStart(const TSharedPtr<FJsonObject>& JsonObject)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行声明成员或局部变量 `Obj`，用于保存obj。
    const TSharedPtr<FJsonObject>* Obj = nullptr;
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!JsonObject->TryGetObjectField(TEXT("recorder_start"), Obj) || !Obj || !Obj->IsValid())
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return MakeErrorResponse(TEXT("Missing recorder_start field"));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行声明成员或局部变量 `Path`，用于保存path。
    FString Path;
    // 解释：调用 `TryGetStringField` 执行当前步骤需要的功能逻辑。
    (*Obj)->TryGetStringField(TEXT("path"), Path);

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!USimulationRecorder::GetInstance()->Start(Path))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return MakeErrorResponse(TEXT("recorder start failed"));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return FString::Printf(TEXT("{\"status\":\"ok\",\"path\":\"%s\"}"), *USimulationRecorder::GetInstance()->GetPath().ReplaceCharWithEscapedChar());
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 停止录制 */
// 解释：这一行定义函数 `HandleRecorderStop`，开始实现handle记录器stop的具体逻辑。
FString UCommandRouter::HandleRecorderStop()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：调用 `GetInstance` 执行当前步骤需要的功能逻辑。
    USimulationRecorder::GetInstance()->Stop();
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return MakeOkResponse(TEXT("recorder stopped"));
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 查询录制状态（是否正在录制、路径、已记录条数） */
// 解释：这一行定义函数 `HandleRecorderStatus`，开始实现handle记录器status的具体逻辑。
FString UCommandRouter::HandleRecorderStatus()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `USimulationRecorder* Recorder`，完成 usimulation记录器记录器 的更新。
    USimulationRecorder* Recorder = USimulationRecorder::GetInstance();
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return FString::Printf(
        // 解释：这一行位于构造函数初始化列表中，把 `TEXT` 直接初始化为 `"{\"status\":\"ok\",\"recording\":%s,\"path\":\"%s\",\"count\":%lld}"`，减少进入函数体后的额外赋值开销。
        TEXT("{\"status\":\"ok\",\"recording\":%s,\"path\":\"%s\",\"count\":%lld}"),
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Recorder->IsRecording() ? TEXT("true") : TEXT("false"),
        *Recorder->GetPath().ReplaceCharWithEscapedChar(),
        // 解释：调用 `GetRecordCount` 执行当前步骤需要的功能逻辑。
        Recorder->GetRecordCount());
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 立即记录一帧全部智能体状态到录制文件 */
// 解释：这一行定义函数 `HandleRecorderRecordState`，开始实现handle记录器记录状态的具体逻辑。
FString UCommandRouter::HandleRecorderRecordState(UWorld* World)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!World)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return MakeErrorResponse(TEXT("No World"));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行把右侧表达式的结果写入 `USimulationRecorder* Recorder`，完成 usimulation记录器记录器 的更新。
    USimulationRecorder* Recorder = USimulationRecorder::GetInstance();
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!Recorder->IsRecording())
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return MakeErrorResponse(TEXT("recorder is not running"));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行把右侧表达式的结果写入 `UAgentManager* Manager`，完成 uagent管理器管理器 的更新。
    UAgentManager* Manager = UAgentManager::GetInstance();
    // 解释：这一行把右侧表达式的结果写入 `FString Payload`，完成 fstringpayload 的更新。
    FString Payload = TEXT("{\"agents\":[");

    // 解释：调用 `GetAllAgentIds` 执行当前步骤需要的功能逻辑。
    const TArray<FString> Ids = Manager->GetAllAgentIds();
    // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
    for (int32 Index = 0; Index < Ids.Num(); ++Index)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行声明成员或局部变量 `Id`，用于保存id。
        const FString& Id = Ids[Index];
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (ADronePawn* Drone = Cast<ADronePawn>(Manager->GetAgent(Id)))
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行把右侧表达式的结果写入 `const FVector Pos`，完成 constfvectorpos 的更新。
            const FVector Pos = Drone->GetCurrentPosition();
            // 解释：这一行把右侧表达式的结果写入 `const FVector Vel`，完成 constfvectorvel 的更新。
            const FVector Vel = Drone->GetCurrentVelocity();
            // 解释：这一行在 `Payload` 的原有基础上继续累加新量，用于持续更新 payload。
            Payload += FString::Printf(TEXT("{\"id\":\"%s\",\"type\":\"drone\",\"position\":[%.4f,%.4f,%.4f],\"velocity\":[%.4f,%.4f,%.4f]}"), *Id, Pos.X, Pos.Y, Pos.Z, Vel.X, Vel.Y, Vel.Z);
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
        // 解释：这一行声明补充分支，当上一个条件不成立且当前条件成立时执行。
        else if (ATurretPawn* Turret = Cast<ATurretPawn>(Manager->GetAgent(Id)))
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行在 `Payload` 的原有基础上继续累加新量，用于持续更新 payload。
            Payload += FString::Printf(TEXT("{\"id\":\"%s\",\"type\":\"turret\",\"pitch\":%.3f,\"yaw\":%.3f}"), *Id, Turret->GetCurrentPitch(), Turret->GetCurrentYaw());
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
        // 解释：这一行声明补充分支，当上一个条件不成立且当前条件成立时执行。
        else if (Cast<AGuidanceActor>(Manager->GetAgent(Id)))
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行在 `Payload` 的原有基础上继续累加新量，用于持续更新 payload。
            Payload += FString::Printf(TEXT("{\"id\":\"%s\",\"type\":\"guidance\"}"), *Id);
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
        // 解释：这一行给出兜底分支，当上面的条件都不满足时执行。
        else
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行在 `Payload` 的原有基础上继续累加新量，用于持续更新 payload。
            Payload += FString::Printf(TEXT("{\"id\":\"%s\",\"type\":\"actor\"}"), *Id);
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (Index + 1 < Ids.Num())
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行在 `Payload` 的原有基础上继续累加新量，用于持续更新 payload。
            Payload += TEXT(",");
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行在 `Payload` 的原有基础上继续累加新量，用于持续更新 payload。
    Payload += TEXT("]}");

    // 解释：调用 `RecordJsonLine` 执行当前步骤需要的功能逻辑。
    Recorder->RecordJsonLine(TEXT("state"), Payload);
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return MakeOkResponse(TEXT("state recorded"));
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

// ──── 响应工具 ────

/** @brief 生成错误 JSON 响应 */
// 解释：这一行定义函数 `MakeErrorResponse`，开始实现makeerrorresponse的具体逻辑。
FString UCommandRouter::MakeErrorResponse(const FString& Error)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return FString::Printf(TEXT("{\"status\":\"error\",\"message\":\"%s\"}"), *JsonEscape(Error));
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 生成成功 JSON 响应 */
// 解释：这一行定义函数 `MakeOkResponse`，开始实现makeokresponse的具体逻辑。
FString UCommandRouter::MakeOkResponse(const FString& Message)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return FString::Printf(TEXT("{\"status\":\"ok\",\"message\":\"%s\"}"), *JsonEscape(Message));
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}




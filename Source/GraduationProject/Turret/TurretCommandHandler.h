#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "TurretCommandHandler.generated.h"

class ATurretPawn;

/**
 * 转台 TCP 命令处理器
 * 解析 turret JSON 命令，
 * 通过 AgentManager 查找指定 ID 的 TurretPawn 并委托执行
 * 返回 JSON 格式的响应（status: ok/error + message）
 */
UCLASS()
class GRADUATIONPROJECT_API UTurretCommandHandler : public UObject
{
    GENERATED_BODY()

public:
    /**
     * @brief 处理 call_turret 命令
     * @param JsonObject 完整的 JSON 请求对象
     * @param World 当前 UWorld 指针
     * @return JSON 格式的响应字符串
     * 支持:
     * - set_angles: 设置目标 Pitch/Yaw 角度
     * - fire: 发射弹丸
     * - start_tracking: 跟踪目标
     * - stop_tracking: 停止跟踪
     * - show_prediction: 显示预测弹道线
     * - hide_prediction: 隐藏预测弹道线
     * - reset: 重置转台
     */
    FString HandleCallTurret(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World);

    /**
     * @brief 处理 get_turret_state 命令
     * @param JsonObject 完整的 JSON 请求对象
     * @param World 当前 UWorld 指针
     * @return JSON 格式的状态数据
     */
    FString HandleGetTurretState(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World);

private:
    /**
     * @brief 通过 Agent ID 查找转台 Pawn
     * @param TurretId 转台 Agent ID
     * @param World 当前 UWorld 指针
     * @return ATurretPawn 指针
     */
    ATurretPawn* FindTurretPawn(const FString& TurretId, UWorld* World);

    /**
     * @brief 生成错误响应 JSON
     * @param Msg 错误信息
     * @return {"status":"error","message":"..."}
     */
    FString MakeError(const FString& Msg);

    /**
     * @brief 生成成功响应 JSON
     * @param Msg 成功信息，默认 "ok"
     * @return {"status":"ok","message":"..."}
     */
    FString MakeOk(const FString& Msg = TEXT("ok"));
};

/**
 * @file CommandRouter.h
 * @brief TCP 命令路由器的头文件
 *
 * 本文件定义了 UCommandRouter 类，负责接收 TCP 传来的 JSON 命令字符串，
 * 解析后根据命令类型路由到对应的 Handler（DroneCommandHandler、TurretCommandHandler、
 * GuidanceCommandHandler），或直接处理内置命令（ping、暂停、恢复、重置等）。
 * 参考旧项目 DroneCommandHandle（拆分为路由 + 各模块 Handler）。
 */

#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "CommandRouter.generated.h"

class UDroneCommandHandler;
class UTurretCommandHandler;
class UGuidanceCommandHandler;

/**
 * TCP 命令路由器
 *
 * 核心职责：
 * 1. 解析 JSON 字符串
 * 2. 根据 JSON 字段名路由到对应的处理器
 * 3. 返回 JSON 格式的响应字符串
 *
 * 路由规则（按 JSON 字段匹配）：
 * - "ping" → HandlePing()
 * - "sim_pause/sim_resume/sim_reset" → 仿真控制
 * - "get_agent_list" → 获取智能体列表
 * - "get_image" → 获取转台摄像头图像
 * - "call_drone/get_drone_state" → DroneCommandHandler
 * - "call_turret/get_turret_state" → TurretCommandHandler
 * - "call_guidance/get_guidance_state" → GuidanceCommandHandler
 *
 * Handler 采用懒加载方式创建（首次使用时 NewObject）。
 */
UCLASS()
class GRADUATIONPROJECT_API UCommandRouter : public UObject
{
    GENERATED_BODY()

public:
    /**
     * @brief 处理一条 TCP 命令
     * @param JsonString 收到的 JSON 字符串
     * @param World 当前 UWorld 指针（用于获取 GameMode、Actor 等）
     * @return 响应 JSON 字符串，包含 status 和 message 字段
     */
    FString HandleCommand(const FString& JsonString, UWorld* World);

private:
    // ---- 内置命令处理 ----

    /**
     * @brief 处理 ping 命令，返回 pong 响应
     * @return JSON 响应 {"status":"ok","message":"pong"}
     */
    FString HandlePing();

    /**
     * @brief 暂停仿真
     * @param World 当前 World
     * @return JSON 响应
     */
    FString HandleSimPause(UWorld* World);

    /**
     * @brief 恢复仿真
     * @param World 当前 World
     * @return JSON 响应
     */
    FString HandleSimResume(UWorld* World);

    /**
     * @brief 重置仿真
     * @param World 当前 World
     * @return JSON 响应
     */
    FString HandleSimReset(UWorld* World);

    /**
     * @brief 获取已注册智能体列表
     * @return JSON 响应，包含 agents 数组和 count 字段
     */
    FString HandleGetAgentList();

    /**
     * @brief 获取转台摄像头图像（Base64 编码的 JPEG）
     * @param World 当前 World
     * @return JSON 响应，包含 data(Base64)、camera_pos、camera_rot、fov 等字段
     */
    FString HandleGetImage(UWorld* World);

    // ---- 工具方法 ----

    /**
     * @brief 构造错误响应 JSON
     * @param Error 错误信息
     * @return {"status":"error","message":"..."}
     */
    FString MakeErrorResponse(const FString& Error);

    /**
     * @brief 构造成功响应 JSON
     * @param Message 成功信息，默认 "ok"
     * @return {"status":"ok","message":"..."}
     */
    FString MakeOkResponse(const FString& Message = TEXT("ok"));

    /** @brief 无人机命令处理器（懒加载创建） */
    UPROPERTY()
    UDroneCommandHandler* DroneHandler = nullptr;

    /** @brief 转台命令处理器（懒加载创建） */
    UPROPERTY()
    UTurretCommandHandler* TurretHandler = nullptr;

    /** @brief 制导命令处理器（懒加载创建） */
    UPROPERTY()
    UGuidanceCommandHandler* GuidanceHandler = nullptr;
};

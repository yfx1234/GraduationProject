#pragma once

#include "CineCameraComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Components/StaticMeshComponent.h"
#include "CoreMinimal.h"
#include "DroneParameters.h"
#include "DroneState.h"
#include "Engine/TextureRenderTarget2D.h"
#include "GameFramework/Pawn.h"
#include "DronePawn.generated.h"

class UDroneApi;
class UDroneMovementComponent;

/**
 * @brief 无人机任务角色
 *
 * 用于区分场景中的无人机在拦截任务中的职责，便于 Guidance 模块自动查找。
 */
UENUM(BlueprintType)
enum class EDroneMissionRole : uint8
{
    /** @brief 未指定任务角色 */
    Unknown     UMETA(DisplayName = "Unknown"),

    /** @brief 目标无人机，被拦截对象 */
    Target      UMETA(DisplayName = "Target"),

    /** @brief 拦截无人机，主动执行追踪/捕获任务 */
    Interceptor UMETA(DisplayName = "Interceptor")
};

/**
 * @brief 无人机主 Pawn
 *
 * 该类集成了：
 * 1. 飞行动力学组件 `UDroneMovementComponent`；
 * 2. 对外控制接口 `UDroneApi`；
 * 3. 云台相机与图像抓取组件；
 * 4. 与 AgentManager 的注册逻辑。
 *
 * 因此它既是仿真中的物理载体，也是 Python / Blueprint 侧远程控制的核心入口。
 */
UCLASS()
class GRADUATIONPROJECT_API ADronePawn : public APawn
{
    GENERATED_BODY()

public:
    /** @brief 构造函数：创建全部子组件并设置默认参数 */
    ADronePawn();

    /** @brief 初始化动力学、相机 RenderTarget、API，并注册到 AgentManager */
    virtual void BeginPlay() override;

    /**
     * @brief 每帧更新无人机显示状态
     * @param DeltaTime 帧间隔（s）
     *
     * 主要负责：
     * - 从动力学组件同步当前状态；
     * - 把状态写回 Actor 位置/姿态；
     * - 刷新旋翼动画；
     * - 插值更新云台相机姿态；
     * - 同步电影相机后处理到捕获组件。
     */
    virtual void Tick(float DeltaTime) override;

    /**
     * @brief 设置目标位置，切换到位置控制模式
     * @param NewTargetPosition 目标位置（m）
     * @param Speed             飞行速度上限（m/s），0 表示使用默认值
     * @param Frame             输入坐标系，支持 `ue` 与 `ned`
     */
    UFUNCTION(BlueprintCallable, Category = "DroneControl")
    void SetTargetPosition(const FVector& NewTargetPosition, float Speed = 0.0f, FString Frame = TEXT("ue"));

    /**
     * @brief 设置目标速度，切换到速度控制模式
     * @param NewTargetVelocity 目标速度（m/s）
     * @param Frame             输入坐标系，支持 `ue` 与 `ned`
     */
    UFUNCTION(BlueprintCallable, Category = "DroneControl")
    void SetTargetVelocity(const FVector& NewTargetVelocity, FString Frame = TEXT("ue"));

    /**
     * @brief 设置航向控制模式
     * @param NewYawMode     偏航控制模式
     * @param NewDrivetrain  驱动模式
     * @param YawDeg         目标偏航角或偏航角速度（视模式而定）
     */
    UFUNCTION(BlueprintCallable, Category = "DroneControl")
    void SetHeadingControl(EDroneYawMode NewYawMode, EDroneDrivetrainMode NewDrivetrain, float YawDeg = 0.0f);

    /** @brief 在当前位置悬停 */
    UFUNCTION(BlueprintCallable, Category = "DroneControl")
    void Hover();

    /**
     * @brief 以速度模式飞行的便捷封装
     * @param Vx X 方向速度（m/s）
     * @param Vy Y 方向速度（m/s）
     * @param Vz Z 方向速度（m/s）
     * @param Frame 输入坐标系
     */
    UFUNCTION(BlueprintCallable, Category = "DroneControl")
    void MoveByVelocity(float Vx, float Vy, float Vz, FString Frame = TEXT("ue"));

    /**
     * @brief 起飞到指定高度
     * @param Altitude 目标高度（m）
     */
    UFUNCTION(BlueprintCallable, Category = "Drone")
    void Takeoff(float Altitude = 3.0f);

    /** @brief 降落到地面高度 */
    UFUNCTION(BlueprintCallable, Category = "Drone")
    void Land();

    /**
     * @brief 启用或禁用 API 远程控制标志
     * @param bEnable true 为启用，false 为禁用
     */
    UFUNCTION(BlueprintCallable, Category = "Drone")
    void EnableApiControl(bool bEnable = true);

    /**
     * @brief 把无人机重置到指定位置和姿态
     * @param NewLocation 目标位置（m）
     * @param NewRotation 目标姿态
     * @param Frame       输入坐标系
     */
    UFUNCTION(BlueprintCallable, Category = "DroneControl")
    void ResetDrone(const FVector& NewLocation, const FRotator& NewRotation, FString Frame = TEXT("ue"));

    /** @brief 重置 API / 控制状态，但不直接修改当前物理位置 */
    UFUNCTION(BlueprintCallable, Category = "DroneControl")
    void ResetActorState();

    /**
     * @brief 设置目标姿态与总推力，进入姿态控制模式
     * @param RollDeg  横滚角（deg）
     * @param PitchDeg 俯仰角（deg）
     * @param YawDeg   偏航角（deg）
     * @param Thrust   总推力（N）
     * @param Frame    输入坐标系，默认 `ned`
     */
    UFUNCTION(BlueprintCallable, Category = "DroneControl")
    void SetTargetAttitude(float RollDeg, float PitchDeg, float YawDeg, float Thrust = 9.81f, FString Frame = TEXT("ned"));

    /**
     * @brief 直接设置四个电机角速度，进入电机转速控制模式
     * @param M0 电机 0 转速（rad/s）
     * @param M1 电机 1 转速（rad/s）
     * @param M2 电机 2 转速（rad/s）
     * @param M3 电机 3 转速（rad/s）
     */
    UFUNCTION(BlueprintCallable, Category = "DroneControl")
    void SetMotorSpeeds(float M0, float M1, float M2, float M3);

    /**
     * @brief 设置位置环控制器增益
     * @param Kp 比例增益
     * @param Kd 微分增益
     */
    UFUNCTION(BlueprintCallable, Category = "DroneControl")
    void SetPositionControllerGains(float Kp, float Kd = 0.0f);

    /**
     * @brief 设置速度环控制器增益
     * @param Kp 比例增益
     * @param Ki 积分增益
     * @param Kd 微分增益
     */
    UFUNCTION(BlueprintCallable, Category = "DroneControl")
    void SetVelocityControllerGains(float Kp, float Ki = 0.0f, float Kd = 0.0f);

    /**
     * @brief 设置姿态环控制器增益
     * @param Kp 比例增益
     * @param Kd 微分增益
     */
    UFUNCTION(BlueprintCallable, Category = "DroneControl")
    void SetAttitudeControllerGains(float Kp, float Kd = 0.0f);

    /**
     * @brief 设置角速度环比例增益
     * @param Kp 比例增益
     */
    UFUNCTION(BlueprintCallable, Category = "DroneControl")
    void SetAngleRateControllerGains(float Kp);

    /** @brief 获取当前位置（m） */
    FVector GetCurrentPosition() const;

    /** @brief 获取当前速度（m/s） */
    FVector GetCurrentVelocity() const;

    /**
     * @brief 设置云台目标角度
     * @param TargetPitch 目标俯仰角（deg）
     * @param TargetYaw   目标偏航角（deg）
     */
    UFUNCTION(BlueprintCallable, Category = "Drone|Camera")
    void SetCameraAngles(float TargetPitch, float TargetYaw);

    /**
     * @brief 设置该无人机的语义分割 Stencil ID
     * @param NewSegmentationId 0-255 的分割标识
     */
    UFUNCTION(BlueprintCallable, Category = "Drone|Camera")
    void SetSegmentationId(int32 NewSegmentationId);

    /**
     * @brief 获取无人机状态 JSON
     * @param Frame 输出坐标系，支持 `ue` 与 `ned`
     * @return JSON 格式状态快照
     */
    UFUNCTION(BlueprintCallable, Category = "Drone")
    FString GetState(FString Frame = TEXT("ue"));

    /**
     * @brief 获取机载相机图像 JSON
     * @param ImageType      图像类型，如 `scene`、`depth_planar`
     * @param Quality        JPEG 质量，小于等于 0 时使用默认值
     * @param MaxDepthMeters 深度图最大映射距离（m）
     * @return JSON 格式图像结果
     */
    UFUNCTION(BlueprintCallable, Category = "Drone|Camera")
    FString GetImage(FString ImageType = TEXT("scene"), int32 Quality = -1, float MaxDepthMeters = 200.0f);

    /**
     * @brief 抓取彩色图像并返回 Base64 JPEG
     * @param Quality JPEG 质量，小于等于 0 时使用默认值
     * @return Base64 编码的 JPEG 字符串
     */
    FString CaptureImageBase64(int32 Quality = -1);

    /** @brief 获取当前云台俯仰角（deg） */
    float GetCameraCurrentPitch() const { return CameraCurrentPitch; }

    /** @brief 获取当前云台偏航角（deg） */
    float GetCameraCurrentYaw() const { return CameraCurrentYaw; }

    /** @brief 当前飞行状态，包含位置、速度、姿态和电机转速等量 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|State")
    FDroneState CurrentState;

    /** @brief 当前控制模式 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|State")
    EDroneControlMode ControlMode = EDroneControlMode::Idle;

    /** @brief 无人机物理参数 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters")
    FDroneParameters Parameters;

    /** @brief 飞行动力学与级联控制组件 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    UDroneMovementComponent* MovementComp = nullptr;

    /** @brief 高层控制 API 对象 */
    UPROPERTY()
    UDroneApi* Api = nullptr;

    /** @brief 无人机唯一标识，用于注册和远程调用 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Config")
    FString DroneId = TEXT("drone_0");

    /** @brief 无人机任务角色 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Config")
    EDroneMissionRole MissionRole = EDroneMissionRole::Unknown;

    /** @brief 是否允许 API 远程控制 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Config")
    bool bApiControlEnabled = true;

    /** @brief 根场景组件 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    USceneComponent* RootComp;

    /** @brief 机体静态网格 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    UStaticMeshComponent* BodyMesh;

    /** @brief 旋翼 0 网格 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    UStaticMeshComponent* Fan0;

    /** @brief 旋翼 1 网格 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    UStaticMeshComponent* Fan1;

    /** @brief 旋翼 2 网格 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    UStaticMeshComponent* Fan2;

    /** @brief 旋翼 3 网格 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    UStaticMeshComponent* Fan3;

    /** @brief 云台偏航轴网格 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    UStaticMeshComponent* CameraYawMesh;

    /** @brief 云台俯仰轴网格 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    UStaticMeshComponent* CameraPitchMesh;

    /** @brief 场景捕获组件，用于生成 API 图像 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Camera")
    USceneCaptureComponent2D* DroneSceneCapture;

    /** @brief 电影相机组件，用于同步后处理参数 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Camera")
    UCineCameraComponent* DroneCineCamera;

    /** @brief 相机水平视场角（deg） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera")
    float CameraFOV = 90.0f;

    /** @brief 捕获图像宽度（px） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera")
    int32 CameraWidth = 1280;

    /** @brief 捕获图像高度（px） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera")
    int32 CameraHeight = 720;

    /** @brief 语义分割模板值 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera", meta = (ClampMin = "0", ClampMax = "255"))
    int32 SegmentationId = 20;

    /** @brief 曝光补偿偏置 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera", meta = (ClampMin = "-10.0", ClampMax = "10.0"))
    float ExposureBias = 0.0f;

    /** @brief JPEG 默认编码质量 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera", meta = (ClampMin = "1", ClampMax = "100"))
    int32 JpegQuality = 90;

    /** @brief 云台角度插值速度 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera")
    float CameraRotationSpeed = 8.0f;

private:
    /**
     * @brief 把飞行状态写回到 Actor 世界变换
     * @param State 当前飞行状态
     */
    void ApplyStateToActor(const FDroneState& State);

    /**
     * @brief 更新四个旋翼的可视化转动动画
     * @param DeltaTime 帧间隔（s）
     */
    void UpdatePropellerAnimation(float DeltaTime);

    /**
     * @brief 根据索引返回对应旋翼网格
     * @param Index 旋翼索引 0-3
     * @return 对应网格指针，不存在时返回 nullptr
     */
    UStaticMeshComponent* GetFanMesh(int32 Index) const;

    /**
     * @brief 平滑插值更新云台姿态
     * @param DeltaTime 帧间隔（s）
     */
    void UpdateCameraRotation(float DeltaTime);

    /** @brief 云台目标俯仰角（deg） */
    float CameraTargetPitch = 0.0f;

    /** @brief 云台目标偏航角（deg） */
    float CameraTargetYaw = 0.0f;

    /** @brief 云台当前俯仰角（deg） */
    float CameraCurrentPitch = 0.0f;

    /** @brief 云台当前偏航角（deg） */
    float CameraCurrentYaw = 0.0f;
};
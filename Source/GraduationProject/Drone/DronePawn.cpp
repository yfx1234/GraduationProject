// 解释：引入当前实现文件对应的头文件 `DronePawn.h`，使实现部分能够看到类和函数声明。
#include "DronePawn.h"

// 解释：引入 `DroneApi.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "DroneApi.h"
// 解释：引入 `DroneMovementComponent.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "DroneMovementComponent.h"
// 解释：引入 `AgentManager.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "GraduationProject/Core/Manager/AgentManager.h"
// 解释：引入 `CameraCaptureUtils.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "GraduationProject/Vision/CameraCaptureUtils.h"

// 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
namespace
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    /**
     * @brief 坐标/姿态参数所使用的参考坐标系
     *
     * - `UE`：直接采用 Unreal 世界坐标约定；
     * - `NED`：采用 North-East-Down 约定，需要对竖直轴和相关姿态分量做符号转换。
     */
    // 解释：这一行声明枚举 `EDroneFrame`，用于约束一组有限的状态或模式取值。
    enum class EDroneFrame : uint8
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        UE,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        NED,
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    };

    /** @brief 对 JSON 字符串中的反斜杠和双引号做转义 */
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

    /** @brief 解析坐标系字符串，无法识别时默认回落到 `UE` */
    // 解释：这一行定义函数 `ParseFrame`，开始实现parseframe的具体逻辑。
    EDroneFrame ParseFrame(const FString& FrameText)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行声明成员或局部变量 `Frame`，用于保存frame。
        FString Frame = FrameText;
        // 解释：调用 `TrimStartAndEndInline` 执行当前步骤需要的功能逻辑。
        Frame.TrimStartAndEndInline();
        // 解释：调用 `ToLowerInline` 执行当前步骤需要的功能逻辑。
        Frame.ToLowerInline();
        // 解释：这一行使用三目表达式选择返回值，在正常路径和兜底路径之间做快速切换。
        return (Frame == TEXT("ned")) ? EDroneFrame::NED : EDroneFrame::UE;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    /** @brief 坐标系枚举转字符串，便于状态 JSON 输出 */
    // 解释：这一行定义函数 `FrameToString`，开始实现frametostring的具体逻辑。
    FString FrameToString(EDroneFrame Frame)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行使用三目表达式选择返回值，在正常路径和兜底路径之间做快速切换。
        return (Frame == EDroneFrame::NED) ? TEXT("ned") : TEXT("ue");
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    /**
     * @brief 将外部输入向量转换为 UE 坐标
     * @param Value 原始向量
     * @param Frame 输入坐标系
     * @return UE 坐标下的向量
     *
     * 当前 `NED -> UE` 转换的核心为：
     * $z_{ue} = -z_{ned}$。
     */
    // 解释：这一行定义函数 `ConvertInputToUE`，开始实现convertinputtoue的具体逻辑。
    FVector ConvertInputToUE(const FVector& Value, EDroneFrame Frame)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (Frame == EDroneFrame::NED)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return FVector(Value.X, Value.Y, -Value.Z);
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return Value;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    /** @brief 将 UE 内部向量转换为外部请求的输出坐标系 */
    // 解释：这一行定义函数 `ConvertUEToOutput`，开始实现convertuetooutput的具体逻辑。
    FVector ConvertUEToOutput(const FVector& Value, EDroneFrame Frame)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (Frame == EDroneFrame::NED)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return FVector(Value.X, Value.Y, -Value.Z);
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return Value;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    /**
     * @brief 将 UE 姿态角转换为目标坐标系表示
     * @param RotUE UE 坐标下的姿态
     * @param Frame 目标坐标系
     * @return 转换后的欧拉角
     *
     * 在当前约定下，`NED` 模式对俯仰与横滚取反，
     * 以匹配常见飞控/航空坐标定义。
     */
    // 解释：这一行定义函数 `ConvertUEToFrameRotator`，开始实现convertuetoframerotator的具体逻辑。
    FRotator ConvertUEToFrameRotator(const FRotator& RotUE, EDroneFrame Frame)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (Frame == EDroneFrame::NED)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return FRotator(-RotUE.Pitch, RotUE.Yaw, -RotUE.Roll);
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return RotUE;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    /** @brief 将外部姿态角转换回 UE 旋转约定 */
    // 解释：这一行定义函数 `ConvertFrameToUERotator`，开始实现convertframetouerotator的具体逻辑。
    FRotator ConvertFrameToUERotator(const FRotator& RotFrame, EDroneFrame Frame)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (Frame == EDroneFrame::NED)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return FRotator(-RotFrame.Pitch, RotFrame.Yaw, -RotFrame.Roll);
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return RotFrame;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    /** @brief 偏航控制模式转字符串，便于状态序列化 */
    // 解释：这一行定义函数 `YawModeToString`，开始实现yaw模式tostring的具体逻辑。
    FString YawModeToString(EDroneYawMode Mode)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行进入 `switch` 分支分派结构，后续会按不同枚举或状态值执行不同逻辑。
        switch (Mode)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
        // 解释：这一行声明 `switch` 中的一个分支标签，对应某个具体取值的处理路径。
        case EDroneYawMode::Hold:
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return TEXT("hold");
        // 解释：这一行声明 `switch` 中的一个分支标签，对应某个具体取值的处理路径。
        case EDroneYawMode::Angle:
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return TEXT("angle");
        // 解释：这一行声明 `switch` 中的一个分支标签，对应某个具体取值的处理路径。
        case EDroneYawMode::Rate:
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return TEXT("rate");
        // 解释：这一行声明 `switch` 中的一个分支标签，对应某个具体取值的处理路径。
        case EDroneYawMode::Auto:
        // 解释：这一行声明 `switch` 的默认分支，当前面所有 `case` 都不匹配时执行。
        default:
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return TEXT("auto");
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    /** @brief 驱动模式转字符串 */
    // 解释：这一行定义函数 `DrivetrainToString`，开始实现drivetraintostring的具体逻辑。
    FString DrivetrainToString(EDroneDrivetrainMode Mode)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return (Mode == EDroneDrivetrainMode::MaxDegreeOfFreedom)
            // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
            ? TEXT("max_degree_of_freedom")
            // 解释：这一行开始构造函数初始化列表，下面会依次初始化成员变量。
            : TEXT("forward_only");
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    /** @brief 任务角色转字符串 */
    // 解释：这一行定义函数 `RoleToString`，开始实现roletostring的具体逻辑。
    FString RoleToString(EDroneMissionRole Role)
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

    /** @brief 控制模式转字符串 */
    // 解释：这一行定义函数 `ControlModeToString`，开始实现control模式tostring的具体逻辑。
    FString ControlModeToString(EDroneControlMode Mode)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行进入 `switch` 分支分派结构，后续会按不同枚举或状态值执行不同逻辑。
        switch (Mode)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
        // 解释：这一行声明 `switch` 中的一个分支标签，对应某个具体取值的处理路径。
        case EDroneControlMode::Idle:
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return TEXT("idle");
        // 解释：这一行声明 `switch` 中的一个分支标签，对应某个具体取值的处理路径。
        case EDroneControlMode::Position:
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return TEXT("position");
        // 解释：这一行声明 `switch` 中的一个分支标签，对应某个具体取值的处理路径。
        case EDroneControlMode::Velocity:
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return TEXT("velocity");
        // 解释：这一行声明 `switch` 中的一个分支标签，对应某个具体取值的处理路径。
        case EDroneControlMode::AttitudeThrust:
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return TEXT("attitude");
        // 解释：这一行声明 `switch` 中的一个分支标签，对应某个具体取值的处理路径。
        case EDroneControlMode::MotorSpeed:
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return TEXT("motor_speed");
        // 解释：这一行声明 `switch` 中的一个分支标签，对应某个具体取值的处理路径。
        case EDroneControlMode::TorqueThrust:
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return TEXT("torque_thrust");
        // 解释：这一行声明 `switch` 的默认分支，当前面所有 `case` 都不匹配时执行。
        default:
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return TEXT("unknown");
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 构造无人机 Pawn 并创建全部可视化/控制子组件
 *
 * 这里只完成组件装配和默认属性设置，
 * 不做运行时注册、纹理创建或动力学状态初始化。
 */
// 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
ADronePawn::ADronePawn()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `PrimaryActorTick.bCanEverTick`，完成 布尔标志 canevertick 的更新。
    PrimaryActorTick.bCanEverTick = true;

    // 解释：这一行把右侧表达式的结果写入 `RootComp`，完成 rootcomp 的更新。
    RootComp = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
    // 解释：调用 `SetRootComponent` 执行当前步骤需要的功能逻辑。
    SetRootComponent(RootComp);

    // 解释：这一行把右侧表达式的结果写入 `BodyMesh`，完成 bodymesh 的更新。
    BodyMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("BodyMesh"));
    // 解释：调用 `SetupAttachment` 执行当前步骤需要的功能逻辑。
    BodyMesh->SetupAttachment(RootComp);

    // 解释：这一行把右侧表达式的结果写入 `Fan0`，完成 fan0 的更新。
    Fan0 = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Fan0"));
    // 解释：调用 `SetupAttachment` 执行当前步骤需要的功能逻辑。
    Fan0->SetupAttachment(BodyMesh, TEXT("Drone_Fan_002"));
    // 解释：这一行把右侧表达式的结果写入 `Fan1`，完成 fan1 的更新。
    Fan1 = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Fan1"));
    // 解释：调用 `SetupAttachment` 执行当前步骤需要的功能逻辑。
    Fan1->SetupAttachment(BodyMesh, TEXT("Drone_Fan1_002"));
    // 解释：这一行把右侧表达式的结果写入 `Fan2`，完成 fan2 的更新。
    Fan2 = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Fan2"));
    // 解释：调用 `SetupAttachment` 执行当前步骤需要的功能逻辑。
    Fan2->SetupAttachment(BodyMesh, TEXT("Drone_Fan2_002"));
    // 解释：这一行把右侧表达式的结果写入 `Fan3`，完成 fan3 的更新。
    Fan3 = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Fan3"));
    // 解释：调用 `SetupAttachment` 执行当前步骤需要的功能逻辑。
    Fan3->SetupAttachment(BodyMesh, TEXT("Drone_Fan3_002"));

    // 解释：这一行把右侧表达式的结果写入 `CameraYawMesh`，完成 相机yawmesh 的更新。
    CameraYawMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("CameraYawMesh"));
    // 解释：调用 `SetupAttachment` 执行当前步骤需要的功能逻辑。
    CameraYawMesh->SetupAttachment(BodyMesh, TEXT("Camera_Yaw_002"));
    // 解释：这一行把右侧表达式的结果写入 `CameraPitchMesh`，完成 相机pitchmesh 的更新。
    CameraPitchMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("CameraPitchMesh"));
    // 解释：调用 `SetupAttachment` 执行当前步骤需要的功能逻辑。
    CameraPitchMesh->SetupAttachment(CameraYawMesh, TEXT("Camera_Pitch_002"));

    // 解释：这一行把右侧表达式的结果写入 `DroneSceneCapture`，完成 无人机scene采集 的更新。
    DroneSceneCapture = CreateDefaultSubobject<USceneCaptureComponent2D>(TEXT("DroneSceneCapture"));
    // 解释：调用 `SetupAttachment` 执行当前步骤需要的功能逻辑。
    DroneSceneCapture->SetupAttachment(CameraPitchMesh);
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    DroneSceneCapture->bCaptureEveryFrame = true;
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    DroneSceneCapture->bCaptureOnMovement = false;
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    DroneSceneCapture->bAlwaysPersistRenderingState = false;
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    DroneSceneCapture->PostProcessSettings.bOverride_MotionBlurAmount = true;
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    DroneSceneCapture->PostProcessSettings.MotionBlurAmount = 0.0f;
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    DroneSceneCapture->PostProcessSettings.bOverride_MotionBlurMax = true;
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    DroneSceneCapture->PostProcessSettings.MotionBlurMax = 0.0f;
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    DroneSceneCapture->PostProcessSettings.bOverride_MotionBlurPerObjectSize = true;
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    DroneSceneCapture->PostProcessSettings.MotionBlurPerObjectSize = 0.0f;
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    DroneSceneCapture->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;

    // 解释：这一行把右侧表达式的结果写入 `DroneCineCamera`，完成 无人机cine相机 的更新。
    DroneCineCamera = CreateDefaultSubobject<UCineCameraComponent>(TEXT("DroneCineCamera"));
    // 解释：调用 `SetupAttachment` 执行当前步骤需要的功能逻辑。
    DroneCineCamera->SetupAttachment(CameraPitchMesh);
    // 解释：调用 `SetActive` 执行当前步骤需要的功能逻辑。
    DroneCineCamera->SetActive(false);

    // 解释：这一行把右侧表达式的结果写入 `MovementComp`，完成 飞行运动组件 的更新。
    MovementComp = CreateDefaultSubobject<UDroneMovementComponent>(TEXT("Movement"));
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 初始化动力学、图像采集与 API 组件
 *
 * 主要完成三件事：
 * 1. 将当前 Actor 初始位置写入动力学状态，注意 `cm -> m` 的单位换算；
 * 2. 为机载相机创建 RenderTarget，并同步分割模板值；
 * 3. 创建高层 API 对象并向 AgentManager 注册当前无人机。
 */
// 解释：这一行定义函数 `BeginPlay`，开始实现beginplay的具体逻辑。
void ADronePawn::BeginPlay()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：调用 `BeginPlay` 执行当前步骤需要的功能逻辑。
    Super::BeginPlay();

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (MovementComp)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `SetParameters` 执行当前步骤需要的功能逻辑。
        MovementComp->SetParameters(Parameters);
        // 解释：这一行声明成员或局部变量 `InitState`，用于保存init状态。
        FDroneState InitState;
        // 解释：调用 `SetPosition` 执行当前步骤需要的功能逻辑。
        InitState.SetPosition(GetActorLocation() / 100.0f);
        // 解释：调用 `SetInitialState` 执行当前步骤需要的功能逻辑。
        MovementComp->SetInitialState(InitState);
        // 解释：调用 `SetControlMode` 执行当前步骤需要的功能逻辑。
        MovementComp->SetControlMode(ControlMode);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (UTextureRenderTarget2D* RT = CameraCaptureUtils::CreateColorRenderTarget(this, CameraWidth, CameraHeight))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        DroneSceneCapture->TextureTarget = RT;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    DroneSceneCapture->FOVAngle = CameraFOV;
    // 解释：调用 `ApplySegmentationStencil` 执行当前步骤需要的功能逻辑。
    CameraCaptureUtils::ApplySegmentationStencil(this, SegmentationId);

    // 解释：这一行把右侧表达式的结果写入 `Api`，完成 接口 的更新。
    Api = NewObject<UDroneApi>(this);
    // 解释：调用 `Initialize` 执行当前步骤需要的功能逻辑。
    Api->Initialize(this);

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (UAgentManager* Manager = UAgentManager::GetInstance())
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `const FString ResolvedId`，完成 constfstringresolvedid 的更新。
        const FString ResolvedId = Manager->RegisterOrResolveAgent(DroneId, this);
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (!ResolvedId.IsEmpty())
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行把右侧表达式的结果写入 `DroneId`，完成 无人机id 的更新。
            DroneId = ResolvedId;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 每帧同步飞行状态、旋翼动画和云台姿态
 * @param DeltaTime 帧间隔（s）
 */
// 解释：这一行定义函数 `Tick`，开始实现tick的具体逻辑。
void ADronePawn::Tick(float DeltaTime)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：调用 `Tick` 执行当前步骤需要的功能逻辑。
    Super::Tick(DeltaTime);

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (MovementComp)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `CurrentState`，完成 current状态 的更新。
        CurrentState = MovementComp->GetCurrentState();
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：调用 `ApplyStateToActor` 执行当前步骤需要的功能逻辑。
    ApplyStateToActor(CurrentState);
    // 解释：调用 `UpdatePropellerAnimation` 执行当前步骤需要的功能逻辑。
    UpdatePropellerAnimation(DeltaTime);
    // 解释：调用 `UpdateCameraRotation` 执行当前步骤需要的功能逻辑。
    UpdateCameraRotation(DeltaTime);
    // 解释：调用 `SyncPostProcessToCapture` 执行当前步骤需要的功能逻辑。
    CameraCaptureUtils::SyncPostProcessToCapture(DroneCineCamera, DroneSceneCapture, ExposureBias, true);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 将动力学状态写回 Actor 世界变换
 * @param State 当前飞行状态
 *
 * 动力学内部以米为单位，UE 世界位置以厘米为单位，故有：
 * $p_{ue} = 100 \cdot p_{sim}$。
 */
// 解释：这一行定义函数 `ApplyStateToActor`，开始实现apply状态toActor的具体逻辑。
void ADronePawn::ApplyStateToActor(const FDroneState& State)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：调用 `SetActorLocationAndRotation` 执行当前步骤需要的功能逻辑。
    SetActorLocationAndRotation(State.GetPosition() * 100.0f, State.GetRotator());
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 根据电机角速度刷新旋翼可视化动画
 * @param DeltaTime 帧间隔（s）
 *
 * 若电机角速度为 $\omega$（rad/s），则：
 * - `RPM = \omega * 60 / (2\pi)`
 * - $\Delta\theta = RPM * 360 / 60 * \Delta t = \omega * 180 / \pi * \Delta t$
 *
 * 同时通过奇偶号电机的方向符号差异模拟顺/逆时针旋转。
 */
// 解释：这一行定义函数 `UpdatePropellerAnimation`，开始实现updatepropelleranimation的具体逻辑。
void ADronePawn::UpdatePropellerAnimation(float DeltaTime)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (CurrentState.MotorSpeeds.Num() < 4)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
    for (int32 Index = 0; Index < 4; ++Index)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `UStaticMeshComponent* Fan`，完成 ustaticmesh组件fan 的更新。
        UStaticMeshComponent* Fan = GetFanMesh(Index);
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (!Fan)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行跳过本轮循环剩余语句，直接进入下一轮迭代。
            continue;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行把右侧表达式的结果写入 `const float RPM`，完成 constfloatrpm 的更新。
        const float RPM = CurrentState.MotorSpeeds[Index] * 60.0f / (2.0f * PI);
        // 解释：这一行声明成员或局部变量 `DeltaAngle`，用于保存deltaangle。
        const float DeltaAngle = RPM * 360.0f / 60.0f * DeltaTime;
        // 解释：这一行把右侧表达式的结果写入 `const float Direction`，完成 constfloatdirection 的更新。
        const float Direction = (Index % 2 == 0) ? 1.0f : -1.0f;
        // 解释：调用 `AddLocalRotation` 执行当前步骤需要的功能逻辑。
        Fan->AddLocalRotation(FRotator(0.0f, DeltaAngle * Direction, 0.0f));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 按索引获取对应旋翼网格组件 */
// 解释：这一行定义函数 `GetFanMesh`，开始实现getfanmesh的具体逻辑。
UStaticMeshComponent* ADronePawn::GetFanMesh(int32 Index) const
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行进入 `switch` 分支分派结构，后续会按不同枚举或状态值执行不同逻辑。
    switch (Index)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
    // 解释：这一行声明 `switch` 中的一个分支标签，对应某个具体取值的处理路径。
    case 0:
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return Fan0;
    // 解释：这一行声明 `switch` 中的一个分支标签，对应某个具体取值的处理路径。
    case 1:
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return Fan1;
    // 解释：这一行声明 `switch` 中的一个分支标签，对应某个具体取值的处理路径。
    case 2:
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return Fan2;
    // 解释：这一行声明 `switch` 中的一个分支标签，对应某个具体取值的处理路径。
    case 3:
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return Fan3;
    // 解释：这一行声明 `switch` 的默认分支，当前面所有 `case` 都不匹配时执行。
    default:
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return nullptr;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 切换到位置控制模式并设置目标位置
 * @param NewTargetPosition 目标位置（m）
 * @param Speed 期望速度上限（m/s）
 * @param Frame 输入坐标系
 */
// 解释：这一行定义函数 `SetTargetPosition`，开始实现settargetposition的具体逻辑。
void ADronePawn::SetTargetPosition(const FVector& NewTargetPosition, float Speed, FString Frame)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!MovementComp)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (ControlMode != EDroneControlMode::Position)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `SetControlMode` 执行当前步骤需要的功能逻辑。
        MovementComp->SetControlMode(EDroneControlMode::Position);
        // 解释：这一行把右侧表达式的结果写入 `ControlMode`，完成 control模式 的更新。
        ControlMode = EDroneControlMode::Position;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行把右侧表达式的结果写入 `const FVector TargetUE`，完成 constfvectortargetue 的更新。
    const FVector TargetUE = ConvertInputToUE(NewTargetPosition, ParseFrame(Frame));
    // 解释：调用 `SetTargetPosition` 执行当前步骤需要的功能逻辑。
    MovementComp->SetTargetPosition(TargetUE, Speed);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 切换到速度控制模式并设置目标速度
 * @param NewTargetVelocity 目标速度（m/s）
 * @param Frame 输入坐标系
 */
// 解释：这一行定义函数 `SetTargetVelocity`，开始实现settargetvelocity的具体逻辑。
void ADronePawn::SetTargetVelocity(const FVector& NewTargetVelocity, FString Frame)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!MovementComp)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (ControlMode != EDroneControlMode::Velocity)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `SetControlMode` 执行当前步骤需要的功能逻辑。
        MovementComp->SetControlMode(EDroneControlMode::Velocity);
        // 解释：这一行把右侧表达式的结果写入 `ControlMode`，完成 control模式 的更新。
        ControlMode = EDroneControlMode::Velocity;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行把右侧表达式的结果写入 `const FVector VelocityUE`，完成 constfvectorvelocityue 的更新。
    const FVector VelocityUE = ConvertInputToUE(NewTargetVelocity, ParseFrame(Frame));
    // 解释：调用 `SetTargetVelocity` 执行当前步骤需要的功能逻辑。
    MovementComp->SetTargetVelocity(VelocityUE);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 标量形式的速度控制便捷封装 */
// 解释：这一行定义函数 `MoveByVelocity`，开始实现movebyvelocity的具体逻辑。
void ADronePawn::MoveByVelocity(float Vx, float Vy, float Vz, FString Frame)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：调用 `SetTargetVelocity` 执行当前步骤需要的功能逻辑。
    SetTargetVelocity(FVector(Vx, Vy, Vz), MoveTemp(Frame));
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 仅更新偏航控制策略，不直接修改平移控制目标 */
// 解释：这一行定义函数 `SetHeadingControl`，开始实现setheadingcontrol的具体逻辑。
void ADronePawn::SetHeadingControl(EDroneYawMode NewYawMode, EDroneDrivetrainMode NewDrivetrain, float YawDeg)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (MovementComp)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `SetHeadingControl` 执行当前步骤需要的功能逻辑。
        MovementComp->SetHeadingControl(NewYawMode, NewDrivetrain, YawDeg);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 当前位置悬停，本质上是把当前位置重新设为位置控制目标 */
// 解释：这一行定义函数 `Hover`，开始实现hover的具体逻辑。
void ADronePawn::Hover()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：调用 `SetTargetPosition` 执行当前步骤需要的功能逻辑。
    SetTargetPosition(CurrentState.GetPosition());
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 在当前平面位置上抬升到指定高度 */
// 解释：这一行定义函数 `Takeoff`，开始实现takeoff的具体逻辑。
void ADronePawn::Takeoff(float Altitude)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `const FVector CurrentPos`，完成 constfvectorcurrentpos 的更新。
    const FVector CurrentPos = CurrentState.GetPosition();
    // 解释：调用 `SetTargetPosition` 执行当前步骤需要的功能逻辑。
    SetTargetPosition(FVector(CurrentPos.X, CurrentPos.Y, Altitude));
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 在当前平面位置下降到地面高度 `z=0` */
// 解释：这一行定义函数 `Land`，开始实现land的具体逻辑。
void ADronePawn::Land()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `const FVector CurrentPos`，完成 constfvectorcurrentpos 的更新。
    const FVector CurrentPos = CurrentState.GetPosition();
    // 解释：调用 `SetTargetPosition` 执行当前步骤需要的功能逻辑。
    SetTargetPosition(FVector(CurrentPos.X, CurrentPos.Y, 0.0f));
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 更新远程 API 控制标志位 */
// 解释：这一行定义函数 `EnableApiControl`，开始实现enable接口control的具体逻辑。
void ADronePawn::EnableApiControl(bool bEnable)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `bApiControlEnabled`，完成 布尔标志 接口controlenabled 的更新。
    bApiControlEnabled = bEnable;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 强制把无人机重置到指定位置和姿态
 * @param NewLocation 目标位置（m）
 * @param NewRotation 目标姿态
 * @param Frame 输入坐标系
 *
 * 该接口会同时：
 * - 重置动力学组件内部状态；
 * - 将控制模式恢复到 `Idle`；
 * - 立即把新状态同步到 Actor 变换。
 */
// 解释：这一行定义函数 `ResetDrone`，开始实现reset无人机的具体逻辑。
void ADronePawn::ResetDrone(const FVector& NewLocation, const FRotator& NewRotation, FString Frame)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行声明成员或局部变量 `NewState`，用于保存new状态。
    FDroneState NewState;
    // 解释：调用 `SetPosition` 执行当前步骤需要的功能逻辑。
    NewState.SetPosition(ConvertInputToUE(NewLocation, ParseFrame(Frame)));
    // 解释：调用 `SetQuaternion` 执行当前步骤需要的功能逻辑。
    NewState.SetQuaternion(ConvertFrameToUERotator(NewRotation, ParseFrame(Frame)).Quaternion());
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (MovementComp)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `ResetState` 执行当前步骤需要的功能逻辑。
        MovementComp->ResetState(NewState);
        // 解释：调用 `SetControlMode` 执行当前步骤需要的功能逻辑。
        MovementComp->SetControlMode(EDroneControlMode::Idle);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行把右侧表达式的结果写入 `CurrentState`，完成 current状态 的更新。
    CurrentState = NewState;
    // 解释：这一行把右侧表达式的结果写入 `ControlMode`，完成 control模式 的更新。
    ControlMode = EDroneControlMode::Idle;
    // 解释：调用 `ApplyStateToActor` 执行当前步骤需要的功能逻辑。
    ApplyStateToActor(NewState);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 重置 API 层内部控制状态，但不直接改动当前位置 */
// 解释：这一行定义函数 `ResetActorState`，开始实现resetActor状态的具体逻辑。
void ADronePawn::ResetActorState()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (Api)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `Reset` 执行当前步骤需要的功能逻辑。
        Api->Reset();
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 设置姿态角与总推力控制目标
 * @param RollDeg 横滚角（deg）
 * @param PitchDeg 俯仰角（deg）
 * @param YawDeg 偏航角（deg）
 * @param Thrust 总推力（N）
 * @param Frame 输入坐标系
 *
 * 外部常用 `NED` 姿态表达，因此这里先完成坐标系转换，
 * 再交给 `UDroneApi` 写入姿态控制链路。
 */
// 解释：这一行定义函数 `SetTargetAttitude`，开始实现settargetattitude的具体逻辑。
void ADronePawn::SetTargetAttitude(float RollDeg, float PitchDeg, float YawDeg, float Thrust, FString Frame)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!Api)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行把右侧表达式的结果写入 `const FRotator UERot`，完成 constfrotatoruerot 的更新。
    const FRotator UERot = ConvertFrameToUERotator(FRotator(PitchDeg, YawDeg, RollDeg), ParseFrame(Frame));
    // 解释：调用 `SetTargetAttitude` 执行当前步骤需要的功能逻辑。
    Api->SetTargetAttitude(UERot.Roll, UERot.Pitch, UERot.Yaw, Thrust);
    // 解释：这一行把右侧表达式的结果写入 `ControlMode`，完成 control模式 的更新。
    ControlMode = EDroneControlMode::AttitudeThrust;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 直接下发四路电机角速度控制命令 */
// 解释：这一行定义函数 `SetMotorSpeeds`，开始实现setmotorspeeds的具体逻辑。
void ADronePawn::SetMotorSpeeds(float M0, float M1, float M2, float M3)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!Api)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：调用 `SetMotorSpeeds` 执行当前步骤需要的功能逻辑。
    Api->SetMotorSpeeds(M0, M1, M2, M3);
    // 解释：这一行把右侧表达式的结果写入 `ControlMode`，完成 control模式 的更新。
    ControlMode = EDroneControlMode::MotorSpeed;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 设置位置环控制器增益 */
// 解释：这一行定义函数 `SetPositionControllerGains`，开始实现setposition控制器gains的具体逻辑。
void ADronePawn::SetPositionControllerGains(float Kp, float Kd)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (Api)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `SetPositionControllerGains` 执行当前步骤需要的功能逻辑。
        Api->SetPositionControllerGains(Kp, Kd);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 设置速度环控制器增益 */
// 解释：这一行定义函数 `SetVelocityControllerGains`，开始实现setvelocity控制器gains的具体逻辑。
void ADronePawn::SetVelocityControllerGains(float Kp, float Ki, float Kd)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (Api)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `SetVelocityControllerGains` 执行当前步骤需要的功能逻辑。
        Api->SetVelocityControllerGains(Kp, Ki, Kd);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 设置姿态环控制器增益 */
// 解释：这一行定义函数 `SetAttitudeControllerGains`，开始实现setattitude控制器gains的具体逻辑。
void ADronePawn::SetAttitudeControllerGains(float Kp, float Kd)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (Api)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `SetAttitudeControllerGains` 执行当前步骤需要的功能逻辑。
        Api->SetAttitudeControllerGains(Kp, Kd);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 设置角速度环比例增益 */
// 解释：这一行定义函数 `SetAngleRateControllerGains`，开始实现setanglerate控制器gains的具体逻辑。
void ADronePawn::SetAngleRateControllerGains(float Kp)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (Api)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `SetAngleRateControllerGains` 执行当前步骤需要的功能逻辑。
        Api->SetAngleRateControllerGains(Kp);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 返回当前状态中的位置分量 */
// 解释：这一行定义函数 `GetCurrentPosition`，开始实现getcurrentposition的具体逻辑。
FVector ADronePawn::GetCurrentPosition() const
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return CurrentState.GetPosition();
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 返回当前状态中的速度分量 */
// 解释：这一行定义函数 `GetCurrentVelocity`，开始实现getcurrentvelocity的具体逻辑。
FVector ADronePawn::GetCurrentVelocity() const
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return CurrentState.GetVelocity();
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 写入云台目标角，实际转动由 Tick 中插值完成 */
// 解释：这一行定义函数 `SetCameraAngles`，开始实现set相机angles的具体逻辑。
void ADronePawn::SetCameraAngles(float TargetPitch, float TargetYaw)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `CameraTargetPitch`，完成 相机targetpitch 的更新。
    CameraTargetPitch = TargetPitch;
    // 解释：这一行把右侧表达式的结果写入 `CameraTargetYaw`，完成 相机targetyaw 的更新。
    CameraTargetYaw = TargetYaw;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 更新该机体的语义分割模板值并同步到场景捕获对象 */
// 解释：这一行定义函数 `SetSegmentationId`，开始实现setsegmentationid的具体逻辑。
void ADronePawn::SetSegmentationId(int32 NewSegmentationId)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行先对计算结果做限幅，再写入 `SegmentationId`，防止 segmentationid 超出允许范围。
    SegmentationId = FMath::Clamp(NewSegmentationId, 0, 255);
    // 解释：调用 `ApplySegmentationStencil` 执行当前步骤需要的功能逻辑。
    CameraCaptureUtils::ApplySegmentationStencil(this, SegmentationId);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 以 JSON 形式导出当前无人机状态
 * @param Frame 输出坐标系
 * @return 状态快照 JSON
 *
 * 该接口会按请求坐标系转换位置、速度与姿态，
 * 并附带电机转速、偏航模式、任务角色、云台角和控制模式等信息。
 */
// 解释：这一行定义函数 `GetState`，开始实现get状态的具体逻辑。
FString ADronePawn::GetState(FString Frame)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `const EDroneFrame OutputFrame`，完成 constedroneframeoutputframe 的更新。
    const EDroneFrame OutputFrame = ParseFrame(Frame);
    // 解释：这一行把右侧表达式的结果写入 `const FVector Pos`，完成 constfvectorpos 的更新。
    const FVector Pos = ConvertUEToOutput(GetCurrentPosition(), OutputFrame);
    // 解释：这一行把右侧表达式的结果写入 `const FVector Vel`，完成 constfvectorvel 的更新。
    const FVector Vel = ConvertUEToOutput(GetCurrentVelocity(), OutputFrame);
    // 解释：这一行把右侧表达式的结果写入 `const FRotator Rot`，完成 constfrotatorrot 的更新。
    const FRotator Rot = ConvertUEToFrameRotator(CurrentState.GetRotator(), OutputFrame);

    // 解释：这一行声明成员或局部变量 `YawMode`，用于保存yaw模式。
    EDroneYawMode YawMode = EDroneYawMode::Auto;
    // 解释：这一行声明成员或局部变量 `Drivetrain`，用于保存drivetrain。
    EDroneDrivetrainMode Drivetrain = EDroneDrivetrainMode::MaxDegreeOfFreedom;
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (MovementComp)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `YawMode`，完成 yaw模式 的更新。
        YawMode = MovementComp->GetYawMode();
        // 解释：这一行把右侧表达式的结果写入 `Drivetrain`，完成 drivetrain 的更新。
        Drivetrain = MovementComp->GetDrivetrainMode();
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：调用 `GetMotorSpeeds` 执行当前步骤需要的功能逻辑。
    const TArray<float> Motors = Api ? Api->GetMotorSpeeds() : TArray<float>();
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return FString::Printf(
        // 解释：这一行位于构造函数初始化列表中，把 `TEXT` 直接初始化为 `"{\"status\":\"ok\",\"id\":\"%s\",\"frame\":\"%s\",\"orientation_frame\":\"%s\",\"position\":[%.4f,%.4f,%.4f],\"velocity\":[%.4f,%.4f,%.4f],\"orientation\":{\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f},\"motor_speeds\":[%.1f,%.1f,%.1f,%.1f],\"yaw_mode\":\"%s\",\"drivetrain\":\"%s\",\"role\":\"%s\",\"camera_pitch\":%.2f,\"camera_yaw\":%.2f,\"control_mode\":\"%s\",\"api_control\":%s}"`，减少进入函数体后的额外赋值开销。
        TEXT("{\"status\":\"ok\",\"id\":\"%s\",\"frame\":\"%s\",\"orientation_frame\":\"%s\",\"position\":[%.4f,%.4f,%.4f],\"velocity\":[%.4f,%.4f,%.4f],\"orientation\":{\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f},\"motor_speeds\":[%.1f,%.1f,%.1f,%.1f],\"yaw_mode\":\"%s\",\"drivetrain\":\"%s\",\"role\":\"%s\",\"camera_pitch\":%.2f,\"camera_yaw\":%.2f,\"control_mode\":\"%s\",\"api_control\":%s}"),
        *JsonEscape(DroneId),
        *FrameToString(OutputFrame),
        *FrameToString(OutputFrame),
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Pos.X,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Pos.Y,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Pos.Z,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Vel.X,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Vel.Y,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Vel.Z,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Rot.Roll,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Rot.Pitch,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Rot.Yaw,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Motors.Num() >= 4 ? Motors[0] : 0.0f,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Motors.Num() >= 4 ? Motors[1] : 0.0f,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Motors.Num() >= 4 ? Motors[2] : 0.0f,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Motors.Num() >= 4 ? Motors[3] : 0.0f,
        *YawModeToString(YawMode),
        *DrivetrainToString(Drivetrain),
        *RoleToString(MissionRole),
        // 解释：这一行位于构造函数初始化列表中，把 `GetCameraCurrentPitch` 直接初始化为 ``，减少进入函数体后的额外赋值开销。
        GetCameraCurrentPitch(),
        // 解释：这一行位于构造函数初始化列表中，把 `GetCameraCurrentYaw` 直接初始化为 ``，减少进入函数体后的额外赋值开销。
        GetCameraCurrentYaw(),
        *ControlModeToString(ControlMode),
        // 解释：调用 `TEXT` 执行当前步骤需要的功能逻辑。
        bApiControlEnabled ? TEXT("true") : TEXT("false"));
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 调用图像工具抓取当前机载相机画面，并按 AirSim 风格封装成 JSON */
// 解释：这一行定义函数 `GetImage`，开始实现get图像的具体逻辑。
FString ADronePawn::GetImage(FString ImageType, int32 Quality, float MaxDepthMeters)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return CameraCaptureUtils::CaptureAirSimImageJson(
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        DroneSceneCapture,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        DroneId,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        CameraWidth,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        CameraHeight,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        CameraFOV,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        JpegQuality,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        ImageType,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Quality,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        MaxDepthMeters);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 抓取彩色图像并返回 Base64 JPEG */
// 解释：这一行定义函数 `CaptureImageBase64`，开始实现采集图像base64的具体逻辑。
FString ADronePawn::CaptureImageBase64(int32 Quality)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (Quality <= 0)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `Quality`，完成 quality 的更新。
        Quality = JpegQuality;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return CameraCaptureUtils::CaptureColorJpegBase64(DroneSceneCapture, CameraWidth, CameraHeight, Quality);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 以一阶插值方式平滑更新云台偏航/俯仰角
 * @param DeltaTime 帧间隔（s）
 *
 * 偏航先通过 `NormalizeAxis` 求最短角差，再做插值，
 * 避免跨越 `-180/180` 度边界时出现跳变。
 */
// 解释：这一行定义函数 `UpdateCameraRotation`，开始实现update相机rotation的具体逻辑。
void ADronePawn::UpdateCameraRotation(float DeltaTime)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `const float YawDiff`，完成 constfloatyawdiff 的更新。
    const float YawDiff = FRotator::NormalizeAxis(CameraTargetYaw - CameraCurrentYaw);
    // 解释：这一行声明成员或局部变量 `AdjustedTargetYaw`，用于保存adjustedtargetyaw。
    const float AdjustedTargetYaw = CameraCurrentYaw + YawDiff;
    // 解释：这一行把右侧表达式的结果写入 `CameraCurrentYaw`，完成 相机currentyaw 的更新。
    CameraCurrentYaw = FMath::FInterpTo(CameraCurrentYaw, AdjustedTargetYaw, DeltaTime, CameraRotationSpeed);
    // 解释：这一行把右侧表达式的结果写入 `CameraCurrentPitch`，完成 相机currentpitch 的更新。
    CameraCurrentPitch = FMath::FInterpTo(CameraCurrentPitch, CameraTargetPitch, DeltaTime, CameraRotationSpeed);

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (CameraYawMesh)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `SetRelativeRotation` 执行当前步骤需要的功能逻辑。
        CameraYawMesh->SetRelativeRotation(FRotator(0.0f, CameraCurrentYaw, 0.0f));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (CameraPitchMesh)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `SetRelativeRotation` 执行当前步骤需要的功能逻辑。
        CameraPitchMesh->SetRelativeRotation(FRotator(0.0f, 0.0f, -CameraCurrentPitch));
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

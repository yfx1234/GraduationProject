// 解释：引入当前实现文件对应的头文件 `CameraPawn.h`，使实现部分能够看到类和函数声明。
#include "CameraPawn.h"

// 解释：引入 `InputComponent.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "Components/InputComponent.h"
// 解释：引入 `World.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "Engine/World.h"
// 解释：引入 `PlayerController.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "GameFramework/PlayerController.h"
// 解释：引入 `AgentManager.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "GraduationProject/Core/Manager/AgentManager.h"
// 解释：引入 `DronePawn.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "GraduationProject/Drone/DronePawn.h"
// 解释：引入 `InputCoreTypes.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "InputCoreTypes.h"

/**
 * @brief 构造自由相机 Pawn
 * 创建根节点、弹簧臂和主相机，并默认接管 Player0 输入。
 */
// 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
ACameraPawn::ACameraPawn()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `PrimaryActorTick.bCanEverTick`，完成 布尔标志 canevertick 的更新。
    PrimaryActorTick.bCanEverTick = true;

    // 解释：这一行把右侧表达式的结果写入 `RootComp`，完成 rootcomp 的更新。
    RootComp = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
    // 解释：调用 `SetRootComponent` 执行当前步骤需要的功能逻辑。
    SetRootComponent(RootComp);

    // 解释：这一行把右侧表达式的结果写入 `SpringArm`，完成 springarm 的更新。
    SpringArm = CreateDefaultSubobject<USpringArmComponent>(TEXT("SpringArm"));
    // 解释：调用 `SetupAttachment` 执行当前步骤需要的功能逻辑。
    SpringArm->SetupAttachment(RootComp);
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    SpringArm->TargetArmLength = 0.0f;
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    SpringArm->bDoCollisionTest = false;

    // 解释：这一行把右侧表达式的结果写入 `Camera`，完成 相机 的更新。
    Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera"));
    // 解释：调用 `SetupAttachment` 执行当前步骤需要的功能逻辑。
    Camera->SetupAttachment(SpringArm);

    // Keep camera controls available even if a placed CameraPawn is used in map.
    // 解释：这一行把右侧表达式的结果写入 `AutoPossessPlayer`，完成 autopossessplayer 的更新。
    AutoPossessPlayer = EAutoReceiveInput::Player0;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 初始化观察相机输入模式
 * 进入游戏后默认隐藏鼠标，并切换到纯游戏输入。
 */
// 解释：这一行定义函数 `BeginPlay`，开始实现beginplay的具体逻辑。
void ACameraPawn::BeginPlay()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：调用 `BeginPlay` 执行当前步骤需要的功能逻辑。
    Super::BeginPlay();

    // 解释：这一行把右侧表达式的结果写入 `APlayerController* PC`，完成 aplayer控制器pc 的更新。
    APlayerController* PC = Cast<APlayerController>(GetController());
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!PC && GetWorld())
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `PC`，完成 pc 的更新。
        PC = GetWorld()->GetFirstPlayerController();
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (PC)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        PC->bShowMouseCursor = false;
        // 解释：调用 `SetInputMode` 执行当前步骤需要的功能逻辑。
        PC->SetInputMode(FInputModeGameOnly());
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 每帧更新观察相机位置
 * @param DeltaTime 帧间隔（秒）
 * 根据当前模式在目标环绕、俯视、第一人称和自由移动之间切换。
 */
// 解释：这一行定义函数 `Tick`，开始实现tick的具体逻辑。
void ACameraPawn::Tick(float DeltaTime)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：调用 `Tick` 执行当前步骤需要的功能逻辑。
    Super::Tick(DeltaTime);

    // 解释：这一行把右侧表达式的结果写入 `const bool bTopDownMode`，完成 constboolBtopdown模式 的更新。
    const bool bTopDownMode = (!bIsTracking && !ActiveDroneId.IsEmpty() && DroneViewMode == EDroneViewCycleMode::TopDown);
    // 解释：这一行把右侧表达式的结果写入 `const bool bFPVMode`，完成 constboolBfpvmode 的更新。
    const bool bFPVMode = (!bIsTracking && !ActiveDroneId.IsEmpty() && DroneViewMode == EDroneViewCycleMode::FPV);

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (bIsTracking && TrackingTarget)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `UpdateTrackingCamera` 执行当前步骤需要的功能逻辑。
        UpdateTrackingCamera(DeltaTime);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行声明补充分支，当上一个条件不成立且当前条件成立时执行。
    else if (bTopDownMode)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `UpdateTopDownCamera` 执行当前步骤需要的功能逻辑。
        UpdateTopDownCamera();
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行声明补充分支，当上一个条件不成立且当前条件成立时执行。
    else if (bFPVMode)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `UpdateFPVCamera` 执行当前步骤需要的功能逻辑。
        UpdateFPVCamera();
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行给出兜底分支，当上面的条件都不满足时执行。
    else
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (!InputMoveDirection.IsNearlyZero())
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行把右侧表达式的结果写入 `const FRotator CameraRotation`，完成 constfrotator相机rotation 的更新。
            const FRotator CameraRotation = GetActorRotation();
            // 解释：这一行把右侧表达式的结果写入 `const FVector Forward`，完成 constfvectorforward 的更新。
            const FVector Forward = CameraRotation.Vector();
            // 解释：这一行把右侧表达式的结果写入 `const FVector Right`，完成 constfvectorright 的更新。
            const FVector Right = FRotationMatrix(CameraRotation).GetScaledAxis(EAxis::Y);
            // 解释：这一行声明成员或局部变量 `Up`，用于保存up。
            const FVector Up = FVector::UpVector;
            // 解释：这一行声明成员或局部变量 `Movement`，用于保存运动。
            const FVector Movement =
                // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                Forward * InputMoveDirection.X +
                // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                Right * InputMoveDirection.Y +
                // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                Up * InputMoveDirection.Z;

            // 解释：调用 `AddActorWorldOffset` 执行当前步骤需要的功能逻辑。
            AddActorWorldOffset(Movement * MoveSpeed * DeltaTime);
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行把右侧表达式的结果写入 `InputMoveDirection`，完成 inputmovedirection 的更新。
    InputMoveDirection = FVector::ZeroVector;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 绑定输入轴和快捷键
 * @param PlayerInputComponent 输入组件
 */
// 解释：这一行定义函数 `SetupPlayerInputComponent`，开始实现setupplayerinput组件的具体逻辑。
void ACameraPawn::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：调用 `SetupPlayerInputComponent` 执行当前步骤需要的功能逻辑。
    Super::SetupPlayerInputComponent(PlayerInputComponent);

    // 解释：调用 `BindAxis` 执行当前步骤需要的功能逻辑。
    PlayerInputComponent->BindAxis("MoveForward", this, &ACameraPawn::MoveForward);
    // 解释：调用 `BindAxis` 执行当前步骤需要的功能逻辑。
    PlayerInputComponent->BindAxis("MoveRight", this, &ACameraPawn::MoveRight);
    // 解释：调用 `BindAxis` 执行当前步骤需要的功能逻辑。
    PlayerInputComponent->BindAxis("MoveUp", this, &ACameraPawn::MoveUp);
    // 解释：调用 `BindAxis` 执行当前步骤需要的功能逻辑。
    PlayerInputComponent->BindAxis("CameraPitch", this, &ACameraPawn::CameraPitch);
    // 解释：调用 `BindAxis` 执行当前步骤需要的功能逻辑。
    PlayerInputComponent->BindAxis("CameraYaw", this, &ACameraPawn::CameraYaw);

    // 解释：调用 `BindAction` 执行当前步骤需要的功能逻辑。
    PlayerInputComponent->BindAction("ZoomIn", IE_Pressed, this, &ACameraPawn::OnZoomIn);
    // 解释：调用 `BindAction` 执行当前步骤需要的功能逻辑。
    PlayerInputComponent->BindAction("ZoomOut", IE_Pressed, this, &ACameraPawn::OnZoomOut);

    // AirSim-like view hotkeys.
    // 解释：调用 `BindKey` 执行当前步骤需要的功能逻辑。
    PlayerInputComponent->BindKey(EKeys::F, IE_Pressed, this, &ACameraPawn::OnCycleDroneView);
    // 解释：调用 `BindKey` 执行当前步骤需要的功能逻辑。
    PlayerInputComponent->BindKey(EKeys::M, IE_Pressed, this, &ACameraPawn::OnSwitchFreeView);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 记录前后移动输入 */
// 解释：这一行定义函数 `MoveForward`，开始实现moveforward的具体逻辑。
void ACameraPawn::MoveForward(float Value)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行在 `InputMoveDirection.X` 的原有基础上继续累加新量，用于持续更新 状态向量 X。
    InputMoveDirection.X += Value;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 记录左右平移输入 */
// 解释：这一行定义函数 `MoveRight`，开始实现moveright的具体逻辑。
void ACameraPawn::MoveRight(float Value)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行在 `InputMoveDirection.Y` 的原有基础上继续累加新量，用于持续更新 Y。
    InputMoveDirection.Y += Value;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 记录上下移动输入 */
// 解释：这一行定义函数 `MoveUp`，开始实现moveup的具体逻辑。
void ACameraPawn::MoveUp(float Value)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行在 `InputMoveDirection.Z` 的原有基础上继续累加新量，用于持续更新 Z。
    InputMoveDirection.Z += Value;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 处理俯仰输入
 * @param Value 输入量
 * 跟踪模式下调整环绕俯仰角，自由模式下直接修改相机朝向。
 */
// 解释：这一行定义函数 `CameraPitch`，开始实现相机pitch的具体逻辑。
void ACameraPawn::CameraPitch(float Value)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (FMath::Abs(Value) <= 0.01f)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (bIsTracking)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行先对计算结果做限幅，再写入 `OrbitPitch`，防止 orbitpitch 超出允许范围。
        OrbitPitch = FMath::Clamp(OrbitPitch + Value * MouseSensitivity, -89.0f, 89.0f);
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!ActiveDroneId.IsEmpty() && DroneViewMode != EDroneViewCycleMode::Chase)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行把右侧表达式的结果写入 `FRotator CurrentRot`，完成 frotatorcurrentrot 的更新。
    FRotator CurrentRot = GetActorRotation();
    // 解释：这一行先对计算结果做限幅，再写入 `CurrentRot.Pitch`，防止 pitch 超出允许范围。
    CurrentRot.Pitch = FMath::Clamp(CurrentRot.Pitch + Value * MouseSensitivity, -89.0f, 89.0f);
    // 解释：调用 `SetActorRotation` 执行当前步骤需要的功能逻辑。
    SetActorRotation(CurrentRot);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 处理偏航输入
 * @param Value 输入量
 * 跟踪模式下仅累积轨道偏航角，其余模式下直接旋转相机。
 */
// 解释：这一行定义函数 `CameraYaw`，开始实现相机yaw的具体逻辑。
void ACameraPawn::CameraYaw(float Value)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (FMath::Abs(Value) <= 0.01f)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (bIsTracking)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行在 `OrbitYaw` 的原有基础上继续累加新量，用于持续更新 orbityaw。
        OrbitYaw += Value * MouseSensitivity;
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!ActiveDroneId.IsEmpty() && DroneViewMode != EDroneViewCycleMode::Chase)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：调用 `AddActorWorldRotation` 执行当前步骤需要的功能逻辑。
    AddActorWorldRotation(FRotator(0.0f, Value * MouseSensitivity, 0.0f));
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 缩小当前跟踪或俯视距离 */
// 解释：这一行定义函数 `OnZoomIn`，开始实现onzoomin的具体逻辑。
void ACameraPawn::OnZoomIn()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (bIsTracking || (!ActiveDroneId.IsEmpty() && DroneViewMode == EDroneViewCycleMode::TopDown))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行先对计算结果做限幅，再写入 `TrackingDistance`，防止 trackingdistance 超出允许范围。
        TrackingDistance = FMath::Clamp(TrackingDistance - ZoomSpeed, MinTrackingDistance, MaxTrackingDistance);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 放大当前跟踪或俯视距离 */
// 解释：这一行定义函数 `OnZoomOut`，开始实现onzoomout的具体逻辑。
void ACameraPawn::OnZoomOut()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (bIsTracking || (!ActiveDroneId.IsEmpty() && DroneViewMode == EDroneViewCycleMode::TopDown))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行先对计算结果做限幅，再写入 `TrackingDistance`，防止 trackingdistance 超出允许范围。
        TrackingDistance = FMath::Clamp(TrackingDistance + ZoomSpeed, MinTrackingDistance, MaxTrackingDistance);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 开始跟踪指定目标
 * @param Target 目标 Actor
 * 会重置环绕角和距离，并在目标是无人机时同步激活无人机视角模式。
 */
// 解释：这一行定义函数 `StartTracking`，开始实现starttracking的具体逻辑。
void ACameraPawn::StartTracking(AActor* Target)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!Target)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行把右侧表达式的结果写入 `TrackingTarget`，完成 trackingtarget 的更新。
    TrackingTarget = Target;
    // 解释：这一行把右侧表达式的结果写入 `bIsTracking`，完成 布尔标志 istracking 的更新。
    bIsTracking = true;
    // 解释：这一行把右侧表达式的结果写入 `OrbitYaw`，完成 orbityaw 的更新。
    OrbitYaw = 0.0f;
    // 解释：这一行把右侧表达式的结果写入 `OrbitPitch`，完成 orbitpitch 的更新。
    OrbitPitch = -30.0f;
    // 解释：这一行把右侧表达式的结果写入 `TrackingDistance`，完成 trackingdistance 的更新。
    TrackingDistance = 500.0f;

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (const ADronePawn* Drone = Cast<ADronePawn>(Target))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `ActiveDroneId`，完成 active无人机id 的更新。
        ActiveDroneId = Drone->DroneId;
        // 解释：这一行把右侧表达式的结果写入 `DroneViewMode`，完成 无人机view模式 的更新。
        DroneViewMode = EDroneViewCycleMode::Chase;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：调用 `UE_LOG` 输出调试日志，便于运行时观察状态和排查问题。
    UE_LOG(LogTemp, Log, TEXT("[Camera] Start tracking: %s"), *Target->GetName());
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 退出跟踪并回到自由视角 */
// 解释：这一行定义函数 `StopTracking`，开始实现stoptracking的具体逻辑。
void ACameraPawn::StopTracking()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `bIsTracking`，完成 布尔标志 istracking 的更新。
    bIsTracking = false;
    // 解释：这一行把右侧表达式的结果写入 `TrackingTarget`，完成 trackingtarget 的更新。
    TrackingTarget = nullptr;
    // 解释：调用 `UE_LOG` 输出调试日志，便于运行时观察状态和排查问题。
    UE_LOG(LogTemp, Log, TEXT("[Camera] Stop tracking, free mode"));
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 处理 Agent 列表点击结果
 * @param AgentId 被点击的 Agent ID
 * @param Actor 对应 Actor
 * 选中有效目标时切换到跟踪模式，空选择时恢复自由视角。
 */
// 解释：这一行定义函数 `OnItemClicked`，开始实现on项clicked的具体逻辑。
void ACameraPawn::OnItemClicked(const FString& AgentId, AActor* Actor)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (Actor)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (const ADronePawn* Drone = Cast<ADronePawn>(Actor))
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行把右侧表达式的结果写入 `ActiveDroneId`，完成 active无人机id 的更新。
            ActiveDroneId = Drone->DroneId;
            // 解释：这一行把右侧表达式的结果写入 `DroneViewMode`，完成 无人机view模式 的更新。
            DroneViewMode = EDroneViewCycleMode::Chase;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
        // 解释：调用 `StartTracking` 执行当前步骤需要的功能逻辑。
        StartTracking(Actor);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行给出兜底分支，当上面的条件都不满足时执行。
    else
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `Empty` 执行当前步骤需要的功能逻辑。
        ActiveDroneId.Empty();
        // 解释：调用 `StopTracking` 执行当前步骤需要的功能逻辑。
        StopTracking();
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 在追踪、俯视和第一人称视角之间循环切换 */
void ACameraPawn::OnCycleDroneView()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `ADronePawn* Drone`，完成 adronePawn无人机 的更新。
    ADronePawn* Drone = ResolveActiveDrone(true);
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!Drone)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `UE_LOG` 输出调试日志，便于运行时观察状态和排查问题。
        UE_LOG(LogTemp, Warning, TEXT("[Camera] No drone available for view cycle"));
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行进入 `switch` 分支分派结构，后续会按不同枚举或状态值执行不同逻辑。
    switch (DroneViewMode)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
    // 解释：这一行声明 `switch` 中的一个分支标签，对应某个具体取值的处理路径。
    case EDroneViewCycleMode::Chase:
        // 解释：这一行把右侧表达式的结果写入 `DroneViewMode`，完成 无人机view模式 的更新。
        DroneViewMode = EDroneViewCycleMode::TopDown;
        // 解释：这一行把右侧表达式的结果写入 `bIsTracking`，完成 布尔标志 istracking 的更新。
        bIsTracking = false;
        // 解释：这一行把右侧表达式的结果写入 `TrackingTarget`，完成 trackingtarget 的更新。
        TrackingTarget = Drone;
        // 解释：调用 `UE_LOG` 输出调试日志，便于运行时观察状态和排查问题。
        UE_LOG(LogTemp, Log, TEXT("[Camera] Drone view -> TopDown (%s)"), *ActiveDroneId);
        // 解释：这一行立即跳出当前循环或 `switch` 分支，避免继续执行后续分支。
        break;
    // 解释：这一行声明 `switch` 中的一个分支标签，对应某个具体取值的处理路径。
    case EDroneViewCycleMode::TopDown:
        // 解释：这一行把右侧表达式的结果写入 `DroneViewMode`，完成 无人机view模式 的更新。
        DroneViewMode = EDroneViewCycleMode::FPV;
        // 解释：这一行把右侧表达式的结果写入 `bIsTracking`，完成 布尔标志 istracking 的更新。
        bIsTracking = false;
        // 解释：这一行把右侧表达式的结果写入 `TrackingTarget`，完成 trackingtarget 的更新。
        TrackingTarget = Drone;
        // 解释：调用 `UE_LOG` 输出调试日志，便于运行时观察状态和排查问题。
        UE_LOG(LogTemp, Log, TEXT("[Camera] Drone view -> FPV (%s)"), *ActiveDroneId);
        // 解释：这一行立即跳出当前循环或 `switch` 分支，避免继续执行后续分支。
        break;
    // 解释：这一行声明 `switch` 中的一个分支标签，对应某个具体取值的处理路径。
    case EDroneViewCycleMode::FPV:
    // 解释：这一行声明 `switch` 的默认分支，当前面所有 `case` 都不匹配时执行。
    default:
        // 解释：这一行把右侧表达式的结果写入 `DroneViewMode`，完成 无人机view模式 的更新。
        DroneViewMode = EDroneViewCycleMode::Chase;
        // 解释：调用 `StartTracking` 执行当前步骤需要的功能逻辑。
        StartTracking(Drone);
        // 解释：调用 `UE_LOG` 输出调试日志，便于运行时观察状态和排查问题。
        UE_LOG(LogTemp, Log, TEXT("[Camera] Drone view -> Chase (%s)"), *ActiveDroneId);
        // 解释：这一行立即跳出当前循环或 `switch` 分支，避免继续执行后续分支。
        break;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 切回自由视角并清除当前无人机选择 */
// 解释：这一行定义函数 `OnSwitchFreeView`，开始实现onswitchfreeview的具体逻辑。
void ACameraPawn::OnSwitchFreeView()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `DroneViewMode`，完成 无人机view模式 的更新。
    DroneViewMode = EDroneViewCycleMode::Chase;
    // 解释：调用 `Empty` 执行当前步骤需要的功能逻辑。
    ActiveDroneId.Empty();
    // 解释：调用 `StopTracking` 执行当前步骤需要的功能逻辑。
    StopTracking();
    // 解释：调用 `UE_LOG` 输出调试日志，便于运行时观察状态和排查问题。
    UE_LOG(LogTemp, Log, TEXT("[Camera] Switch to free view (M)"));
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 更新环绕跟踪相机
 * @param DeltaTime 帧间隔（秒）
 * 基于 `OrbitYaw/OrbitPitch/TrackingDistance` 计算相机相对目标的球面偏移。
 */
// 解释：这一行定义函数 `UpdateTrackingCamera`，开始实现updatetracking相机的具体逻辑。
void ACameraPawn::UpdateTrackingCamera(float DeltaTime)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!TrackingTarget)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行把右侧表达式的结果写入 `const FVector TargetLocation`，完成 constfvectortargetlocation 的更新。
    const FVector TargetLocation = TrackingTarget->GetActorLocation();
    // 解释：调用 `OrbitRotation` 执行当前步骤需要的功能逻辑。
    const FRotator OrbitRotation(OrbitPitch, OrbitYaw, 0.0f);
    // 解释：这一行把右侧表达式的结果写入 `const FVector Offset`，完成 constfvectoroffset 的更新。
    const FVector Offset = OrbitRotation.Vector() * (-TrackingDistance);
    // 解释：这一行声明成员或局部变量 `CameraLocation`，用于保存相机location。
    const FVector CameraLocation = TargetLocation + Offset;

    // 解释：调用 `SetActorLocation` 执行当前步骤需要的功能逻辑。
    SetActorLocation(CameraLocation);

    // 解释：这一行把右侧表达式的结果写入 `const FRotator LookAtRotation`，完成 constfrotatorlookatrotation 的更新。
    const FRotator LookAtRotation = (TargetLocation - CameraLocation).Rotation();
    // 解释：调用 `SetActorRotation` 执行当前步骤需要的功能逻辑。
    SetActorRotation(LookAtRotation);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 更新俯视相机
 * 将相机放置在无人机正上方，并保持朝下观察。
 */
// 解释：这一行定义函数 `UpdateTopDownCamera`，开始实现updatetopdown相机的具体逻辑。
void ACameraPawn::UpdateTopDownCamera()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `ADronePawn* Drone`，完成 adronePawn无人机 的更新。
    ADronePawn* Drone = ResolveActiveDrone(false);
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!Drone)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行把右侧表达式的结果写入 `const FVector TargetLocation`，完成 constfvectortargetlocation 的更新。
    const FVector TargetLocation = Drone->GetActorLocation();
    // 解释：这一行把右侧表达式的结果写入 `const FRotator TargetRot`，完成 constfrotatortargetrot 的更新。
    const FRotator TargetRot = Drone->GetActorRotation();

    // 解释：调用 `SetActorLocation` 执行当前步骤需要的功能逻辑。
    SetActorLocation(TargetLocation + FVector(0.0f, 0.0f, TrackingDistance));
    // 解释：调用 `SetActorRotation` 执行当前步骤需要的功能逻辑。
    SetActorRotation(FRotator(-90.0f, TargetRot.Yaw, 0.0f));
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 更新第一人称视角
 * 优先复用无人机的 `SceneCapture` 位姿，否则退化为无人机自身 Actor 位姿。
 */
// 解释：这一行定义函数 `UpdateFPVCamera`，开始实现updatefpvcamera的具体逻辑。
void ACameraPawn::UpdateFPVCamera()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `ADronePawn* Drone`，完成 adronePawn无人机 的更新。
    ADronePawn* Drone = ResolveActiveDrone(false);
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!Drone)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (Drone->DroneSceneCapture)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        SetActorLocationAndRotation(
            // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
            Drone->DroneSceneCapture->GetComponentLocation(),
            // 解释：调用 `GetComponentRotation` 执行当前步骤需要的功能逻辑。
            Drone->DroneSceneCapture->GetComponentRotation());
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行给出兜底分支，当上面的条件都不满足时执行。
    else
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `SetActorLocationAndRotation` 执行当前步骤需要的功能逻辑。
        SetActorLocationAndRotation(Drone->GetActorLocation(), Drone->GetActorRotation());
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 解析当前激活的无人机
 * @param bAutoSelect 为真时允许自动选择场景中可用无人机
 * @return 当前用于视角切换的无人机实例
 */
// 解释：这一行定义函数 `ResolveActiveDrone`，开始实现resolveactivedrone的具体逻辑。
ADronePawn* ACameraPawn::ResolveActiveDrone(bool bAutoSelect)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `UAgentManager* Manager`，完成 uagent管理器管理器 的更新。
    UAgentManager* Manager = UAgentManager::GetInstance();
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!Manager)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return nullptr;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!ActiveDroneId.IsEmpty())
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (ADronePawn* Drone = Cast<ADronePawn>(Manager->GetAgent(ActiveDroneId)))
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return Drone;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：调用 `Empty` 执行当前步骤需要的功能逻辑。
        ActiveDroneId.Empty();
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!bAutoSelect)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return nullptr;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (ADronePawn* PreferredDrone = Cast<ADronePawn>(Manager->GetAgent(TEXT("drone_0"))))
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `ActiveDroneId`，完成 active无人机id 的更新。
        ActiveDroneId = PreferredDrone->DroneId;
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return PreferredDrone;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：调用 `GetAllAgentIds` 执行当前步骤需要的功能逻辑。
    const TArray<FString> Ids = Manager->GetAllAgentIds();
    // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
    for (const FString& Id : Ids)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (ADronePawn* Drone = Cast<ADronePawn>(Manager->GetAgent(Id)))
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行把右侧表达式的结果写入 `ActiveDroneId`，完成 active无人机id 的更新。
            ActiveDroneId = Drone->DroneId;
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
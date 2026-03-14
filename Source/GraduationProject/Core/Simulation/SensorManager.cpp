// 解释：引入当前实现文件对应的头文件 `SensorManager.h`，使实现部分能够看到类和函数声明。
#include "SensorManager.h"

// 解释：引入 `AgentManager.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "GraduationProject/Core/Manager/AgentManager.h"
// 解释：引入 `DronePawn.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "GraduationProject/Drone/DronePawn.h"

// 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
namespace
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    /**
     * @brief 将 Unreal 坐标向量转换到指定输出坐标系
     * @param ValueUE Unreal 世界坐标下的向量
     * @param Frame 目标坐标系名称
     * @return 转换后的向量
     */
    // 解释：这一行定义函数 `ConvertUEToFrame`，开始实现convertuetoframe的具体逻辑。
    FVector ConvertUEToFrame(const FVector& ValueUE, const FString& Frame)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行声明成员或局部变量 `Lower`，用于保存lower。
        FString Lower = Frame;
        // 解释：调用 `ToLowerInline` 执行当前步骤需要的功能逻辑。
        Lower.ToLowerInline();
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (Lower == TEXT("ned"))
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 当前项目采用 UE 的 X/Y 不变、Z 取反来近似 NED 的 Down 方向。
            // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
            return FVector(ValueUE.X, ValueUE.Y, -ValueUE.Z);
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return ValueUE;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    /**
     * @brief 将 Unreal 欧拉角转换到指定输出坐标系
     * @param RotUE Unreal 世界坐标下的姿态
     * @param Frame 目标坐标系名称
     * @return 转换后的姿态角
     */
    // 解释：这一行定义函数 `ConvertUEToFrameRotator`，开始实现convertuetoframerotator的具体逻辑。
    FRotator ConvertUEToFrameRotator(const FRotator& RotUE, const FString& Frame)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行声明成员或局部变量 `Lower`，用于保存lower。
        FString Lower = Frame;
        // 解释：调用 `ToLowerInline` 执行当前步骤需要的功能逻辑。
        Lower.ToLowerInline();
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (Lower == TEXT("ned"))
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

    /**
     * @brief 根据高度估算标准大气压
     * @param AltitudeMeters 海拔高度（米）
     * @return 估算气压值（Pa）
     */
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    double EstimatePressurePa(double AltitudeMeters)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行通过 `FMath::Max` 给 `const double H` 施加下界约束，避免 constdoubleH 过小。
        const double H = FMath::Max(0.0, static_cast<double>(AltitudeMeters));
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return 101325.0 * FMath::Pow(1.0 - 2.25577e-5 * H, 5.25588);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

// 解释：这一行把右侧表达式的结果写入 `USensorManager* USensorManager::Instance`，完成 usensor管理器usensor管理器instance 的更新。
USensorManager* USensorManager::Instance = nullptr;

/** @brief 获取传感器管理器单例 */
// 解释：这一行定义函数 `GetInstance`，开始实现getinstance的具体逻辑。
USensorManager* USensorManager::GetInstance()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!Instance)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `Instance`，完成 instance 的更新。
        Instance = NewObject<USensorManager>();
        // 解释：调用 `AddToRoot` 执行当前步骤需要的功能逻辑。
        Instance->AddToRoot();
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return Instance;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/** @brief 释放传感器管理器单例并清空缓存 */
// 解释：这一行定义函数 `Cleanup`，开始实现cleanup的具体逻辑。
void USensorManager::Cleanup()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (Instance)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：调用 `Empty` 执行当前步骤需要的功能逻辑。
        Instance->DroneCache.Empty();
        // 解释：调用 `RemoveFromRoot` 执行当前步骤需要的功能逻辑。
        Instance->RemoveFromRoot();
        // 解释：这一行把右侧表达式的结果写入 `Instance`，完成 instance 的更新。
        Instance = nullptr;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

/**
 * @brief 构建无人机传感器 JSON
 * @param DroneId 无人机 ID
 * @param World 当前场景 World
 * @param Frame 输出坐标系，支持 `ue` 与 `ned`
 * @return 传感器数据 JSON 字符串
 * 输出内容包含 IMU、GPS、气压计和运动学信息，其中线加速度由相邻两帧速度差分估计。
 */
// 解释：这一行定义函数 `BuildDroneSensorJson`，开始实现build无人机传感器json的具体逻辑。
FString USensorManager::BuildDroneSensorJson(const FString& DroneId, UWorld* World, const FString& Frame)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!World)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return TEXT("{\"status\":\"error\",\"message\":\"No World\"}");
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行把右侧表达式的结果写入 `UAgentManager* AgentManager`，完成 uagent管理器agent管理器 的更新。
    UAgentManager* AgentManager = UAgentManager::GetInstance();
    // 解释：这一行把右侧表达式的结果写入 `ADronePawn* Drone`，完成 adronePawn无人机 的更新。
    ADronePawn* Drone = Cast<ADronePawn>(AgentManager->GetAgent(DroneId));
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!Drone)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return FString::Printf(TEXT("{\"status\":\"error\",\"message\":\"Drone '%s' not found\"}"), *DroneId);
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行把右侧表达式的结果写入 `const FVector PosUE`，完成 constfvectorposue 的更新。
    const FVector PosUE = Drone->GetCurrentPosition();
    // 解释：这一行把右侧表达式的结果写入 `const FVector VelUE`，完成 constfvectorvelue 的更新。
    const FVector VelUE = Drone->GetCurrentVelocity();
    // 解释：这一行把右侧表达式的结果写入 `const double Now`，完成 constdoublenow 的更新。
    const double Now = FPlatformTime::Seconds();

    // 解释：这一行声明成员或局部变量 `AccUE`，用于保存accue。
    FVector AccUE = FVector::ZeroVector;
    // 解释：这一行把右侧表达式的结果写入 `FSensorKinematicCache& Cache`，完成 fsensorkinematiccachecache 的更新。
    FSensorKinematicCache& Cache = DroneCache.FindOrAdd(DroneId);
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (Cache.bValid)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行通过 `FMath::Max` 给 `const double Dt` 施加下界约束，避免 constdoubledt 过小。
        const double Dt = FMath::Max(1e-4, Now - Cache.LastTimeSec);
        // 解释：这一行把右侧表达式的结果写入 `AccUE`，完成 accue 的更新。
        AccUE = (VelUE - Cache.LastVelocity) / Dt;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行把右侧表达式的结果写入 `Cache.LastVelocity`，完成 lastvelocity 的更新。
    Cache.LastVelocity = VelUE;
    // 解释：这一行把右侧表达式的结果写入 `Cache.LastTimeSec`，完成 lasttimesec 的更新。
    Cache.LastTimeSec = Now;
    // 解释：这一行把右侧表达式的结果写入 `Cache.bValid`，完成 布尔标志 valid 的更新。
    Cache.bValid = true;

    // 解释：这一行把右侧表达式的结果写入 `const FVector Pos`，完成 constfvectorpos 的更新。
    const FVector Pos = ConvertUEToFrame(PosUE, Frame);
    // 解释：这一行把右侧表达式的结果写入 `const FVector Vel`，完成 constfvectorvel 的更新。
    const FVector Vel = ConvertUEToFrame(VelUE, Frame);
    // 解释：这一行把右侧表达式的结果写入 `const FVector Acc`，完成 constfvectoracc 的更新。
    const FVector Acc = ConvertUEToFrame(AccUE, Frame);

    // 解释：这一行把右侧表达式的结果写入 `const FRotator RotUE`，完成 constfrotatorrotue 的更新。
    const FRotator RotUE = Drone->CurrentState.GetRotator();
    // 解释：这一行把右侧表达式的结果写入 `const FRotator Rot`，完成 constfrotatorrot 的更新。
    const FRotator Rot = ConvertUEToFrameRotator(RotUE, Frame);

    // 以项目默认场景原点为“家点”粗略映射经纬度，便于与外部工具统一字段格式。
    // 解释：这一行声明成员或局部变量 `HomeLat`，用于保存homelat。
    const double HomeLat = 31.3020;
    // 解释：这一行声明成员或局部变量 `HomeLon`，用于保存homelon。
    const double HomeLon = 120.5853;
    // 解释：这一行把右侧表达式的结果写入 `const double Lat`，完成 constdoublelat 的更新。
    const double Lat = HomeLat + static_cast<double>(PosUE.Y) / 111111.0;
    // 解释：这一行通过 `FMath::Max` 给 `const double Lon` 施加下界约束，避免 constdoublelon 过小。
    const double Lon = HomeLon + static_cast<double>(PosUE.X) / (111111.0 * FMath::Max(0.1, FMath::Cos(FMath::DegreesToRadians(HomeLat))));
    // 解释：这一行通过 `FMath::Max` 给 `const double Alt` 施加下界约束，避免 constdoublealt 过小。
    const double Alt = FMath::Max(0.0, static_cast<double>(PosUE.Z));

    // 解释：这一行把右侧表达式的结果写入 `const double PressurePa`，完成 constdoublepressurepa 的更新。
    const double PressurePa = EstimatePressurePa(Alt);

    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return FString::Printf(
        // 解释：这一行位于构造函数初始化列表中，把 `TEXT` 直接初始化为 `"{\"status\":\"ok\",\"id\":\"%s\",\"frame\":\"%s\",\"timestamp\":%.6f,\"imu\":{\"linear_accel\":[%.4f,%.4f,%.4f],\"angular_velocity\":[0.0,0.0,0.0],\"orientation\":{\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f}},\"gps\":{\"lat\":%.8f,\"lon\":%.8f,\"alt\":%.3f},\"barometer\":{\"altitude_m\":%.3f,\"pressure_pa\":%.2f},\"kinematics\":{\"position\":[%.4f,%.4f,%.4f],\"velocity\":[%.4f,%.4f,%.4f]}}"`，减少进入函数体后的额外赋值开销。
        TEXT("{\"status\":\"ok\",\"id\":\"%s\",\"frame\":\"%s\",\"timestamp\":%.6f,\"imu\":{\"linear_accel\":[%.4f,%.4f,%.4f],\"angular_velocity\":[0.0,0.0,0.0],\"orientation\":{\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f}},\"gps\":{\"lat\":%.8f,\"lon\":%.8f,\"alt\":%.3f},\"barometer\":{\"altitude_m\":%.3f,\"pressure_pa\":%.2f},\"kinematics\":{\"position\":[%.4f,%.4f,%.4f],\"velocity\":[%.4f,%.4f,%.4f]}}"),
        *DroneId,
        *Frame,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Now,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Acc.X, Acc.Y, Acc.Z,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Rot.Roll, Rot.Pitch, Rot.Yaw,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Lat, Lon, Alt,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Alt, PressurePa,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Pos.X, Pos.Y, Pos.Z,
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        Vel.X, Vel.Y, Vel.Z);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

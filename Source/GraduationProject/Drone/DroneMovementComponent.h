#pragma once
#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "DroneState.h"
#include "DroneParameters.h"
#include "DroneMovementComponent.generated.h"

class UPDController;
class UPIDController;

UCLASS(ClassGroup=(CustomDrone), meta=(BlueprintSpawnableComponent))
class GRADUATIONPROJECT_API UDroneMovementComponent : public UActorComponent
{
    GENERATED_BODY()

public:
    /** @brief 鏋勯€犲嚱鏁帮紝鍒濆鍖栭粯璁ゆ帶鍒舵ā寮忓拰鍛戒护 */
    UDroneMovementComponent();

    /**
     * @brief 缁勪欢鍒濆鍖?
     * 鍒涘缓鎵€鏈?PD/PID 鎺у埗鍣ㄥ疄渚嬶紝璁＄畻鎺у埗鍒嗛厤鐭╅樀銆?
     */
    virtual void BeginPlay() override;

    /**
     * @brief 姣忓抚 Tick 鎵ц鎺у埗鏇存柊鍜岀墿鐞嗕豢鐪?
     * @param DeltaTime 甯ч棿闅旀椂闂达紙绉掞級
     * @param TickType Tick 绫诲瀷
     * @param ThisTickFunction Tick 鍑芥暟寮曠敤
     */
    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

    /**
     * @brief 璁剧疆鍒濆鐘舵€?
     * @param InitialState 鍒濆鏃犱汉鏈虹姸鎬?
     */
    UFUNCTION(BlueprintCallable, Category = "DroneMovement|Setup")
    void SetInitialState(const FDroneState& InitialState);

    /**
     * @brief 璁剧疆鐗╃悊鍙傛暟骞堕噸鏂拌绠楁帶鍒跺垎閰嶇煩闃?
     * @param NewParameters 鏂扮殑鐗╃悊鍙傛暟
     */
    UFUNCTION(BlueprintCallable, Category = "DroneMovement|Setup")
    void SetParameters(const FDroneParameters& NewParameters);

    /**
     * @brief 鍒囨崲鎺у埗妯″紡锛屽悓鏃堕噸缃墍鏈夋帶鍒跺櫒鐘舵€?
     * @param NewMode 鏂扮殑鎺у埗妯″紡
     */
    UFUNCTION(BlueprintCallable, Category = "DroneMovement|Setup")
    void SetControlMode(EDroneControlMode NewMode);

    /**
     * @brief 璁剧疆鍘熷鎺у埗鍛戒护
     * @param Command 鍛戒护鏁扮粍
     */
    UFUNCTION(BlueprintCallable, Category = "DroneMovement|Control")
    void SetControlCommand(const TArray<double>& Command);

    /**
     * @brief 璁剧疆鐩爣浣嶇疆
     * @param TargetPos 鐩爣浣嶇疆
     */
    UFUNCTION(BlueprintCallable, Category = "DroneMovement|Control")
    void SetTargetPosition(const FVector& TargetPos, float Speed = 0.0f);

    /**
     * @brief 璁剧疆鐩爣閫熷害
     * @param TargetVel 鐩爣閫熷害
     */
    UFUNCTION(BlueprintCallable, Category = "DroneMovement|Control")
    void SetTargetVelocity(const FVector& TargetVel);

    /**
     * @brief 璁剧疆鐩爣濮挎€佸拰鎺ㄥ姏
     * @param Attitude 鐩爣濮挎€?
     * @param Thrust 鎺ㄥ姏绯绘暟
     */
    UFUNCTION(BlueprintCallable, Category = "DroneMovement|Control")
    void SetTargetAttitude(const FRotator& Attitude, float Thrust);

    /**
     * @brief 鑾峰彇褰撳墠鏃犱汉鏈虹姸鎬?
     * @return 褰撳墠鐘舵€佺殑鎷疯礉
     */
    UFUNCTION(BlueprintPure, Category = "DroneMovement|State")
    FDroneState GetCurrentState() const { return CurrentState; }

    /**
     * @brief 閲嶇疆鐘舵€佸苟娓呴浂鎵€鏈夋帶鍒跺櫒
     * @param NewState 鏂扮殑鍒濆鐘舵€?
     */
    UFUNCTION(BlueprintCallable, Category = "DroneMovement|State")
    void ResetState(const FDroneState& NewState);

    /**
     * @brief 璁剧疆浣嶇疆鎺у埗鍣?PD)澧炵泭
     * @param Kp 姣斾緥澧炵泭
     * @param Kd 寰垎澧炵泭
     */
    void SetPositionGains(float Kp, float Kd);

    /**
     * @brief 璁剧疆閫熷害鎺у埗鍣?PID)澧炵泭
     * @param Kp 姣斾緥澧炵泭
     * @param Ki 绉垎澧炵泭
     * @param Kd 寰垎澧炵泭
     */
    void SetVelocityGains(float Kp, float Ki, float Kd);

    /**
     * @brief 璁剧疆濮挎€佹帶鍒跺櫒(PD)澧炵泭
     * @param Kp 姣斾緥澧炵泭
     * @param Kd 寰垎澧炵泭
     */
    void SetAttitudeGains(float Kp, float Kd);

    /**
     * @brief 璁剧疆瑙掗€熺巼鎺у埗鍣ㄥ鐩?
     * @param Kp 姣斾緥澧炵泭
     */
    void SetAngleRateGains(float Kp);
    void SetHeadingControl(EDroneYawMode NewYawMode, EDroneDrivetrainMode NewDrivetrain, float YawDeg = 0.0f);

    EDroneYawMode GetYawMode() const { return YawMode; }
    EDroneDrivetrainMode GetDrivetrainMode() const { return DrivetrainMode; }

public:
    /** @brief 鏃犱汉鏈虹墿鐞嗗弬鏁?*/
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DroneMovement|Parameters")
    FDroneParameters Parameters;

    /** @brief 褰撳墠鎺у埗妯″紡 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DroneMovement|Control")
    EDroneControlMode CurrentControlMode;

    /** @brief 褰撳墠鏃犱汉鏈虹姸鎬?*/
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "DroneMovement|State")
    FDroneState CurrentState;

    /** @brief 缁勪欢鏄惁宸插畬鎴愬垵濮嬪寲 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "DroneMovement|State")
    bool bInitialized = false;

    /**
     * @brief 鑷姩鍋忚埅閫熷害闃堝€?(m/s)
     * 姘村钩閫熷害浣庝簬姝ゅ€兼椂淇濇寔褰撳墠鍋忚埅瑙掞紝閬垮厤鎮仠鏃舵姈鍔?
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DroneMovement|Parameters")
    float YawSpeedThreshold = 0.3f;

    /**
     * @brief 每帧允许执行的最大固定子步数
     * 防止极端帧率抖动时子步循环过长。
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DroneMovement|Parameters", meta=(ClampMin="1", ClampMax="64"))
    int32 MaxSubStepsPerTick = 16;

    /**
     * @brief 妯″瀷鍋忚埅鍋忕Щ瑙?(搴?
     * 琛ュ伩妯″瀷姝ｅ墠鏂逛笌 Actor +X 杞寸殑澶硅
     * 榛樿 -90掳锛氭ā鍨嬫鍓嶆柟娌?+Y 杞达紙UE 鍙虫柟鍚戯級
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DroneMovement|Parameters")
    float YawOffset = 0.0f;

protected:
    /**
     * @brief 鎵ц涓€甯х殑鎺у埗鏇存柊
     * @param DeltaTime 甯ч棿闅?
     */
    void ControlUpdate(double DeltaTime);

    /**
     * @brief 浣嶇疆鎺у埗鐜?
     * @param PositionCommand 鐩爣浣嶇疆
     * @return 閫熷害鍛戒护
     */
    FVector PositionLoop(const FVector& PositionCommand);

    /**
     * @brief 閫熷害鎺у埗鐜?
     * @param VelocityCommand 鐩爣閫熷害
     * @param OutThrust 杈撳嚭锛氳绠楀緱鍒扮殑鎬绘帹鍔?
     * @return 鍔犻€熷害鍛戒护
     */
    FVector VelocityLoop(const FVector& VelocityCommand, double& OutThrust);

    /**
     * @brief 濮挎€佹帶鍒剁幆
     * @param AccelerationCommand 鏈熸湜鍔犻€熷害
     * @return 瑙掗€熺巼鍛戒护
     */
    FVector AttitudeLoop(const FVector& AccelerationCommand);

    /**
     * @brief 瑙掗€熺巼鎺у埗鐜?
     * @param AngularVelocityCommand 鐩爣瑙掗€熺巼
     * @return 鍔涚煩鍛戒护
     */
    FVector AngularVelocityLoop(const FVector& AngularVelocityCommand);

    /**
     * @brief 浠庡姏鐭╁拰鎺ㄥ姏鍛戒护璁＄畻鐢垫満杞€?
     * @param TorqueCommand 涓夎酱鍔涚煩鍛戒护
     * @param Thrust 鎬绘帹鍔涘懡浠?
     * @return 鍥涗釜鐢垫満鐨勮浆閫?
     */
    TArray<double> CalculateMotorSpeeds(const FVector& TorqueCommand, double Thrust);

    /**
     * @brief Verlet 绉垎涓€姝ワ紙鏇夸唬 RK4锛岃绠楁洿蹇級
     * @param DeltaTime 绉垎姝ラ暱
     */
    void VerletUpdate(double DeltaTime);

    /**
     * @brief 璁＄畻缁欏畾鐘舵€佷笅鐨勫悎澶栧姏鍜屽悎鍔涚煩
     * @param State 褰撳墠鐘舵€?
     * @param OutForce 杈撳嚭锛氫笘鐣屽潗鏍囩郴涓嬬殑鍚堝鍔?
     * @param OutTorque 杈撳嚭锛氭満浣撳潗鏍囩郴涓嬬殑鍚堝姏鐭?
     */
    void CalculateTotalForcesAndTorques(const FDroneState& State, FVector& OutForce, FVector& OutTorque);

    /**
     * @brief 妫€鏌ュ苟澶勭悊鍦伴潰纰版挒
     * @param GroundZ 鍦伴潰 Z 鍧愭爣锛堜豢鐪熷潗鏍囩郴锛?
     */
    void CheckGroundCollision(double GroundZ = 0.0);

    /**
     * @brief 灏嗘満浣撳潗鏍囩郴鍚戦噺鏃嬭浆鍒颁笘鐣屽潗鏍囩郴
     * @param BodyVector 鏈轰綋鍧愭爣绯诲悜閲?
     * @param Orientation 濮挎€佸洓鍏冩暟
     * @return 涓栫晫鍧愭爣绯诲悜閲?
     */
    FVector RotateBodyToWorld(const FVector& BodyVector, const FQuat& Orientation);

    /** @brief 鍒涘缓骞跺垵濮嬪寲鎵€鏈?PD/PID 鎺у埗鍣ㄥ疄渚?*/
    void InitializeControllers();

    /** @brief 閲嶇疆鎵€鏈夋帶鍒跺櫒鐨勫唴閮ㄧ姸鎬?*/
    void ResetAllControllers();

    /**
     * @brief 瑙掑害褰掍竴鍖栧埌 [-蟺, 蟺] 鑼冨洿
     * @param AngleRad 杈撳叆瑙掑害锛堝姬搴︼級
     * @return 褰掍竴鍖栧悗鐨勮搴?
     */
    static double NormalizeAngle(double AngleRad);

    /** @brief 璁＄畻鎺у埗鍒嗛厤鐭╅樀 G 鍙婂叾閫嗙煩闃?GInv */
    void ComputeControlAllocation();

private:
    /** @brief 鍘熷鎺у埗鍛戒护 */
    TArray<double> ControlCommands;

    /** @brief 鏄惁鍦ㄥ湴闈笂锛圙round Lock锛?*/
    bool bGrounded = false;

    /** @brief 涓婁竴甯у姞閫熷害锛堢敤浜?Verlet 绉垎锛?*/
    FVector PrevLinearAcceleration = FVector::ZeroVector;

    /** @brief 涓婁竴甯ц鍔犻€熷害锛堢敤浜?Verlet 绉垎锛?*/
    FVector PrevAngularAcceleration = FVector::ZeroVector;

    /** @brief 浠跨湡璧峰楂樺害锛堢敤浜庡湴闈㈢鎾炴娴嬶級 */
    double InitialGroundZ = 0.0;

    /** @brief 鐢垫満涓€闃舵护娉㈠櫒杈撳嚭锛堟护娉㈠悗鐨勮浆閫燂級 */
    TArray<double> MotorSpeedsFiltered = {0.0, 0.0, 0.0, 0.0};

    /** @brief 涓婁竴娆℃帶鍒舵洿鏂扮殑 DeltaTime锛堜緵鐢垫満婊ゆ尝鍣ㄤ娇鐢級 */
    double LastControlDeltaTime = 0.003;

    /** @brief 固定步长积分累计时间（秒） */
    double FixedStepAccumulator = 0.0;

    /** @brief 閿佸畾鍋忚埅瑙掞紙浣庨€熸椂淇濇寔鐨勫亸鑸€硷紝寮у害锛?*/
    double LockedYaw = 0.0;

    /** @brief 鍋忚埅鏄惁宸插垵濮嬪寲锛堥娆¤揪鍒伴€熷害闃堝€煎悗缃?true锛?*/
    bool bYawInitialized = false;

    /** @brief 鑷姩鍋忚埅鏈熸湜瑙掞紙鐢?ControlUpdate 璁＄畻锛屽姬搴︼級 */
    double DesiredYaw = 0.0;

    /** @brief 鐩爣浣嶇疆 */
    FVector TargetPosition;

    /** @brief 鐩爣閫熷害 */
    FVector TargetVelocity;

    /** @brief 鐩爣濮挎€?*/
    FRotator TargetAttitude;

    /** @brief 鐩爣鎺ㄥ姏 */
    double TargetThrust;

    double DefaultPositionSpeedLimit = 2.0;
    double PositionAxisSpeedLimit = 2.0;
    double TargetPositionSpeedLimit = 2.0;
    EDroneYawMode YawMode = EDroneYawMode::Auto;
    EDroneDrivetrainMode DrivetrainMode = EDroneDrivetrainMode::ForwardOnly;
    double CommandedYaw = 0.0;

    // 浣嶇疆鎺у埗鍣?PD
    UPROPERTY() UPDController* PxController;    // X 杞翠綅缃帶鍒跺櫒
    UPROPERTY() UPDController* PyController;    // Y 杞翠綅缃帶鍒跺櫒
    UPROPERTY() UPDController* PzController;    // Z 杞翠綅缃帶鍒跺櫒

    // 閫熷害鎺у埗鍣?PID
    UPROPERTY() UPIDController* VxController;   // X 杞撮€熷害鎺у埗鍣?
    UPROPERTY() UPIDController* VyController;   // Y 杞撮€熷害鎺у埗鍣?
    UPROPERTY() UPIDController* VzController;   // Z 杞撮€熷害鎺у埗鍣?

    // 濮挎€佹帶鍒跺櫒 PD
    UPROPERTY() UPDController* RollController;  // 婊氳浆瑙掓帶鍒跺櫒
    UPROPERTY() UPDController* PitchController; // 淇话瑙掓帶鍒跺櫒
    UPROPERTY() UPDController* YawController;   // 鍋忚埅瑙掓帶鍒跺櫒

    // 瑙掗€熺巼鎺у埗鍣?PID
    UPROPERTY() UPIDController* RollRateController;   // 婊氳浆瑙掗€熺巼鎺у埗鍣?
    UPROPERTY() UPIDController* PitchRateController;  // 淇话瑙掗€熺巼鎺у埗鍣?
    UPROPERTY() UPIDController* YawRateController;    // 鍋忚埅瑙掗€熺巼鎺у埗鍣?

    /**
     * @brief 鎺у埗鍒嗛厤鐭╅樀 G (4脳4)
     * G[0][i] = kT (鎺ㄥ姏琛?
     * G[1][i] = 婊氳浆鍔涚煩璐＄尞
     * G[2][i] = 淇话鍔涚煩璐＄尞
     * G[3][i] = 鍋忚埅鍔涚煩璐＄尞
     */
    double G[4][4];

    /** @brief 鎺у埗鍒嗛厤閫嗙煩闃?GInv (4脳4) */
    double GInv[4][4];
};


#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "Components/StaticMeshComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "CineCameraComponent.h"
#include "DroneState.h"
#include "DroneParameters.h"
#include "DronePawn.generated.h"

class UDroneMovementComponent;
class UDroneApi;

UENUM(BlueprintType)
enum class EDroneMissionRole : uint8
{
    Unknown     UMETA(DisplayName = "Unknown"),
    Target      UMETA(DisplayName = "Target"),
    Interceptor UMETA(DisplayName = "Interceptor")
};

/**
 * 鍥涙棆缈兼棤浜烘満 Pawn
 * 缁勪欢灞傜骇锛?
 *   RootComp (USceneComponent)
 *     鈹斺攢 BodyMesh
 *          鈹溾攢 Fan0..Fan3 鈥?鍥涗釜椋庢墖缃戞牸锛堥€氳繃鎻掓Ы闄勭潃锛?
 *          鈹溾攢 CameraYawMesh 鈥?鎽勫儚澶?Yaw 浜戝彴锛圕amera_Yaw_002 鎻掓Ы锛?
 *          鈹?   鈹斺攢 CameraPitchMesh 鈥?鎽勫儚澶?Pitch 浜戝彴锛圕amera_Pitch_002 鎻掓Ы锛?
 *          鈹?        鈹溾攢 DroneSceneCapture 鈥?鍦烘櫙閲囬泦锛堝浘鍍忎紶杈撶敤锛?
 *          鈹?        鈹斺攢 DroneCineCamera 鈥?CineCamera锛圥ostProcess 鍚屾鐢級
 *          鈹斺攢 MovementComp (UDroneMovementComponent)
 */
UCLASS()
class GRADUATIONPROJECT_API ADronePawn : public APawn
{
    GENERATED_BODY()

public:
    ADronePawn();
    virtual void BeginPlay() override;
    virtual void Tick(float DeltaTime) override;

    /**
     * @brief 璁剧疆鐩爣浣嶇疆
     * @param TargetPos 鐩爣浣嶇疆
     */
    UFUNCTION(BlueprintCallable, Category = "DroneControl")
    void SetTargetPosition(const FVector& TargetPos, float Speed = 0.0f);

    /**
     * @brief 璁剧疆鐩爣閫熷害
     * @param TargetVel 鐩爣閫熷害
     */
    UFUNCTION(BlueprintCallable, Category = "DroneControl")
    void SetTargetVelocity(const FVector& TargetVel);
    
    UFUNCTION(BlueprintCallable, Category = "DroneControl")
    void SetHeadingControl(EDroneYawMode YawMode, EDroneDrivetrainMode Drivetrain, float YawDeg = 0.0f);

    /** @brief 鍦ㄥ綋鍓嶄綅缃偓鍋?*/
    UFUNCTION(BlueprintCallable, Category = "DroneControl")
    void Hover();

    /**
     * @brief 鎸夐€熷害椋炶
     */
    UFUNCTION(BlueprintCallable, Category = "DroneControl")
    void MoveByVelocity(float Vx, float Vy, float Vz);

    /**
     * @brief 璧烽鍒版寚瀹氶珮搴?
     * @param Altitude 鐩爣楂樺害
     */
    UFUNCTION(BlueprintCallable, Category = "Drone")
    void Takeoff(float Altitude = 3.0f);

    /** @brief 闄嶈惤鍒板湴闈?*/
    UFUNCTION(BlueprintCallable, Category = "Drone")
    void Land();

    /**
     * @brief 閲嶇疆鏃犱汉鏈哄埌鎸囧畾浣嶇疆鍜屽Э鎬?
     * @param NewLocation 閲嶇疆浣嶇疆
     * @param NewRotation 閲嶇疆濮挎€?
     */
    UFUNCTION(BlueprintCallable, Category = "DroneControl")
    void ResetDrone(const FVector& NewLocation, const FRotator& NewRotation);

    /** @brief 鑾峰彇褰撳墠浣嶇疆 */
    FVector GetCurrentPosition() const;

    /** @brief 鑾峰彇褰撳墠閫熷害 */
    FVector GetCurrentVelocity() const;

    // 鈹€鈹€ 鎽勫儚澶存帶鍒?鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

    /**
     * @brief 璁剧疆鎽勫儚澶翠簯鍙扮洰鏍囪搴?
     * @param TargetPitch 鐩爣淇话瑙?
     * @param TargetYaw 鐩爣鍋忚埅瑙?
     */
    UFUNCTION(BlueprintCallable, Category = "Drone|Camera")
    void SetCameraAngles(float TargetPitch, float TargetYaw);

    /**
     * @brief 閲囬泦涓€甯у浘鍍忓苟杩斿洖 Base64 缂栫爜鐨?JPEG 瀛楃涓?
     * @param Quality JPEG 鍘嬬缉璐ㄩ噺锛?=0 鍒欎娇鐢?JpegQuality 榛樿鍊硷級
     * @return Base64 缂栫爜鐨?JPEG 鍥惧儚鏁版嵁
     */
    FString CaptureImageBase64(int32 Quality = -1);

    /** @brief 鑾峰彇鎽勫儚澶村綋鍓嶄刊浠拌 */
    float GetCameraCurrentPitch() const { return CameraCurrentPitch; }

    /** @brief 鑾峰彇鎽勫儚澶村綋鍓嶅亸鑸 */
    float GetCameraCurrentYaw() const { return CameraCurrentYaw; }

    // 鈹€鈹€ 鐘舵€佷笌鍙傛暟 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

    /** @brief 褰撳墠鏃犱汉鏈虹姸鎬?*/
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|State")
    FDroneState CurrentState;

    /** @brief 褰撳墠鎺у埗妯″紡 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|State")
    EDroneControlMode ControlMode = EDroneControlMode::Idle;

    /** @brief 鏃犱汉鏈虹墿鐞嗗弬鏁?*/
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Parameters")
    FDroneParameters Parameters;

    /** @brief 杩愬姩浠跨湡缁勪欢鎸囬拡 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    UDroneMovementComponent* MovementComp = nullptr;

    /** @brief DroneApi 鎺ュ彛鎸囬拡 */
    UPROPERTY()
    UDroneApi* Api = nullptr;

    /** @brief 鏃犱汉鏈?Agent ID */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Config")
    FString DroneId = TEXT("drone_0");

    /** @brief 无人机任务角色（目标机/拦截机） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Config")
    EDroneMissionRole MissionRole = EDroneMissionRole::Unknown;

    // 鈹€鈹€ 缁勪欢 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

    /** @brief 鏍瑰満鏅粍浠?*/
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    USceneComponent* RootComp;

    /** @brief 鏈鸿韩缃戞牸缁勪欢 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    UStaticMeshComponent* BodyMesh;

    /** @brief 椋庢墖0 缃戞牸 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    UStaticMeshComponent* Fan0;

    /** @brief 椋庢墖1 缃戞牸 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    UStaticMeshComponent* Fan1;

    /** @brief 椋庢墖2 缃戞牸 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    UStaticMeshComponent* Fan2;

    /** @brief 椋庢墖3 缃戞牸 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    UStaticMeshComponent* Fan3;

    /** @brief 鎽勫儚澶?Yaw 浜戝彴缃戞牸锛堥檮鐫€鍒?BodyMesh 鐨?Camera_Yaw_002 鎻掓Ы锛?*/
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    UStaticMeshComponent* CameraYawMesh;

    /** @brief 鎽勫儚澶?Pitch 浜戝彴缃戞牸锛堥檮鐫€鍒?CameraYawMesh 鐨?Camera_Pitch_002 鎻掓Ы锛?*/
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Components")
    UStaticMeshComponent* CameraPitchMesh;

    /** @brief 鍦烘櫙閲囬泦缁勪欢锛岄噰闆嗘憚鍍忓ご瑙嗚鐢婚潰鐢ㄤ簬鍥惧儚浼犺緭 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Camera")
    USceneCaptureComponent2D* DroneSceneCapture;

    /** @brief CineCamera 鈥?鐢ㄤ簬缁ф壙鍦烘櫙 PostProcessVolume 璁剧疆锛屽啀鍚屾鍒?SceneCapture */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Camera")
    UCineCameraComponent* DroneCineCamera;

    // 鈹€鈹€ 鎽勫儚澶村弬鏁?鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

    /** @brief 鎽勫儚澶磋鍦鸿 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera")
    float CameraFOV = 90.0f;

    /** @brief 鎽勫儚澶撮噰闆嗗垎杈ㄧ巼瀹藉害 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera")
    int32 CameraWidth = 1280;

    /** @brief 鎽勫儚澶撮噰闆嗗垎杈ㄧ巼楂樺害 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera")
    int32 CameraHeight = 720;

    /** @brief 语义分割 ID（0-255，对齐 AirSim segmentation 逻辑） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera", meta = (ClampMin = "0", ClampMax = "255"))
    int32 SegmentationId = 20;

    /** @brief 鏇濆厜琛ュ伩 EV */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera", meta = (ClampMin = "-10.0", ClampMax = "10.0"))
    float ExposureBias = 0.0f;

    /** @brief JPEG 鍘嬬缉璐ㄩ噺 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera", meta = (ClampMin = "1", ClampMax = "100"))
    int32 JpegQuality = 90;

    /** @brief 鎽勫儚澶存棆杞彃鍊奸€熷害 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera")
    float CameraRotationSpeed = 8.0f;

private:
    /**
     * @brief 灏?CurrentState 鍚屾鍒?UE Actor 鐨?Transform
     * @param State 瑕佸悓姝ョ殑鐘舵€?
     */
    void ApplyStateToActor(const FDroneState& State);

    /**
     * @brief 鏇存柊铻烘棆妗ㄦ棆杞姩鐢?
     * @param DeltaTime 甯ч棿闅旓紙绉掞級
     * 鏍规嵁鐢垫満杞€熻浆鎹负姣忓抚鏃嬭浆瑙掑害
     */
    void UpdatePropellerAnimation(float DeltaTime);

    /**
     * @brief 鏍规嵁绱㈠紩鑾峰彇瀵瑰簲鐨勯鎵囩綉鏍?
     * @param Index 椋庢墖绱㈠紩
     * @return 瀵瑰簲鐨?UStaticMeshComponent 鎸囬拡
     */
    UStaticMeshComponent* GetFanMesh(int32 Index) const;

    /** @brief 鏇存柊鎽勫儚澶翠簯鍙版棆杞彃鍊?*/
    void UpdateCameraRotation(float DeltaTime);

    /** @brief 灏?CineCamera 鐨?PostProcess 璁剧疆鍚屾鍒?SceneCapture */
    void SyncPostProcessToCapture();

    /** @brief 将该无人机可见网格写入 CustomDepth/Stencil，用于 Segmentation 图像 */
    void ApplySegmentationStencil();

    /** @brief 鎽勫儚澶寸洰鏍囦刊浠拌 */
    float CameraTargetPitch = 0.0f;

    /** @brief 鎽勫儚澶寸洰鏍囧亸鑸 */
    float CameraTargetYaw = 0.0f;

    /** @brief 鎽勫儚澶村綋鍓嶅疄闄呬刊浠拌 */
    float CameraCurrentPitch = 0.0f;

    /** @brief 鎽勫儚澶村綋鍓嶅疄闄呭亸鑸 */
    float CameraCurrentYaw = 0.0f;
};


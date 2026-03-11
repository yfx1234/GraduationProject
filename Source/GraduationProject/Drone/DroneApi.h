#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "DroneParameters.h"
#include "DroneApi.generated.h"

class ADronePawn;

UCLASS()
class GRADUATIONPROJECT_API UDroneApi : public UObject
{
    GENERATED_BODY()

public:
    /**
     *  @brief 鍒濆鍖?API锛岀粦瀹氬埌鐩爣 DronePawn
     * @param Owner 鎷ユ湁姝?API 鐨?DronePawn 瀹炰緥
     */
    void Initialize(ADronePawn* Owner);

    /**
     * @brief 绉诲姩鍒版寚瀹氫綅缃?
     * @param X 鐩爣 X 鍧愭爣 (m)
     * @param Y 鐩爣 Y 鍧愭爣 (m)
     * @param Z 鐩爣 Z 鍧愭爣 (m)
     * @param Speed 绉诲姩閫熷害 (m/s)锛岄粯璁?2
     */
    void MoveToPosition(float X, float Y, float Z, float Speed = 2.0f);

    /** @brief 鍦ㄥ綋鍓嶄綅缃偓鍋?*/
    void Hover();

    /**
     * @brief 璧烽鍒版寚瀹氶珮搴?
     * @param Altitude 鐩爣楂樺害 (m)
     */
    void Takeoff(float Altitude);

    /** @brief 闄嶈惤鍒板湴闈?*/
    void Land();

    /**
     * @brief 鎸夐€熷害椋炶
     * @param Vx X 鏂瑰悜閫熷害 (m/s)
     * @param Vy Y 鏂瑰悜閫熷害 (m/s)
     * @param Vz Z 鏂瑰悜閫熷害 (m/s)
     */
    void MoveByVelocity(float Vx, float Vy, float Vz);
    void SetTargetAttitude(float RollDeg, float PitchDeg, float YawDeg, float Thrust);
    void SetMotorSpeeds(float M0, float M1, float M2, float M3);
    /**
     * @brief 璁剧疆鑸悜鎺у埗妯″紡锛坹aw_mode + drivetrain锛?     * @param YawMode 鍋忚埅妯″紡
     * @param Drivetrain 椹卞姩妯″紡
     * @param YawDeg 褰?YawMode=Angle 鏃剁敓鏁堬紝鍗曚綅搴?     */
    void SetHeadingControl(EDroneYawMode YawMode, EDroneDrivetrainMode Drivetrain, float YawDeg = 0.0f);

    /**
     * @brief 鑾峰彇褰撳墠浣嶇疆
     * @return 浣嶇疆鍚戦噺 (m)
     */
    FVector GetPosition() const;

    /**
     * @brief 鑾峰彇褰撳墠閫熷害
     * @return 閫熷害鍚戦噺 (m/s)
     */
    FVector GetVelocity() const;

    /**
     * @brief 鑾峰彇褰撳墠濮挎€?
     * @return 娆ф媺瑙?(Roll, Pitch, Yaw)
     */
    FRotator GetOrientation() const;

    /**
     * @brief 鑾峰彇鍥涗釜鐢垫満鐨勮浆閫?
     * @return 杞€熸暟缁?(rad/s)
     */
    TArray<float> GetMotorSpeeds() const;

    /**
     * @brief 鑾峰彇褰撳墠鎺у埗妯″紡
     * @return 鎺у埗妯″紡鏋氫妇鍊?
     */
    EDroneControlMode GetControlMode() const;

    /**
     * @brief 璁剧疆浣嶇疆鎺у埗鍣紙PD锛夊鐩?
     * @param Kp 姣斾緥澧炵泭
     * @param Kd 寰垎澧炵泭
     */
    void SetPositionControllerGains(float Kp, float Kd);

    /**
     * @brief 璁剧疆閫熷害鎺у埗鍣紙PID锛夊鐩?
     * @param Kp 姣斾緥澧炵泭
     * @param Ki 绉垎澧炵泭
     * @param Kd 寰垎澧炵泭
     */
    void SetVelocityControllerGains(float Kp, float Ki, float Kd);

    /**
     * @brief 璁剧疆濮挎€佹帶鍒跺櫒锛圥D锛夊鐩?
     * @param Kp 姣斾緥澧炵泭
     * @param Kd 寰垎澧炵泭
     */
    void SetAttitudeControllerGains(float Kp, float Kd);

    /**
     * @brief 璁剧疆瑙掗€熺巼鎺у埗鍣ㄥ鐩?
     * @param Kp 姣斾緥澧炵泭
     */
    void SetAngleRateControllerGains(float Kp);

    /**
     * @brief 閲嶇疆鏃犱汉鏈哄埌鎸囧畾浣嶇疆鍜屽Э鎬?
     * @param Position 鐩爣浣嶇疆 (m)
     * @param Rotation 鐩爣濮挎€?
     */
    void Reset(FVector Position = FVector::ZeroVector, FRotator Rotation = FRotator::ZeroRotator);

private:
    /** @brief 鎷ユ湁姝?API 鐨?DronePawn 瀹炰緥 */
    UPROPERTY()
    ADronePawn* OwnerPawn = nullptr;
};



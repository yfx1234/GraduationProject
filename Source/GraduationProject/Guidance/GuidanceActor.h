// 瑙ｉ噴锛氫娇鐢?`#pragma once` 闃叉璇ュご鏂囦欢鍦ㄧ紪璇戣繃绋嬩腑琚噸澶嶅寘鍚€?
#pragma once

// 瑙ｉ噴锛氬紩鍏?Unreal 鐨勬牳蹇冨熀纭€澶存枃浠讹紝鎻愪緵甯哥敤瀹瑰櫒銆佹暟瀛︾被鍨嬪拰鏃ュ織瀹忋€?
#include "CoreMinimal.h"
// 瑙ｉ噴锛氬紩鍏?`Actor.h`锛屼负褰撳墠鏂囦欢琛ュ厖鎵€渚濊禆鐨勭被鍨嬨€佸嚱鏁版垨鎺ュ彛澹版槑銆?
#include "GameFramework/Actor.h"
// 瑙ｉ噴锛氬紩鍏?`DronePawn.h`锛屼负褰撳墠鏂囦欢琛ュ厖鎵€渚濊禆鐨勭被鍨嬨€佸嚱鏁版垨鎺ュ彛澹版槑銆?
#include "GraduationProject/Drone/DronePawn.h"
// 瑙ｉ噴锛氬紩鍏?`GuidanceActor.generated.h`锛岃 Unreal Header Tool 鐢熸垚鐨勫弽灏勪唬鐮佸湪鏈枃浠朵腑鍙銆?
#include "GuidanceActor.generated.h"

// 瑙ｉ噴锛氳繖涓€琛屽０鏄?绫?`IGuidanceMethod`锛岀敤浜庡皝瑁卛guidancemethod鐩稿叧鐨勬暟鎹笌琛屼负銆?
class IGuidanceMethod;
// 瑙ｉ噴锛氳繖涓€琛屽０鏄?绫?`UKalmanPredictor`锛岀敤浜庡皝瑁卽kalman棰勬祴鍣ㄧ浉鍏崇殑鏁版嵁涓庤涓恒€?
class UKalmanPredictor;
// 瑙ｉ噴锛氳繖涓€琛屽０鏄?绫?`UVisualInterceptController`锛岀敤浜庡皝瑁卽visual鎷︽埅鎺у埗鍣ㄧ浉鍏崇殑鏁版嵁涓庤涓恒€?
class UVisualInterceptController;
// 瑙ｉ噴锛氳繖涓€琛屽０鏄?绫?`UAgentManager`锛岀敤浜庡皝瑁卽agent绠＄悊鍣ㄧ浉鍏崇殑鏁版嵁涓庤涓恒€?
class UAgentManager;

/**
 * @brief 鍒跺涓庢嫤鎴崗璋?Actor
 *
 * 璇ョ被缁熶竴灏佽涓夌被鑳藉姏锛?
 * 1. 甯歌寮归亾/鐬勫噯鍒跺绠楁硶鐨勭鐞嗕笌璋冪敤锛?
 * 2. 鍩轰簬 Kalman 棰勬祴鐨勭洰鏍囩姸鎬佷及璁★紱
 * 3. 鏃犱汉鏈鸿嚜鍔ㄦ嫤鎴笌瑙嗚鎷︽埅鎺у埗鎺ュ彛銆?
 *
 * 鍥犳瀹冩棦鍙互鏈嶅姟鐐鑷姩鐬勫噯锛屼篃鍙互涓烘棤浜烘満杩借釜/鎷︽埅浠诲姟鎻愪緵缁熶竴鐨?Blueprint 涓?TCP 璋冪敤鍏ュ彛銆?
 */
// 瑙ｉ噴锛氫娇鐢?`UCLASS` 瀹忔妸褰撳墠绫诲瀷娉ㄥ唽鍒?Unreal 鍙嶅皠绯荤粺涓紝渚夸簬钃濆浘銆佸簭鍒楀寲鍜岀紪杈戝櫒璇嗗埆銆?
UCLASS()
// 瑙ｉ噴锛氳繖涓€琛屽０鏄?绫?`AGuidanceActor`锛岀敤浜庡皝瑁卆guidanceActor鐩稿叧鐨勬暟鎹笌琛屼负銆?
class GRADUATIONPROJECT_API AGuidanceActor : public AActor
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
{
    // 瑙ｉ噴锛氬睍寮€ Unreal Header Tool 鐢熸垚鐨勬牱鏉夸唬鐮侊紝杩欐槸 UObject銆丄ctor銆佺粍浠剁瓑绫诲瀷姝ｅ父宸ヤ綔鐨勫熀纭€銆?
    GENERATED_BODY()

// 瑙ｉ噴锛氫粠杩欎竴琛屽紑濮嬭繘鍏?`public` 璁块棶鍖猴紝涓嬮潰鐨勬垚鍛樹細瀵瑰閮ㄦā鍧楀叕寮€銆?
public:
    /** @brief 鏋勯€犲嚱鏁帮細鍒涘缓涓€涓殣钘忎笖鏃犵鎾炵殑鍚庡彴鍗忚皟 Actor */
    // 瑙ｉ噴锛氳皟鐢?`AGuidanceActor` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    AGuidanceActor();

    /** @brief 娉ㄥ唽鑷韩鍒?AgentManager锛屼究浜庡閮ㄩ€氳繃 GuidanceId 鏌ユ壘 */
    // 瑙ｉ噴锛氳皟鐢?`BeginPlay` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    virtual void BeginPlay() override;

    /** @brief 缁撴潫鏃堕噴鏀惧埗瀵肩畻娉曞璞★紝骞朵粠 AgentManager 娉ㄩ攢鑷韩 */
    // 瑙ｉ噴锛氳皟鐢?`EndPlay` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

    /** @brief 閲嶇疆棰勬祴鍣ㄣ€佸埗瀵煎櫒鍜屽巻鍙茶緭鍑虹姸鎬?*/
    // 瑙ｉ噴锛氫娇鐢?`UFUNCTION` 瀹忎负涓嬮潰鐨勫嚱鏁板０鏄庡弽灏勫厓鏁版嵁锛屼娇鍏跺彲琚摑鍥俱€丷PC 鎴栫紪杈戝櫒璋冪敤銆?
    UFUNCTION(BlueprintCallable, Category = "Guidance")
    // 瑙ｉ噴锛氳皟鐢?`ResetGuidance` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    FString ResetGuidance();

    /** @brief 鑾峰彇褰撳墠 GuidanceActor 鐨勭姸鎬?JSON */
    // 瑙ｉ噴锛氫娇鐢?`UFUNCTION` 瀹忎负涓嬮潰鐨勫嚱鏁板０鏄庡弽灏勫厓鏁版嵁锛屼娇鍏跺彲琚摑鍥俱€丷PC 鎴栫紪杈戝櫒璋冪敤銆?
    UFUNCTION(BlueprintCallable, Category = "Guidance")
    // 瑙ｉ噴锛氳皟鐢?`GetState` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    FString GetState();

    /**
     * @brief 鍒囨崲甯歌鍒跺绠楁硶
     * @param Method 鍒跺鏂规硶鍚嶏紝鏀寔 `direct`銆乣proportional`銆乣predictive`
     * @param NavConstant 姣斾緥瀵煎紩瀵艰埅甯告暟锛屼粎 `proportional` 妯″紡浣跨敤
     * @param Iterations 棰勬祴鍒跺杩唬娆℃暟锛屼粎 `predictive` 妯″紡浣跨敤
     */
    // 瑙ｉ噴锛氫娇鐢?`UFUNCTION` 瀹忎负涓嬮潰鐨勫嚱鏁板０鏄庡弽灏勫厓鏁版嵁锛屼娇鍏跺彲琚摑鍥俱€丷PC 鎴栫紪杈戝櫒璋冪敤銆?
    UFUNCTION(BlueprintCallable, Category = "Guidance")
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `FString SetMethod(FString Method, float NavConstant`锛屽畬鎴?fstringsetmethodfstringmethodfloatnavconstant 鐨勬洿鏂般€?
    FString SetMethod(FString Method, float NavConstant = 4.0f, int32 Iterations = 3);

    /**
     * @brief 璁剧疆鏃犱汉鏈鸿嚜鍔ㄦ嫤鎴畻娉曠殑榛樿鍙傛暟
     * @param Method 鎷︽埅鏂规硶鍚嶏紝鏀寔 `pure_pursuit` / `proportional_nav` 鍙婂叾鍒悕
     * @param Speed 鎷︽埅鏈洪€熷害涓婇檺锛坢/s锛?
     * @param NavGain 姣斾緥瀵艰埅澧炵泭 N
     * @param LeadTime 鐩爣鍓嶇疆棰勬祴鏃堕棿锛坰锛?
     * @param CaptureRadiusValue 鎹曡幏鍗婂緞锛坢锛?
     */
    // 瑙ｉ噴锛氫娇鐢?`UFUNCTION` 瀹忎负涓嬮潰鐨勫嚱鏁板０鏄庡弽灏勫厓鏁版嵁锛屼娇鍏跺彲琚摑鍥俱€丷PC 鎴栫紪杈戝櫒璋冪敤銆?
    UFUNCTION(BlueprintCallable, Category = "Guidance")
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `FString SetInterceptMethod(FString Method`锛屽畬鎴?fstringset鎷︽埅methodfstringmethod 鐨勬洿鏂般€?
    FString SetInterceptMethod(FString Method = TEXT("pure_pursuit"), float Speed = -1.0f, float NavGain = -1.0f, float LeadTime = -1.0f, float CaptureRadiusValue = -1.0f);

    /** @brief 鍒楀嚭褰撳墠鍦烘櫙涓凡娉ㄥ唽鐨勭洰鏍囨満鍜屾嫤鎴満 ID */
    // 瑙ｉ噴锛氫娇鐢?`UFUNCTION` 瀹忎负涓嬮潰鐨勫嚱鏁板０鏄庡弽灏勫厓鏁版嵁锛屼娇鍏跺彲琚摑鍥俱€丷PC 鎴栫紪杈戝櫒璋冪敤銆?
    UFUNCTION(BlueprintCallable, Category = "Guidance")
    // 瑙ｉ噴锛氳皟鐢?`ListInterceptAgents` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    FString ListInterceptAgents();

    /**
     * @brief 鎵ц涓€姝ヨ嚜鍔ㄦ嫤鎴帶鍒跺苟鐩存帴涓嬪彂鎸囦护
     * @param InterceptorId 鎷︽埅鏃犱汉鏈?ID锛屽彲鐣欑┖鑷姩鏌ユ壘
     * @param TargetId 鐩爣鏃犱汉鏈?ID锛屽彲鐣欑┖鑷姩鏌ユ壘
     * @param Method 鏈璋冪敤涓存椂瑕嗙洊鐨勬嫤鎴柟娉?
     * @param Speed 鏈璋冪敤涓存椂瑕嗙洊鐨勯€熷害涓婇檺锛坢/s锛?
     * @param NavGain 鏈璋冪敤涓存椂瑕嗙洊鐨勫鑸鐩?
     * @param LeadTime 鏈璋冪敤涓存椂瑕嗙洊鐨勫墠缃娴嬫椂闂达紙s锛?
     * @param CaptureRadiusValue 鏈璋冪敤涓存椂瑕嗙洊鐨勬崟鑾峰崐寰勶紙m锛?
     * @param bStopOnCapture 鎹曡幏鍚庢槸鍚﹁嚜鍔ㄦ偓鍋?
     */
    // 瑙ｉ噴锛氫娇鐢?`UFUNCTION` 瀹忎负涓嬮潰鐨勫嚱鏁板０鏄庡弽灏勫厓鏁版嵁锛屼娇鍏跺彲琚摑鍥俱€丷PC 鎴栫紪杈戝櫒璋冪敤銆?
    UFUNCTION(BlueprintCallable, Category = "Guidance")
    // 瑙ｉ噴锛氳繖涓€琛屽畾涔夊嚱鏁?`AutoIntercept`锛屽紑濮嬪疄鐜癮uto鎷︽埅鐨勫叿浣撻€昏緫銆?
    FString AutoIntercept(
        // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `AutoIntercept` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `InterceptorId` 鐢ㄤ簬浼犲叆interceptorid銆?
        FString InterceptorId = TEXT(""),
        // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `AutoIntercept` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `TargetId` 鐢ㄤ簬浼犲叆targetid銆?
        FString TargetId = TEXT(""),
        // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `AutoIntercept` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `Method` 鐢ㄤ簬浼犲叆method銆?
        FString Method = TEXT(""),
        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`Speed`锛岀敤浜庝繚瀛榮peed銆?
        float Speed = -1.0f,
        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`NavGain`锛岀敤浜庝繚瀛榥avgain銆?
        float NavGain = -1.0f,
        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`LeadTime`锛岀敤浜庝繚瀛榣eadtime銆?
        float LeadTime = -1.0f,
        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`CaptureRadiusValue`锛岀敤浜庝繚瀛橀噰闆唕adiusvalue銆?
        float CaptureRadiusValue = -1.0f,
        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`bStopOnCapture`锛岀敤浜庝繚瀛樺竷灏旀爣蹇?stopon閲囬泦銆?
        bool bStopOnCapture = true);

    /**
     * @brief 鍚?Kalman 棰勬祴鍣ㄨ緭鍏ヤ竴甯х洰鏍囦綅缃娴?
     * @param X 瑙傛祴浣嶇疆 X
     * @param Y 瑙傛祴浣嶇疆 Y
     * @param Z 瑙傛祴浣嶇疆 Z
     * @param Dt 閲囨牱鏃堕棿闂撮殧锛坰锛?
     */
    // 瑙ｉ噴锛氫娇鐢?`UFUNCTION` 瀹忎负涓嬮潰鐨勫嚱鏁板０鏄庡弽灏勫厓鏁版嵁锛屼娇鍏跺彲琚摑鍥俱€丷PC 鎴栫紪杈戝櫒璋冪敤銆?
    UFUNCTION(BlueprintCallable, Category = "Guidance")
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `FString UpdateTarget(float X, float Y, float Z, float Dt`锛屽畬鎴?fstringupdatetargetfloatXfloatYfloatZfloatdt 鐨勬洿鏂般€?
    FString UpdateTarget(float X, float Y, float Z, float Dt = 0.1f);

    /**
     * @brief 鏍规嵁褰撳墠鐩爣棰勬祴鐘舵€佽绠楃偖濉旂瀯鍑嗚
     * @param TurretId 鐐 ID
     * @param MuzzleSpeed 寮逛父鍒濋€熷害锛坢/s锛?
     */
    // 瑙ｉ噴锛氫娇鐢?`UFUNCTION` 瀹忎负涓嬮潰鐨勫嚱鏁板０鏄庡弽灏勫厓鏁版嵁锛屼娇鍏跺彲琚摑鍥俱€丷PC 鎴栫紪杈戝櫒璋冪敤銆?
    UFUNCTION(BlueprintCallable, Category = "Guidance")
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `FString ComputeAim(FString TurretId`锛屽畬鎴?fstringcomputeaimfstringturretid 鐨勬洿鏂般€?
    FString ComputeAim(FString TurretId = TEXT("turret_0"), float MuzzleSpeed = 400.0f);

    /**
     * @brief 涓€姝ュ紡鑷姩鐬勫噯/灏勫嚮鎺ュ彛
     * @param TurretId 鐐 ID
     * @param TargetId 鐩爣 Actor ID
     * @param MuzzleSpeed 寮逛父鍒濋€熷害锛坢/s锛?
     * @param Dt 瑙傛祴鍒锋柊鍛ㄦ湡锛坰锛?
     * @param Latency 瑙嗚/閫氫俊寤惰繜琛ュ伩锛坰锛夛紝灏忎簬 0 鏃朵娇鐢ㄩ粯璁ゅ€?
     * @param bFire 鏄惁鍦ㄥ畬鎴愮瀯鍑嗗悗瑙﹀彂寮€鐏?
     */
    // 瑙ｉ噴锛氫娇鐢?`UFUNCTION` 瀹忎负涓嬮潰鐨勫嚱鏁板０鏄庡弽灏勫厓鏁版嵁锛屼娇鍏跺彲琚摑鍥俱€丷PC 鎴栫紪杈戝櫒璋冪敤銆?
    UFUNCTION(BlueprintCallable, Category = "Guidance")
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `FString AutoEngage(FString TurretId`锛屽畬鎴?fstringautoengagefstringturretid 鐨勬洿鏂般€?
    FString AutoEngage(FString TurretId = TEXT("turret_0"), FString TargetId = TEXT("drone_0"), float MuzzleSpeed = 400.0f, float Dt = 0.05f, float Latency = -1.0f, bool bFire = false);

    /**
     * @brief 璁剧疆 Kalman 棰勬祴鍣ㄥ弬鏁?
     * @param ProcessNoise 杩囩▼鍣０鍗忔柟宸郴鏁?
     * @param MeasurementNoise 瑙傛祴鍣０鍗忔柟宸郴鏁?
     */
    // 瑙ｉ噴锛氫娇鐢?`UFUNCTION` 瀹忎负涓嬮潰鐨勫嚱鏁板０鏄庡弽灏勫厓鏁版嵁锛屼娇鍏跺彲琚摑鍥俱€丷PC 鎴栫紪杈戝櫒璋冪敤銆?
    UFUNCTION(BlueprintCallable, Category = "Guidance")
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `FString SetKalmanParams(float ProcessNoise`锛屽畬鎴?fstringset鍗″皵鏇紁aramsfloatprocessnoise 鐨勬洿鏂般€?
    FString SetKalmanParams(float ProcessNoise = 1.0f, float MeasurementNoise = 0.5f);

    /**
     * @brief 鍚姩瑙嗚鎷︽埅浼氳瘽
     *
     * 璇ユ帴鍙ｆ妸 Blueprint/TCP 鍙傛暟鏁寸悊涓?JSON 鍛戒护锛屽啀杞氦缁?`UVisualInterceptController`銆?
     */
    // 瑙ｉ噴锛氫娇鐢?`UFUNCTION` 瀹忎负涓嬮潰鐨勫嚱鏁板０鏄庡弽灏勫厓鏁版嵁锛屼娇鍏跺彲琚摑鍥俱€丷PC 鎴栫紪杈戝櫒璋冪敤銆?
    UFUNCTION(BlueprintCallable, Category = "Guidance|Visual")
    // 瑙ｉ噴锛氳繖涓€琛屽畾涔夊嚱鏁?`VisualInterceptStart`锛屽紑濮嬪疄鐜拌瑙夋嫤鎴猻tart鐨勫叿浣撻€昏緫銆?
    FString VisualInterceptStart(
        FString InterceptorId = TEXT(""),
        FString TargetId = TEXT(""),
        FString Method = TEXT("vision_pid_kalman"),
        float DesiredArea = -1.0f,
        float CaptureArea = -1.0f,
        float CenterTolX = -1.0f,
        float CenterTolY = -1.0f,
        int32 CaptureHoldFrames = -1,
        int32 LostToSearchFrames = -1,
        float MaxForwardSpeed = -1.0f,
        float MaxReverseSpeed = -1.0f,
        float MaxVerticalSpeed = -1.0f,
        float MaxYawRateDeg = -1.0f,
        float RamAreaTarget = -1.0f,
        float MinRamSpeed = -1.0f,
        float InterceptDistance = -1.0f,
        float TrackLeadTime = -1.0f,
        float RamLeadTime = -1.0f,
        float SearchCamYawLimitDeg = -1.0f,
        float SearchCamRateDeg = -1.0f,
        float SearchBodyYawRateDeg = -1.0f,
        float SearchCamPitchDeg = -1000.0f,
        float SearchVzAmp = -1.0f,
        int32 StopOnCaptureFlag = -1,
        int32 UseKalmanFlag = -1);

    /**
     * @brief 鍚戣瑙夋嫤鎴帶鍒跺櫒鎻愪氦涓€甯ф娴嬬粨鏋?
     *
     * 杈撳叆妫€娴嬫涓績銆侀潰绉€佺疆淇″害浠ュ強鍥惧儚灏哄锛岀敱瑙嗚鎺у埗鍣ㄨ緭鍑烘満浣撻€熷害鍜屼簯鍙版帶鍒堕噺銆?
     */
    // 瑙ｉ噴锛氫娇鐢?`UFUNCTION` 瀹忎负涓嬮潰鐨勫嚱鏁板０鏄庡弽灏勫厓鏁版嵁锛屼娇鍏跺彲琚摑鍥俱€丷PC 鎴栫紪杈戝櫒璋冪敤銆?
    UFUNCTION(BlueprintCallable, Category = "Guidance|Visual")
    // 瑙ｉ噴锛氳繖涓€琛屽畾涔夊嚱鏁?`VisualInterceptUpdate`锛屽紑濮嬪疄鐜拌瑙夋嫤鎴猽pdate鐨勫叿浣撻€昏緫銆?
    FString VisualInterceptUpdate(
        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`HasDetection`锛岀敤浜庝繚瀛榟asdetection銆?
        int32 HasDetection = 0,
        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`Cx`锛岀敤浜庝繚瀛榗x銆?
        float Cx = 0.0f,
        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`Cy`锛岀敤浜庝繚瀛榗y銆?
        float Cy = 0.0f,
        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`Area`锛岀敤浜庝繚瀛榓rea銆?
        float Area = 0.0f,
        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`AreaRatio`锛岀敤浜庝繚瀛榓rearatio銆?
        float AreaRatio = -1.0f,
        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`Conf`锛岀敤浜庝繚瀛榗onf銆?
        float Conf = 1.0f,
        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`Dt`锛岀敤浜庝繚瀛榙t銆?
        float Dt = 0.08f,
        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`ImageW`锛岀敤浜庝繚瀛樺浘鍍廤銆?
        float ImageW = 640.0f,
        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`ImageH`锛岀敤浜庝繚瀛樺浘鍍廐銆?
        float ImageH = 480.0f,
        // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `VisualInterceptUpdate` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `InterceptorId` 鐢ㄤ簬浼犲叆interceptorid銆?
        FString InterceptorId = TEXT(""),
        // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `FString TargetId`锛屽畬鎴?fstringtargetid 鐨勬洿鏂般€?
        FString TargetId = TEXT(""));

    /** @brief 鍋滄瑙嗚鎷︽埅锛屽苟璇锋眰鎷︽埅鏃犱汉鏈烘偓鍋?*/
    // 瑙ｉ噴锛氫娇鐢?`UFUNCTION` 瀹忎负涓嬮潰鐨勫嚱鏁板０鏄庡弽灏勫厓鏁版嵁锛屼娇鍏跺彲琚摑鍥俱€丷PC 鎴栫紪杈戝櫒璋冪敤銆?
    UFUNCTION(BlueprintCallable, Category = "Guidance|Visual")
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `FString VisualInterceptStop(FString InterceptorId`锛屽畬鎴?fstring瑙嗚鎷︽埅stopfstringinterceptorid 鐨勬洿鏂般€?
    FString VisualInterceptStop(FString InterceptorId = TEXT(""), FString TargetId = TEXT(""));

    /** @brief 鏌ヨ瑙嗚鎷︽埅鎺у埗鍣ㄥ綋鍓嶇姸鎬?*/
    // 瑙ｉ噴锛氫娇鐢?`UFUNCTION` 瀹忎负涓嬮潰鐨勫嚱鏁板０鏄庡弽灏勫厓鏁版嵁锛屼娇鍏跺彲琚摑鍥俱€丷PC 鎴栫紪杈戝櫒璋冪敤銆?
    UFUNCTION(BlueprintCallable, Category = "Guidance|Visual")
    // 瑙ｉ噴锛氳皟鐢?`VisualInterceptState` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    FString VisualInterceptState();

    /** @brief GuidanceActor 鐨勫敮涓€娉ㄥ唽 ID */
    // 瑙ｉ噴锛氫娇鐢?`UPROPERTY` 瀹忎负涓嬮潰鐨勬垚鍛樺彉閲忓０鏄庡弽灏勫厓鏁版嵁锛屾帶鍒剁紪杈戝櫒鏄剧ず銆佸簭鍒楀寲鍜岃摑鍥惧彲瑙佹€с€?
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Guidance")
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `FString GuidanceId`锛屽畬鎴?fstring鍒跺id 鐨勬洿鏂般€?
    FString GuidanceId = TEXT("guidance_0");

// 瑙ｉ噴锛氫粠杩欎竴琛屽紑濮嬭繘鍏?`private` 璁块棶鍖猴紝涓嬮潰鐨勬垚鍛樺彧鍏佽绫诲唴閮ㄨ闂€?
private:
    /** @brief 鎯版€у垵濮嬪寲棰勬祴鍣ㄣ€佽瑙夋嫤鎴帶鍒跺櫒涓庨粯璁ゅ埗瀵肩畻娉?*/
    // 瑙ｉ噴锛氳皟鐢?`EnsureInitialized` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    void EnsureInitialized();

    /**
     * @brief 鎸変换鍔¤鑹叉煡鎵炬棤浜烘満
     * @param Manager AgentManager 瀹炰緥
     * @param DesiredRole 鐩爣浠诲姟瑙掕壊
     * @param ExcludeId 闇€瑕佹帓闄ょ殑鏃犱汉鏈?ID
     */
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `ADronePawn* FindDroneByRole(UAgentManager* Manager, EDroneMissionRole DesiredRole, const FString& ExcludeId`锛屽畬鎴?adronePawnfind鏃犱汉鏈篵yroleuagent绠＄悊鍣ㄧ鐞嗗櫒edronemissionroledesiredroleconstfstringexcludeid 鐨勬洿鏂般€?
    ADronePawn* FindDroneByRole(UAgentManager* Manager, EDroneMissionRole DesiredRole, const FString& ExcludeId = TEXT("")) const;

    /** @brief 鏋勯€犻敊璇?JSON 瀛楃涓?*/
    // 瑙ｉ噴锛氳皟鐢?`MakeError` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    FString MakeError(const FString& Msg) const;

    /** @brief 鏋勯€犳垚鍔?JSON 瀛楃涓?*/
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `FString MakeOk(const FString& Msg`锛屽畬鎴?fstringmakeokconstfstringmsg 鐨勬洿鏂般€?
    FString MakeOk(const FString& Msg = TEXT("ok")) const;

    /** @brief 鐩爣鐘舵€?Kalman 棰勬祴鍣?*/
    // 瑙ｉ噴锛氫娇鐢?`UPROPERTY` 瀹忎负涓嬮潰鐨勬垚鍛樺彉閲忓０鏄庡弽灏勫厓鏁版嵁锛屾帶鍒剁紪杈戝櫒鏄剧ず銆佸簭鍒楀寲鍜岃摑鍥惧彲瑙佹€с€?
    UPROPERTY()
    // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`Predictor`锛岀敤浜庝繚瀛橀娴嬪櫒銆?
    UKalmanPredictor* Predictor = nullptr;

    /** @brief 瑙嗚鎷︽埅鎺у埗鍣?*/
    // 瑙ｉ噴锛氫娇鐢?`UPROPERTY` 瀹忎负涓嬮潰鐨勬垚鍛樺彉閲忓０鏄庡弽灏勫厓鏁版嵁锛屾帶鍒剁紪杈戝櫒鏄剧ず銆佸簭鍒楀寲鍜岃摑鍥惧彲瑙佹€с€?
    UPROPERTY()
    // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`VisualInterceptController`锛岀敤浜庝繚瀛樿瑙夋嫤鎴帶鍒跺櫒銆?
    UVisualInterceptController* VisualInterceptController = nullptr;

    /** @brief 褰撳墠鍚敤鐨勫父瑙勫埗瀵肩畻娉曞璞★紝鐢熷懡鍛ㄦ湡鐢辨湰绫荤鐞?*/
    // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`CurrentMethod`锛岀敤浜庝繚瀛榗urrentmethod銆?
    IGuidanceMethod* CurrentMethod = nullptr;

    /** @brief 褰撳墠鍒跺绠楁硶鍚嶇О */
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `FString CurrentMethodName`锛屽畬鎴?fstringcurrentmethodname 鐨勬洿鏂般€?
    FString CurrentMethodName = TEXT("predictive");

    /** @brief 鏈€杩戜竴娆¤緭鍑虹殑淇话鐬勫噯瑙掞紙deg锛?*/
    // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`LastPitch`锛岀敤浜庝繚瀛榣astpitch銆?
    float LastPitch = 0.0f;

    /** @brief 鏈€杩戜竴娆¤緭鍑虹殑鍋忚埅鐬勫噯瑙掞紙deg锛?*/
    // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`LastYaw`锛岀敤浜庝繚瀛榣astyaw銆?
    float LastYaw = 0.0f;

    /** @brief 鏈€杩戜竴娆¤绠楀緱鍒扮殑鐬勫噯鐐?*/
    // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`LastAimPoint`锛岀敤浜庝繚瀛榣astaimpoint銆?
    FVector LastAimPoint = FVector::ZeroVector;

    /** @brief 鏈€杩戜竴娆′及璁＄殑寮逛父椋炶鏃堕棿锛坰锛?*/
    // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`LastFlightTime`锛岀敤浜庝繚瀛榣astflighttime銆?
    float LastFlightTime = 0.0f;

    /** @brief 榛樿瑙嗚寤惰繜琛ュ伩鍊硷紙s锛?*/
    // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`DefaultVisionLatency`锛岀敤浜庝繚瀛榙efaultvisionlatency銆?
    float DefaultVisionLatency = 0.08f;

    /** @brief 鏈€杩戜竴娆″弬涓庤绠楃殑寤惰繜琛ュ伩鍊硷紙s锛?*/
    // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`LastLatencyCompensation`锛岀敤浜庝繚瀛榣astlatencycompensation銆?
    float LastLatencyCompensation = 0.08f;

    /** @brief 褰撳墠榛樿鐨勬棤浜烘満鎷︽埅鏂规硶鍚?*/
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `FString CurrentInterceptMethod`锛屽畬鎴?fstringcurrent鎷︽埅method 鐨勬洿鏂般€?
    FString CurrentInterceptMethod = TEXT("pure_pursuit");

    /** @brief 鎷︽埅鏈洪粯璁ら€熷害涓婇檺锛坢/s锛?*/
    // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`InterceptorSpeed`锛岀敤浜庝繚瀛榠nterceptorspeed銆?
    float InterceptorSpeed = 8.0f;

    /** @brief 姣斾緥瀵艰埅澧炵泭 N */
    // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`InterceptNavGain`锛岀敤浜庝繚瀛樻嫤鎴猲avgain銆?
    float InterceptNavGain = 3.0f;

    /** @brief 绾拷韪?鍓嶇疆娉曚娇鐢ㄧ殑棰勬祴鏃堕棿锛坰锛?*/
    // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`InterceptLeadTime`锛岀敤浜庝繚瀛樻嫤鎴猯eadtime銆?
    float InterceptLeadTime = 0.6f;

    /** @brief 鍒ゅ畾鎹曡幏鎴愬姛鐨勮窛绂婚槇鍊硷紙m锛?*/
    // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`CaptureRadius`锛岀敤浜庝繚瀛橀噰闆唕adius銆?
    float CaptureRadius = 1.5f;

    /** @brief 鏈€杩戜竴娆¤嚜鍔ㄦ嫤鎴娇鐢ㄧ殑鎷︽埅鏈?ID */
    // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`LastInterceptorId`锛岀敤浜庝繚瀛榣astinterceptorid銆?
    FString LastInterceptorId;

    /** @brief 鏈€杩戜竴娆¤嚜鍔ㄦ嫤鎴娇鐢ㄧ殑鐩爣鏈?ID */
    // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`LastTargetId`锛岀敤浜庝繚瀛榣asttargetid銆?
    FString LastTargetId;

    /** @brief 鏈€杩戜竴娆′笅鍙戠粰鎷︽埅鏈虹殑閫熷害鎸囦护锛坢/s锛?*/
    // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`LastInterceptorCmdVel`锛岀敤浜庝繚瀛榣astinterceptorcmdvel銆?
    FVector LastInterceptorCmdVel = FVector::ZeroVector;

    /** @brief 鏈€杩戜竴娆¤嚜鍔ㄦ嫤鎴椂鐨勭洰鏍囪窛绂伙紙m锛?*/
    // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`LastDistanceToTarget`锛岀敤浜庝繚瀛榣astdistancetotarget銆?
    float LastDistanceToTarget = 0.0f;

    /** @brief 鏈€杩戜竴娆¤嚜鍔ㄦ嫤鎴椂鐨勯棴鍚堥€熷害锛坢/s锛?*/
    // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`LastClosingSpeed`锛岀敤浜庝繚瀛榣astclosingspeed銆?
    float LastClosingSpeed = 0.0f;

    /** @brief 鏈€杩戜竴娆¤嚜鍔ㄦ嫤鎴槸鍚︾敓鎴愪簡鏈夋晥鎺у埗閲?*/
    // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`bLastInterceptValid`锛岀敤浜庝繚瀛樺竷灏旀爣蹇?last鎷︽埅valid銆?
    bool bLastInterceptValid = false;

    /** @brief 鏈€杩戜竴娆¤嚜鍔ㄦ嫤鎴槸鍚﹀凡鍒ゅ畾鎹曡幏 */
    // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`bLastCaptured`锛岀敤浜庝繚瀛樺竷灏旀爣蹇?lastcaptured銆?
    bool bLastCaptured = false;
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
};



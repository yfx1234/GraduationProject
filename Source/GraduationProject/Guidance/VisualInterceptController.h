// 瑙ｉ噴锛氫娇鐢?`#pragma once` 闃叉璇ュご鏂囦欢鍦ㄧ紪璇戣繃绋嬩腑琚噸澶嶅寘鍚€?
#pragma once

// 瑙ｉ噴锛氬紩鍏?Unreal 鐨勬牳蹇冨熀纭€澶存枃浠讹紝鎻愪緵甯哥敤瀹瑰櫒銆佹暟瀛︾被鍨嬪拰鏃ュ織瀹忋€?
#include "CoreMinimal.h"
// 瑙ｉ噴锛氬紩鍏?`NoExportTypes.h`锛屼负褰撳墠鏂囦欢琛ュ厖鎵€渚濊禆鐨勭被鍨嬨€佸嚱鏁版垨鎺ュ彛澹版槑銆?
#include "UObject/NoExportTypes.h"
// 瑙ｉ噴锛氬紩鍏?`VisualInterceptController.generated.h`锛岃 Unreal Header Tool 鐢熸垚鐨勫弽灏勪唬鐮佸湪鏈枃浠朵腑鍙銆?
#include "VisualInterceptController.generated.h"

// 瑙ｉ噴锛氳繖涓€琛屽０鏄?绫?`FJsonObject`锛岀敤浜庡皝瑁協jsonobject鐩稿叧鐨勬暟鎹笌琛屼负銆?
class FJsonObject;
// 瑙ｉ噴锛氳繖涓€琛屽０鏄?绫?`ADronePawn`锛岀敤浜庡皝瑁卆dronePawn鐩稿叧鐨勬暟鎹笌琛屼负銆?
class ADronePawn;
// 瑙ｉ噴锛氳繖涓€琛屽０鏄?绫?`UKalmanPredictor`锛岀敤浜庡皝瑁卽kalman棰勬祴鍣ㄧ浉鍏崇殑鏁版嵁涓庤涓恒€?
class UKalmanPredictor;
// 瑙ｉ噴锛氳繖涓€琛屽０鏄?绫?`UPIDController`锛岀敤浜庡皝瑁卽pidcontroller鐩稿叧鐨勬暟鎹笌琛屼负銆?
class UPIDController;

/**
 * @brief 鍩轰簬瑙嗚鐨勬棤浜烘満鎷︽埅鎺у埗鍣?
 *
 * 閫氳繃鍥惧儚甯т腑鐩爣妫€娴嬬粨鏋滐紙涓績鍧愭爣 + 闈㈢Н锛夐┍鍔ㄦ嫤鎴棤浜烘満锛?
 * 瀹炵幇 鎼滅储 鈫?璺熻釜 鈫?閫艰繎 鈫?鎹曡幏 鐨勫洓闃舵瑙嗚浼烘湇闂幆銆?
 * 鏀寔 PID / PD 鎺у埗锛屽彲閫?Kalman 婊ゆ尝骞虫粦鐗瑰緛棰勬祴銆?
 * 鎵€鏈夊弬鏁板潎鍙€氳繃 JSON 鍛戒护鍦ㄧ嚎璋冩暣銆?
 */
// 瑙ｉ噴锛氫娇鐢?`UCLASS` 瀹忔妸褰撳墠绫诲瀷娉ㄥ唽鍒?Unreal 鍙嶅皠绯荤粺涓紝渚夸簬钃濆浘銆佸簭鍒楀寲鍜岀紪杈戝櫒璇嗗埆銆?
UCLASS()
// 瑙ｉ噴锛氳繖涓€琛屽０鏄?绫?`UVisualInterceptController`锛岀敤浜庡皝瑁卽visual鎷︽埅鎺у埗鍣ㄧ浉鍏崇殑鏁版嵁涓庤涓恒€?
class GRADUATIONPROJECT_API UVisualInterceptController : public UObject
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
{
    // 瑙ｉ噴锛氬睍寮€ Unreal Header Tool 鐢熸垚鐨勬牱鏉夸唬鐮侊紝杩欐槸 UObject銆丄ctor銆佺粍浠剁瓑绫诲瀷姝ｅ父宸ヤ綔鐨勫熀纭€銆?
    GENERATED_BODY()

// 瑙ｉ噴锛氫粠杩欎竴琛屽紑濮嬭繘鍏?`public` 璁块棶鍖猴紝涓嬮潰鐨勬垚鍛樹細瀵瑰閮ㄦā鍧楀叕寮€銆?
public:
    /** @brief 鎯版€у垵濮嬪寲锛氶娆′娇鐢ㄦ椂鍒涘缓 Kalman 棰勬祴鍣ㄥ拰涓変釜 PID 鎺у埗鍣?*/
    // 瑙ｉ噴锛氳皟鐢?`EnsureInitialized` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    void EnsureInitialized();

    /** @brief 瀹屽叏閲嶇疆鎺у埗鍣ㄢ€斺€旈噸寤洪娴嬪櫒銆丳ID銆佽繍琛屾椂鐘舵€?*/
    // 瑙ｉ噴锛氳皟鐢?`Reset` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    void Reset();

    /**
     * @brief 澶勭悊 "visual_intercept_start" 鍛戒护
     * @param CmdObj JSON 鍛戒护瀵硅薄锛堝惈 method / interceptor_id / target_id / 鍙傛暟瑕嗙洊绛夛級
     * @param World  褰撳墠 UWorld
     * @return JSON 鍝嶅簲瀛楃涓诧紙鍖呭惈宸插簲鐢ㄧ殑鍙傛暟蹇収锛?
     */
    // 瑙ｉ噴锛氳皟鐢?`HandleStart` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    FString HandleStart(const TSharedPtr<FJsonObject>& CmdObj, UWorld* World);

    /**
     * @brief 澶勭悊 "visual_intercept_update" 鍛戒护鈥斺€旀瘡甯ц皟鐢?
     *        鎺ユ敹妫€娴嬫淇℃伅锛坈x, cy, area, conf锛夛紝杈撳嚭閫熷害 / 鍋忚埅 / 浜戝彴瑙掓寚浠?
     * @param CmdObj JSON 鍛戒护瀵硅薄锛堝惈 has_detection / cx / cy / area / dt 绛夛級
     * @param World  褰撳墠 UWorld
     * @return JSON 鍝嶅簲瀛楃涓诧紙鍚帶鍒堕噺涓庣姸鎬佷俊鎭級
     */
    // 瑙ｉ噴锛氳皟鐢?`HandleUpdate` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    FString HandleUpdate(const TSharedPtr<FJsonObject>& CmdObj, UWorld* World);

    /**
     * @brief 澶勭悊 "visual_intercept_stop" 鍛戒护鈥斺€旀偓鍋滃苟鍋滄鎺у埗
     */
    // 瑙ｉ噴锛氳皟鐢?`HandleStop` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    FString HandleStop(const TSharedPtr<FJsonObject>& CmdObj, UWorld* World);

    /** @brief 鏌ヨ褰撳墠鎷︽埅鐘舵€侊紙JSON 鏍煎紡锛?*/
    // 瑙ｉ噴锛氳皟鐢?`HandleState` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    FString HandleState() const;

// 瑙ｉ噴锛氫粠杩欎竴琛屽紑濮嬭繘鍏?`private` 璁块棶鍖猴紝涓嬮潰鐨勬垚鍛樺彧鍏佽绫诲唴閮ㄨ闂€?
private:
    // 鈹€鈹€鈹€鈹€ 鐘舵€佹灇涓?鈹€鈹€鈹€鈹€

    /** @brief 瑙嗚鎷︽埅鐘舵€佹満鐨勪簲涓樁娈?*/
    // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢灇涓?`EVisualState`锛岀敤浜庣害鏉熶竴缁勬湁闄愮殑鐘舵€佹垨妯″紡鍙栧€笺€?
    enum class EVisualState : uint8
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        Idle,       ///< 绌洪棽鈥斺€旀湭鍚姩鎴栧凡鍋滄
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        Search,     ///< 鎼滅储鈥斺€斾簯鍙版壂鎻?+ 鏈轰綋缂撹浆锛屽鎵剧洰鏍?
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        Track,      ///< 璺熻釜鈥斺€擯ID 绾犲亸锛屼繚鎸佺洰鏍囧湪鐢婚潰涓績
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        Approach,   ///< 閫艰繎鈥斺€旈潰绉凡瓒呰繃 CaptureArea锛屾寔缁墠鍐?
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        Captured,   ///< 鎹曡幏鈥斺€旇窛绂诲皬浜?InterceptDistance锛屼换鍔″畬鎴?
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    };

    // 鈹€鈹€鈹€鈹€ 鍙皟鍙傛暟 鈹€鈹€鈹€鈹€

    /** @brief 瑙嗚鎷︽埅绠楁硶鐨勫叏閮ㄥ彲璋冨弬鏁帮紙鍙€氳繃 JSON 鍦ㄧ嚎瑕嗙洊锛?*/
    // 瑙ｉ噴锛氳繖涓€琛屽０鏄?缁撴瀯浣?`FVisualParams`锛岀敤浜庡皝瑁協visualparams鐩稿叧鐨勬暟鎹笌琛屼负銆?
    struct FVisualParams
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`DesiredArea`锛岀敤浜庝繚瀛榙esiredarea銆?
        float DesiredArea = 0.05f;            ///< 鏈熸湜鐩爣闈㈢Н鍗犳瘮锛堝綊涓€鍖栵級
        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`CaptureArea`锛岀敤浜庝繚瀛橀噰闆哸rea銆?
        float CaptureArea = 0.09f;            ///< 鍒ゅ畾涓?Approach 闃舵鐨勯潰绉槇鍊?
        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`CenterTolX`锛岀敤浜庝繚瀛榗entertolX銆?
        float CenterTolX = 0.08f;             ///< 姘村钩灞呬腑瀹瑰樊锛堝綊涓€鍖栧潗鏍囷級
        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`CenterTolY`锛岀敤浜庝繚瀛榗entertolY銆?
        float CenterTolY = 0.10f;             ///< 鍨傜洿灞呬腑瀹瑰樊
        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`CaptureHoldFrames`锛岀敤浜庝繚瀛橀噰闆唄oldframes銆?
        int32 CaptureHoldFrames = 12;         ///< 杩炵画婊¤冻鎹曡幏鏉′欢澶氬皯甯ф墠纭
        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`LostToSearchFrames`锛岀敤浜庝繚瀛榣osttosearchframes銆?
        int32 LostToSearchFrames = 8;         ///< 杩炵画涓㈠け瓒呰繃璇ュ抚鏁板悗閫€鍥?Search

        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`MaxForwardSpeed`锛岀敤浜庝繚瀛榤axforwardspeed銆?
        float MaxForwardSpeed = 7.5f;         ///< 鏈€澶у墠杩涢€熷害 (m/s)
        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`MaxReverseSpeed`锛岀敤浜庝繚瀛榤axreversespeed銆?
        float MaxReverseSpeed = 1.5f;         ///< 鏈€澶у悗閫€閫熷害 (m/s)
        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`MaxVerticalSpeed`锛岀敤浜庝繚瀛榤axverticalspeed銆?
        float MaxVerticalSpeed = 2.5f;        ///< 鏈€澶у瀭鐩撮€熷害 (m/s)
        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`MaxYawRateDeg`锛岀敤浜庝繚瀛榤axyawratedeg銆?
        float MaxYawRateDeg = 60.0f;          ///< 鏈€澶у亸鑸閫熷害 (掳/s)
        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`RamAreaTarget`锛岀敤浜庝繚瀛榬amareatarget銆?
        float RamAreaTarget = 0.28f;          ///< 鍐插埡闃舵鐩爣闈㈢Н鍗犳瘮闃堝€?
        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`MinRamSpeed`锛岀敤浜庝繚瀛榤inramspeed銆?
        float MinRamSpeed = 1.8f;             ///< 鏈€灏忓啿鍒洪€熷害 (m/s)
        float InterceptDistance = 1.5f;       ///< 拦截判定距离 (m)
        // 中文注释：常规跟踪阶段只前视较短时间，避免预测过远导致画面抖动。
        float TrackLeadTime = 0.18f;          ///< 跟踪阶段预测时域 (s)
        // 中文注释：末段撞击阶段前视更远，机头和速度一起提前量瞄准。
        float RamLeadTime = 0.38f;            ///< 撞击阶段预测时域 (s)

        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`SearchCamYawLimitDeg`锛岀敤浜庝繚瀛榮earchcamyawlimitdeg銆?
        float SearchCamYawLimitDeg = 165.0f;  ///< 鎼滅储鏃朵簯鍙板亸鑸瀬闄?(掳)
        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`SearchCamRateDeg`锛岀敤浜庝繚瀛榮earchcamratedeg銆?
        float SearchCamRateDeg = 70.0f;       ///< 鎼滅储鏃朵簯鍙版壂鎻忛€熷害 (掳/s)
        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`SearchBodyYawRateDeg`锛岀敤浜庝繚瀛榮earchbodyyawratedeg銆?
        float SearchBodyYawRateDeg = 22.0f;   ///< 鎼滅储鏃舵満浣撳亸鑸棆杞€熷害 (掳/s)
        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`SearchCamPitchDeg`锛岀敤浜庝繚瀛榮earchcampitchdeg銆?
        float SearchCamPitchDeg = -5.0f;      ///< 鎼滅储鏃朵簯鍙颁刊浠拌 (掳)
        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`SearchVzAmp`锛岀敤浜庝繚瀛榮earchvzamp銆?
        float SearchVzAmp = 0.4f;             ///< 鎼滅储鏃跺瀭鐩存尟鑽″箙搴?(m/s)

        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`bStopOnCapture`锛岀敤浜庝繚瀛樺竷灏旀爣蹇?stopon閲囬泦銆?
        bool bStopOnCapture = false;          ///< 鎹曡幏鍚庢槸鍚﹁嚜鍔ㄦ偓鍋?
        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`bUseKalman`锛岀敤浜庝繚瀛樺竷灏旀爣蹇?use鍗″皵鏇笺€?
        bool bUseKalman = true;               ///< 鏄惁鍚敤 Kalman 棰勬祴
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    };

    // 鈹€鈹€鈹€鈹€ 杩愯鏃剁姸鎬?鈹€鈹€鈹€鈹€

    /** @brief 鎺у埗鍣ㄨ繍琛屾椂鏁版嵁锛屾瘡娆?BeginSession 鏃堕噸缃?*/
    // 瑙ｉ噴锛氳繖涓€琛屽０鏄?缁撴瀯浣?`FVisualRuntime`锛岀敤浜庡皝瑁協visualruntime鐩稿叧鐨勬暟鎹笌琛屼负銆?
    struct FVisualRuntime
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`bEnabled`锛岀敤浜庝繚瀛樺竷灏旀爣蹇?enabled銆?
        bool bEnabled = false;                           ///< 鏄惁宸插惎鍔?
        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`bCaptured`锛岀敤浜庝繚瀛樺竷灏旀爣蹇?captured銆?
        bool bCaptured = false;                          ///< 鏄惁宸插畬鎴愭崟鑾?

        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`InterceptorId`锛岀敤浜庝繚瀛榠nterceptorid銆?
        FString InterceptorId;                           ///< 鎷︽埅鏃犱汉鏈?ID
        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`TargetId`锛岀敤浜庝繚瀛榯argetid銆?
        FString TargetId;                                ///< 鐩爣鏃犱汉鏈?ID
        // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `FString Method`锛屽畬鎴?fstringmethod 鐨勬洿鏂般€?
        FString Method = TEXT("vision_pid_kalman");       ///< 褰撳墠浣跨敤鐨勬帶鍒舵柟娉?

        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`State`锛岀敤浜庝繚瀛樼姸鎬併€?
        EVisualState State = EVisualState::Idle;         ///< 褰撳墠鐘舵€佹満闃舵

        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`FrameCount`锛岀敤浜庝繚瀛榝ramecount銆?
        int32 FrameCount = 0;                            ///< 宸插鐞嗗抚鏁?
        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`DetectionCount`锛岀敤浜庝繚瀛榙etectioncount銆?
        int32 DetectionCount = 0;                        ///< 绱妫€娴嬪埌鐩爣甯ф暟
        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`LostCount`锛岀敤浜庝繚瀛榣ostcount銆?
        int32 LostCount = 0;                             ///< 杩炵画涓㈠け甯ц鏁?
        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`CaptureCount`锛岀敤浜庝繚瀛橀噰闆哻ount銆?
        int32 CaptureCount = 0;                          ///< 杩炵画婊¤冻鎹曡幏鏉′欢甯ф暟

        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`LastEx`锛岀敤浜庝繚瀛榣astex銆?
        float LastEx = 0.0f;                             ///< 涓婁竴甯ф按骞冲綊涓€鍖栬宸?
        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`LastEy`锛岀敤浜庝繚瀛榣astey銆?
        float LastEy = 0.0f;                             ///< 涓婁竴甯у瀭鐩村綊涓€鍖栬宸?
        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`LastAreaNorm`锛岀敤浜庝繚瀛榣astareanorm銆?
        float LastAreaNorm = 0.0f;                       ///< 涓婁竴甯х洰鏍囬潰绉綊涓€鍖栧€?
        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`LastConf`锛岀敤浜庝繚瀛榣astconf銆?
        float LastConf = 0.0f;                           ///< 涓婁竴甯ф娴嬬疆淇″害
        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`LastYawCmdDeg`锛岀敤浜庝繚瀛榣astyawcmddeg銆?
        float LastYawCmdDeg = 0.0f;                      ///< 涓婁竴甯у亸鑸寚浠?(掳)
        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`LastYawRateDeg`锛岀敤浜庝繚瀛榣astyawratedeg銆?
        float LastYawRateDeg = 0.0f;                     ///< 涓婁竴甯у亸鑸閫熷害 (掳/s)
        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`LastCmdVel`锛岀敤浜庝繚瀛榣astcmdvel銆?
        FVector LastCmdVel = FVector::ZeroVector;         ///< 涓婁竴甯ч€熷害鎸囦护 (m/s)
        float LastTargetDistance = -1.0f;                ///< 上一帧到目标的真实距离 (m)
        // 中文注释：记录本帧控制实际使用的预测时间，便于 Python 端调试显示。
        float LastPredictionLeadTime = 0.0f;

        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`SearchCamYawDeg`锛岀敤浜庝繚瀛榮earchcamyawdeg銆?
        float SearchCamYawDeg = 0.0f;                    ///< 鎼滅储妯″紡涓嬩簯鍙板綋鍓嶅亸鑸
        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`SearchDir`锛岀敤浜庝繚瀛榮earchdir銆?
        float SearchDir = 1.0f;                          ///< 鎼滅储鎵弿鏂瑰悜 (+1 / -1)
        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`SearchTime`锛岀敤浜庝繚瀛榮earchtime銆?
        float SearchTime = 0.0f;                         ///< 鎼滅储鎸佺画鏃堕棿绱

        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`LastFrameW`锛岀敤浜庝繚瀛榣astframeW銆?
        float LastFrameW = 640.0f;                       ///< 涓婁竴甯у浘鍍忓搴?(px)
        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`LastFrameH`锛岀敤浜庝繚瀛榣astframeH銆?
        float LastFrameH = 480.0f;                       ///< 上一帧图像高度 (px)
        // 供 Python 端可视化使用的最近一次量测/跟踪/Kalman 状态。
        bool bLastHasMeasurement = false;
        FVector LastMeasurementFeature = FVector::ZeroVector;
        bool bLastHasTrackFeature = false;
        bool bLastUsedPrediction = false;
        FVector LastTrackFeature = FVector::ZeroVector;
        bool bLastHasPrediction = false;
        FVector LastPredictionFeature = FVector::ZeroVector;
        bool bLastKalmanValid = false;
        FVector LastKalmanPosition = FVector::ZeroVector;
        FVector LastKalmanVelocity = FVector::ZeroVector;
        FVector LastKalmanAcceleration = FVector::ZeroVector;
        float LastKalmanUncertainty = 0.0f;
        float LastKalmanProcessNoise = 0.0f;
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    };

    // 鈹€鈹€鈹€鈹€ 鍐呴儴杈呭姪鏂规硶 鈹€鈹€鈹€鈹€

    /** @brief 浠?JSON 瀵硅薄璇诲彇骞舵牎楠岃瑙夋嫤鎴弬鏁帮紝鏇存柊 Params */
    // 瑙ｉ噴锛氳皟鐢?`ApplyParamsFromJson` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    void ApplyParamsFromJson(const TSharedPtr<FJsonObject>& CmdObj);

    /** @brief 浠呴噸缃繍琛屾椂鐘舵€侊紝涓嶅奖鍝?PID / 棰勬祴鍣?*/
    // 瑙ｉ噴锛氳皟鐢?`ResetRuntimeOnly` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    void ResetRuntimeOnly();

    /** @brief 閲嶇疆鎵€鏈?PID 鎺у埗鍣ㄧ殑绉垎椤逛笌鍘嗗彶璇樊 */
    // 瑙ｉ噴锛氳皟鐢?`ResetPidControllers` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    void ResetPidControllers();

    /** @brief 寮€鍚柊鎷︽埅浼氳瘽锛屽垵濮嬪寲杩愯鏃舵暟鎹?*/
    // 瑙ｉ噴锛氳皟鐢?`BeginSession` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    void BeginSession(const ADronePawn* Interceptor);

    /**
     * @brief 鏌ユ壘鎷︽埅鏃犱汉鏈衡€斺€斾緷娆℃寜 interceptor_id 鈫?MissionRole::Interceptor 鈫?drone_1 鈫?drone_0
     */
    // 瑙ｉ噴锛氳皟鐢?`ResolveInterceptor` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    ADronePawn* ResolveInterceptor(const TSharedPtr<FJsonObject>& CmdObj);

    /** @brief 鏌ユ壘鐩爣鏃犱汉鏈衡€斺€旈€昏緫涓?ResolveInterceptor 绫讳技 */
    // 瑙ｉ噴锛氳皟鐢?`ResolveTarget` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    ADronePawn* ResolveTarget(const TSharedPtr<FJsonObject>& CmdObj);

    /** @brief 瀵?Dt 鎵ц鏈夋晥鎬ф鏌ュ苟闄愬箙鍒?[0.01, 0.25] 绉?*/
    // 瑙ｉ噴锛氳皟鐢?`SafeDt` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    float SafeDt(float Dt) const;

    /** @brief 灏嗗綋鍓?Dt 鍚屾鍒颁笁涓?PID 鎺у埗鍣ㄧ殑鏃堕棿姝ラ暱 */
    // 瑙ｉ噴锛氳皟鐢?`SyncPidTimeStep` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    void SyncPidTimeStep(float Dt);

    // 鈹€鈹€鈹€鈹€ JSON 鍝嶅簲鏋勫缓 鈹€鈹€鈹€鈹€

    /** @brief 鏋勫缓 HandleStart 鐨?JSON 鍝嶅簲锛堝惈瀹屾暣鍙傛暟蹇収锛?*/
    // 瑙ｉ噴锛氳皟鐢?`BuildStartJson` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    FString BuildStartJson() const;

    /** @brief 鏋勫缓 HandleUpdate 鐨?JSON 鍝嶅簲锛堝惈鎺у埗閲?+ 缁熻淇℃伅锛?*/
    // 瑙ｉ噴锛氳皟鐢?`BuildUpdateJson` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    FString BuildUpdateJson(bool bValidControl) const;

    /** @brief 鏋勫缓閿欒 JSON锛歿"status":"error","message":"..."} */
    // 瑙ｉ噴锛氳皟鐢?`MakeError` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    FString MakeError(const FString& Msg) const;

    /** @brief 鏋勫缓鎴愬姛 JSON锛歿"status":"ok","message":"..."} */
    // 瑙ｉ噴锛氳皟鐢?`MakeOk` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    FString MakeOk(const FString& Msg) const;

    // 鈹€鈹€鈹€鈹€ 闈欐€佸伐鍏峰嚱鏁?鈹€鈹€鈹€鈹€

    /** @brief 甯冨皵鍊艰浆 JSON 瀛楅潰閲?("true" / "false") */
    // 瑙ｉ噴锛氳皟鐢?`BoolLiteral` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    static FString BoolLiteral(bool bValue);

    /** @brief 鍋忚埅瑙掑綊涓€鍖栧埌 [-180, 180) */
    // 瑙ｉ噴锛氳皟鐢?`NormalizeYawDeg` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    static float NormalizeYawDeg(float YawDeg);

    /** @brief 鐘舵€佹灇涓捐浆瀛楃涓?*/
    // 瑙ｉ噴锛氳皟鐢?`StateToString` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    static FString StateToString(EVisualState State);

    /**
     * @brief 瑙勮寖鍖栨柟娉曞悕绉帮紙濡?"pid" 鈫?"vision_pid"锛?
     * @return 鏄惁涓烘敮鎸佺殑鏂规硶鍚?
     */
    // 瑙ｉ噴锛氳皟鐢?`NormalizeMethod` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    static bool NormalizeMethod(const FString& InMethod, FString& OutCanonical);

    // 鈹€鈹€鈹€鈹€ 鏍稿績鎺у埗璁＄畻 鈹€鈹€鈹€鈹€

    /**
     * @brief 璺熻釜/閫艰繎妯″紡鐨勬帶鍒惰绠?
     *
     * 鏍规嵁妫€娴嬬壒寰侊紙涓績 + 闈㈢Н锛夎绠楀亸鑸閫熷害銆佸瀭鐩撮€熷害銆佸墠杩涢€熷害銆?
     * 浜戝彴瑙掓寚浠わ紝骞跺彂閫佸埌鎷︽埅鏃犱汉鏈恒€?
     *
     * @param Interceptor    鎷︽埅鏃犱汉鏈?
     * @param Feature        (Cx, Cy, Area) 鍍忕礌鍧愭爣涓庨潰绉?
     * @param FrameW         鍥惧儚瀹藉害 (px)
     * @param FrameH         鍥惧儚楂樺害 (px)
     * @param Confidence     妫€娴嬬疆淇″害
     * @param Dt             鏃堕棿姝ラ暱 (s)
     * @param bDetectionReal 鏄惁涓虹湡瀹炴娴嬶紙鍚﹀垯涓?Kalman 棰勬祴锛?
     * @param bOutCaptured   [out] 鏈抚鏄惁鍒ゅ畾涓烘崟鑾?
     * @return 鏄惁鎴愬姛璁＄畻
     */
    // 瑙ｉ噴锛氳繖涓€琛屽畾涔夊嚱鏁?`ComputeTrackControl`锛屽紑濮嬪疄鐜癱omputetrackcontrol鐨勫叿浣撻€昏緫銆?
    bool ComputeTrackControl(
        // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `ComputeTrackControl` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `Interceptor` 鐢ㄤ簬浼犲叆interceptor銆?
        ADronePawn* Interceptor,
        // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `ComputeTrackControl` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `Feature` 鐢ㄤ簬浼犲叆feature銆?
        const FVector& Feature,
        // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `ComputeTrackControl` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `FrameW` 鐢ㄤ簬浼犲叆frameW銆?
        float FrameW,
        // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `ComputeTrackControl` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `FrameH` 鐢ㄤ簬浼犲叆frameH銆?
        float FrameH,
        // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `ComputeTrackControl` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `Confidence` 鐢ㄤ簬浼犲叆confidence銆?
        float Confidence,
        // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `ComputeTrackControl` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `Dt` 鐢ㄤ簬浼犲叆dt銆?
        float Dt,
        // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `ComputeTrackControl` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `bDetectionReal` 鐢ㄤ簬浼犲叆甯冨皵鏍囧織 detectionreal銆?
        bool bDetectionReal,
        // 瑙ｉ噴锛氳繖涓€琛岀户缁ˉ鍏呭嚱鏁?`ComputeTrackControl` 鐨勫弬鏁板垪琛ㄣ€侀檺瀹氱鎴栬繑鍥炵被鍨嬭鏄庛€?
        bool& bOutCaptured);

    /**
     * @brief 鎼滅储妯″紡鐨勬帶鍒惰绠椻€斺€斾簯鍙板乏鍙虫壂鎻?+ 鏈轰綋缂撴參鏃嬭浆 + 鍨傜洿鎸崱
     */
    // 瑙ｉ噴锛氳皟鐢?`ComputeSearchControl` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    void ComputeSearchControl(ADronePawn* Interceptor, float Dt);

    // 鈹€鈹€鈹€鈹€ 鎴愬憳鍙橀噺 鈹€鈹€鈹€鈹€

// 瑙ｉ噴锛氫粠杩欎竴琛屽紑濮嬭繘鍏?`private` 璁块棶鍖猴紝涓嬮潰鐨勬垚鍛樺彧鍏佽绫诲唴閮ㄨ闂€?
private:
    // 瑙ｉ噴锛氫娇鐢?`UPROPERTY` 瀹忎负涓嬮潰鐨勬垚鍛樺彉閲忓０鏄庡弽灏勫厓鏁版嵁锛屾帶鍒剁紪杈戝櫒鏄剧ず銆佸簭鍒楀寲鍜岃摑鍥惧彲瑙佹€с€?
    UPROPERTY()
    // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`FeaturePredictor`锛岀敤浜庝繚瀛榝eature棰勬祴鍣ㄣ€?
    UKalmanPredictor* FeaturePredictor = nullptr;  ///< Kalman 婊ゆ尝鍣ㄢ€斺€斿钩婊戠壒寰佸苟棰勬祴涓㈠け甯?

    // 瑙ｉ噴锛氫娇鐢?`UPROPERTY` 瀹忎负涓嬮潰鐨勬垚鍛樺彉閲忓０鏄庡弽灏勫厓鏁版嵁锛屾帶鍒剁紪杈戝櫒鏄剧ず銆佸簭鍒楀寲鍜岃摑鍥惧彲瑙佹€с€?
    UPROPERTY()
    // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`YawPid`锛岀敤浜庝繚瀛榶awPID銆?
    UPIDController* YawPid = nullptr;              ///< 鍋忚埅瑙掗€熷害 PID

    // 瑙ｉ噴锛氫娇鐢?`UPROPERTY` 瀹忎负涓嬮潰鐨勬垚鍛樺彉閲忓０鏄庡弽灏勫厓鏁版嵁锛屾帶鍒剁紪杈戝櫒鏄剧ず銆佸簭鍒楀寲鍜岃摑鍥惧彲瑙佹€с€?
    UPROPERTY()
    // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`VerticalPid`锛岀敤浜庝繚瀛榲erticalPID銆?
    UPIDController* VerticalPid = nullptr;         ///< 鍨傜洿閫熷害 PID

    // 瑙ｉ噴锛氫娇鐢?`UPROPERTY` 瀹忎负涓嬮潰鐨勬垚鍛樺彉閲忓０鏄庡弽灏勫厓鏁版嵁锛屾帶鍒剁紪杈戝櫒鏄剧ず銆佸簭鍒楀寲鍜岃摑鍥惧彲瑙佹€с€?
    UPROPERTY()
    // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`ForwardPid`锛岀敤浜庝繚瀛榝orwardPID銆?
    UPIDController* ForwardPid = nullptr;          ///< 鍓嶈繘閫熷害 PID

    // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`Params`锛岀敤浜庝繚瀛榩arams銆?
    FVisualParams Params;                          ///< 绠楁硶鍙傛暟锛堝彲鍦ㄧ嚎璋冩暣锛?
    // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`Runtime`锛岀敤浜庝繚瀛榬untime銆?
    FVisualRuntime Runtime;                        ///< 杩愯鏃剁姸鎬?
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
};






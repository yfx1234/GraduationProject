// 瑙ｉ噴锛氬紩鍏ュ綋鍓嶅疄鐜版枃浠跺搴旂殑澶存枃浠?`GuidanceActor.h`锛屼娇瀹炵幇閮ㄥ垎鑳藉鐪嬪埌绫诲拰鍑芥暟澹版槑銆?
#include "GuidanceActor.h"

// 瑙ｉ噴锛氬紩鍏?`JsonObject.h`锛屼负褰撳墠鏂囦欢琛ュ厖鎵€渚濊禆鐨勭被鍨嬨€佸嚱鏁版垨鎺ュ彛澹版槑銆?
#include "Dom/JsonObject.h"
// 瑙ｉ噴锛氬紩鍏?`GuidanceMethods.h`锛屼负褰撳墠鏂囦欢琛ュ厖鎵€渚濊禆鐨勭被鍨嬨€佸嚱鏁版垨鎺ュ彛澹版槑銆?
#include "GuidanceMethods.h"
// 瑙ｉ噴锛氬紩鍏?`KalmanPredictor.h`锛屼负褰撳墠鏂囦欢琛ュ厖鎵€渚濊禆鐨勭被鍨嬨€佸嚱鏁版垨鎺ュ彛澹版槑銆?
#include "KalmanPredictor.h"
// 瑙ｉ噴锛氬紩鍏?`VisualInterceptController.h`锛屼负褰撳墠鏂囦欢琛ュ厖鎵€渚濊禆鐨勭被鍨嬨€佸嚱鏁版垨鎺ュ彛澹版槑銆?
#include "VisualInterceptController.h"
// 瑙ｉ噴锛氬紩鍏?`AgentManager.h`锛屼负褰撳墠鏂囦欢琛ュ厖鎵€渚濊禆鐨勭被鍨嬨€佸嚱鏁版垨鎺ュ彛澹版槑銆?
#include "GraduationProject/Core/Manager/AgentManager.h"
// 瑙ｉ噴锛氬紩鍏?`TurretPawn.h`锛屼负褰撳墠鏂囦欢琛ュ厖鎵€渚濊禆鐨勭被鍨嬨€佸嚱鏁版垨鎺ュ彛澹版槑銆?
#include "GraduationProject/Turret/TurretPawn.h"

// 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
namespace
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
{
    /** @brief 甯冨皵鍊艰浆 JSON 瀛楅潰閲忔枃鏈?*/
    // 瑙ｉ噴锛氳繖涓€琛屽畾涔夊嚱鏁?`BoolLiteral`锛屽紑濮嬪疄鐜癰oolliteral鐨勫叿浣撻€昏緫銆?
    FString BoolLiteral(bool bValue)
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛屼娇鐢ㄤ笁鐩〃杈惧紡閫夋嫨杩斿洖鍊硷紝鍦ㄦ甯歌矾寰勫拰鍏滃簳璺緞涔嬮棿鍋氬揩閫熷垏鎹€?
        return bValue ? TEXT("true") : TEXT("false");
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }

    /** @brief 灏嗗瓧绗︿覆鏁扮粍搴忓垪鍖栦负 JSON 鏁扮粍鏂囨湰 */
    // 瑙ｉ噴锛氳繖涓€琛屽畾涔夊嚱鏁?`StringArrayToJson`锛屽紑濮嬪疄鐜皊tringarraytojson鐨勫叿浣撻€昏緫銆?
    FString StringArrayToJson(const TArray<FString>& Values)
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `FString Out`锛屽畬鎴?fstringout 鐨勬洿鏂般€?
        FString Out = TEXT("[");
        // 瑙ｉ噴锛氳繖涓€琛屽紑濮?`for` 寰幆锛岀敤浜庢寜鏃㈠畾娆℃暟鎴栧簭鍒楅亶鍘嗘墽琛屽悗缁€昏緫銆?
        for (int32 Index = 0; Index < Values.Num(); ++Index)
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        {
            // 瑙ｉ噴锛氳繖涓€琛屽湪 `Out` 鐨勫師鏈夊熀纭€涓婄户缁疮鍔犳柊閲忥紝鐢ㄤ簬鎸佺画鏇存柊 out銆?
            Out += FString::Printf(TEXT("\"%s\""), *Values[Index].ReplaceCharWithEscapedChar());
            // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
            if (Index + 1 < Values.Num())
            // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
            {
                // 瑙ｉ噴锛氳繖涓€琛屽湪 `Out` 鐨勫師鏈夊熀纭€涓婄户缁疮鍔犳柊閲忥紝鐢ㄤ簬鎸佺画鏇存柊 out銆?
                Out += TEXT(",");
            // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
            }
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        }
        // 瑙ｉ噴锛氳繖涓€琛屽湪 `Out` 鐨勫師鏈夊熀纭€涓婄户缁疮鍔犳柊閲忥紝鐢ㄤ簬鎸佺画鏇存柊 out銆?
        Out += TEXT("]");
        // 瑙ｉ噴锛氳繖涓€琛岃繑鍥炲綋鍓嶅嚱鏁扮殑璁＄畻缁撴灉锛屾妸鎺у埗鏉冧氦鍥炶皟鐢ㄦ柟銆?
        return Out;
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }

    /**
     * @brief 瑙勮寖鍖栨棤浜烘満鑷姩鎷︽埅鏂规硶鍚嶇О
     * @param Method 鍘熷鏂规硶鍚?
     * @return 褰掍竴鍖栧悗鐨勬柟娉曞悕
     *
     * 鐩墠缁熶竴鏀舵暃涓猴細
     * - `pure_pursuit`
     * - `proportional_nav`
     * - `smc`
     */
    // 瑙ｉ噴锛氳繖涓€琛屽畾涔夊嚱鏁?`NormalizeInterceptMethodName`锛屽紑濮嬪疄鐜皀ormalize鎷︽埅methodname鐨勫叿浣撻€昏緫銆?
    FString NormalizeInterceptMethodName(const FString& Method)
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`Normalized`锛岀敤浜庝繚瀛榥ormalized銆?
        FString Normalized = Method;
        // 瑙ｉ噴锛氳皟鐢?`TrimStartAndEndInline` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
        Normalized.TrimStartAndEndInline();
        // 瑙ｉ噴锛氳皟鐢?`ToLowerInline` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
        Normalized.ToLowerInline();
        // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
        if (Normalized == TEXT("pn") || Normalized == TEXT("proportional") || Normalized == TEXT("proportional_navigation"))
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        {
            // 瑙ｉ噴锛氳繖涓€琛岃繑鍥炲綋鍓嶅嚱鏁扮殑璁＄畻缁撴灉锛屾妸鎺у埗鏉冧氦鍥炶皟鐢ㄦ柟銆?
            return TEXT("proportional_nav");
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        }
        // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
        if (Normalized == TEXT("smc") || Normalized == TEXT("sliding_mode") || Normalized == TEXT("slidingmode"))
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        {
            // 瑙ｉ噴锛氳繖涓€琛岃繑鍥炲綋鍓嶅嚱鏁扮殑璁＄畻缁撴灉锛屾妸鎺у埗鏉冧氦鍥炶皟鐢ㄦ柟銆?
            return TEXT("smc");
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        }
        // 瑙ｉ噴锛氳繖涓€琛屼娇鐢ㄤ笁鐩〃杈惧紡閫夋嫨杩斿洖鍊硷紝鍦ㄦ甯歌矾寰勫拰鍏滃簳璺緞涔嬮棿鍋氬揩閫熷垏鎹€?
        return (Normalized == TEXT("pure_pursuit")) ? Normalized : TEXT("pure_pursuit");
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }

    /**
     * @brief 浼拌鐐鐐彛涓栫晫鍧愭爣
     * @param Turret 鐐瀹炰緥
     * @return 鐐彛浣嶇疆锛涜嫢鐐缃戞牸涓嶅瓨鍦紝鍒欓€€鍖栦负 Actor 浣嶇疆
     */
    // 瑙ｉ噴锛氳繖涓€琛屽畾涔夊嚱鏁?`GetTurretMuzzlePosition`锛屽紑濮嬪疄鐜癵etturretmuzzleposition鐨勫叿浣撻€昏緫銆?
    FVector GetTurretMuzzlePosition(const ATurretPawn* Turret)
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
        if (!Turret)
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        {
            // 瑙ｉ噴锛氳繖涓€琛岃繑鍥炲綋鍓嶅嚱鏁扮殑璁＄畻缁撴灉锛屾妸鎺у埗鏉冧氦鍥炶皟鐢ㄦ柟銆?
            return FVector::ZeroVector;
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        }
        // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
        if (!Turret->GunMesh)
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        {
            // 瑙ｉ噴锛氳繖涓€琛岃繑鍥炲綋鍓嶅嚱鏁扮殑璁＄畻缁撴灉锛屾妸鎺у埗鏉冧氦鍥炶皟鐢ㄦ柟銆?
            return Turret->GetActorLocation();
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        }

        // 瑙ｉ噴锛氳繖涓€琛岃繑鍥炲綋鍓嶅嚱鏁扮殑璁＄畻缁撴灉锛屾妸鎺у埗鏉冧氦鍥炶皟鐢ㄦ柟銆?
        return Turret->GunMesh->GetComponentLocation() +
            // 瑙ｉ噴锛氳皟鐢?`GetComponentRotation` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
            Turret->GunMesh->GetComponentRotation().RotateVector(Turret->MuzzleOffset);
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }

    /** @brief 褰撳瓧绗︿覆闈炵┖鏃跺啓鍏?JSON 瀛楁 */
    // 瑙ｉ噴锛氳繖涓€琛屽畾涔夊嚱鏁?`SetStringIfNotEmpty`锛屽紑濮嬪疄鐜皊etstringifnotempty鐨勫叿浣撻€昏緫銆?
    void SetStringIfNotEmpty(const TSharedPtr<FJsonObject>& Object, const TCHAR* FieldName, const FString& Value)
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
        if (Object.IsValid() && !Value.IsEmpty())
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        {
            // 瑙ｉ噴锛氳皟鐢?`SetStringField` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
            Object->SetStringField(FieldName, Value);
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        }
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }

    /** @brief 褰撴暟鍊间笉鏄摠鍏靛€兼椂鍐欏叆 JSON 瀛楁 */
    // 瑙ｉ噴锛氳繖涓€琛屽畾涔夊嚱鏁?`SetNumberIfValid`锛屽紑濮嬪疄鐜皊etnumberifvalid鐨勫叿浣撻€昏緫銆?
    void SetNumberIfValid(const TSharedPtr<FJsonObject>& Object, const TCHAR* FieldName, float Value, float InvalidSentinel = -1.0f)
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
        if (Object.IsValid() && !FMath::IsNearlyEqual(Value, InvalidSentinel))
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        {
            // 瑙ｉ噴锛氳皟鐢?`SetNumberField` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
            Object->SetNumberField(FieldName, Value);
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        }
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }

    /** @brief 褰撴暣鍨嬪弬鏁版湁鏁堟椂鍐欏叆 JSON 瀛楁 */
    // 瑙ｉ噴锛氳繖涓€琛屽畾涔夊嚱鏁?`SetIntIfValid`锛屽紑濮嬪疄鐜皊etintifvalid鐨勫叿浣撻€昏緫銆?
    void SetIntIfValid(const TSharedPtr<FJsonObject>& Object, const TCHAR* FieldName, int32 Value)
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
        if (Object.IsValid() && Value >= 0)
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        {
            // 瑙ｉ噴锛氳皟鐢?`SetNumberField` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
            Object->SetNumberField(FieldName, Value);
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        }
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
}

/** @brief 鏋勯€?GuidanceActor锛屽苟灏嗗叾璁句负闅愯棌鐨勫悗鍙板崗璋?Actor */
// 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
AGuidanceActor::AGuidanceActor()
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
{
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `PrimaryActorTick.bCanEverTick`锛屽畬鎴?甯冨皵鏍囧織 canevertick 鐨勬洿鏂般€?
    PrimaryActorTick.bCanEverTick = false;
    // 瑙ｉ噴锛氳皟鐢?`SetActorHiddenInGame` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    SetActorHiddenInGame(true);
    // 瑙ｉ噴锛氳皟鐢?`SetActorEnableCollision` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    SetActorEnableCollision(false);
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
}

/**
 * @brief 鍚姩鏃舵敞鍐屽埌 AgentManager
 *
 * 濡傛灉鍦烘櫙涓凡鏈夋寚鍚戞湰瀵硅薄鐨勬敞鍐岄」锛屽垯浼樺厛澶嶇敤鍏?ID锛?
 * 閬垮厤鍚屼竴涓?GuidanceActor 琚噸澶嶆敞鍐屼负澶氫釜鍚嶇О銆?
 */
// 瑙ｉ噴锛氳繖涓€琛屽畾涔夊嚱鏁?`BeginPlay`锛屽紑濮嬪疄鐜癰eginplay鐨勫叿浣撻€昏緫銆?
void AGuidanceActor::BeginPlay()
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
{
    // 瑙ｉ噴锛氳皟鐢?`BeginPlay` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    Super::BeginPlay();

    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `UAgentManager* Manager`锛屽畬鎴?uagent绠＄悊鍣ㄧ鐞嗗櫒 鐨勬洿鏂般€?
    UAgentManager* Manager = UAgentManager::GetInstance();
    // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
    if (Manager)
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`ExistingId`锛岀敤浜庝繚瀛榚xistingid銆?
        FString ExistingId;
        // 瑙ｉ噴锛氳皟鐢?`GetAllAgentIds` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
        const TArray<FString> ExistingIds = Manager->GetAllAgentIds();
        // 瑙ｉ噴锛氳繖涓€琛屽紑濮?`for` 寰幆锛岀敤浜庢寜鏃㈠畾娆℃暟鎴栧簭鍒楅亶鍘嗘墽琛屽悗缁€昏緫銆?
        for (const FString& Id : ExistingIds)
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        {
            // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
            if (Manager->GetAgent(Id) == this)
            // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
            {
                // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `ExistingId`锛屽畬鎴?existingid 鐨勬洿鏂般€?
                ExistingId = Id;
                // 瑙ｉ噴锛氳繖涓€琛岀珛鍗宠烦鍑哄綋鍓嶅惊鐜垨 `switch` 鍒嗘敮锛岄伩鍏嶇户缁墽琛屽悗缁垎鏀€?
                break;
            // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
            }
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        }

        // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
        if (!ExistingId.IsEmpty() && ExistingId != GuidanceId)
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        {
            // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `GuidanceId`锛屽畬鎴?鍒跺id 鐨勬洿鏂般€?
            GuidanceId = ExistingId;
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        }

        // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
        if (Manager->GetAgent(GuidanceId) != this)
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        {
            // 瑙ｉ噴锛氳皟鐢?`RegisterAgent` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
            Manager->RegisterAgent(GuidanceId, this);
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        }
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
}

/**
 * @brief 缁撴潫鏃堕噴鏀惧唴閮ㄨ祫婧愬苟娉ㄩ攢鑷韩
 * @param EndPlayReason Actor 缁撴潫鍘熷洜
 */
// 瑙ｉ噴锛氳繖涓€琛屽畾涔夊嚱鏁?`EndPlay`锛屽紑濮嬪疄鐜癳ndplay鐨勫叿浣撻€昏緫銆?
void AGuidanceActor::EndPlay(const EEndPlayReason::Type EndPlayReason)
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
{
    // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
    if (CurrentMethod)
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`CurrentMethod`锛岀敤浜庝繚瀛榗urrentmethod銆?
        delete CurrentMethod;
        // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `CurrentMethod`锛屽畬鎴?currentmethod 鐨勬洿鏂般€?
        CurrentMethod = nullptr;
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }

    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `UAgentManager* Manager`锛屽畬鎴?uagent绠＄悊鍣ㄧ鐞嗗櫒 鐨勬洿鏂般€?
    UAgentManager* Manager = UAgentManager::GetInstance();
    // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
    if (Manager && !GuidanceId.IsEmpty() && Manager->GetAgent(GuidanceId) == this)
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳皟鐢?`UnregisterAgent` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
        Manager->UnregisterAgent(GuidanceId);
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }

    // 瑙ｉ噴锛氳皟鐢?`EndPlay` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    Super::EndPlay(EndPlayReason);
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
}

/**
 * @brief 鎯版€у垵濮嬪寲鏍稿績瀛愭ā鍧?
 *
 * - `Predictor`锛氱洰鏍囩姸鎬佷及璁′笌寤惰繜琛ュ伩锛?
 * - `VisualInterceptController`锛氳瑙夋嫤鎴棴鐜帶鍒讹紱
 * - `CurrentMethod`锛氶粯璁ら噰鐢ㄩ娴嬪埗瀵肩畻娉曘€?
 */
// 瑙ｉ噴锛氳繖涓€琛屽畾涔夊嚱鏁?`EnsureInitialized`锛屽紑濮嬪疄鐜癳nsureinitialized鐨勫叿浣撻€昏緫銆?
void AGuidanceActor::EnsureInitialized()
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
{
    // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
    if (!Predictor)
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `Predictor`锛屽畬鎴?棰勬祴鍣?鐨勬洿鏂般€?
        Predictor = NewObject<UKalmanPredictor>(this);
        // 瑙ｉ噴锛氳皟鐢?`Initialize` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
        Predictor->Initialize(100.0f, 0.01f);
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }

    // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
    if (!VisualInterceptController)
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `VisualInterceptController`锛屽畬鎴?瑙嗚鎷︽埅鎺у埗鍣?鐨勬洿鏂般€?
        VisualInterceptController = NewObject<UVisualInterceptController>(this);
        // 瑙ｉ噴锛氳皟鐢?`EnsureInitialized` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
        VisualInterceptController->EnsureInitialized();
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }

    // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
    if (!CurrentMethod)
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `CurrentMethod`锛屽畬鎴?currentmethod 鐨勬洿鏂般€?
        CurrentMethod = new FPredictiveGuidance(Predictor, 3);
        // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `CurrentMethodName`锛屽畬鎴?currentmethodname 鐨勬洿鏂般€?
        CurrentMethodName = TEXT("predictive");
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
}

/**
 * @brief 鎸変换鍔¤鑹叉煡鎵鹃涓尮閰嶇殑鏃犱汉鏈?
 * @param Manager AgentManager 瀹炰緥
 * @param DesiredRole 鐩爣瑙掕壊
 * @param ExcludeId 闇€瑕佽烦杩囩殑 ID
 * @return 鎵惧埌鐨勬棤浜烘満鎸囬拡锛屽け璐ヨ繑鍥?nullptr
 */
// 瑙ｉ噴锛氳繖涓€琛屽畾涔夊嚱鏁?`FindDroneByRole`锛屽紑濮嬪疄鐜癴ind鏃犱汉鏈篵yrole鐨勫叿浣撻€昏緫銆?
ADronePawn* AGuidanceActor::FindDroneByRole(UAgentManager* Manager, EDroneMissionRole DesiredRole, const FString& ExcludeId) const
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
{
    // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
    if (!Manager)
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛岃繑鍥炲綋鍓嶅嚱鏁扮殑璁＄畻缁撴灉锛屾妸鎺у埗鏉冧氦鍥炶皟鐢ㄦ柟銆?
        return nullptr;
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }

    // 瑙ｉ噴锛氳皟鐢?`GetAllAgentIds` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    const TArray<FString> AgentIds = Manager->GetAllAgentIds();
    // 瑙ｉ噴锛氳繖涓€琛屽紑濮?`for` 寰幆锛岀敤浜庢寜鏃㈠畾娆℃暟鎴栧簭鍒楅亶鍘嗘墽琛屽悗缁€昏緫銆?
    for (const FString& AgentId : AgentIds)
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
        if (!ExcludeId.IsEmpty() && AgentId == ExcludeId)
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        {
            // 瑙ｉ噴锛氳繖涓€琛岃烦杩囨湰杞惊鐜墿浣欒鍙ワ紝鐩存帴杩涘叆涓嬩竴杞凯浠ｃ€?
            continue;
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        }

        // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `ADronePawn* Drone`锛屽畬鎴?adronePawn鏃犱汉鏈?鐨勬洿鏂般€?
        ADronePawn* Drone = Cast<ADronePawn>(Manager->GetAgent(AgentId));
        // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
        if (Drone && Drone->MissionRole == DesiredRole)
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        {
            // 瑙ｉ噴锛氳繖涓€琛岃繑鍥炲綋鍓嶅嚱鏁扮殑璁＄畻缁撴灉锛屾妸鎺у埗鏉冧氦鍥炶皟鐢ㄦ柟銆?
            return Drone;
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        }
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }

    // 瑙ｉ噴锛氳繖涓€琛岃繑鍥炲綋鍓嶅嚱鏁扮殑璁＄畻缁撴灉锛屾妸鎺у埗鏉冧氦鍥炶皟鐢ㄦ柟銆?
    return nullptr;
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
}

/** @brief 鐢熸垚缁熶竴閿欒杩斿洖 JSON */
// 瑙ｉ噴锛氳繖涓€琛屽畾涔夊嚱鏁?`MakeError`锛屽紑濮嬪疄鐜癿akeerror鐨勫叿浣撻€昏緫銆?
FString AGuidanceActor::MakeError(const FString& Msg) const
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
{
    // 瑙ｉ噴锛氳繖涓€琛岃繑鍥炲綋鍓嶅嚱鏁扮殑璁＄畻缁撴灉锛屾妸鎺у埗鏉冧氦鍥炶皟鐢ㄦ柟銆?
    return FString::Printf(TEXT("{\"status\":\"error\",\"message\":\"%s\"}"), *Msg.ReplaceCharWithEscapedChar());
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
}

/** @brief 鐢熸垚缁熶竴鎴愬姛杩斿洖 JSON */
// 瑙ｉ噴锛氳繖涓€琛屽畾涔夊嚱鏁?`MakeOk`锛屽紑濮嬪疄鐜癿akeok鐨勫叿浣撻€昏緫銆?
FString AGuidanceActor::MakeOk(const FString& Msg) const
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
{
    // 瑙ｉ噴锛氳繖涓€琛岃繑鍥炲綋鍓嶅嚱鏁扮殑璁＄畻缁撴灉锛屾妸鎺у埗鏉冧氦鍥炶皟鐢ㄦ柟銆?
    return FString::Printf(TEXT("{\"status\":\"ok\",\"message\":\"%s\"}"), *Msg.ReplaceCharWithEscapedChar());
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
}

/**
 * @brief 鍒囨崲甯歌鐬勫噯/鍒跺绠楁硶
 * @param Method 绠楁硶鍚嶇О
 * @param NavConstant 姣斾緥瀵煎紩瀵艰埅甯告暟
 * @param Iterations 棰勬祴鍒跺杩唬娆℃暟
 */
// 瑙ｉ噴锛氳繖涓€琛屽畾涔夊嚱鏁?`SetMethod`锛屽紑濮嬪疄鐜皊etmethod鐨勫叿浣撻€昏緫銆?
FString AGuidanceActor::SetMethod(FString Method, float NavConstant, int32 Iterations)
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
{
    // 瑙ｉ噴锛氳皟鐢?`EnsureInitialized` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    EnsureInitialized();

    // 瑙ｉ噴锛氳皟鐢?`TrimStartAndEndInline` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    Method.TrimStartAndEndInline();
    // 瑙ｉ噴锛氳皟鐢?`ToLowerInline` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    Method.ToLowerInline();

    // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
    if (CurrentMethod)
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`CurrentMethod`锛岀敤浜庝繚瀛榗urrentmethod銆?
        delete CurrentMethod;
        // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `CurrentMethod`锛屽畬鎴?currentmethod 鐨勬洿鏂般€?
        CurrentMethod = nullptr;
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }

    // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
    if (Method == TEXT("direct"))
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `CurrentMethod`锛屽畬鎴?currentmethod 鐨勬洿鏂般€?
        CurrentMethod = new FDirectAiming();
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }
    // 瑙ｉ噴锛氳繖涓€琛屽０鏄庤ˉ鍏呭垎鏀紝褰撲笂涓€涓潯浠朵笉鎴愮珛涓斿綋鍓嶆潯浠舵垚绔嬫椂鎵ц銆?
    else if (Method == TEXT("proportional"))
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `CurrentMethod`锛屽畬鎴?currentmethod 鐨勬洿鏂般€?
        CurrentMethod = new FProportionalNavigation(NavConstant <= 0.0f ? 4.0f : NavConstant);
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }
    // 瑙ｉ噴锛氳繖涓€琛屽０鏄庤ˉ鍏呭垎鏀紝褰撲笂涓€涓潯浠朵笉鎴愮珛涓斿綋鍓嶆潯浠舵垚绔嬫椂鎵ц銆?
    else if (Method == TEXT("predictive"))
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `CurrentMethod`锛屽畬鎴?currentmethod 鐨勬洿鏂般€?
        CurrentMethod = new FPredictiveGuidance(Predictor, Iterations > 0 ? Iterations : 3);
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }
    // 瑙ｉ噴锛氳繖涓€琛岀粰鍑哄厹搴曞垎鏀紝褰撲笂闈㈢殑鏉′欢閮戒笉婊¤冻鏃舵墽琛屻€?
    else
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛岃繑鍥炲綋鍓嶅嚱鏁扮殑璁＄畻缁撴灉锛屾妸鎺у埗鏉冧氦鍥炶皟鐢ㄦ柟銆?
        return MakeError(FString::Printf(TEXT("Unknown method: %s"), *Method));
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }

    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `CurrentMethodName`锛屽畬鎴?currentmethodname 鐨勬洿鏂般€?
    CurrentMethodName = Method;
    // 瑙ｉ噴锛氳繖涓€琛岃繑鍥炲綋鍓嶅嚱鏁扮殑璁＄畻缁撴灉锛屾妸鎺у埗鏉冧氦鍥炶皟鐢ㄦ柟銆?
    return MakeOk(FString::Printf(TEXT("Method set to %s"), *Method));
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
}

/**
 * @brief 鏇存柊鏃犱汉鏈鸿嚜鍔ㄦ嫤鎴殑榛樿鏂规硶涓庡弬鏁?
 * @param Method 鎷︽埅鏂规硶鍚?
 * @param Speed 鎷︽埅閫熷害涓婇檺锛坢/s锛?
 * @param NavGain 瀵艰埅澧炵泭
 * @param LeadTime 鐩爣鍓嶇疆棰勬祴鏃堕棿锛坰锛?
 * @param CaptureRadiusValue 鎹曡幏鍗婂緞锛坢锛?
 */
// 瑙ｉ噴锛氳繖涓€琛屽畾涔夊嚱鏁?`SetInterceptMethod`锛屽紑濮嬪疄鐜皊et鎷︽埅method鐨勫叿浣撻€昏緫銆?
FString AGuidanceActor::SetInterceptMethod(FString Method, float Speed, float NavGain, float LeadTime, float CaptureRadiusValue)
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
{
    // 瑙ｉ噴锛氳皟鐢?`EnsureInitialized` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    EnsureInitialized();

    // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
    if (!Method.IsEmpty())
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `CurrentInterceptMethod`锛屽畬鎴?current鎷︽埅method 鐨勬洿鏂般€?
        CurrentInterceptMethod = NormalizeInterceptMethodName(Method);
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }
    // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
    if (Speed > 0.0f)
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `InterceptorSpeed`锛屽畬鎴?interceptorspeed 鐨勬洿鏂般€?
        InterceptorSpeed = Speed;
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }
    // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
    if (NavGain > 0.0f)
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `InterceptNavGain`锛屽畬鎴?鎷︽埅navgain 鐨勬洿鏂般€?
        InterceptNavGain = NavGain;
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }
    // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
    if (LeadTime >= 0.0f)
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `InterceptLeadTime`锛屽畬鎴?鎷︽埅leadtime 鐨勬洿鏂般€?
        InterceptLeadTime = LeadTime;
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }
    // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
    if (CaptureRadiusValue > 0.0f)
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `CaptureRadius`锛屽畬鎴?閲囬泦radius 鐨勬洿鏂般€?
        CaptureRadius = CaptureRadiusValue;
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }

    // 瑙ｉ噴锛氳繖涓€琛岃繑鍥炲綋鍓嶅嚱鏁扮殑璁＄畻缁撴灉锛屾妸鎺у埗鏉冧氦鍥炶皟鐢ㄦ柟銆?
    return FString::Printf(
        // 瑙ｉ噴锛氳繖涓€琛屼綅浜庢瀯閫犲嚱鏁板垵濮嬪寲鍒楄〃涓紝鎶?`TEXT` 鐩存帴鍒濆鍖栦负 `"{\"status\":\"ok\",\"intercept_method\":\"%s\",\"speed\":%.2f,\"nav_gain\":%.2f,\"lead_time\":%.2f,\"capture_radius\":%.2f}"`锛屽噺灏戣繘鍏ュ嚱鏁颁綋鍚庣殑棰濆璧嬪€煎紑閿€銆?
        TEXT("{\"status\":\"ok\",\"intercept_method\":\"%s\",\"speed\":%.2f,\"nav_gain\":%.2f,\"lead_time\":%.2f,\"capture_radius\":%.2f}"),
        *CurrentInterceptMethod,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        InterceptorSpeed,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        InterceptNavGain,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        InterceptLeadTime,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        CaptureRadius);
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
}

/**
 * @brief 缁熻褰撳墠鍦烘櫙涓彲鍙備笌鎷︽埅浠诲姟鐨勬棤浜烘満
 * @return 鐩爣鏈轰笌鎷︽埅鏈?ID 鍒楄〃 JSON
 */
// 瑙ｉ噴锛氳繖涓€琛屽畾涔夊嚱鏁?`ListInterceptAgents`锛屽紑濮嬪疄鐜板垪琛ㄦ嫤鎴猘gents鐨勫叿浣撻€昏緫銆?
FString AGuidanceActor::ListInterceptAgents()
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
{
    // 瑙ｉ噴锛氳皟鐢?`EnsureInitialized` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    EnsureInitialized();

    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `UAgentManager* Manager`锛屽畬鎴?uagent绠＄悊鍣ㄧ鐞嗗櫒 鐨勬洿鏂般€?
    UAgentManager* Manager = UAgentManager::GetInstance();
    // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
    if (!Manager)
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛岃繑鍥炲綋鍓嶅嚱鏁扮殑璁＄畻缁撴灉锛屾妸鎺у埗鏉冧氦鍥炶皟鐢ㄦ柟銆?
        return MakeError(TEXT("Agent manager unavailable"));
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }

    // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`Targets`锛岀敤浜庝繚瀛榯argets銆?
    TArray<FString> Targets;
    // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`Interceptors`锛岀敤浜庝繚瀛榠nterceptors銆?
    TArray<FString> Interceptors;

    // 瑙ｉ噴锛氳皟鐢?`GetAllAgentIds` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    const TArray<FString> AgentIds = Manager->GetAllAgentIds();
    // 瑙ｉ噴锛氳繖涓€琛屽紑濮?`for` 寰幆锛岀敤浜庢寜鏃㈠畾娆℃暟鎴栧簭鍒楅亶鍘嗘墽琛屽悗缁€昏緫銆?
    for (const FString& AgentId : AgentIds)
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `ADronePawn* Drone`锛屽畬鎴?adronePawn鏃犱汉鏈?鐨勬洿鏂般€?
        ADronePawn* Drone = Cast<ADronePawn>(Manager->GetAgent(AgentId));
        // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
        if (!Drone)
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        {
            // 瑙ｉ噴锛氳繖涓€琛岃烦杩囨湰杞惊鐜墿浣欒鍙ワ紝鐩存帴杩涘叆涓嬩竴杞凯浠ｃ€?
            continue;
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        }

        // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
        if (Drone->MissionRole == EDroneMissionRole::Target)
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        {
            // 瑙ｉ噴锛氳皟鐢?`Add` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
            Targets.Add(Drone->DroneId);
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        }
        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庤ˉ鍏呭垎鏀紝褰撲笂涓€涓潯浠朵笉鎴愮珛涓斿綋鍓嶆潯浠舵垚绔嬫椂鎵ц銆?
        else if (Drone->MissionRole == EDroneMissionRole::Interceptor)
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        {
            // 瑙ｉ噴锛氳皟鐢?`Add` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
            Interceptors.Add(Drone->DroneId);
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        }
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }

    // 瑙ｉ噴锛氳繖涓€琛岃繑鍥炲綋鍓嶅嚱鏁扮殑璁＄畻缁撴灉锛屾妸鎺у埗鏉冧氦鍥炶皟鐢ㄦ柟銆?
    return FString::Printf(
        // 瑙ｉ噴锛氳繖涓€琛屼綅浜庢瀯閫犲嚱鏁板垵濮嬪寲鍒楄〃涓紝鎶?`TEXT` 鐩存帴鍒濆鍖栦负 `"{\"status\":\"ok\",\"targets\":%s,\"interceptors\":%s,\"target_count\":%d,\"interceptor_count\":%d}"`锛屽噺灏戣繘鍏ュ嚱鏁颁綋鍚庣殑棰濆璧嬪€煎紑閿€銆?
        TEXT("{\"status\":\"ok\",\"targets\":%s,\"interceptors\":%s,\"target_count\":%d,\"interceptor_count\":%d}"),
        *StringArrayToJson(Targets),
        *StringArrayToJson(Interceptors),
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        Targets.Num(),
        // 瑙ｉ噴锛氳皟鐢?`Num` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
        Interceptors.Num());
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
}

/**
 * @brief 璁＄畻涓€姝ユ棤浜烘満鑷姩鎷︽埅鎺у埗骞剁洿鎺ュ啓鍏ユ嫤鎴満
 * @param InterceptorId 鎷︽埅鏈?ID
 * @param TargetId 鐩爣鏈?ID
 * @param Method 涓存椂瑕嗙洊鐨勬嫤鎴柟娉?
 * @param Speed 涓存椂瑕嗙洊鐨勯€熷害涓婇檺锛坢/s锛?
 * @param NavGain 涓存椂瑕嗙洊鐨勫鑸鐩?N
 * @param LeadTime 涓存椂瑕嗙洊鐨勯娴嬫椂闂达紙s锛?
 * @param CaptureRadiusValue 涓存椂瑕嗙洊鐨勬崟鑾峰崐寰勶紙m锛?
 * @param bStopOnCapture 鎹曡幏鍚庢槸鍚︽偓鍋?
 * @return 褰撳墠涓€姝ユ嫤鎴姸鎬?JSON
 *
 * 褰撳墠瀹炵幇閲囩敤涓ょ被閫熷害鎸囦护妯″瀷锛?
 * 1. 绾拷韪?鍓嶇疆娉曪細
 *    - $p_{pred} = p_t + v_t \cdot t_{lead}$
 *    - $v_{cmd} = speed \cdot \mathrm{normalize}(p_{pred} - p_i)$
 * 2. 绠€鍖栨瘮渚嬪鑸細
 *    - $v_{rel} = v_t - v_i$
 *    - $LOS = \mathrm{normalize}(p_t - p_i)$
 *    - $v_{rel}^{lat} = v_{rel} - (v_{rel}\cdot LOS)LOS$
 *    - $v_{cmd} = v_t + N \cdot v_{rel}^{lat}$
 *
 * 鎹曡幏鍒ゆ嵁涓猴細
 * $\|p_t - p_i\| \le R_{capture}$銆?
 */
// 瑙ｉ噴锛氳繖涓€琛屽畾涔夊嚱鏁?`AutoIntercept`锛屽紑濮嬪疄鐜癮uto鎷︽埅鐨勫叿浣撻€昏緫銆?
FString AGuidanceActor::AutoIntercept(
    // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `AutoIntercept` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `InterceptorId` 鐢ㄤ簬浼犲叆interceptorid銆?
    FString InterceptorId,
    // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `AutoIntercept` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `TargetId` 鐢ㄤ簬浼犲叆targetid銆?
    FString TargetId,
    // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `AutoIntercept` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `Method` 鐢ㄤ簬浼犲叆method銆?
    FString Method,
    // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `AutoIntercept` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `Speed` 鐢ㄤ簬浼犲叆speed銆?
    float Speed,
    // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `AutoIntercept` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `NavGain` 鐢ㄤ簬浼犲叆navgain銆?
    float NavGain,
    // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `AutoIntercept` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `LeadTime` 鐢ㄤ簬浼犲叆leadtime銆?
    float LeadTime,
    // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `AutoIntercept` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `CaptureRadiusValue` 鐢ㄤ簬浼犲叆閲囬泦radiusvalue銆?
    float CaptureRadiusValue,
    // 瑙ｉ噴锛氳繖涓€琛屾敹鏉熷嚱鏁?`AutoIntercept` 鐨勭鍚嶏紝鍚庨潰浼氳繘鍏ュ疄鐜颁綋鎴栦互鍒嗗彿缁撴潫澹版槑銆?
    bool bStopOnCapture)
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
{
    // 瑙ｉ噴锛氳皟鐢?`EnsureInitialized` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    EnsureInitialized();

    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `UAgentManager* Manager`锛屽畬鎴?uagent绠＄悊鍣ㄧ鐞嗗櫒 鐨勬洿鏂般€?
    UAgentManager* Manager = UAgentManager::GetInstance();
    // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
    if (!Manager)
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛岃繑鍥炲綋鍓嶅嚱鏁扮殑璁＄畻缁撴灉锛屾妸鎺у埗鏉冧氦鍥炶皟鐢ㄦ柟銆?
        return MakeError(TEXT("Agent manager unavailable"));
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }

    // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`TargetDrone`锛岀敤浜庝繚瀛榯arget鏃犱汉鏈恒€?
    ADronePawn* TargetDrone = nullptr;
    // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`InterceptorDrone`锛岀敤浜庝繚瀛榠nterceptor鏃犱汉鏈恒€?
    ADronePawn* InterceptorDrone = nullptr;

    // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
    if (!TargetId.IsEmpty())
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `TargetDrone`锛屽畬鎴?target鏃犱汉鏈?鐨勬洿鏂般€?
        TargetDrone = Cast<ADronePawn>(Manager->GetAgent(TargetId));
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }
    // 瑙ｉ噴锛氳繖涓€琛岀粰鍑哄厹搴曞垎鏀紝褰撲笂闈㈢殑鏉′欢閮戒笉婊¤冻鏃舵墽琛屻€?
    else
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `TargetDrone`锛屽畬鎴?target鏃犱汉鏈?鐨勬洿鏂般€?
        TargetDrone = FindDroneByRole(Manager, EDroneMissionRole::Target);
        // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
        if (TargetDrone)
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        {
            // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `TargetId`锛屽畬鎴?targetid 鐨勬洿鏂般€?
            TargetId = TargetDrone->DroneId;
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        }
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }

    // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
    if (!InterceptorId.IsEmpty())
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `InterceptorDrone`锛屽畬鎴?interceptor鏃犱汉鏈?鐨勬洿鏂般€?
        InterceptorDrone = Cast<ADronePawn>(Manager->GetAgent(InterceptorId));
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }
    // 瑙ｉ噴锛氳繖涓€琛岀粰鍑哄厹搴曞垎鏀紝褰撲笂闈㈢殑鏉′欢閮戒笉婊¤冻鏃舵墽琛屻€?
    else
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `const FString ExcludeTarget`锛屽畬鎴?constfstringexcludetarget 鐨勬洿鏂般€?
        const FString ExcludeTarget = TargetDrone ? TargetDrone->DroneId : TEXT("");
        // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `InterceptorDrone`锛屽畬鎴?interceptor鏃犱汉鏈?鐨勬洿鏂般€?
        InterceptorDrone = FindDroneByRole(Manager, EDroneMissionRole::Interceptor, ExcludeTarget);
        // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
        if (InterceptorDrone)
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        {
            // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `InterceptorId`锛屽畬鎴?interceptorid 鐨勬洿鏂般€?
            InterceptorId = InterceptorDrone->DroneId;
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        }
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }

    // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
    if (!TargetDrone)
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛岃繑鍥炲綋鍓嶅嚱鏁扮殑璁＄畻缁撴灉锛屾妸鎺у埗鏉冧氦鍥炶皟鐢ㄦ柟銆?
        return MakeError(TEXT("Target drone not found. Provide target_id or set MissionRole=Target."));
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }
    // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
    if (!InterceptorDrone)
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛岃繑鍥炲綋鍓嶅嚱鏁扮殑璁＄畻缁撴灉锛屾妸鎺у埗鏉冧氦鍥炶皟鐢ㄦ柟銆?
        return MakeError(TEXT("Interceptor drone not found. Provide interceptor_id or set MissionRole=Interceptor."));
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }
    // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
    if (TargetDrone == InterceptorDrone)
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛岃繑鍥炲綋鍓嶅嚱鏁扮殑璁＄畻缁撴灉锛屾妸鎺у埗鏉冧氦鍥炶皟鐢ㄦ柟銆?
        return MakeError(TEXT("target_id and interceptor_id must be different"));
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }

    // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`EffectiveMethod`锛岀敤浜庝繚瀛榚ffectivemethod銆?
    FString EffectiveMethod = CurrentInterceptMethod;
    // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
    if (!Method.IsEmpty())
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `EffectiveMethod`锛屽畬鎴?effectivemethod 鐨勬洿鏂般€?
        EffectiveMethod = NormalizeInterceptMethodName(Method);
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }

    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `const float EffectiveSpeed`锛屽畬鎴?constfloateffectivespeed 鐨勬洿鏂般€?
    const float EffectiveSpeed = (Speed > 0.0f) ? Speed : InterceptorSpeed;
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `const float EffectiveNavGain`锛屽畬鎴?constfloateffectivenavgain 鐨勬洿鏂般€?
    const float EffectiveNavGain = (NavGain > 0.0f) ? NavGain : InterceptNavGain;
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `const float EffectiveLeadTime`锛屽畬鎴?constfloateffectiveleadtime 鐨勬洿鏂般€?
    const float EffectiveLeadTime = (LeadTime >= 0.0f) ? LeadTime : InterceptLeadTime;
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `const float EffectiveCaptureRadius`锛屽畬鎴?constfloateffective閲囬泦radius 鐨勬洿鏂般€?
    const float EffectiveCaptureRadius = (CaptureRadiusValue > 0.0f) ? CaptureRadiusValue : CaptureRadius;

    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `const FVector InterceptorPos`锛屽畬鎴?constfvectorinterceptorpos 鐨勬洿鏂般€?
    const FVector InterceptorPos = InterceptorDrone->GetCurrentPosition();
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `const FVector InterceptorVel`锛屽畬鎴?constfvectorinterceptorvel 鐨勬洿鏂般€?
    const FVector InterceptorVel = InterceptorDrone->GetCurrentVelocity();
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `const FVector TargetPos`锛屽畬鎴?constfvectortargetpos 鐨勬洿鏂般€?
    const FVector TargetPos = TargetDrone->GetCurrentPosition();
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `const FVector TargetVel`锛屽畬鎴?constfvectortargetvel 鐨勬洿鏂般€?
    const FVector TargetVel = TargetDrone->GetCurrentVelocity();

    // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`RelativePos`锛岀敤浜庝繚瀛榬elativepos銆?
    const FVector RelativePos = TargetPos - InterceptorPos;
    // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`RelativeVel`锛岀敤浜庝繚瀛榬elativevel銆?
    const FVector RelativeVel = TargetVel - InterceptorVel;
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍚戦噺妯￠暱鍐欏叆 `const float Distance`锛岀敤浜庤〃绀鸿窛绂汇€侀€熷害澶у皬鎴栦笉纭畾搴︺€?
    const float Distance = RelativePos.Size();
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `const FVector LOS`锛屽畬鎴?constfvectorlos 鐨勬洿鏂般€?
    const FVector LOS = RelativePos.GetSafeNormal();

    // 闂悎閫熷害閲囩敤瑙嗙嚎鏂瑰悜涓婄殑鐩稿閫熷害鎶曞奖锛歝losing = -v_rel路LOS銆?
    // 瑙ｉ噴锛氳繖涓€琛屽埄鐢ㄥ悜閲忕偣涔樼粨鏋滄洿鏂?`const float ClosingSpeed`锛屾彁鍙栨煇涓€鏂瑰悜涓婄殑鎶曞奖閲忋€?
    const float ClosingSpeed = -FVector::DotProduct(RelativeVel, LOS);

    // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`CommandVelocity`锛岀敤浜庝繚瀛樺懡浠elocity銆?
    FVector CommandVelocity = FVector::ZeroVector;
    // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`bCaptured`锛岀敤浜庝繚瀛樺竷灏旀爣蹇?captured銆?
    const bool bCaptured = Distance <= EffectiveCaptureRadius;
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `bool bValidCmd`锛屽畬鎴?boolBvalidcmd 鐨勬洿鏂般€?
    bool bValidCmd = !RelativePos.IsNearlyZero();

    // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
    if (bCaptured)
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
        if (bStopOnCapture)
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        {
            // 瑙ｉ噴锛氳皟鐢?`Hover` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
            InterceptorDrone->Hover();
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        }
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }
    // 瑙ｉ噴锛氳繖涓€琛岀粰鍑哄厹搴曞垎鏀紝褰撲笂闈㈢殑鏉′欢閮戒笉婊¤冻鏃舵墽琛屻€?
    else
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
        if (EffectiveMethod == TEXT("proportional_nav"))
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        {
            // 瑙ｉ噴锛氳繖涓€琛屽埄鐢ㄥ悜閲忕偣涔樼粨鏋滄洿鏂?`const FVector LateralRelVel`锛屾彁鍙栨煇涓€鏂瑰悜涓婄殑鎶曞奖閲忋€?
            const FVector LateralRelVel = RelativeVel - FVector::DotProduct(RelativeVel, LOS) * LOS;
            // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `CommandVelocity`锛屽畬鎴?鍛戒护velocity 鐨勬洿鏂般€?
            CommandVelocity = TargetVel + EffectiveNavGain * LateralRelVel;
            // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
            if (CommandVelocity.IsNearlyZero())
            // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
            {
                // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `CommandVelocity`锛屽畬鎴?鍛戒护velocity 鐨勬洿鏂般€?
                CommandVelocity = LOS * EffectiveSpeed;
            // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
            }
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        }
        // 瑙ｉ噴锛氳繖涓€琛岀粰鍑哄厹搴曞垎鏀紝褰撲笂闈㈢殑鏉′欢閮戒笉婊¤冻鏃舵墽琛屻€?
        else
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        {
            // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`PredictedTarget`锛岀敤浜庝繚瀛榩redictedtarget銆?
            const FVector PredictedTarget = TargetPos + TargetVel * EffectiveLeadTime;
            // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `CommandVelocity`锛屽畬鎴?鍛戒护velocity 鐨勬洿鏂般€?
            CommandVelocity = (PredictedTarget - InterceptorPos).GetSafeNormal() * EffectiveSpeed;
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        }

        // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
        if (!CommandVelocity.IsNearlyZero())
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        {
            // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `CommandVelocity`锛屽畬鎴?鍛戒护velocity 鐨勬洿鏂般€?
            CommandVelocity = CommandVelocity.GetClampedToMaxSize(EffectiveSpeed);
            // 瑙ｉ噴锛氳皟鐢?`SetHeadingControl` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
            InterceptorDrone->SetHeadingControl(EDroneYawMode::Auto, EDroneDrivetrainMode::ForwardOnly);
            // 瑙ｉ噴锛氳皟鐢?`SetTargetVelocity` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
            InterceptorDrone->SetTargetVelocity(CommandVelocity);
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        }
        // 瑙ｉ噴锛氳繖涓€琛岀粰鍑哄厹搴曞垎鏀紝褰撲笂闈㈢殑鏉′欢閮戒笉婊¤冻鏃舵墽琛屻€?
        else
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        {
            // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `bValidCmd`锛屽畬鎴?甯冨皵鏍囧織 validcmd 鐨勬洿鏂般€?
            bValidCmd = false;
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        }
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }

    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `LastInterceptorId`锛屽畬鎴?lastinterceptorid 鐨勬洿鏂般€?
    LastInterceptorId = InterceptorDrone->DroneId;
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `LastTargetId`锛屽畬鎴?lasttargetid 鐨勬洿鏂般€?
    LastTargetId = TargetDrone->DroneId;
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `LastDistanceToTarget`锛屽畬鎴?lastdistancetotarget 鐨勬洿鏂般€?
    LastDistanceToTarget = Distance;
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `LastClosingSpeed`锛屽畬鎴?lastclosingspeed 鐨勬洿鏂般€?
    LastClosingSpeed = ClosingSpeed;
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `LastInterceptorCmdVel`锛屽畬鎴?lastinterceptorcmdvel 鐨勬洿鏂般€?
    LastInterceptorCmdVel = CommandVelocity;
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `bLastInterceptValid`锛屽畬鎴?甯冨皵鏍囧織 last鎷︽埅valid 鐨勬洿鏂般€?
    bLastInterceptValid = bValidCmd;
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `bLastCaptured`锛屽畬鎴?甯冨皵鏍囧織 lastcaptured 鐨勬洿鏂般€?
    bLastCaptured = bCaptured;

    // 瑙ｉ噴锛氳繖涓€琛岃繑鍥炲綋鍓嶅嚱鏁扮殑璁＄畻缁撴灉锛屾妸鎺у埗鏉冧氦鍥炶皟鐢ㄦ柟銆?
    return FString::Printf(
        // 瑙ｉ噴锛氳繖涓€琛屼綅浜庢瀯閫犲嚱鏁板垵濮嬪寲鍒楄〃涓紝鎶?`TEXT` 鐩存帴鍒濆鍖栦负 `"{\"status\":\"ok\",\"mode\":\"auto_intercept\",\"method\":\"%s\",\"target_id\":\"%s\",\"interceptor_id\":\"%s\",\"distance\":%.3f,\"closing_speed\":%.3f,\"captured\":%s,\"valid\":%s,\"cmd_velocity\":[%.3f,%.3f,%.3f]}"`锛屽噺灏戣繘鍏ュ嚱鏁颁綋鍚庣殑棰濆璧嬪€煎紑閿€銆?
        TEXT("{\"status\":\"ok\",\"mode\":\"auto_intercept\",\"method\":\"%s\",\"target_id\":\"%s\",\"interceptor_id\":\"%s\",\"distance\":%.3f,\"closing_speed\":%.3f,\"captured\":%s,\"valid\":%s,\"cmd_velocity\":[%.3f,%.3f,%.3f]}"),
        *EffectiveMethod,
        *LastTargetId,
        *LastInterceptorId,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        LastDistanceToTarget,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        LastClosingSpeed,
        *BoolLiteral(bCaptured),
        *BoolLiteral(bValidCmd),
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        LastInterceptorCmdVel.X,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        LastInterceptorCmdVel.Y,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        LastInterceptorCmdVel.Z);
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
}

/**
 * @brief 鍚?Kalman 棰勬祴鍣ㄨ緭鍏ヤ竴甯х洰鏍囦綅缃娴?
 * @param X 瑙傛祴浣嶇疆 X
 * @param Y 瑙傛祴浣嶇疆 Y
 * @param Z 瑙傛祴浣嶇疆 Z
 * @param Dt 閲囨牱鏃堕棿闂撮殧锛坰锛?
 * @return 褰撳墠浼拌鐘舵€?JSON
 */
// 瑙ｉ噴锛氳繖涓€琛屽畾涔夊嚱鏁?`UpdateTarget`锛屽紑濮嬪疄鐜皍pdatetarget鐨勫叿浣撻€昏緫銆?
FString AGuidanceActor::UpdateTarget(float X, float Y, float Z, float Dt)
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
{
    // 瑙ｉ噴锛氳皟鐢?`EnsureInitialized` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    EnsureInitialized();

    // 瑙ｉ噴锛氳皟鐢?`Update` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    Predictor->Update(FVector(X, Y, Z), Dt);
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `const FVector EstPos`锛屽畬鎴?constfvectorestpos 鐨勬洿鏂般€?
    const FVector EstPos = Predictor->GetEstimatedPosition();
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `const FVector EstVel`锛屽畬鎴?constfvectorestvel 鐨勬洿鏂般€?
    const FVector EstVel = Predictor->GetEstimatedVelocity();
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `const FVector EstAcc`锛屽畬鎴?constfvectorestacc 鐨勬洿鏂般€?
    const FVector EstAcc = Predictor->GetEstimatedAcceleration();
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `const float AdaptiveQ`锛屽畬鎴?constfloatadaptiveQ 鐨勬洿鏂般€?
    const float AdaptiveQ = Predictor->GetAdaptiveProcessNoise();

    // 瑙ｉ噴锛氳繖涓€琛岃繑鍥炲綋鍓嶅嚱鏁扮殑璁＄畻缁撴灉锛屾妸鎺у埗鏉冧氦鍥炶皟鐢ㄦ柟銆?
    return FString::Printf(
        // 瑙ｉ噴锛氳繖涓€琛屼綅浜庢瀯閫犲嚱鏁板垵濮嬪寲鍒楄〃涓紝鎶?`TEXT` 鐩存帴鍒濆鍖栦负 `"{\"status\":\"ok\",\"est_pos\":[%.2f,%.2f,%.2f],\"est_vel\":[%.2f,%.2f,%.2f],\"est_acc\":[%.2f,%.2f,%.2f],\"adaptive_q\":%.4f}"`锛屽噺灏戣繘鍏ュ嚱鏁颁綋鍚庣殑棰濆璧嬪€煎紑閿€銆?
        TEXT("{\"status\":\"ok\",\"est_pos\":[%.2f,%.2f,%.2f],\"est_vel\":[%.2f,%.2f,%.2f],\"est_acc\":[%.2f,%.2f,%.2f],\"adaptive_q\":%.4f}"),
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        EstPos.X,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        EstPos.Y,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        EstPos.Z,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        EstVel.X,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        EstVel.Y,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        EstVel.Z,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        EstAcc.X,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        EstAcc.Y,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        EstAcc.Z,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        AdaptiveQ);
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
}

/**
 * @brief 鍩轰簬褰撳墠鐩爣棰勬祴鐘舵€佽绠楃偖濉旂瀯鍑嗚В
 * @param TurretId 鐐 ID
 * @param MuzzleSpeed 寮逛父鍒濋€熷害锛坢/s锛?
 * @return 鐬勫噯瑙掍笌棰勮椋炶鏃堕棿 JSON
 */
// 瑙ｉ噴锛氳繖涓€琛屽畾涔夊嚱鏁?`ComputeAim`锛屽紑濮嬪疄鐜癱omputeaim鐨勫叿浣撻€昏緫銆?
FString AGuidanceActor::ComputeAim(FString TurretId, float MuzzleSpeed)
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
{
    // 瑙ｉ噴锛氳皟鐢?`EnsureInitialized` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    EnsureInitialized();

    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `UAgentManager* Manager`锛屽畬鎴?uagent绠＄悊鍣ㄧ鐞嗗櫒 鐨勬洿鏂般€?
    UAgentManager* Manager = UAgentManager::GetInstance();
    // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
    if (!Manager)
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛岃繑鍥炲綋鍓嶅嚱鏁扮殑璁＄畻缁撴灉锛屾妸鎺у埗鏉冧氦鍥炶皟鐢ㄦ柟銆?
        return MakeError(TEXT("Agent manager unavailable"));
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }

    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `const FString EffectiveTurretId`锛屽畬鎴?constfstringeffectiveturretid 鐨勬洿鏂般€?
    const FString EffectiveTurretId = TurretId.IsEmpty() ? TEXT("turret_0") : TurretId;
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `ATurretPawn* Turret`锛屽畬鎴?aturretPawnturret 鐨勬洿鏂般€?
    ATurretPawn* Turret = Cast<ATurretPawn>(Manager->GetAgent(EffectiveTurretId));
    // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
    if (!Turret)
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛岃繑鍥炲綋鍓嶅嚱鏁扮殑璁＄畻缁撴灉锛屾妸鎺у埗鏉冧氦鍥炶皟鐢ㄦ柟銆?
        return MakeError(FString::Printf(TEXT("Turret '%s' not found"), *EffectiveTurretId));
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }

    // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`Input`锛岀敤浜庝繚瀛榠nput銆?
    FGuidanceInput Input;
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `Input.TurretPos`锛屽畬鎴?turretpos 鐨勬洿鏂般€?
    Input.TurretPos = Turret->GetActorLocation();
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `Input.MuzzlePos`锛屽畬鎴?muzzlepos 鐨勬洿鏂般€?
    Input.MuzzlePos = GetTurretMuzzlePosition(Turret);
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `Input.TargetPos`锛屽畬鎴?鐩爣浣嶇疆 鐨勬洿鏂般€?
    Input.TargetPos = Predictor->GetEstimatedPosition();
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `Input.TargetVel`锛屽畬鎴?鐩爣閫熷害 鐨勬洿鏂般€?
    Input.TargetVel = Predictor->GetEstimatedVelocity();
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `Input.PredictedPos`锛屽畬鎴?predictedpos 鐨勬洿鏂般€?
    Input.PredictedPos = Predictor->PredictPosition(0.5f);
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `Input.MuzzleSpeed`锛屽畬鎴?muzzlespeed 鐨勬洿鏂般€?
    Input.MuzzleSpeed = MuzzleSpeed;
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `Input.DeltaTime`锛屽畬鎴?鏃堕棿姝ラ暱 鐨勬洿鏂般€?
    Input.DeltaTime = 0.1f;

    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `const FGuidanceOutput Output`锛屽畬鎴?constfguidanceoutputoutput 鐨勬洿鏂般€?
    const FGuidanceOutput Output = CurrentMethod->ComputeAim(Input);
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `LastPitch`锛屽畬鎴?lastpitch 鐨勬洿鏂般€?
    LastPitch = Output.Pitch;
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `LastYaw`锛屽畬鎴?lastyaw 鐨勬洿鏂般€?
    LastYaw = Output.Yaw;
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `LastAimPoint`锛屽畬鎴?lastaimpoint 鐨勬洿鏂般€?
    LastAimPoint = Output.AimPoint;
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `LastFlightTime`锛屽畬鎴?lastflighttime 鐨勬洿鏂般€?
    LastFlightTime = Output.EstFlightTime;

    // 瑙ｉ噴锛氳繖涓€琛岃繑鍥炲綋鍓嶅嚱鏁扮殑璁＄畻缁撴灉锛屾妸鎺у埗鏉冧氦鍥炶皟鐢ㄦ柟銆?
    return FString::Printf(
        // 瑙ｉ噴锛氳繖涓€琛屼綅浜庢瀯閫犲嚱鏁板垵濮嬪寲鍒楄〃涓紝鎶?`TEXT` 鐩存帴鍒濆鍖栦负 `"{\"status\":\"ok\",\"pitch\":%.2f,\"yaw\":%.2f,\"aim_point\":[%.1f,%.1f,%.1f],\"flight_time\":%.4f,\"method\":\"%s\"}"`锛屽噺灏戣繘鍏ュ嚱鏁颁綋鍚庣殑棰濆璧嬪€煎紑閿€銆?
        TEXT("{\"status\":\"ok\",\"pitch\":%.2f,\"yaw\":%.2f,\"aim_point\":[%.1f,%.1f,%.1f],\"flight_time\":%.4f,\"method\":\"%s\"}"),
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        Output.Pitch,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        Output.Yaw,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        Output.AimPoint.X,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        Output.AimPoint.Y,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        Output.AimPoint.Z,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        Output.EstFlightTime,
        *CurrentMethodName);
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
}

/**
 * @brief 鑷姩瀹屾垚涓€娆♀€滆娴?-> 棰勬祴 -> 鐬勫噯 -> 鍙€夊紑鐏€濇祦绋?
 * @param TurretId 鐐 ID
 * @param TargetId 鐩爣 ID
 * @param MuzzleSpeed 寮逛父鍒濋€熷害锛坢/s锛?
 * @param Dt 瑙傛祴鍛ㄦ湡锛坰锛?
 * @param Latency 寤惰繜琛ュ伩鏃堕棿锛坰锛?
 * @param bFire 鏄惁瑙﹀彂寮€鐏?
 * @return 鏈鑷姩浜ゆ垬缁撴灉 JSON
 *
 * 寤惰繜琛ュ伩鐨勬牳蹇冩€濇兂涓猴細
 * $p_{comp} = \hat{p}(t + latency)$锛?
 * 鍗冲厛鐢ㄩ娴嬪櫒鎶婄洰鏍囩姸鎬佸鎺ㄥ埌灏勫嚮鐢熸晥鏃跺埢锛屽啀浜ょ敱鍒跺绠楁硶姹傝В鐬勫噯瑙掋€?
 */
// 瑙ｉ噴锛氳繖涓€琛屽畾涔夊嚱鏁?`AutoEngage`锛屽紑濮嬪疄鐜癮utoengage鐨勫叿浣撻€昏緫銆?
FString AGuidanceActor::AutoEngage(FString TurretId, FString TargetId, float MuzzleSpeed, float Dt, float Latency, bool bFire)
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
{
    // 瑙ｉ噴锛氳皟鐢?`EnsureInitialized` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    EnsureInitialized();

    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `UAgentManager* Manager`锛屽畬鎴?uagent绠＄悊鍣ㄧ鐞嗗櫒 鐨勬洿鏂般€?
    UAgentManager* Manager = UAgentManager::GetInstance();
    // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
    if (!Manager)
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛岃繑鍥炲綋鍓嶅嚱鏁扮殑璁＄畻缁撴灉锛屾妸鎺у埗鏉冧氦鍥炶皟鐢ㄦ柟銆?
        return MakeError(TEXT("Agent manager unavailable"));
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }

    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `const FString EffectiveTargetId`锛屽畬鎴?constfstringeffectivetargetid 鐨勬洿鏂般€?
    const FString EffectiveTargetId = TargetId.IsEmpty() ? TEXT("drone_0") : TargetId;
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `const FString EffectiveTurretId`锛屽畬鎴?constfstringeffectiveturretid 鐨勬洿鏂般€?
    const FString EffectiveTurretId = TurretId.IsEmpty() ? TEXT("turret_0") : TurretId;
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `AActor* Target`锛屽畬鎴?aactortarget 鐨勬洿鏂般€?
    AActor* Target = Manager->GetAgent(EffectiveTargetId);
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `ATurretPawn* Turret`锛屽畬鎴?aturretPawnturret 鐨勬洿鏂般€?
    ATurretPawn* Turret = Cast<ATurretPawn>(Manager->GetAgent(EffectiveTurretId));
    // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
    if (!Target)
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛岃繑鍥炲綋鍓嶅嚱鏁扮殑璁＄畻缁撴灉锛屾妸鎺у埗鏉冧氦鍥炶皟鐢ㄦ柟銆?
        return MakeError(FString::Printf(TEXT("Target '%s' not found"), *EffectiveTargetId));
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }
    // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
    if (!Turret)
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛岃繑鍥炲綋鍓嶅嚱鏁扮殑璁＄畻缁撴灉锛屾妸鎺у埗鏉冧氦鍥炶皟鐢ㄦ柟銆?
        return MakeError(FString::Printf(TEXT("Turret '%s' not found"), *EffectiveTurretId));
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }

    // 瑙ｉ噴锛氳繖涓€琛屽厛瀵硅绠楃粨鏋滃仛闄愬箙锛屽啀鍐欏叆 `const float EffectiveLatency`锛岄槻姝?constfloateffectivelatency 瓒呭嚭鍏佽鑼冨洿銆?
    const float EffectiveLatency = (Latency >= 0.0f) ? FMath::Clamp(Latency, 0.0f, 1.0f) : DefaultVisionLatency;
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `LastLatencyCompensation`锛屽畬鎴?lastlatencycompensation 鐨勬洿鏂般€?
    LastLatencyCompensation = EffectiveLatency;

    // 瑙ｉ噴锛氳皟鐢?`Update` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    Predictor->Update(Target->GetActorLocation(), Dt);
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `const FVector CompensatedTargetPos`锛屽畬鎴?constfvectorcompensatedtargetpos 鐨勬洿鏂般€?
    const FVector CompensatedTargetPos = Predictor->PredictPosition(EffectiveLatency);

    // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`Input`锛岀敤浜庝繚瀛榠nput銆?
    FGuidanceInput Input;
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `Input.TurretPos`锛屽畬鎴?turretpos 鐨勬洿鏂般€?
    Input.TurretPos = Turret->GetActorLocation();
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `Input.MuzzlePos`锛屽畬鎴?muzzlepos 鐨勬洿鏂般€?
    Input.MuzzlePos = GetTurretMuzzlePosition(Turret);
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `Input.TargetPos`锛屽畬鎴?鐩爣浣嶇疆 鐨勬洿鏂般€?
    Input.TargetPos = CompensatedTargetPos;
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `Input.TargetVel`锛屽畬鎴?鐩爣閫熷害 鐨勬洿鏂般€?
    Input.TargetVel = Predictor->GetEstimatedVelocity();
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `Input.PredictedPos`锛屽畬鎴?predictedpos 鐨勬洿鏂般€?
    Input.PredictedPos = Predictor->PredictPosition(EffectiveLatency + 0.5f);
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `Input.MuzzleSpeed`锛屽畬鎴?muzzlespeed 鐨勬洿鏂般€?
    Input.MuzzleSpeed = MuzzleSpeed;
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `Input.DeltaTime`锛屽畬鎴?鏃堕棿姝ラ暱 鐨勬洿鏂般€?
    Input.DeltaTime = Dt;

    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `const FGuidanceOutput Output`锛屽畬鎴?constfguidanceoutputoutput 鐨勬洿鏂般€?
    const FGuidanceOutput Output = CurrentMethod->ComputeAim(Input);
    // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
    if (Output.bValid)
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳皟鐢?`SetTargetAngles` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
        Turret->SetTargetAngles(Output.Pitch, Output.Yaw);
        // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `LastPitch`锛屽畬鎴?lastpitch 鐨勬洿鏂般€?
        LastPitch = Output.Pitch;
        // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `LastYaw`锛屽畬鎴?lastyaw 鐨勬洿鏂般€?
        LastYaw = Output.Yaw;
        // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `LastAimPoint`锛屽畬鎴?lastaimpoint 鐨勬洿鏂般€?
        LastAimPoint = Output.AimPoint;
        // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `LastFlightTime`锛屽畬鎴?lastflighttime 鐨勬洿鏂般€?
        LastFlightTime = Output.EstFlightTime;
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }

    // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
    if (bFire)
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳皟鐢?`FireX` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
        Turret->FireX(MuzzleSpeed);
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }

    // 瑙ｉ噴锛氳繖涓€琛岃繑鍥炲綋鍓嶅嚱鏁扮殑璁＄畻缁撴灉锛屾妸鎺у埗鏉冧氦鍥炶皟鐢ㄦ柟銆?
    return FString::Printf(
        // 瑙ｉ噴锛氳繖涓€琛屼綅浜庢瀯閫犲嚱鏁板垵濮嬪寲鍒楄〃涓紝鎶?`TEXT` 鐩存帴鍒濆鍖栦负 `"{\"status\":\"ok\",\"pitch\":%.2f,\"yaw\":%.2f,\"fired\":%s,\"flight_time\":%.4f,\"latency\":%.4f,\"method\":\"%s\"}"`锛屽噺灏戣繘鍏ュ嚱鏁颁綋鍚庣殑棰濆璧嬪€煎紑閿€銆?
        TEXT("{\"status\":\"ok\",\"pitch\":%.2f,\"yaw\":%.2f,\"fired\":%s,\"flight_time\":%.4f,\"latency\":%.4f,\"method\":\"%s\"}"),
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        Output.Pitch,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        Output.Yaw,
        *BoolLiteral(bFire),
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        Output.EstFlightTime,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        LastLatencyCompensation,
        *CurrentMethodName);
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
}

/**
 * @brief 閲嶆柊璁剧疆 Kalman 棰勬祴鍣ㄥ櫔澹板弬鏁?
 * @param ProcessNoise 杩囩▼鍣０
 * @param MeasurementNoise 瑙傛祴鍣０
 */
// 瑙ｉ噴锛氳繖涓€琛屽畾涔夊嚱鏁?`SetKalmanParams`锛屽紑濮嬪疄鐜皊et鍗″皵鏇紁arams鐨勫叿浣撻€昏緫銆?
FString AGuidanceActor::SetKalmanParams(float ProcessNoise, float MeasurementNoise)
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
{
    // 瑙ｉ噴锛氳皟鐢?`EnsureInitialized` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    EnsureInitialized();
    // 瑙ｉ噴锛氳皟鐢?`Initialize` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    Predictor->Initialize(ProcessNoise, MeasurementNoise);
    // 瑙ｉ噴锛氳繖涓€琛岃繑鍥炲綋鍓嶅嚱鏁扮殑璁＄畻缁撴灉锛屾妸鎺у埗鏉冧氦鍥炶皟鐢ㄦ柟銆?
    return MakeOk(FString::Printf(TEXT("kalman Q=%.2f R=%.2f"), ProcessNoise, MeasurementNoise));
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
}

/**
 * @brief 閲嶇疆 GuidanceActor 鐨勫唴閮ㄨ繍琛岀姸鎬?
 *
 * 璇ユ帴鍙ｄ細娓呯┖锛?
 * - Kalman 棰勬祴鍣ㄥ巻鍙茬姸鎬侊紱
 * - 褰撳墠鍒跺绠楁硶鐨勫唴閮ㄨ蹇嗛噺锛?
 * - 鏈€杩戜竴娆＄瀯鍑?鎷︽埅缁撴灉缂撳瓨锛?
 * - 瑙嗚鎷︽埅鎺у埗鍣ㄨ繍琛岀姸鎬併€?
 */
// 瑙ｉ噴锛氳繖涓€琛屽畾涔夊嚱鏁?`ResetGuidance`锛屽紑濮嬪疄鐜皉eset鍒跺鐨勫叿浣撻€昏緫銆?
FString AGuidanceActor::ResetGuidance()
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
{
    // 瑙ｉ噴锛氳皟鐢?`EnsureInitialized` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    EnsureInitialized();

    // 瑙ｉ噴锛氳皟鐢?`Reset` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    Predictor->Reset();
    // 瑙ｉ噴锛氳皟鐢?`Initialize` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    Predictor->Initialize(100.0f, 0.01f);

    // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
    if (CurrentMethod)
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳皟鐢?`Reset` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
        CurrentMethod->Reset();
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }

    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `LastPitch`锛屽畬鎴?lastpitch 鐨勬洿鏂般€?
    LastPitch = 0.0f;
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `LastYaw`锛屽畬鎴?lastyaw 鐨勬洿鏂般€?
    LastYaw = 0.0f;
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `LastAimPoint`锛屽畬鎴?lastaimpoint 鐨勬洿鏂般€?
    LastAimPoint = FVector::ZeroVector;
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `LastFlightTime`锛屽畬鎴?lastflighttime 鐨勬洿鏂般€?
    LastFlightTime = 0.0f;
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `LastLatencyCompensation`锛屽畬鎴?lastlatencycompensation 鐨勬洿鏂般€?
    LastLatencyCompensation = DefaultVisionLatency;

    // 瑙ｉ噴锛氳皟鐢?`Empty` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    LastInterceptorId.Empty();
    // 瑙ｉ噴锛氳皟鐢?`Empty` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    LastTargetId.Empty();
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `LastInterceptorCmdVel`锛屽畬鎴?lastinterceptorcmdvel 鐨勬洿鏂般€?
    LastInterceptorCmdVel = FVector::ZeroVector;
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `LastDistanceToTarget`锛屽畬鎴?lastdistancetotarget 鐨勬洿鏂般€?
    LastDistanceToTarget = 0.0f;
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `LastClosingSpeed`锛屽畬鎴?lastclosingspeed 鐨勬洿鏂般€?
    LastClosingSpeed = 0.0f;
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `bLastInterceptValid`锛屽畬鎴?甯冨皵鏍囧織 last鎷︽埅valid 鐨勬洿鏂般€?
    bLastInterceptValid = false;
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `bLastCaptured`锛屽畬鎴?甯冨皵鏍囧織 lastcaptured 鐨勬洿鏂般€?
    bLastCaptured = false;

    // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
    if (VisualInterceptController)
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳皟鐢?`Reset` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
        VisualInterceptController->Reset();
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }

    // 瑙ｉ噴锛氳繖涓€琛岃繑鍥炲綋鍓嶅嚱鏁扮殑璁＄畻缁撴灉锛屾妸鎺у埗鏉冧氦鍥炶皟鐢ㄦ柟銆?
    return MakeOk(TEXT("guidance reset"));
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
}

/**
 * @brief 姹囨€诲綋鍓嶅埗瀵笺€侀娴嬩笌鎷︽埅鐘舵€?
 * @return 瀹屾暣鐘舵€?JSON
 */
// 瑙ｉ噴锛氳繖涓€琛屽畾涔夊嚱鏁?`GetState`锛屽紑濮嬪疄鐜癵et鐘舵€佺殑鍏蜂綋閫昏緫銆?
FString AGuidanceActor::GetState()
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
{
    // 瑙ｉ噴锛氳皟鐢?`EnsureInitialized` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    EnsureInitialized();

    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `const FVector EstPos`锛屽畬鎴?constfvectorestpos 鐨勬洿鏂般€?
    const FVector EstPos = Predictor->GetEstimatedPosition();
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `const FVector EstVel`锛屽畬鎴?constfvectorestvel 鐨勬洿鏂般€?
    const FVector EstVel = Predictor->GetEstimatedVelocity();
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `const FVector EstAcc`锛屽畬鎴?constfvectorestacc 鐨勬洿鏂般€?
    const FVector EstAcc = Predictor->GetEstimatedAcceleration();
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `const float Uncertainty`锛屽畬鎴?constfloatuncertainty 鐨勬洿鏂般€?
    const float Uncertainty = Predictor->GetPositionUncertainty();
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `const float AdaptiveQ`锛屽畬鎴?constfloatadaptiveQ 鐨勬洿鏂般€?
    const float AdaptiveQ = Predictor->GetAdaptiveProcessNoise();

    // 瑙ｉ噴锛氳繖涓€琛岃繑鍥炲綋鍓嶅嚱鏁扮殑璁＄畻缁撴灉锛屾妸鎺у埗鏉冧氦鍥炶皟鐢ㄦ柟銆?
    return FString::Printf(
        // 瑙ｉ噴锛氳繖涓€琛屼綅浜庢瀯閫犲嚱鏁板垵濮嬪寲鍒楄〃涓紝鎶?`TEXT` 鐩存帴鍒濆鍖栦负 `"{\"status\":\"ok\",\"id\":\"%s\",\"method\":\"%s\",\"initialized\":%s,\"est_pos\":[%.2f,%.2f,%.2f],\"est_vel\":[%.2f,%.2f,%.2f],\"est_acc\":[%.2f,%.2f,%.2f],\"uncertainty\":%.2f,\"adaptive_q\":%.4f,\"aim_pitch\":%.2f,\"aim_yaw\":%.2f,\"aim_point\":[%.1f,%.1f,%.1f],\"flight_time\":%.4f,\"latency\":%.4f,\"intercept\":{\"method\":\"%s\",\"target_id\":\"%s\",\"interceptor_id\":\"%s\",\"distance\":%.3f,\"closing_speed\":%.3f,\"captured\":%s,\"valid\":%s,\"cmd_velocity\":[%.3f,%.3f,%.3f]}}"`锛屽噺灏戣繘鍏ュ嚱鏁颁綋鍚庣殑棰濆璧嬪€煎紑閿€銆?
        TEXT("{\"status\":\"ok\",\"id\":\"%s\",\"method\":\"%s\",\"initialized\":%s,\"est_pos\":[%.2f,%.2f,%.2f],\"est_vel\":[%.2f,%.2f,%.2f],\"est_acc\":[%.2f,%.2f,%.2f],\"uncertainty\":%.2f,\"adaptive_q\":%.4f,\"aim_pitch\":%.2f,\"aim_yaw\":%.2f,\"aim_point\":[%.1f,%.1f,%.1f],\"flight_time\":%.4f,\"latency\":%.4f,\"intercept\":{\"method\":\"%s\",\"target_id\":\"%s\",\"interceptor_id\":\"%s\",\"distance\":%.3f,\"closing_speed\":%.3f,\"captured\":%s,\"valid\":%s,\"cmd_velocity\":[%.3f,%.3f,%.3f]}}"),
        *GuidanceId.ReplaceCharWithEscapedChar(),
        *CurrentMethodName,
        *BoolLiteral(Predictor->IsInitialized()),
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        EstPos.X,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        EstPos.Y,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        EstPos.Z,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        EstVel.X,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        EstVel.Y,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        EstVel.Z,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        EstAcc.X,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        EstAcc.Y,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        EstAcc.Z,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        Uncertainty,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        AdaptiveQ,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        LastPitch,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        LastYaw,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        LastAimPoint.X,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        LastAimPoint.Y,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        LastAimPoint.Z,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        LastFlightTime,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        LastLatencyCompensation,
        *CurrentInterceptMethod,
        *LastTargetId.ReplaceCharWithEscapedChar(),
        *LastInterceptorId.ReplaceCharWithEscapedChar(),
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        LastDistanceToTarget,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        LastClosingSpeed,
        *BoolLiteral(bLastCaptured),
        *BoolLiteral(bLastInterceptValid),
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        LastInterceptorCmdVel.X,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        LastInterceptorCmdVel.Y,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        LastInterceptorCmdVel.Z);
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
}

/**
 * @brief 灏?Blueprint/TCP 鍙傛暟鎵撳寘鎴愬惎鍔ㄥ懡浠ゅ苟杞彂缁欒瑙夋嫤鎴帶鍒跺櫒
 * @return 瑙嗚鎷︽埅鍚姩缁撴灉 JSON
 */
// 瑙ｉ噴锛氳繖涓€琛屽畾涔夊嚱鏁?`VisualInterceptStart`锛屽紑濮嬪疄鐜拌瑙夋嫤鎴猻tart鐨勫叿浣撻€昏緫銆?
FString AGuidanceActor::VisualInterceptStart(
    // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `VisualInterceptStart` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `InterceptorId` 鐢ㄤ簬浼犲叆interceptorid銆?
    FString InterceptorId,
    // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `VisualInterceptStart` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `TargetId` 鐢ㄤ簬浼犲叆targetid銆?
    FString TargetId,
    // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `VisualInterceptStart` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `Method` 鐢ㄤ簬浼犲叆method銆?
    FString Method,
    // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `VisualInterceptStart` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `DesiredArea` 鐢ㄤ簬浼犲叆desiredarea銆?
    float DesiredArea,
    // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `VisualInterceptStart` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `CaptureArea` 鐢ㄤ簬浼犲叆閲囬泦area銆?
    float CaptureArea,
    // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `VisualInterceptStart` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `CenterTolX` 鐢ㄤ簬浼犲叆centertolX銆?
    float CenterTolX,
    // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `VisualInterceptStart` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `CenterTolY` 鐢ㄤ簬浼犲叆centertolY銆?
    float CenterTolY,
    // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `VisualInterceptStart` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `CaptureHoldFrames` 鐢ㄤ簬浼犲叆閲囬泦holdframes銆?
    int32 CaptureHoldFrames,
    // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `VisualInterceptStart` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `LostToSearchFrames` 鐢ㄤ簬浼犲叆losttosearchframes銆?
    int32 LostToSearchFrames,
    // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `VisualInterceptStart` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `MaxForwardSpeed` 鐢ㄤ簬浼犲叆maxforwardspeed銆?
    float MaxForwardSpeed,
    // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `VisualInterceptStart` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `MaxReverseSpeed` 鐢ㄤ簬浼犲叆maxreversespeed銆?
    float MaxReverseSpeed,
    // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `VisualInterceptStart` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `MaxVerticalSpeed` 鐢ㄤ簬浼犲叆maxverticalspeed銆?
    float MaxVerticalSpeed,
    // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `VisualInterceptStart` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `MaxYawRateDeg` 鐢ㄤ簬浼犲叆maxyawratedeg銆?
    float MaxYawRateDeg,
    float RamAreaTarget,
    float MinRamSpeed,
    float InterceptDistance,
    float TrackLeadTime,
    float RamLeadTime,
    // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `VisualInterceptStart` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `SearchCamYawLimitDeg` 鐢ㄤ簬浼犲叆searchcamyawlimitdeg銆?
    float SearchCamYawLimitDeg,
    // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `VisualInterceptStart` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `SearchCamRateDeg` 鐢ㄤ簬浼犲叆searchcamratedeg銆?
    float SearchCamRateDeg,
    // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `VisualInterceptStart` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `SearchBodyYawRateDeg` 鐢ㄤ簬浼犲叆searchbodyyawratedeg銆?
    float SearchBodyYawRateDeg,
    // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `VisualInterceptStart` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `SearchCamPitchDeg` 鐢ㄤ簬浼犲叆searchcampitchdeg銆?
    float SearchCamPitchDeg,
    // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `VisualInterceptStart` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `SearchVzAmp` 鐢ㄤ簬浼犲叆searchvzamp銆?
    float SearchVzAmp,
    // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `VisualInterceptStart` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `StopOnCaptureFlag` 鐢ㄤ簬浼犲叆stopon閲囬泦flag銆?
    int32 StopOnCaptureFlag,
    // 瑙ｉ噴锛氳繖涓€琛屾敹鏉熷嚱鏁?`VisualInterceptStart` 鐨勭鍚嶏紝鍚庨潰浼氳繘鍏ュ疄鐜颁綋鎴栦互鍒嗗彿缁撴潫澹版槑銆?
    int32 UseKalmanFlag)
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
{
    // 瑙ｉ噴锛氳皟鐢?`EnsureInitialized` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    EnsureInitialized();

    // 瑙ｉ噴锛氳皟鐢?`MakeShareable` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    const TSharedPtr<FJsonObject> Cmd = MakeShareable(new FJsonObject);
    // 瑙ｉ噴锛氳皟鐢?`SetStringIfNotEmpty` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    SetStringIfNotEmpty(Cmd, TEXT("interceptor_id"), InterceptorId);
    // 瑙ｉ噴锛氳皟鐢?`SetStringIfNotEmpty` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    SetStringIfNotEmpty(Cmd, TEXT("target_id"), TargetId);
    // 瑙ｉ噴锛氳皟鐢?`SetStringIfNotEmpty` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    SetStringIfNotEmpty(Cmd, TEXT("method"), Method);
    // 瑙ｉ噴锛氳皟鐢?`SetNumberIfValid` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    SetNumberIfValid(Cmd, TEXT("desired_area"), DesiredArea);
    // 瑙ｉ噴锛氳皟鐢?`SetNumberIfValid` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    SetNumberIfValid(Cmd, TEXT("capture_area"), CaptureArea);
    // 瑙ｉ噴锛氳皟鐢?`SetNumberIfValid` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    SetNumberIfValid(Cmd, TEXT("center_tol_x"), CenterTolX);
    // 瑙ｉ噴锛氳皟鐢?`SetNumberIfValid` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    SetNumberIfValid(Cmd, TEXT("center_tol_y"), CenterTolY);
    // 瑙ｉ噴锛氳皟鐢?`SetIntIfValid` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    SetIntIfValid(Cmd, TEXT("capture_hold_frames"), CaptureHoldFrames);
    // 瑙ｉ噴锛氳皟鐢?`SetIntIfValid` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    SetIntIfValid(Cmd, TEXT("lost_to_search_frames"), LostToSearchFrames);
    // 瑙ｉ噴锛氳皟鐢?`SetNumberIfValid` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    SetNumberIfValid(Cmd, TEXT("max_forward_speed"), MaxForwardSpeed);
    // 瑙ｉ噴锛氳皟鐢?`SetNumberIfValid` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    SetNumberIfValid(Cmd, TEXT("max_reverse_speed"), MaxReverseSpeed);
    // 瑙ｉ噴锛氳皟鐢?`SetNumberIfValid` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    SetNumberIfValid(Cmd, TEXT("max_vertical_speed"), MaxVerticalSpeed);
    // 瑙ｉ噴锛氳皟鐢?`SetNumberIfValid` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    SetNumberIfValid(Cmd, TEXT("max_yaw_rate_deg"), MaxYawRateDeg);
    SetNumberIfValid(Cmd, TEXT("ram_area_target"), RamAreaTarget);
    SetNumberIfValid(Cmd, TEXT("min_ram_speed"), MinRamSpeed);
    SetNumberIfValid(Cmd, TEXT("intercept_distance"), InterceptDistance);
    SetNumberIfValid(Cmd, TEXT("track_lead_time"), TrackLeadTime);
    SetNumberIfValid(Cmd, TEXT("ram_lead_time"), RamLeadTime);
    // 瑙ｉ噴锛氳皟鐢?`SetNumberIfValid` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    SetNumberIfValid(Cmd, TEXT("search_cam_yaw_limit_deg"), SearchCamYawLimitDeg);
    // 瑙ｉ噴锛氳皟鐢?`SetNumberIfValid` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    SetNumberIfValid(Cmd, TEXT("search_cam_rate_deg"), SearchCamRateDeg);
    // 瑙ｉ噴锛氳皟鐢?`SetNumberIfValid` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    SetNumberIfValid(Cmd, TEXT("search_body_yaw_rate_deg"), SearchBodyYawRateDeg);
    // 瑙ｉ噴锛氳皟鐢?`SetNumberIfValid` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    SetNumberIfValid(Cmd, TEXT("search_vz_amp"), SearchVzAmp);
    // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
    if (!FMath::IsNearlyEqual(SearchCamPitchDeg, -1000.0f))
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳皟鐢?`SetNumberField` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
        Cmd->SetNumberField(TEXT("search_cam_pitch_deg"), SearchCamPitchDeg);
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }
    // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
    if (StopOnCaptureFlag >= 0)
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳皟鐢?`SetBoolField` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
        Cmd->SetBoolField(TEXT("stop_on_capture"), StopOnCaptureFlag != 0);
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }
    // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
    if (UseKalmanFlag >= 0)
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳皟鐢?`SetBoolField` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
        Cmd->SetBoolField(TEXT("use_kalman"), UseKalmanFlag != 0);
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }

    // 瑙ｉ噴锛氳繖涓€琛岃繑鍥炲綋鍓嶅嚱鏁扮殑璁＄畻缁撴灉锛屾妸鎺у埗鏉冧氦鍥炶皟鐢ㄦ柟銆?
    return VisualInterceptController->HandleStart(Cmd, GetWorld());
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
}

/**
 * @brief 鎻愪氦涓€甯ц瑙夋娴嬬粨鏋滅粰瑙嗚鎷︽埅鎺у埗鍣?
 * @return 鏈抚瑙嗚鎷︽埅鐘舵€?JSON
 */
// 瑙ｉ噴锛氳繖涓€琛屽畾涔夊嚱鏁?`VisualInterceptUpdate`锛屽紑濮嬪疄鐜拌瑙夋嫤鎴猽pdate鐨勫叿浣撻€昏緫銆?
FString AGuidanceActor::VisualInterceptUpdate(
    // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `VisualInterceptUpdate` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `HasDetection` 鐢ㄤ簬浼犲叆hasdetection銆?
    int32 HasDetection,
    // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `VisualInterceptUpdate` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `Cx` 鐢ㄤ簬浼犲叆cx銆?
    float Cx,
    // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `VisualInterceptUpdate` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `Cy` 鐢ㄤ簬浼犲叆cy銆?
    float Cy,
    // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `VisualInterceptUpdate` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `Area` 鐢ㄤ簬浼犲叆area銆?
    float Area,
    // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `VisualInterceptUpdate` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `AreaRatio` 鐢ㄤ簬浼犲叆arearatio銆?
    float AreaRatio,
    // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `VisualInterceptUpdate` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `Conf` 鐢ㄤ簬浼犲叆conf銆?
    float Conf,
    // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `VisualInterceptUpdate` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `Dt` 鐢ㄤ簬浼犲叆dt銆?
    float Dt,
    // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `VisualInterceptUpdate` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `ImageW` 鐢ㄤ簬浼犲叆鍥惧儚W銆?
    float ImageW,
    // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `VisualInterceptUpdate` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `ImageH` 鐢ㄤ簬浼犲叆鍥惧儚H銆?
    float ImageH,
    // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `VisualInterceptUpdate` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `InterceptorId` 鐢ㄤ簬浼犲叆interceptorid銆?
    FString InterceptorId,
    // 瑙ｉ噴锛氳繖涓€琛屾敹鏉熷嚱鏁?`VisualInterceptUpdate` 鐨勭鍚嶏紝鍚庨潰浼氳繘鍏ュ疄鐜颁綋鎴栦互鍒嗗彿缁撴潫澹版槑銆?
    FString TargetId)
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
{
    // 瑙ｉ噴锛氳皟鐢?`EnsureInitialized` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    EnsureInitialized();

    // 瑙ｉ噴锛氳皟鐢?`MakeShareable` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    const TSharedPtr<FJsonObject> Cmd = MakeShareable(new FJsonObject);
    // 瑙ｉ噴锛氳皟鐢?`SetBoolField` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    Cmd->SetBoolField(TEXT("has_detection"), HasDetection != 0);
    // 瑙ｉ噴锛氳皟鐢?`SetNumberField` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    Cmd->SetNumberField(TEXT("cx"), Cx);
    // 瑙ｉ噴锛氳皟鐢?`SetNumberField` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    Cmd->SetNumberField(TEXT("cy"), Cy);
    // 瑙ｉ噴锛氳皟鐢?`SetNumberField` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    Cmd->SetNumberField(TEXT("area"), Area);
    // 瑙ｉ噴锛氳皟鐢?`SetNumberField` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    Cmd->SetNumberField(TEXT("area_ratio"), AreaRatio);
    // 瑙ｉ噴锛氳皟鐢?`SetNumberField` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    Cmd->SetNumberField(TEXT("conf"), Conf);
    // 瑙ｉ噴锛氳皟鐢?`SetNumberField` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    Cmd->SetNumberField(TEXT("dt"), Dt);
    // 瑙ｉ噴锛氳皟鐢?`SetNumberField` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    Cmd->SetNumberField(TEXT("image_w"), ImageW);
    // 瑙ｉ噴锛氳皟鐢?`SetNumberField` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    Cmd->SetNumberField(TEXT("image_h"), ImageH);
    // 瑙ｉ噴锛氳皟鐢?`SetStringIfNotEmpty` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    SetStringIfNotEmpty(Cmd, TEXT("interceptor_id"), InterceptorId);
    // 瑙ｉ噴锛氳皟鐢?`SetStringIfNotEmpty` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    SetStringIfNotEmpty(Cmd, TEXT("target_id"), TargetId);

    // 瑙ｉ噴锛氳繖涓€琛岃繑鍥炲綋鍓嶅嚱鏁扮殑璁＄畻缁撴灉锛屾妸鎺у埗鏉冧氦鍥炶皟鐢ㄦ柟銆?
    return VisualInterceptController->HandleUpdate(Cmd, GetWorld());
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
}

/**
 * @brief 鍋滄瑙嗚鎷︽埅浼氳瘽
 * @return 鍋滄缁撴灉 JSON
 */
// 瑙ｉ噴锛氳繖涓€琛屽畾涔夊嚱鏁?`VisualInterceptStop`锛屽紑濮嬪疄鐜拌瑙夋嫤鎴猻top鐨勫叿浣撻€昏緫銆?
FString AGuidanceActor::VisualInterceptStop(FString InterceptorId, FString TargetId)
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
{
    // 瑙ｉ噴锛氳皟鐢?`EnsureInitialized` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    EnsureInitialized();

    // 瑙ｉ噴锛氳皟鐢?`MakeShareable` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    const TSharedPtr<FJsonObject> Cmd = MakeShareable(new FJsonObject);
    // 瑙ｉ噴锛氳皟鐢?`SetStringIfNotEmpty` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    SetStringIfNotEmpty(Cmd, TEXT("interceptor_id"), InterceptorId);
    // 瑙ｉ噴锛氳皟鐢?`SetStringIfNotEmpty` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    SetStringIfNotEmpty(Cmd, TEXT("target_id"), TargetId);
    // 瑙ｉ噴锛氳繖涓€琛岃繑鍥炲綋鍓嶅嚱鏁扮殑璁＄畻缁撴灉锛屾妸鎺у埗鏉冧氦鍥炶皟鐢ㄦ柟銆?
    return VisualInterceptController->HandleStop(Cmd, GetWorld());
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
}

/** @brief 鏌ヨ瑙嗚鎷︽埅鎺у埗鍣ㄥ綋鍓嶇姸鎬?*/
// 瑙ｉ噴锛氳繖涓€琛屽畾涔夊嚱鏁?`VisualInterceptState`锛屽紑濮嬪疄鐜拌瑙夋嫤鎴姸鎬佺殑鍏蜂綋閫昏緫銆?
FString AGuidanceActor::VisualInterceptState()
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
{
    // 瑙ｉ噴锛氳皟鐢?`EnsureInitialized` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    EnsureInitialized();
    // 瑙ｉ噴锛氳繖涓€琛岃繑鍥炲綋鍓嶅嚱鏁扮殑璁＄畻缁撴灉锛屾妸鎺у埗鏉冧氦鍥炶皟鐢ㄦ柟銆?
    return VisualInterceptController->HandleState();
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
}




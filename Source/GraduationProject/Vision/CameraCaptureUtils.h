// 瑙ｉ噴锛氫娇鐢?`#pragma once` 闃叉璇ュご鏂囦欢鍦ㄧ紪璇戣繃绋嬩腑琚噸澶嶅寘鍚€?
#pragma once

// 瑙ｉ噴锛氬紩鍏?Unreal 鐨勬牳蹇冨熀纭€澶存枃浠讹紝鎻愪緵甯哥敤瀹瑰櫒銆佹暟瀛︾被鍨嬪拰鏃ュ織瀹忋€?
#include "CoreMinimal.h"

// 瑙ｉ噴锛氳繖涓€琛屽０鏄?绫?`AActor`锛岀敤浜庡皝瑁卆actor鐩稿叧鐨勬暟鎹笌琛屼负銆?
class AActor;
// 瑙ｉ噴锛氳繖涓€琛屽０鏄?绫?`UCineCameraComponent`锛岀敤浜庡皝瑁卽cine鐩告満缁勪欢鐩稿叧鐨勬暟鎹笌琛屼负銆?
class UCineCameraComponent;
// 瑙ｉ噴锛氳繖涓€琛屽０鏄?绫?`UObject`锛岀敤浜庡皝瑁卽object鐩稿叧鐨勬暟鎹笌琛屼负銆?
class UObject;
// 瑙ｉ噴锛氳繖涓€琛屽０鏄?绫?`USceneCaptureComponent2D`锛岀敤浜庡皝瑁卽scene閲囬泦component2D鐩稿叧鐨勬暟鎹笌琛屼负銆?
class USceneCaptureComponent2D;
// 瑙ｉ噴锛氳繖涓€琛屽０鏄?绫?`UTextureRenderTarget2D`锛岀敤浜庡皝瑁卽texturerendertarget2D鐩稿叧鐨勬暟鎹笌琛屼负銆?
class UTextureRenderTarget2D;

/**
 * @brief 鐩告満鎹曡幏涓庡悗澶勭悊杈呭姪宸ュ叿闆?
 *
 * 璇ュ懡鍚嶇┖闂存妸鈥滃垱寤?RenderTarget銆佹姄鍙栧浘鍍忋€佸悓姝ュ悗澶勭悊鍙傛暟銆?
 * 璁剧疆鍒嗗壊 Stencil鈥濈瓑妯法鏃犱汉鏈哄拰鍏朵粬杞戒綋鐨勫叕鍏遍€昏緫闆嗕腑绠＄悊锛?
 * 閬垮厤鍦ㄥ涓?Pawn 涓噸澶嶅疄鐜扮浉鍚屼唬鐮併€?
 */
// 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
namespace CameraCaptureUtils
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
{
    /**
     * @brief 鍒涘缓褰╄壊 RenderTarget
     * @param Owner  RenderTarget 鐨?Outer
     * @param Width  鐩爣瀹藉害
     * @param Height 鐩爣楂樺害
     * @return 鏂板缓鐨?RenderTarget锛涘昂瀵搁潪娉曟垨鍒涘缓澶辫触鏃惰繑鍥?nullptr
     */
    // 瑙ｉ噴锛氳皟鐢?`CreateColorRenderTarget` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    UTextureRenderTarget2D* CreateColorRenderTarget(UObject* Owner, int32 Width, int32 Height);

    /**
     * @brief 鎶撳彇涓€甯у僵鑹插浘鍍忓苟缂栫爜涓?Base64 JPEG
     * @param Capture 鍦烘櫙鎹曡幏缁勪欢
     * @param Width   杈撳嚭瀹藉害
     * @param Height  杈撳嚭楂樺害
     * @param Quality JPEG 璐ㄩ噺
     * @return Base64 瀛楃涓诧紱澶辫触鏃惰繑鍥炵┖涓?
     */
    // 瑙ｉ噴锛氳皟鐢?`CaptureColorJpegBase64` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    FString CaptureColorJpegBase64(USceneCaptureComponent2D* Capture, int32 Width, int32 Height, int32 Quality);

    /**
     * @brief 鎸?AirSim 椋庢牸灏佽鍥惧儚 JSON 鍝嶅簲
     * @param Capture        鍦烘櫙鎹曡幏缁勪欢
     * @param SourceId       鍥惧儚鏉ユ簮 ID
     * @param Width          鍥惧儚瀹藉害
     * @param Height         鍥惧儚楂樺害
     * @param FieldOfView    鐩告満瑙嗗満瑙?
     * @param DefaultQuality 榛樿 JPEG 璐ㄩ噺
     * @param ImageType      鍥惧儚绫诲瀷瀛楃涓?
     * @param Quality        澶栭儴鎸囧畾 JPEG 璐ㄩ噺
     * @param MaxDepthMeters 娣卞害鍥炬渶澶ф槧灏勮窛绂?
     * @return JSON 瀛楃涓?
     */
    // 瑙ｉ噴锛氳繖涓€琛屽畾涔夊嚱鏁?`CaptureImageJson`锛屽紑濮嬪疄鐜伴噰闆嗙┖涓豢鐪熷浘鍍廽son鐨勫叿浣撻€昏緫銆?
    FString CaptureImageJson(
        // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `CaptureImageJson` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `Capture` 鐢ㄤ簬浼犲叆閲囬泦銆?
        USceneCaptureComponent2D* Capture,
        // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `CaptureImageJson` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `SourceId` 鐢ㄤ簬浼犲叆sourceid銆?
        const FString& SourceId,
        // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `CaptureImageJson` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `Width` 鐢ㄤ簬浼犲叆width銆?
        int32 Width,
        // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `CaptureImageJson` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `Height` 鐢ㄤ簬浼犲叆height銆?
        int32 Height,
        // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `CaptureImageJson` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `FieldOfView` 鐢ㄤ簬浼犲叆fieldofview銆?
        float FieldOfView,
        // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `CaptureImageJson` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `DefaultQuality` 鐢ㄤ簬浼犲叆defaultquality銆?
        int32 DefaultQuality,
        // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `CaptureImageJson` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `ImageType` 鐢ㄤ簬浼犲叆鍥惧儚type銆?
        const FString& ImageType,
        // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `CaptureImageJson` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `Quality` 鐢ㄤ簬浼犲叆quality銆?
        int32 Quality,
        // 瑙ｉ噴锛氳繖涓€琛岀户缁ˉ鍏呭嚱鏁?`CaptureImageJson` 鐨勫弬鏁板垪琛ㄣ€侀檺瀹氱鎴栬繑鍥炵被鍨嬭鏄庛€?
        float MaxDepthMeters);

    /**
     * @brief 涓?Actor 鍏ㄩ儴 Primitive 缁勪欢搴旂敤璇箟鍒嗗壊 Stencil 鍊?
     * @param Owner          鐩爣 Actor
     * @param SegmentationId 0-255 鐨勮嚜瀹氫箟娣卞害妯℃澘鍊?
     */
    // 瑙ｉ噴锛氳皟鐢?`ApplySegmentationStencil` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    void ApplySegmentationStencil(AActor* Owner, int32 SegmentationId);

    /**
     * @brief 鎶?CineCamera 鐨勫悗澶勭悊璁剧疆鍚屾鍒?SceneCapture
     * @param CineCamera         鐢靛奖鐩告満缁勪欢
     * @param Capture            鍦烘櫙鎹曡幏缁勪欢
     * @param ExposureBias       棰濆鏇濆厜琛ュ伩
     * @param bDisableMotionBlur 鏄惁寮哄埗鍏抽棴鍔ㄦ€佹ā绯?
     */
    // 瑙ｉ噴锛氳繖涓€琛屽畾涔夊嚱鏁?`SyncPostProcessToCapture`锛屽紑濮嬪疄鐜皊yncpostprocessto閲囬泦鐨勫叿浣撻€昏緫銆?
    void SyncPostProcessToCapture(
        // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `SyncPostProcessToCapture` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `CineCamera` 鐢ㄤ簬浼犲叆cine鐩告満銆?
        UCineCameraComponent* CineCamera,
        // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `SyncPostProcessToCapture` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `Capture` 鐢ㄤ簬浼犲叆閲囬泦銆?
        USceneCaptureComponent2D* Capture,
        // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `SyncPostProcessToCapture` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `ExposureBias` 鐢ㄤ簬浼犲叆exposurebias銆?
        float ExposureBias,
        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`bDisableMotionBlur`锛岀敤浜庝繚瀛樺竷灏旀爣蹇?disablemotionblur銆?
        bool bDisableMotionBlur = false);
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
}

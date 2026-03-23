// 瑙ｉ噴锛氬紩鍏ュ綋鍓嶅疄鐜版枃浠跺搴旂殑澶存枃浠?`CameraCaptureUtils.h`锛屼娇瀹炵幇閮ㄥ垎鑳藉鐪嬪埌绫诲拰鍑芥暟澹版槑銆?
#include "CameraCaptureUtils.h"

// 瑙ｉ噴锛氬紩鍏?`AirSimImageUtils.h`锛屼负褰撳墠鏂囦欢琛ュ厖鎵€渚濊禆鐨勭被鍨嬨€佸嚱鏁版垨鎺ュ彛澹版槑銆?
#include "AirSimImageUtils.h"
// 瑙ｉ噴锛氬紩鍏?`CineCameraComponent.h`锛屼负褰撳墠鏂囦欢琛ュ厖鎵€渚濊禆鐨勭被鍨嬨€佸嚱鏁版垨鎺ュ彛澹版槑銆?
#include "CineCameraComponent.h"
// 瑙ｉ噴锛氬紩鍏?`PrimitiveComponent.h`锛屼负褰撳墠鏂囦欢琛ュ厖鎵€渚濊禆鐨勭被鍨嬨€佸嚱鏁版垨鎺ュ彛澹版槑銆?
#include "Components/PrimitiveComponent.h"
// 瑙ｉ噴锛氬紩鍏?`SceneCaptureComponent2D.h`锛屼负褰撳墠鏂囦欢琛ュ厖鎵€渚濊禆鐨勭被鍨嬨€佸嚱鏁版垨鎺ュ彛澹版槑銆?
#include "Components/SceneCaptureComponent2D.h"
// 瑙ｉ噴锛氬紩鍏?`TextureRenderTarget2D.h`锛屼负褰撳墠鏂囦欢琛ュ厖鎵€渚濊禆鐨勭被鍨嬨€佸嚱鏁版垨鎺ュ彛澹版槑銆?
#include "Engine/TextureRenderTarget2D.h"
// 瑙ｉ噴锛氬紩鍏?`Actor.h`锛屼负褰撳墠鏂囦欢琛ュ厖鎵€渚濊禆鐨勭被鍨嬨€佸嚱鏁版垨鎺ュ彛澹版槑銆?
#include "GameFramework/Actor.h"
// 瑙ｉ噴锛氬紩鍏?`IConsoleManager.h`锛屼负褰撳墠鏂囦欢琛ュ厖鎵€渚濊禆鐨勭被鍨嬨€佸嚱鏁版垨鎺ュ彛澹版槑銆?
#include "HAL/IConsoleManager.h"
// 瑙ｉ噴锛氬紩鍏?`IImageWrapper.h`锛屼负褰撳墠鏂囦欢琛ュ厖鎵€渚濊禆鐨勭被鍨嬨€佸嚱鏁版垨鎺ュ彛澹版槑銆?
#include "IImageWrapper.h"
// 瑙ｉ噴锛氬紩鍏?`IImageWrapperModule.h`锛屼负褰撳墠鏂囦欢琛ュ厖鎵€渚濊禆鐨勭被鍨嬨€佸嚱鏁版垨鎺ュ彛澹版槑銆?
#include "IImageWrapperModule.h"
// 瑙ｉ噴锛氬紩鍏?`Base64.h`锛屼负褰撳墠鏂囦欢琛ュ厖鎵€渚濊禆鐨勭被鍨嬨€佸嚱鏁版垨鎺ュ彛澹版槑銆?
#include "Misc/Base64.h"
// 瑙ｉ噴锛氬紩鍏?`ModuleManager.h`锛屼负褰撳墠鏂囦欢琛ュ厖鎵€渚濊禆鐨勭被鍨嬨€佸嚱鏁版垨鎺ュ彛澹版槑銆?
#include "Modules/ModuleManager.h"

// 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
namespace
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
{
    /**
     * @brief 杞箟 JSON 涓殑瀛楃涓插瓧娈?
     * @param In 鍘熷瀛楃涓?
     * @return 閫傚悎鐩存帴宓屽叆 JSON 鐨勮浆涔夌粨鏋?
     */
    // 瑙ｉ噴锛氳繖涓€琛屽畾涔夊嚱鏁?`EscapeJsonValue`锛屽紑濮嬪疄鐜癳scapejsonvalue鐨勫叿浣撻€昏緫銆?
    FString EscapeJsonValue(const FString& In)
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`Out`锛岀敤浜庝繚瀛榦ut銆?
        FString Out = In;
        // 瑙ｉ噴锛氳皟鐢?`ReplaceInline` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
        Out.ReplaceInline(TEXT("\\"), TEXT("\\\\"));
        // 瑙ｉ噴锛氳皟鐢?`ReplaceInline` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
        Out.ReplaceInline(TEXT("\""), TEXT("\\\""));
        // 瑙ｉ噴锛氳繖涓€琛岃繑鍥炲綋鍓嶅嚱鏁扮殑璁＄畻缁撴灉锛屾妸鎺у埗鏉冧氦鍥炶皟鐢ㄦ柟銆?
        return Out;
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }

    /**
     * @brief 瑙ｆ瀽鏈€缁?JPEG 璐ㄩ噺
     * @param Quality        澶栭儴浼犲叆璐ㄩ噺
     * @param DefaultQuality 榛樿璐ㄩ噺
     * @return 鍚堟硶鑼冨洿 [1, 100] 鍐呯殑鏈€缁堣川閲?
     */
    // 瑙ｉ噴锛氳繖涓€琛屽畾涔夊嚱鏁?`ResolveJpegQuality`锛屽紑濮嬪疄鐜皉esolvejpegquality鐨勫叿浣撻€昏緫銆?
    int32 ResolveJpegQuality(int32 Quality, int32 DefaultQuality)
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛屼娇鐢ㄤ笁鐩〃杈惧紡閫夋嫨杩斿洖鍊硷紝鍦ㄦ甯歌矾寰勫拰鍏滃簳璺緞涔嬮棿鍋氬揩閫熷垏鎹€?
        return (Quality > 0) ? Quality : FMath::Clamp(DefaultQuality, 1, 100);
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }

    /**
     * @brief 鎶?BGRA 鍍忕礌缂栫爜涓?Base64 JPEG
     * @param Pixels    鍍忕礌鏁扮粍
     * @param Width     鍥惧儚瀹藉害
     * @param Height    鍥惧儚楂樺害
     * @param Quality   JPEG 璐ㄩ噺
     * @param OutBase64 杈撳嚭 Base64 瀛楃涓?
     * @return 缂栫爜鎴愬姛鏃惰繑鍥?true
     */
    // 瑙ｉ噴锛氳繖涓€琛屽畾涔夊嚱鏁?`EncodeBgraToJpegBase64`锛屽紑濮嬪疄鐜癳ncodebgratojpegbase64鐨勫叿浣撻€昏緫銆?
    bool EncodeBgraToJpegBase64(const TArray<FColor>& Pixels, int32 Width, int32 Height, int32 Quality, FString& OutBase64)
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
        if (Pixels.Num() != Width * Height || Width <= 0 || Height <= 0)
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        {
            // 瑙ｉ噴锛氳繖涓€琛岃繑鍥炲綋鍓嶅嚱鏁扮殑璁＄畻缁撴灉锛屾妸鎺у埗鏉冧氦鍥炶皟鐢ㄦ柟銆?
            return false;
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        }

        // `FColor` 浠?BGRA 椤哄簭瀛樺偍锛岃繖閲屾樉寮忓睍寮€涓鸿繛缁紦鍐插尯渚?ImageWrapper 鍘嬬缉銆?
        // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`RawData`锛岀敤浜庝繚瀛榬awdata銆?
        TArray<uint8> RawData;
        // 瑙ｉ噴锛氳皟鐢?`SetNum` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
        RawData.SetNum(Pixels.Num() * 4);
        // 瑙ｉ噴锛氳繖涓€琛屽紑濮?`for` 寰幆锛岀敤浜庢寜鏃㈠畾娆℃暟鎴栧簭鍒楅亶鍘嗘墽琛屽悗缁€昏緫銆?
        for (int32 i = 0; i < Pixels.Num(); ++i)
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        {
            // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `RawData[i * 4 + 0]`锛屽畬鎴?rawdata 鐨勬洿鏂般€?
            RawData[i * 4 + 0] = Pixels[i].B;
            // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `RawData[i * 4 + 1]`锛屽畬鎴?rawdata 鐨勬洿鏂般€?
            RawData[i * 4 + 1] = Pixels[i].G;
            // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `RawData[i * 4 + 2]`锛屽畬鎴?rawdata 鐨勬洿鏂般€?
            RawData[i * 4 + 2] = Pixels[i].R;
            // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `RawData[i * 4 + 3]`锛屽畬鎴?rawdata 鐨勬洿鏂般€?
            RawData[i * 4 + 3] = Pixels[i].A;
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        }

        // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `IImageWrapperModule& ImageWrapperModule`锛屽畬鎴?iimagewrappermodule鍥惧儚wrappermodule 鐨勬洿鏂般€?
        IImageWrapperModule& ImageWrapperModule = FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));
        // 瑙ｉ噴锛氳皟鐢?`CreateImageWrapper` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
        TSharedPtr<IImageWrapper> Wrapper = ImageWrapperModule.CreateImageWrapper(EImageFormat::JPEG);
        // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
        if (!Wrapper.IsValid())
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        {
            // 瑙ｉ噴锛氳繖涓€琛岃繑鍥炲綋鍓嶅嚱鏁扮殑璁＄畻缁撴灉锛屾妸鎺у埗鏉冧氦鍥炶皟鐢ㄦ柟銆?
            return false;
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        }

        // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
        if (!Wrapper->SetRaw(RawData.GetData(), RawData.Num(), Width, Height, ERGBFormat::BGRA, 8))
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        {
            // 瑙ｉ噴锛氳繖涓€琛岃繑鍥炲綋鍓嶅嚱鏁扮殑璁＄畻缁撴灉锛屾妸鎺у埗鏉冧氦鍥炶皟鐢ㄦ柟銆?
            return false;
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        }

        // 瑙ｉ噴锛氳皟鐢?`GetCompressed` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
        const TArray64<uint8>& JpegData = Wrapper->GetCompressed(FMath::Clamp(Quality, 1, 100));
        // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `OutBase64`锛屽畬鎴?outbase64 鐨勬洿鏂般€?
        OutBase64 = FBase64::Encode(JpegData.GetData(), JpegData.Num());
        // 瑙ｉ噴锛氳繖涓€琛岃繑鍥炲綋鍓嶅嚱鏁扮殑璁＄畻缁撴灉锛屾妸鎺у埗鏉冧氦鍥炶皟鐢ㄦ柟銆?
        return true;
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
}

/**
 * @brief 鍒涘缓褰╄壊 RenderTarget
 * @param Owner  RenderTarget 鐨?Outer
 * @param Width  瀹藉害
 * @param Height 楂樺害
 * @return 鏂板缓 RenderTarget锛涘け璐ユ椂杩斿洖 nullptr
 */
// 瑙ｉ噴锛氳繖涓€琛屽畾涔夊嚱鏁?`CreateColorRenderTarget`锛屽紑濮嬪疄鐜癱reatecolorrendertarget鐨勫叿浣撻€昏緫銆?
UTextureRenderTarget2D* CameraCaptureUtils::CreateColorRenderTarget(UObject* Owner, int32 Width, int32 Height)
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
{
    // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
    if (Width <= 0 || Height <= 0)
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛岃繑鍥炲綋鍓嶅嚱鏁扮殑璁＄畻缁撴灉锛屾妸鎺у埗鏉冧氦鍥炶皟鐢ㄦ柟銆?
        return nullptr;
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }

    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `UObject* Outer`锛屽畬鎴?uobjectouter 鐨勬洿鏂般€?
    UObject* Outer = Owner ? Owner : GetTransientPackage();
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `UTextureRenderTarget2D* RenderTarget`锛屽畬鎴?utexturerendertarget2Drendertarget 鐨勬洿鏂般€?
    UTextureRenderTarget2D* RenderTarget = NewObject<UTextureRenderTarget2D>(Outer);
    // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
    if (!RenderTarget)
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛岃繑鍥炲綋鍓嶅嚱鏁扮殑璁＄畻缁撴灉锛屾妸鎺у埗鏉冧氦鍥炶皟鐢ㄦ柟銆?
        return nullptr;
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }

    // 瑙ｉ噴锛氳皟鐢?`InitCustomFormat` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    RenderTarget->InitCustomFormat(Width, Height, PF_B8G8R8A8, false);
    // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
    RenderTarget->ClearColor = FLinearColor::Black;
    // 瑙ｉ噴锛氳繖涓€琛岃繑鍥炲綋鍓嶅嚱鏁扮殑璁＄畻缁撴灉锛屾妸鎺у埗鏉冧氦鍥炶皟鐢ㄦ柟銆?
    return RenderTarget;
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
}

/**
 * @brief 鎶撳彇涓€甯у僵鑹插浘鍍忓苟缂栫爜涓?Base64 JPEG
 * @param Capture 鍦烘櫙鎹曡幏缁勪欢
 * @param Width   杈撳嚭瀹藉害
 * @param Height  杈撳嚭楂樺害
 * @param Quality JPEG 璐ㄩ噺
 * @return Base64 瀛楃涓诧紱澶辫触鏃惰繑鍥炵┖涓?
 */
// 瑙ｉ噴锛氳繖涓€琛屽畾涔夊嚱鏁?`CaptureColorJpegBase64`锛屽紑濮嬪疄鐜伴噰闆哻olorjpegbase64鐨勫叿浣撻€昏緫銆?
FString CameraCaptureUtils::CaptureColorJpegBase64(USceneCaptureComponent2D* Capture, int32 Width, int32 Height, int32 Quality)
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
{
    // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
    if (!Capture || !Capture->TextureTarget || Width <= 0 || Height <= 0)
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛岃繑鍥炲綋鍓嶅嚱鏁扮殑璁＄畻缁撴灉锛屾妸鎺у埗鏉冧氦鍥炶皟鐢ㄦ柟銆?
        return TEXT("");
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }

    // 鑻ュ叧闂簡閫愬抚鎹曡幏锛屽垯鎵嬪姩瑙﹀彂涓€娆℃覆鏌擄紝纭繚璇诲彇鍒版渶鏂板浘鍍忋€?
    // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
    if (!Capture->bCaptureEveryFrame)
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳皟鐢?`CaptureScene` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
        Capture->CaptureScene();
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }

    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `FTextureRenderTargetResource* Resource`锛屽畬鎴?ftexturerendertargetresourceresource 鐨勬洿鏂般€?
    FTextureRenderTargetResource* Resource = Capture->TextureTarget->GameThread_GetRenderTargetResource();
    // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
    if (!Resource)
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛岃繑鍥炲綋鍓嶅嚱鏁扮殑璁＄畻缁撴灉锛屾妸鎺у埗鏉冧氦鍥炶皟鐢ㄦ柟銆?
        return TEXT("");
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }

    // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`Pixels`锛岀敤浜庝繚瀛榩ixels銆?
    TArray<FColor> Pixels;
    // 瑙ｉ噴锛氳皟鐢?`SetNum` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    Pixels.SetNum(Width * Height);

    // 瑙ｉ噴锛氳皟鐢?`ReadFlags` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    FReadSurfaceDataFlags ReadFlags(RCM_UNorm, CubeFace_MAX);
    // 瑙ｉ噴锛氳皟鐢?`SetLinearToGamma` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    ReadFlags.SetLinearToGamma(false);
    // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
    if (!Resource->ReadPixels(Pixels, ReadFlags))
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛岃繑鍥炲綋鍓嶅嚱鏁扮殑璁＄畻缁撴灉锛屾妸鎺у埗鏉冧氦鍥炶皟鐢ㄦ柟銆?
        return TEXT("");
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }

    // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`Base64`锛岀敤浜庝繚瀛榖ase64銆?
    FString Base64;
    // 瑙ｉ噴锛氳繖涓€琛屼娇鐢ㄤ笁鐩〃杈惧紡閫夋嫨杩斿洖鍊硷紝鍦ㄦ甯歌矾寰勫拰鍏滃簳璺緞涔嬮棿鍋氬揩閫熷垏鎹€?
    return EncodeBgraToJpegBase64(Pixels, Width, Height, Quality, Base64) ? Base64 : TEXT("");
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
}

/**
 * @brief 浠?AirSim 椋庢牸灏佽鍥惧儚 JSON
 * @param Capture        鍦烘櫙鎹曡幏缁勪欢
 * @param SourceId       鍥惧儚鏉ユ簮 ID
 * @param Width          鍥惧儚瀹藉害
 * @param Height         鍥惧儚楂樺害
 * @param FieldOfView    鐩告満瑙嗗満瑙?
 * @param DefaultQuality 榛樿 JPEG 璐ㄩ噺
 * @param ImageType      鍥惧儚绫诲瀷
 * @param Quality        璇锋眰 JPEG 璐ㄩ噺
 * @param MaxDepthMeters 娣卞害鍥炬渶澶ц窛绂?
 * @return JSON 瀛楃涓?
 */
// 瑙ｉ噴锛氳繖涓€琛屽畾涔夊嚱鏁?`CaptureImageJson`锛屽紑濮嬪疄鐜伴噰闆嗙┖涓豢鐪熷浘鍍廽son鐨勫叿浣撻€昏緫銆?
FString CameraCaptureUtils::CaptureImageJson(
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
    // 瑙ｉ噴锛氳繖涓€琛屾敹鏉熷嚱鏁?`CaptureImageJson` 鐨勭鍚嶏紝鍚庨潰浼氳繘鍏ュ疄鐜颁綋鎴栦互鍒嗗彿缁撴潫澹版槑銆?
    float MaxDepthMeters)
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
{
    // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`ParsedType`锛岀敤浜庝繚瀛榩arsedtype銆?
    AirSimImageUtils::EImageType ParsedType = AirSimImageUtils::EImageType::Scene;
    // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
    if (!AirSimImageUtils::TryParseImageType(ImageType, ParsedType))
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛岃繑鍥炲綋鍓嶅嚱鏁扮殑璁＄畻缁撴灉锛屾妸鎺у埗鏉冧氦鍥炶皟鐢ㄦ柟銆?
        return TEXT("{\"status\":\"error\",\"message\":\"Unsupported image_type\"}");
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }

    // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
    const FString Base64 = AirSimImageUtils::CaptureJpegBase64(
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        Capture,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        Width,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        Height,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        ParsedType,
        // 瑙ｉ噴锛氳繖涓€琛屼綅浜庢瀯閫犲嚱鏁板垵濮嬪寲鍒楄〃涓紝鎶?`ResolveJpegQuality` 鐩存帴鍒濆鍖栦负 `Quality, DefaultQuality`锛屽噺灏戣繘鍏ュ嚱鏁颁綋鍚庣殑棰濆璧嬪€煎紑閿€銆?
        ResolveJpegQuality(Quality, DefaultQuality),
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        MaxDepthMeters);
    // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
    if (Base64.IsEmpty())
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛岃繑鍥炲綋鍓嶅嚱鏁扮殑璁＄畻缁撴灉锛屾妸鎺у埗鏉冧氦鍥炶皟鐢ㄦ柟銆?
        return TEXT("{\"status\":\"error\",\"message\":\"capture failed\"}");
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }

    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `const FVector CameraPosition`锛屽畬鎴?constfvector鐩告満position 鐨勬洿鏂般€?
    const FVector CameraPosition = Capture ? Capture->GetComponentLocation() : FVector::ZeroVector;
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `const FRotator CameraRotation`锛屽畬鎴?constfrotator鐩告満rotation 鐨勬洿鏂般€?
    const FRotator CameraRotation = Capture ? Capture->GetComponentRotation() : FRotator::ZeroRotator;
    // 瑙ｉ噴锛氳繖涓€琛屾妸鍙充晶琛ㄨ揪寮忕殑缁撴灉鍐欏叆 `const FString CanonicalType`锛屽畬鎴?constfstringcanonicaltype 鐨勬洿鏂般€?
    const FString CanonicalType = AirSimImageUtils::ToCanonicalString(ParsedType);

    // 瑙ｉ噴锛氳繖涓€琛岃繑鍥炲綋鍓嶅嚱鏁扮殑璁＄畻缁撴灉锛屾妸鎺у埗鏉冧氦鍥炶皟鐢ㄦ柟銆?
    return FString::Printf(
        // 瑙ｉ噴锛氳繖涓€琛屼綅浜庢瀯閫犲嚱鏁板垵濮嬪寲鍒楄〃涓紝鎶?`TEXT` 鐩存帴鍒濆鍖栦负 `"{\"status\":\"ok\",\"source\":\"%s\",\"image_type\":\"%s\",\"image_type_id\":%d,\"width\":%d,\"height\":%d,\"format\":\"jpeg\",\"camera_pos\":[%.2f,%.2f,%.2f],\"camera_rot\":[%.2f,%.2f,%.2f],\"fov\":%.2f,\"data\":\"%s\"}"`锛屽噺灏戣繘鍏ュ嚱鏁颁綋鍚庣殑棰濆璧嬪€煎紑閿€銆?
        TEXT("{\"status\":\"ok\",\"source\":\"%s\",\"image_type\":\"%s\",\"image_type_id\":%d,\"width\":%d,\"height\":%d,\"format\":\"jpeg\",\"camera_pos\":[%.2f,%.2f,%.2f],\"camera_rot\":[%.2f,%.2f,%.2f],\"fov\":%.2f,\"data\":\"%s\"}"),
        *EscapeJsonValue(SourceId),
        *CanonicalType,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        static_cast<int32>(ParsedType),
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        Width,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        Height,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        CameraPosition.X,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        CameraPosition.Y,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        CameraPosition.Z,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        CameraRotation.Pitch,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        CameraRotation.Yaw,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        CameraRotation.Roll,
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        FieldOfView,
        *Base64);
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
}

/**
 * @brief 搴旂敤璇箟鍒嗗壊 Stencil 鍊?
 * @param Owner          鐩爣 Actor
 * @param SegmentationId 妯℃澘鍊?
 */
// 瑙ｉ噴锛氳繖涓€琛屽畾涔夊嚱鏁?`ApplySegmentationStencil`锛屽紑濮嬪疄鐜癮pplysegmentationstencil鐨勫叿浣撻€昏緫銆?
void CameraCaptureUtils::ApplySegmentationStencil(AActor* Owner, int32 SegmentationId)
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
{
    // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
    if (!Owner)
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛岃繑鍥炲綋鍓嶅嚱鏁扮殑璁＄畻缁撴灉锛屾妸鎺у埗鏉冧氦鍥炶皟鐢ㄦ柟銆?
        return;
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }

    // CustomDepth=3 琛ㄧず鍚敤骞跺厑璁稿啓鍏?Stencil锛岃涔夊垎鍓蹭緷璧栬妯″紡銆?
    // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
    if (IConsoleVariable* CustomDepthVar = IConsoleManager::Get().FindConsoleVariable(TEXT("r.CustomDepth")))
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
        if (CustomDepthVar->GetInt() < 3)
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        {
            // 瑙ｉ噴锛氳皟鐢?`Set` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
            CustomDepthVar->Set(3, ECVF_SetByCode);
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        }
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }

    // 瑙ｉ噴锛氳繖涓€琛屽厛瀵硅绠楃粨鏋滃仛闄愬箙锛屽啀鍐欏叆 `const int32 ClampedId`锛岄槻姝?constint32clampedid 瓒呭嚭鍏佽鑼冨洿銆?
    const int32 ClampedId = FMath::Clamp(SegmentationId, 0, 255);

    // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`PrimitiveComponents`锛岀敤浜庝繚瀛榩rimitivecomponents銆?
    TArray<UPrimitiveComponent*> PrimitiveComponents;
    // 瑙ｉ噴锛氳皟鐢?`UPrimitiveComponent>` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    Owner->GetComponents<UPrimitiveComponent>(PrimitiveComponents);
    // 瑙ｉ噴锛氳繖涓€琛屽紑濮?`for` 寰幆锛岀敤浜庢寜鏃㈠畾娆℃暟鎴栧簭鍒楅亶鍘嗘墽琛屽悗缁€昏緫銆?
    for (UPrimitiveComponent* Primitive : PrimitiveComponents)
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
        if (!Primitive)
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        {
            // 瑙ｉ噴锛氳繖涓€琛岃烦杩囨湰杞惊鐜墿浣欒鍙ワ紝鐩存帴杩涘叆涓嬩竴杞凯浠ｃ€?
            continue;
        // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
        }

        // 瑙ｉ噴锛氳皟鐢?`SetRenderCustomDepth` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
        Primitive->SetRenderCustomDepth(true);
        // 瑙ｉ噴锛氳皟鐢?`SetCustomDepthStencilValue` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
        Primitive->SetCustomDepthStencilValue(ClampedId);
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
}

/**
 * @brief 鍚屾 CineCamera 鍚庡鐞嗗埌 SceneCapture
 * @param CineCamera         鐢靛奖鐩告満
 * @param Capture            鍦烘櫙鎹曡幏缁勪欢
 * @param ExposureBias       鏇濆厜鍋忕疆
 * @param bDisableMotionBlur 鏄惁寮哄埗绂佺敤鍔ㄦ€佹ā绯?
 */
// 瑙ｉ噴锛氳繖涓€琛屽畾涔夊嚱鏁?`SyncPostProcessToCapture`锛屽紑濮嬪疄鐜皊yncpostprocessto閲囬泦鐨勫叿浣撻€昏緫銆?
void CameraCaptureUtils::SyncPostProcessToCapture(
    // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `SyncPostProcessToCapture` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `CineCamera` 鐢ㄤ簬浼犲叆cine鐩告満銆?
    UCineCameraComponent* CineCamera,
    // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `SyncPostProcessToCapture` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `Capture` 鐢ㄤ簬浼犲叆閲囬泦銆?
    USceneCaptureComponent2D* Capture,
    // 瑙ｉ噴锛氳繖涓€琛岀户缁睍寮€ `SyncPostProcessToCapture` 鐨勫弬鏁板垪琛紝澹版槑鍙傛暟 `ExposureBias` 鐢ㄤ簬浼犲叆exposurebias銆?
    float ExposureBias,
    // 瑙ｉ噴锛氳繖涓€琛屾敹鏉熷嚱鏁?`SyncPostProcessToCapture` 鐨勭鍚嶏紝鍚庨潰浼氳繘鍏ュ疄鐜颁綋鎴栦互鍒嗗彿缁撴潫澹版槑銆?
    bool bDisableMotionBlur)
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
{
    // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
    if (!CineCamera || !Capture)
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛岃繑鍥炲綋鍓嶅嚱鏁扮殑璁＄畻缁撴灉锛屾妸鎺у埗鏉冧氦鍥炶皟鐢ㄦ柟銆?
        return;
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }

    // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`ViewInfo`锛岀敤浜庝繚瀛榲iewinfo銆?
    FMinimalViewInfo ViewInfo;
    // 瑙ｉ噴锛氳皟鐢?`GetCameraView` 鎵ц褰撳墠姝ラ闇€瑕佺殑鍔熻兘閫昏緫銆?
    CineCamera->GetCameraView(0.0f, ViewInfo);

    // 淇濈暀宸叉湁 Blendables锛岄伩鍏嶇洿鎺ヨ鐩栧悗涓㈠け闄勫姞鍚庡鐞嗘潗璐ㄣ€?
    // 瑙ｉ噴锛氳繖涓€琛屽０鏄庢垚鍛樻垨灞€閮ㄥ彉閲?`SavedBlendables`锛岀敤浜庝繚瀛榮avedblendables銆?
    const FWeightedBlendables SavedBlendables = Capture->PostProcessSettings.WeightedBlendables;
    // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
    Capture->PostProcessSettings = ViewInfo.PostProcessSettings;
    // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
    Capture->PostProcessSettings.WeightedBlendables = SavedBlendables;
    // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
    Capture->PostProcessSettings.bOverride_AutoExposureBias = true;
    // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
    Capture->PostProcessSettings.AutoExposureBias += ExposureBias;

    // 瑙ｉ噴锛氳繖涓€琛岀粰鍑?`if` 鏉′欢鍒ゆ柇锛屽彧鏈夋潯浠舵垚绔嬫椂鎵嶄細杩涘叆涓嬮潰鐨勫垎鏀€昏緫銆?
    if (bDisableMotionBlur)
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    {
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        Capture->PostProcessSettings.bOverride_MotionBlurAmount = true;
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        Capture->PostProcessSettings.MotionBlurAmount = 0.0f;
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        Capture->PostProcessSettings.bOverride_MotionBlurMax = true;
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        Capture->PostProcessSettings.MotionBlurMax = 0.0f;
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        Capture->PostProcessSettings.bOverride_MotionBlurPerObjectSize = true;
        // 瑙ｉ噴锛氳繖涓€琛岃惤瀹炲綋鍓嶆ā鍧椾腑鐨勫叿浣撳疄鐜扮粏鑺傦紝涓轰笂闈㈢殑澹版槑銆佸叕寮忔垨鎺у埗娴佺▼鎻愪緵瀹為檯鎵ц璇彞銆?
        Capture->PostProcessSettings.MotionBlurPerObjectSize = 0.0f;
    // 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
    }
// 瑙ｉ噴锛氳繖涓€琛岀敤浜庡紑濮嬫垨缁撴潫褰撳墠浣滅敤鍩燂紝鎺у埗绫汇€佸嚱鏁版垨鏉′欢鍧楃殑杈圭晫銆?
}

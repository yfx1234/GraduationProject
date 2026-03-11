#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "Dom/JsonObject.h"
#include "FCommandHandle.h"
#include "CommandRouter.generated.h"

class UDroneCommandHandler;
class UTurretCommandHandler;
class UGuidanceCommandHandler;


/**
 * TCP 鍛戒护璺敱鍣?
 * 浣滅敤锛?
 * 1. 瑙ｆ瀽 JSON 瀛楃涓?
 * 2. 鏍规嵁 JSON 瀛楁鍚嶈矾鐢卞埌瀵瑰簲鐨勫鐞嗗櫒
 * 3. 杩斿洖 JSON 鏍煎紡鐨勫搷搴斿瓧绗︿覆
 * 璺敱瑙勫垯锛堟寜 JSON 瀛楁鍖归厤锛夛細
 * - "ping" 鈫?HandlePing()
 * - "sim_pause/sim_resume/sim_reset" 鈫?浠跨湡鎺у埗
 * - "get_agent_list" 鈫?鑾峰彇鏅鸿兘浣撳垪琛?
 * - "get_image" 鈫?鑾峰彇鎽勫儚澶村浘鍍忥紙鏀寔 Turret 鍜?Drone锛?
 * - "call_drone/get_drone_state" 鈫?DroneCommandHandler
 * - "call_turret/get_turret_state" 鈫?TurretCommandHandler
 * - "call_guidance/get_guidance_state" 鈫?GuidanceCommandHandler
 * - 鏂板锛?add_actor", "remove_actor", "call_actor" 鈫?FCommandHandle
 */
UCLASS()
class GRADUATIONPROJECT_API UCommandRouter : public UObject
{
    GENERATED_BODY()

public:
    /**
     * @brief 澶勭悊 TCP 鍛戒护
     * @param JsonString 鏀跺埌鐨?JSON 瀛楃涓?
     * @param World 褰撳墠 UWorld 鎸囬拡
     * @return 鍝嶅簲 JSON 瀛楃涓诧紝鍖呭惈 status 鍜?message 瀛楁
     */
    FString HandleCommand(const FString& JsonString, UWorld* World);

private:
    /**
     * @brief 澶勭悊 ping 鍛戒护锛岃繑鍥?pong 鍝嶅簲
     * @return JSON 鍝嶅簲 {"status":"ok","message":"pong"}
     */
    FString HandlePing();

    /**
     * @brief 鏆傚仠浠跨湡
     * @param World 褰撳墠 World
     * @return JSON 鍝嶅簲
     */
    FString HandleSimPause(UWorld* World);

    /**
     * @brief 鎭㈠浠跨湡
     * @param World 褰撳墠 World
     * @return JSON 鍝嶅簲
     */
    FString HandleSimResume(UWorld* World);

    /**
     * @brief 閲嶇疆浠跨湡
     * @param World 褰撳墠 World
     * @return JSON 鍝嶅簲
     */
    FString HandleSimReset(UWorld* World);
    FString HandleSimGetTime(UWorld* World);
    FString HandleSimSetTimeScale(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World);
    FString HandleSimStep(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World);

    /**
     * @brief 鑾峰彇宸叉敞鍐屾櫤鑳戒綋鍒楄〃
     * @return JSON 鍝嶅簲锛屽寘鍚?agents 鏁扮粍鍜?count 瀛楁
     */
    FString HandleGetAgentList();

    FString HandleGetCommandStatus(const TSharedPtr<FJsonObject>& JsonObject);
    FString HandleCancelCommand(const TSharedPtr<FJsonObject>& JsonObject);
    FString HandleGetSensorData(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World);
    FString HandleRecorderStart(const TSharedPtr<FJsonObject>& JsonObject);
    FString HandleRecorderStop();
    FString HandleRecorderStatus();
    FString HandleRecorderRecordState(UWorld* World);

    /**
     * @brief 鑾峰彇鎽勫儚澶村浘鍍忥紙鏀寔 Turret 鍜?Drone锛?
     * @param JsonObject 宸茶В鏋愮殑 JSON 瀵硅薄锛屽彲鍖呭惈 get_image.id 瀛楁鎸囧畾 Agent
     * @param World 褰撳墠 World
     * @return JSON 鍝嶅簲锛屽寘鍚?data(Base64)銆乧amera_pos銆乧amera_rot銆乫ov 绛夊瓧娈?
     */
    FString HandleGetImage(const TSharedPtr<FJsonObject>& JsonObject, UWorld* World);

    /**
     * @brief 鏋勯€犻敊璇搷搴?JSON
     * @param Error 閿欒淇℃伅
     * @return {"status":"error","message":"..."}
     */
    FString MakeErrorResponse(const FString& Error);

    /**
     * @brief 鏋勯€犳垚鍔熷搷搴?JSON
     * @param Message 鎴愬姛淇℃伅锛岄粯璁?"ok"
     * @return {"status":"ok","message":"..."}
     */
    FString MakeOkResponse(const FString& Message = TEXT("ok"));

    /** @brief 鏂扮殑閫氱敤鍛戒护鎵ц鍣?*/
    TUniquePtr<FCommandHandle> CommandHandle;

    /** @brief 鏃犱汉鏈哄懡浠ゅ鐞嗗櫒 */
    UPROPERTY()
    UDroneCommandHandler* DroneHandler = nullptr;

    /** @brief 杞彴鍛戒护澶勭悊鍣?*/
    UPROPERTY()
    UTurretCommandHandler* TurretHandler = nullptr;

    /** @brief 鍒跺鍛戒护澶勭悊鍣?*/
    UPROPERTY()
    UGuidanceCommandHandler* GuidanceHandler = nullptr;
};











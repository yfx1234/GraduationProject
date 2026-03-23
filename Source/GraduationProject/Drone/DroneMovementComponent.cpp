// 鐟欙綁鍣撮敍姘穿閸忋儱缍嬮崜宥呯杽閻滅増鏋冩禒璺侯嚠鎼存梻娈戞径瀛樻瀮娴?`DroneMovementComponent.h`閿涘奔濞囩€圭偟骞囬柈銊ュ瀻閼宠棄顧勯惇瀣煂缁鎷伴崙鑺ユ殶婢圭増妲戦妴?
#include "DroneMovementComponent.h"
// 鐟欙綁鍣撮敍姘穿閸?`PDController.h`閿涘奔璐熻ぐ鎾冲閺傚洣娆㈢悰銉ュ帠閹碘偓娓氭繆绂嗛惃鍕閸ㄥ鈧礁鍤遍弫鐗堝灗閹恒儱褰涙竟鐗堟閵?
#include "GraduationProject/Core/Controller/PDController.h"
// 鐟欙綁鍣撮敍姘穿閸?`PIDController.h`閿涘奔璐熻ぐ鎾冲閺傚洣娆㈢悰銉ュ帠閹碘偓娓氭繆绂嗛惃鍕閸ㄥ鈧礁鍤遍弫鐗堝灗閹恒儱褰涙竟鐗堟閵?
#include "GraduationProject/Core/Controller/PIDController.h"


/** @brief 閺嬪嫰鈧姴鍤遍弫?*/
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘矁鎯ょ€圭偛缍嬮崜宥喣侀崸妞捐厬閻ㄥ嫬鍙挎担鎾崇杽閻滄壆绮忛懞鍌︾礉娑撹桨绗傞棃銏㈡畱婢圭増妲戦妴浣稿彆瀵繑鍨ㄩ幒褍鍩楀ù浣衡柤閹绘劒绶电€圭偤妾幍褑顢戠拠顓炲綖閵?
UDroneMovementComponent::UDroneMovementComponent()
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
{
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `PrimaryComponentTick.bCanEverTick`閿涘苯鐣幋?鐢啫鐨甸弽鍥х箶 canevertick 閻ㄥ嫭娲块弬鑸偓?
    PrimaryComponentTick.bCanEverTick = true;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `PrimaryComponentTick.TickGroup`閿涘苯鐣幋?tickgroup 閻ㄥ嫭娲块弬鑸偓?
    PrimaryComponentTick.TickGroup = TG_PrePhysics; 
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `CurrentControlMode`閿涘苯鐣幋?currentcontrol濡€崇础 閻ㄥ嫭娲块弬鑸偓?
    CurrentControlMode = EDroneControlMode::Idle;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `ControlCommands`閿涘苯鐣幋?controlcommands 閻ㄥ嫭娲块弬鑸偓?
    ControlCommands = {0.0, 0.0, 0.0, 0.0};
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `TargetPosition`閿涘苯鐣幋?targetposition 閻ㄥ嫭娲块弬鑸偓?
    TargetPosition = FVector::ZeroVector;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `TargetVelocity`閿涘苯鐣幋?targetvelocity 閻ㄥ嫭娲块弬鑸偓?
    TargetVelocity = FVector::ZeroVector;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `TargetAttitude`閿涘苯鐣幋?targetattitude 閻ㄥ嫭娲块弬鑸偓?
    TargetAttitude = FRotator::ZeroRotator;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `TargetThrust`閿涘苯鐣幋?targetthrust 閻ㄥ嫭娲块弬鑸偓?
    TargetThrust = 0.0;
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
}

/**
 * @brief 缂佸嫪娆㈤崚婵嗩潗閸?
 * 閸掓稑缂?12 娑?PD/PID 閹貉冨煑閸ｃ劌鐤勬笟瀣剁礉鐠侊紕鐣婚幒褍鍩楅崚鍡涘帳閻晠妯€閸欏﹤鍙鹃柅鍡欑叐闂冪偣鈧?
 */
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯鐣炬稊澶婂毐閺?`BeginPlay`閿涘苯绱戞慨瀣杽閻滅櫚eginplay閻ㄥ嫬鍙挎担鎾烩偓鏄忕帆閵?
void UDroneMovementComponent::BeginPlay()
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
{
    // 鐟欙綁鍣撮敍姘崇殶閻?`BeginPlay` 閹笛嗩攽瑜版挸澧犲銉╊€冮棁鈧憰浣烘畱閸旂喕鍏橀柅鏄忕帆閵?
    Super::BeginPlay();
    // 鐟欙綁鍣撮敍姘崇殶閻?`InitializeComputed` 閹笛嗩攽瑜版挸澧犲銉╊€冮棁鈧憰浣烘畱閸旂喕鍏橀柅鏄忕帆閵?
    Parameters.InitializeComputed();
    // 鐟欙綁鍣撮敍姘崇殶閻?`InitializeControllers` 閹笛嗩攽瑜版挸澧犲銉╊€冮棁鈧憰浣烘畱閸旂喕鍏橀柅鏄忕帆閵?
    InitializeControllers();
    // 鐟欙綁鍣撮敍姘崇殶閻?`ComputeControlAllocation` 閹笛嗩攽瑜版挸澧犲銉╊€冮棁鈧憰浣烘畱閸旂喕鍏橀柅鏄忕帆閵?
    ComputeControlAllocation();
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `bInitialized`閿涘苯鐣幋?鐢啫鐨甸弽鍥х箶 initialized 閻ㄥ嫭娲块弬鑸偓?
    bInitialized = true;
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
}

/**
 * @brief 濮ｅ繐鎶?Tick 閹貉冨煑閺囧瓨鏌?
 * @param DeltaTime 鐢囨？闂呮梹妞傞梻?
 */
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯鐣炬稊澶婂毐閺?`TickComponent`閿涘苯绱戞慨瀣杽閻滅殞ick缂佸嫪娆㈤惃鍕徔娴ｆ捇鈧槒绶妴?
void UDroneMovementComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
{
    // 鐟欙綁鍣撮敍姘崇殶閻?`TickComponent` 閹笛嗩攽瑜版挸澧犲銉╊€冮棁鈧憰浣烘畱閸旂喕鍏橀柅鏄忕帆閵?
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼绮伴崙?`if` 閺夆€叉閸掋倖鏌囬敍灞藉涧閺堝娼禒鑸靛灇缁斿妞傞幍宥勭窗鏉╂稑鍙嗘稉瀣桨閻ㄥ嫬鍨庨弨顖炩偓鏄忕帆閵?
    if (!bInitialized) return;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼绮伴崙?`if` 閺夆€叉閸掋倖鏌囬敍灞藉涧閺堝娼禒鑸靛灇缁斿妞傞幍宥勭窗鏉╂稑鍙嗘稉瀣桨閻ㄥ嫬鍨庨弨顖炩偓鏄忕帆閵?
    if (CurrentControlMode == EDroneControlMode::Idle) return;

    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘矂鈧俺绻?`FMath::Max` 缂?`const double FixedStep` 閺傝棄濮炴稉瀣櫕缁撅附娼敍宀勪缉閸?constdoublefixedstep 鏉╁洤鐨妴?
    const double FixedStep = FMath::Max(0.0005, static_cast<double>(Parameters.TimeStep));
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘矂鈧俺绻?`FMath::Max` 缂?`const int32 MaxSubSteps` 閺傝棄濮炴稉瀣櫕缁撅附娼敍宀勪缉閸?constint32maxsubsteps 鏉╁洤鐨妴?
    const int32 MaxSubSteps = FMath::Max(1, MaxSubStepsPerTick);

    // 閸ュ搫鐣惧銉╂毐缁夘垰鍨庨敍姘辩柈鐠侊紕婀＄€圭偛鎶氶弮鍫曟？閿涘本瀵?TimeStep 缁傜粯鏆庨幒銊ㄧ箻閵?
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯鍘涚€电顓哥粻妤冪波閺嬫粌浠涢梽鎰畽閿涘苯鍟€閸愭瑥鍙?`FixedStepAccumulator`閿涘矂妲诲?fixedstepaccumulator 鐡掑懎鍤崗浣筋啅閼煎啫娲块妴?
    FixedStepAccumulator = FMath::Clamp(FixedStepAccumulator + static_cast<double>(DeltaTime), 0.0, FixedStep * MaxSubSteps);

    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯锛愰弰搴㈠灇閸涙ɑ鍨ㄧ仦鈧柈銊ュ綁闁?`BackupState`閿涘瞼鏁ゆ禍搴濈箽鐎涙ackup閻樿埖鈧降鈧?
    FDroneState BackupState = CurrentState;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯锛愰弰搴㈠灇閸涙ɑ鍨ㄧ仦鈧柈銊ュ綁闁?`ExecutedSteps`閿涘瞼鏁ゆ禍搴濈箽鐎涙xecutedsteps閵?
    int32 ExecutedSteps = 0;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯绱戞慨?`while` 瀵邦亞骞嗛敍灞藉涧鐟曚焦娼禒鏈电箽閹镐椒璐熼惇鐔锋皑閹镐胶鐢婚柌宥咁槻閹笛嗩攽閵?
    while (FixedStepAccumulator >= FixedStep && ExecutedSteps < MaxSubSteps)
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
    {
        // 閹貉冨煑閺囧瓨鏌婇敍鍫熺槨娑擃亜鐡欏銉╁厴鏉╂劘顢戦敍灞肩瑢閻椻晝鎮婇崥宀勵暥閿?
        // 鐟欙綁鍣撮敍姘崇殶閻?`ControlUpdate` 閹笛嗩攽瑜版挸澧犲銉╊€冮棁鈧憰浣烘畱閸旂喕鍏橀柅鏄忕帆閵?
        ControlUpdate(FixedStep);

        // 閻椻晝鎮婇弴瀛樻煀
        // 鐟欙綁鍣撮敍姘崇殶閻?`VerletUpdate` 閹笛嗩攽瑜版挸澧犲銉╊€冮棁鈧憰浣烘畱閸旂喕鍏橀柅鏄忕帆閵?
        VerletUpdate(FixedStep);
        // 鐟欙綁鍣撮敍姘崇殶閻?`CheckGroundCollision` 閹笛嗩攽瑜版挸澧犲銉╊€冮棁鈧憰浣烘畱閸旂喕鍏橀柅鏄忕帆閵?
        CheckGroundCollision(InitialGroundZ);

        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘奔绮?`FixedStepAccumulator` 娑擃厼鍣洪崢缁樻煀闁插骏绱濋悽銊ょ艾娣囶喗顒滈幋鏍ㄥХ濞?fixedstepaccumulator閵?
        FixedStepAccumulator -= FixedStep;
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘矁鎯ょ€圭偛缍嬮崜宥喣侀崸妞捐厬閻ㄥ嫬鍙挎担鎾崇杽閻滄壆绮忛懞鍌︾礉娑撹桨绗傞棃銏㈡畱婢圭増妲戦妴浣稿彆瀵繑鍨ㄩ幒褍鍩楀ù浣衡柤閹绘劒绶电€圭偤妾幍褑顢戠拠顓炲綖閵?
        ++ExecutedSteps;

        // NaN 娣囨繃濮?
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼绮伴崙?`if` 閺夆€叉閸掋倖鏌囬敍灞藉涧閺堝娼禒鑸靛灇缁斿妞傞幍宥勭窗鏉╂稑鍙嗘稉瀣桨閻ㄥ嫬鍨庨弨顖炩偓鏄忕帆閵?
        if (CurrentState.HasNaN())
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
        {
            // 鐟欙綁鍣撮敍姘崇殶閻?`UE_LOG` 鏉堟挸鍤拫鍐槸閺冦儱绻旈敍灞肩┒娴滃氦绻嶇悰灞炬鐟欏倸鐧傞悩鑸碘偓浣告嫲閹烘帗鐓￠梻顕€顣介妴?
            UE_LOG(LogTemp, Warning, TEXT("[DroneMovement] NaN detected, reverting to backup state"));
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `CurrentState`閿涘苯鐣幋?current閻樿埖鈧?閻ㄥ嫭娲块弬鑸偓?
            CurrentState = BackupState;
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `CurrentState.AngRollRate`閿涘苯鐣幋?angrollrate 閻ㄥ嫭娲块弬鑸偓?
            CurrentState.AngRollRate = 0.0;
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `CurrentState.AngPitchRate`閿涘苯鐣幋?angpitchrate 閻ㄥ嫭娲块弬鑸偓?
            CurrentState.AngPitchRate = 0.0;
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `CurrentState.AngYawRate`閿涘苯鐣幋?angyawrate 閻ㄥ嫭娲块弬鑸偓?
            CurrentState.AngYawRate = 0.0;
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `PrevLinearAcceleration`閿涘苯鐣幋?prevlinearacceleration 閻ㄥ嫭娲块弬鑸偓?
            PrevLinearAcceleration = FVector::ZeroVector;
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `PrevAngularAcceleration`閿涘苯鐣幋?prevangularacceleration 閻ㄥ嫭娲块弬鑸偓?
            PrevAngularAcceleration = FVector::ZeroVector;
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `FixedStepAccumulator`閿涘苯鐣幋?fixedstepaccumulator 閻ㄥ嫭娲块弬鑸偓?
            FixedStepAccumulator = 0.0;
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鐝涢崡瀹犵儲閸戝搫缍嬮崜宥呮儕閻滎垱鍨?`switch` 閸掑棙鏁敍宀勪缉閸忓秶鎴风紒顓熷⒔鐞涘苯鎮楃紒顓炲瀻閺€顖樷偓?
            break;
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
        }
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
    }
    // 闁喎瀹崇憗浣稿闂冩彃褰傞弫?
    // 鐟欙綁鍣撮敍姘崇殶閻?`ClampVelocities` 閹笛嗩攽瑜版挸澧犲銉╊€冮棁鈧憰浣烘畱閸旂喕鍏橀柅鏄忕帆閵?
    CurrentState.ClampVelocities(50.0, 50.0);
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
}

/**
 * @brief 鐠佸墽鐤嗛悧鈺冩倞閸欏倹鏆熼獮鍫曞櫢閺傛澘鍨垫慨瀣
 * @param NewParameters 閺傛壆娈戦悧鈺冩倞閸欏倹鏆?
 * 閺囧瓨鏌婇崣鍌涙殶閸氬酣鍣搁弬鎷岊吀缁犳甯堕崚璺哄瀻闁板秶鐓╅梼闈涙嫲闁插秵鏌婇崚婵嗩潗閸栨牗甯堕崚璺烘珤
 */
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯鐣炬稊澶婂毐閺?`SetParameters`閿涘苯绱戞慨瀣杽閻滅殜et閸欏倹鏆熼惃鍕徔娴ｆ捇鈧槒绶妴?
void UDroneMovementComponent::SetParameters(const FDroneParameters& NewParameters)
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
{
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `Parameters`閿涘苯鐣幋?閸欏倹鏆?閻ㄥ嫭娲块弬鑸偓?
    Parameters = NewParameters;
    // 鐟欙綁鍣撮敍姘崇殶閻?`InitializeComputed` 閹笛嗩攽瑜版挸澧犲銉╊€冮棁鈧憰浣烘畱閸旂喕鍏橀柅鏄忕帆閵?
    Parameters.InitializeComputed();
    // 鐟欙綁鍣撮敍姘崇殶閻?`ComputeControlAllocation` 閹笛嗩攽瑜版挸澧犲銉╊€冮棁鈧憰浣烘畱閸旂喕鍏橀柅鏄忕帆閵?
    ComputeControlAllocation();
    // 鐟欙綁鍣撮敍姘崇殶閻?`InitializeControllers` 閹笛嗩攽瑜版挸澧犲銉╊€冮棁鈧憰浣烘畱閸旂喕鍏橀柅鏄忕帆閵?
    InitializeControllers();
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `FixedStepAccumulator`閿涘苯鐣幋?fixedstepaccumulator 閻ㄥ嫭娲块弬鑸偓?
    FixedStepAccumulator = 0.0;
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
}

/**
 * @brief 閸掓稑缂撻獮璺哄灥婵瀵查幍鈧張?PD/PID 閹貉冨煑閸?
 * 娴ｅ秶鐤嗛悳?PD)閿涙p=1.0, Kd=0.3, 鏉堟挸鍤梽鎰畽 2.0 m/s
 * 闁喎瀹抽悳?PID)閿涙p=2.0, Ki=0.1, Kd=0.1, 鏉堟挸鍤梽鎰畽 3.0 m/s铏?
 * 婵寧鈧胶骞?PD)閿涙p=6.0, Kd=0.3閿涘湻aw 閸戝繐宕愰敍澶涚礉鏉堟挸鍤梽鎰畽 3.0 rad/s
 * 鐟欐帡鈧喓宸奸悳?PID)閿涙p=0.5, Ki=0.0, Kd=0.0閿涘矁绶崙娲楠?1.0 N璺痬
 */
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯鐣炬稊澶婂毐閺?`InitializeControllers`閿涘苯绱戞慨瀣杽閻滅櫡nitializecontrollers閻ㄥ嫬鍙挎担鎾烩偓鏄忕帆閵?
void UDroneMovementComponent::InitializeControllers()
{
    const double Ts = Parameters.TimeStep;

    // 位置环先保守一些，避免速度指令过大。
    // Calm hover tuning.
    const float PosPGainXY = 0.45f;
    const float PosDGainXY = 0.02f;
    const float PosPGainZ = 1.80f;
    const float PosDGainZ = 0.00f;

    PositionAxisSpeedLimit = FMath::Min(DefaultPositionSpeedLimit, 1.0);
    TargetPositionSpeedLimit = PositionAxisSpeedLimit;
    const float VelMaxLimitXY = static_cast<float>(PositionAxisSpeedLimit);
    const float VelMaxLimitZ = 1.2f;

    PxController = NewObject<UPDController>(this);
    PxController->Initialize(PosPGainXY, PosDGainXY, 0.12, VelMaxLimitXY, Ts);
    PyController = NewObject<UPDController>(this);
    PyController->Initialize(PosPGainXY, PosDGainXY, 0.12, VelMaxLimitXY, Ts);
    PzController = NewObject<UPDController>(this);
    PzController->Initialize(PosPGainZ, PosDGainZ, 0.12, VelMaxLimitZ, Ts);

    // 速度环去掉积分和微分，先把悬停稳定下来。
    // Velocity loop uses conservative gains.
    const float VelPGainXY = 1.05f;
    const float VelPGainZ = 5.20f;
    const float VelOutputLimitXY = 3.2f;
    const float VelOutputLimitZ = 6.2f;

    VxController = NewObject<UPIDController>(this);
    VxController->Initialize(VelPGainXY, 0.0, 0.0, 0.10, VelOutputLimitXY, Ts, -0.8, 0.8);
    VyController = NewObject<UPIDController>(this);
    VyController->Initialize(VelPGainXY, 0.0, 0.0, 0.10, VelOutputLimitXY, Ts, -0.8, 0.8);
    VzController = NewObject<UPIDController>(this);
    VzController->Initialize(VelPGainZ, 0.0, 0.0, 0.10, VelOutputLimitZ, Ts, -0.8, 0.8);

    // 姿态环和角速度环靠近 UESIM 的温和参数。
    // Attitude loop stays close to UESIM.
    const float AnglePGainXY = 2.90f;
    const float AngleDGainXY = 0.08f;
    const float AnglePGainYaw = 3.20f;
    const float AngleDGainYaw = 0.05f;

    RollController = NewObject<UPDController>(this);
    RollController->Initialize(AnglePGainXY, AngleDGainXY, 0.10, PI / 3.0, Ts);
    PitchController = NewObject<UPDController>(this);
    PitchController->Initialize(AnglePGainXY, AngleDGainXY, 0.10, PI / 3.0, Ts);
    YawController = NewObject<UPDController>(this);
    YawController->Initialize(AnglePGainYaw, AngleDGainYaw, 0.10, PI / 2.0, Ts);

    const float RatePGainXY = 0.14f;
    const float RatePGainYaw = 0.18f;

    RollRateController = NewObject<UPIDController>(this);
    RollRateController->Initialize(RatePGainXY, 0.0, 0.0, 0.08, 0.32, Ts, -0.5, 0.5);
    PitchRateController = NewObject<UPIDController>(this);
    PitchRateController->Initialize(RatePGainXY, 0.0, 0.0, 0.08, 0.32, Ts, -0.5, 0.5);
    YawRateController = NewObject<UPIDController>(this);
    YawRateController->Initialize(RatePGainYaw, 0.0, 0.0, 0.08, 0.22, Ts, -0.4, 0.4);
}

/**
 * @brief 鐠侊紕鐣婚幒褍鍩楅崚鍡涘帳閻晠妯€ G 閸欏﹤鍙鹃柅鍡欑叐闂?GInv
 *  閹粯甯归崝?T = F1 + F2 + F3 + F4 = kT * 锠?铏?+ kT * 锠?铏?+ kT * 锠?铏?+ kT * 锠?铏?
 *  濠婃俺娴嗛崝娑氱叐 锜縳 = F1 * L * sin灏?- F2 * L * sin灏?- F3 * L * sin灏?+ F4 * L * sin灏?
 *  娣囶垯璇濋崝娑氱叐 锜縴 = -F1 * L * cos灏?- F2 * L * cos灏?+ F3 * L * cos灏?+ F4 * L * cos灏?
 *  閸嬪繗鍩呴崝娑氱叐 锜縵 = 锠?铏?* kQ - 锠?铏?* kQ + 锠?铏?* kQ - 锠?铏?* kQ
 *  閹貉冨煑閸掑棝鍘ら惌鈺呮█ [T, 锜縳, 锜縴, 锜縵]^T = G * [锠?铏? 锠?铏? 锠?铏? 锠?铏廬^T
 */
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯鐣炬稊澶婂毐閺?`ComputeControlAllocation`閿涘苯绱戞慨瀣杽閻滅櫛omputecontrolallocation閻ㄥ嫬鍙挎担鎾烩偓鏄忕帆閵?
void UDroneMovementComponent::ComputeControlAllocation()
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
{
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯锛愰弰搴㈠灇閸涙ɑ鍨ㄧ仦鈧柈銊ュ綁闁?`kT`閿涘瞼鏁ゆ禍搴濈箽鐎涙グT閵?
    double kT = Parameters.ThrustCoefficient;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯锛愰弰搴㈠灇閸涙ɑ鍨ㄧ仦鈧柈銊ュ綁闁?`kQ`閿涘瞼鏁ゆ禍搴濈箽鐎涙グQ閵?
    double kQ = Parameters.TorqueCoefficient;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯锛愰弰搴㈠灇閸涙ɑ鍨ㄧ仦鈧柈銊ュ綁闁?`L`閿涘瞼鏁ゆ禍搴濈箽鐎涙ゲ閵?
    double L = Parameters.ArmLength;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `double Beta`閿涘苯鐣幋?doublebeta 閻ㄥ嫭娲块弬鑸偓?
    double Beta = FMath::DegreesToRadians(Parameters.MotorAngle); // 閻㈠灚婧€閼峰倽顫楁惔?
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `double SinB`閿涘苯鐣幋?doublesinB 閻ㄥ嫭娲块弬鑸偓?
    double SinB = FMath::Sin(Beta);
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `double CosB`閿涘苯鐣幋?doublecosB 閻ㄥ嫭娲块弬鑸偓?
    double CosB = FMath::Cos(Beta);
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `G[0][0]`閿涘苯鐣幋?G 閻ㄥ嫭娲块弬鑸偓?
    G[0][0] = kT; 
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `G[0][1]`閿涘苯鐣幋?G 閻ㄥ嫭娲块弬鑸偓?
    G[0][1] = kT; 
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `G[0][2]`閿涘苯鐣幋?G 閻ㄥ嫭娲块弬鑸偓?
    G[0][2] = kT; 
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `G[0][3]`閿涘苯鐣幋?G 閻ㄥ嫭娲块弬鑸偓?
    G[0][3] = kT;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `G[1][0]`閿涘苯鐣幋?G 閻ㄥ嫭娲块弬鑸偓?
    G[1][0] = kT*L*SinB; 
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `G[1][1]`閿涘苯鐣幋?G 閻ㄥ嫭娲块弬鑸偓?
    G[1][1] = -kT*L*SinB; 
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `G[1][2]`閿涘苯鐣幋?G 閻ㄥ嫭娲块弬鑸偓?
    G[1][2] = -kT*L*SinB; 
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `G[1][3]`閿涘苯鐣幋?G 閻ㄥ嫭娲块弬鑸偓?
    G[1][3] = kT*L*SinB;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `G[2][0]`閿涘苯鐣幋?G 閻ㄥ嫭娲块弬鑸偓?
    G[2][0] = kT*L*CosB; 
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `G[2][1]`閿涘苯鐣幋?G 閻ㄥ嫭娲块弬鑸偓?
    G[2][1] = kT*L*CosB; 
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `G[2][2]`閿涘苯鐣幋?G 閻ㄥ嫭娲块弬鑸偓?
    G[2][2] = -kT*L*CosB; 
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `G[2][3]`閿涘苯鐣幋?G 閻ㄥ嫭娲块弬鑸偓?
    G[2][3] = -kT*L*CosB;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `G[3][0]`閿涘苯鐣幋?G 閻ㄥ嫭娲块弬鑸偓?
    G[3][0] = kQ; 
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `G[3][1]`閿涘苯鐣幋?G 閻ㄥ嫭娲块弬鑸偓?
    G[3][1] = -kQ; 
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `G[3][2]`閿涘苯鐣幋?G 閻ㄥ嫭娲块弬鑸偓?
    G[3][2] = kQ; 
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `G[3][3]`閿涘苯鐣幋?G 閻ㄥ嫭娲块弬鑸偓?
    G[3][3] = -kQ;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `double a`閿涘苯鐣幋?doubleA 閻ㄥ嫭娲块弬鑸偓?
    double a = 1.0 / (4.0 * kT);             
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `double b`閿涘苯鐣幋?doubleB 閻ㄥ嫭娲块弬鑸偓?
    double b = 1.0 / (4.0 * kT * L * SinB);  
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `double c`閿涘苯鐣幋?doubleC 閻ㄥ嫭娲块弬鑸偓?
    double c = 1.0 / (4.0 * kT * L * CosB);  
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `double d`閿涘苯鐣幋?doubleD 閻ㄥ嫭娲块弬鑸偓?
    double d = 1.0 / (4.0 * kQ);             
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `GInv[0][0]`閿涘苯鐣幋?ginv 閻ㄥ嫭娲块弬鑸偓?
    GInv[0][0] = a; GInv[0][1] = b;  GInv[0][2] = c;  GInv[0][3] = d;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `GInv[1][0]`閿涘苯鐣幋?ginv 閻ㄥ嫭娲块弬鑸偓?
    GInv[1][0] = a; GInv[1][1] = -b; GInv[1][2] = c;  GInv[1][3] = -d;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `GInv[2][0]`閿涘苯鐣幋?ginv 閻ㄥ嫭娲块弬鑸偓?
    GInv[2][0] = a; GInv[2][1] = -b; GInv[2][2] = -c; GInv[2][3] = d;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `GInv[3][0]`閿涘苯鐣幋?ginv 閻ㄥ嫭娲块弬鑸偓?
    GInv[3][0] = a; GInv[3][1] = b;  GInv[3][2] = -c; GInv[3][3] = -d;
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
}

// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯鐣炬稊澶婂毐閺?`SyncControllerTimeSteps`閿涘苯绱戞慨瀣杽閻滅殜ync閹貉冨煑閸ｂ暟imesteps閻ㄥ嫬鍙挎担鎾烩偓鏄忕帆閵?
void UDroneMovementComponent::SyncControllerTimeSteps(double ControlTimeStep)
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
{
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼绮伴崙?`if` 閺夆€叉閸掋倖鏌囬敍灞藉涧閺堝娼禒鑸靛灇缁斿妞傞幍宥勭窗鏉╂稑鍙嗘稉瀣桨閻ㄥ嫬鍨庨弨顖炩偓鏄忕帆閵?
    if (PxController) PxController->SetTimeStep(ControlTimeStep);
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼绮伴崙?`if` 閺夆€叉閸掋倖鏌囬敍灞藉涧閺堝娼禒鑸靛灇缁斿妞傞幍宥勭窗鏉╂稑鍙嗘稉瀣桨閻ㄥ嫬鍨庨弨顖炩偓鏄忕帆閵?
    if (PyController) PyController->SetTimeStep(ControlTimeStep);
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼绮伴崙?`if` 閺夆€叉閸掋倖鏌囬敍灞藉涧閺堝娼禒鑸靛灇缁斿妞傞幍宥勭窗鏉╂稑鍙嗘稉瀣桨閻ㄥ嫬鍨庨弨顖炩偓鏄忕帆閵?
    if (PzController) PzController->SetTimeStep(ControlTimeStep);
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼绮伴崙?`if` 閺夆€叉閸掋倖鏌囬敍灞藉涧閺堝娼禒鑸靛灇缁斿妞傞幍宥勭窗鏉╂稑鍙嗘稉瀣桨閻ㄥ嫬鍨庨弨顖炩偓鏄忕帆閵?
    if (VxController) VxController->SetTimeStep(ControlTimeStep);
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼绮伴崙?`if` 閺夆€叉閸掋倖鏌囬敍灞藉涧閺堝娼禒鑸靛灇缁斿妞傞幍宥勭窗鏉╂稑鍙嗘稉瀣桨閻ㄥ嫬鍨庨弨顖炩偓鏄忕帆閵?
    if (VyController) VyController->SetTimeStep(ControlTimeStep);
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼绮伴崙?`if` 閺夆€叉閸掋倖鏌囬敍灞藉涧閺堝娼禒鑸靛灇缁斿妞傞幍宥勭窗鏉╂稑鍙嗘稉瀣桨閻ㄥ嫬鍨庨弨顖炩偓鏄忕帆閵?
    if (VzController) VzController->SetTimeStep(ControlTimeStep);
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼绮伴崙?`if` 閺夆€叉閸掋倖鏌囬敍灞藉涧閺堝娼禒鑸靛灇缁斿妞傞幍宥勭窗鏉╂稑鍙嗘稉瀣桨閻ㄥ嫬鍨庨弨顖炩偓鏄忕帆閵?
    if (RollController) RollController->SetTimeStep(ControlTimeStep);
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼绮伴崙?`if` 閺夆€叉閸掋倖鏌囬敍灞藉涧閺堝娼禒鑸靛灇缁斿妞傞幍宥勭窗鏉╂稑鍙嗘稉瀣桨閻ㄥ嫬鍨庨弨顖炩偓鏄忕帆閵?
    if (PitchController) PitchController->SetTimeStep(ControlTimeStep);
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼绮伴崙?`if` 閺夆€叉閸掋倖鏌囬敍灞藉涧閺堝娼禒鑸靛灇缁斿妞傞幍宥勭窗鏉╂稑鍙嗘稉瀣桨閻ㄥ嫬鍨庨弨顖炩偓鏄忕帆閵?
    if (YawController) YawController->SetTimeStep(ControlTimeStep);
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼绮伴崙?`if` 閺夆€叉閸掋倖鏌囬敍灞藉涧閺堝娼禒鑸靛灇缁斿妞傞幍宥勭窗鏉╂稑鍙嗘稉瀣桨閻ㄥ嫬鍨庨弨顖炩偓鏄忕帆閵?
    if (RollRateController) RollRateController->SetTimeStep(ControlTimeStep);
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼绮伴崙?`if` 閺夆€叉閸掋倖鏌囬敍灞藉涧閺堝娼禒鑸靛灇缁斿妞傞幍宥勭窗鏉╂稑鍙嗘稉瀣桨閻ㄥ嫬鍨庨弨顖炩偓鏄忕帆閵?
    if (PitchRateController) PitchRateController->SetTimeStep(ControlTimeStep);
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼绮伴崙?`if` 閺夆€叉閸掋倖鏌囬敍灞藉涧閺堝娼禒鑸靛灇缁斿妞傞幍宥勭窗鏉╂稑鍙嗘稉瀣桨閻ㄥ嫬鍨庨弨顖炩偓鏄忕帆閵?
    if (YawRateController) YawRateController->SetTimeStep(ControlTimeStep);
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
}

// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯鐣炬稊澶婂毐閺?`UpdateDesiredYawFromPlanarVector`閿涘苯绱戞慨瀣杽閻滅殟pdatedesiredyawfromplanarvector閻ㄥ嫬鍙挎担鎾烩偓鏄忕帆閵?
void UDroneMovementComponent::UpdateDesiredYawFromPlanarVector(double DirX, double DirY, double CurrentYaw)
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
{
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `const double DirectionMagnitude`閿涘苯鐣幋?constdoubledirectionmagnitude 閻ㄥ嫭娲块弬鑸偓?
    const double DirectionMagnitude = FMath::Sqrt(DirX * DirX + DirY * DirY);

    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼绮伴崙?`if` 閺夆€叉閸掋倖鏌囬敍灞藉涧閺堝娼禒鑸靛灇缁斿妞傞幍宥勭窗鏉╂稑鍙嗘稉瀣桨閻ㄥ嫬鍨庨弨顖炩偓鏄忕帆閵?
    if (YawMode == EDroneYawMode::Angle)
    {
        DesiredYaw = CommandedYaw;
        LockedYaw = DesiredYaw;
        bYawInitialized = true;
        return;
    }

    if (YawMode == EDroneYawMode::Rate)
    {
        LockedYaw = CurrentYaw;
        DesiredYaw = CurrentYaw;
        bYawInitialized = true;
        return;
    }

    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼绮伴崙?`if` 閺夆€叉閸掋倖鏌囬敍灞藉涧閺堝娼禒鑸靛灇缁斿妞傞幍宥勭窗鏉╂稑鍙嗘稉瀣桨閻ㄥ嫬鍨庨弨顖炩偓鏄忕帆閵?
    if (YawMode == EDroneYawMode::Hold || DrivetrainMode == EDroneDrivetrainMode::MaxDegreeOfFreedom)
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
    {
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼绮伴崙?`if` 閺夆€叉閸掋倖鏌囬敍灞藉涧閺堝娼禒鑸靛灇缁斿妞傞幍宥勭窗鏉╂稑鍙嗘稉瀣桨閻ㄥ嫬鍨庨弨顖炩偓鏄忕帆閵?
        if (!bYawInitialized)
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
        {
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `LockedYaw`閿涘苯鐣幋?lockedyaw 閻ㄥ嫭娲块弬鑸偓?
            LockedYaw = CurrentYaw;
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `bYawInitialized`閿涘苯鐣幋?鐢啫鐨甸弽鍥х箶 yawinitialized 閻ㄥ嫭娲块弬鑸偓?
            bYawInitialized = true;
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
        }
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `DesiredYaw`閿涘苯鐣幋?desiredyaw 閻ㄥ嫭娲块弬鑸偓?
        DesiredYaw = LockedYaw;
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘矁绻戦崶鐐茬秼閸撳秴鍤遍弫鎵畱鐠侊紕鐣荤紒鎾寸亯閿涘本濡搁幒褍鍩楅弶鍐ф唉閸ョ偠鐨熼悽銊︽煙閵?
        return;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
    }

    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼绮伴崙?`if` 閺夆€叉閸掋倖鏌囬敍灞藉涧閺堝娼禒鑸靛灇缁斿妞傞幍宥勭窗鏉╂稑鍙嗘稉瀣桨閻ㄥ嫬鍨庨弨顖炩偓鏄忕帆閵?
    if (DirectionMagnitude > YawSpeedThreshold)
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
    {
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `DesiredYaw`閿涘苯鐣幋?desiredyaw 閻ㄥ嫭娲块弬鑸偓?
        DesiredYaw = FMath::Atan2(DirY, DirX) + FMath::DegreesToRadians(YawOffset);
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `LockedYaw`閿涘苯鐣幋?lockedyaw 閻ㄥ嫭娲块弬鑸偓?
        LockedYaw = DesiredYaw;
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `bYawInitialized`閿涘苯鐣幋?鐢啫鐨甸弽鍥х箶 yawinitialized 閻ㄥ嫭娲块弬鑸偓?
        bYawInitialized = true;
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘矁绻戦崶鐐茬秼閸撳秴鍤遍弫鎵畱鐠侊紕鐣荤紒鎾寸亯閿涘本濡搁幒褍鍩楅弶鍐ф唉閸ョ偠鐨熼悽銊︽煙閵?
        return;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
    }

    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `DesiredYaw`閿涘苯鐣幋?desiredyaw 閻ㄥ嫭娲块弬鑸偓?
    DesiredYaw = bYawInitialized ? LockedYaw : CurrentYaw;
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
}

/**
 * @brief 閹笛嗩攽娑撯偓鐢呮畱缁狙嗕粓閹貉冨煑閺囧瓨鏌?
 * @param DeltaTime 鐢囨？闂?
 * 閺嶈宓侀幒褍鍩楀Ο鈥崇础閺囧瓨鏌婇幒銊ュ閸旀稓鐓╅敍灞藉晙瀵版鍩岄悽鍨簚鏉烆剟鈧?
 */
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯鐣炬稊澶婂毐閺?`ControlUpdate`閿涘苯绱戞慨瀣杽閻滅櫛ontrolupdate閻ㄥ嫬鍙挎担鎾烩偓鏄忕帆閵?
void UDroneMovementComponent::ControlUpdate(double DeltaTime)
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
{
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `LastControlDeltaTime`閿涘苯鐣幋?lastcontroldeltatime 閻ㄥ嫭娲块弬鑸偓?
    LastControlDeltaTime = DeltaTime;  // 娣囨繂鐡ㄧ敮褎妞傞梻缈犵返閻㈠灚婧€濠娿倖灏濋崳銊ゅ▏閻?
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘矂鈧俺绻?`FMath::Max` 缂?`const double ControlTimeStep` 閺傝棄濮炴稉瀣櫕缁撅附娼敍宀勪缉閸?constdoublecontroltimestep 鏉╁洤鐨妴?
    const double ControlTimeStep = FMath::Max(0.001, static_cast<double>(DeltaTime));
    // 鐟欙綁鍣撮敍姘崇殶閻?`SyncControllerTimeSteps` 閹笛嗩攽瑜版挸澧犲銉╊€冮棁鈧憰浣烘畱閸旂喕鍏橀柅鏄忕帆閵?
    SyncControllerTimeSteps(ControlTimeStep);

    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯锛愰弰搴㈠灇閸涙ɑ鍨ㄧ仦鈧柈銊ュ綁闁?`TorqueCommand`閿涘瞼鏁ゆ禍搴濈箽鐎涙Οorque閸涙垝鎶ら妴?
    FVector TorqueCommand = FVector::ZeroVector;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯锛愰弰搴㈠灇閸涙ɑ鍨ㄧ仦鈧柈銊ュ綁闁?`ThrustCommand`閿涘瞼鏁ゆ禍搴濈箽鐎涙Οhrust閸涙垝鎶ら妴?
    double ThrustCommand = Parameters.Mass * Parameters.Gravity;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘矁绻橀崗?`switch` 閸掑棙鏁崚鍡樻烦缂佹挻鐎敍灞芥倵缂侇厺绱伴幐澶夌瑝閸氬本鐏囨稉鐐灗閻樿埖鈧礁鈧吋澧界悰灞肩瑝閸氬矂鈧槒绶妴?
    switch (CurrentControlMode)
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
    {
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯锛愰弰?`switch` 娑擃厾娈戞稉鈧稉顏勫瀻閺€顖涚垼缁涙拝绱濈€电懓绨查弻鎰嚋閸忚渹缍嬮崣鏍р偓鑲╂畱婢跺嫮鎮婄捄顖氱窞閵?
    case EDroneControlMode::MotorSpeed:
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼绮伴崙?`if` 閺夆€叉閸掋倖鏌囬敍灞藉涧閺堝娼禒鑸靛灇缁斿妞傞幍宥勭窗鏉╂稑鍙嗘稉瀣桨閻ㄥ嫬鍨庨弨顖炩偓鏄忕帆閵?
        if (ControlCommands.Num() >= 4) CurrentState.MotorSpeeds = ControlCommands;
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘矁绻戦崶鐐茬秼閸撳秴鍤遍弫鎵畱鐠侊紕鐣荤紒鎾寸亯閿涘本濡搁幒褍鍩楅弶鍐ф唉閸ョ偠鐨熼悽銊︽煙閵?
        return;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯锛愰弰?`switch` 娑擃厾娈戞稉鈧稉顏勫瀻閺€顖涚垼缁涙拝绱濈€电懓绨查弻鎰嚋閸忚渹缍嬮崣鏍р偓鑲╂畱婢跺嫮鎮婄捄顖氱窞閵?
    case EDroneControlMode::Position:
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
        {
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `const FVector Pos`閿涘苯鐣幋?constfvectorpos 閻ㄥ嫭娲块弬鑸偓?
            const FVector Pos = CurrentState.GetPosition();
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `const double CurrentYaw`閿涘苯鐣幋?constdoublecurrentyaw 閻ㄥ嫭娲块弬鑸偓?
            const double CurrentYaw = FMath::DegreesToRadians(CurrentState.GetRotator().Yaw);
            // 鐟欙綁鍣撮敍姘崇殶閻?`UpdateDesiredYawFromPlanarVector` 閹笛嗩攽瑜版挸澧犲銉╊€冮棁鈧憰浣烘畱閸旂喕鍏橀柅鏄忕帆閵?
            UpdateDesiredYawFromPlanarVector(TargetPosition.X - Pos.X, TargetPosition.Y - Pos.Y, CurrentYaw);

            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `FVector VelCmd`閿涘苯鐣幋?fvectorvelcmd 閻ㄥ嫭娲块弬鑸偓?
            FVector VelCmd = PositionLoop(TargetPosition);
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `FVector AccCmd`閿涘苯鐣幋?fvectoracccmd 閻ㄥ嫭娲块弬鑸偓?
            FVector AccCmd = VelocityLoop(VelCmd, ThrustCommand);
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `FVector AngVelCmd`閿涘苯鐣幋?fvectorangvelcmd 閻ㄥ嫭娲块弬鑸偓?
            FVector AngVelCmd = AttitudeLoop(AccCmd);
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `TorqueCommand`閿涘苯鐣幋?torque閸涙垝鎶?閻ㄥ嫭娲块弬鑸偓?
            TorqueCommand = AngularVelocityLoop(AngVelCmd);
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
        }
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鐝涢崡瀹犵儲閸戝搫缍嬮崜宥呮儕閻滎垱鍨?`switch` 閸掑棙鏁敍宀勪缉閸忓秶鎴风紒顓熷⒔鐞涘苯鎮楃紒顓炲瀻閺€顖樷偓?
        break;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯锛愰弰?`switch` 娑擃厾娈戞稉鈧稉顏勫瀻閺€顖涚垼缁涙拝绱濈€电懓绨查弻鎰嚋閸忚渹缍嬮崣鏍р偓鑲╂畱婢跺嫮鎮婄捄顖氱窞閵?
    case EDroneControlMode::Velocity:
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
        {
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `const double CurrentYaw`閿涘苯鐣幋?constdoublecurrentyaw 閻ㄥ嫭娲块弬鑸偓?
            const double CurrentYaw = FMath::DegreesToRadians(CurrentState.GetRotator().Yaw);
            // 鐟欙綁鍣撮敍姘崇殶閻?`UpdateDesiredYawFromPlanarVector` 閹笛嗩攽瑜版挸澧犲銉╊€冮棁鈧憰浣烘畱閸旂喕鍏橀柅鏄忕帆閵?
            UpdateDesiredYawFromPlanarVector(TargetVelocity.X, TargetVelocity.Y, CurrentYaw);

            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `FVector AccCmd`閿涘苯鐣幋?fvectoracccmd 閻ㄥ嫭娲块弬鑸偓?
            FVector AccCmd = VelocityLoop(TargetVelocity, ThrustCommand);
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `FVector AngVelCmd`閿涘苯鐣幋?fvectorangvelcmd 閻ㄥ嫭娲块弬鑸偓?
            FVector AngVelCmd = AttitudeLoop(AccCmd);
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `TorqueCommand`閿涘苯鐣幋?torque閸涙垝鎶?閻ㄥ嫭娲块弬鑸偓?
            TorqueCommand = AngularVelocityLoop(AngVelCmd);
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
        }
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鐝涢崡瀹犵儲閸戝搫缍嬮崜宥呮儕閻滎垱鍨?`switch` 閸掑棙鏁敍宀勪缉閸忓秶鎴风紒顓熷⒔鐞涘苯鎮楃紒顓炲瀻閺€顖樷偓?
        break;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯锛愰弰?`switch` 娑擃厾娈戞稉鈧稉顏勫瀻閺€顖涚垼缁涙拝绱濈€电懓绨查弻鎰嚋閸忚渹缍嬮崣鏍р偓鑲╂畱婢跺嫮鎮婄捄顖氱窞閵?
    case EDroneControlMode::AttitudeThrust:
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
        {
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `double RollDes`閿涘苯鐣幋?doublerolldes 閻ㄥ嫭娲块弬鑸偓?
            double RollDes = FMath::DegreesToRadians(TargetAttitude.Roll);
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `double PitchDes`閿涘苯鐣幋?doublepitchdes 閻ㄥ嫭娲块弬鑸偓?
            double PitchDes = FMath::DegreesToRadians(TargetAttitude.Pitch);
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `double YawDes`閿涘苯鐣幋?doubleyawdes 閻ㄥ嫭娲块弬鑸偓?
            double YawDes = FMath::DegreesToRadians(TargetAttitude.Yaw);
            // UE FRotator 閻?Roll/Pitch 娑撳孩甯堕崚璺哄瀻闁板秶鐓╅梼鐢殿儊閸欓娴夐崣宥忕礉闂団偓鐟曚礁褰囬崣?
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `FRotator CurrentRot`閿涘苯鐣幋?frotatorcurrentrot 閻ㄥ嫭娲块弬鑸偓?
            FRotator CurrentRot = CurrentState.GetRotator();
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `double Roll`閿涘苯鐣幋?doubleroll 閻ㄥ嫭娲块弬鑸偓?
            double Roll = -FMath::DegreesToRadians(CurrentRot.Roll);
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `double Pitch`閿涘苯鐣幋?doublepitch 閻ㄥ嫭娲块弬鑸偓?
            double Pitch = -FMath::DegreesToRadians(CurrentRot.Pitch);
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `double Yaw`閿涘苯鐣幋?doubleyaw 閻ㄥ嫭娲块弬鑸偓?
            double Yaw = FMath::DegreesToRadians(CurrentRot.Yaw);
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `double RollTarget`閿涘苯鐣幋?doublerolltarget 閻ㄥ嫭娲块弬鑸偓?
            double RollTarget = Roll + NormalizeAngle(RollDes - Roll);
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `double PitchTarget`閿涘苯鐣幋?doublepitchtarget 閻ㄥ嫭娲块弬鑸偓?
            double PitchTarget = Pitch + NormalizeAngle(PitchDes - Pitch);
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `double YawTarget`閿涘苯鐣幋?doubleyawtarget 閻ㄥ嫭娲块弬鑸偓?
            double YawTarget = Yaw + NormalizeAngle(YawDes - Yaw);
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `double RollRateCmd`閿涘苯鐣幋?doublerollratecmd 閻ㄥ嫭娲块弬鑸偓?
            double RollRateCmd = RollController ? RollController->Update(RollTarget, Roll) : 0.0;
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `double PitchRateCmd`閿涘苯鐣幋?doublepitchratecmd 閻ㄥ嫭娲块弬鑸偓?
            double PitchRateCmd = PitchController ? PitchController->Update(PitchTarget, Pitch) : 0.0;
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `double YawRateCmd`閿涘苯鐣幋?doubleyawratecmd 閻ㄥ嫭娲块弬鑸偓?
            double YawRateCmd = YawController ? YawController->Update(YawTarget, Yaw) : 0.0;
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `TorqueCommand`閿涘苯鐣幋?torque閸涙垝鎶?閻ㄥ嫭娲块弬鑸偓?
            TorqueCommand = AngularVelocityLoop(FVector(RollRateCmd, PitchRateCmd, YawRateCmd));
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `ThrustCommand`閿涘苯鐣幋?thrust閸涙垝鎶?閻ㄥ嫭娲块弬鑸偓?
            ThrustCommand = TargetThrust;
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
        }
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鐝涢崡瀹犵儲閸戝搫缍嬮崜宥呮儕閻滎垱鍨?`switch` 閸掑棙鏁敍宀勪缉閸忓秶鎴风紒顓熷⒔鐞涘苯鎮楃紒顓炲瀻閺€顖樷偓?
        break;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯锛愰弰?`switch` 娑擃厾娈戞稉鈧稉顏勫瀻閺€顖涚垼缁涙拝绱濈€电懓绨查弻鎰嚋閸忚渹缍嬮崣鏍р偓鑲╂畱婢跺嫮鎮婄捄顖氱窞閵?
    case EDroneControlMode::TorqueThrust:
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼绮伴崙?`if` 閺夆€叉閸掋倖鏌囬敍灞藉涧閺堝娼禒鑸靛灇缁斿妞傞幍宥勭窗鏉╂稑鍙嗘稉瀣桨閻ㄥ嫬鍨庨弨顖炩偓鏄忕帆閵?
        if (ControlCommands.Num() >= 4)
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
        {
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `TorqueCommand`閿涘苯鐣幋?torque閸涙垝鎶?閻ㄥ嫭娲块弬鑸偓?
            TorqueCommand = FVector(ControlCommands[0], ControlCommands[1], ControlCommands[2]);
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `ThrustCommand`閿涘苯鐣幋?thrust閸涙垝鎶?閻ㄥ嫭娲块弬鑸偓?
            ThrustCommand = ControlCommands[3];
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
        }
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鐝涢崡瀹犵儲閸戝搫缍嬮崜宥呮儕閻滎垱鍨?`switch` 閸掑棙鏁敍宀勪缉閸忓秶鎴风紒顓熷⒔鐞涘苯鎮楃紒顓炲瀻閺€顖樷偓?
        break;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯锛愰弰?`switch` 娑擃厾娈戞稉鈧稉顏勫瀻閺€顖涚垼缁涙拝绱濈€电懓绨查弻鎰嚋閸忚渹缍嬮崣鏍р偓鑲╂畱婢跺嫮鎮婄捄顖氱窞閵?
    case EDroneControlMode::Idle:
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯锛愰弰?`switch` 閻ㄥ嫰绮拋銈呭瀻閺€顖ょ礉瑜版挸澧犻棃銏″閺?`case` 闁垝绗夐崠褰掑帳閺冭埖澧界悰灞烩偓?
    default:
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘矁绻戦崶鐐茬秼閸撳秴鍤遍弫鎵畱鐠侊紕鐣荤紒鎾寸亯閿涘本濡搁幒褍鍩楅弶鍐ф唉閸ョ偠鐨熼悽銊︽煙閵?
        return;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
    }
    // 鐟欙綁鍣撮敍姘崇殶閻?`CalculateMotorSpeeds` 閹笛嗩攽瑜版挸澧犲銉╊€冮棁鈧憰浣烘畱閸旂喕鍏橀柅鏄忕帆閵?
    TArray<double> MotorSpeeds = CalculateMotorSpeeds(TorqueCommand, ThrustCommand);
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `CurrentState.MotorSpeeds`閿涘苯鐣幋?motorspeeds 閻ㄥ嫭娲块弬鑸偓?
    CurrentState.MotorSpeeds = MotorSpeeds;
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
}

/**
 * @brief 娴犲骸濮忛惌鈺佹嫲閹恒劌濮忛崨鎴掓姢鐠侊紕鐣婚崶娑楅嚋閻㈠灚婧€閻ㄥ嫯娴嗛柅?
 * @param TorqueCommand 娑撳閰遍崝娑氱叐閸涙垝鎶?
 * @param Thrust 閹粯甯归崝娑樻嚒娴?
 * @return 閸ユ稐閲滈悽鍨簚鏉烆剟鈧?
 * 1. 闁棗鍨庨柊宥囩叐闂冨灚鐪扮憴?锠呰檹
 * 2. Mixer 妤楀崬鎷扮悰銉ヤ缉閿涘牅绗呭┃銏犱焊缁?+ 娑撳﹥瀛╃紓鈺傛杹閿?
 * 3. 娑撯偓闂冩湹缍嗛柅姘姢濞夈垺膩閹风喓鏁搁張鐑樺劵閹?
 */
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯鐣炬稊澶婂毐閺?`CalculateMotorSpeeds`閿涘苯绱戞慨瀣杽閻滅櫛alculatemotorspeeds閻ㄥ嫬鍙挎担鎾烩偓鏄忕帆閵?
TArray<double> UDroneMovementComponent::CalculateMotorSpeeds(const FVector& TorqueCommand, double Thrust)
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
{
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘矁鎯ょ€圭偛缍嬮崜宥喣侀崸妞捐厬閻ㄥ嫬鍙挎担鎾崇杽閻滄壆绮忛懞鍌︾礉娑撹桨绗傞棃銏㈡畱婢圭増妲戦妴浣稿彆瀵繑鍨ㄩ幒褍鍩楀ù浣衡柤閹绘劒绶电€圭偤妾幍褑顢戠拠顓炲綖閵?
    TArray<double> Input = {Thrust, TorqueCommand.X, TorqueCommand.Y, TorqueCommand.Z};
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘矁鎯ょ€圭偛缍嬮崜宥喣侀崸妞捐厬閻ㄥ嫬鍙挎担鎾崇杽閻滄壆绮忛懞鍌︾礉娑撹桨绗傞棃銏㈡畱婢圭増妲戦妴浣稿彆瀵繑鍨ㄩ幒褍鍩楀ù浣衡柤閹绘劒绶电€圭偤妾幍褑顢戠拠顓炲綖閵?
    TArray<double> OmegaSquared = {0.0, 0.0, 0.0, 0.0};
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯绱戞慨?`for` 瀵邦亞骞嗛敍宀€鏁ゆ禍搴㈠瘻閺冦垹鐣惧▎鈩冩殶閹存牕绨崚妤呬憾閸樺棙澧界悰灞芥倵缂侇參鈧槒绶妴?
    for (int32 i = 0; i < 4; ++i)
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯绱戞慨?`for` 瀵邦亞骞嗛敍宀€鏁ゆ禍搴㈠瘻閺冦垹鐣惧▎鈩冩殶閹存牕绨崚妤呬憾閸樺棙澧界悰灞芥倵缂侇參鈧槒绶妴?
        for (int32 j = 0; j < 4; ++j)
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯婀?`OmegaSquared[i]` 閻ㄥ嫬甯張澶婄唨绾偓娑撳﹦鎴风紒顓犵柈閸旂姵鏌婇柌蹇ョ礉閻劋绨幐浣虹敾閺囧瓨鏌?omegasquared閵?
            OmegaSquared[i] += GInv[i][j] * Input[j];

    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯锛愰弰搴㈠灇閸涙ɑ鍨ㄧ仦鈧柈銊ュ綁闁?`MotorSpeeds`閿涘瞼鏁ゆ禍搴濈箽鐎涙Δotorspeeds閵?
    TArray<double> MotorSpeeds;
    // 鐟欙綁鍣撮敍姘崇殶閻?`SetNum` 閹笛嗩攽瑜版挸澧犲銉╊€冮棁鈧憰浣烘畱閸旂喕鍏橀柅鏄忕帆閵?
    MotorSpeeds.SetNum(4);
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯绱戞慨?`for` 瀵邦亞骞嗛敍宀€鏁ゆ禍搴㈠瘻閺冦垹鐣惧▎鈩冩殶閹存牕绨崚妤呬憾閸樺棙澧界悰灞芥倵缂侇參鈧槒绶妴?
    for (int32 i = 0; i < 4; ++i)
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘矂鈧俺绻?`FMath::Max` 缂?`MotorSpeeds[i]` 閺傝棄濮炴稉瀣櫕缁撅附娼敍宀勪缉閸?motorspeeds 鏉╁洤鐨妴?
        MotorSpeeds[i] = FMath::Sqrt(FMath::Max(0.0, OmegaSquared[i]));

    // --- Mixer 妤楀崬鎷扮悰銉ヤ缉 ---
    // 娑撳瀛╃悰銉ヤ缉閿涙俺瀚㈡禒璁崇閻㈠灚婧€娴ｅ簼绨張鈧亸蹇撯偓纭风礉閹碘偓閺堝鏁搁張鍝勫閸嬪繒些
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯锛愰弰搴㈠灇閸涙ɑ鍨ㄧ仦鈧柈銊ュ綁闁?`MinSpeed`閿涘瞼鏁ゆ禍搴濈箽鐎涙Δinspeed閵?
    double MinSpeed = MotorSpeeds[0];
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯绱戞慨?`for` 瀵邦亞骞嗛敍宀€鏁ゆ禍搴㈠瘻閺冦垹鐣惧▎鈩冩殶閹存牕绨崚妤呬憾閸樺棙澧界悰灞芥倵缂侇參鈧槒绶妴?
    for (int32 i = 1; i < 4; ++i)
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘矂鈧俺绻?`FMath::Min` 缂?`MinSpeed` 閺傝棄濮炴稉濠勬櫕缁撅附娼敍宀勪缉閸?minspeed 鏉╁洤銇囬妴?
        MinSpeed = FMath::Min(MinSpeed, MotorSpeeds[i]);
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼绮伴崙?`if` 閺夆€叉閸掋倖鏌囬敍灞藉涧閺堝娼禒鑸靛灇缁斿妞傞幍宥勭窗鏉╂稑鍙嗘稉瀣桨閻ㄥ嫬鍨庨弨顖炩偓鏄忕帆閵?
    if (MinSpeed < Parameters.MinMotorSpeed)
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
    {
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯锛愰弰搴㈠灇閸涙ɑ鍨ㄧ仦鈧柈銊ュ綁闁?`Undershoot`閿涘瞼鏁ゆ禍搴濈箽鐎涙Πndershoot閵?
        double Undershoot = Parameters.MinMotorSpeed - MinSpeed;
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯绱戞慨?`for` 瀵邦亞骞嗛敍宀€鏁ゆ禍搴㈠瘻閺冦垹鐣惧▎鈩冩殶閹存牕绨崚妤呬憾閸樺棙澧界悰灞芥倵缂侇參鈧槒绶妴?
        for (int32 i = 0; i < 4; ++i)
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯婀?`MotorSpeeds[i]` 閻ㄥ嫬甯張澶婄唨绾偓娑撳﹦鎴风紒顓犵柈閸旂姵鏌婇柌蹇ョ礉閻劋绨幐浣虹敾閺囧瓨鏌?motorspeeds閵?
            MotorSpeeds[i] += Undershoot;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
    }

    // 娑撳﹥瀛╃悰銉ヤ缉閿涙俺瀚㈡禒璁崇閻㈠灚婧€鐡掑懓绻冮張鈧径褍鈧》绱濈粵澶嬬槷缂傗晜鏂?
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯锛愰弰搴㈠灇閸涙ɑ鍨ㄧ仦鈧柈銊ュ綁闁?`MaxSpeed`閿涘瞼鏁ゆ禍搴濈箽鐎涙ɑ娓舵径褔鈧喎瀹崇痪锔芥将閵?
    double MaxSpeed = MotorSpeeds[0];
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯绱戞慨?`for` 瀵邦亞骞嗛敍宀€鏁ゆ禍搴㈠瘻閺冦垹鐣惧▎鈩冩殶閹存牕绨崚妤呬憾閸樺棙澧界悰灞芥倵缂侇參鈧槒绶妴?
    for (int32 i = 1; i < 4; ++i)
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘矂鈧俺绻?`FMath::Max` 缂?`MaxSpeed` 閺傝棄濮炴稉瀣櫕缁撅附娼敍宀勪缉閸?閺堚偓婢堆団偓鐔峰缁撅附娼?鏉╁洤鐨妴?
        MaxSpeed = FMath::Max(MaxSpeed, MotorSpeeds[i]);
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼绮伴崙?`if` 閺夆€叉閸掋倖鏌囬敍灞藉涧閺堝娼禒鑸靛灇缁斿妞傞幍宥勭窗鏉╂稑鍙嗘稉瀣桨閻ㄥ嫬鍨庨弨顖炩偓鏄忕帆閵?
    if (MaxSpeed > Parameters.MaxMotorSpeed)
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
    {
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯锛愰弰搴㈠灇閸涙ɑ鍨ㄧ仦鈧柈銊ュ綁闁?`Scale`閿涘瞼鏁ゆ禍搴濈箽鐎涙Ξcale閵?
        double Scale = Parameters.MaxMotorSpeed / MaxSpeed;
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯绱戞慨?`for` 瀵邦亞骞嗛敍宀€鏁ゆ禍搴㈠瘻閺冦垹鐣惧▎鈩冩殶閹存牕绨崚妤呬憾閸樺棙澧界悰灞芥倵缂侇參鈧槒绶妴?
        for (int32 i = 0; i < 4; ++i)
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `MotorSpeeds[i]`閿涘苯鐣幋?motorspeeds 閻ㄥ嫭娲块弬鑸偓?
            MotorSpeeds[i] *= Scale;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
    }

    // 閺堚偓缂?Clamp
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯绱戞慨?`for` 瀵邦亞骞嗛敍宀€鏁ゆ禍搴㈠瘻閺冦垹鐣惧▎鈩冩殶閹存牕绨崚妤呬憾閸樺棙澧界悰灞芥倵缂侇參鈧槒绶妴?
    for (int32 i = 0; i < 4; ++i)
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯鍘涚€电顓哥粻妤冪波閺嬫粌浠涢梽鎰畽閿涘苯鍟€閸愭瑥鍙?`MotorSpeeds[i]`閿涘矂妲诲?motorspeeds 鐡掑懎鍤崗浣筋啅閼煎啫娲块妴?
        MotorSpeeds[i] = FMath::Clamp(MotorSpeeds[i], Parameters.MinMotorSpeed, Parameters.MaxMotorSpeed);

    // --- 娑撯偓闂冩湹缍嗛柅姘姢濞夈垺膩閹风喓鏁搁張鐑樺劵閹?---
    // 娴ｈ法鏁ら幒褍鍩楅弴瀛樻煀閻ㄥ嫬鐤勯梽?DeltaTime閿涘矁鈧矂娼悧鈺冩倞鐎涙劖顒炲銉╂毐
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼绮伴崙?`if` 閺夆€叉閸掋倖鏌囬敍灞藉涧閺堝娼禒鑸靛灇缁斿妞傞幍宥勭窗鏉╂稑鍙嗘稉瀣桨閻ㄥ嫬鍨庨弨顖炩偓鏄忕帆閵?
    if (MotorSpeedsFiltered.Num() < 4)
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `MotorSpeedsFiltered`閿涘苯鐣幋?motorspeedsfiltered 閻ㄥ嫭娲块弬鑸偓?
        MotorSpeedsFiltered = {0.0, 0.0, 0.0, 0.0};
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘矂鈧俺绻?`FMath::Max` 缂?`double FilterDt` 閺傝棄濮炴稉瀣櫕缁撅附娼敍宀勪缉閸?doublefilterdt 鏉╁洤鐨妴?
    double FilterDt = FMath::Max(0.001, LastControlDeltaTime);
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘矂鈧俺绻?`FMath::Max` 缂?`double Alpha` 閺傝棄濮炴稉瀣櫕缁撅附娼敍宀勪缉閸?doublealpha 鏉╁洤鐨妴?
    double Alpha = FMath::Exp(-FilterDt / FMath::Max(0.001, Parameters.MotorFilterTC));
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯绱戞慨?`for` 瀵邦亞骞嗛敍宀€鏁ゆ禍搴㈠瘻閺冦垹鐣惧▎鈩冩殶閹存牕绨崚妤呬憾閸樺棙澧界悰灞芥倵缂侇參鈧槒绶妴?
    for (int32 i = 0; i < 4; ++i)
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `MotorSpeedsFiltered[i]`閿涘苯鐣幋?motorspeedsfiltered 閻ㄥ嫭娲块弬鑸偓?
        MotorSpeedsFiltered[i] = MotorSpeedsFiltered[i] * Alpha + MotorSpeeds[i] * (1.0 - Alpha);

    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘矁绻戦崶鐐茬秼閸撳秴鍤遍弫鎵畱鐠侊紕鐣荤紒鎾寸亯閿涘本濡搁幒褍鍩楅弶鍐ф唉閸ョ偠鐨熼悽銊︽煙閵?
    return MotorSpeedsFiltered;
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
}

/**
 * @brief Verlet 缁夘垰鍨庢稉鈧?
 * @param DeltaTime 缁夘垰鍨庡銉╂毐
 * 娴ｈ法鏁?Velocity Verlet 閺傝纭堕敍?
 *   x(t+dt) = x(t) + v(t)*dt + 0.5*a(t)*dt铏?
 *   a(t+dt) = F(t+dt) / m
 *   v(t+dt) = v(t) + 0.5*(a(t) + a(t+dt))*dt
 * 鐟欐帒瀹抽弴瀛樻煀娴ｈ法鏁ょ憴鎺椻偓鐔峰閸?AngleAxis 閺傝纭?
 */
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯鐣炬稊澶婂毐閺?`VerletUpdate`閿涘苯绱戞慨瀣杽閻滅殢erletupdate閻ㄥ嫬鍙挎担鎾烩偓鏄忕帆閵?
void UDroneMovementComponent::VerletUpdate(double DeltaTime)
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
{
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯锛愰弰搴㈠灇閸涙ɑ鍨ㄧ仦鈧柈銊ュ綁闁?`dt`閿涘瞼鏁ゆ禍搴濈箽鐎涙t閵?
    const double dt = DeltaTime;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯锛愰弰搴㈠灇閸涙ɑ鍨ㄧ仦鈧柈銊ュ綁闁?`halfDt`閿涘瞼鏁ゆ禍搴濈箽鐎涙alfdt閵?
    const double halfDt = 0.5 * dt;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯锛愰弰搴㈠灇閸涙ɑ鍨ㄧ仦鈧柈銊ュ綁闁?`halfDtSq`閿涘瞼鏁ゆ禍搴濈箽鐎涙alfdtsq閵?
    const double halfDtSq = 0.5 * dt * dt;

    // --- Ground Lock閿涙艾婀撮棃顫瑐閺冭埖顥呴弻銉﹀腹閸旀稓娈戦崹鍌滄纯閸掑棝鍣洪弰顖氭儊鐡掑懓绻冮柌宥呭 ---
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼绮伴崙?`if` 閺夆€叉閸掋倖鏌囬敍灞藉涧閺堝娼禒鑸靛灇缁斿妞傞幍宥勭窗鏉╂稑鍙嗘稉瀣桨閻ㄥ嫬鍨庨弨顖炩偓鏄忕帆閵?
    if (bGrounded)
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
    {
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯锛愰弰搴㈠灇閸涙ɑ鍨ㄧ仦鈧柈銊ュ綁闁?`Torque`閿涘瞼鏁ゆ禍搴濈箽鐎涙Οorque閵?
        FVector Force, Torque;
        // 鐟欙綁鍣撮敍姘崇殶閻?`CalculateTotalForcesAndTorques` 閹笛嗩攽瑜版挸澧犲銉╊€冮棁鈧憰浣烘畱閸旂喕鍏橀柅鏄忕帆閵?
        CalculateTotalForcesAndTorques(CurrentState, Force, Torque);
        // 濡偓閺屻儲甯归崝娑樻躬娑撴牜鏅?Z 鏉炲娈戦崚鍡涘櫤閺勵垰鎯佺搾鍛扮箖闁插秴濮?
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯锛愰弰搴㈠灇閸涙ɑ鍨ㄧ仦鈧柈銊ュ綁闁?`WeightForce`閿涘瞼鏁ゆ禍搴濈箽鐎涙Τeightforce閵?
        double WeightForce = Parameters.Mass * Parameters.Gravity;
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼绮伴崙?`if` 閺夆€叉閸掋倖鏌囬敍灞藉涧閺堝娼禒鑸靛灇缁斿妞傞幍宥勭窗鏉╂稑鍙嗘稉瀣桨閻ㄥ嫬鍨庨弨顖炩偓鏄忕帆閵?
        if (Force.Z > WeightForce * 1.01)  // 5% 鐟佹洟鍣洪梼鍙夘剾閹躲垼鎹ｉ崣鍫ｆ儰娑?
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
        {
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `bGrounded`閿涘苯鐣幋?鐢啫鐨甸弽鍥х箶 grounded 閻ㄥ嫭娲块弬鑸偓?
            bGrounded = false;
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘奔缍呮禍搴㈢€柅鐘插毐閺佹澘鍨垫慨瀣閸掓銆冩稉顓ㄧ礉閹?`UE_LOG` 閻╁瓨甯撮崚婵嗩潗閸栨牔璐?`LogTemp, Log, TEXT("[DroneMovement] Lifting off ground, Force.Z=%.2f > Weight=%.2f"`閿涘苯鍣虹亸鎴ｇ箻閸忋儱鍤遍弫棰佺秼閸氬海娈戞０婵嗩樆鐠у鈧厧绱戦柨鈧妴?
            UE_LOG(LogTemp, Log, TEXT("[DroneMovement] Lifting off ground, Force.Z=%.2f > Weight=%.2f"),
                // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘矁鎯ょ€圭偛缍嬮崜宥喣侀崸妞捐厬閻ㄥ嫬鍙挎担鎾崇杽閻滄壆绮忛懞鍌︾礉娑撹桨绗傞棃銏㈡畱婢圭増妲戦妴浣稿彆瀵繑鍨ㄩ幒褍鍩楀ù浣衡柤閹绘劒绶电€圭偤妾幍褑顢戠拠顓炲綖閵?
                Force.Z, WeightForce);
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
        }
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼绮伴崙鍝勫幑鎼存洖鍨庨弨顖ょ礉瑜版挷绗傞棃銏㈡畱閺夆€叉闁垝绗夊陇鍐婚弮鑸靛⒔鐞涘被鈧?
        else
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
        {
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘矁绻戦崶鐐茬秼閸撳秴鍤遍弫鎵畱鐠侊紕鐣荤紒鎾寸亯閿涘本濡搁幒褍鍩楅弶鍐ф唉閸ョ偠鐨熼悽銊︽煙閵?
            return; // 閻ｆ瑥婀崷浼存桨閿涘奔绗夐弴瀛樻煀閻樿埖鈧?
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
        }
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
    }

    // --- 娴ｅ秶鐤嗛弴瀛樻煀閿涙(t+dt) = x(t) + v(t)*dt + 0.5*a(t)*dt铏?---
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `FVector Pos`閿涘苯鐣幋?fvectorpos 閻ㄥ嫭娲块弬鑸偓?
    FVector Pos = CurrentState.GetPosition();
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `FVector Vel`閿涘苯鐣幋?fvectorvel 閻ㄥ嫭娲块弬鑸偓?
    FVector Vel = CurrentState.GetVelocity();
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `FVector AngVel`閿涘苯鐣幋?fvectorangvel 閻ㄥ嫭娲块弬鑸偓?
    FVector AngVel = CurrentState.GetAngularVelocity();

    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯婀?`Pos` 閻ㄥ嫬甯張澶婄唨绾偓娑撳﹦鎴风紒顓犵柈閸旂姵鏌婇柌蹇ョ礉閻劋绨幐浣虹敾閺囧瓨鏌?pos閵?
    Pos += Vel * dt + PrevLinearAcceleration * halfDtSq;
    // 鐟欙綁鍣撮敍姘崇殶閻?`SetPosition` 閹笛嗩攽瑜版挸澧犲銉╊€冮棁鈧憰浣烘畱閸旂喕鍏橀柅鏄忕帆閵?
    CurrentState.SetPosition(Pos);

    // --- 婵寧鈧焦娲块弬甯窗閻劏顫楅柅鐔峰鐠侊紕鐣荤憴鎺戝婢х偤鍣?---
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯锛愰弰搴㈠灇閸涙ɑ鍨ㄧ仦鈧柈銊ュ綁闁?`AvgAngVel`閿涘瞼鏁ゆ禍搴濈箽鐎涙vgangvel閵?
    FVector AvgAngVel = AngVel + PrevAngularAcceleration * halfDt;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崥鎴﹀櫤濡繝鏆遍崘娆忓弳 `double AnglePerUnit`閿涘瞼鏁ゆ禍搴ゃ€冪粈楦跨獩缁傛眹鈧線鈧喎瀹虫径褍鐨幋鏍︾瑝绾喖鐣炬惔锔衡偓?
    double AnglePerUnit = AvgAngVel.Size();
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼绮伴崙?`if` 閺夆€叉閸掋倖鏌囬敍灞藉涧閺堝娼禒鑸靛灇缁斿妞傞幍宥勭窗鏉╂稑鍙嗘稉瀣桨閻ㄥ嫬鍨庨弨顖炩偓鏄忕帆閵?
    if (AnglePerUnit > KINDA_SMALL_NUMBER)
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
    {
        // 鐟欙綁鍣撮敍姘崇殶閻?`DeltaQ` 閹笛嗩攽瑜版挸澧犲銉╊€冮棁鈧憰浣烘畱閸旂喕鍏橀柅鏄忕帆閵?
        FQuat DeltaQ(AvgAngVel / AnglePerUnit, AnglePerUnit * dt);
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `FQuat CurrentQ`閿涘苯鐣幋?fquatcurrentQ 閻ㄥ嫭娲块弬鑸偓?
        FQuat CurrentQ = CurrentState.GetQuaternion();
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯锛愰弰搴㈠灇閸涙ɑ鍨ㄧ仦鈧柈銊ュ綁闁?`NewQ`閿涘瞼鏁ゆ禍搴濈箽鐎涙ΕewQ閵?
        FQuat NewQ = CurrentQ * DeltaQ;
        // 鐟欙綁鍣撮敍姘崇殶閻?`Normalize` 閹笛嗩攽瑜版挸澧犲銉╊€冮棁鈧憰浣烘畱閸旂喕鍏橀柅鏄忕帆閵?
        NewQ.Normalize();
        // 鐟欙綁鍣撮敍姘崇殶閻?`SetQuaternion` 閹笛嗩攽瑜版挸澧犲銉╊€冮棁鈧憰浣烘畱閸旂喕鍏橀柅鏄忕帆閵?
        CurrentState.SetQuaternion(NewQ);
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
    }

    // --- 鐠侊紕鐣婚弬棰佺秴缂冾喖顦╅惃鍕閸滃苯濮忛惌?---
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯锛愰弰搴㈠灇閸涙ɑ鍨ㄧ仦鈧柈銊ュ綁闁?`Torque`閿涘瞼鏁ゆ禍搴濈箽鐎涙Οorque閵?
    FVector Force, Torque;
    // 鐟欙綁鍣撮敍姘崇殶閻?`CalculateTotalForcesAndTorques` 閹笛嗩攽瑜版挸澧犲銉╊€冮棁鈧憰浣烘畱閸旂喕鍏橀柅鏄忕帆閵?
    CalculateTotalForcesAndTorques(CurrentState, Force, Torque);

    // --- 缁惧灝濮為柅鐔峰閿涙瓫(t+dt) = F/m ---
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `FVector NewLinearAcc`閿涘苯鐣幋?fvectornewlinearacc 閻ㄥ嫭娲块弬鑸偓?
    FVector NewLinearAcc = Force / Parameters.Mass + FVector(0.0, 0.0, -Parameters.Gravity);

    // --- 鐟欐帒濮為柅鐔峰閿涙碍顑傞幏澶嬫鏉烆剚鏌熺粙?---
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯锛愰弰搴㈠灇閸涙ɑ鍨ㄧ仦鈧柈銊ュ綁闁?`Jx`閿涘瞼鏁ゆ禍搴濈箽鐎涙Αx閵?
    double Jx = Parameters.Jx, Jy = Parameters.Jy, Jz = Parameters.Jz;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯锛愰弰搴㈠灇閸涙ɑ鍨ㄧ仦鈧柈銊ュ綁闁?`p`閿涘瞼鏁ゆ禍搴濈箽鐎涙ザ閵?
    double p = CurrentState.AngRollRate;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯锛愰弰搴㈠灇閸涙ɑ鍨ㄧ仦鈧柈銊ュ綁闁?`q`閿涘瞼鏁ゆ禍搴濈箽鐎涙ズ閵?
    double q = CurrentState.AngPitchRate;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯锛愰弰搴㈠灇閸涙ɑ鍨ㄧ仦鈧柈銊ュ綁闁?`r`閿涘瞼鏁ゆ禍搴濈箽鐎涙セ閵?
    double r = CurrentState.AngYawRate;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯锛愰弰搴㈠灇閸涙ɑ鍨ㄧ仦鈧柈銊ュ綁闁?`AngMomentumRate`閿涘瞼鏁ゆ禍搴濈箽鐎涙ngmomentumrate閵?
    FVector AngMomentumRate;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `AngMomentumRate.X`閿涘苯鐣幋?閻樿埖鈧礁鎮滈柌?X 閻ㄥ嫭娲块弬鑸偓?
    AngMomentumRate.X = (Torque.X - (Jz - Jy) * q * r) / Jx;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `AngMomentumRate.Y`閿涘苯鐣幋?Y 閻ㄥ嫭娲块弬鑸偓?
    AngMomentumRate.Y = (Torque.Y - (Jx - Jz) * p * r) / Jy;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `AngMomentumRate.Z`閿涘苯鐣幋?Z 閻ㄥ嫭娲块弬鑸偓?
    AngMomentumRate.Z = (Torque.Z - (Jy - Jx) * p * q) / Jz;

    // --- 闁喎瀹抽弴瀛樻煀閿涙(t+dt) = v(t) + 0.5*(a(t) + a(t+dt))*dt ---
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `FVector NewVel`閿涘苯鐣幋?fvectornewvel 閻ㄥ嫭娲块弬鑸偓?
    FVector NewVel = Vel + (PrevLinearAcceleration + NewLinearAcc) * halfDt;
    // 鐟欙綁鍣撮敍姘崇殶閻?`SetVelocity` 閹笛嗩攽瑜版挸澧犲銉╊€冮棁鈧憰浣烘畱閸旂喕鍏橀柅鏄忕帆閵?
    CurrentState.SetVelocity(NewVel);

    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `FVector NewAngVel`閿涘苯鐣幋?fvectornewangvel 閻ㄥ嫭娲块弬鑸偓?
    FVector NewAngVel = AngVel + (PrevAngularAcceleration + AngMomentumRate) * halfDt;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `CurrentState.AngRollRate`閿涘苯鐣幋?angrollrate 閻ㄥ嫭娲块弬鑸偓?
    CurrentState.AngRollRate = NewAngVel.X;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `CurrentState.AngPitchRate`閿涘苯鐣幋?angpitchrate 閻ㄥ嫭娲块弬鑸偓?
    CurrentState.AngPitchRate = NewAngVel.Y;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `CurrentState.AngYawRate`閿涘苯鐣幋?angyawrate 閻ㄥ嫭娲块弬鑸偓?
    CurrentState.AngYawRate = NewAngVel.Z;

    // --- 娣囨繂鐡ㄩ崝鐘烩偓鐔峰娓氭稐绗呮稉鈧敮褌濞囬悽?---
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `PrevLinearAcceleration`閿涘苯鐣幋?prevlinearacceleration 閻ㄥ嫭娲块弬鑸偓?
    PrevLinearAcceleration = NewLinearAcc;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `PrevAngularAcceleration`閿涘苯鐣幋?prevangularacceleration 閻ㄥ嫭娲块弬鑸偓?
    PrevAngularAcceleration = AngMomentumRate;

    // --- 閸ユ稑鍘撻弫鏉跨秺娑撯偓閸?---
    // 鐟欙綁鍣撮敍姘崇殶閻?`NormalizeQuaternion` 閹笛嗩攽瑜版挸澧犲銉╊€冮棁鈧憰浣烘畱閸旂喕鍏橀柅鏄忕帆閵?
    CurrentState.NormalizeQuaternion();
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
}

/**
 * @brief 鐠侊紕鐣荤紒娆忕暰閻樿埖鈧椒绗呴惃鍕値婢舵牕濮忛崪灞芥値閸旀稓鐓?
 * @param State 瑜版挸澧犻悩鑸碘偓?
 * @param OutForce 娑撴牜鏅崸鎰垼缁鎮庢径鏍у閿涘牅绗夐崥顐﹀櫢閸旀冻绱濋柌宥呭閸?Verlet 娑擃厼宕熼悪顒€顦╅悶鍡礆
 * @param OutTorque 閺堣桨缍嬮崸鎰垼缁鎮庨崝娑氱叐
 */
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯鐣炬稊澶婂毐閺?`CalculateTotalForcesAndTorques`閿涘苯绱戞慨瀣杽閻滅櫛alculatetotalforcesandtorques閻ㄥ嫬鍙挎担鎾烩偓鏄忕帆閵?
void UDroneMovementComponent::CalculateTotalForcesAndTorques(const FDroneState& State, FVector& OutForce, FVector& OutTorque)
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
{
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `OutForce`閿涘苯鐣幋?outforce 閻ㄥ嫭娲块弬鑸偓?
    OutForce = FVector::ZeroVector;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `OutTorque`閿涘苯鐣幋?outtorque 閻ㄥ嫭娲块弬鑸偓?
    OutTorque = FVector::ZeroVector;

    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯锛愰弰搴㈠灇閸涙ɑ鍨ㄧ仦鈧柈銊ュ綁闁?`TotalThrust`閿涘瞼鏁ゆ禍搴濈箽鐎涙Οotalthrust閵?
    double TotalThrust = 0.0;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯锛愰弰搴㈠灇閸涙ɑ鍨ㄧ仦鈧柈銊ュ綁闁?`TotalTorque`閿涘瞼鏁ゆ禍搴濈箽鐎涙Οotaltorque閵?
    FVector TotalTorque = FVector::ZeroVector;

    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼绮伴崙?`if` 閺夆€叉閸掋倖鏌囬敍灞藉涧閺堝娼禒鑸靛灇缁斿妞傞幍宥勭窗鏉╂稑鍙嗘稉瀣桨閻ㄥ嫬鍨庨弨顖炩偓鏄忕帆閵?
    if (State.MotorSpeeds.Num() >= 4)
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
    {
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘矁鎯ょ€圭偛缍嬮崜宥喣侀崸妞捐厬閻ㄥ嫬鍙挎担鎾崇杽閻滄壆绮忛懞鍌︾礉娑撹桨绗傞棃銏㈡畱婢圭増妲戦妴浣稿彆瀵繑鍨ㄩ幒褍鍩楀ù浣衡柤閹绘劒绶电€圭偤妾幍褑顢戠拠顓炲綖閵?
        TArray<double> OmegaSq = {
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘矁鎯ょ€圭偛缍嬮崜宥喣侀崸妞捐厬閻ㄥ嫬鍙挎担鎾崇杽閻滄壆绮忛懞鍌︾礉娑撹桨绗傞棃銏㈡畱婢圭増妲戦妴浣稿彆瀵繑鍨ㄩ幒褍鍩楀ù浣衡柤閹绘劒绶电€圭偤妾幍褑顢戠拠顓炲綖閵?
            State.MotorSpeeds[0] * State.MotorSpeeds[0],
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘矁鎯ょ€圭偛缍嬮崜宥喣侀崸妞捐厬閻ㄥ嫬鍙挎担鎾崇杽閻滄壆绮忛懞鍌︾礉娑撹桨绗傞棃銏㈡畱婢圭増妲戦妴浣稿彆瀵繑鍨ㄩ幒褍鍩楀ù浣衡柤閹绘劒绶电€圭偤妾幍褑顢戠拠顓炲綖閵?
            State.MotorSpeeds[1] * State.MotorSpeeds[1],
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘矁鎯ょ€圭偛缍嬮崜宥喣侀崸妞捐厬閻ㄥ嫬鍙挎担鎾崇杽閻滄壆绮忛懞鍌︾礉娑撹桨绗傞棃銏㈡畱婢圭増妲戦妴浣稿彆瀵繑鍨ㄩ幒褍鍩楀ù浣衡柤閹绘劒绶电€圭偤妾幍褑顢戠拠顓炲綖閵?
            State.MotorSpeeds[2] * State.MotorSpeeds[2],
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘矁鎯ょ€圭偛缍嬮崜宥喣侀崸妞捐厬閻ㄥ嫬鍙挎担鎾崇杽閻滄壆绮忛懞鍌︾礉娑撹桨绗傞棃銏㈡畱婢圭増妲戦妴浣稿彆瀵繑鍨ㄩ幒褍鍩楀ù浣衡柤閹绘劒绶电€圭偤妾幍褑顢戠拠顓炲綖閵?
            State.MotorSpeeds[3] * State.MotorSpeeds[3]
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
        };
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯绱戞慨?`for` 瀵邦亞骞嗛敍宀€鏁ゆ禍搴㈠瘻閺冦垹鐣惧▎鈩冩殶閹存牕绨崚妤呬憾閸樺棙澧界悰灞芥倵缂侇參鈧槒绶妴?
        for (int32 i = 0; i < 4; ++i)
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
        {
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯婀?`TotalThrust` 閻ㄥ嫬甯張澶婄唨绾偓娑撳﹦鎴风紒顓犵柈閸旂姵鏌婇柌蹇ョ礉閻劋绨幐浣虹敾閺囧瓨鏌?totalthrust閵?
            TotalThrust += G[0][i] * OmegaSq[i];
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯婀?`TotalTorque.X` 閻ㄥ嫬甯張澶婄唨绾偓娑撳﹦鎴风紒顓犵柈閸旂姵鏌婇柌蹇ョ礉閻劋绨幐浣虹敾閺囧瓨鏌?閻樿埖鈧礁鎮滈柌?X閵?
            TotalTorque.X += G[1][i] * OmegaSq[i];
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯婀?`TotalTorque.Y` 閻ㄥ嫬甯張澶婄唨绾偓娑撳﹦鎴风紒顓犵柈閸旂姵鏌婇柌蹇ョ礉閻劋绨幐浣虹敾閺囧瓨鏌?Y閵?
            TotalTorque.Y += G[2][i] * OmegaSq[i];
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯婀?`TotalTorque.Z` 閻ㄥ嫬甯張澶婄唨绾偓娑撳﹦鎴风紒顓犵柈閸旂姵鏌婇柌蹇ョ礉閻劋绨幐浣虹敾閺囧瓨鏌?Z閵?
            TotalTorque.Z += G[3][i] * OmegaSq[i];
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
        }
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
    }

    // 閹恒劌濮忛崥鎴﹀櫤娴犲孩婧€娴?Z 鏉炵娴嗛崚棰佺瑯閻ｅ苯娼楅弽鍥╅兇
    // 鐟欙綁鍣撮敍姘崇殶閻?`ThrustVectorBody` 閹笛嗩攽瑜版挸澧犲銉╊€冮棁鈧憰浣烘畱閸旂喕鍏橀柅鏄忕帆閵?
    FVector ThrustVectorBody(0.0, 0.0, TotalThrust);
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `FQuat Orientation`閿涘苯鐣幋?fquatorientation 閻ㄥ嫭娲块弬鑸偓?
    FQuat Orientation = State.GetQuaternion();
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `FVector ThrustVectorWorld`閿涘苯鐣幋?fvectorthrustvectorworld 閻ㄥ嫭娲块弬鑸偓?
    FVector ThrustVectorWorld = RotateBodyToWorld(ThrustVectorBody, Orientation);

    // 闂冭濮忛敍鍫㈢暆閸栨牗膩閸ㄥ绱漃hase 4 鐏忓棗宕岀痪褌璐熼崥鍕倻瀵倹鈧嶇礆
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `FVector Velocity`閿涘苯鐣幋?fvectorvelocity 閻ㄥ嫭娲块弬鑸偓?
    FVector Velocity = State.GetVelocity();
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崥鎴﹀櫤濡繝鏆遍崘娆忓弳 `FVector Drag`閿涘瞼鏁ゆ禍搴ゃ€冪粈楦跨獩缁傛眹鈧線鈧喎瀹虫径褍鐨幋鏍︾瑝绾喖鐣炬惔锔衡偓?
    FVector Drag = -Parameters.DragCoefficient * Velocity.Size() * Velocity;

    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `OutForce`閿涘苯鐣幋?outforce 閻ㄥ嫭娲块弬鑸偓?
    OutForce = ThrustVectorWorld + Drag;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `OutTorque`閿涘苯鐣幋?outtorque 閻ㄥ嫭娲块弬鑸偓?
    const FVector AngularDamping(
        State.AngRollRate * 0.028,
        State.AngPitchRate * 0.028,
        State.AngYawRate * 0.018);
    OutTorque = TotalTorque - AngularDamping;
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
}

/**
 * @brief 濡偓閺屻儱鑻熸径鍕倞閸︿即娼扮喊鐗堟寬
 * @param GroundZ 閸︿即娼?Z 閸ф劖鐖ｉ敍鍫滆雹閻喎娼楅弽鍥╅兇閿?
 * 瑜版挻妫ゆ禍鐑樻簚娴ｅ秶鐤嗘担搴濈艾閸︿即娼版稉鏂挎倻娑撳绻嶉崝銊︽閿?
 * - 鐏忓棔缍呯純顕€鎸︾€规艾鍩岄崷浼存桨
 * - 濞撳懘娅庨崥鎴滅瑓闁喎瀹?
 * - 娴ｈ法鏁ら幁銏狀槻缁粯鏆熸径鍕倞瀵鐑?
 * - Ground Lock閿涙矮缍嗛柅鐔烘絻闂勫棙妞傞柨浣哥暰婵寧鈧?
 */
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯鐣炬稊澶婂毐閺?`CheckGroundCollision`閿涘苯绱戞慨瀣杽閻滅櫛heckgroundcollision閻ㄥ嫬鍙挎担鎾烩偓鏄忕帆閵?
void UDroneMovementComponent::CheckGroundCollision(double GroundZ)
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
{
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼绮伴崙?`if` 閺夆€叉閸掋倖鏌囬敍灞藉涧閺堝娼禒鑸靛灇缁斿妞傞幍宥勭窗鏉╂稑鍙嗘稉瀣桨閻ㄥ嫬鍨庨弨顖炩偓鏄忕帆閵?
    if (CurrentState.Z <= GroundZ && CurrentState.Vz < 0.0)
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
    {
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `CurrentState.Z`閿涘苯鐣幋?Z 閻ㄥ嫭娲块弬鑸偓?
        CurrentState.Z = GroundZ;

        // 閸掋倖鏌囬弰顖氭儊娑撹桨缍嗛柅鐔烘絻闂勫棴绱欓崹鍌滄纯闁喎瀹抽崡鐘卞瘜鐎电》绱?
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `double SpeedXY`閿涘苯鐣幋?doublespeedxy 閻ㄥ嫭娲块弬鑸偓?
        double SpeedXY = FMath::Sqrt(CurrentState.Vx * CurrentState.Vx + CurrentState.Vy * CurrentState.Vy);
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `bool bLanding`閿涘苯鐣幋?boolBlanding 閻ㄥ嫭娲块弬鑸偓?
        bool bLanding = FMath::Abs(CurrentState.Vz) > SpeedXY;

        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼绮伴崙?`if` 閺夆€叉閸掋倖鏌囬敍灞藉涧閺堝娼禒鑸靛灇缁斿妞傞幍宥勭窗鏉╂稑鍙嗘稉瀣桨閻ㄥ嫬鍨庨弨顖炩偓鏄忕帆閵?
        FVector ContactForce, ContactTorque;
        CalculateTotalForcesAndTorques(CurrentState, ContactForce, ContactTorque);
        const double ContactLift = ContactForce.Z;
        const double WeightForce = Parameters.Mass * Parameters.Gravity;

        // 飞控仍在主动工作时不要重新锁地，否则刚离地就会被强行贴回地面。
        const bool bLowPower = ContactLift <= WeightForce * 0.90;
        const bool bLowMotion = SpeedXY < 0.5 &&
            FMath::Abs(CurrentState.AngRollRate) < 0.5 &&
            FMath::Abs(CurrentState.AngPitchRate) < 0.5 &&
            FMath::Abs(CurrentState.AngYawRate) < 0.5;
        const bool bAllowGroundLock =
            (CurrentControlMode == EDroneControlMode::Idle) || (bLowPower && bLowMotion);

        if (bAllowGroundLock && bLanding && FMath::Abs(CurrentState.Vz) < 1.0 && ContactLift <= WeightForce * 1.02)
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
        {
            // Ground Lock閿涙矮缍嗛柅鐔烘絻闂勫棴绱濋柨浣哥暰閸︺劌婀撮棃?
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `bGrounded`閿涘苯鐣幋?鐢啫鐨甸弽鍥х箶 grounded 閻ㄥ嫭娲块弬鑸偓?
            bGrounded = true;
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `CurrentState.Vx`閿涘苯鐣幋?vx 閻ㄥ嫭娲块弬鑸偓?
            CurrentState.Vx = 0.0;
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `CurrentState.Vy`閿涘苯鐣幋?vy 閻ㄥ嫭娲块弬鑸偓?
            CurrentState.Vy = 0.0;
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `CurrentState.Vz`閿涘苯鐣幋?vz 閻ㄥ嫭娲块弬鑸偓?
            CurrentState.Vz = 0.0;
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `CurrentState.AngRollRate`閿涘苯鐣幋?angrollrate 閻ㄥ嫭娲块弬鑸偓?
            CurrentState.AngRollRate = 0.0;
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `CurrentState.AngPitchRate`閿涘苯鐣幋?angpitchrate 閻ㄥ嫭娲块弬鑸偓?
            CurrentState.AngPitchRate = 0.0;
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `CurrentState.AngYawRate`閿涘苯鐣幋?angyawrate 閻ㄥ嫭娲块弬鑸偓?
            CurrentState.AngYawRate = 0.0;

            // 閻偅顒滄慨鎸庘偓渚婄窗Roll/Pitch 瑜版帡娴傞敍灞肩箽閻?Yaw
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `FRotator Rot`閿涘苯鐣幋?frotatorrot 閻ㄥ嫭娲块弬鑸偓?
            FRotator Rot = CurrentState.GetRotator();
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `Rot.Roll`閿涘苯鐣幋?roll 閻ㄥ嫭娲块弬鑸偓?
            Rot.Roll = 0.0f;
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `Rot.Pitch`閿涘苯鐣幋?pitch 閻ㄥ嫭娲块弬鑸偓?
            Rot.Pitch = 0.0f;
            // 鐟欙綁鍣撮敍姘崇殶閻?`SetQuaternion` 閹笛嗩攽瑜版挸澧犲銉╊€冮棁鈧憰浣烘畱閸旂喕鍏橀柅鏄忕帆閵?
            CurrentState.SetQuaternion(Rot.Quaternion());

            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `PrevLinearAcceleration`閿涘苯鐣幋?prevlinearacceleration 閻ㄥ嫭娲块弬鑸偓?
            PrevLinearAcceleration = FVector::ZeroVector;
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `PrevAngularAcceleration`閿涘苯鐣幋?prevangularacceleration 閻ㄥ嫭娲块弬鑸偓?
            PrevAngularAcceleration = FVector::ZeroVector;

            // 鐟欙綁鍣撮敍姘崇殶閻?`UE_LOG` 鏉堟挸鍤拫鍐槸閺冦儱绻旈敍灞肩┒娴滃氦绻嶇悰灞炬鐟欏倸鐧傞悩鑸碘偓浣告嫲閹烘帗鐓￠梻顕€顣介妴?
            UE_LOG(LogTemp, Log, TEXT("[DroneMovement] Ground locked"));
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
        }
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼绮伴崙鍝勫幑鎼存洖鍨庨弨顖ょ礉瑜版挷绗傞棃銏㈡畱閺夆€叉闁垝绗夊陇鍐婚弮鑸靛⒔鐞涘被鈧?
        else
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
        {
            // 瀵鐑﹂崫宥呯安閿涙艾寮藉鐟扮€惄鎾偓鐔峰
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯锛愰弰搴㈠灇閸涙ɑ鍨ㄧ仦鈧柈銊ュ綁闁?`Restitution`閿涘瞼鏁ゆ禍搴濈箽鐎涙Μestitution閵?
            double Restitution = Parameters.Restitution;
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `CurrentState.Vz`閿涘苯鐣幋?vz 閻ㄥ嫭娲块弬鑸偓?
            CurrentState.Vz = -CurrentState.Vz * Restitution;

            // 閹解晜鎽濈悰鏉垮櫤濮樻潙閽╅柅鐔峰
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯锛愰弰搴㈠灇閸涙ɑ鍨ㄧ仦鈧柈銊ュ綁闁?`FrictionFactor`閿涘瞼鏁ゆ禍搴濈箽鐎涙rictionfactor閵?
            double FrictionFactor = 1.0 - Parameters.Friction * 0.5;
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `CurrentState.Vx`閿涘苯鐣幋?vx 閻ㄥ嫭娲块弬鑸偓?
            CurrentState.Vx *= FrictionFactor;
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `CurrentState.Vy`閿涘苯鐣幋?vy 閻ㄥ嫭娲块弬鑸偓?
            CurrentState.Vy *= FrictionFactor;

            // 鐟欐帡鈧喎瀹崇悰鏉垮櫤
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `CurrentState.AngRollRate`閿涘苯鐣幋?angrollrate 閻ㄥ嫭娲块弬鑸偓?
            CurrentState.AngRollRate *= 0.9;
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `CurrentState.AngPitchRate`閿涘苯鐣幋?angpitchrate 閻ㄥ嫭娲块弬鑸偓?
            CurrentState.AngPitchRate *= 0.9;
            // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `CurrentState.AngYawRate`閿涘苯鐣幋?angyawrate 閻ㄥ嫭娲块弬鑸偓?
            CurrentState.AngYawRate *= 0.9;
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
        }
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
    }
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
}

/**
 * @brief 娴ｅ秶鐤嗛幒褍鍩楅悳?PD閹貉冨煑閸?
 * @param PositionCommand 閻╊喗鐖ｆ担宥囩枂
 * @return 闁喎瀹抽崨鎴掓姢
 */
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯鐣炬稊澶婂毐閺?`PositionLoop`閿涘苯绱戞慨瀣杽閻滅殐ositionloop閻ㄥ嫬鍙挎担鎾烩偓鏄忕帆閵?
FVector UDroneMovementComponent::PositionLoop(const FVector& PositionCommand)
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
{
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `FVector Pos`閿涘苯鐣幋?fvectorpos 閻ㄥ嫭娲块弬鑸偓?
    FVector Pos = CurrentState.GetPosition();
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `double VxCmd`閿涘苯鐣幋?doublevxcmd 閻ㄥ嫭娲块弬鑸偓?
    double VxCmd = PxController ? PxController->Update(PositionCommand.X, Pos.X) : 0.0;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `double VyCmd`閿涘苯鐣幋?doublevycmd 閻ㄥ嫭娲块弬鑸偓?
    double VyCmd = PyController ? PyController->Update(PositionCommand.Y, Pos.Y) : 0.0;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `double VzCmd`閿涘苯鐣幋?doublevzcmd 閻ㄥ嫭娲块弬鑸偓?
    double VzCmd = PzController ? PzController->Update(PositionCommand.Z, Pos.Z) : 0.0;
    // 鐟欙綁鍣撮敍姘崇殶閻?`VelocityCommand` 閹笛嗩攽瑜版挸澧犲銉╊€冮棁鈧憰浣烘畱閸旂喕鍏橀柅鏄忕帆閵?
    FVector VelocityCommand(VxCmd, VyCmd, VzCmd);
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘矂鈧俺绻?`FMath::Max` 缂?`const double MaxSpeed` 閺傝棄濮炴稉瀣櫕缁撅附娼敍宀勪缉閸?constdoublemaxspeed 鏉╁洤鐨妴?
    const double MaxSpeed = FMath::Max(0.1, TargetPositionSpeedLimit);
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼绮伴崙?`if` 閺夆€叉閸掋倖鏌囬敍灞藉涧閺堝娼禒鑸靛灇缁斿妞傞幍宥勭窗鏉╂稑鍙嗘稉瀣桨閻ㄥ嫬鍨庨弨顖炩偓鏄忕帆閵?
    if (VelocityCommand.SizeSquared() > MaxSpeed * MaxSpeed)
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
    {
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `VelocityCommand`閿涘苯鐣幋?velocity閸涙垝鎶?閻ㄥ嫭娲块弬鑸偓?
        VelocityCommand = VelocityCommand.GetSafeNormal() * MaxSpeed;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
    }
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘矁绻戦崶鐐茬秼閸撳秴鍤遍弫鎵畱鐠侊紕鐣荤紒鎾寸亯閿涘本濡搁幒褍鍩楅弶鍐ф唉閸ョ偠鐨熼悽銊︽煙閵?
    return VelocityCommand;
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
}

/**
 * @brief 闁喎瀹抽幒褍鍩楅悳?PID閹貉冨煑閸?
 * @param VelocityCommand 閻╊喗鐖ｉ柅鐔峰
 * @param OutThrust 鏉堟挸鍤幒銊ュ
 * @return 閸旂娀鈧喎瀹抽崨鎴掓姢
 * Z 鏉炴潙濮為柅鐔峰閸涙垝鎶ょ紒蹇氱箖闁插秴濮忕悰銉ヤ缉閿涙瓖hrust = m * (Az + g)
 * 閸婄偓鏋╃悰銉ヤ缉閿涙艾鐤勯梽鍛腹閸?= Thrust / (cosRoll * cosPitch)
 */
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯鐣炬稊澶婂毐閺?`VelocityLoop`閿涘苯绱戞慨瀣杽閻滅殢elocityloop閻ㄥ嫬鍙挎担鎾烩偓鏄忕帆閵?
FVector UDroneMovementComponent::VelocityLoop(const FVector& VelocityCommand, double& OutThrust)
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
{
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `FVector Vel`閿涘苯鐣幋?fvectorvel 閻ㄥ嫭娲块弬鑸偓?
    FVector Vel = CurrentState.GetVelocity();
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `double AxCmd`閿涘苯鐣幋?doubleaxcmd 閻ㄥ嫭娲块弬鑸偓?
    double AxCmd = VxController ? VxController->Update(VelocityCommand.X, Vel.X) : 0.0;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `double AyCmd`閿涘苯鐣幋?doubleaycmd 閻ㄥ嫭娲块弬鑸偓?
    double AyCmd = VyController ? VyController->Update(VelocityCommand.Y, Vel.Y) : 0.0;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `double AzCmd`閿涘苯鐣幋?doubleazcmd 閻ㄥ嫭娲块弬鑸偓?
    double AzCmd = VzController ? VzController->Update(VelocityCommand.Z, Vel.Z) : 0.0;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `OutThrust`閿涘苯鐣幋?outthrust 閻ㄥ嫭娲块弬鑸偓?
    OutThrust = Parameters.Mass * (AzCmd + Parameters.Gravity);
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘矂鈧俺绻?`FMath::Max` 缂?`OutThrust` 閺傝棄濮炴稉瀣櫕缁撅附娼敍宀勪缉閸?outthrust 鏉╁洤鐨妴?
    OutThrust = FMath::Max(0.0, OutThrust);
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `FRotator CurrentRot`閿涘苯鐣幋?frotatorcurrentrot 閻ㄥ嫭娲块弬鑸偓?
    FRotator CurrentRot = CurrentState.GetRotator();
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `double CosRoll`閿涘苯鐣幋?doublecosroll 閻ㄥ嫭娲块弬鑸偓?
    double CosRoll = FMath::Cos(FMath::DegreesToRadians(CurrentRot.Roll));
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `double CosPitch`閿涘苯鐣幋?doublecospitch 閻ㄥ嫭娲块弬鑸偓?
    double CosPitch = FMath::Cos(FMath::DegreesToRadians(CurrentRot.Pitch));
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯锛愰弰搴㈠灇閸涙ɑ鍨ㄧ仦鈧柈銊ュ綁闁?`CosAttitude`閿涘瞼鏁ゆ禍搴濈箽鐎涙osattitude閵?
    double CosAttitude = CosRoll * CosPitch;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼绮伴崙?`if` 閺夆€叉閸掋倖鏌囬敍灞藉涧閺堝娼禒鑸靛灇缁斿妞傞幍宥勭窗鏉╂稑鍙嗘稉瀣桨閻ㄥ嫬鍨庨弨顖炩偓鏄忕帆閵?
    if (CosAttitude > 0.5) OutThrust /= CosAttitude;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘矁绻戦崶鐐茬秼閸撳秴鍤遍弫鎵畱鐠侊紕鐣荤紒鎾寸亯閿涘本濡搁幒褍鍩楅弶鍐ф唉閸ョ偠鐨熼悽銊︽煙閵?
    return FVector(AxCmd, AyCmd, AzCmd);
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
}

/**
 * @brief 婵寧鈧焦甯堕崚鍓佸箚 PD閹貉冨煑閸?
 * @param AccelerationCommand 閺堢喐婀滈崝鐘烩偓鐔峰
 * @return 鐟欐帡鈧喓宸奸崨鎴掓姢
 * 娴犲孩婀￠張娑欐寜楠炲啿濮為柅鐔峰鐠侊紕鐣婚張鐔告箿閸婄偓鏋╃憴鎺戝閿?
 *   RollDes = atan2(-Ay, g)  閳ユ柡鈧?閸欏磭些闂団偓鐟曚礁褰搁崐?
 *   PitchDes = atan2(Ax, g)  閳ユ柡鈧?閸撳秷绻橀棁鈧憰浣风秵婢?
 */
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯鐣炬稊澶婂毐閺?`AttitudeLoop`閿涘苯绱戞慨瀣杽閻滅櫘ttitudeloop閻ㄥ嫬鍙挎担鎾烩偓鏄忕帆閵?
FVector UDroneMovementComponent::AttitudeLoop(const FVector& AccelerationCommand)
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
{
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯锛愰弰搴㈠灇閸涙ɑ鍨ㄧ仦鈧柈銊ュ綁闁?`AxDes`閿涘瞼鏁ゆ禍搴濈箽鐎涙xdes閵?
    double AxDes = AccelerationCommand.X;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯锛愰弰搴㈠灇閸涙ɑ鍨ㄧ仦鈧柈銊ュ綁闁?`AyDes`閿涘瞼鏁ゆ禍搴濈箽鐎涙ydes閵?
    double AyDes = AccelerationCommand.Y;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯锛愰弰搴㈠灇閸涙ɑ鍨ㄧ仦鈧柈銊ュ綁闁?`g`閿涘瞼鏁ゆ禍搴濈箽鐎涙オ閵?
    double g = Parameters.Gravity;
    const double CurrentYaw = FMath::DegreesToRadians(CurrentState.GetRotator().Yaw);
    const double CosYaw = FMath::Cos(CurrentYaw);
    const double SinYaw = FMath::Sin(CurrentYaw);
    const double BodyAxDes = CosYaw * AxDes + SinYaw * AyDes;
    const double BodyAyDes = -SinYaw * AxDes + CosYaw * AyDes;

    // 娴犲孩婀￠張娑欐寜楠炲啿濮為柅鐔峰鐠侊紕鐣婚張鐔告箿閸婄偓鏋╃憴?
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `double RollDes`閿涘苯鐣幋?doublerolldes 閻ㄥ嫭娲块弬鑸偓?
    double RollDes = FMath::Atan2(-BodyAyDes, g);
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `double PitchDes`閿涘苯鐣幋?doublepitchdes 閻ㄥ嫭娲块弬鑸偓?
    double PitchDes = FMath::Atan2(BodyAxDes, g);

    // 閼奉亜濮╅崑蹇氬焻鐟欐帒鍑￠崷?ControlUpdate 娑擃厺绮犻惄顔界垼閺傜懓鎮滅拋锛勭暬婵傛枻绱欑€涙ê婀?DesiredYaw閿?
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯锛愰弰搴㈠灇閸涙ɑ鍨ㄧ仦鈧柈銊ュ綁闁?`YawDes`閿涘瞼鏁ゆ禍搴濈箽鐎涙Χawdes閵?
    double YawDes = DesiredYaw;

    // 闂勬劕鍩楅張鈧径褍鈧偓鏋╃憴鎺炵礄缁?2鎺抽敍灞剧槷 AirSim 姒涙顓婚惃?~15鎺?缁嬪秳绻氱€瑰牞绱濈涵顔荤箽楠炶櫕绮﹂敍?
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯鍘涚€电顓哥粻妤冪波閺嬫粌浠涢梽鎰畽閿涘苯鍟€閸愭瑥鍙?`RollDes`閿涘矂妲诲?rolldes 鐡掑懎鍤崗浣筋啅閼煎啫娲块妴?
    const double MaxTiltRad = 0.42;
    RollDes = FMath::Clamp(RollDes, -MaxTiltRad, MaxTiltRad);
    PitchDes = FMath::Clamp(PitchDes, -MaxTiltRad, MaxTiltRad);

    // UE FRotator 閻?Roll/Pitch 濮濓絾鏌熼崥鎴滅瑢閹貉冨煑閸掑棝鍘ら惌鈺呮█ G 閻ㄥ嫬濮忛惌鈺冾儊閸欓娴夐崣?
    // 閸欐牕寮芥担?PD 閹貉冨煑閸ｃ劎娈戠拠顖氭▕閺傜懓鎮滄稉搴＄杽闂勫懎濮忛惌鈺傛煙閸氭垳绔撮懛?
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `FRotator CurrentRot`閿涘苯鐣幋?frotatorcurrentrot 閻ㄥ嫭娲块弬鑸偓?
    FRotator CurrentRot = CurrentState.GetRotator();
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `double Roll`閿涘苯鐣幋?doubleroll 閻ㄥ嫭娲块弬鑸偓?
    double Roll = -FMath::DegreesToRadians(CurrentRot.Roll);
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `double Pitch`閿涘苯鐣幋?doublepitch 閻ㄥ嫭娲块弬鑸偓?
    double Pitch = -FMath::DegreesToRadians(CurrentRot.Pitch);
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `double Yaw`閿涘苯鐣幋?doubleyaw 閻ㄥ嫭娲块弬鑸偓?
    double Yaw = FMath::DegreesToRadians(CurrentRot.Yaw);

    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `double RollTarget`閿涘苯鐣幋?doublerolltarget 閻ㄥ嫭娲块弬鑸偓?
    double RollTarget = Roll + NormalizeAngle(RollDes - Roll);
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `double PitchTarget`閿涘苯鐣幋?doublepitchtarget 閻ㄥ嫭娲块弬鑸偓?
    double PitchTarget = Pitch + NormalizeAngle(PitchDes - Pitch);
    double RollRateCmd = RollController ? RollController->Update(RollTarget, Roll) : 0.0;
    double PitchRateCmd = PitchController ? PitchController->Update(PitchTarget, Pitch) : 0.0;
    double YawRateCmd = CommandedYawRate;
    if (YawMode != EDroneYawMode::Rate)
    {
        const double YawTarget = Yaw + NormalizeAngle(YawDes - Yaw);
        YawRateCmd = YawController ? YawController->Update(YawTarget, Yaw) : 0.0;
    }
    return FVector(RollRateCmd, PitchRateCmd, YawRateCmd);
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
}

/**
 * @brief 鐟欐帡鈧喓宸奸幒褍鍩楅悳?PID閹貉冨煑閸?
 * @param AngularVelocityCommand 閻╊喗鐖ｇ憴鎺椻偓鐔哄芳
 * @return 閸旀稓鐓╅崨鎴掓姢
 */
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯鐣炬稊澶婂毐閺?`AngularVelocityLoop`閿涘苯绱戞慨瀣杽閻滅櫘ngularvelocityloop閻ㄥ嫬鍙挎担鎾烩偓鏄忕帆閵?
FVector UDroneMovementComponent::AngularVelocityLoop(const FVector& AngularVelocityCommand)
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
{
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `double TorqueX`閿涘苯鐣幋?doubletorqueX 閻ㄥ嫭娲块弬鑸偓?
    double TorqueX = RollRateController ? RollRateController->Update(AngularVelocityCommand.X, CurrentState.AngRollRate) : 0.0;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `double TorqueY`閿涘苯鐣幋?doubletorqueY 閻ㄥ嫭娲块弬鑸偓?
    double TorqueY = PitchRateController ? PitchRateController->Update(AngularVelocityCommand.Y, CurrentState.AngPitchRate) : 0.0;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `double TorqueZ`閿涘苯鐣幋?doubletorqueZ 閻ㄥ嫭娲块弬鑸偓?
    double TorqueZ = YawRateController ? YawRateController->Update(AngularVelocityCommand.Z, CurrentState.AngYawRate) : 0.0;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘矁绻戦崶鐐茬秼閸撳秴鍤遍弫鎵畱鐠侊紕鐣荤紒鎾寸亯閿涘本濡搁幒褍鍩楅弶鍐ф唉閸ョ偠鐨熼悽銊︽煙閵?
    return FVector(TorqueX, TorqueY, TorqueZ);
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
}

/**
 * @brief 鐠佸墽鐤嗛崚婵嗩潗閻樿埖鈧?
 * @param InitialState 閸掓繂顫愰悩鑸碘偓?
 * 鐠佹澘缍嶇挧宄邦潗妤傛ê瀹抽悽銊ょ艾閸︿即娼扮喊鐗堟寬濡偓濞村绱濋崚婵嗩潗閸栨牜鏁搁張鐑樻姢濞夈垹娅?
 */
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯鐣炬稊澶婂毐閺?`SetInitialState`閿涘苯绱戞慨瀣杽閻滅殜etinitial閻樿埖鈧胶娈戦崗铚傜秼闁槒绶妴?
void UDroneMovementComponent::SetInitialState(const FDroneState& InitialState)
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
{
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `CurrentState`閿涘苯鐣幋?current閻樿埖鈧?閻ㄥ嫭娲块弬鑸偓?
    CurrentState = InitialState;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `double HoverSpeed`閿涘苯鐣幋?doublehoverspeed 閻ㄥ嫭娲块弬鑸偓?
    double HoverSpeed = Parameters.GetHoverMotorSpeed();
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼绮伴崙?`if` 閺夆€叉閸掋倖鏌囬敍灞藉涧閺堝娼禒鑸靛灇缁斿妞傞幍宥勭窗鏉╂稑鍙嗘稉瀣桨閻ㄥ嫬鍨庨弨顖炩偓鏄忕帆閵?
    bool bNeedSeedHoverMotor = CurrentState.MotorSpeeds.Num() < 4;
    if (!bNeedSeedHoverMotor)
    {
        bNeedSeedHoverMotor = true;
        for (double MotorSpeed : CurrentState.MotorSpeeds)
        {
            if (FMath::Abs(MotorSpeed) > SMALL_NUMBER)
            {
                bNeedSeedHoverMotor = false;
                break;
            }
        }
    }
    // 初始电机如果还是全零，就注入悬停转速，避免控制器第一步拿到一个“坠落态”。
    if (bNeedSeedHoverMotor)
    {
        CurrentState.MotorSpeeds = {HoverSpeed, HoverSpeed, HoverSpeed, HoverSpeed};
    }

    // 鐠佹澘缍嶇挧宄邦潗閸︿即娼版妯哄閿涙氨鏁ら崚婵嗩潗 Z 娴ｅ秶鐤嗘担婊€璐熼崷浼存桨閸欏倽鈧?
    // 閺冪姳姹夐張鍝勫灥婵鏁撻幋鎰躬閸︿即娼版稉濠忕礉鐠у嘲顫愭稉?Ground Lock 閻樿埖鈧?
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `InitialGroundZ`閿涘苯鐣幋?initialgroundZ 閻ㄥ嫭娲块弬鑸偓?
    InitialGroundZ = CurrentState.Z;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `bGrounded`閿涘苯鐣幋?鐢啫鐨甸弽鍥х箶 grounded 閻ㄥ嫭娲块弬鑸偓?
    bGrounded = true;

    // 閸掓繂顫愰崠鏍暩閺堢儤鎶ゅ▔銏犳珤閸掔増鍋撻崑婊嗘祮闁?
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `MotorSpeedsFiltered`閿涘苯鐣幋?motorspeedsfiltered 閻ㄥ嫭娲块弬鑸偓?
    MotorSpeedsFiltered = CurrentState.MotorSpeeds;

    // 濞撳懘娴傞崝鐘烩偓鐔峰閸樺棗褰?
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `PrevLinearAcceleration`閿涘苯鐣幋?prevlinearacceleration 閻ㄥ嫭娲块弬鑸偓?
    PrevLinearAcceleration = FVector::ZeroVector;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `PrevAngularAcceleration`閿涘苯鐣幋?prevangularacceleration 閻ㄥ嫭娲块弬鑸偓?
    PrevAngularAcceleration = FVector::ZeroVector;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `FixedStepAccumulator`閿涘苯鐣幋?fixedstepaccumulator 閻ㄥ嫭娲块弬鑸偓?
    FixedStepAccumulator = 0.0;

    // 闁插秶鐤嗛懛顏勫З閸嬪繗鍩呴悩鑸碘偓?
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `LockedYaw`閿涘苯鐣幋?lockedyaw 閻ㄥ嫭娲块弬鑸偓?
    LockedYaw = FMath::DegreesToRadians(CurrentState.GetRotator().Yaw);
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `CommandedYaw`閿涘苯鐣幋?commandedyaw 閻ㄥ嫭娲块弬鑸偓?
    CommandedYaw = LockedYaw;
    CommandedYawRate = 0.0;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `bYawInitialized`閿涘苯鐣幋?鐢啫鐨甸弽鍥х箶 yawinitialized 閻ㄥ嫭娲块弬鑸偓?
    bYawInitialized = false;

    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘奔缍呮禍搴㈢€柅鐘插毐閺佹澘鍨垫慨瀣閸掓銆冩稉顓ㄧ礉閹?`UE_LOG` 閻╁瓨甯撮崚婵嗩潗閸栨牔璐?`LogTemp, Log, TEXT("[DroneMovement] InitialState set, HoverSpeed=%.1f rad/s, GroundZ=%.2f, Grounded=true"`閿涘苯鍣虹亸鎴ｇ箻閸忋儱鍤遍弫棰佺秼閸氬海娈戞０婵嗩樆鐠у鈧厧绱戦柨鈧妴?
    UE_LOG(LogTemp, Log, TEXT("[DroneMovement] InitialState set, HoverSpeed=%.1f rad/s, GroundZ=%.2f, Grounded=true"),
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘矁鎯ょ€圭偛缍嬮崜宥喣侀崸妞捐厬閻ㄥ嫬鍙挎担鎾崇杽閻滄壆绮忛懞鍌︾礉娑撹桨绗傞棃銏㈡畱婢圭増妲戦妴浣稿彆瀵繑鍨ㄩ幒褍鍩楀ù浣衡柤閹绘劒绶电€圭偤妾幍褑顢戠拠顓炲綖閵?
        HoverSpeed, InitialGroundZ);
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
}

/**
 * @brief 閸掑洦宕查幒褍鍩楀Ο鈥崇础楠炲爼鍣哥純顔藉閺堝甯堕崚璺烘珤
 * @param NewMode 閺傛壆娈戦幒褍鍩楀Ο鈥崇础
 */
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯鐣炬稊澶婂毐閺?`SetControlMode`閿涘苯绱戞慨瀣杽閻滅殜etcontrol濡€崇础閻ㄥ嫬鍙挎担鎾烩偓鏄忕帆閵?
void UDroneMovementComponent::SetControlMode(EDroneControlMode NewMode)
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
{
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `CurrentControlMode`閿涘苯鐣幋?currentcontrol濡€崇础 閻ㄥ嫭娲块弬鑸偓?
    CurrentControlMode = NewMode;
    // 鐟欙綁鍣撮敍姘崇殶閻?`ResetAllControllers` 閹笛嗩攽瑜版挸澧犲銉╊€冮棁鈧憰浣烘畱閸旂喕鍏橀柅鏄忕帆閵?
    ResetAllControllers();
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `FixedStepAccumulator`閿涘苯鐣幋?fixedstepaccumulator 閻ㄥ嫭娲块弬鑸偓?
    FixedStepAccumulator = 0.0;
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
}

/** @brief 鐠佸墽鐤嗛崢鐔奉潗閹貉冨煑閸涙垝鎶?*/
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘矁鎯ょ€圭偛缍嬮崜宥喣侀崸妞捐厬閻ㄥ嫬鍙挎担鎾崇杽閻滄壆绮忛懞鍌︾礉娑撹桨绗傞棃銏㈡畱婢圭増妲戦妴浣稿彆瀵繑鍨ㄩ幒褍鍩楀ù浣衡柤閹绘劒绶电€圭偤妾幍褑顢戠拠顓炲綖閵?
void UDroneMovementComponent::SetControlCommand(const TArray<double>& Command) { ControlCommands = Command; }

/** @brief 鐠佸墽鐤嗛惄顔界垼娴ｅ秶鐤?*/
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯鐣炬稊澶婂毐閺?`SetTargetPosition`閿涘苯绱戞慨瀣杽閻滅殜ettargetposition閻ㄥ嫬鍙挎担鎾烩偓鏄忕帆閵?
void UDroneMovementComponent::SetTargetPosition(const FVector& TargetPos, float Speed)
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
{
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `TargetPosition`閿涘苯鐣幋?targetposition 閻ㄥ嫭娲块弬鑸偓?
    TargetPosition = TargetPos;

    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼绮伴崙?`if` 閺夆€叉閸掋倖鏌囬敍灞藉涧閺堝娼禒鑸靛灇缁斿妞傞幍宥勭窗鏉╂稑鍙嗘稉瀣桨閻ㄥ嫬鍨庨弨顖炩偓鏄忕帆閵?
    if (Speed > KINDA_SMALL_NUMBER)
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
    {
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘矂鈧俺绻?`FMath::Max` 缂?`PositionAxisSpeedLimit` 閺傝棄濮炴稉瀣櫕缁撅附娼敍宀勪缉閸?positionaxisspeedlimit 鏉╁洤鐨妴?
        PositionAxisSpeedLimit = FMath::Max(0.1, static_cast<double>(Speed));
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
    }
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼绮伴崙鍝勫幑鎼存洖鍨庨弨顖ょ礉瑜版挷绗傞棃銏㈡畱閺夆€叉闁垝绗夊陇鍐婚弮鑸靛⒔鐞涘被鈧?
    else
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
    {
        // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `PositionAxisSpeedLimit`閿涘苯鐣幋?positionaxisspeedlimit 閻ㄥ嫭娲块弬鑸偓?
        PositionAxisSpeedLimit = DefaultPositionSpeedLimit;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
    }

    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `TargetPositionSpeedLimit`閿涘苯鐣幋?targetpositionspeedlimit 閻ㄥ嫭娲块弬鑸偓?
    TargetPositionSpeedLimit = PositionAxisSpeedLimit;

    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼绮伴崙?`if` 閺夆€叉閸掋倖鏌囬敍灞藉涧閺堝娼禒鑸靛灇缁斿妞傞幍宥勭窗鏉╂稑鍙嗘稉瀣桨閻ㄥ嫬鍨庨弨顖炩偓鏄忕帆閵?
    if (PxController) PxController->SetParameters(PxController->Kp, PxController->Kd, PxController->DiffFilterTau, PositionAxisSpeedLimit);
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼绮伴崙?`if` 閺夆€叉閸掋倖鏌囬敍灞藉涧閺堝娼禒鑸靛灇缁斿妞傞幍宥勭窗鏉╂稑鍙嗘稉瀣桨閻ㄥ嫬鍨庨弨顖炩偓鏄忕帆閵?
    if (PyController) PyController->SetParameters(PyController->Kp, PyController->Kd, PyController->DiffFilterTau, PositionAxisSpeedLimit);
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼绮伴崙?`if` 閺夆€叉閸掋倖鏌囬敍灞藉涧閺堝娼禒鑸靛灇缁斿妞傞幍宥勭窗鏉╂稑鍙嗘稉瀣桨閻ㄥ嫬鍨庨弨顖炩偓鏄忕帆閵?
    if (PzController) PzController->SetParameters(PzController->Kp, PzController->Kd, PzController->DiffFilterTau, PositionAxisSpeedLimit);
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
}

/** @brief 鐠佸墽鐤嗛惄顔界垼闁喎瀹?*/
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘矁鎯ょ€圭偛缍嬮崜宥喣侀崸妞捐厬閻ㄥ嫬鍙挎担鎾崇杽閻滄壆绮忛懞鍌︾礉娑撹桨绗傞棃銏㈡畱婢圭増妲戦妴浣稿彆瀵繑鍨ㄩ幒褍鍩楀ù浣衡柤閹绘劒绶电€圭偤妾幍褑顢戠拠顓炲綖閵?
void UDroneMovementComponent::SetTargetVelocity(const FVector& TargetVel) { TargetVelocity = TargetVel; }

/**
 * @brief 鐠佸墽鐤嗛惄顔界垼婵寧鈧礁鎷伴幒銊ュ
 * @param Attitude 閻╊喗鐖ｆ慨鎸庘偓?
 * @param Thrust 閹恒劌濮忕化缁樻殶
 */
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯鐣炬稊澶婂毐閺?`SetTargetAttitude`閿涘苯绱戞慨瀣杽閻滅殜ettargetattitude閻ㄥ嫬鍙挎担鎾烩偓鏄忕帆閵?
void UDroneMovementComponent::SetTargetAttitude(const FRotator& Attitude, float Thrust)
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
{
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `TargetAttitude`閿涘苯鐣幋?targetattitude 閻ㄥ嫭娲块弬鑸偓?
    TargetAttitude = Attitude;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `TargetThrust`閿涘苯鐣幋?targetthrust 閻ㄥ嫭娲块弬鑸偓?
    TargetThrust = Thrust * Parameters.Mass * Parameters.Gravity;
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
}

/**
 * @brief 闁插秶鐤嗛悩鑸碘偓浣歌嫙濞撳懘娴傞幍鈧張澶嬪付閸掕泛娅?
 * @param NewState 閺傛壆娈戦崚婵嗩潗閻樿埖鈧?
 */
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯鐣炬稊澶婂毐閺?`ResetState`閿涘苯绱戞慨瀣杽閻滅殙eset閻樿埖鈧胶娈戦崗铚傜秼闁槒绶妴?
void UDroneMovementComponent::ResetState(const FDroneState& NewState)
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
{
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `CurrentState`閿涘苯鐣幋?current閻樿埖鈧?閻ㄥ嫭娲块弬鑸偓?
    CurrentState = NewState;
    // 鐟欙綁鍣撮敍姘崇殶閻?`ResetAllControllers` 閹笛嗩攽瑜版挸澧犲銉╊€冮棁鈧憰浣烘畱閸旂喕鍏橀柅鏄忕帆閵?
    ResetAllControllers();
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `FixedStepAccumulator`閿涘苯鐣幋?fixedstepaccumulator 閻ㄥ嫭娲块弬鑸偓?
    FixedStepAccumulator = 0.0;

    // 闁插秶鐤嗛懛顏勫З閸嬪繗鍩呴悩鑸碘偓?
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `LockedYaw`閿涘苯鐣幋?lockedyaw 閻ㄥ嫭娲块弬鑸偓?
    LockedYaw = FMath::DegreesToRadians(NewState.GetRotator().Yaw);
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `CommandedYaw`閿涘苯鐣幋?commandedyaw 閻ㄥ嫭娲块弬鑸偓?
    CommandedYaw = LockedYaw;
    CommandedYawRate = 0.0;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘本濡搁崣鍏呮櫠鐞涖劏鎻蹇曟畱缂佹挻鐏夐崘娆忓弳 `bYawInitialized`閿涘苯鐣幋?鐢啫鐨甸弽鍥х箶 yawinitialized 閻ㄥ嫭娲块弬鑸偓?
    bYawInitialized = false;
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
}

/** @brief 闁插秶鐤嗛幍鈧張?12 娑擃亝甯堕崚璺烘珤閻ㄥ嫬鍞撮柈銊уЦ閹?*/
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯鐣炬稊澶婂毐閺?`ResetAllControllers`閿涘苯绱戞慨瀣杽閻滅殙esetallcontrollers閻ㄥ嫬鍙挎担鎾烩偓鏄忕帆閵?
void UDroneMovementComponent::ResetAllControllers()
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
{
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼绮伴崙?`if` 閺夆€叉閸掋倖鏌囬敍灞藉涧閺堝娼禒鑸靛灇缁斿妞傞幍宥勭窗鏉╂稑鍙嗘稉瀣桨閻ㄥ嫬鍨庨弨顖炩偓鏄忕帆閵?
    if (PxController) PxController->Reset();
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼绮伴崙?`if` 閺夆€叉閸掋倖鏌囬敍灞藉涧閺堝娼禒鑸靛灇缁斿妞傞幍宥勭窗鏉╂稑鍙嗘稉瀣桨閻ㄥ嫬鍨庨弨顖炩偓鏄忕帆閵?
    if (PyController) PyController->Reset();
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼绮伴崙?`if` 閺夆€叉閸掋倖鏌囬敍灞藉涧閺堝娼禒鑸靛灇缁斿妞傞幍宥勭窗鏉╂稑鍙嗘稉瀣桨閻ㄥ嫬鍨庨弨顖炩偓鏄忕帆閵?
    if (PzController) PzController->Reset();
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼绮伴崙?`if` 閺夆€叉閸掋倖鏌囬敍灞藉涧閺堝娼禒鑸靛灇缁斿妞傞幍宥勭窗鏉╂稑鍙嗘稉瀣桨閻ㄥ嫬鍨庨弨顖炩偓鏄忕帆閵?
    if (VxController) VxController->Reset();
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼绮伴崙?`if` 閺夆€叉閸掋倖鏌囬敍灞藉涧閺堝娼禒鑸靛灇缁斿妞傞幍宥勭窗鏉╂稑鍙嗘稉瀣桨閻ㄥ嫬鍨庨弨顖炩偓鏄忕帆閵?
    if (VyController) VyController->Reset();
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼绮伴崙?`if` 閺夆€叉閸掋倖鏌囬敍灞藉涧閺堝娼禒鑸靛灇缁斿妞傞幍宥勭窗鏉╂稑鍙嗘稉瀣桨閻ㄥ嫬鍨庨弨顖炩偓鏄忕帆閵?
    if (VzController) VzController->Reset();
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼绮伴崙?`if` 閺夆€叉閸掋倖鏌囬敍灞藉涧閺堝娼禒鑸靛灇缁斿妞傞幍宥勭窗鏉╂稑鍙嗘稉瀣桨閻ㄥ嫬鍨庨弨顖炩偓鏄忕帆閵?
    if (RollController) RollController->Reset();
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼绮伴崙?`if` 閺夆€叉閸掋倖鏌囬敍灞藉涧閺堝娼禒鑸靛灇缁斿妞傞幍宥勭窗鏉╂稑鍙嗘稉瀣桨閻ㄥ嫬鍨庨弨顖炩偓鏄忕帆閵?
    if (PitchController) PitchController->Reset();
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼绮伴崙?`if` 閺夆€叉閸掋倖鏌囬敍灞藉涧閺堝娼禒鑸靛灇缁斿妞傞幍宥勭窗鏉╂稑鍙嗘稉瀣桨閻ㄥ嫬鍨庨弨顖炩偓鏄忕帆閵?
    if (YawController) YawController->Reset();
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼绮伴崙?`if` 閺夆€叉閸掋倖鏌囬敍灞藉涧閺堝娼禒鑸靛灇缁斿妞傞幍宥勭窗鏉╂稑鍙嗘稉瀣桨閻ㄥ嫬鍨庨弨顖炩偓鏄忕帆閵?
    if (RollRateController) RollRateController->Reset();
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼绮伴崙?`if` 閺夆€叉閸掋倖鏌囬敍灞藉涧閺堝娼禒鑸靛灇缁斿妞傞幍宥勭窗鏉╂稑鍙嗘稉瀣桨閻ㄥ嫬鍨庨弨顖炩偓鏄忕帆閵?
    if (PitchRateController) PitchRateController->Reset();
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼绮伴崙?`if` 閺夆€叉閸掋倖鏌囬敍灞藉涧閺堝娼禒鑸靛灇缁斿妞傞幍宥勭窗鏉╂稑鍙嗘稉瀣桨閻ㄥ嫬鍨庨弨顖炩偓鏄忕帆閵?
    if (YawRateController) YawRateController->Reset();
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
}

/**
 * @brief 鐠佸墽鐤嗘担宥囩枂閹貉冨煑閸?PD 婢х偟娉?
 * @param Kp 濮ｆ柧绶ユ晶鐐垫抄
 * @param Kd 瀵邦喖鍨庢晶鐐垫抄
 */
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯鐣炬稊澶婂毐閺?`SetPositionGains`閿涘苯绱戞慨瀣杽閻滅殜etpositiongains閻ㄥ嫬鍙挎担鎾烩偓鏄忕帆閵?
void UDroneMovementComponent::SetPositionGains(float Kp, float Kd)
{
    if (PxController) PxController->SetParameters(Kp, Kd, 0.12, PositionAxisSpeedLimit);
    if (PyController) PyController->SetParameters(Kp, Kd, 0.12, PositionAxisSpeedLimit);
    if (PzController) PzController->SetParameters(Kp, Kd, 0.12, FMath::Max(1.2, PositionAxisSpeedLimit));
}

/**
 * @brief 鐠佸墽鐤嗛柅鐔峰閹貉冨煑閸?PID 婢х偟娉?
 * @param Kp 濮ｆ柧绶ユ晶鐐垫抄
 * @param Ki 缁夘垰鍨庢晶鐐垫抄
 * @param Kd 瀵邦喖鍨庢晶鐐垫抄
 */
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯鐣炬稊澶婂毐閺?`SetVelocityGains`閿涘苯绱戞慨瀣杽閻滅殜etvelocitygains閻ㄥ嫬鍙挎担鎾烩偓鏄忕帆閵?
void UDroneMovementComponent::SetVelocityGains(float Kp, float Ki, float Kd)
{
    if (VxController)
    {
        VxController->SetParameters(Kp, Ki, Kd, 0.10, 3.2);
        VxController->SetIntegratorLimits(-0.8, 0.8);
    }
    if (VyController)
    {
        VyController->SetParameters(Kp, Ki, Kd, 0.10, 3.2);
        VyController->SetIntegratorLimits(-0.8, 0.8);
    }
    if (VzController)
    {
        VzController->SetParameters(Kp, Ki, Kd, 0.10, 6.2);
        VzController->SetIntegratorLimits(-0.8, 0.8);
    }
}

/**
 * @brief 鐠佸墽鐤嗘慨鎸庘偓浣瑰付閸掕泛娅?PD 婢х偟娉?
 * @param Kp 濮ｆ柧绶ユ晶鐐垫抄
 * @param Kd 瀵邦喖鍨庢晶鐐垫抄
 */
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯鐣炬稊澶婂毐閺?`SetAttitudeGains`閿涘苯绱戞慨瀣杽閻滅殜etattitudegains閻ㄥ嫬鍙挎担鎾烩偓鏄忕帆閵?
void UDroneMovementComponent::SetAttitudeGains(float Kp, float Kd)
{
    if (RollController) RollController->SetParameters(Kp, Kd, 0.10, PI / 3.0);
    if (PitchController) PitchController->SetParameters(Kp, Kd, 0.10, PI / 3.0);
    if (YawController) YawController->SetParameters(Kp * 0.9f, Kd * 0.7f, 0.10, PI / 2.0);
}

/**
 * @brief 鐠佸墽鐤嗙憴鎺椻偓鐔哄芳閹貉冨煑閸ｃ劌顤冮惄?
 * @param Kp 濮ｆ柧绶ユ晶鐐垫抄
 */
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯鐣炬稊澶婂毐閺?`SetAngleRateGains`閿涘苯绱戞慨瀣杽閻滅殜etanglerategains閻ㄥ嫬鍙挎担鎾烩偓鏄忕帆閵?
void UDroneMovementComponent::SetAngleRateGains(float Kp)
{
    if (RollRateController)
    {
        RollRateController->SetParameters(Kp, 0.0, 0.0, 0.08, 0.32);
        RollRateController->SetIntegratorLimits(-0.5, 0.5);
    }
    if (PitchRateController)
    {
        PitchRateController->SetParameters(Kp, 0.0, 0.0, 0.08, 0.32);
        PitchRateController->SetIntegratorLimits(-0.5, 0.5);
    }
    if (YawRateController)
    {
        YawRateController->SetParameters(Kp * 1.25f, 0.0, 0.0, 0.08, 0.22);
        YawRateController->SetIntegratorLimits(-0.4, 0.4);
    }
}

// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯鐣炬稊澶婂毐閺?`SetHeadingControl`閿涘苯绱戞慨瀣杽閻滅殜etheadingcontrol閻ㄥ嫬鍙挎担鎾烩偓鏄忕帆閵?
void UDroneMovementComponent::SetHeadingControl(EDroneYawMode NewYawMode, EDroneDrivetrainMode NewDrivetrain, float YawDeg)
{
    YawMode = NewYawMode;
    DrivetrainMode = NewDrivetrain;
    CommandedYawRate = 0.0;

    if (YawMode == EDroneYawMode::Angle)
    {
        CommandedYaw = FMath::DegreesToRadians(YawDeg);
        LockedYaw = CommandedYaw;
        bYawInitialized = true;
    }
    else if (YawMode == EDroneYawMode::Hold)
    {
        LockedYaw = FMath::DegreesToRadians(CurrentState.GetRotator().Yaw);
        bYawInitialized = true;
    }
    else if (YawMode == EDroneYawMode::Rate)
    {
        CommandedYawRate = FMath::DegreesToRadians(YawDeg);
        LockedYaw = FMath::DegreesToRadians(CurrentState.GetRotator().Yaw);
        DesiredYaw = LockedYaw;
        bYawInitialized = true;
    }
}

/**
 * @brief 鐏忓棙婧€娴ｆ挸娼楅弽鍥╅兇閸氭垿鍣洪弮瀣祮閸掗绗橀悾灞芥綏閺嶅洨閮?
 * @param BodyVector 閺堣桨缍嬮崸鎰垼缁鎮滈柌?
 * @param Orientation 婵寧鈧礁娲撻崗鍐╂殶
 * @return 娑撴牜鏅崸鎰垼缁鎮滈柌?
 */
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯鐣炬稊澶婂毐閺?`RotateBodyToWorld`閿涘苯绱戞慨瀣杽閻滅殙otatebodytoworld閻ㄥ嫬鍙挎担鎾烩偓鏄忕帆閵?
FVector UDroneMovementComponent::RotateBodyToWorld(const FVector& BodyVector, const FQuat& Orientation)
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
{
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘矁绻戦崶鐐茬秼閸撳秴鍤遍弫鎵畱鐠侊紕鐣荤紒鎾寸亯閿涘本濡搁幒褍鍩楅弶鍐ф唉閸ョ偠鐨熼悽銊︽煙閵?
    return Orientation.RotateVector(BodyVector);
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
}

/**
 * @brief 鐟欐帒瀹宠ぐ鎺嶇閸栨牕鍩?[-锜? 锜篯 閼煎啫娲?
 * @param AngleRad 鏉堟挸鍙嗙憴鎺戝
 * @return 瑜版帊绔撮崠鏍ф倵閻ㄥ嫯顫楁惔?
 */
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘矁鎯ょ€圭偛缍嬮崜宥喣侀崸妞捐厬閻ㄥ嫬鍙挎担鎾崇杽閻滄壆绮忛懞鍌︾礉娑撹桨绗傞棃銏㈡畱婢圭増妲戦妴浣稿彆瀵繑鍨ㄩ幒褍鍩楀ù浣衡柤閹绘劒绶电€圭偤妾幍褑顢戠拠顓炲綖閵?
double UDroneMovementComponent::NormalizeAngle(double AngleRad)
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
{
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯绱戞慨?`while` 瀵邦亞骞嗛敍灞藉涧鐟曚焦娼禒鏈电箽閹镐椒璐熼惇鐔锋皑閹镐胶鐢婚柌宥咁槻閹笛嗩攽閵?
    while (AngleRad > PI) AngleRad -= 2.0 * PI;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘苯绱戞慨?`while` 瀵邦亞骞嗛敍灞藉涧鐟曚焦娼禒鏈电箽閹镐椒璐熼惇鐔锋皑閹镐胶鐢婚柌宥咁槻閹笛嗩攽閵?
    while (AngleRad < -PI) AngleRad += 2.0 * PI;
    // 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘矁绻戦崶鐐茬秼閸撳秴鍤遍弫鎵畱鐠侊紕鐣荤紒鎾寸亯閿涘本濡搁幒褍鍩楅弶鍐ф唉閸ョ偠鐨熼悽銊︽煙閵?
    return AngleRad;
// 鐟欙綁鍣撮敍姘崇箹娑撯偓鐞涘瞼鏁ゆ禍搴＄磻婵鍨ㄧ紒鎾存将瑜版挸澧犳担婊呮暏閸╃噦绱濋幒褍鍩楃猾姹団偓浣稿毐閺佺増鍨ㄩ弶鈥叉閸ф娈戞潏鍦櫕閵?
}



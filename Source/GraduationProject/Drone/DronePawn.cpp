#include "DronePawn.h"
#include "DroneMovementComponent.h"
#include "DroneApi.h"
#include "GraduationProject/Core/Manager/AgentManager.h"
#include "Misc/Base64.h"
#include "IImageWrapperModule.h"
#include "IImageWrapper.h"
#include "GraduationProject/Vision/CameraCaptureUtils.h"
#include "Components/PrimitiveComponent.h"
#include "HAL/IConsoleManager.h"

/**
 * @brief 闁哄瀚伴埀顒傚Т閸ら亶寮?
 * 闁告帗绋戠紓鎾诲嫉妤﹀潡鐓╃紓鍐╁灦閻楁悂濡存担鍛婄＝濞戞搩浜濆Λ鍡欑礄绾绀勯梺顐ｄ亢缁诲啴骞撻幒鎾旑偊姊介崟顓熺祷闁挎稑顦埀?
 * 闁硅棄瀚崕姘緞?Yaw/Pitch 濞存粍鍨佃ぐ鎾晬閸儮鍋撳宕囩畺闁圭粯甯楄潕闂傚嫬瀚鍐晬婢跺牃鍋?
 * SceneCaptureComponent2D 闁?CineCameraComponent闁挎稑鐗撳顕€鎯堥埀顒勫礆?Pitch 濞存粍鍨佃ぐ鎾晬?
 */
ADronePawn::ADronePawn()
{
    PrimaryActorTick.bCanEverTick = true;

    // 闁冲厜鍋撻柍鍏夊亾 闁哄秴婀辩划宥嗙鐠虹儤瀚查柡鍫ョ細闂?闁冲厜鍋撻柍鍏夊亾
    RootComp = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
    SetRootComponent(RootComp);
    BodyMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("BodyMesh"));
    BodyMesh->SetupAttachment(RootComp);

    // 闁冲厜鍋撻柍鍏夊亾 闁搞儲绋愰柌婊堝籍鐎ｎ剝鈧?闁?闂侇偅淇虹换?BodyMesh 濞戞挸锕﹀▓鎴﹀箵閹烘挃顐︽⒔閸曨厽绲?闁冲厜鍋撻柍鍏夊亾
    Fan0 = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Fan0"));
    Fan0->SetupAttachment(BodyMesh, TEXT("Drone_Fan_002"));
    Fan1 = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Fan1"));
    Fan1->SetupAttachment(BodyMesh, TEXT("Drone_Fan1_002"));
    Fan2 = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Fan2"));
    Fan2->SetupAttachment(BodyMesh, TEXT("Drone_Fan2_002"));
    Fan3 = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Fan3"));
    Fan3->SetupAttachment(BodyMesh, TEXT("Drone_Fan3_002"));

    // 闁冲厜鍋撻柍鍏夊亾 闁硅棄瀚崕姘緞缂堢姷闅橀柛?闁?Yaw 闂傚嫬瀚鍐礆?BodyMesh 闁圭粯甯楄潕闁挎稑顒tch 闂傚嫬瀚鍐礆?Yaw 闁圭粯甯楄潕 闁冲厜鍋撻柍鍏夊亾
    CameraYawMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("CameraYawMesh"));
    CameraYawMesh->SetupAttachment(BodyMesh, TEXT("Camera_Yaw_002"));
    CameraPitchMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("CameraPitchMesh"));
    CameraPitchMesh->SetupAttachment(CameraYawMesh, TEXT("Camera_Pitch_002"));

    // 闁冲厜鍋撻柍鍏夊亾 闁革妇鍎ゅ▍娆撴煂閸ヮ剚鑲犵紓浣稿濞?闁?闂傚嫬瀚鍐礆?Pitch 濞存粍鍨佃ぐ?闁冲厜鍋撻柍鍏夊亾
    DroneSceneCapture = CreateDefaultSubobject<USceneCaptureComponent2D>(TEXT("DroneSceneCapture"));
    DroneSceneCapture->SetupAttachment(CameraPitchMesh);
    DroneSceneCapture->bCaptureEveryFrame = true;
    DroneSceneCapture->bCaptureOnMovement = false;
    DroneSceneCapture->bAlwaysPersistRenderingState = false;
    DroneSceneCapture->PostProcessSettings.bOverride_MotionBlurAmount = true;
    DroneSceneCapture->PostProcessSettings.MotionBlurAmount = 0.0f;
    DroneSceneCapture->PostProcessSettings.bOverride_MotionBlurMax = true;
    DroneSceneCapture->PostProcessSettings.MotionBlurMax = 0.0f;
    DroneSceneCapture->PostProcessSettings.bOverride_MotionBlurPerObjectSize = true;
    DroneSceneCapture->PostProcessSettings.MotionBlurPerObjectSize = 0.0f;
    DroneSceneCapture->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;

    // 闁冲厜鍋撻柍鍏夊亾 CineCamera 闁?闁活潿鍔嬬花顒傜磼瑜庢竟娆撳捶閻戞ɑ鐝?PostProcess 閻犱礁澧介悿?闁冲厜鍋撻柍鍏夊亾
    DroneCineCamera = CreateDefaultSubobject<UCineCameraComponent>(TEXT("DroneCineCamera"));
    DroneCineCamera->SetupAttachment(CameraPitchMesh);
    DroneCineCamera->SetActive(false);

    // 闁冲厜鍋撻柍鍏夊亾 閺夆晜鍔曟慨鈺冪磼閸曨亝顐?闁冲厜鍋撻柍鍏夊亾
    MovementComp = CreateDefaultSubobject<UDroneMovementComponent>(TEXT("Movement"));
}

/**
 * @brief 婵炴挸鎲￠崹娆忣嚕閳ь剚鎱ㄧ€ｎ偅顦ч柛鎺撶箓椤劙宕?
 * 闁告帗绻傞～鎰板礌閺嶎剛绠ラ柛鏂诲妺鐠炪垽鎯囬悢宄版闁轰浇鍩囬埀顑跨閸ㄥ崬顕?RenderTarget 妤犵偛鐖奸崢銈囩磾?SceneCapture闁靛棔鐒﹂弫鐐哄礃?Agent
 */
void ADronePawn::BeginPlay()
{
    Super::BeginPlay();
    if (MovementComp)
    {
        MovementComp->SetParameters(Parameters);
        FDroneState InitState;
        FVector Pos = GetActorLocation() / 100.0f;
        InitState.SetPosition(Pos);
        MovementComp->SetInitialState(InitState);
        MovementComp->SetControlMode(ControlMode);
    }

    // 闁告帗绋戠紓?RenderTarget 妤犵偛鐖奸崢銈囩磾?SceneCapture
    UTextureRenderTarget2D* RT = NewObject<UTextureRenderTarget2D>(this);
    RT->InitCustomFormat(CameraWidth, CameraHeight, PF_B8G8R8A8, false);
    RT->ClearColor = FLinearColor::Black;
    DroneSceneCapture->TextureTarget = RT;
    DroneSceneCapture->FOVAngle = CameraFOV;
    ApplySegmentationStencil();

    Api = NewObject<UDroneApi>(this);
    Api->Initialize(this);
    UAgentManager* Manager = UAgentManager::GetInstance();
    if (Manager)
    {
        FString ExistingId;
        const TArray<FString> ExistingIds = Manager->GetAllAgentIds();
        for (const FString& Id : ExistingIds)
        {
            if (Manager->GetAgent(Id) == this)
            {
                ExistingId = Id;
                break;
            }
        }

        if (!ExistingId.IsEmpty() && ExistingId != DroneId)
        {
            UE_LOG(LogTemp, Warning, TEXT("[DronePawn] Adopt pre-registered ID '%s' (old DroneId='%s')"), *ExistingId, *DroneId);
            DroneId = ExistingId;
        }

        if (Manager->GetAgent(DroneId) != this)
        {
            Manager->RegisterAgent(DroneId, this);
        }
    }
    UE_LOG(LogTemp, Log, TEXT("[DronePawn] BeginPlay: %s at %s, Camera %dx%d FOV=%.0f"),
        *DroneId, *GetActorLocation().ToString(), CameraWidth, CameraHeight, CameraFOV);
}

/**
 * @brief 婵絽绻愰幎姘跺即鐎涙ɑ鐓€
 * @param DeltaTime 閻㈩垎鍥紵闂傚懏姊瑰鍌炴⒒?
 */
void ADronePawn::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
    if (MovementComp) CurrentState = MovementComp->GetCurrentState();
    ApplyStateToActor(CurrentState);
    UpdatePropellerAnimation(DeltaTime);
    UpdateCameraRotation(DeltaTime);
    SyncPostProcessToCapture();
}

/**
 * @brief 閻忓繐妫旂挒銏ゆ儑閻斿搫笑闁诡兛绀侀幃鎾愁潰閵夈儱鐓?UE Actor Transform
 * @param State 闁哄啰濮冲Ч澶愬嫉閸濆嫮绉奸柛鎾崇Ф婵悂骞€?
 */
void ADronePawn::ApplyStateToActor(const FDroneState& State)
{
    FVector UEPos = State.GetPosition() * 100.0f;
    FRotator UERot = State.GetRotator();
    SetActorLocationAndRotation(UEPos, UERot);
}

/**
 * @brief 闁哄洤鐡ㄩ弻濠囨懚閻戞ɑ顥嬫俊妞煎妽濡棙娼鈧慨鈺呮偨?
 * @param DeltaTime 閻㈩垎鍥紵闂傚懏妫戠槐娆戠矓閹虹偟绀?
 * 閻忓繐妫涢弫鎼佸嫉妤︽寧绁梺?(rad/s) 閺夌儐鍓氬畷鍙夌▔閻戞妲ㄩ悽顖嗗嫭顥嬮弶鐑嗗墲椤鎯旈敂鑲╃獥
 *   RPM = 閿?* 60 / (2閿?
 *   閾绘渽ngle = RPM * 360 / 60 * DeltaTime
 */
void ADronePawn::UpdatePropellerAnimation(float DeltaTime)
{
    if (CurrentState.MotorSpeeds.Num() < 4) return;
    for (int32 i = 0; i < 4; i++)
    {
        UStaticMeshComponent* Fan = GetFanMesh(i);
        if (Fan)
        {
            float RPM = CurrentState.MotorSpeeds[i] * 60.0f / (2.0f * PI);
            float DeltaAngle = RPM * 360.0f / 60.0f * DeltaTime;
            float Direction = (i % 2 == 0) ? 1.0f : -1.0f;
            Fan->AddLocalRotation(FRotator(0.0f, DeltaAngle * Direction, 0.0f));
        }
    }
}

/**
 * @brief 闁哄秷顫夊畵浣烘閵忕姷绌块柤鎯у槻瑜板洦顦版惔銏狀暬缂傚啯鍨堕悧鍝ョ磼閸曨亝顐?
 * @param Index 濡炲瀛╂晶鏍閵忕姷绌?(0-3)
 * @return 閻庣數鎳撶花鏌ユ儍?UStaticMeshComponent
 */
UStaticMeshComponent* ADronePawn::GetFanMesh(int32 Index) const
{
    switch (Index)
    {
    case 0: return Fan0;
    case 1: return Fan1;
    case 2: return Fan2;
    case 3: return Fan3;
    default: return nullptr;
    }
}

/**
 * @brief 閻犱礁澧介悿鍡涙儎椤旂晫鍨煎ù锝呯Ф閻?
 * @param TargetPos 闁烩晩鍠楅悥锝嗘媴瀹ュ洨鏋?
 */
void ADronePawn::SetTargetPosition(const FVector& TargetPos, float Speed)
{
    if (MovementComp)
    {
        if (ControlMode != EDroneControlMode::Position)
        {
            MovementComp->SetControlMode(EDroneControlMode::Position);
            ControlMode = EDroneControlMode::Position;
        }
        MovementComp->SetTargetPosition(TargetPos, Speed);
    }
}

/**
 * @brief 閻犱礁澧介悿鍡涙儎椤旂晫鍨奸梺顐ゅ枎鐎?
 * @param TargetVel 闁烩晩鍠楅悥锝夋焻閻斿嘲顔?
 */
void ADronePawn::SetTargetVelocity(const FVector& TargetVel)
{
    if (MovementComp)
    {
        if (ControlMode != EDroneControlMode::Velocity)
        {
            MovementComp->SetControlMode(EDroneControlMode::Velocity);
            ControlMode = EDroneControlMode::Velocity;
        }
        MovementComp->SetTargetVelocity(TargetVel);
    }
}

void ADronePawn::MoveByVelocity(float Vx, float Vy, float Vz)
{
    SetTargetVelocity(FVector(Vx, Vy, Vz));
}

/**
 * @brief 閻犙呭厴椤ワ綁宕氶悧鍫濈樄閻庤宀搁悵顔芥償?
 * @param Altitude 闁烩晩鍠楅悥锝嗩殗濡搫顔?
 */
void ADronePawn::SetHeadingControl(EDroneYawMode YawMode, EDroneDrivetrainMode Drivetrain, float YawDeg)
{
    if (MovementComp)
    {
        MovementComp->SetHeadingControl(YawMode, Drivetrain, YawDeg);
    }
}

void ADronePawn::Takeoff(float Altitude)
{
    FVector CurrentPos = CurrentState.GetPosition();
    FVector TargetPos(CurrentPos.X, CurrentPos.Y, Altitude);
    SetTargetPosition(TargetPos);
    UE_LOG(LogTemp, Log, TEXT("[DronePawn] Takeoff to %.1f m"), Altitude);
}

/** @brief 闂傚嫬绉烽幆銈夊礆閺夋寧鍕鹃梻?*/
void ADronePawn::Land()
{
    FVector CurrentPos = CurrentState.GetPosition();
    FVector TargetPos(CurrentPos.X, CurrentPos.Y, 0.0f);
    SetTargetPosition(TargetPos);
    UE_LOG(LogTemp, Log, TEXT("[DronePawn] Landing"));
}

/** @brief 闁革负鍔岀紞瀣礈瀹ュ嫮绉寸紓鍐惧枟閸嬫捇宕?*/
void ADronePawn::Hover()
{
    FVector CurrentPos = CurrentState.GetPosition();
    SetTargetPosition(CurrentPos);
    UE_LOG(LogTemp, Log, TEXT("[DronePawn] Hover at %s"), *CurrentPos.ToString());
}

/** @brief 闁兼儳鍢茶ぐ鍥亹閹惧啿顤呭ù锝呯Ф閻?*/
FVector ADronePawn::GetCurrentPosition() const
{
    return CurrentState.GetPosition();
}

/** @brief 闁兼儳鍢茶ぐ鍥亹閹惧啿顤呴梺顐ゅ枎鐎?*/
FVector ADronePawn::GetCurrentVelocity() const
{
    return CurrentState.GetVelocity();
}

/**
 * @brief 闂佹彃绉堕悿鍡涘籍閻樿鲸鐪介柡鍫濇惈閸╁矂骞愰崶褏鏆板ù锝呯Ф閻ゅ棝宕仦鍙參骞€?
 * @param NewLocation 闁烩晩鍠楅悥锝嗘媴瀹ュ洨鏋?
 * @param NewRotation 闁烩晩鍠楅悥锝嗘叏閹稿簶鍋?
 */
void ADronePawn::ResetDrone(const FVector& NewLocation, const FRotator& NewRotation)
{
    FDroneState NewState;
    NewState.SetPosition(NewLocation);
    NewState.SetQuaternion(NewRotation.Quaternion());
    if (MovementComp)
    {
        MovementComp->ResetState(NewState);
        MovementComp->SetControlMode(EDroneControlMode::Idle);
    }
    CurrentState = NewState;
    ControlMode = EDroneControlMode::Idle;
    ApplyStateToActor(NewState);
    UE_LOG(LogTemp, Log, TEXT("[DronePawn] Reset to %s"), *NewLocation.ToString());
}

// 闁冲厜鍋撻柍鍏夊亾 闁硅棄瀚崕姘緞缂堢姷闅橀柛娆戝鐢爼宕?闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾

/**
 * @brief 閻犱礁澧介悿鍡涘箺閸曨偄鍓煎鍓佺節缁垶宕ｉ幍顔界獥闁哄秴娲╅～妤佹償?
 * @param TargetPitch 闁烩晩鍠楅悥锝嗙┍椤栨瑨鐦介悷?
 * @param TargetYaw 闁烩晩鍠楅悥锝夊磻韫囨艾鐒婚悷?
 */
void ADronePawn::SetCameraAngles(float TargetPitch, float TargetYaw)
{
    CameraTargetPitch = TargetPitch;
    CameraTargetYaw = TargetYaw;
}

/**
 * @brief 闁哄洤鐡ㄩ弻濠囧箺閸曨偄鍓煎?Yaw/Pitch 闁哄啫顑堝ù鍡涘箵閹烘垟鍋?
 * @param DeltaTime 閻㈩垎鍥紵闂傚懏姊瑰鍌炴⒒?
 * Yaw 闁哄啫顑堝ù鍡樻償閺冨倹鏆忛柛?CameraYawMesh闁挎稑顒tch 闁哄啫顑堝ù鍡樻償閺冨倹鏆忛柛?CameraPitchMesh
 */
void ADronePawn::UpdateCameraRotation(float DeltaTime)
{
    // Yaw 闁圭粯甯掗埀顒傘€嬬槐娆愬緞閸曨厽鍊?-180~180 缂備焦娲栧﹢鈧梻鍌ゅ櫍椤ｄ粙鏁?
    float YawDiff = FRotator::NormalizeAxis(CameraTargetYaw - CameraCurrentYaw);
    float AdjustedTargetYaw = CameraCurrentYaw + YawDiff;
    CameraCurrentYaw = FMath::FInterpTo(CameraCurrentYaw, AdjustedTargetYaw, DeltaTime, CameraRotationSpeed);

    // Pitch 闁圭粯甯掗埀?
    CameraCurrentPitch = FMath::FInterpTo(CameraCurrentPitch, CameraTargetPitch, DeltaTime, CameraRotationSpeed);

    // 閹煎瓨姊婚弫銈夊籍鐎ｎ厽绁柛鎺楊暒缁垶宕ｉ幍顔剧Ч闁?
    if (CameraYawMesh) CameraYawMesh->SetRelativeRotation(FRotator(0.0f, CameraCurrentYaw, 0.0f));
    if (CameraPitchMesh) CameraPitchMesh->SetRelativeRotation(FRotator(0.0f, 0.0f, -CameraCurrentPitch));
}

/** @brief 閻?CineCamera 闁?PostProcess 閻犱礁澧介悿鍡涘触鐏炵虎鍔勯柛?SceneCapture */
void ADronePawn::ApplySegmentationStencil()
{
    IConsoleVariable* CustomDepthVar = IConsoleManager::Get().FindConsoleVariable(TEXT("r.CustomDepth"));
    if (CustomDepthVar && CustomDepthVar->GetInt() < 3)
    {
        CustomDepthVar->Set(3, ECVF_SetByCode);
    }

    const int32 ClampedId = FMath::Clamp(SegmentationId, 0, 255);

    TArray<UPrimitiveComponent*> PrimitiveComponents;
    GetComponents<UPrimitiveComponent>(PrimitiveComponents);
    for (UPrimitiveComponent* Primitive : PrimitiveComponents)
    {
        if (!Primitive)
        {
            continue;
        }

        Primitive->SetRenderCustomDepth(true);
        Primitive->SetCustomDepthStencilValue(ClampedId);
    }
}

void ADronePawn::SyncPostProcessToCapture()
{
    if (!DroneCineCamera || !DroneSceneCapture) return;
    FMinimalViewInfo ViewInfo;
    DroneCineCamera->GetCameraView(0.0f, ViewInfo);
    FWeightedBlendables SavedBlendables = DroneSceneCapture->PostProcessSettings.WeightedBlendables;
    DroneSceneCapture->PostProcessSettings = ViewInfo.PostProcessSettings;
    DroneSceneCapture->PostProcessSettings.WeightedBlendables = SavedBlendables;
    DroneSceneCapture->PostProcessSettings.bOverride_AutoExposureBias = true;
    DroneSceneCapture->PostProcessSettings.AutoExposureBias += ExposureBias;

    // Avoid ghosting/stacked-frame artifacts in remote viewer captures.
    DroneSceneCapture->PostProcessSettings.bOverride_MotionBlurAmount = true;
    DroneSceneCapture->PostProcessSettings.MotionBlurAmount = 0.0f;
    DroneSceneCapture->PostProcessSettings.bOverride_MotionBlurMax = true;
    DroneSceneCapture->PostProcessSettings.MotionBlurMax = 0.0f;
    DroneSceneCapture->PostProcessSettings.bOverride_MotionBlurPerObjectSize = true;
    DroneSceneCapture->PostProcessSettings.MotionBlurPerObjectSize = 0.0f;
}

/**
 * @brief 闂佹彃娲▔锔界▔閳ь剛鏁濞存﹢宕撹箛鎾瑰珯閺夆晜鏌ㄥú?Base64 缂傚倹鐗滈悥婊堟儍?JPEG 閻庢稒顨堥浣圭▔?
 * @param Quality JPEG 闁告ê顑囩紓澶屾嫻閵娾晛娅?
 * @return Base64 缂傚倹鐗滈悥婊堟儍?JPEG 闁轰胶澧楀畵?
 * 闁归潧顑呮慨鈺冩喆閿曗偓瑜?SceneCapture 闂佹彃娲▔锔界▔閳ь剛鏁?闁?閻犲洩顕цぐ鍥磽韫囨洜顦?闁?JPEG 缂傚倹鐗滈悥?闁?Base64
 */
FString ADronePawn::CaptureImageBase64(int32 Quality)
{
    if (Quality <= 0)
    {
        Quality = JpegQuality;
    }

    return CameraCaptureUtils::CaptureColorJpegBase64(DroneSceneCapture, CameraWidth, CameraHeight, Quality);
}








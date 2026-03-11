#pragma once

#include "CoreMinimal.h"
#include "GameFramework/HUD.h"
#include "SimHUD.generated.h"

namespace AirSimImageUtils
{
    enum class EImageType : int32;
}

class USceneCaptureComponent2D;
class UTexture;
class UTexture2D;
class UTextureRenderTarget2D;
class UAgentListWidget;

USTRUCT()
struct FPipSlotState
{
    GENERATED_BODY()

    UPROPERTY()
    bool bVisible = false;

    UPROPERTY()
    FString AgentId;

    UPROPERTY()
    float Width = 320.0f;

    UPROPERTY()
    float Height = 180.0f;

    UPROPERTY()
    int32 ImageType = 0;

    UPROPERTY(Transient)
    UTexture2D* ProcessedTexture = nullptr;

    UPROPERTY(Transient)
    TArray<FColor> WorkingPixels;

    UPROPERTY()
    float LastUpdateTimeSec = -1.0f;
};

UCLASS()
class GRADUATIONPROJECT_API ASimHUD : public AHUD
{
    GENERATED_BODY()

public:
    virtual void BeginPlay() override;
    virtual void DrawHUD() override;

    UFUNCTION(BlueprintCallable, Category = "PIP")
    void TogglePipSlot(int32 SlotIndex);

    UFUNCTION(BlueprintCallable, Category = "PIP")
    void SetPipSource(int32 SlotIndex, const FString& AgentId);

    UFUNCTION(BlueprintCallable, Category = "PIP")
    void SetPipImageType(int32 SlotIndex, int32 ImageType);

private:
    void BindPipHotkeys();
    void EnsureAgentListWidget();

    void OnHotkeyTogglePip1();
    void OnHotkeyTogglePip2();
    void OnHotkeyTogglePip3();
    void OnHotkeySetImageTypeScene();
    void OnHotkeySetImageTypeDepthPlanar();
    void OnHotkeySetImageTypeDepthVis();
    void OnHotkeySetImageTypeSegmentation();
    void OnHotkeySetImageTypeInfrared();
    void SetActivePipSlotImageType(AirSimImageUtils::EImageType ImageType);

    void DrawAgentInfo(float& Y);
    void DrawGuidanceInfo(float& Y);
    void DrawFPS(float& Y);
    void DrawTextLine(const FString& Text, float X, float& Y, FLinearColor Color = FLinearColor::White, float Scale = 1.0f);
    void DrawSectionHeader(const FString& Title, float X, float& Y);

    void DrawPipWindows();
    void EnsurePipInitialized();
    void RefreshPipSourcesIfNeeded();
    TArray<FString> GetSortedCameraAgentIds() const;
    FString FindNextAgentWithCamera(const FString& CurrentId = TEXT("")) const;
    UTextureRenderTarget2D* ResolvePipRenderTarget(const FString& AgentId) const;
    bool ResolvePipCaptureInfo(const FString& AgentId, USceneCaptureComponent2D*& OutCapture, int32& OutWidth, int32& OutHeight) const;
    UTexture* ResolvePipTexture(FPipSlotState& Slot);
    bool UpdateSlotProcessedTexture(FPipSlotState& Slot, const TArray<FColor>& Pixels, int32 Width, int32 Height, AirSimImageUtils::EImageType SourceType);
    AirSimImageUtils::EImageType NormalizeImageType(int32 ImageType) const;
    FString GetImageTypeLabel(AirSimImageUtils::EImageType ImageType) const;

private:
    UPROPERTY()
    TArray<FPipSlotState> PipSlots;

    UPROPERTY(Transient)
    UAgentListWidget* AgentListWidget = nullptr;

    bool bPipInitialized = false;
    bool bPipHotkeysBound = false;
    int32 ActivePipSlotIndex = 0;

    UPROPERTY(EditAnywhere, Category = "AgentList")
    bool bEnableAgentListWidget = true;

    UPROPERTY(EditAnywhere, Category = "AgentList")
    bool bEnableAgentListMouse = false;

    UPROPERTY(EditAnywhere, Category = "PIP")
    float PipMaxDepthMeters = 120.0f;

    UPROPERTY(EditAnywhere, Category = "PIP")
    float NonSceneUpdateHz = 8.0f;

    UPROPERTY(EditAnywhere, Category = "PIP")
    int32 NonSceneMaxWidth = 960;

    UPROPERTY(EditAnywhere, Category = "PIP")
    int32 NonSceneMaxHeight = 540;

    UPROPERTY(EditAnywhere, Category = "PIP")
    int32 DepthModeMaxWidth = 800;

    UPROPERTY(EditAnywhere, Category = "PIP")
    int32 DepthModeMaxHeight = 450;
};



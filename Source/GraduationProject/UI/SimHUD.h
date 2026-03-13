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
class AActor;
class APlayerController;

/**
 * @brief 单个画中画窗口状态
 * 保存当前窗口是否显示、绑定的 Agent、图像类型以及缓存纹理等运行时信息。
 */
USTRUCT()
struct FPipSlotState
{
    GENERATED_BODY()

    /** @brief 当前窗口是否可见 */
    UPROPERTY()
    bool bVisible = false;

    /** @brief 当前窗口绑定的 Agent ID */
    UPROPERTY()
    FString AgentId;

    /** @brief 画中画窗口宽度 */
    UPROPERTY()
    float Width = 320.0f;

    /** @brief 画中画窗口高度 */
    UPROPERTY()
    float Height = 180.0f;

    /** @brief 当前图像类型，对应 `AirSimImageUtils::EImageType` */
    UPROPERTY()
    int32 ImageType = 0;

    /** @brief 处理后的显示纹理 */
    UPROPERTY(Transient)
    UTexture2D* ProcessedTexture = nullptr;

    /** @brief 图像处理中用的像素缓存 */
    UPROPERTY(Transient)
    TArray<FColor> WorkingPixels;

    /** @brief 上次刷新图像的仿真时间 */
    UPROPERTY()
    float LastUpdateTimeSec = -1.0f;
};

/**
 * @brief 仿真 HUD
 * 绘制 Agent 状态、制导信息、FPS 以及多路 PIP 画面，并管理 Agent 列表控件。
 */
UCLASS()
class GRADUATIONPROJECT_API ASimHUD : public AHUD
{
    GENERATED_BODY()

public:
    /** @brief 初始化 HUD 资源、快捷键和可选 UI */
    virtual void BeginPlay() override;

    /** @brief 每帧绘制文本信息与 PIP 窗口 */
    virtual void DrawHUD() override;

    /**
     * @brief 切换指定 PIP 窗口显隐
     * @param SlotIndex 窗口索引
     */
    UFUNCTION(BlueprintCallable, Category = "PIP")
    void TogglePipSlot(int32 SlotIndex);

    /**
     * @brief 设置指定 PIP 窗口的数据源 Agent
     * @param SlotIndex 窗口索引
     * @param AgentId 目标 Agent ID
     */
    UFUNCTION(BlueprintCallable, Category = "PIP")
    void SetPipSource(int32 SlotIndex, const FString& AgentId);

    /**
     * @brief 设置指定 PIP 窗口的图像类型
     * @param SlotIndex 窗口索引
     * @param ImageType 图像类型枚举值
     */
    UFUNCTION(BlueprintCallable, Category = "PIP")
    void SetPipImageType(int32 SlotIndex, int32 ImageType);

private:
    /** @brief 绑定 PIP 相关快捷键 */
    void BindPipHotkeys();

    /** @brief 创建并显示 Agent 列表控件 */
    void EnsureAgentListWidget();

    /** @brief 解析当前 PlayerController */
    APlayerController* ResolvePlayerController() const;
    /** @brief 绑定 1/2/3 键到 PIP 窗口 */
    void BindPipSlotKeys();
    /** @brief 绑定 Ctrl+数字键到图像类型 */
    void BindPipImageTypeKeys();

    /** @brief 快捷键：切换 PIP 1 */
    void OnHotkeyTogglePip1();

    /** @brief 快捷键：切换 PIP 2 */
    void OnHotkeyTogglePip2();

    /** @brief 快捷键：切换 PIP 3 */
    void OnHotkeyTogglePip3();

    /** @brief 快捷键：切换到 Scene 图像 */
    void OnHotkeySetImageTypeScene();

    /** @brief 快捷键：切换到 DepthPlanar 图像 */
    void OnHotkeySetImageTypeDepthPlanar();

    /** @brief 快捷键：切换到 DepthVis 图像 */
    void OnHotkeySetImageTypeDepthVis();

    /** @brief 快捷键：切换到 Segmentation 图像 */
    void OnHotkeySetImageTypeSegmentation();

    /** @brief 快捷键：切换到 Infrared 图像 */
    void OnHotkeySetImageTypeInfrared();

    /** @brief 将图像类型应用到当前激活的 PIP 窗口 */
    void SetActivePipSlotImageType(AirSimImageUtils::EImageType ImageType);

    /** @brief 绘制 Agent 状态摘要 */
    void DrawAgentInfo(float& Y);
    /** @brief 绘制单个 Agent 摘要行 */
    void DrawAgentSummaryLine(const FString& AgentId, AActor* Agent, float X, float& Y);

    /** @brief 绘制制导状态摘要 */
    void DrawGuidanceInfo(float& Y);

    /** @brief 绘制帧率信息 */
    void DrawFPS(float& Y);

    /** @brief 绘制一行 HUD 文本并推进 Y 坐标 */
    void DrawTextLine(const FString& Text, float X, float& Y, FLinearColor Color = FLinearColor::White, float Scale = 1.0f);

    /** @brief 绘制分节标题 */
    void DrawSectionHeader(const FString& Title, float X, float& Y);

    /** @brief 绘制所有 PIP 窗口 */
    void DrawPipWindows();
    /** @brief 绘制单个 PIP 窗口 */
    void DrawPipSlot(int32 SlotIndex, FPipSlotState& Slot, float X, float Y);

    /** @brief 初始化 PIP 窗口默认状态 */
    void EnsurePipInitialized();
    /** @brief 初始化单个 PIP 窗口 */
    void InitializePipSlot(FPipSlotState& Slot, int32 DefaultImageType);

    /** @brief 在 Agent 集合变化后刷新 PIP 数据源 */
    void RefreshPipSourcesIfNeeded();

    /** @brief 获取支持相机输出的 Agent ID 列表 */
    TArray<FString> GetSortedCameraAgentIds() const;

    /** @brief 查找下一个可用于 PIP 的 Agent */
    FString FindNextAgentWithCamera(const FString& CurrentId = TEXT("")) const;

    /** @brief 解析指定 Agent 的原始 RenderTarget */
    UTextureRenderTarget2D* ResolvePipRenderTarget(const FString& AgentId) const;

    /** @brief 解析指定 Agent 的捕获组件与图像尺寸 */
    bool ResolvePipCaptureInfo(const FString& AgentId, USceneCaptureComponent2D*& OutCapture, int32& OutWidth, int32& OutHeight) const;

    /** @brief 根据窗口配置生成当前需要显示的纹理 */
    UTexture* ResolvePipTexture(FPipSlotState& Slot);
    /** @brief 获取非 Scene 图像的最大捕获尺寸 */
    void GetNonSceneCaptureSizeLimits(AirSimImageUtils::EImageType ImageType, int32& OutMaxWidth, int32& OutMaxHeight) const;
    /** @brief 判断缓存纹理是否仍可复用 */
    bool CanReuseProcessedTexture(const FPipSlotState& Slot) const;

    /** @brief 把处理后的像素缓存上传到纹理 */
    bool UpdateSlotProcessedTexture(FPipSlotState& Slot, const TArray<FColor>& Pixels, int32 Width, int32 Height, AirSimImageUtils::EImageType SourceType);

    /** @brief 将整数类型码规范化为图像枚举 */
    AirSimImageUtils::EImageType NormalizeImageType(int32 ImageType) const;

    /** @brief 获取图像类型的人类可读标签 */
    FString GetImageTypeLabel(AirSimImageUtils::EImageType ImageType) const;

private:
    /** @brief 所有 PIP 窗口状态 */
    UPROPERTY()
    TArray<FPipSlotState> PipSlots;

    /** @brief Agent 列表 UI */
    UPROPERTY(Transient)
    UAgentListWidget* AgentListWidget = nullptr;

    /** @brief PIP 状态是否已经初始化 */
    bool bPipInitialized = false;

    /** @brief PIP 快捷键是否已绑定 */
    bool bPipHotkeysBound = false;

    /** @brief 当前激活的 PIP 窗口索引 */
    int32 ActivePipSlotIndex = 0;

    /** @brief 是否启用 Agent 列表控件 */
    UPROPERTY(EditAnywhere, Category = "AgentList")
    bool bEnableAgentListWidget = true;

    /** @brief 是否为 Agent 列表启用鼠标交互 */
    UPROPERTY(EditAnywhere, Category = "AgentList")
    bool bEnableAgentListMouse = false;

    /** @brief 深度图显示时的最大深度范围（米） */
    UPROPERTY(EditAnywhere, Category = "PIP")
    float PipMaxDepthMeters = 120.0f;

    /** @brief 非 Scene 图像的刷新频率 */
    UPROPERTY(EditAnywhere, Category = "PIP")
    float NonSceneUpdateHz = 8.0f;

    /** @brief 非 Scene 图像的最大宽度 */
    UPROPERTY(EditAnywhere, Category = "PIP")
    int32 NonSceneMaxWidth = 960;

    /** @brief 非 Scene 图像的最大高度 */
    UPROPERTY(EditAnywhere, Category = "PIP")
    int32 NonSceneMaxHeight = 540;

    /** @brief 深度模式下的最大宽度 */
    UPROPERTY(EditAnywhere, Category = "PIP")
    int32 DepthModeMaxWidth = 800;

    /** @brief 深度模式下的最大高度 */
    UPROPERTY(EditAnywhere, Category = "PIP")
    int32 DepthModeMaxHeight = 450;
};
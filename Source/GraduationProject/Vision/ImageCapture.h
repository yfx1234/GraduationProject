#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Components/SceneCaptureComponent2D.h"
#include "ImageCapture.generated.h"

/**
 * @brief 通用图像采集组件
 * 基于 `SceneCaptureComponent2D` 捕获场景画面，
 * 可输出 JPEG 字节流或 Base64 字符串，供网络接口或调试工具复用。
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class GRADUATIONPROJECT_API UImageCapture : public UActorComponent
{
    GENERATED_BODY()

public:
    /** @brief 构造图像采集组件 */
    UImageCapture();

protected:
    /** @brief 在 BeginPlay 时创建采集组件与 RenderTarget */
    virtual void BeginPlay() override;

public:
    /**
     * @brief 采集一帧图像并编码为 JPEG
     * @param Quality JPEG 压缩质量
     * @return JPEG 字节数组
     */
    TArray<uint8> CaptureJpeg(int32 Quality = 85);

    /**
     * @brief 采集一帧图像并返回 Base64 字符串
     * @param Quality JPEG 压缩质量
     * @return Base64 编码后的 JPEG 字符串
     */
    FString CaptureBase64(int32 Quality = 85);

    /**
     * @brief 获取当前图像尺寸
     * @return 图像宽高
     */
    FIntPoint GetImageSize() const { return FIntPoint(ImageWidth, ImageHeight); }

    /** @brief 输出图像宽度 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ImageCapture")
    int32 ImageWidth = 640;

    /** @brief 输出图像高度 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ImageCapture")
    int32 ImageHeight = 480;

    /** @brief 采集视场角 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ImageCapture")
    float CaptureFOV = 90.0f;

private:
    /** @brief 内部 `SceneCaptureComponent2D` 实例 */
    UPROPERTY()
    USceneCaptureComponent2D* CaptureComponent;

    /** @brief 采集结果写入的 RenderTarget */
    UPROPERTY()
    UTextureRenderTarget2D* RenderTarget;

    /**
     * @brief 创建并配置内部采集链路
     * 负责创建 RenderTarget、挂接捕获组件，并关闭逐帧采集以便按需读取。
     */
    void SetupCapture();
};
#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Components/SceneCaptureComponent2D.h"
#include "ImageCapture.generated.h"

/**
 * 图像采集组件
 * 使用 SceneCaptureComponent2D 捕获场景图像
 * 支持 TCP 命令 get_image 返回 Base64 编码的 JPEG
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class GRADUATIONPROJECT_API UImageCapture : public UActorComponent
{
    GENERATED_BODY()

public:
    /** @brief 构造函数 */
    UImageCapture();

protected:
    /** @brief 游戏开始时调用，初始化采集组件和 RenderTarget */
    virtual void BeginPlay() override;

public:
    /**
     * @brief 采集一帧图像，返回 JPEG 字节数据
     * @param Quality JPEG 压缩质量
     * @return JPEG 编码后的字节数组
     */
    TArray<uint8> CaptureJpeg(int32 Quality = 85);

    /**
     * @brief 采集一帧图像，返回 Base64 字符串
     * @param Quality JPEG 压缩质量
     * @return Base64 编码的 JPEG 字符串
     */
    FString CaptureBase64(int32 Quality = 85);

    /**
     * @brief 获取图像尺寸
     * @return 图像宽高
     */
    FIntPoint GetImageSize() const { return FIntPoint(ImageWidth, ImageHeight); }

    /** @brief 采集图像宽度 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ImageCapture")
    int32 ImageWidth = 640;

    /** @brief 采集图像高度 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ImageCapture")
    int32 ImageHeight = 480;

    /** @brief 采集视场角 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ImageCapture")
    float CaptureFOV = 90.0f;

private:
    /** @brief 场景采集组件 */
    UPROPERTY()
    USceneCaptureComponent2D* CaptureComponent;

    /** @brief 渲染目标纹理 */
    UPROPERTY()
    UTextureRenderTarget2D* RenderTarget;

    /** @brief 创建并配置 SceneCaptureComponent2D 和 RenderTarget */
    void SetupCapture();
};

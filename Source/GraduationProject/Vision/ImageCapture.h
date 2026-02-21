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
    UImageCapture();

protected:
    virtual void BeginPlay() override;

public:
    /** 采集一帧图像，返回 JPEG 字节数据 */
    TArray<uint8> CaptureJpeg(int32 Quality = 85);

    /** 采集一帧图像，返回 Base64 字符串（用于 TCP 传输） */
    FString CaptureBase64(int32 Quality = 85);

    /** 获取图像尺寸 */
    FIntPoint GetImageSize() const { return FIntPoint(ImageWidth, ImageHeight); }

    // ---- 配置 ----
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ImageCapture")
    int32 ImageWidth = 640;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ImageCapture")
    int32 ImageHeight = 480;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ImageCapture")
    float CaptureFOV = 90.0f;

private:
    UPROPERTY()
    USceneCaptureComponent2D* CaptureComponent;

    UPROPERTY()
    UTextureRenderTarget2D* RenderTarget;

    void SetupCapture();
};

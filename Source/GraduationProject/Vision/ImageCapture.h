// 解释：使用 `#pragma once` 防止该头文件在编译过程中被重复包含。
#pragma once

// 解释：引入 Unreal 的核心基础头文件，提供常用容器、数学类型和日志宏。
#include "CoreMinimal.h"
// 解释：引入 `ActorComponent.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "Components/ActorComponent.h"
// 解释：引入 `TextureRenderTarget2D.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "Engine/TextureRenderTarget2D.h"
// 解释：引入 `SceneCaptureComponent2D.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "Components/SceneCaptureComponent2D.h"
// 解释：引入 `ImageCapture.generated.h`，让 Unreal Header Tool 生成的反射代码在本文件中可见。
#include "ImageCapture.generated.h"

/**
 * @brief 通用图像采集组件
 * 基于 `SceneCaptureComponent2D` 捕获场景画面，
 * 可输出 JPEG 字节流或 Base64 字符串，供网络接口或调试工具复用。
 */
// 解释：使用 `UCLASS` 宏把当前类型注册到 Unreal 反射系统中，便于蓝图、序列化和编辑器识别。
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
// 解释：这一行声明 类 `UImageCapture`，用于封装uimage采集相关的数据与行为。
class GRADUATIONPROJECT_API UImageCapture : public UActorComponent
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：展开 Unreal Header Tool 生成的样板代码，这是 UObject、Actor、组件等类型正常工作的基础。
    GENERATED_BODY()

// 解释：从这一行开始进入 `public` 访问区，下面的成员会对外部模块公开。
public:
    /** @brief 构造图像采集组件 */
    // 解释：调用 `UImageCapture` 执行当前步骤需要的功能逻辑。
    UImageCapture();

// 解释：从这一行开始进入 `protected` 访问区，下面的成员只对本类及其派生类可见。
protected:
    /** @brief 在 BeginPlay 时创建采集组件与 RenderTarget */
    // 解释：调用 `BeginPlay` 执行当前步骤需要的功能逻辑。
    virtual void BeginPlay() override;

// 解释：从这一行开始进入 `public` 访问区，下面的成员会对外部模块公开。
public:
    /**
     * @brief 采集一帧图像并编码为 JPEG
     * @param Quality JPEG 压缩质量
     * @return JPEG 字节数组
     */
    // 解释：调用 `CaptureJpeg` 执行当前步骤需要的功能逻辑。
    TArray<uint8> CaptureJpeg(int32 Quality = 85);

    /**
     * @brief 采集一帧图像并返回 Base64 字符串
     * @param Quality JPEG 压缩质量
     * @return Base64 编码后的 JPEG 字符串
     */
    // 解释：这一行把右侧表达式的结果写入 `FString CaptureBase64(int32 Quality`，完成 fstring采集base64int32quality 的更新。
    FString CaptureBase64(int32 Quality = 85);

    /**
     * @brief 获取当前图像尺寸
     * @return 图像宽高
     */
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    FIntPoint GetImageSize() const { return FIntPoint(ImageWidth, ImageHeight); }

    /** @brief 输出图像宽度 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ImageCapture")
    // 解释：这一行声明成员或局部变量 `ImageWidth`，用于保存图像width。
    int32 ImageWidth = 640;

    /** @brief 输出图像高度 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ImageCapture")
    // 解释：这一行声明成员或局部变量 `ImageHeight`，用于保存图像height。
    int32 ImageHeight = 480;

    /** @brief 采集视场角 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ImageCapture")
    // 解释：这一行声明成员或局部变量 `CaptureFOV`，用于保存采集fov。
    float CaptureFOV = 90.0f;

// 解释：从这一行开始进入 `private` 访问区，下面的成员只允许类内部访问。
private:
    /** @brief 内部 `SceneCaptureComponent2D` 实例 */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY()
    // 解释：这一行声明成员或局部变量 `CaptureComponent`，用于保存采集组件。
    USceneCaptureComponent2D* CaptureComponent;

    /** @brief 采集结果写入的 RenderTarget */
    // 解释：使用 `UPROPERTY` 宏为下面的成员变量声明反射元数据，控制编辑器显示、序列化和蓝图可见性。
    UPROPERTY()
    // 解释：这一行声明成员或局部变量 `RenderTarget`，用于保存rendertarget。
    UTextureRenderTarget2D* RenderTarget;

    /**
     * @brief 创建并配置内部采集链路
     * 负责创建 RenderTarget、挂接捕获组件，并关闭逐帧采集以便按需读取。
     */
    // 解释：调用 `SetupCapture` 执行当前步骤需要的功能逻辑。
    void SetupCapture();
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
};

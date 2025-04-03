// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"

#include "Misc/Paths.h" // FPaths::ProjectContentDir
#include "Misc/FileHelper.h" // FFileHelper::LoadFileToString

//#include <onnxruntime_cxx_api.h>

#include "NNE.h"
#include "NNERuntimeCPU.h"
#include "NNERuntimeGPU.h"

#include "Async/Async.h"


// OpenCV 头文件
#include <OpenCVHelper/Public/PreOpenCVHeaders.h>
#include "opencv2/core.hpp"
#include <OpenCVHelper/Public/PostOpenCVHeaders.h>

#include "MyUtil.generated.h"

// 工具类的声明
UCLASS()
class CESIUM_PROJECT1_API UMyUtil : public UObject
{
    GENERATED_BODY()

public:
    UMyUtil();
    ~UMyUtil();

};

int32 GetViewportSize(int32& OutWidth, int32& OutHeight);

// 定义兴趣点结构体
USTRUCT(BlueprintType)
struct FCylindricalInterestPoint
{
    GENERATED_BODY()

    // 中心位置
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "InterestPoint")
    FVector Center;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "InterestPoint")
    FVector BottomCenter;

    // 半径，定义兴趣区域的大小
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "InterestPoint")
    float Radius;

    // 高度，定义兴趣区域的高度
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "InterestPoint")
    float Height;

    // 安全距离，无人机需要避开该兴趣区域的最小距离
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "InterestPoint")
    float MinSafetyDistance;

    // 构造函数
    FCylindricalInterestPoint()
        : Center(FVector::ZeroVector), BottomCenter(FVector::ZeroVector), Radius(0.0f), Height(0.0f), MinSafetyDistance(500.0f) {}
};

USTRUCT(BlueprintType)
struct FAngleRange
{
    GENERATED_BODY()

    float LowerBound;
    float UpperBound;

    // 默认构造函数
    FAngleRange()
        : LowerBound(0.0f)
        , UpperBound(0.0f)
    {
    }

};

USTRUCT(BlueprintType)
struct FPathPointWithOrientation
{
    GENERATED_BODY()

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    FVector Point;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    FRotator Orientation;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    float FOV; // 新增FOV字段

    // 新增属性
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    float AestheticScore;  // 美学评分

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    float CoverageAngle;   // 视角覆盖度

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    int32 AOIIndex;        // 关联的兴趣区域索引

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    FAngleRange AngleRange; // 俯仰角范围

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float SegmentSpeed; // 段速度

    // 默认构造函数
    FPathPointWithOrientation()
        : Point(FVector::ZeroVector)
        , Orientation(FRotator::ZeroRotator)
        , FOV(90.0f)
        , AestheticScore(0.0f)
        , CoverageAngle(0.0f)
        , AOIIndex(-1)
        , AngleRange()
        , SegmentSpeed(0)
    {
    }

    // 只提供位置和方向的情况
    // 新的构造函数，只接受 Point 和 Orientation 参数
    FPathPointWithOrientation(const FVector& InPoint, const FRotator& InOrientation)
        : Point(InPoint)
        , Orientation(InOrientation)
        , FOV(90.0f)
        , AestheticScore(0.0f)
        , CoverageAngle(0.0f)
        , AOIIndex(-1)
        , AngleRange()
        , SegmentSpeed(0)// 初始化AngleRange字段
    {
    }

	// 新的构造函数，接受 Point、Orientation 和 FOV 参数
    FPathPointWithOrientation(const FVector& InPoint, const FRotator& InOrientation, const float FOV)
        : Point(InPoint)
        , Orientation(InOrientation)
        , FOV(FOV)
        , AestheticScore(0.0f)
        , CoverageAngle(0.0f)
        , AOIIndex(-1)
        , AngleRange()
        , SegmentSpeed(0)// 初始化AngleRange字段
    {
    }

};

// 点击的状态
UENUM(BlueprintType)
enum class EInterestPointState : uint8
{
    Idle,
    CenterSelected,
    RadiusSelected,
    HeightSelected
};

USTRUCT()
struct FCandidateViewpoint
{
    GENERATED_BODY()

    FVector Location;
    FRotator OriginalRotation; // 原始旋转
    FRotator TargetRotation;   // 目标旋转
    int32 AOIIndex; // Index of the associated AOI
    float FOV;

    // Cost values
    float CompositionCost;
    float VisibilityCost;
    float TotalCost;

    FVector2D TargetImagePosition; // 目标在图像平面上的理想位置

    FCandidateViewpoint()
        : Location(FVector::ZeroVector),
        OriginalRotation(FRotator::ZeroRotator),
        TargetRotation(FRotator::ZeroRotator),
        AOIIndex(-1),
        FOV(90.0f),
        CompositionCost(0.0f),
        VisibilityCost(0.0f),
        TotalCost(0.0f),
        TargetImagePosition(FVector2D::ZeroVector)
    {
    }
};

USTRUCT()
// 增加对每个兴趣区域路径的支持
struct FInterestArea
{
    GENERATED_BODY()

    TArray<FPathPointWithOrientation> PathPoints; // 航线中的多个路径点
    TArray<FCandidateViewpoint> CandidateViewpoints; // 兴趣区域中的多个候选视点
};

USTRUCT(BlueprintType)
struct FLinkRoute
{
    GENERATED_BODY()

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    int32 StartNodeIndex; // 起点节点的索引

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    int32 EndNodeIndex;   // 终点节点的索引

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    double Cost;          // 该路径的代价

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    TArray<FPathPointWithOrientation> PathPoints; // 实际的路径点数据
};


// --------------------------------------------
// 使用原生引入的ONNXRuntime加载模型并预测
// 可以实现CPU预测，但是GPU调用会出现错误，因此改为使用UE5的NNE模块

//// 加载 ONNX 模型
//Ort::Session LoadONNXModel(const ORTCHAR_T* model_path, Ort::Env& env, Ort::SessionOptions& session_options);
//
//// 使用 ONNX Runtime 进行预测
//std::vector<float> ONNXPredict(Ort::Session& session, const std::vector<float>& input_tensor_values, const char* input_node_name, const char* output_node_name);
//
//// 利用ONNXRuntime加载模型并预测
//std::vector<float> ONNXPredict(const std::vector<float>& input_tensor_values, const ORTCHAR_T* model_path, const char* input_node_name, const char* output_node_name);
//
//// 计算最终分数
//float GetScore(const std::vector<float>& modelOutput);

// --------------------------------------------
// 使用UE5的NNE模块实现YOLO 目标检测
// 使用Cuda的GPU方法还是有问题，要么无法识别Cudnn和Cuda
// 要么突然要求TensorRT的配置文件，因此改为使用CPU方法
// 升级到UE5.4后，使用RuntimeORTDml后端则可以调用GPU运行了

struct FMyModelHelper
{
    TSharedPtr<UE::NNE::IModelInstanceGPU> ModelInstance;
    TArray<float> InputData;
    TArray<TArray<float>> OutputData;
    TArray<UE::NNE::FTensorBindingGPU> InputBindings;
    TArray<UE::NNE::FTensorBindingGPU> OutputBindings;
    bool bIsRunning = false;
};

struct FDetectionResult
{
    FDetectionResult()
        : x1(0.0f), y1(0.0f), x2(0.0f), y2(0.0f), Confidence(0.0f), ClassID(0)
    {
        // 默认构造器的实现
    }

    ~FDetectionResult()
    {
        // 析构函数的实现
    }

    FDetectionResult(float InX1, float InY1, float InX2, float InY2, float InMaxClassScore, int32 InClassID)
        : x1(InX1), y1(InY1), x2(InX2), y2(InY2), Confidence(InMaxClassScore), ClassID(InClassID)
    {
    }

    float x1, y1, x2, y2; // 边界框的左上角和右下角坐标
    float Confidence;     // 置信度
    int32 ClassID;        // 类别ID
    // 去除与掩码相关的成员变量
};

/**
 * YoloObjectTracker 类，用于处理 YOLO 模型的推理
 */

class YoloObjectTracker
{

public:
    YoloObjectTracker();
    YoloObjectTracker(const FString& ModelPath);
    virtual ~YoloObjectTracker();

    virtual bool Initialize();
    //bool RunInference(UTexture2D* InputTexture, FVector2D TrackingPoint);
    bool RunInference(UTextureRenderTarget2D* RenderTarget, FVector2D TrackingPoint);
    FVector2D GetTrackedPosition() const;
    //void StartTracking(UTexture2D* InputTexture, FVector2D ClickPosition);
    void StartTracking(UTextureRenderTarget2D* RenderTarget, FVector2D ClickPosition);
    bool IsInferencing() const;// 用于检查是否正在进行推理
    bool IsTracking() const; // 用于检查是否正常追踪
	bool ShouldStopTracking() const; // 用于检查是否应该停止追踪

    // 第一个 const：确保返回的 FDetectionResult 对象不能被修改。
    // 第二个 const：确保 GetTrackedDetection 成员函数不会修改所属对象的状态。
    const FDetectionResult& GetTrackedDetection() const;

    // 用于停止追踪
    void StopTracking();

    bool bIsInitialized = false;
    bool bHasInitialTracking = false;

    /*
    首先按置信度对检测结果进行排序。
    通过嵌套循环，计算每个检测结果与其他检测结果的 IoU。
    如果 IoU 超过阈值 IoUThreshold，则抑制置信度较低的检测结果。
    */
    void ApplyNMS(TArray<FDetectionResult>& Detections, float IoUThreshold);
    /*
    计算两个边界框之间的 IoU。
    */
    float ComputeIoU(const FDetectionResult& A, const FDetectionResult& B);

protected:

    TSharedPtr<UE::NNE::IModelGPU> ModelGPU;
    TSharedPtr<FMyModelHelper> ModelHelper;

private:
    FVector2D TrackedPosition;

    cv::Mat LetterboxImage(const cv::Mat& src, int32 target_width, int32 target_height, float& scale, int32& pad_w, int32& pad_h);

    //void PrepareInputData(UTexture2D* InputTexture, TArray<float>& ProcessedData);
    void PrepareInputData(UTextureRenderTarget2D* RenderTarget, TArray<float>& ProcessedData, float& OutScale, int32& OutPadW, int32& OutPadH,
        int32& OutOriginalWidth, int32& OutOriginalHeight);
    void ParseOutputData(const TArray<float>& OutputData, FVector2D TrackingPoint, float scale, int32 pad_w, int32 pad_h, int32 original_width, int32 original_height);
    FDetectionResult TrackedDetection; // 用于存储检测到的目标

    int32 TrackedClassID; // 用于存储检测到的目标的类别ID

	bool bIsTracking = false; // 用于检查是否正常追踪物体
	bool bShouldStopTracking = false; // 用于停止追踪物体

    // 预处理参数存储
    float LastInferenceScale;
    int32 LastInferencePadW;
    int32 LastInferencePadH;
    int32 LastInferenceOriginalWidth;
    int32 LastInferenceOriginalHeight;

    // 互斥锁，保护共享资源
    mutable FCriticalSection InferenceCriticalSection;
};

/**
 * NimaObjectTracker 类，用于处理 NIMA 模型的推理
 */
class NimaObjectTracker : public YoloObjectTracker
{

public:
    NimaObjectTracker() {};
    ~NimaObjectTracker() override {} ;
    NimaObjectTracker(const FString& ModelPath);

    bool Initialize() override;
    bool RunInference(UTextureRenderTarget2D* RenderTarget);

	// 用于获取评分
	float GetNimaScore() const;

	void ResetNimaScore();

    bool IfSaveImage = false;
private:
	bool ParseNimaInput(UTextureRenderTarget2D* RenderTarget, TArray<float>& ProcessedData, cv::Mat& BGRImage);
    void ParseNimaOutput(const TArray<float>& OutputData, float& Score);

    mutable FCriticalSection InferenceCriticalSection;

    float Score=0;
};
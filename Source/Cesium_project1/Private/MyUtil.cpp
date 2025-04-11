// Fill out your copyright notice in the Description page of Project Settings.


#include "MyUtil.h"

#include "Engine/TextureRenderTarget2D.h"

#include <iostream>
#include <vector>
#include <tuple>

#include "NNERuntime.h"
#include "NNETypes.h"
#include "NNEModelData.h"
#include "NNEStatus.h"
#include "NNETensor.h"  // This header contains definitions related to tensors

#include <OpenCVHelper/Public/PreOpenCVHeaders.h>
#include "opencv2/highgui.hpp"
#include <opencv2/imgproc.hpp> // 对于 cvtColor 和 resize
#include <opencv2/imgproc/types_c.h> // 对于 COLOR_BGR2RGB

#include <OpenCVHelper/Public/PostOpenCVHeaders.h>

UMyUtil::UMyUtil()
{
}

UMyUtil::~UMyUtil()
{
}

// 获取当前视窗的宽度和高度
int32 GetViewportSize(int32& OutWidth, int32& OutHeight)
{
	if (GEngine && GEngine->GameViewport)
	{
		FVector2D ViewportSize;
		GEngine->GameViewport->GetViewportSize(ViewportSize);
		OutWidth = ViewportSize.X;
		OutHeight = ViewportSize.Y;
		return true;
	}
	return false;
}

//// 加载 ONNX 模型
//Ort::Session LoadONNXModel(const ORTCHAR_T* model_path, Ort::Env& env, Ort::SessionOptions& session_options) {
//	// 创建会话 GPU版本无法运行,所以使用CPU
//	Ort::Session session(env, model_path, session_options);
//	return session;
//}
//
//// 使用 ONNX Runtime 进行预测
//std::vector<float> ONNXPredict(Ort::Session& session, const std::vector<float>& input_tensor_values, const char* input_node_name, const char* output_node_name) {
//	std::vector<int64_t> input_tensor_shape = { 1, 3, 224, 224 };
//	auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeDefault);
//	Ort::Value input_tensor = Ort::Value::CreateTensor<float>(memory_info, const_cast<float*>(input_tensor_values.data()), input_tensor_values.size(), input_tensor_shape.data(), input_tensor_shape.size());
//
//	const char* input_names[] = { input_node_name };
//	const char* output_names[] = { output_node_name };
//	auto output_tensors = session.Run(Ort::RunOptions{ nullptr }, input_names, &input_tensor, 1, output_names, 1);
//
//	float* output_data = output_tensors.front().GetTensorMutableData<float>();
//	std::vector<float> output_values(output_data, output_data + output_tensors.front().GetTensorTypeAndShapeInfo().GetElementCount());
//
//	return output_values;
//}
//
//// 使用 ONNX Runtime 进行预测
//std::vector<float> ONNXPredict(const std::vector<float>& input_tensor_values, const ORTCHAR_T* model_path, const char* input_node_name, const char* output_node_name) {
//	Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "test");
//	Ort::SessionOptions session_options;
//	session_options.SetIntraOpNumThreads(1);
//	// 添加 CUDA 执行提供程序
//	OrtCUDAProviderOptions cuda_options{};
//	/*cuda_options.device_id = 0;
//	cuda_options.arena_extend_strategy = 0;
//	cuda_options.GPU_mem_limit = 2 * 1024 * 1024 * 1024;
//	cuda_options.cudnn_conv_algo_search = OrtCudnnConvAlgoSearch::OrtCudnnConvAlgoSearchExhaustive;
//	cuda_options.do_copy_in_default_stream = 1;
//	session_options.AppendExecutionProvider_CUDA(cuda_options);*/
//
//	// 创建会话 GPU 版本无法运行，所以使用CPU
//    Ort::Session session(env, model_path, session_options = Ort::SessionOptions(nullptr));
//
//	std::vector<int64_t> input_tensor_shape = { 1, 3, 224, 224 };
//	auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeDefault);
//	Ort::Value input_tensor = Ort::Value::CreateTensor<float>(memory_info, const_cast<float*>(input_tensor_values.data()), input_tensor_values.size(), input_tensor_shape.data(), input_tensor_shape.size());
//
//	const char* input_names[] = { input_node_name };
//	const char* output_names[] = { output_node_name };
//	auto output_tensors = session.Run(Ort::RunOptions{ nullptr }, input_names, &input_tensor, 1, output_names, 1);
//
//	float* output_data = output_tensors.front().GetTensorMutableData<float>();
//	std::vector<float> output_values(output_data, output_data + output_tensors.front().GetTensorTypeAndShapeInfo().GetElementCount());
//
//	return output_values;
//}
//
//float GetScore(const std::vector<float>& modelOutput)
//{
//	// 定义权重，与 Python 中的 np.linspace(1, 10, 10) 相似
//	std::vector<float> weights = { 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f, 10.0f };
//	float score = 0.0f;
//
//	// 计算加权分数
//	for (int i = 0; i < modelOutput.size(); ++i)
//	{
//		score += modelOutput[i] * weights[i];
//	}
//
//	return score;
//}
//
//// 读入原始ONNX模型 事实证明这样子不需要，将ONNX导入为ASSET即可
//static UNNEModelData* ImportONNXModel(const FString& ModelPath)
//{
//    // Load the ONNX file into memory
//    TArray<uint8> FileData;
//    if (!FFileHelper::LoadFileToArray(FileData, *ModelPath))
//    {
//        UE_LOG(LogTemp, Error, TEXT("Failed to load ONNX model file: %s"), *ModelPath);
//        return nullptr;
//    }
//
//    // Create the UNNEModelData asset
//    UNNEModelData* NewModelData = NewObject<UNNEModelData>();
//    if (!NewModelData)
//    {
//        UE_LOG(LogTemp, Error, TEXT("Failed to create UNNEModelData object."));
//        return nullptr;
//    }
//
//    // Initialize the model data with the file data
//    FString FileType = FPaths::GetExtension(ModelPath);
//    NewModelData->Init(FileType, FileData);
//
//    return NewModelData;
//}
//
//int32 GetDataTypeSize(ENNETensorDataType DataType)
//{
//    switch (DataType)
//    {
//    case ENNETensorDataType::Float:
//        return sizeof(float);
//    case ENNETensorDataType::Int32:
//        return sizeof(int32);
//    case ENNETensorDataType::UInt8:
//        return sizeof(uint8);
//        // Add other data types as necessary
//    default:
//        UE_LOG(LogTemp, Error, TEXT("Unknown tensor data type."));
//        return 0;
//    }
//}

YoloObjectTracker::YoloObjectTracker() {

}

YoloObjectTracker::YoloObjectTracker(const FString& ModelPath)
{
	TObjectPtr<UNNEModelData> YOLOModelData = LoadObject<UNNEModelData>(GetTransientPackage(), TEXT("/Game/Models/yolo11m-seg.yolo11m-seg"));
	//TObjectPtr<UNNEModelData> YOLOModelData = UNNEModelData::Load(*ModelPath);
	//UNNEModelData* YOLOModelData = ImportONNXModel(ModelPath);
	ModelHelper = MakeShared<FMyModelHelper>();

	if (YOLOModelData)
	{
		UE_LOG(LogTemp, Display, TEXT("YOLOModelData loaded %s"), *YOLOModelData->GetName());
		// You can use ManuallyLoadedModelData here to create a model and corresponding model instance(s)
		// ManuallyLoadedModelData will be unloaded when the current function returns
	}
	else
	{
		UE_LOG(LogTemp, Error, TEXT("YOLOModelData is not loaded, please check the static path to your asset"));
	}

	TWeakInterfacePtr<INNERuntimeGPU> Runtime = UE::NNE::GetRuntime<INNERuntimeGPU>(FString("NNERuntimeORTDml")); // NNERuntimeORTDml 或者 NNERuntimeORTCpu
	if (Runtime.IsValid())
	{
		UE_LOG(LogTemp, Warning, TEXT("Successfully retrieved GPU runtime backend."));
		TSharedPtr<UE::NNE::IModelGPU> Model = Runtime->CreateModelGPU(YOLOModelData);
		UE_LOG(LogTemp, Warning, TEXT("Model created"));
		if (Model.IsValid())
		{
			UE_LOG(LogTemp, Warning, TEXT("Model is valid."));
			ModelGPU = Model;
			if (!Initialize()) {
				UE_LOG(LogTemp, Error, TEXT("Initialization failed."));
				// Handle the failure, possibly set a flag or return early
				bIsInitialized = false;
				return;
			}
			else {
				UE_LOG(LogTemp, Warning, TEXT("Initialization successful."));
				bIsInitialized = true;
			}
		}
		else
		{
			UE_LOG(LogTemp, Error, TEXT("Failed to create a valid model."));
			return;
		}

		// 获取所有注册的运行时名称
		TArray<FString> RuntimeNames = UE::NNE::GetAllRuntimeNames<INNERuntime>();

		// 用于存储所有运行时的指针
		TArray<TWeakInterfacePtr<INNERuntime>> AllRuntimes;

		// 遍历名称数组并获取对应的运行时
		for (const FString& Name : RuntimeNames)
		{
			TWeakInterfacePtr<INNERuntime> IterRuntime = UE::NNE::GetRuntime<INNERuntime>(Name);
			if (IterRuntime.IsValid())
			{
				AllRuntimes.Add(IterRuntime);
			}
		}

		UE_LOG(LogTemp, Log, TEXT("Available NNE Runtimes:"));
		for (const TWeakInterfacePtr<INNERuntime>& IterRuntime : AllRuntimes)
		{
			if (IterRuntime.IsValid())
			{
				FString RuntimeName = IterRuntime->GetRuntimeName();
				UE_LOG(LogTemp, Log, TEXT("Runtime: %s"), *RuntimeName);
			}
			else
			{
				UE_LOG(LogTemp, Warning, TEXT("Invalid runtime found."));
			}
		}
	}
	else
	{
		UE_LOG(LogTemp, Error, TEXT("Failed to find GPU runtime backend"));

		/* TArray<FString> PossibleRuntimes = {
			 TEXT("NNERuntimeORTCpu"),
			 TEXT("NNERuntimeORTCPU"),
			 TEXT("NNERuntimeORT"),
			 TEXT("NNERuntimeTensorRT"),
			 TEXT("NNERuntimeDirectML"),
			 TEXT("NNERuntimeOpenVINO")
		 };

		 UE_LOG(LogTemp, Log, TEXT("Checking available NNE Runtimes:"));
		 for (const FString& RuntimeName : PossibleRuntimes)
		 {
			 TWeakInterfacePtr<INNERuntime> IterRuntime = UE::NNE::GetRuntime<INNERuntime>(RuntimeName);
			 if (IterRuntime.IsValid())
			 {
				 UE_LOG(LogTemp, Log, TEXT("Available Runtime: %s"), *RuntimeName);
			 }
			 else
			 {
				 UE_LOG(LogTemp, Warning, TEXT("Runtime not available: %s"), *RuntimeName);
			 }
		 }*/

		 //TArrayView<TWeakInterfacePtr<INNERuntime>> AllRuntimes = UE::NNE::GetAllRuntimes();
		 // 获取所有注册的运行时名称
		TArray<FString> RuntimeNames = UE::NNE::GetAllRuntimeNames<INNERuntime>();

		// 用于存储所有运行时的指针
		TArray<TWeakInterfacePtr<INNERuntime>> AllRuntimes;

		// 遍历名称数组并获取对应的运行时
		for (const FString& Name : RuntimeNames)
		{
			TWeakInterfacePtr<INNERuntime> IterRuntime = UE::NNE::GetRuntime<INNERuntime>(Name);
			if (IterRuntime.IsValid())
			{
				AllRuntimes.Add(IterRuntime);
			}
		}

		UE_LOG(LogTemp, Log, TEXT("Available NNE Runtimes:"));
		for (const TWeakInterfacePtr<INNERuntime>& IterRuntime : AllRuntimes)
		{
			if (IterRuntime.IsValid())
			{
				FString RuntimeName = IterRuntime->GetRuntimeName();
				UE_LOG(LogTemp, Log, TEXT("Runtime: %s"), *RuntimeName);
			}
			else
			{
				UE_LOG(LogTemp, Warning, TEXT("Invalid runtime found."));
			}
		}

		return; // Add this line to exit the function
	}
}

YoloObjectTracker::~YoloObjectTracker() {}

bool YoloObjectTracker::Initialize()
{
	// 创建模型实例
	auto ModelInstance = ModelGPU->CreateModelInstanceGPU();
	if (!ModelInstance)
	{
		UE_LOG(LogTemp, Error, TEXT("Failed to create model instance"));
		return false;
	}
	UE_LOG(LogTemp, Log, TEXT("Model instance created successfully."));
	ModelHelper->ModelInstance = ModelInstance;

	// 获取输入张量描述
	TConstArrayView<UE::NNE::FTensorDesc> InputTensorDescs = ModelHelper->ModelInstance->GetInputTensorDescs();
	UE_LOG(LogTemp, Log, TEXT("Number of input tensor descriptions: %d"), InputTensorDescs.Num());
	if (InputTensorDescs.Num() == 0)
	{
		UE_LOG(LogTemp, Error, TEXT("No input tensor descriptions available."));
		return false;
	}

	// 准备输入张量形状和绑定
	TArray<UE::NNE::FTensorShape> InputShapes;
	InputShapes.SetNum(InputTensorDescs.Num());
	ModelHelper->InputBindings.SetNum(InputTensorDescs.Num());
	ModelHelper->InputData.Reset();

	int64 TotalInputVolume = 0;

	for (int32 i = 0; i < InputTensorDescs.Num(); ++i)
	{
		const UE::NNE::FTensorDesc& TensorDesc = InputTensorDescs[i];
		const UE::NNE::FSymbolicTensorShape& SymbolicShape = TensorDesc.GetShape();

		// 获取维度信息
		TConstArrayView<int32> SymbolicDims = SymbolicShape.GetData();

		TArray<uint32> ConcreteDims;
		FString ShapeStr = TEXT("[");
		for (int32 DimIndex = 0; DimIndex < SymbolicDims.Num(); ++DimIndex)
		{
			int32 Dim = SymbolicDims[DimIndex]; // 获取维度
			uint32 ConcreteDim = (Dim < 0) ? 1 : static_cast<uint32>(Dim); // 处理动态维度
			ConcreteDims.Add(ConcreteDim);

			ShapeStr += FString::Printf(TEXT("%d"), ConcreteDim);
			if (DimIndex < SymbolicDims.Num() - 1)
			{
				ShapeStr += TEXT(", ");
			}
		}
		ShapeStr += TEXT("]");
		UE_LOG(LogTemp, Log, TEXT("Input tensor %d shape: %s"), i, *ShapeStr); // 例如：[1, 3, 640, 640]

		// 创建具体的张量形状
		UE::NNE::FTensorShape ConcreteShape = UE::NNE::FTensorShape::Make(ConcreteDims);

		InputShapes[i] = ConcreteShape;

		// 计算张量体积
		uint64 Volume = ConcreteShape.Volume();
		if (Volume == 0)
		{
			UE_LOG(LogTemp, Error, TEXT("Invalid tensor volume for input %d"), i);
			return false;
		}

		// 分配输入数据
		ModelHelper->InputData.AddZeroed(Volume);

		// 设置输入绑定
		ModelHelper->InputBindings[i].Data = ModelHelper->InputData.GetData() + TotalInputVolume;
		ModelHelper->InputBindings[i].SizeInBytes = Volume * sizeof(float);

		TotalInputVolume += Volume;
	}

	// 设置输入张量形状
	if (ModelHelper->ModelInstance->SetInputTensorShapes(InputShapes) != UE::NNE::EResultStatus::Ok)
	{
		UE_LOG(LogTemp, Error, TEXT("Failed to set input tensor shapes."));
		return false;
	}

	// 获取输出张量形状
	TConstArrayView<UE::NNE::FTensorShape> OutputShapes = ModelHelper->ModelInstance->GetOutputTensorShapes();
	if (OutputShapes.Num() == 0)
	{
		UE_LOG(LogTemp, Error, TEXT("Failed to retrieve output tensor shapes after setting input shapes."));
		return false;
	}
	else {
		UE_LOG(LogTemp, Log, TEXT("Number of output tensor shapes: %d"), OutputShapes.Num());
	}

	// 获取输出张量描述
	TConstArrayView<UE::NNE::FTensorDesc> OutputTensorDescs = ModelHelper->ModelInstance->GetOutputTensorDescs();
	if (OutputTensorDescs.Num() == 0)
	{
		UE_LOG(LogTemp, Error, TEXT("No output tensor descriptions available."));
		return false;
	}

	// 准备输出绑定
	ModelHelper->OutputBindings.SetNum(OutputTensorDescs.Num());
	ModelHelper->OutputData.Reset();

	int64 TotalOutputVolume = 0;

	for (int32 i = 0; i < OutputTensorDescs.Num(); ++i)
	{
		const UE::NNE::FTensorShape& ConcreteShape = OutputShapes[i];

		// 计算张量体积
		uint64 Volume = ConcreteShape.Volume();
		if (Volume == 0)
		{
			UE_LOG(LogTemp, Error, TEXT("Invalid tensor volume for output %d"), i);
			return false;
		}
		else {
			UE_LOG(LogTemp, Log, TEXT("Output tensor %d volume: %d"), i, Volume);
		}

		TArray<float> OutputT;
		ModelHelper->OutputData.Push(OutputT);
		// 分配输出数据
		ModelHelper->OutputData[i].Reset();
		ModelHelper->OutputData[i].AddZeroed(Volume);

		// 设置输出绑定
		ModelHelper->OutputBindings[i].Data = ModelHelper->OutputData[i].GetData();
		ModelHelper->OutputBindings[i].SizeInBytes = Volume * sizeof(float);

		TotalOutputVolume += Volume;
	}

	UE_LOG(LogTemp, Log, TEXT("Initialization completed successfully."));
	return true;
}

void YoloObjectTracker::ApplyNMS(TArray<FDetectionResult>& Detections, float IoUThreshold)
{
	// 按置信度从高到低排序
	Detections.Sort([](const FDetectionResult& A, const FDetectionResult& B) {
		return A.Confidence > B.Confidence;
		});

	TArray<bool> Suppressed;
	Suppressed.Init(false, Detections.Num());

	for (int32 i = 0; i < Detections.Num(); ++i)
	{
		if (Suppressed[i])
			continue;

		const FDetectionResult& DetectionA = Detections[i];

		for (int32 j = i + 1; j < Detections.Num(); ++j)
		{
			if (Suppressed[j])
				continue;

			const FDetectionResult& DetectionB = Detections[j];

			// 如果类别不同，可以选择跳过（根据需要）
			// if (DetectionA.ClassID != DetectionB.ClassID)
			//     continue;

			// 计算 IoU
			float IoU = ComputeIoU(DetectionA, DetectionB);

			if (IoU > IoUThreshold)
			{
				// 抑制较低置信度的检测结果
				Suppressed[j] = true;
			}
		}
	}

	// 移除被抑制的检测结果
	for (int32 i = Detections.Num() - 1; i >= 0; --i)
	{
		if (Suppressed[i])
		{
			Detections.RemoveAt(i);
		}
	}
}

float YoloObjectTracker::ComputeIoU(const FDetectionResult& A, const FDetectionResult& B)
{
	float x1 = FMath::Max(A.x1, B.x1);
	float y1 = FMath::Max(A.y1, B.y1);
	float x2 = FMath::Min(A.x2, B.x2);
	float y2 = FMath::Min(A.y2, B.y2);

	float IntersectionWidth = FMath::Max(0.0f, x2 - x1);
	float IntersectionHeight = FMath::Max(0.0f, y2 - y1);
	float IntersectionArea = IntersectionWidth * IntersectionHeight;

	float AreaA = (A.x2 - A.x1) * (A.y2 - A.y1);
	float AreaB = (B.x2 - B.x1) * (B.y2 - B.y1);
	float UnionArea = AreaA + AreaB - IntersectionArea;

	if (UnionArea <= 0.0f)
		return 0.0f;

	return IntersectionArea / UnionArea;
}


//std::mutex InferenceMutex;

bool YoloObjectTracker::RunInference(UTextureRenderTarget2D* RenderTarget, FVector2D TrackingPoint)
{
	// 使用 FScopeLock 保护临界区
	FScopeLock Lock(&InferenceCriticalSection);
	/*std::lock_guard<std::mutex> lock(InferenceMutex);*/
	if (ModelHelper->bIsRunning) return false;
	if (!bIsInitialized) {
		UE_LOG(LogTemp, Error, TEXT("Model is not initialized."));
		return false;
	}

	TArray<float> ProcessedData;

	PrepareInputData(RenderTarget, ProcessedData, LastInferenceScale, LastInferencePadW, LastInferencePadH, LastInferenceOriginalWidth, LastInferenceOriginalHeight);

	if (ModelHelper->InputBindings.Num() == 0 || ModelHelper->InputBindings[0].Data == nullptr)
	{
		UE_LOG(LogTemp, Error, TEXT("InputBindings are not properly initialized."));
		return false;
	}
	float* InputBindingData = static_cast<float*>(ModelHelper->InputBindings[0].Data);
	FMemory::Memcpy(InputBindingData, ProcessedData.GetData(), ProcessedData.Num() * sizeof(float));

	ModelHelper->bIsRunning = true;
	TSharedPtr<FMyModelHelper> ModelHelperPtr = ModelHelper;

	AsyncTask(ENamedThreads::AnyNormalThreadNormalTask, [ModelHelperPtr, TrackingPoint, this]()
		{
			FScopeLock Lock(&InferenceCriticalSection);
			float scale = this->LastInferenceScale;
			int32 pad_w = this->LastInferencePadW;
			int32 pad_h = this->LastInferencePadH;
			int32 original_width = this->LastInferenceOriginalWidth;
			int32 original_height = this->LastInferenceOriginalHeight;
			if (ModelHelperPtr->ModelInstance->RunSync(ModelHelperPtr->InputBindings, ModelHelperPtr->OutputBindings) == UE::NNE::EResultStatus::Ok)
			{
				ParseOutputData(ModelHelperPtr->OutputData[0], TrackingPoint, scale, pad_w, pad_h, original_width, original_height);
			}
			else
			{
				UE_LOG(LogTemp, Error, TEXT("Failed to run model inference"));
			}

			AsyncTask(ENamedThreads::GameThread, [ModelHelperPtr, this]()
				{
					FScopeLock Lock(&InferenceCriticalSection);
					ModelHelperPtr->bIsRunning = false;
				});
		});

	return true;
}

void YoloObjectTracker::StartTracking(UTextureRenderTarget2D* RenderTarget, FVector2D ClickPosition)
{
	if (!IsInferencing())
	{
		bIsTracking = false;
		bHasInitialTracking = false;
		bShouldStopTracking = false;
		TrackedPosition = ClickPosition;
		RunInference(RenderTarget, TrackedPosition);
	}
}

bool YoloObjectTracker::IsInferencing() const
{
	return ModelHelper.IsValid() && ModelHelper->bIsRunning;// 正在运行中
}

/// <summary>
/// 准备输入数据，将渲染目标的像素数据转换为模型所需的格式。
/// Scale和pad自动计算得到
/// </summary>
/// <param name="RenderTarget">渲染目标。</param>
/// <param name="ProcessedData">处理后的数据。</param>
/// <param name="scale">缩放比例。</param>
/// <param name="pad_w">宽度填充。</param>
/// <param name="pad_h">高度填充。</param>
void YoloObjectTracker::PrepareInputData(UTextureRenderTarget2D* RenderTarget, TArray<float>& ProcessedData,
	float& OutScale, int32& OutPadW, int32& OutPadH,
	int32& OutOriginalWidth, int32& OutOriginalHeight)
{
	if (!RenderTarget) {
		UE_LOG(LogTemp, Error, TEXT("RenderTarget is null."));
		return;
	}

	FTextureRenderTargetResource* RenderTargetResource = RenderTarget->GameThread_GetRenderTargetResource();
	if (!RenderTargetResource) {
		UE_LOG(LogTemp, Error, TEXT("Failed to get RenderTargetResource."));
		return;
	}

	// 读取渲染目标的像素数据
	TArray<FColor> PixelData;
	if (!RenderTargetResource->ReadPixels(PixelData)) {
		UE_LOG(LogTemp, Error, TEXT("Failed to read pixels from render target."));
		return;
	}

	int32 OriginalWidth = RenderTarget->SizeX;
	int32 OriginalHeight = RenderTarget->SizeY;

	if (PixelData.Num() != OriginalWidth * OriginalHeight) {
		UE_LOG(LogTemp, Error, TEXT("PixelData size does not match render target dimensions."));
		return;
	}

	// 将像素数据转换为 OpenCV Mat 格式（BGRA）
	cv::Mat OriginalImage(OriginalHeight, OriginalWidth, CV_8UC4, PixelData.GetData());

	// 将 BGRA 转换为 BGR
	cv::Mat BGRImage;
	cv::cvtColor(OriginalImage, BGRImage, cv::COLOR_BGRA2BGR);

	// 使用 letterbox 方法调整图像尺寸并保持纵横比
	cv::Mat ResizedImage = LetterboxImage(BGRImage, 640, 640, OutScale, OutPadW, OutPadH);

	// 转换为浮点型并归一化到 0-1
	ResizedImage.convertTo(ResizedImage, CV_32FC3, 1.0f / 255.0f);

	// 如果模型需要 RGB 颜色顺序，可以在这里转换
	cv::cvtColor(ResizedImage, ResizedImage, cv::COLOR_BGR2RGB);

	// 转换为 CHW 格式（通道、高度、宽度）
	std::vector<cv::Mat> Channels(3);
	cv::split(ResizedImage, Channels);

	// 将数据展平成一维数组，按照 CHW 顺序
	ProcessedData.Reserve(640 * 640 * 3); // 预分配内存
	for (int c = 0; c < 3; ++c)
	{
		// 获取通道数据指针
		float* ChannelData = (float*)Channels[c].data;
		ProcessedData.Append(ChannelData, 640 * 640);
	}

	// 输出原始图像尺寸
	OutOriginalWidth = OriginalWidth;
	OutOriginalHeight = OriginalHeight;

	// **保存图片到本地以供测试**
   // // 选择保存目录：项目的 Saved/CapturedImages 目录
   // FString SaveDirectory = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("CapturedImages"));
   // IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();

   // // 检查目录是否存在，如果不存在则创建
   // if (!PlatformFile.DirectoryExists(*SaveDirectory))
   // {
   //     bool bCreated = PlatformFile.CreateDirectory(*SaveDirectory);
   //     if (!bCreated)
   //     {
   //         UE_LOG(LogTemp, Error, TEXT("Failed to create directory: %s"), *SaveDirectory);
   //         return;
   //     }
   // }

	//// 生成唯一的文件名，使用当前时间戳
	//FDateTime Now = FDateTime::UtcNow();
	//FString FileName = FString::Printf(TEXT("CapturedImage_%04d%02d%02d_%02d%02d%02d.png"),
	//    Now.GetYear(), Now.GetMonth(), Now.GetDay(),
	//    Now.GetHour(), Now.GetMinute(), Now.GetSecond());

	//FString FullPath = FPaths::Combine(SaveDirectory, FileName);

	//// 将 FString 转换为 std::string
	//std::string StdFullPath = std::string(TCHAR_TO_UTF8(*FullPath));

	//// 在保存之前，将颜色空间从线性转换为 Gamma 校正 事实证明问题不在于Gamma校正
	////cv::Mat GammaCorrected;
	////cv::cvtColor(BGRImage, GammaCorrected, cv::COLOR_BGR2RGB);
	////GammaCorrected = GammaCorrected * 255.0f;
	////GammaCorrected.convertTo(GammaCorrected, CV_8UC3);

	//// 使用 OpenCV 保存 ResizedImage 或 OriginalImage，根据需要选择
	//// 这里我们保存 BGRImage（原始捕捉的图像）
	//bool bSuccess = cv::imwrite(StdFullPath, BGRImage);
	//if (!bSuccess)
	//{
	//    UE_LOG(LogTemp, Error, TEXT("Failed to save captured image to %s"), *FullPath);
	//}
	//else
	//{
	//    UE_LOG(LogTemp, Log, TEXT("Successfully saved captured image to %s"), *FullPath);
	//}
}


/// <summary>
/// Resizes the input image to the target dimensions while maintaining the aspect ratio.
/// Pads the image with a specified color to fit the target dimensions.
/// </summary>
/// <param name="src">The source image to be resized.</param>
/// <param name="target_width">The target width for the resized image.</param>
/// <param name="target_height">The target height for the resized image.</param>
/// <returns>The resized and padded image.</returns>
cv::Mat YoloObjectTracker::LetterboxImage(const cv::Mat& src, int32 target_width, int32 target_height, float& scale, int32& pad_w, int32& pad_h)
{
	int32 width = src.cols;
	int32 height = src.rows;

	// 计算缩放比例，保持纵横比
	scale = std::min((float)target_width / width, (float)target_height / height);

	int32 new_width = int32(width * scale);
	int32 new_height = int32(height * scale);

	pad_w = (target_width - new_width) / 2;
	pad_h = (target_height - new_height) / 2;

	// 调整图像尺寸
	cv::Mat resized;
	cv::resize(src, resized, cv::Size(new_width, new_height));

	// 创建填充后的图像，默认填充颜色为 (114, 114, 114)
	cv::Mat output(target_height, target_width, src.type(), cv::Scalar(114, 114, 114));
	resized.copyTo(output(cv::Rect(pad_w, pad_h, new_width, new_height)));

	return output;
}


void YoloObjectTracker::ParseOutputData(const TArray<float>& OutputData, FVector2D TrackingPoint,
	float scale, int32 pad_w, int32 pad_h,
	int32 original_width, int32 original_height)
{
	// 输出此时追踪点
	UE_LOG(LogTemp, Log, TEXT("TrackingPoint: (%f, %f)"), TrackingPoint.X, TrackingPoint.Y);

	// 模型的输入尺寸
	const float ModelInputWidth = 640.0f;
	const float ModelInputHeight = 640.0f;

	// 距离阈值
	const float DistanceThreshold = 600.0f; // 根据需要调整

	// 置信度阈值
	const float ConfidenceThreshold = 0.3f;
	const int32 NumClasses = 80; // 类别数量

	// 假设 OutputData 的每个检测结果占用 85 个浮点数
	const int32 NumFeaturesPerDetection = 116;
	const int32 NumDetections = OutputData.Num() / NumFeaturesPerDetection;
	const int32 Nm = 32; // 掩膜数量（如果存在）

	// 检测结果数量
	UE_LOG(LogTemp, Log, TEXT("Number of detections: %d"), NumDetections);

	// 定义检测结果列表
	TArray<FDetectionResult> Detections;

	// CPP的输出是按照所有检测结果的所有特征块的顺序排列的
	// 定位各特征块的起始位置
	int32 OffsetCx = 0;
	int32 OffsetCy = OffsetCx + NumDetections;
	int32 OffsetW = OffsetCy + NumDetections;
	int32 OffsetH = OffsetW + NumDetections;
	int32 OffsetClassScores = OffsetH + NumDetections;
	int32 OffsetMasks = OffsetClassScores + NumDetections * NumClasses; // 如果包含掩膜

	for (int32 i = 0; i < NumDetections; ++i) {
		// 提取边界框特征
		float cx = OutputData[OffsetCx + i];
		float cy = OutputData[OffsetCy + i];
		float w = OutputData[OffsetW + i];
		float h = OutputData[OffsetH + i];

		// 提取最大类别分数和对应类别ID
		float MaxClassScore = -FLT_MAX;
		int32 ClassID = -1;
		for (int32 j = 0; j < NumClasses; ++j) {
			float ClassScore = OutputData[OffsetClassScores + i + NumDetections * j];
			if (ClassScore > MaxClassScore) {
				MaxClassScore = ClassScore;
				ClassID = j;
			}
		}
		// 应用置信度过滤
		if (MaxClassScore < ConfidenceThreshold) continue;

		// 计算边界框的左上角和右下角坐标
		float x1 = (cx - w / 2.0f - pad_w) / scale;
		float y1 = (cy - h / 2.0f - pad_h) / scale;
		float x2 = (cx + w / 2.0f - pad_w) / scale;
		float y2 = (cy + h / 2.0f - pad_h) / scale;

		// 确保坐标在原始图像范围内
		x1 = FMath::Clamp(x1, 0.0f, static_cast<float>(original_width));
		y1 = FMath::Clamp(y1, 0.0f, static_cast<float>(original_height));
		x2 = FMath::Clamp(x2, 0.0f, static_cast<float>(original_width));
		y2 = FMath::Clamp(y2, 0.0f, static_cast<float>(original_height));

		// 存储检测结果
		FDetectionResult Detection = { x1, y1, x2, y2, MaxClassScore, ClassID };
		Detections.Add(Detection);
	}

	//for (int32 i = 0; i < NumDetections; ++i)
	//{
	//    int32 Offset = i * NumFeaturesPerDetection;

	//    // 找到最大类别分数和对应的类别ID
	//    float MaxClassScore = -FLT_MAX;
	//    int32 ClassID = -1;
	//    for (int32 j = 4; j < 4 + NumClasses; ++j) {
	//        float ClassScore = OutputData[Offset + j];
	//        if (ClassScore > MaxClassScore) {
	//            MaxClassScore = ClassScore;
	//            ClassID = j - 4;
	//        }
	//    }
	//    // 使用最大类别分数过滤
	//    if (MaxClassScore < ConfidenceThreshold) continue; // 跳过低置信度项

	//    // 获取边界框坐标（cx, cy, w, h）基于预处理后的图像
	//    float cx = OutputData[Offset];
	//    float cy = OutputData[Offset + 1];
	//    float w = OutputData[Offset + 2];
	//    float h = OutputData[Offset + 3];

	//    // 转换为左上角和右下角坐标（调整为原始图像坐标系）
	//    float x1 = (cx - w / 2.0f - pad_w) / scale;
	//    float y1 = (cy - h / 2.0f - pad_h) / scale;
	//    float x2 = (cx + w / 2.0f - pad_w) / scale;
	//    float y2 = (cy + h / 2.0f - pad_h) / scale;

	//    // 确保坐标在原始图像范围内
	//    x1 = FMath::Clamp(x1, 0.0f, static_cast<float>(original_width));
	//    y1 = FMath::Clamp(y1, 0.0f, static_cast<float>(original_height));
	//    x2 = FMath::Clamp(x2, 0.0f, static_cast<float>(original_width));
	//    y2 = FMath::Clamp(y2, 0.0f, static_cast<float>(original_height));
	// 
	//    // 创建检测结果对象
	//    FDetectionResult Detection;
	//    Detection.x1 = x1;
	//    Detection.y1 = y1;
	//    Detection.x2 = x2;
	//    Detection.y2 = y2;
	//    Detection.Confidence = MaxClassScore;
	//    Detection.ClassID = ClassID;

	//    Detections.Add(Detection);
	//}

	// 应用自定义 NMS 过滤
	float IoUThreshold = 0.55f; // 根据需要调整
	ApplyNMS(Detections, IoUThreshold);

	// 寻找距离 TrackingPoint 最近的检测结果
	float MinDistance = FLT_MAX;
	bool bFound = false;
	FDetectionResult ClosestDetection;

	for (const FDetectionResult& Detection : Detections)
	{
		// 如果已经有初始跟踪的类别，只考虑相同类别的目标
		if (bHasInitialTracking && Detection.ClassID != TrackedClassID)
		{
			continue;
		}

		// 计算检测结果的中心点
		float DetectionCenterX = (Detection.x1 + Detection.x2) / 2.0f;
		float DetectionCenterY = (Detection.y1 + Detection.y2) / 2.0f;

		// 计算与 TrackingPoint 的距离
		float Distance = FMath::Sqrt(FMath::Pow(DetectionCenterX - TrackingPoint.X, 2) + FMath::Pow(DetectionCenterY - TrackingPoint.Y, 2));

		// 找到距离 TrackingPoint 最近的检测结果
		if (Distance < MinDistance)
		{
			MinDistance = Distance;
			bFound = true;
			ClosestDetection = Detection;

			TrackedPosition = FVector2D(DetectionCenterX, DetectionCenterY);
		}
	}

	// 判断是否找到有效的检测结果
	if (bFound && MinDistance < DistanceThreshold)
	{
		//FScopeLock Lock(&InferenceCriticalSection);
		TrackedDetection = ClosestDetection;

		if (!bHasInitialTracking && !ShouldStopTracking())
		{
			// 初始帧，记录 TrackedClassID
			TrackedClassID = ClosestDetection.ClassID;
			bHasInitialTracking = true;
			UE_LOG(LogTemp, Warning, TEXT("Initial tracking class ID: %d Confidence: %f "), TrackedClassID, TrackedDetection.Confidence);
		}

		bIsTracking = true;
	}
	else
	{
		//FScopeLock Lock(&InferenceCriticalSection);
		UE_LOG(LogTemp, Warning, TEXT("No valid detections found near the previous target with MinDistance: %f"), MinDistance);
		bIsTracking = false;
	}
}

FVector2D YoloObjectTracker::GetTrackedPosition() const
{
	return TrackedPosition;
}

const FDetectionResult& YoloObjectTracker::GetTrackedDetection() const
{
	return TrackedDetection;
}

void YoloObjectTracker::StopTracking()
{
	bIsTracking = false;
	bHasInitialTracking = false;
	ModelHelper->bIsRunning = false;
	bShouldStopTracking = true;
	UE_LOG(LogTemp, Warning, TEXT("Tracking has been stopped."));
}

bool YoloObjectTracker::IsTracking() const
{
	return bIsTracking;
}

bool YoloObjectTracker::ShouldStopTracking() const
{
	return bShouldStopTracking;
}


/*
Relic模型的初始化和推理

*/
NimaObjectTracker::NimaObjectTracker(const FString& ModelPath) {
	TObjectPtr<UNNEModelData> NimaModelData = LoadObject<UNNEModelData>(GetTransientPackage(), TEXT("/Game/Models/relic2_model.relic2_model"));
	ModelHelper = MakeShared<FMyModelHelper>();

	if (NimaModelData)
	{
		UE_LOG(LogTemp, Display, TEXT("NimaModelData loaded %s"), *NimaModelData->GetName());
		// You can use ManuallyLoadedModelData here to create a model and corresponding model instance(s)
		// ManuallyLoadedModelData will be unloaded when the current function returns
	}
	else
	{
		UE_LOG(LogTemp, Error, TEXT("NimaModelData is not loaded, please check the static path to your asset"));
	}

	TWeakInterfacePtr<INNERuntimeGPU> Runtime = UE::NNE::GetRuntime<INNERuntimeGPU>(FString("NNERuntimeORTDml")); // NNERuntimeORTDml 或者 NNERuntimeORTCpu
	if (Runtime.IsValid())
	{
		UE_LOG(LogTemp, Warning, TEXT("Successfully retrieved GPU runtime backend."));
		TSharedPtr<UE::NNE::IModelGPU> Model = Runtime->CreateModelGPU(NimaModelData);
		UE_LOG(LogTemp, Warning, TEXT("Model created"));
		if (Model.IsValid())
		{
			UE_LOG(LogTemp, Warning, TEXT("Model is valid."));
			ModelGPU = Model;
			if (!Initialize()) {
				UE_LOG(LogTemp, Error, TEXT("Initialization failed."));
				// Handle the failure, possibly set a flag or return early
				bIsInitialized = false;
				return;
			}
			else {
				UE_LOG(LogTemp, Warning, TEXT("Initialization successful."));
				bIsInitialized = true;
			}
		}
		else
		{
			UE_LOG(LogTemp, Error, TEXT("Failed to create a valid model."));
			return;
		}

	}
	else
	{
		UE_LOG(LogTemp, Error, TEXT("Failed to find GPU runtime backend"));

		//TArrayView<TWeakInterfacePtr<INNERuntime>> AllRuntimes = UE::NNE::GetAllRuntimes();
		// 获取所有注册的运行时名称
		TArray<FString> RuntimeNames = UE::NNE::GetAllRuntimeNames<INNERuntime>();

		// 用于存储所有运行时的指针
		TArray<TWeakInterfacePtr<INNERuntime>> AllRuntimes;

		// 遍历名称数组并获取对应的运行时
		for (const FString& Name : RuntimeNames)
		{
			TWeakInterfacePtr<INNERuntime> IterRuntime = UE::NNE::GetRuntime<INNERuntime>(Name);
			if (IterRuntime.IsValid())
			{
				AllRuntimes.Add(IterRuntime);
			}
		}

		UE_LOG(LogTemp, Log, TEXT("Available NNE Runtimes:"));
		for (const TWeakInterfacePtr<INNERuntime>& IterRuntime : AllRuntimes)
		{
			if (IterRuntime.IsValid())
			{
				FString RuntimeName = IterRuntime->GetRuntimeName();
				UE_LOG(LogTemp, Log, TEXT("Runtime: %s"), *RuntimeName);
			}
			else
			{
				UE_LOG(LogTemp, Warning, TEXT("Invalid runtime found."));
			}
		}

		return; // Add this line to exit the function
	}
}

bool NimaObjectTracker::ParseNimaInput(UTextureRenderTarget2D* RenderTarget, TArray<float>& ProcessedData, cv::Mat& BGRImage) {
	// 检查输入
	if (!RenderTarget) {
		UE_LOG(LogTemp, Error, TEXT("RenderTarget is null."));
		return false;
	}

	FTextureRenderTargetResource* RenderTargetResource = RenderTarget->GameThread_GetRenderTargetResource();
	if (!RenderTargetResource) {
		UE_LOG(LogTemp, Error, TEXT("Failed to get RenderTargetResource."));
		return false;
	}

	// 读取渲染目标的像素数据
	TArray<FColor> PixelData;
	if (!RenderTargetResource->ReadPixels(PixelData)) {
		UE_LOG(LogTemp, Error, TEXT("Failed to read pixels from render target."));
		return false;
	}

	int32 OriginalWidth = RenderTarget->SizeX;
	int32 OriginalHeight = RenderTarget->SizeY;

	// 将像素数据转换为 OpenCV Mat 格式
	cv::Mat OriginalImage(OriginalHeight, OriginalWidth, CV_8UC4, PixelData.GetData());

	// 转换为BGR格式
	//cv::Mat BGRImage;
	cv::cvtColor(OriginalImage, BGRImage, cv::COLOR_BGRA2BGR);

	// 调整图像大小为224x224
	cv::Mat ResizedImage;
	cv::resize(BGRImage, ResizedImage, cv::Size(224, 224));

	// 转换为float类型并归一化到0-1
	cv::Mat FloatImage;
	ResizedImage.convertTo(FloatImage, CV_32FC3, 1.0f / 255.0f);

	// 转换为RGB顺序
	cv::cvtColor(FloatImage, FloatImage, cv::COLOR_BGR2RGB);

	// ImageNet均值和标准差
	const float IMAGE_NET_MEAN[3] = { 0.485f, 0.456f, 0.406f };
	const float IMAGE_NET_STD[3] = { 0.229f, 0.224f, 0.225f };

	// 标准化处理
	std::vector<cv::Mat> Channels(3);
	cv::split(FloatImage, Channels);

	// 预分配输出数组大小 (224*224*3)
	ProcessedData.Empty();
	ProcessedData.Reserve(224 * 224 * 3);

	// 按CHW顺序处理数据，同时进行标准化
	for (int c = 0; c < 3; ++c) {
		for (int h = 0; h < 224; ++h) {
			for (int w = 0; w < 224; ++w) {
				float pixel = Channels[c].at<float>(h, w);
				// 应用ImageNet标准化
				float normalizedPixel = (pixel - IMAGE_NET_MEAN[c]) / IMAGE_NET_STD[c];
				ProcessedData.Add(normalizedPixel);
			}
		}
	}




	return true;
}

bool NimaObjectTracker::Initialize()
{
	// 调用基类初始化
	if (!YoloObjectTracker::Initialize())
	{
		UE_LOG(LogTemp, Error, TEXT("Base YoloObjectTracker initialization failed."));
		return false;
	}

	// 特定于 NIMA 的初始化（如果需要）

	UE_LOG(LogTemp, Log, TEXT("NimaObjectTracker initialized successfully."));
	return true;
}

//bool NimaObjectTracker::RunInference(UTextureRenderTarget2D* RenderTarget)
//{
//    
//    // 1. 先检查基本条件而不获取锁
//    if (!bIsInitialized) {
//        UE_LOG(LogTemp, Error, TEXT("Model is not initialized."));
//        return false;
//    }
//
//    // 2. 使用作用域锁检查运行状态
//    {
//        FScopeLock Lock(&InferenceCriticalSection);
//        if (ModelHelper->bIsRunning) {
//            UE_LOG(LogTemp, Warning, TEXT("Model is already running an inference."));
//            return false;
//        }
//        // 标记为运行中
//        ModelHelper->bIsRunning = true;
//    }
//
//    TArray<float> ProcessedData;
//	cv::Mat BGRImage; // 用于保存原始图像
//    ParseNimaInput(RenderTarget, ProcessedData, BGRImage);
//
//    if (ModelHelper->InputBindings.Num() == 0 || ModelHelper->InputBindings[0].Data == nullptr)
//    {
//        UE_LOG(LogTemp, Error, TEXT("InputBindings are not properly initialized."));
//        return false;
//    }
//    float* InputBindingData = static_cast<float*>(ModelHelper->InputBindings[0].Data);
//    FMemory::Memcpy(InputBindingData, ProcessedData.GetData(), ProcessedData.Num() * sizeof(float));
//
//    ModelHelper->bIsRunning = true;
//    TSharedPtr<FMyModelHelper> ModelHelperPtr = ModelHelper;
//
//    AsyncTask(ENamedThreads::AnyNormalThreadNormalTask, [ModelHelperPtr, BGRImage, RenderTarget, this]()
//        {
//            FScopeLock Lock(&InferenceCriticalSection);
//            if (ModelHelperPtr->ModelInstance->RunSync(ModelHelperPtr->InputBindings, ModelHelperPtr->OutputBindings) == UE::NNE::EResultStatus::Ok)
//            {
//                // 在基类的推理完成后，解析 NIMA 的输出
//                // 假设 OutputData 现在包含了 NIMA 模型的输出
//                float LocalScore;
//                if (ModelHelper->OutputData.Num() > 0)
//                {
//                    ParseNimaOutput(ModelHelper->OutputData[0], Score);
//                    LocalScore = Score;
//                    //UE_LOG(LogTemp, Log, TEXT("Score after parsing: %f"), Score);
//                    // 根据需要使用 Score 进行后续处理
//                    // GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Green, FString::Printf(TEXT("NIMA Score: %f"), Score));
//                    // UE_LOG(LogTemp, Warning, TEXT("NIMA Score: %f"), Score);
//				}
//				else
//				{
//					UE_LOG(LogTemp, Error, TEXT("No output data found."));
//				}
//
//				bool LocalIfSaveImage = this->IfSaveImage;
//                AsyncTask(ENamedThreads::GameThread, [ModelHelperPtr, BGRImage, LocalScore, LocalIfSaveImage,this]()
//                    {
//                        //UE_LOG(LogTemp, Log, TEXT("Inside AsyncTask GameThread Callback"));
//                        if (BGRImage.empty())
//                        {
//                            UE_LOG(LogTemp, Error, TEXT("BGRImage is empty"));
//                        }
//                        FScopeLock Lock(&InferenceCriticalSection);
//                        ModelHelperPtr->bIsRunning = false;
//
//                        if (LocalIfSaveImage && LocalScore != 0) {
//                            // 选择保存目录：项目的 Saved/CapturedImages 目录
//                            FString SaveDirectory = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("OriginalPathViews"));
//                            IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
//
//                            // 检查目录是否存在，如果不存在则创建
//                            if (!PlatformFile.DirectoryExists(*SaveDirectory))
//                            {
//                                bool bCreated = PlatformFile.CreateDirectory(*SaveDirectory);
//                                if (!bCreated)
//                                {
//                                    UE_LOG(LogTemp, Error, TEXT("Failed to create directory: %s"), *SaveDirectory);
//                                }
//                            }
//
//                            // 生成唯一的文件名，使用当前时间戳 UtcNow返回的是标准UTC时间
//                            /*FDateTime Now = FDateTime::UtcNow();
//                            FString FileName = FString::Printf(TEXT("CapturedImage_%04d%02d%02d_%02d%02d%02d.png"),
//                                Now.GetYear(), Now.GetMonth(), Now.GetDay(),
//                                Now.GetHour(), Now.GetMinute(), Now.GetSecond());*/
//
//                                // 或者这样格式化时间为字符串（例如：20231025_153045）
//                            FDateTime Now = FDateTime::Now();
//                            //FString Timestamp = Now.ToString(TEXT("CapturedImage_%Y%m%d_%H%M%S.png"));
//                            /*FString FileName = FString::Printf(TEXT("CapturedImage_%f_%04d%02d%02d_%02d%02d%02d.png"),
//                                LocalScore,
//                                Now.GetYear(), Now.GetMonth(), Now.GetDay(),
//                                Now.GetHour(), Now.GetMinute(), Now.GetSecond());*/
//
//                            FGuid UniqueID = FGuid::NewGuid();
//                            FString FileName = FString::Printf(TEXT("CapturedImage_%f_%04d%02d%02d_%02d%02d%02d_%03d_%s.png"),
//                                LocalScore,
//                                Now.GetYear(), Now.GetMonth(), Now.GetDay(),
//                                Now.GetHour(), Now.GetMinute(), Now.GetSecond(),
//                                Now.GetMillisecond(),
//                                *UniqueID.ToString());
//
//                            FString FullPath = FPaths::Combine(SaveDirectory, FileName);
//
//                            // 将 FString 转换为 std::string
//                            std::string StdFullPath = std::string(TCHAR_TO_UTF8(*FullPath));
//
//                            // 使用 OpenCV 保存 ResizedImage 或 OriginalImage，根据需要选择
//                            // 这里我们保存 BGRImage（原始捕捉的图像）
//                            bool bSuccess = cv::imwrite(StdFullPath, BGRImage);
//                            if (!bSuccess)
//                            {
//                                UE_LOG(LogTemp, Error, TEXT("Failed to save captured image to %s"), *FullPath);
//                            }
//                            else
//                            {
//                                //UE_LOG(LogTemp, Log, TEXT("Successfully saved captured image to %s"), *FullPath);
//                            }
//                        }
//                    });
//            }
//            else
//            {
//                UE_LOG(LogTemp, Error, TEXT("Failed to run model inference. Retrying..."));
//                ResetState(); // 重置状态
//                RunInference(RenderTarget); // 重试推理
//            }
//
//			
//        });
//
//    return true;
//}

bool NimaObjectTracker::RunInference(UTextureRenderTarget2D* RenderTarget)
{
	// 1. 先检查基本条件而不获取锁
	if (!bIsInitialized) {
		UE_LOG(LogTemp, Error, TEXT("Model is not initialized."));
		return false;
	}

	// 2. 使用作用域锁检查运行状态
	{
		FScopeLock Lock(&InferenceCriticalSection);
		if (ModelHelper->bIsRunning) {
			UE_LOG(LogTemp, Warning, TEXT("Model is already running an inference."));
			return false;
		}
		// 标记为运行中
		ModelHelper->bIsRunning = true;
	}

	// 3. 创建预处理任务 - 在后台线程处理图像
	AsyncTask(ENamedThreads::GameThread, [this, RenderTarget]() {
		// 创建本地变量
		TArray<float> ProcessedData;
		cv::Mat BGRImageCopy; // 使用副本避免生命周期问题
		bool bPreprocessingSuccess = false;

		// 预处理图像
		{
			bPreprocessingSuccess = ParseNimaInput(RenderTarget, ProcessedData, BGRImageCopy);
			if (!bPreprocessingSuccess) {

				FScopeLock Lock(&InferenceCriticalSection);
				ModelHelper->bIsRunning = false;
				UE_LOG(LogTemp, Error, TEXT("Failed to preprocess input image."));

				return;
			}
		}

		// 复制数据到模型输入
		{
			FScopeLock Lock(&InferenceCriticalSection);
			if (ModelHelper->InputBindings.Num() == 0 || ModelHelper->InputBindings[0].Data == nullptr) {
				ModelHelper->bIsRunning = false;
				UE_LOG(LogTemp, Error, TEXT("InputBindings are not properly initialized."));
				return;
			}

			float* InputBindingData = static_cast<float*>(ModelHelper->InputBindings[0].Data);
			FMemory::Memcpy(InputBindingData, ProcessedData.GetData(), ProcessedData.Num() * sizeof(float));
		}

		// 4. 运行推理，添加超时机制
		bool bInferenceSuccess = false;
		{
			const double StartTime = FPlatformTime::Seconds();
			const double TimeoutSeconds = 5.0; // 设置合理的超时时间

			UE::NNE::EResultStatus Status = ModelHelper->ModelInstance->RunSync(
				ModelHelper->InputBindings, ModelHelper->OutputBindings);

			bInferenceSuccess = (Status == UE::NNE::EResultStatus::Ok);

			if (!bInferenceSuccess) {
				UE_LOG(LogTemp, Error, TEXT("Inference failed with status: %d"), static_cast<int32>(Status));
			}

			const double ElapsedTime = FPlatformTime::Seconds() - StartTime;
			//UE_LOG(LogTemp, Log, TEXT("Inference took %.4f seconds"), ElapsedTime);

			if (ElapsedTime > TimeoutSeconds) {
				UE_LOG(LogTemp, Warning, TEXT("Inference took longer than expected (%.2f seconds)"), ElapsedTime);
			}
		}

		// 5. 处理结果
		if (bInferenceSuccess) {
			float LocalScore = 0.0f;
			bool LocalSaveImage = false;

			{
				FScopeLock Lock(&InferenceCriticalSection);
				if (ModelHelper->OutputData.Num() > 0) {
					ParseNimaOutput(ModelHelper->OutputData[0], Score);
					LocalScore = Score;
				}
				LocalSaveImage = IfSaveImage;
			}

			// 6. 在游戏线程上更新状态和保存图像(如果需要)

				// 一定要先释放锁再处理耗时的图像保存操作
			{
				FScopeLock Lock(&InferenceCriticalSection);
				ModelHelper->bIsRunning = false;
			}

			if (LocalSaveImage && LocalScore > 0.0f && !BGRImageCopy.empty()) {
				SaveImageToFile(BGRImageCopy, LocalScore);
			}

		}
		else {
			// 失败处理
				FScopeLock Lock(&InferenceCriticalSection);
				ModelHelper->bIsRunning = false;
				Score = 0.0f;
				UE_LOG(LogTemp, Error, TEXT("Inference failed."));
		}

		//释放存储
		// 无论成功与否都确保释放资源
		BGRImageCopy.release(); // 显式释放OpenCV矩阵
		ProcessedData.Empty();  // 释放预处理数据

		});

	return true;
}

// 新增辅助函数处理图像保存 - 分离耗时操作
void NimaObjectTracker::SaveImageToFile(const cv::Mat& Image, float _Score)
{
	if (Image.empty()) {
		UE_LOG(LogTemp, Error, TEXT("Cannot save empty image"));
		return;
	}

	// 选择保存目录：项目的 Saved/OriginalPathViews 目录
	FString SaveDirectory = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("OriginalPathViews"));
	IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();

	// 检查目录是否存在，如果不存在则创建
	if (!PlatformFile.DirectoryExists(*SaveDirectory)) {
		bool bCreated = PlatformFile.CreateDirectory(*SaveDirectory);
		if (!bCreated) {
			UE_LOG(LogTemp, Error, TEXT("Failed to create directory: %s"), *SaveDirectory);
			return;
		}
	}

	// 生成唯一的文件名
	FDateTime Now = FDateTime::Now();
	FGuid UniqueID = FGuid::NewGuid();
	FString FileName = FString::Printf(TEXT("CapturedImage_%f_%04d%02d%02d_%02d%02d%02d_%03d_%s.png"),
		_Score,
		Now.GetYear(), Now.GetMonth(), Now.GetDay(),
		Now.GetHour(), Now.GetMinute(), Now.GetSecond(),
		Now.GetMillisecond(),
		*UniqueID.ToString());

	FString FullPath = FPaths::Combine(SaveDirectory, FileName);
	std::string StdFullPath = std::string(TCHAR_TO_UTF8(*FullPath));

	// 使用OpenCV保存图像到文件
	bool bSuccess = cv::imwrite(StdFullPath, Image);
	if (!bSuccess) {
		UE_LOG(LogTemp, Error, TEXT("Failed to save image to %s"), *FullPath);
	}
}


void NimaObjectTracker::ParseNimaOutput(const TArray<float>& OutputData, float& _Score)
{
	// 假设 NIMA 的输出是一个长度为10的数组，表示各个评分的概率
	if (OutputData.Num() != 10)
	{
		UE_LOG(LogTemp, Error, TEXT("Unexpected NIMA output size: %d"), OutputData.Num());
		_Score = 0.0f;
		return;
	}

	// 计算加权评分
	float WeightedSum = 0.0f;
	float Total = 0.0f;
	for (int32 i = 0; i < OutputData.Num(); ++i)
	{
		WeightedSum += OutputData[i] * (i + 1); // 分数从1到10
		Total += OutputData[i];
	}

	if (Total > 0.0f)
	{
		_Score = WeightedSum / Total;
	}
	else
	{
		_Score = 0.0f;
	}
}

float NimaObjectTracker::GetNimaScore() const
{
	return Score;
}

void NimaObjectTracker::ResetNimaScore()
{
	Score = 0.0f;
}

void NimaObjectTracker::ResetState()
{
	FScopeLock Lock(&InferenceCriticalSection); // 确保线程安全
	Score = 0.0f; // 重置评分
	IfSaveImage = false; // 重置保存图像标志
	ModelHelper->bIsRunning = false; // 重置运行状态
	UE_LOG(LogTemp, Warning, TEXT("NimaTracker state has been reset."));
}


void NimaObjectTracker::CleanupResources()
{
	// 检测当前线程
	if (IsInGameThread())
	{
		// 已经在游戏线程上，直接执行清理
		CleanupResourcesInternal();
	}
	else
	{
		// 不在游戏线程上，创建一个可以等待的任务
		FGraphEventRef TaskCompleteEvent = FFunctionGraphTask::CreateAndDispatchWhenReady(
			[this]()
			{
				CleanupResourcesInternal();
			},
			TStatId(),
			nullptr,
			ENamedThreads::GameThread
		);

		// 等待任务完成
		FTaskGraphInterface::Get().WaitUntilTaskCompletes(TaskCompleteEvent, ENamedThreads::AnyThread);

		UE_LOG(LogTemp, Log, TEXT("CleanupResources: 异步资源清理任务已完成"));
	}
}

void NimaObjectTracker::CleanupResourcesInternal()
{
	// 确保在游戏线程上执行
	check(IsInGameThread());

	UE_LOG(LogTemp, Log, TEXT("CleanupResourcesInternal: 开始清理资源..."));

	// 指示清理状态
	bIsCleaningUp = true;

	// 标记为不运行状态，避免多线程冲突
	{
		FScopeLock Lock(&InferenceCriticalSection);
		ModelHelper->bIsRunning = false;
		Score = 0.0f; // 重置评分但不清除必要的数据结构
	}

	// 谨慎清理：仅刷新画面但不清空模型绑定
	if (GEngine)
	{
		// 只做轻量级的垃圾回收，不影响正在使用的对象
		GEngine->ForceGarbageCollection(false);

		// 显式调用显存释放命令
		ENQUEUE_RENDER_COMMAND(FlushGPUResources)(
			[](FRHICommandListImmediate& RHICmdList)
			{
				RHICmdList.ImmediateFlush(EImmediateFlushType::FlushRHIThreadFlushResources);
			});

		// 刷新渲染命令 (必须在游戏线程上调用)
		FlushRenderingCommands();
	}

	// 重要：不清空 ModelHelper->OutputData，因为它包含模型输出绑定
	// 但可以重置临时变量
	IfSaveImage = false;

	UE_LOG(LogTemp, Log, TEXT("CleanupResourcesInternal: 资源清理完成，保留了模型绑定"));

	bIsCleaningUp = false;
}


// Fill out your copyright notice in the Description page of Project Settings.

#include "DroneActor1.h"
#include <string>
#include <algorithm>
#include "HighResScreenshot.h"
#include "Misc/DateTime.h"
#include "ImageUtils.h"
#include "Rendering/RenderingCommon.h"
#include "RenderingThread.h"
#include "Kismet/GameplayStatics.h"

#include <Engine/SceneCapture2D.h>
#include "Engine/TextureRenderTarget2D.h"
#include "Blueprint/UserWidget.h"

#include "HAL/PlatformTime.h"

#include "CesiumRuntime.h"
#include "CesiumGeoreference.h"
#include "Cesium3DTileset.h"


#include "Public/MyRRTClass.h"



// Sets default values
ADroneActor1::ADroneActor1()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	// 创建相机组件并将其附加到根组件
	CameraComponent = CreateDefaultSubobject<UCameraComponent>(TEXT("CameraComponent"));
	// 如果这个Actor有根组件，比如一个静态网格体，可以将相机附加到它上面
	// 否则，将相机设置为根组件
	RootComponent = CameraComponent;

	SceneCaptureComponent = CreateDefaultSubobject<USceneCaptureComponent2D>(TEXT("SceneCaptureComponent"));
	SceneCaptureComponent->SetupAttachment(CameraComponent);
	SceneCaptureComponent->bCaptureEveryFrame = false; // 设置为false,手动调用CaptureScene()函数
	SceneCaptureComponent->bAlwaysPersistRenderingState = true; // 设置为true,使捕获的场景保持渲染状态

	FloatingMovement = CreateDefaultSubobject<UFloatingPawnMovement>(TEXT("FloatingPawnMovement"));
	FloatingMovement->UpdatedComponent = RootComponent;

	// 初始化 CesiumGlobeAnchor 组件
	GlobeAnchorComponent = CreateDefaultSubobject<UCesiumGlobeAnchorComponent>(TEXT("GlobeAnchorComponent"));

	// 创建并初始化 SplineComponent
	SplineComponent = CreateDefaultSubobject<USplineComponent>(TEXT("SplineComponent"));

	bIsRightMousePressed = false;

	bIsSlowMove = false;

	bIsShowDebugPoints = false;
	bIsShowGlobalPath = true;
	bIsShowLocalOriginalPath = false;
	bIsSavePhotos = false;

	SpeedMultiplier = 30.0f;

	fMinFlightSpeed = 300.0f;
	fMaxFlightSpeed = 2000.0f;

	SingleScore = 0;

	currentIndex = 0;
	// 初始化移动速度和旋转插值速度
	MoveSpeed = 2000.0f;          // 调整无人机移动速度 10m/s?
	RotationInterpSpeed = 2.0f;   // 调整旋转插值速度
	FOVInterpSpeed = 2.0f; // 调整FOV插值速度
	bIsFlyingAlongPath = false;   // 默认未开始飞行

	bShouldDrawPathPoints = false;
	fGenerationFinished = false;

	fMinHeight = 0.0f;
	fMaxHeight = 10000.0f;

	fMinDisBetwenPoints = 100.0f; // 路径点之间的最小距离

	// 初始化起点和终点
	StartLocation = FVector(0, 0, 0);
	EndLocation = FVector(0, 0, 0);
	

	// 初始化鼠标灵敏度
	MouseSensitivity = 1.0f;
	// 启用鼠标输入
	AutoPossessPlayer = EAutoReceiveInput::Player0;
	bUseControllerRotationPitch = false;
	bUseControllerRotationYaw = false;

	TimeSinceLastInference = 0.0f;
	InferenceInterval = 0.2f; // 例如，每0.1秒调用一次推理

	CurrentState = EInterestPointState::Idle;

}

// Called when the game starts or when spawned
void ADroneActor1::BeginPlay()
{
	Super::BeginPlay();

	if (!GetViewportSize(ViewportWidth, ViewportHeight))
	{
		UE_LOG(LogTemp, Error, TEXT("Failed to get viewport size."));

	}

	// 创建并添加 Widget
	if (SpeedDisplayWidgetClass)
	{
		UUserWidget* SpeedWidget = CreateWidget<UUserWidget>(GetWorld(), SpeedDisplayWidgetClass);
		if (SpeedWidget)
		{
			SpeedWidget->AddToViewport();
		}
	}

	// 延迟初始化 RenderTarget
	FTimerHandle TimerHandle;
	GetWorld()->GetTimerManager().SetTimer(TimerHandle, this, &ADroneActor1::InitializeRenderTarget, 0.1f, false);

	// 初始化 RRTClass
	UWorld* World = GetWorld();
	if (!World)
	{
		UE_LOG(LogTemp, Error, TEXT("Failed to get valid World pointer"));
		return;
	}

	GlobalRRTClass = NewObject<UMyRRTClass>();
	if (!GlobalRRTClass)
	{
		UE_LOG(LogTemp, Error, TEXT("Failed to create RRTClass"));
		return;
	}

	GlobalRRTClass->SetWorld(World);

	// 查找并保存 CesiumGeoreference 引用
	for (TActorIterator<ACesiumGeoreference> It(GetWorld()); It; ++It)
	{
		CesiumGeoreference = *It;
		break;
	}
	if (!CesiumGeoreference)
	{
		GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Red, TEXT("CesiumGeoreference not found"));
	}

	if (Controller != nullptr && Controller->IsA<APlayerController>())
	{
		PlayerController = Cast<APlayerController>(Controller);
		FVector CameraLocation;
		FRotator CameraRotation;
		PlayerController->PlayerCameraManager->GetCameraViewPoint(CameraLocation, CameraRotation);
		/* SceneCaptureComponent->SetWorldLocation(CameraLocation);
		 SceneCaptureComponent->SetWorldRotation(CameraRotation);*/

		 // 匹配 FOV
		float MainCameraFOV = PlayerController->PlayerCameraManager->GetFOVAngle();
		SceneCaptureComponent->FOVAngle = MainCameraFOV;


		// 设置鼠标光标和点击事件
		PlayerController->bShowMouseCursor = true;
		PlayerController->bEnableClickEvents = true;
		PlayerController->bEnableMouseOverEvents = true;
		PlayerController->SetMouseLocation(FMath::TruncToInt(GEngine->GameViewport->Viewport->GetSizeXY().X / 2.0f), FMath::TruncToInt(GEngine->GameViewport->Viewport->GetSizeXY().Y / 2.0f));

		// 输出消息证明 PlayerController 获取成功
		GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Green, TEXT("PlayerController successfully obtained!"));
		UE_LOG(LogTemp, Warning, TEXT("PlayerController successfully obtained!"));

		if (CameraComponent)
		{
			// 复制相机的FOV (视野)
			SceneCaptureComponent->FOVAngle = CameraComponent->FieldOfView;

			SceneCaptureComponent->ShowFlags = FEngineShowFlags(EShowFlagInitMode::ESFIM_Game); // 设置与游戏视口一致的渲染标志

			// 复制后期处理设置
			//SceneCaptureComponent->ShowFlags.SetStaticMeshes(true);      // 启用静态网格
			//SceneCaptureComponent->ShowFlags.SetInstancedFoliage(true);  // 启用实例化网格
			//SceneCaptureComponent->ShowFlags.SetLighting(true);          // 启用光照
			//SceneCaptureComponent->ShowFlags.SetDynamicShadows(true);    // 启用动态阴影
			//SceneCaptureComponent->ShowFlags.SetPostProcessing(true);    // 启用后处理
			//SceneCaptureComponent->ShowFlags.SetNaniteMeshes(true);      // 启用 Nanite 网格
			//SceneCaptureComponent->ShowFlags.SetTranslucency(true);      // 启用透明度


			SceneCaptureComponent->PostProcessSettings = CameraComponent->PostProcessSettings; // 必要！复制所有摄像机设置
			SceneCaptureComponent->PostProcessSettings.bOverride_MotionBlurAmount = true;
			SceneCaptureComponent->PostProcessSettings.MotionBlurAmount = 0.0f;
			SceneCaptureComponent->PostProcessBlendWeight = 1.0f; // 完全应用后处理效果
			
			/*SceneCaptureComponent->bOverride_CustomNearClippingPlane = CameraComponent->bUseCustomNearClippingPlane;
			SceneCaptureComponent->CustomNearClippingPlane = CameraComponent->CustomNearClippingPlane;*/

			// 将其他相机设置应用于SceneCaptureComponent
			SceneCaptureComponent->ProjectionType = CameraComponent->ProjectionMode;
			if (CameraComponent->ProjectionMode == ECameraProjectionMode::Orthographic)
			{
				SceneCaptureComponent->OrthoWidth = CameraComponent->OrthoWidth;
			}

			UE_LOG(LogTemp, Log, TEXT("Camera settings successfully copied to SceneCaptureComponent."));
		}
	}

	
	if (GlobeAnchorComponent)
	{
		GlobeAnchorComponent->SetAdjustOrientationForGlobeWhenMoving(true);

		if (CesiumGeoreference && GlobeAnchorComponent)
		{
			// 设置 GlobeAnchorComponent 的 Georeference
			GlobeAnchorComponent->SetGeoreference(CesiumGeoreference);
		}
		else
		{
			UE_LOG(LogTemp, Error, TEXT("Failed to cast to CesiumGeoreference or GlobeAnchorComponent is null."));
		}

		//// 查找场景中的 CesiumGeoreference
		//TArray<AActor*> GeoreferenceActors;
		//UGameplayStatics::GetAllActorsOfClass(GetWorld(), ACesiumGeoreference::StaticClass(), GeoreferenceActors);

		//if (GeoreferenceActors.Num() > 0)
		//{
		//    ACesiumGeoreference* CesiumGeoreference = Cast<ACesiumGeoreference>(GeoreferenceActors[0]);
		//    
		//}
		//else
		//{
		//    UE_LOG(LogTemp, Error, TEXT("No CesiumGeoreference found in the scene."));
		//}
	}
	else
	{
		UE_LOG(LogTemp, Error, TEXT("GlobeAnchorComponent is null."));
	}


	// ------------------- ONNX 模型加载 -------------------

	/*FString ONNX_ModelPath = FPaths::Combine(*FPaths::ProjectDir(), TEXT("Resources"), TEXT("pretrain_model/path_to_save_model.onnx"));
#ifdef _WIN32
	const ORTCHAR_T* model_path = *ONNX_ModelPath;
#else
	const ORTCHAR_T* model_path = TCHAR_TO_ANSI(*ModelPath);
#endif

	Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "test");
	Ort::SessionOptions session_options(nullptr);

	currentONNXModel = std::make_unique<Ort::Session>(env, model_path, session_options);*/
	//currentONNXModel = LoadONNXModel(model_path,env,session_options);


	//// 创建一个1x1的张量
	//torch::Tensor tensor = torch::rand({ 1, 1 });
	//// 打印张量的值
	//UE_LOG(LogTemp, Warning, TEXT("Tensor value: %f"), tensor.item<float>());
	//UE_LOG(LogTemp, Warning, TEXT("Tensor value"));



	// ------------------- YOLO 模型加载 -------------------
	// Specify the local path to your YOLO model file
	// 无法直接引入ONNX文件，依然需要先导入为Asset，然后获取路径
	ModelPath_Yolo = FPaths::Combine(TEXT("Game"), TEXT("/Models/yolov11m-seg.yolov11m-seg"));

	//ModelPath = FPaths::ConvertRelativePathToFull(FPaths::ProjectDir() + TEXT("Resources")+TEXT("/pretrain_model/yolov8s.onnx"));

	UE_LOG(LogTemp, Warning, TEXT("YOLO Model Path: %s"), *ModelPath_Yolo);

	// Initialize the YoloObjectTracker
	YoloTracker = MakeShared<YoloObjectTracker>(ModelPath_Yolo);

	/*if (!YoloTracker->Initialize())
	{
		UE_LOG(LogTemp, Error, TEXT("Failed to initialize YoloObjectTracker."));
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("YoloObjectTracker initialized successfully."));
	}*/

	// ------------------- NIMA 模型加载 -------------------
	ModelPath_Nima = FPaths::Combine(TEXT("Game"), TEXT("/Models/relic2_model.relic2_model"));

	NimaTracker = MakeShared<NimaObjectTracker>(ModelPath_Nima);
}


bool ADroneActor1::GetViewportSize(int32& OutWidth, int32& OutHeight)
{
	if (GEngine && GEngine->GameViewport)
	{
		FViewport* Viewport = GEngine->GameViewport->Viewport;
		if (Viewport)
		{
			FIntPoint ViewportSize = Viewport->GetSizeXY();
			OutWidth = ViewportSize.X;
			OutHeight = ViewportSize.Y;
			return true;
		}
	}
	return false;
}

void ADroneActor1::InitializeRenderTarget()
{
	if (!GetViewportSize(ViewportWidth, ViewportHeight))
	{
		UE_LOG(LogTemp, Error, TEXT("Failed to get viewport size."));
		return;
	}

	if (ViewportWidth <= 0 || ViewportHeight <= 0)
	{
		UE_LOG(LogTemp, Error, TEXT("Invalid viewport size: %dx%d"), ViewportWidth, ViewportHeight);
		return;
	}

	if (!RenderTarget)
	{
		RenderTarget = NewObject<UTextureRenderTarget2D>(this, TEXT("ScreenShot"));
		RenderTarget->InitAutoFormat(ViewportWidth, ViewportHeight);
		RenderTarget->UpdateResourceImmediate(true);
		SceneCaptureComponent->TextureTarget = RenderTarget;
		SceneCaptureComponent->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;

		UE_LOG(LogTemp, Log, TEXT("RenderTarget initialized with size: %dx%d"), ViewportWidth, ViewportHeight);
	}
}


void ADroneActor1::PrecomputeAllDuration()
{
	GlobalFlightDurations.Empty();
	for (int i = 0;i < GlobalPathPoints.Num();i++) {
		FPathPointWithOrientation localOriginalPathPoint;
		if (i >= 1)
		{
			localOriginalPathPoint = GlobalPathPoints[i - 1];
		}
		else {
			localOriginalPathPoint = OriginalPathPoint;
		}

		FRotator localStartRotation = localOriginalPathPoint.Orientation;
		FRotator localTargetRotation = GlobalPathPoints[i].Orientation;

		// 计算角度差
		float YawDiff = FMath::Abs(FMath::FindDeltaAngleDegrees(localStartRotation.Yaw, localTargetRotation.Yaw));
		float PitchDiff = FMath::Abs(FMath::FindDeltaAngleDegrees(localStartRotation.Pitch, localTargetRotation.Pitch));
		float MaxAngleDiff = FMath::Max(YawDiff, PitchDiff);

		// 计算到下一个路径点的距离
		float DistanceToTarget = FVector::Dist(localOriginalPathPoint.Point, GlobalPathPoints[i].Point);

		// 计算旋转所需时间，基于转角大小和期望旋转速度
		float RotationSpeedDegreesPerSecond = 45.0f; // 每秒旋转多少度
		float MinRotationDuration = MaxAngleDiff / RotationSpeedDegreesPerSecond;

		// 计算当前段的期望速度
		float SegmentSpeed;
		if (i == 0) {
			SegmentSpeed = GlobalPathPoints[i].SegmentSpeed;
		}
		else {
			SegmentSpeed = GlobalPathPoints[i - 1].SegmentSpeed; // 否则使用出发点的速度
		}

		float MoveDuration = DistanceToTarget / SegmentSpeed;

		// 根据是否慢速模式，决定最终旋转时长
		float localTototalRotationDurtation;
		if (bIsSlowMove)
		{
			float ForcedMinRotationDuration = 3.0f;
			// 慢速移动模式，旋转时间取移动和旋转中更大的那个
			localTototalRotationDurtation = FMath::Max(MoveDuration, ForcedMinRotationDuration);
		}
		else
		{
			// 常规模式，允许更快旋转
			localTototalRotationDurtation = MoveDuration;
		}

		GlobalFlightDurations.Add(localTototalRotationDurtation);
	}

	UE_LOG(LogTemp, Warning, TEXT("GlobalFlightDurations Calculation Completed: %d"), GlobalFlightDurations.Num());
}


void ADroneActor1::StartNewPathPoint ()
{
	CurrentRotationTime = 0.0f;

	//// currentIndex=0的时候已经定义了
	if (currentIndex >= 1)
	{
		OriginalPathPoint = GlobalPathPoints[currentIndex - 1];
	}

	StartRotation = OriginalPathPoint.Orientation;
	TargetRotation = GlobalPathPoints[currentIndex].Orientation;

	//// 计算角度差
	//float YawDiff = FMath::Abs(FMath::FindDeltaAngleDegrees(StartRotation.Yaw, TargetRotation.Yaw));
	//float PitchDiff = FMath::Abs(FMath::FindDeltaAngleDegrees(StartRotation.Pitch, TargetRotation.Pitch));
	//float MaxAngleDiff = FMath::Max(YawDiff, PitchDiff);

	//// 计算到下一个路径点的距离
	//float DistanceToTarget = FVector::Dist(OriginalPathPoint.Point, GlobalPathPoints[currentIndex].Point);

	//// 计算旋转所需时间，基于转角大小和期望旋转速度
	//float RotationSpeedDegreesPerSecond = 45.0f; // 每秒旋转多少度
	//float MinRotationDuration = MaxAngleDiff / RotationSpeedDegreesPerSecond;

	// 计算当前段的期望速度
	float SegmentSpeed;
	if (currentIndex == 0) {
		SegmentSpeed = GlobalPathPoints[currentIndex].SegmentSpeed;
	}
	else {
		SegmentSpeed = GlobalPathPoints[currentIndex-1].SegmentSpeed; // 否则使用出发点的速度
	}
	CurrentSpeed = SegmentSpeed; // 设置当前速度

	// 计算本段飞行的预计时间
	// 考虑到加速和减速，使用平均速度来估算
	//float AvgSpeed = (CurrentSpeed + SegmentSpeed) / 2.0f;
	//if (AvgSpeed < 1.0f) // 避免除0
	//{
	//	AvgSpeed = 1.0f;
	//}
	
	//float MoveDuration = DistanceToTarget / SegmentSpeed;

	//// 根据是否慢速模式，决定最终旋转时长
	//if (bIsSlowMove)
	//{
	//	float ForcedMinRotationDuration = 3.0f;
	//	// 慢速移动模式，旋转时间取移动和旋转中更大的那个
	//	TotalRotationDuration = FMath::Max(MoveDuration, ForcedMinRotationDuration);
	//}
	//else
	//{
	//	// 常规模式，允许更快旋转
	//	TotalRotationDuration = FMath::Max(MoveDuration, MinRotationDuration);
	//}

	TotalRotationDuration = GlobalFlightDurations[currentIndex];

	if (bIsSavePhotos)
	{
		OnCaptureScreenshotWithUI(false); // 如果需要截图
	}

	// 可以根据需要添加调试信息
	/*
	UE_LOG(LogTemp, Warning, TEXT("Start Rotation: %s, Target Rotation: %s"), *StartRotation.ToString(), *TargetRotation.ToString());
	UE_LOG(LogTemp, Warning, TEXT("Yaw Diff: %f, Pitch Diff: %f, Max Angle Diff: %f, Total Rotation Duration: %f"),
		YawDiff, PitchDiff, MaxAngleDiff, TotalRotationDuration);
	*/
}




//void ADroneActor1::UpdateActorRotation(float DeltaTime)
//{
//	CurrentRotationTime += DeltaTime;
//	float Alpha = FMath::Clamp(CurrentRotationTime / TotalRotationDuration, 0.0f, 1.0f);
//
//	// 通过插值平滑过渡旋转速度，避免旋转速度突变
//	/*float SmoothRotationSpeed = FMath::Lerp(LastRotationSpeed, CurrentRotationSpeed, Alpha);
//	float SmoothedAlpha = FMath::InterpEaseInOut(0.0f, 1.0f, Alpha, SmoothRotationSpeed);*/
//
//	FQuat StartQuat = StartRotation.Quaternion();
//	FQuat TargetQuat = TargetRotation.Quaternion();
//	FQuat NewQuat = FQuat::Slerp(StartQuat, TargetQuat, Alpha);
//
//	SetActorRotation(NewQuat);
//	if (PlayerController != nullptr)
//	{
//		PlayerController->SetControlRotation(NewQuat.Rotator());
//	}
//	float StartFOV = OriginalPathPoint.FOV;
//	//float CurrentFOV = CameraComponent->FieldOfView;
//	float TargetFOV = GlobalPathPoints[currentIndex].FOV;
//	float NewFOV = FMath::Lerp(StartFOV, TargetFOV, Alpha);
//	CameraComponent->SetFieldOfView(NewFOV);
//}

void ADroneActor1::DrawDebugHelpers(bool bIfShowDebugAreas)
{
	if (PlayerController != nullptr)
	{
		FVector CameraLocation;
		FRotator CameraRotation;
		PlayerController->PlayerCameraManager->GetCameraViewPoint(CameraLocation, CameraRotation);

		FVector WorldLocation, WorldDirection;
		if (PlayerController->DeprojectMousePositionToWorld(WorldLocation, WorldDirection))
		{
			FVector Start = WorldLocation;
			FVector End = Start + (WorldDirection * 100000.0f);

			FHitResult HitResult;
			FCollisionQueryParams Params;
			Params.AddIgnoredActor(this);

			if (GetWorld()->LineTraceSingleByChannel(HitResult, Start, End, ECC_Visibility, Params))
			{
				FVector HitLocation = HitResult.Location;
				// 根据状态绘制实时辅助线和圆形半径
				if (CurrentState == EInterestPointState::CenterSelected)
				{
					// 在第一个点平面绘制水平辅助线
					float PlaneZ = CenterPoint.Z;
					float DistanceToPlane = (PlaneZ - Start.Z) / WorldDirection.Z;
					FVector _ProjectedLocation = Start + DistanceToPlane * WorldDirection; // 当前投影位置

					DrawDebugLine(GetWorld(), CenterPoint, _ProjectedLocation, FColor::Red, false, -1.0f, 0, 2.0f);
				}
				else if (CurrentState == EInterestPointState::RadiusSelected || CurrentState == EInterestPointState::HeightSelected)
				{
					// 在选择高度时绘制圆形半径
					DrawDebugLine(GetWorld(), CenterPoint, SecondPoint, FColor::Red, bIfShowDebugAreas, 1.0f, 0, 10.0f);
					DrawDebugCircle(GetWorld(), CenterPoint, Radius, 50, FColor::Blue, bIfShowDebugAreas, 1.0f, 0, 10.0f, FVector(1, 0, 0), FVector(0, 1, 0), false);

					if (CurrentState == EInterestPointState::RadiusSelected)
					{
						// 获取相机位置并计算垂直平面的法向量
						FVector ViewDirection = (CameraLocation - SecondPoint).GetSafeNormal();
						FVector IntermediateVector = FVector::CrossProduct(FVector::UpVector, ViewDirection).GetSafeNormal();
						FVector PlaneNormal = FVector::CrossProduct(IntermediateVector, FVector::UpVector).GetSafeNormal();

						// 计算鼠标射线与垂直平面的交点
						float DotProduct = FVector::DotProduct(WorldDirection, PlaneNormal);
						if (FMath::Abs(DotProduct) > KINDA_SMALL_NUMBER)
						{
							float DistanceToPlane = FVector::DotProduct((SecondPoint - Start), PlaneNormal) / DotProduct;
							FVector IntersectionPoint = Start + DistanceToPlane * WorldDirection;

							// 绘制从第二个点到交点的垂直辅助线
							FVector _VerticalLocation = FVector(SecondPoint.X, SecondPoint.Y, IntersectionPoint.Z); // 当前垂直位置
							DrawDebugLine(GetWorld(), SecondPoint, _VerticalLocation, FColor::Red, false, -1.0f, 0, 2.0f);
						}
					}
					else {
						FVector SecondCenter = FVector(CenterPoint.X, CenterPoint.Y, ThirdPoint.Z);
						DrawDebugLine(GetWorld(), SecondPoint, ThirdPoint, FColor::Red, bIfShowDebugAreas, 0.1f, 0, 10.0f);
						DrawDebugCircle(GetWorld(), SecondCenter, Radius, 50, FColor::Blue, bIfShowDebugAreas, 0.1f, 0, 10.0f, FVector(1, 0, 0), FVector(0, 1, 0), false);
					}
				}
			}
		}
	}
}


void ADroneActor1::ShowPathPoints()
{

	if ( GlobalPathPoints.Num() > 0&& bIsShowGlobalPath)
	{
		// 清除 SplineComponent 的现有控制点
		SplineComponent->ClearSplinePoints();

		for (int32 i = 0; i < GlobalPathPoints.Num(); ++i)
		{
			const FPathPointWithOrientation& PathPoint = GlobalPathPoints[i];
			// 绘制航点位置的小圆球
			DrawDebugSphere(GetWorld(), PathPoint.Point, 50.0f, 8, FColor::Blue, true, 0.1f, 0, 2.0f);

			// 绘制相机朝向的线
			FVector OrientationEnd = PathPoint.Point + PathPoint.Orientation.Vector() * 50.0f;
			DrawDebugLine(GetWorld(), PathPoint.Point, OrientationEnd, FColor::Red, true, 0.1f, 0, 2.0f);
			
			// 绘制航线（连接当前点和下一个点）
			if (i < GlobalPathPoints.Num() - 1)
			{
				const FPathPointWithOrientation& NextPathPoint = GlobalPathPoints[i + 1];
				DrawDebugLine(GetWorld(), PathPoint.Point, NextPathPoint.Point, FColor::Green, true, 0.1f, 0, 10.0f);
			}

			//SplineComponent->AddSplinePoint(PathPoint.Point, ESplineCoordinateSpace::World);
		}
		
		//// 更新 SplineComponent 的曲线
		//SplineComponent->UpdateSpline();	

		//// 清除现有的 SplineMeshComponents
		//for (USplineMeshComponent* SplineMesh : SplineMeshComponents)
		//{
		//	SplineMesh->DestroyComponent();
		//}
		//SplineMeshComponents.Empty();

		//// 创建新的 SplineMeshComponents
		//const int32 NumSegments = SplineComponent->GetNumberOfSplinePoints() - 1;
		//for (int32 i = 0; i < NumSegments; ++i)
		//{
		//	USplineMeshComponent* SplineMesh = NewObject<USplineMeshComponent>(this);

		//	UStaticMesh* CylinderMesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
		//	if (CylinderMesh)
		//	{
		//		SplineMesh->SetStaticMesh(CylinderMesh);
		//	}

		//	SplineMesh->SetForwardAxis(ESplineMeshAxis::X); // 设置网格的前向轴

		//	FVector StartPoint, StartTangent, EndPoint, EndTangent;
		//	SplineComponent->GetLocationAndTangentAtSplinePoint(i, StartPoint, StartTangent, ESplineCoordinateSpace::World);
		//	SplineComponent->GetLocationAndTangentAtSplinePoint(i + 1, EndPoint, EndTangent, ESplineCoordinateSpace::World);

		//	SplineMesh->SetStartAndEnd(StartPoint, StartTangent, EndPoint, EndTangent);
		//	SplineMesh->AttachToComponent(SplineComponent, FAttachmentTransformRules::KeepRelativeTransform);
		//	SplineMesh->RegisterComponent();

		//	SplineMeshComponents.Add(SplineMesh);
		//}
	}

	if (!GlobalBestSplinePoints.IsEmpty() && bIsShowDebugPoints)
	{
		for (const FPathPointWithOrientation& PathPoint : GlobalBestSplinePoints)
		{
			// 绘制航点位置的小圆球
			DrawDebugSphere(GetWorld(), PathPoint.Point, 10.0f, 12, FColor::Magenta, true, 0.1f, 0, 2.0f);
		}

		for (const FPathPointWithOrientation& PathPoint : GlobalBestKeyPoints)
		{
			// 绘制航点位置的小圆球
			DrawDebugSphere(GetWorld(), PathPoint.Point, 50.0f, 12, FColor::Green, true, 0.1f, 0, 2.0f);
			// 绘制相机朝向的线
			FVector OrientationEnd = PathPoint.Point + PathPoint.Orientation.Vector() * 50.0f;
			DrawDebugLine(GetWorld(), PathPoint.Point, OrientationEnd, FColor::Red, true, 0.1f, 0, 5.0f);

			// 启用调试信息
			GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Green, FString::Printf(TEXT("Key Point Aesthetic Score: %f"), PathPoint.AestheticScore));
			UE_LOG(LogTemp, Warning, TEXT("Key Point Aesthetic Score: %f"), PathPoint.AestheticScore);
		}
	}

	if (!TestPathPoints.IsEmpty() && bIsShowLocalOriginalPath)
	{
		for (const FPathPointWithOrientation& PathPoint : TestPathPoints)
		{
			// 绘制航点位置的小圆球
			DrawDebugSphere(GetWorld(), PathPoint.Point, 50.0f, 12, FColor::Green, true, 0.1f, 0, 2.0f);
			// 绘制相机朝向的线
			FVector OrientationEnd = PathPoint.Point + PathPoint.Orientation.Vector() * 50.0f;
			DrawDebugLine(GetWorld(), PathPoint.Point, OrientationEnd, FColor::Red, true, 0.1f, 0, 5.0f);
		}
	}
}

void ADroneActor1::DestroyPathPoints() {

	SplineComponent->ClearSplinePoints();
	SplineComponent->UpdateSpline();
	FlushPersistentDebugLines(GetWorld());
}


// Called every frame
void ADroneActor1::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (PlayerController != nullptr)
	{
		 // 匹配 FOV
		float MainCameraFOV = PlayerController->PlayerCameraManager->GetFOVAngle();
		SceneCaptureComponent->FOVAngle = MainCameraFOV;

		DrawDebugHelpers(true); // 绘制实时辅助线和圆形半径 控制是否持续显示

	}
	else {
		UE_LOG(LogTemp, Warning, TEXT("PlayerController not found."));
	}

	// 只有在 bShouldDrawPathPoints 为 true 时才绘制航点和相机朝向
	if (bShouldDrawPathPoints && GlobalPathPoints.Num() > 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("Drawing Path Points..."));
		ShowPathPoints();
		bShouldDrawPathPoints = false;
	}


	// 飞行逻辑
	if (bIsFlyingAlongPath && currentIndex < GlobalPathPoints.Num())
	{
		// 目标点信息
		const FPathPointWithOrientation& TargetPathPoint = GlobalPathPoints[currentIndex];
		FVector CurrentLocation = GetActorLocation();
		FVector TargetLocation = TargetPathPoint.Point;

		// 计算每帧移动距离
		float DistancePerFrame = CurrentSpeed * DeltaTime;
		float ReachThreshold = DistancePerFrame * 0.5f;

		//--------------------------------------------------------------------------
		// 1）平滑旋转
		//--------------------------------------------------------------------------
		CurrentRotationTime += DeltaTime;
		float Alpha = FMath::Clamp(CurrentRotationTime / TotalRotationDuration, 0.0f, 1.0f);

		

		//--------------------------------------------------------------------------
		// 2）平滑 FOV
		//--------------------------------------------------------------------------
		float StartFOV = OriginalPathPoint.FOV;
		float TargetFOV = TargetPathPoint.FOV;
		float NewFOV = FMath::Lerp(StartFOV, TargetFOV, Alpha);
		CameraComponent->SetFieldOfView(NewFOV);

		//------------------------------------------------------------------
		// 3) 移动
		//------------------------------------------------------------------
		FVector Direction = (TargetLocation - CurrentLocation).GetSafeNormal();
		float DistanceRemaining = FVector::Dist(CurrentLocation, TargetLocation);

		bool bRotationFinished = true;
		if (bIsSlowMove) {
			bRotationFinished = (abs(CurrentRotationTime - TotalRotationDuration) <= 0.01f) ? true : false;	// 仅当慢速移动的时候考虑这一项
		}

		if (DistanceRemaining <= ReachThreshold&& bRotationFinished)
		{
			// 直接设置为目标点
			SetActorLocation(TargetLocation);
			SetActorRotation(TargetPathPoint.Orientation);
			CameraComponent->SetFieldOfView(TargetPathPoint.FOV);

			// OnCaptureScreenshotWithUI();	// 如果需要截图
			// 切换到下一个目标点
			if (currentIndex + 1 < GlobalPathPoints.Num())
			{
				currentIndex += 1;
				StartNewPathPoint();
			}
			else
			{
				bIsFlyingAlongPath = false;
				GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Green, TEXT("Reached final path point."));
				//bShouldDrawPathPoints = true;
			}
		}
		else
		{
			// 正常移动
			FQuat StartQuat = StartRotation.Quaternion();
			FQuat TargetQuat = TargetPathPoint.Orientation.Quaternion();
			FQuat NewQuat = FQuat::Slerp(StartQuat, TargetQuat, Alpha);

			SetActorRotation(NewQuat);
			if (PlayerController != nullptr)
			{
				PlayerController->SetControlRotation(NewQuat.Rotator());
			}

			FVector NewLocation = CurrentLocation + Direction * DistancePerFrame;
			SetActorLocation(NewLocation);
		}
	}


	/*UE_LOG(LogTemp, Warning, TEXT("IsTracking: %s"), YoloTracker->IsTracking() ? TEXT("True") : TEXT("False"));
	UE_LOG(LogTemp, Warning, TEXT("bIsInitialized: %s"), YoloTracker->bIsInitialized ? TEXT("True") : TEXT("False"));*/
	// 进行连续追踪

	// 累积时间
	TimeSinceLastInference += DeltaTime;

	if (TimeSinceLastInference >= InferenceInterval)
	{
		TimeSinceLastInference = 0.0f; // 重置累积时间

		if ((!YoloTracker->IsInferencing()) && YoloTracker->bIsInitialized && bIsTrackingActive && (!YoloTracker->ShouldStopTracking()))
		{
			// 创建或重用 RenderTarget
			if (!RenderTarget)
			{
				RenderTarget = NewObject<UTextureRenderTarget2D>(this, TEXT("ScreenShot"));
				RenderTarget->InitAutoFormat(ViewportWidth, ViewportHeight);
				RenderTarget->UpdateResourceImmediate(true);
				SceneCaptureComponent->TextureTarget = RenderTarget;

				//// 配置 SceneCaptureComponent 以确保捕捉正确的渲染内容
				SceneCaptureComponent->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;

				//// 调整后处理设置，避免过度曝光
				//SceneCaptureComponent->PostProcessSettings.bOverride_AutoExposureMethod = true;
				//SceneCaptureComponent->PostProcessSettings.AutoExposureMethod = EAutoExposureMethod::AEM_Manual;
				//SceneCaptureComponent->PostProcessSettings.AutoExposureBias = 0.0f; // 根据需要调整

				//// 设置 RenderTarget 的清除颜色为黑色，避免背景白色
				//RenderTarget->ClearColor = FLinearColor::Black;

				//SceneCaptureComponent->PostProcessSettings.bOverride_BloomIntensity = true;
				//SceneCaptureComponent->PostProcessSettings.BloomIntensity = 1.0f; // 根据需要调整

				UE_LOG(LogTemp, Log, TEXT("RenderTarget initialized with size: %dx%d"), ViewportWidth, ViewportHeight);
			}

			SceneCaptureComponent->CaptureScene();
			LastTrackedPosition = YoloTracker->GetTrackedPosition();

			if (RenderTarget)
			{
				YoloTracker->RunInference(RenderTarget, LastTrackedPosition);
			}

			if (!YoloTracker->IsTracking())
			{
				YoloTracker->StopTracking();
				bIsTrackingActive = false;
				UE_LOG(LogTemp, Warning, TEXT("Track Lost."));
			}
		}
	}

}

void ADroneActor1::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);

	PlayerInputComponent->BindAction("LeftMouseClick", IE_Pressed, this, &ADroneActor1::OnLeftMouseClick);

	PlayerInputComponent->BindAction("Track_Yolo", IE_Pressed, this, &ADroneActor1::OnStartTracking);

	PlayerInputComponent->BindAction("StartGeneratePath", IE_Pressed, this, &ADroneActor1::OnGenerateOrbitFlightPath);
	PlayerInputComponent->BindAction("Inference_Relic", IE_Pressed, this, &ADroneActor1::OnPredictAction);

	// 绑定读取路径文件按键
	PlayerInputComponent->BindAction("ReadPathFromFile", IE_Pressed, this, &ADroneActor1::OnReadPathFromFile);

	// 1) 创建一个 FInputActionBinding，指定 ActionName 和 EInputEvent
	FInputActionBinding CaptureWithUIBinding("CaptureWithUI", IE_Pressed);

	// 2) 给它的 ActionDelegate 绑定 Lambda
	CaptureWithUIBinding.ActionDelegate.GetDelegateForManualSet().BindLambda([this]()
		{
			// 在这里传参数
			OnCaptureScreenshotWithUI(true);
		});

	// 3) 把这个 Binding 加进 PlayerInputComponent
	PlayerInputComponent->AddActionBinding(CaptureWithUIBinding);

	// 测试候选视角功能
	PlayerInputComponent->BindAction("ViewPointsTest", IE_Pressed, this, &ADroneActor1::OnTestGenerateCandidateViewpoints);

	// 绑定飞行启动按键，例如按下 "F" 键开始飞行
	PlayerInputComponent->BindAction("StartFlightAlongPath", IE_Pressed, this, &ADroneActor1::OnStartFlightAlongPath);

	// 绑定鼠标右键事件
	PlayerInputComponent->BindAction("RightMouse", IE_Pressed, this, &ADroneActor1::OnRightMousePressed);
	PlayerInputComponent->BindAction("RightMouse", IE_Released, this, &ADroneActor1::OnRightMouseReleased);
	PlayerInputComponent->BindAxis("MoveForward", this, &ADroneActor1::MoveForward);
	PlayerInputComponent->BindAxis("MoveRight", this, &ADroneActor1::MoveRight);
	PlayerInputComponent->BindAxis("MoveUp", this, &ADroneActor1::MoveUp);

	// 绑定鼠标移动事件
	PlayerInputComponent->BindAxis("Turn", this, &ADroneActor1::Turn);
	PlayerInputComponent->BindAxis("LookUp", this, &ADroneActor1::LookUp);

	// 绑定鼠标滚轮调整飞行速度
	PlayerInputComponent->BindAxis("AdjustFlySpeed", this, &ADroneActor1::AdjustFlySpeed);
}

void ADroneActor1::OnMouseMove(float DeltaX, float DeltaY)
{
	if (Controller != nullptr) {
		// 添加鼠标输入的视角旋转逻辑
		AddControllerYawInput(DeltaX * MouseSensitivity);
		AddControllerPitchInput(DeltaY * MouseSensitivity);
	}
}


void ADroneActor1::OnRightMousePressed()
{
	bIsRightMousePressed = true;
	PressedTime = GetWorld()->GetTimeSeconds(); // 记录按下时间

	bUseControllerRotationYaw = true;
	bUseControllerRotationPitch = true;
	//APlayerController* PlayerController = Cast<APlayerController>(GetController());
	if (PlayerController)
	{
		PlayerController->bShowMouseCursor = false; // 或 true, 根据需求切换
		PlayerController->SetInputMode(FInputModeGameOnly()); // 切换为仅游戏模式
	}
}

void ADroneActor1::OnRightMouseReleased()
{
	bIsRightMousePressed = false;

	float ReleasedTime = GetWorld()->GetTimeSeconds(); // 记录释放时间
	float PressDuration = ReleasedTime - PressedTime;  // 计算按下持续时间

	// 判断长按或短按
	const float LongPressThreshold = 0.5f; // 长按阈值，单位为秒，可调整

	if (PressDuration <= LongPressThreshold)
	{
		CurrentState = EInterestPointState::Idle; // 重置状态
		if (InterestPoints.Num() == 0) {
			FlushPersistentDebugLines(GetWorld());
		}	
	}

	bUseControllerRotationYaw = false;
	bUseControllerRotationPitch = false;

	//APlayerController* PlayerController = Cast<APlayerController>(GetController());
	if (PlayerController)
	{
		PlayerController->bShowMouseCursor = true;
		PlayerController->SetInputMode(FInputModeGameAndUI());
	}
}

void ADroneActor1::MoveForward(float Value)
{
	if (Value != 0.0f) {
		if (bIsRightMousePressed)
		{
			//UE_LOG(LogTemp, Warning, TEXT("MoveForward"));
			//AddMovementInput(CameraComponent->GetForwardVector(), Value * SpeedMultiplier);
			FVector NewLocation = GetActorLocation() + CameraComponent->GetForwardVector() * Value * SpeedMultiplier;
			SetActorLocation(NewLocation, true);
		}

	}

}

void ADroneActor1::MoveRight(float Value)
{
	if (Value != 0.0f)
	{
		if (bIsRightMousePressed)
		{
			//AddMovementInput(CameraComponent->GetRightVector(), Value * SpeedMultiplier);
			FVector NewLocation = GetActorLocation() + CameraComponent->GetRightVector() * Value * SpeedMultiplier;
			SetActorLocation(NewLocation, true);
		}
	}
}

void ADroneActor1::MoveUp(float Value)
{
	if (Value != 0.0f)
	{
		if (bIsRightMousePressed)
		{
			//AddMovementInput(FVector::UpVector, Value* SpeedMultiplier);
			//FloatingMovement->AddInputVector(FVector::UpVector * Value * SpeedMultiplier );
			FVector NewLocation = GetActorLocation() + FVector::UpVector * Value * SpeedMultiplier;
			SetActorLocation(NewLocation, true);
		}
	}
}

void ADroneActor1::Turn(float Value)
{
	if (Value != 0.0f)
	{
		if (bIsRightMousePressed)
		{
			AddControllerYawInput(Value);
		}
	}
}

void ADroneActor1::LookUp(float Value)
{
	if (Value != 0.0f)
	{
		if (bIsRightMousePressed)
		{
			AddControllerPitchInput(-Value);
		}
	}
}


void ADroneActor1::AdjustFlySpeed(float AxisValue)
{
	if (AxisValue != 0.0f)
	{
		if (bIsRightMousePressed)
		{
			// 调整 SpeedMultiplier，并限制其范围
			SpeedMultiplier = FMath::Clamp(SpeedMultiplier + AxisValue * 1.0f, 0.1f, 5000.0f);
			UE_LOG(LogTemp, Log, TEXT("Current SpeedMultiplier: %f"), SpeedMultiplier);
		}	
	}
}


void ADroneActor1::OnPredictAction()
{
	if (NimaTracker) {
		UE_LOG(LogTemp, Warning, TEXT("Predict action using Relic triggered."));

		FPathPointWithOrientation ViewPoint = { GetActorLocation(),GetActorRotation(),90.0f };
		Force3DTilesLoad(); // 强制加载 3D Tiles

		NimaTracker->IfSaveImage = true;
		// 在游戏线程上调用渲染函数
		RenderViewpointToRenderTarget(ViewPoint);

		// 在游戏线程上运行推理任务
		if (NimaTracker && RenderTarget)
		{
			UE_LOG(LogTemp, Warning, TEXT("Running inference on the game thread."));
			NimaTracker->RunInference(RenderTarget);
		}

		// 等待异步推理完成,直到获取到有效的美学评分
		while (NimaTracker->GetNimaScore() <= 0)
		{
			FPlatformProcess::Sleep(0.001f);
		}

		ViewPoint.AestheticScore = NimaTracker->GetNimaScore();
		SingleScore = ViewPoint.AestheticScore;
		NimaTracker->ResetNimaScore(); // 重置美学评分


		// 等待异步推理完成,直到获取到有效的美学评分
		while (ViewPoint.AestheticScore <= 0)
		{
			FPlatformProcess::Sleep(0.001f);
		}
		NimaTracker->IfSaveImage = false;
		DisableForce3DTilesLoad();

		if (SingleScore != 0) {
			UE_LOG(LogTemp, Warning, TEXT("Inference successful with score of %f"), SingleScore);
			//Score = NimaTracker->GetNimaScore();
		}
	}
	// 当 "PredictAction" 动作被触发时，执行模型加载操作
	//UE_LOG(LogTemp, Warning, TEXT("Model loaded ongoing"));
	//try {
	//    


	//    // 设置 ONNX 模型路径和输入输出节点名称
	//    //FString ModelPath = FPaths::Combine(*FPaths::ProjectDir(), TEXT("Resources"), TEXT("pretrain_model/path_to_save_model.onnx"));

	//    const char* input_node_name = "input_image";
	//    const char* output_node_name = "output_scores";

	//    // 进行预测并获取输出结果
	//    UE_LOG(LogTemp, Warning, TEXT("Ready to load model and predict"));
	//    //std::vector<float> output_values = ONNXPredict(input_tensor_values, model_path, input_node_name, output_node_name);
	//    std::vector<float> output_values=ONNXPredict(*currentONNXModel, input_tensor_values, input_node_name, output_node_name);

	//    float score = GetScore(output_values);
	//    // 打印所有值
	//    for (size_t i = 0; i < output_values.size(); ++i) {
	//        UE_LOG(LogTemp, Warning, TEXT("Output value %d: %f"), i, output_values[i]);
	//    }
	//    UE_LOG(LogTemp, Warning, TEXT("Score: %f"), score);

	//}
	//catch (const c10::Error& e) {
	//    UE_LOG(LogTemp, Warning, TEXT("模型加载失败: %s"), *FString(e.msg().c_str()));
	//}
	//catch (const Ort::Exception& e) {
	//    // 处理 OnnxRuntimeException 异常
	//    UE_LOG(LogTemp, Warning, TEXT("模型加载失败: %s"), *FString(e.what()));
	//    std::cout << "Error adding CUDA execution provider: " << e.what() << std::endl;
	//    // 在此处进行错误处理,例如记录日志、显示错误消息或采取其他恰当的操作
	//    // ...
	//}
}

void ADroneActor1::GenerateSimpleFlightPath()
{
	GlobalPathPoints.Empty(); // 清空路径点

	const float ExtraHeightAboveCylinder = 5.0f; // 额外高度，例如5米，确保拍到屋顶
	const int NumHeightSteps = 10; // 高度的分段数，可以根据需要调整

	for (const FCylindricalInterestPoint& InterestPoint : InterestPoints)
	{
		// 动态设置最低、最高高度和高度增量
		float _MinHeight = InterestPoint.BottomCenter.Z + InterestPoint.MinSafetyDistance; // 保持安全距离
		float _MaxHeight = InterestPoint.BottomCenter.Z + InterestPoint.Height + ExtraHeightAboveCylinder; // 超出兴趣点高度
		float _HeightIncrement = FMath::Max((_MaxHeight - _MinHeight) / NumHeightSteps, 2.0f); // 动态增量，最小2米

		float CurrentHeight = _MinHeight;
		float CurrentRadius = InterestPoint.Radius;
		float Angle = 0.0f; // 初始角度

		FVector PathPointLocation;

		// 生成围绕兴趣点的路径
		while (CurrentHeight <= _MaxHeight)
		{
			// 使用角度计算圆周偏移，并逐渐增加角度
			FVector Offset = FVector(FMath::Cos(FMath::DegreesToRadians(Angle)) * CurrentRadius,
				FMath::Sin(FMath::DegreesToRadians(Angle)) * CurrentRadius,
				CurrentHeight);

			PathPointLocation = InterestPoint.Center + Offset;

			FPathPointWithOrientation PathPoint;
			PathPoint.Point = PathPointLocation;
			PathPoint.Orientation = FRotationMatrix::MakeFromX(InterestPoint.Center - PathPointLocation).Rotator(); // 朝向兴趣点中心
			PathPoint.FOV = 90.0f; // 默认FOV

			GlobalPathPoints.Add(PathPoint);

			// 更新高度、半径和角度
			CurrentHeight += _HeightIncrement;

			Angle += 30.0f; // 每次增加30度，可根据需要调整
		}

		// 添加返回点到中心
		FPathPointWithOrientation ReturnPoint;
		ReturnPoint.Point = InterestPoint.Center;
		ReturnPoint.Orientation = FRotationMatrix::MakeFromX(InterestPoint.Center - PathPointLocation).Rotator();
		ReturnPoint.FOV = 90.0f;

		GlobalPathPoints.Add(ReturnPoint);
	}

	GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Yellow, TEXT("Flight path generated"));
}


void ADroneActor1::GenerateOptimizedFlightPath(const FVector& _StartPoint, const FVector& _EndPoint)
{
	// 清空现有路径点
	GlobalPathPoints.Empty();

	// 配置参数，转为厘米单位
	const float ExtraHeightAboveCylinder = 500.0f;  // 圆柱体顶部额外高度（5米 = 500厘米）
	const float MinTransitionDistance = 1000.0f;    // 最小过渡距离（10米 = 1000厘米）
	const float HeightPerTurn = 800.0f;             // 每圈旋转时的高度增量

	if (InterestPoints.Num() == 0) return;

	FVector CurrentPosition = _StartPoint;

	// 添加起始点
	FPathPointWithOrientation StartPathPoint;
	StartPathPoint.Point = _StartPoint;
	StartPathPoint.FOV = 90.0f;
	StartPathPoint.Orientation = FRotationMatrix::MakeFromX(
		InterestPoints[0].Center - _StartPoint).Rotator();
	GlobalPathPoints.Add(StartPathPoint);

	// 遍历所有兴趣区域
	for (int32 i = 0; i < InterestPoints.Num(); ++i)
	{
		const FCylindricalInterestPoint& CurrentCylinder = InterestPoints[i];

		float CylinderBottom = CurrentCylinder.BottomCenter.Z;
		float CylinderTop = CylinderBottom + CurrentCylinder.Height + ExtraHeightAboveCylinder;
		float SafeRadius = CurrentCylinder.Radius + CurrentCylinder.MinSafetyDistance;

		int32 DynamicPointsPerCircle = FMath::Clamp(static_cast<int32>(12 * (SafeRadius / 1000.0f)), 8, 20);
		float AngleIncrementD = 360.0f / DynamicPointsPerCircle;

		FVector EntryPoint;
		FVector DirToCenter = CurrentCylinder.Center - CurrentPosition;
		DirToCenter.Z = 0;
		float DistanceToCenter = DirToCenter.Size();
		float EntryAngle = 0.0f;

		if (DistanceToCenter <= SafeRadius)
		{
			EntryPoint = CurrentPosition;
		}
		else
		{
			float TangentAngleR = FMath::Acos(SafeRadius / DistanceToCenter);
			FVector DirToCenterNorm = DirToCenter.GetSafeNormal();
			FRotator Rotation(0.0f, FMath::RadiansToDegrees(TangentAngleR), 0.0f); // 顺时针旋转
			FVector TangentDir = Rotation.RotateVector(DirToCenterNorm);

			//输出TangentDir
			GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Yellow,
				FString::Printf(TEXT("TangentDir: %s"), *TangentDir.ToString()));
			UE_LOG(LogTemp, Warning, TEXT("TangentDir: %s"), *TangentDir.ToString());
			EntryPoint = CurrentCylinder.Center - TangentDir * SafeRadius;
			EntryAngle = FMath::Atan2(-TangentDir.Y, -TangentDir.X); // Radians
			GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Yellow,
				FString::Printf(TEXT("Entry angle: %f"), FMath::RadiansToDegrees(EntryAngle)));
			UE_LOG(LogTemp, Warning, TEXT("Entry angle: %f"), FMath::RadiansToDegrees(EntryAngle));
		}
		EntryPoint.Z = CylinderBottom + CurrentCylinder.MinSafetyDistance;

		//DrawDebugLine(GetWorld(), EntryPoint, CurrentCylinder.Center, FColor::Red, true, -1.0f, 0, 2.0f);

		int32 TransitionSamples = FMath::Max(static_cast<int32>(DistanceToCenter / MinTransitionDistance), 5);
		for (int32 j = 1; j <= TransitionSamples; ++j)
		{
			float Alpha = static_cast<float>(j) / static_cast<float>(TransitionSamples);
			FVector InterpolatedPoint = FMath::Lerp(CurrentPosition, EntryPoint, Alpha);

			FPathPointWithOrientation TransitionPathPoint;
			TransitionPathPoint.Point = InterpolatedPoint;
			TransitionPathPoint.Orientation = FRotationMatrix::MakeFromX(
				CurrentCylinder.Center - InterpolatedPoint).Rotator();
			TransitionPathPoint.FOV = 90.0f;
			GlobalPathPoints.Add(TransitionPathPoint);
			//DrawDebugLine(GetWorld(), TransitionPathPoint.Point, CurrentCylinder.Center, FColor::Blue, true, -1.0f, 0, 2.0f);
		}

		float TotalHeight = CylinderTop - EntryPoint.Z;
		int32 NumTurns = FMath::CeilToInt(TotalHeight / HeightPerTurn);  // 动态计算旋转圈数
		float TotalAngle = 360.0f * NumTurns;                            // 总旋转角度
		float HeightIncrement = TotalHeight / TotalAngle;
		float CentertoDirAngle = FMath::Atan2(-DirToCenter.Y, -DirToCenter.X); // Radians
		UE_LOG(LogTemp, Warning, TEXT("CentertoDir: %f"), FMath::RadiansToDegrees(CentertoDirAngle));
		float CurrentAngle = FMath::RadiansToDegrees(EntryAngle);
		float CurrentHeight = EntryPoint.Z;

		while (CurrentHeight < CylinderTop)
		{
			CurrentAngle += AngleIncrementD; // 逆时针旋转
			CurrentHeight += HeightIncrement * AngleIncrementD;

			FVector PointLocation(
				CurrentCylinder.Center.X + SafeRadius * FMath::Cos(FMath::DegreesToRadians(CurrentAngle)),
				CurrentCylinder.Center.Y + SafeRadius * FMath::Sin(FMath::DegreesToRadians(CurrentAngle)),
				CurrentHeight
			);

			FPathPointWithOrientation PathPoint;
			PathPoint.Point = PointLocation;
			PathPoint.Orientation = FRotationMatrix::MakeFromX(
				CurrentCylinder.Center - PointLocation).Rotator();
			PathPoint.FOV = 90.0f;
			GlobalPathPoints.Add(PathPoint);

		}

		FVector LastPoint = GlobalPathPoints.Last().Point;
		//DrawDebugLine(GetWorld(), LastPoint, CurrentCylinder.Center, FColor::Orange, true, -1.0f, 0, 2.0f);

		float CurrentAngleRelativeToXR = FMath::Atan2(LastPoint.Y - CurrentCylinder.Center.Y, LastPoint.X - CurrentCylinder.Center.X); // 范围是-pi到pi
		UE_LOG(LogTemp, Warning, TEXT("CurrentAngleRelativeToXR: %f"), FMath::RadiansToDegrees(CurrentAngleRelativeToXR));

		// 计算到下一个兴趣点的共切线点
		if (i < InterestPoints.Num() - 1) // 如果有下一个兴趣点
		{
			const FCylindricalInterestPoint& NextCylinder = InterestPoints[i + 1];
			FVector DirToNextCylinder = NextCylinder.Center - CurrentCylinder.Center;
			DirToNextCylinder.Z = 0;
			float DistanceToNextCylinder = DirToNextCylinder.Size();
			float SafeRadiusNext = NextCylinder.Radius + NextCylinder.MinSafetyDistance;

			if (DistanceToNextCylinder > SafeRadius + SafeRadiusNext)
			{
				// 计算共切线的夹角
				float TangentAngleToNext = FMath::Acos((SafeRadius + SafeRadiusNext) / DistanceToNextCylinder); // > 0?
				FVector DirToNextNorm = DirToNextCylinder.GetSafeNormal();

				// 逆时针旋转到达共切线方向
				FRotator RotationToTangent(0.0f, -FMath::RadiansToDegrees(TangentAngleToNext), 0.0f);
				FVector TangentDirToNext = RotationToTangent.RotateVector(DirToNextNorm);

				FVector TangentPointNext = CurrentCylinder.Center + TangentDirToNext * SafeRadius;

				// 计算共切线点的角度
				float TargetAngleRelativeToX = FMath::Atan2(TangentPointNext.Y - CurrentCylinder.Center.Y, TangentPointNext.X - CurrentCylinder.Center.X);
				UE_LOG(LogTemp, Warning, TEXT("TargetAngleRelativeToX: %f"), FMath::RadiansToDegrees(TargetAngleRelativeToX));

				// 选择最佳的切点
				if (TargetAngleRelativeToX - CurrentAngleRelativeToXR < 0 && TargetAngleRelativeToX - CurrentAngleRelativeToXR + 2 * TangentAngleToNext>0) {
					TargetAngleRelativeToX += 2 * TangentAngleToNext;
				}

				// 计算需要旋转的额外角度
				float AdditionalRotationAngleD = FMath::RadiansToDegrees(TargetAngleRelativeToX - CurrentAngleRelativeToXR);
				if (AdditionalRotationAngleD < 0) {
					AdditionalRotationAngleD += 360.0f;
				}

				UE_LOG(LogTemp, Warning, TEXT("AdditionalRotationAngleD: %f"), AdditionalRotationAngleD);

				// 生成额外旋转路径，从当前角度旋转到共切线角度
				float ExtraHeightPerDegree = HeightPerTurn / 360.0f;
				while (AdditionalRotationAngleD > 0)
				{
					UE_LOG(LogTemp, Warning, TEXT("AngleIncrementD: %f"), AngleIncrementD);

					if (AdditionalRotationAngleD < AngleIncrementD)
					{
						CurrentAngleRelativeToXR += FMath::DegreesToRadians(AdditionalRotationAngleD);
					}
					else {
						CurrentAngleRelativeToXR += FMath::DegreesToRadians(AngleIncrementD);
					}
					UE_LOG(LogTemp, Warning, TEXT("CurrentAngleRelativeToXR: %f"), CurrentAngleRelativeToXR);

					LastPoint.Z += ExtraHeightPerDegree * AngleIncrementD;

					FVector ExtraPointLocation(
						CurrentCylinder.Center.X + SafeRadius * FMath::Cos(CurrentAngleRelativeToXR),
						CurrentCylinder.Center.Y + SafeRadius * FMath::Sin(CurrentAngleRelativeToXR),
						LastPoint.Z
					);

					FPathPointWithOrientation ExtraPathPoint;
					ExtraPathPoint.Point = ExtraPointLocation;
					ExtraPathPoint.Orientation = FRotationMatrix::MakeFromX(
						CurrentCylinder.Center - ExtraPointLocation).Rotator();
					ExtraPathPoint.FOV = 90.0f;
					GlobalPathPoints.Add(ExtraPathPoint);
					//DrawDebugLine(GetWorld(), ExtraPathPoint.Point, CurrentCylinder.Center, FColor::Green, true, -1.0f, 0, 2.0f);
					if (AdditionalRotationAngleD < AngleIncrementD)
					{
						AdditionalRotationAngleD -= AdditionalRotationAngleD;

					}
					else {
						AdditionalRotationAngleD -= FMath::Abs(AngleIncrementD); // 减去已旋转角度
					}
				}

				CurrentPosition = GlobalPathPoints.Last().Point; // 更新为共切线点位置
			}
			else
			{
				CurrentPosition = LastPoint; // 如果没有足够距离，只能直接到下一个兴趣点
			}
		}
		else { // 如果目标是终点，也类似的前进到切点 考虑到都使用顺时针旋转，切点选择从终点看来的右手侧

			if (DistanceToCenter <= SafeRadius) // 若小于安全距离
			{
				CurrentPosition = LastPoint;
			}
			else
			{
				FVector DirToEndPoint = _EndPoint - CurrentCylinder.Center;

				float DistanceToEnd = DirToEndPoint.Size();
				/*float SafeRadius = CurrentCylinder.Radius + CurrentCylinder.MinSafetyDistance;*/

				float TangentAngleR = FMath::Acos(SafeRadius / DistanceToEnd);
				FVector DirToEnd = DirToEndPoint.GetSafeNormal();
				FRotator Rotation(0.0f, -FMath::RadiansToDegrees(TangentAngleR), 0.0f); // 顺时针旋转
				FVector TangentDir = Rotation.RotateVector(DirToEnd);

				FVector TangentPointNext = CurrentCylinder.Center + TangentDir * SafeRadius;

				// 计算共切线点的角度
				float TargetAngleRelativeToX = FMath::Atan2(TangentPointNext.Y - CurrentCylinder.Center.Y, TangentPointNext.X - CurrentCylinder.Center.X);
				UE_LOG(LogTemp, Warning, TEXT("TargetAngleRelativeToX: %f"), FMath::RadiansToDegrees(TargetAngleRelativeToX));

				// 计算需要旋转的额外角度
				float AdditionalRotationAngleD = FMath::RadiansToDegrees(TargetAngleRelativeToX - CurrentAngleRelativeToXR);
				if (AdditionalRotationAngleD < 0) {
					AdditionalRotationAngleD += 360.0f;
				}

				// 生成额外旋转路径，从当前角度旋转到共切线角度
				float ExtraHeightPerDegree = HeightPerTurn / 360.0f;
				while (AdditionalRotationAngleD > 0)
				{
					UE_LOG(LogTemp, Warning, TEXT("AngleIncrementD: %f"), AngleIncrementD);
					if (AdditionalRotationAngleD < AngleIncrementD)
					{
						CurrentAngleRelativeToXR += FMath::DegreesToRadians(AdditionalRotationAngleD);
					}
					else {
						CurrentAngleRelativeToXR += FMath::DegreesToRadians(AngleIncrementD);
					}

					UE_LOG(LogTemp, Warning, TEXT("CurrentAngleRelativeToXR: %f"), CurrentAngleRelativeToXR);

					LastPoint.Z += ExtraHeightPerDegree * AngleIncrementD;

					FVector ExtraPointLocation(
						CurrentCylinder.Center.X + SafeRadius * FMath::Cos(CurrentAngleRelativeToXR),
						CurrentCylinder.Center.Y + SafeRadius * FMath::Sin(CurrentAngleRelativeToXR),
						LastPoint.Z
					);

					FPathPointWithOrientation ExtraPathPoint;
					ExtraPathPoint.Point = ExtraPointLocation;
					ExtraPathPoint.Orientation = FRotationMatrix::MakeFromX(
						CurrentCylinder.Center - ExtraPointLocation).Rotator();
					ExtraPathPoint.FOV = 90.0f;
					GlobalPathPoints.Add(ExtraPathPoint);
					// DrawDebugLine(GetWorld(), ExtraPathPoint.Point, CurrentCylinder.Center, FColor::Green, true, -1.0f, 0, 2.0f);
					if (AdditionalRotationAngleD < AngleIncrementD)
					{
						AdditionalRotationAngleD -= AdditionalRotationAngleD;

					}
					else {
						AdditionalRotationAngleD -= FMath::Abs(AngleIncrementD); // 减去已旋转角度
					}

				}

				CurrentPosition = GlobalPathPoints.Last().Point; // 更新为共切线点位置
			}

		}

	}

	float DistanceToEnd = FVector::Dist(CurrentPosition, _EndPoint);
	int32 FinalTransitionSamples = FMath::Max(static_cast<int32>(DistanceToEnd / MinTransitionDistance), 5);
	for (int32 j = 1; j <= FinalTransitionSamples; ++j)
	{
		float Alpha = static_cast<float>(j) / static_cast<float>(FinalTransitionSamples);
		FVector InterpolatedPoint = FMath::Lerp(CurrentPosition, _EndPoint, Alpha);

		FPathPointWithOrientation TransitionPathPoint;
		TransitionPathPoint.Point = InterpolatedPoint;
		TransitionPathPoint.Orientation = FRotationMatrix::MakeFromX(
			InterestPoints.Last().Center - InterpolatedPoint).Rotator();
		TransitionPathPoint.FOV = 90.0f;
		GlobalPathPoints.Add(TransitionPathPoint);
	}

	GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Yellow,
		FString::Printf(TEXT("Generated path with %d points"), GlobalPathPoints.Num()));
}


void ADroneActor1::CalculateOrbitParameters(const FCylindricalInterestPoint& InterestPoint, 
	float& OutOrbitRadius, float& OutOrbitRadius1, float& OutOrbitRadius2,
	float& OutMinHeight, float& OutMaxHeight)
{
	// Camera FOV (in degrees)
	float FOV_h = CameraComponent->FieldOfView;
	float FOV_v = CameraComponent->FieldOfView / CameraComponent->AspectRatio;

	// Convert FOV to radians
	float FOV_h_rad = FMath::DegreesToRadians(FOV_h);
	float FOV_v_rad = FMath::DegreesToRadians(FOV_v);

	// Target dimensions
	float W = InterestPoint.Radius * 2; // Width (diameter)
	float H = InterestPoint.Height;     // Height

	// Target radius
	float r_c = InterestPoint.Radius;

	// Calculate D_h and D_v
	float D_h1 = (1 * W) / (2 * FMath::Tan(FOV_h_rad / 2));
	float D_v1 = (1 * H) / (2 * FMath::Tan(FOV_v_rad / 2));
	float D_h2 = (2 * W) / (2 * FMath::Tan(FOV_h_rad / 2));
	float D_v2 = (2 * H) / (2 * FMath::Tan(FOV_v_rad / 2));
	float D_h = (3 * W) / (2 * FMath::Tan(FOV_h_rad / 2));
	float D_v = (3 * H) / (2 * FMath::Tan(FOV_v_rad / 2));

	// Orbit radius D_C
	float D_C = r_c + FMath::Max(D_h, D_v);
	float D_C1 = r_c + FMath::Max(D_h1, D_v1);
	float D_C2 = r_c + FMath::Max(D_h2, D_v2);
	OutOrbitRadius = D_C;
	OutOrbitRadius1 = D_C1;
	OutOrbitRadius2 = D_C2;

	// Minimum and maximum heights
	float H_min = InterestPoint.BottomCenter.Z;                // Base height
	float H_max = H_min + InterestPoint.Height;                // Top height
	float h_min = H_min + (2.0f / 3.0f) * (D_C - r_c) * FMath::Tan(FOV_v_rad / 2);
	float h_max = (3 * r_c) / FMath::Tan(FOV_h_rad / 2) + H_max;

	OutMinHeight = h_min;
	OutMaxHeight = h_max;
}


// 修改构图代价计算
float ADroneActor1::CalculateCompositionCost(FCandidateViewpoint& Candidate)
{
	// 计算旋转差异
	float PitchDiff = FMath::Abs(Candidate.OriginalRotation.Pitch - Candidate.TargetRotation.Pitch);
	float YawDiff = FMath::Abs(Candidate.OriginalRotation.Yaw - Candidate.TargetRotation.Yaw);

	// 标准化角度差异 (0-180度)
	PitchDiff = FMath::Fmod(PitchDiff + 180.0f, 360.0f) - 180.0f;
	YawDiff = FMath::Fmod(YawDiff + 180.0f, 360.0f) - 180.0f;

	// 计算归一化代价 (0-1范围)
	float NormalizedCost = FMath::Sqrt(
		FMath::Square(PitchDiff / 180.0f) +
		FMath::Square(YawDiff / 180.0f)
	) / FMath::Sqrt(2.0f);

	return NormalizedCost;
}


// 辅助函数：投影世界坐标到屏幕
bool ADroneActor1::ProjectWorldPointToScreen(const FVector& WorldPoint, FVector2D& ScreenPoint,
	const FVector& CameraLocation, const FRotator& CameraRotation, float FOV)
{
	FMatrix ViewMatrix = FMatrix::Identity;
	FMatrix ProjectionMatrix = FMatrix::Identity;

	// 构建视图矩阵
	const FVector ForwardVector = CameraRotation.Vector();
	const FVector RightVector = CameraRotation.RotateVector(FVector::RightVector);
	const FVector UpVector = CameraRotation.RotateVector(FVector::UpVector);
	ViewMatrix = FLookAtMatrix(CameraLocation, CameraLocation + ForwardVector, UpVector);

	// 构建投影矩阵
	float AspectRatio = ViewportWidth / ViewportHeight;
	float HalfFOV = FMath::DegreesToRadians(FOV) / 2.0f;
	float HalfFOVTan = FMath::Tan(HalfFOV);
	ProjectionMatrix = FReversedZPerspectiveMatrix(HalfFOVTan, AspectRatio, 1.0f, 10000.0f);

	// 计算视图投影矩阵
	FMatrix ViewProjectionMatrix = ViewMatrix * ProjectionMatrix;

	// 将世界坐标转换为裁剪空间坐标
	FVector4 ClipSpacePosition = ViewProjectionMatrix.TransformPosition(WorldPoint);

	if (ClipSpacePosition.W > 0.0f)
	{
		// 执行透视除法
		float RHW = 1.0f / ClipSpacePosition.W;
		float NormalizedX = ClipSpacePosition.X * RHW;
		float NormalizedY = ClipSpacePosition.Y * RHW;

		// 转换为屏幕空间坐标 [-1, +1]
		ScreenPoint.X = NormalizedX;
		ScreenPoint.Y = NormalizedY;

		return true;
	}

	return false;
}


// 修改可见性代价计算
float ADroneActor1::CalculateVisibilityCost(FCandidateViewpoint& Candidate)
{
	// 获取目标在目标旋转下的边界框
	TArray<FVector> BoundingPoints;
	GetAOIBoundingPoints(InterestPoints[Candidate.AOIIndex], BoundingPoints);

	float TotalVisiblePoints = 0.0f;
	bool bIsFullyVisible = true;
	const float EdgeThreshold = 0.1f; // 边缘阈值(10%)

	for (const FVector& Point : BoundingPoints)
	{
		FVector2D ScreenPosition;
		// 使用目标旋转进行投影
		if (ProjectWorldPointToScreen(Point, ScreenPosition, Candidate.Location,
			Candidate.TargetRotation, Candidate.FOV))
		{
			// 检查是否在安全区域内
			if (ScreenPosition.X < -1.0f + EdgeThreshold || ScreenPosition.X > 1.0f - EdgeThreshold ||
				ScreenPosition.Y < -1.0f + EdgeThreshold || ScreenPosition.Y > 1.0f - EdgeThreshold)
			{
				bIsFullyVisible = false;
			}
			else
			{
				TotalVisiblePoints += 1.0f;
			}
		}
		else
		{
			bIsFullyVisible = false;
		}
	}

	if (bIsFullyVisible)
	{
		return 0.0f;
	}

	// 计算可见性比率和FOV调整
	float VisibilityRatio = TotalVisiblePoints / BoundingPoints.Num();

	// 如果可见性太低，尝试调整FOV
	if (VisibilityRatio < 0.8f && Candidate.FOV < 120.0f)
	{
		float NewFOV = Candidate.FOV * (1.0f + (1.0f - VisibilityRatio));
		Candidate.FOV = FMath::Clamp(NewFOV, 60.0f, 120.0f);

		// 递归检查新FOV下的可见性
		return CalculateVisibilityCost(Candidate);
	}

	return 1.0f - VisibilityRatio;
}


float ADroneActor1::CalculateDynamicEdgeThreshold(const FCylindricalInterestPoint& InterestPoint)
{
	// Dynamic threshold calculation based on InterestPoint's size or camera properties
	// Example: The larger the interest area (e.g., radius or height), the larger the edge threshold
	float ScaleFactor = FMath::Clamp(InterestPoint.Radius / 1000.0f, 0.05f, 0.2f);  // Example scaling factor
	return 0.05f + ScaleFactor;  // Dynamic threshold between 0.05 and 0.2
}

void ADroneActor1::GetAOIBoundingPoints(const FCylindricalInterestPoint& InterestPoint, TArray<FVector>& OutPoints)
{
	// Define the top and bottom circle points
	int32 NumSegments = 8; // Number of points around the circle

	for (int32 i = 0; i < NumSegments; ++i)
	{
		float AngleDegrees = i * (360.0f / NumSegments);
		float AngleRadians = FMath::DegreesToRadians(AngleDegrees);

		// Bottom circle
		FVector BottomPoint;
		BottomPoint.X = InterestPoint.BottomCenter.X + InterestPoint.Radius * FMath::Cos(AngleRadians);
		BottomPoint.Y = InterestPoint.BottomCenter.Y + InterestPoint.Radius * FMath::Sin(AngleRadians);
		BottomPoint.Z = InterestPoint.BottomCenter.Z;

		// Top circle
		FVector TopPoint = BottomPoint;
		TopPoint.Z += InterestPoint.Height;

		OutPoints.Add(BottomPoint);
		OutPoints.Add(TopPoint);
	}
}


bool ADroneActor1::IsObjectVisible(const FVector& ViewpointLocation, const FRotator& CameraRotation, const FCylindricalInterestPoint& InterestPoint)
{
	FVector ForwardVector = CameraRotation.Vector();  // 相机朝向的前方向量
	FVector ToInterestPoint = InterestPoint.Center - ViewpointLocation;  // 从视点到兴趣点的向量

	// 计算相机视锥体与兴趣区域的相交部分
	float DistanceToInterestPoint = ToInterestPoint.Size();
	float AngleToInterestPoint = FMath::Acos(FVector::DotProduct(ForwardVector, ToInterestPoint) / DistanceToInterestPoint);

	// 判断兴趣区域是否在相机的视锥体内
	// 假设视角为60度（可根据实际需求调整）
	if (AngleToInterestPoint < FMath::DegreesToRadians(30.0f))
	{
		// 检查兴趣区域是否完全或部分可见
		float DistanceToInterestArea = FVector::Dist(ViewpointLocation, InterestPoint.Center);
		if (DistanceToInterestArea < InterestPoint.Radius + InterestPoint.MinSafetyDistance)
		{
			return true; // 可见
		}
	}

	return false; // 不可见
}

// -----------------------------------------------------------------------------------------------------------------
// 动态计算所有速度
float ADroneActor1::CalculateTurnAngle(const FVector& PrevPoint, const FVector& CurrentPoint, const FVector& NextPoint)
{
	FVector v1 = (CurrentPoint - PrevPoint).GetSafeNormal();
	FVector v2 = (NextPoint - CurrentPoint).GetSafeNormal();

	float DotVal = FVector::DotProduct(v1, v2);
	DotVal = FMath::Clamp(DotVal, -1.0f, 1.0f);
	float Radians = FMath::Acos(DotVal);
	return FMath::RadiansToDegrees(Radians);
}

float ADroneActor1::CalculateCumulativeTurnAngle(int32 CurrentIndex, int32 LookaheadPoints)
{
	const int32 NumPts = GlobalPathPoints.Num();
	if (CurrentIndex < 0 || CurrentIndex >= NumPts || LookaheadPoints < 1)
	{
		return 0.0f; // 无法计算转角
	}

	float CumulativeTurnAngle = 0.0f;

	// 遍历前面的路径点
	for (int32 i = 1; i <= LookaheadPoints && CurrentIndex - i >= 0; ++i)
	{
		FVector PrevPoint = GlobalPathPoints[CurrentIndex - i].Point;
		FVector CurrentPoint = GlobalPathPoints[CurrentIndex - i + 1].Point;
		FVector NextPoint = GlobalPathPoints[CurrentIndex - i + 2].Point;

		// 计算当前段的转角并累加
		float TurnAngle = CalculateTurnAngle(PrevPoint, CurrentPoint, NextPoint);

		// 衰减权重（距离越远，权重越低）
		float Weight = 1.0f / (i + 1); // 简单的反比权重
		CumulativeTurnAngle += TurnAngle * Weight;
	}

	// 遍历后面的路径点
	for (int32 i = 1; i <= LookaheadPoints && CurrentIndex + i + 1 < NumPts; ++i)
	{
		FVector PrevPoint = GlobalPathPoints[CurrentIndex + i - 1].Point;
		FVector CurrentPoint = GlobalPathPoints[CurrentIndex + i].Point;
		FVector NextPoint = GlobalPathPoints[CurrentIndex + i + 1].Point;

		// 计算当前段的转角并累加
		float TurnAngle = CalculateTurnAngle(PrevPoint, CurrentPoint, NextPoint);

		// 衰减权重（距离越远，权重越低）
		float Weight = 1.0f / (i + 1); // 简单的反比权重
		CumulativeTurnAngle += TurnAngle * Weight;
	}

	return CumulativeTurnAngle;
}

float ADroneActor1::CalculateViewChange(const FRotator& PrevOrientation, const FRotator& CurrentOrientation, 
	const FRotator& NextOrientation, float PrevFOV, float CurrentFOV, float NextFOV)
{
	// 计算方向变化（角度差）
	float YawChange = FMath::Abs(FMath::FindDeltaAngleDegrees(PrevOrientation.Yaw, CurrentOrientation.Yaw)) +
		FMath::Abs(FMath::FindDeltaAngleDegrees(CurrentOrientation.Yaw, NextOrientation.Yaw));
	float PitchChange = FMath::Abs(FMath::FindDeltaAngleDegrees(PrevOrientation.Pitch, CurrentOrientation.Pitch)) +
		FMath::Abs(FMath::FindDeltaAngleDegrees(CurrentOrientation.Pitch, NextOrientation.Pitch));

	// 计算 FOV 变化
	float FOVChange = FMath::Abs(CurrentFOV - PrevFOV) + FMath::Abs(NextFOV - CurrentFOV);

	// 综合视角变化
	return YawChange + PitchChange + FOVChange;
}

float ADroneActor1::CalculateCumulativeViewChange(int32 CurrentIndex, int32 LookaheadPoints)
{
	const int32 NumPts = GlobalPathPoints.Num();
	if (CurrentIndex < 0 || CurrentIndex >= NumPts || LookaheadPoints < 1)
	{
		return 0.0f; // 无法计算视角变化
	}

	float CumulativeViewChange = 0.0f;

	// 遍历前面的路径点
	for (int32 i = 1; i <= LookaheadPoints && CurrentIndex - i >= 0; ++i)
	{
		FRotator PrevOrientation = GlobalPathPoints[CurrentIndex - i].Orientation;
		FRotator CurrentOrientation = GlobalPathPoints[CurrentIndex - i + 1].Orientation;
		FRotator NextOrientation = GlobalPathPoints[CurrentIndex - i + 2].Orientation;

		float PrevFOV = GlobalPathPoints[CurrentIndex - i].FOV;
		float CurrentFOV = GlobalPathPoints[CurrentIndex - i + 1].FOV;
		float NextFOV = GlobalPathPoints[CurrentIndex - i + 2].FOV;

		// 计算当前段的视角变化并累加
		float ViewChange = CalculateViewChange(PrevOrientation, CurrentOrientation, NextOrientation, PrevFOV, CurrentFOV, NextFOV);

		// 衰减权重（距离越远，权重越低）
		float Weight = 1.0f / (i + 1); // 简单的反比权重
		CumulativeViewChange += ViewChange * Weight;
	}

	// 遍历后面的路径点
	for (int32 i = 1; i <= LookaheadPoints && CurrentIndex + i + 1 < NumPts; ++i)
	{
		FRotator PrevOrientation = GlobalPathPoints[CurrentIndex + i - 1].Orientation;
		FRotator CurrentOrientation = GlobalPathPoints[CurrentIndex + i].Orientation;
		FRotator NextOrientation = GlobalPathPoints[CurrentIndex + i + 1].Orientation;

		float PrevFOV = GlobalPathPoints[CurrentIndex + i - 1].FOV;
		float CurrentFOV = GlobalPathPoints[CurrentIndex + i].FOV;
		float NextFOV = GlobalPathPoints[CurrentIndex + i + 1].FOV;

		// 计算当前段的视角变化并累加
		float ViewChange = CalculateViewChange(PrevOrientation, CurrentOrientation, NextOrientation, PrevFOV, CurrentFOV, NextFOV);

		// 衰减权重（距离越远，权重越低）
		float Weight = 1.0f / (i + 1); // 简单的反比权重
		CumulativeViewChange += ViewChange * Weight;
	}

	return CumulativeViewChange;
}

float ADroneActor1::CalculateCumulativeViewChange_BothSides(
	int32 CurrentIndex,
	int32 BackPointsToLook,
	int32 FrontPointsToLook)
{
	float CumulativeView = 0.0f;

	// 前面
	for (int32 offset = 1; offset <= BackPointsToLook; ++offset)
	{
		if (CurrentIndex - offset - 1 >= 0)
		{
			FRotator PrevOri = GlobalPathPoints[CurrentIndex - offset - 1].Orientation;
			FRotator CurrOri = GlobalPathPoints[CurrentIndex - offset].Orientation;
			FRotator NextOri = GlobalPathPoints[CurrentIndex - offset + 1].Orientation;

			float PrevFOV = GlobalPathPoints[CurrentIndex - offset - 1].FOV;
			float CurrFOV = GlobalPathPoints[CurrentIndex - offset].FOV;
			float NextFOV = GlobalPathPoints[CurrentIndex - offset + 1].FOV;

			float ViewChange = CalculateViewChange(PrevOri, CurrOri, NextOri, PrevFOV, CurrFOV, NextFOV);

			float Weight = 1.0f / (offset + 1);
			CumulativeView += ViewChange * Weight;
		}
	}

	// 后面
	for (int32 offset = 1; offset <= FrontPointsToLook; ++offset)
	{
		if (CurrentIndex + offset + 1 < GlobalPathPoints.Num())
		{
			FRotator PrevOri = GlobalPathPoints[CurrentIndex + offset - 1].Orientation;
			FRotator CurrOri = GlobalPathPoints[CurrentIndex + offset].Orientation;
			FRotator NextOri = GlobalPathPoints[CurrentIndex + offset + 1].Orientation;

			float PrevFOV = GlobalPathPoints[CurrentIndex + offset - 1].FOV;
			float CurrFOV = GlobalPathPoints[CurrentIndex + offset].FOV;
			float NextFOV = GlobalPathPoints[CurrentIndex + offset + 1].FOV;

			float ViewChange = CalculateViewChange(PrevOri, CurrOri, NextOri, PrevFOV, CurrFOV, NextFOV);

			float Weight = 1.0f / (offset + 1);
			CumulativeView += ViewChange * Weight;
		}
	}

	return CumulativeView;
}

float ADroneActor1::CalculateCumulativeTurnAngle_BothSides(
	int32 CurrentIndex,
	int32 BackPoints,
	int32 FrontPoints)
{
	float CumulativeAngle = 0.f;

	// 前半部分
	for (int32 offset = 1; offset <= BackPoints; ++offset)
	{
		if (CurrentIndex - offset - 1 >= 0)
		{
			FVector Prev = GlobalPathPoints[CurrentIndex - offset - 1].Point;
			FVector Curr = GlobalPathPoints[CurrentIndex - offset].Point;
			FVector Next = GlobalPathPoints[CurrentIndex - offset + 1].Point;

			float TurnAngle = CalculateTurnAngle(Prev, Curr, Next);

			// 距离越远，权重越低
			float Weight = 1.0f / (offset + 1);

			// 如果角度接近180°，增加额外权重（例如乘以2）
			if (TurnAngle > 150.f)  // 可以调整阈值
			{
				Weight *= 2.0f;
			}

			CumulativeAngle += TurnAngle * Weight;
		}
	}

	// 后半部分处理类似
	for (int32 offset = 1; offset <= FrontPoints; ++offset)
	{
		if (CurrentIndex + offset + 1 < GlobalPathPoints.Num())
		{
			FVector Prev = GlobalPathPoints[CurrentIndex + offset - 1].Point;
			FVector Curr = GlobalPathPoints[CurrentIndex + offset].Point;
			FVector Next = GlobalPathPoints[CurrentIndex + offset + 1].Point;

			float TurnAngle = CalculateTurnAngle(Prev, Curr, Next);

			float Weight = 1.0f / (offset + 1);
			if (TurnAngle > 150.f)  // 调整阈值
			{
				Weight *= 2.0f;
			}

			CumulativeAngle += TurnAngle * Weight;
		}
	}

	return CumulativeAngle;
}


void ADroneActor1::SmoothSegmentSpeed(int32 Iterations)
{
	const int32 NumPts = GlobalPathPoints.Num();
	if (NumPts < 3)
	{
		return;
	}

	for (int32 iter = 0; iter < Iterations; ++iter)
	{
		// 创建临时数组存储平滑后的速度
		TArray<float> TempSpeed;
		TempSpeed.SetNum(NumPts);

		// 首尾速度可以保持不变（如果你想保持最大速度不变）
		TempSpeed[0] = GlobalPathPoints[0].SegmentSpeed;
		TempSpeed[NumPts - 1] = GlobalPathPoints[NumPts - 1].SegmentSpeed;

		// 对中间点进行平滑
		for (int32 i = 1; i < NumPts - 1; ++i)
		{
			float PrevSpd = GlobalPathPoints[i - 1].SegmentSpeed;
			float CurrSpd = GlobalPathPoints[i].SegmentSpeed;
			float NextSpd = GlobalPathPoints[i + 1].SegmentSpeed;

			// 简单的三点平均，也可以加权
			float Smoothed = (PrevSpd + CurrSpd + NextSpd) / 3.0f;

			TempSpeed[i] = Smoothed;
		}

		// 写回GlobalPathPoints
		for (int32 i = 0; i < NumPts; ++i)
		{
			GlobalPathPoints[i].SegmentSpeed = TempSpeed[i];
		}
	}
}

void ADroneActor1::ExportPathPointsToWGS84Txt()
{
	// 使用时间命名文件
	FString TimeString = FDateTime::Now().ToString();
	TimeString = TimeString.Replace(TEXT(":"), TEXT("-"));
	TimeString = TimeString.Replace(TEXT(" "), TEXT("_"));
	FString SaveFilePath = FPaths::ProjectDir() + TEXT("Path_") + TimeString + TEXT(".txt");

	if (!CesiumGeoreference)
	{
		UE_LOG(LogTemp, Warning, TEXT("CesiumGeoreference is null, cannot export WGS84 coordinates."));
		return;
	}
	if (GlobalPathPoints.Num() == 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("GlobalPathPoints is empty, nothing to export."));
		return;
	}

	TArray<FString> OutputLines;
	OutputLines.Reserve(GlobalPathPoints.Num());

	int IndexNum = 0;
	for (const FPathPointWithOrientation& PathPt : GlobalPathPoints)
	{
		//--------------------------------------------------------------
		// (1) 位置：Unreal -> (Longitude, Latitude, Height)
		//--------------------------------------------------------------
		FVector LonLatHeight =
			CesiumGeoreference->TransformUnrealPositionToLongitudeLatitudeHeight(PathPt.Point);

		double Longitude = LonLatHeight.X; // 经度(度)
		double Latitude = LonLatHeight.Y; // 纬度(度)
		double Altitude = LonLatHeight.Z; // 高度(米)

		//--------------------------------------------------------------
		// (2) UnrealRotator -> East-South-Up (ESU)
		//--------------------------------------------------------------
		FRotator RotESU = CesiumGeoreference->TransformUnrealRotatorToEastSouthUp(
			PathPt.Orientation,
			PathPt.Point
		);
		// RotESU: 在 ESU 坐标系下，Yaw=0=>East, Y=South

		//--------------------------------------------------------------
		// (3) ESU -> NEU (North-East-Up)
		//     NEU.X = -ESU.Y (North), NEU.Y = ESU.X (East), NEU.Z = ESU.Z (Up)
		//--------------------------------------------------------------
		FTransform TmpTransform(RotESU);
		FMatrix MatESU = TmpTransform.ToMatrixNoScale();

		//FVector ESU_X = MatESU.GetScaledAxis(EAxis::X); // East
		//FVector ESU_Y = MatESU.GetScaledAxis(EAxis::Y); // South
		//FVector ESU_Z = MatESU.GetScaledAxis(EAxis::Z); // Up

		//FVector NEU_X = -ESU_Y; // North
		//FVector NEU_Y = ESU_X; // East
		//FVector NEU_Z = ESU_Z; // Up

		//FMatrix MatNEU(
		//	FPlane(NEU_X.X, NEU_X.Y, NEU_X.Z, 0.f),
		//	FPlane(NEU_Y.X, NEU_Y.Y, NEU_Y.Z, 0.f),
		//	FPlane(NEU_Z.X, NEU_Z.Y, NEU_Z.Z, 0.f),
		//	FPlane(0.f, 0.f, 0.f, 1.f)
		//);

		//FRotator RotNEU = MatNEU.Rotator();
		FRotator SaledRotESU = MatESU.Rotator();
		// 在 NEU 坐标系：Yaw=0=>X=North, 逆时针增大

		//--------------------------------------------------------------
		// (4) 修正 Yaw => 0=北, 且顺时针增大
		//--------------------------------------------------------------
		float YawNEU = SaledRotESU.Yaw+90.0f;
		//float Heading = 360.f - YawNEU; // 转换逆时针=>顺时针
		YawNEU = FMath::Fmod(YawNEU, 360.f);
		if (YawNEU > 180) {
			YawNEU = YawNEU - 360;
		}
		/*if (YawNEU < 0.f)
		{
			YawNEU += 360.f;
		}*/

		// 若你对 Pitch、Roll 的符号和方向有特别要求，可在这里再行调整
		float PitchNEU = SaledRotESU.Pitch;
		float RollNEU = SaledRotESU.Roll;

		//--------------------------------------------------------------
		// (5) 构造输出行，示例： 
		// X=114.700551846925 Y=30.6755666252893 Z=92.793542918495092 
		// Pitch=-31.541243843168534 Yaw=-87.513293016852828 Roll=0.0000000000000000
		//--------------------------------------------------------------
		// 说明：
		//  - "%.12f" 表示保留12位小数，你可根据需求改成 "%.15f"、"%.9f" 等
		//  - Pitch/Yaw/Roll 可以输出更多小数位，如 "%.15f" 或 "%g"
		//  - 若想完全匹配示例，就可统一 %.15f
		FString Line = FString::Printf(
			TEXT("id=%d X=%.12f Y=%.12f Z=%.12f Pitch=%.15f Yaw=%.15f Roll=%.15f Speed=%.2f"),
			IndexNum,
			Longitude,
			Latitude,
			Altitude,
			PitchNEU,
			YawNEU,
			RollNEU,
			PathPt.SegmentSpeed/100
		);
		IndexNum++;

		OutputLines.Add(Line);
	}

	//--------------------------------------------------------------
	// (6) 写入 txt 文件
	//--------------------------------------------------------------
	if (FPaths::FileExists(SaveFilePath))
	{
		UE_LOG(LogTemp, Warning, TEXT("File %s already exists and will be overwritten."), *SaveFilePath);
	}

	bool bSuccess = FFileHelper::SaveStringArrayToFile(OutputLines, *SaveFilePath);
	if (bSuccess)
	{
		UE_LOG(LogTemp, Log, TEXT("Exported path points to %s"), *SaveFilePath);
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("Failed to export to %s"), *SaveFilePath);
	}
}

/// <summary>
/// 从指定的文本文件路径导入路径点。
/// </summary>
/// <param name="LoadFilePath">要加载的文件路径。</param>
/// <returns>如果成功加载并解析文件，则返回 true；否则返回 false。</returns>
bool ADroneActor1::ImportPathPointsFromTxt(const FString& LoadFilePath)
{
	// 1) 读取文件
	TArray<FString> FileLines;
	if (!FFileHelper::LoadFileToStringArray(FileLines, *LoadFilePath))
	{
		UE_LOG(LogTemp, Warning, TEXT("Failed to load file: %s"), *LoadFilePath);
		return false;
	}

	TArray<FPathPointWithOrientation> LoadedPoints;
	LoadedPoints.Reserve(FileLines.Num());

	// 2) 逐行解析
	for (const FString& Line : FileLines)
	{
		// 按空格拆分
		TArray<FString> Tokens;
		Line.ParseIntoArray(Tokens, TEXT(" "), true);

		if (Tokens.Num() < 8)
		{
			UE_LOG(LogTemp, Warning, TEXT("Line format invalid: %s"), *Line);
			continue;
		}

		// 小Lambda, 解析 "X=114.7005518" => 114.7005518
		auto ParseKeyValue = [&](const FString& InText) -> double
			{
				FString Left, Right;
				if (InText.Split(TEXT("="), &Left, &Right))
				{
					return FCString::Atod(*Right); // Atod返回double（UE5+可用）
				}
				return 0.0;
			};

		// 3) 读出 (Longitude,Latitude,Altitude, PitchNEU,YawNEU,RollNEU, Speed)
		int IndexNum = ParseKeyValue(Tokens[0]); // id=...
		double LonVal = ParseKeyValue(Tokens[1]); // X=...
		double LatVal = ParseKeyValue(Tokens[2]); // Y=...
		double AltVal = ParseKeyValue(Tokens[3]); // Z=...
		double PitchNEU = ParseKeyValue(Tokens[4]); // Pitch=...
		double YawNEU = ParseKeyValue(Tokens[5]); // Yaw=...
		double RollNEU = ParseKeyValue(Tokens[6]); // Roll=...
		double SpeedVal = ParseKeyValue(Tokens[7]); // Speed=...

		double RealSpeed = (double)(SpeedVal * 100.0); // 若导出时是 /100

		// 4) 坐标: (Lon,Lat,Alt)->UnrealPos
		FVector UnrealPos = CesiumGeoreference->TransformLongitudeLatitudeHeightPositionToUnreal(
			FVector((double)LonVal, (double)LatVal, (double)AltVal)
		);

		// 5) 角度: NEU->ESU->Unreal

		// 5.1) NEU->ESU
		//  Export时: YawNEU = RotESU.Yaw + 90
		//  => 读回:  YawESU = YawNEU - 90
		double YawESU = YawNEU - 90.0;
		double PitchESU = PitchNEU;
		double RollESU = RollNEU;

		// 5.2) 构造 RotESU (尽量在范围内)
		FRotator RotESU((double)PitchESU, (double)YawESU, (double)RollESU);

		// 5.3) ESU->Unreal
		FRotator FinalUnrealRot = CesiumGeoreference->TransformEastSouthUpRotatorToUnreal(
			RotESU,
			UnrealPos
		);

		// 6) 组装 FPathPointWithOrientation
		FPathPointWithOrientation NewPt;
		NewPt.Point = UnrealPos;
		NewPt.Orientation = FinalUnrealRot;
		NewPt.SegmentSpeed = RealSpeed;

		// 如果你还有 FOV/AngleRange 等字段，请在这里补全
		// ...

		LoadedPoints.Add(NewPt);
		UE_LOG(LogTemp, Log, TEXT("Loaded id %d"), IndexNum);
	}

	// 7) 赋值到 GlobalPathPoints 或其它容器
	GlobalPathPoints = LoadedPoints;

	UE_LOG(LogTemp, Log, TEXT("Imported %d path points from file: %s"), LoadedPoints.Num(), *LoadFilePath);
	return true;
}


void ADroneActor1::SmoothGlobalPathPoints_PositionOrientation(int32 Iterations /*=1*/)
{
	for (int32 iter = 0; iter < Iterations; ++iter)
	{
		TArray<FPathPointWithOrientation> TempArray = GlobalPathPoints;

		for (int32 i = 1; i < GlobalPathPoints.Num() - 1; ++i)
		{
			// 平滑位置
			const FVector& PrevPos = TempArray[i - 1].Point;
			const FVector& CurrPos = TempArray[i].Point;
			const FVector& NextPos = TempArray[i + 1].Point;
			FVector SmoothedPos = (PrevPos + CurrPos + NextPos) / 3.0f;
			GlobalPathPoints[i].Point = SmoothedPos;

			// 检测前后点之间的大幅朝向变化
			const FRotator& OriPrev = TempArray[i - 1].Orientation;
			const FRotator& OriCurr = TempArray[i].Orientation;
			const FRotator& OriNext = TempArray[i + 1].Orientation;

			float YawDiffPrev = FMath::Abs(FRotator::NormalizeAxis(OriCurr.Yaw - OriPrev.Yaw));
			float YawDiffNext = FMath::Abs(FRotator::NormalizeAxis(OriNext.Yaw - OriCurr.Yaw));

			bool bLargeYawChange = (YawDiffPrev > 40.f) || (YawDiffNext > 40.f); // 阈值可调整

			if (bLargeYawChange)
			{
				// 定义平滑范围
				int32 localRadius = 2; // 包括当前点及后续两点
				TArray<FQuat> QuatsToSmooth;

				for (int32 Offset = -localRadius; Offset <= localRadius; ++Offset)
				{
					int32 Index = FMath::Clamp(i + Offset, 0, GlobalPathPoints.Num() - 1);
					QuatsToSmooth.Add(TempArray[Index].Orientation.Quaternion());
				}

				// 计算平滑的平均四元数
				FQuat SmoothedQuat = QuatsToSmooth[0];
				for (int32 j = 1; j < QuatsToSmooth.Num(); ++j)
				{
					SmoothedQuat = FQuat::Slerp(SmoothedQuat, QuatsToSmooth[j], 1.0f / (j + 1));
				}

				// 将平滑后的四元数应用到范围内的点
				for (int32 Offset = -localRadius; Offset <= localRadius; ++Offset)
				{
					int32 Index = FMath::Clamp(i + Offset, 0, GlobalPathPoints.Num() - 1);
					GlobalPathPoints[Index].Orientation = SmoothedQuat.Rotator();
				}

				// 跳过已平滑的范围，避免重复操作
				i += localRadius;
			}
			else
			{
				// 原有逻辑：保持目标在屏幕上位置不变
				int32 AOIIndex = TempArray[i].AOIIndex;
				if (!InterestPoints.IsValidIndex(AOIIndex))
				{
					GlobalPathPoints[i].Orientation = TempArray[i].Orientation;
					continue;
				}

				FVector2D OldScreenPos = CalculateScreenPosition(TempArray[i]);
				FVector AOICenter = InterestPoints[AOIIndex].Center;
				float ThisFOV = TempArray[i].FOV;

				FRotator NewOrientation = CalculateOrientationFromScreenPosition(
					SmoothedPos,
					AOICenter,
					OldScreenPos,
					ThisFOV
				);

				GlobalPathPoints[i].Orientation = NewOrientation;
			}
		}
	}
}


void ADroneActor1::ComputeSpeedByCurvatureAndViewChange(float MaxSpeed, float MinSpeed, float SharpTurnAngle, float MaxViewChangeAngle, int32 LookaheadPoints)
{
	const int32 NumPts = GlobalPathPoints.Num();
	if (NumPts < 3)
	{
		// 路径点太少，直接设置所有点速度为MaxSpeed
		for (auto& P : GlobalPathPoints)
		{
			P.SegmentSpeed = MaxSpeed;
		}
		return;
	}

	// 首尾速度可固定设置为MaxSpeed(可根据需求修改)
	GlobalPathPoints[0].SegmentSpeed = (MinSpeed+MaxSpeed)/2;
	GlobalPathPoints[NumPts - 1].SegmentSpeed = MinSpeed;

	// 标记高Yaw变化区域
	TArray<bool> bSlowRegion;
	bSlowRegion.Init(false, NumPts);
	int32 InfluenceRadius = 3;
	float YawThreshold = 90.0f;
	for (int32 i = 1; i < NumPts - 1; ++i)
	{
		const FRotator& OriPrev = GlobalPathPoints[i - 1].Orientation;
		const FRotator& OriCurr = GlobalPathPoints[i].Orientation;
		const FRotator& OriNext = GlobalPathPoints[i + 1].Orientation;

		float YawDiffPrev = FMath::Abs(OriCurr.Yaw - OriPrev.Yaw);
		float YawDiffNext = FMath::Abs(OriNext.Yaw - OriCurr.Yaw);
		if (YawDiffPrev > 180.f) YawDiffPrev = 360.f - YawDiffPrev;
		if (YawDiffNext > 180.f) YawDiffNext = 360.f - YawDiffNext;

		if (YawDiffPrev > YawThreshold || YawDiffNext > YawThreshold)
		{
			int32 Start = FMath::Max(0, i - InfluenceRadius);
			int32 End = FMath::Min(NumPts - 1, i + InfluenceRadius);
			for (int32 j = Start; j <= End; ++j)
			{
				bSlowRegion[j] = true;
			}
		}
	}
		
	// 遍历路径点（从第1到倒数第2）
	for (int32 i = 1; i < NumPts - 1; ++i)
	{
		// 计算当前i前面、后面可用的点数
		int32 AvailableBack = i;                  // 前面有多少个？
		int32 AvailableFront = (NumPts - 1) - i;    // 后面有多少个？

		// 实际能向前看多少
		int32 BackPointsToLook = FMath::Min(LookaheadPoints, AvailableBack);
		// 实际能向后看多少
		int32 FrontPointsToLook = FMath::Min(LookaheadPoints, AvailableFront);

		// 如果两边都没有，就设成MaxSpeed即可(说明在极端位置)
		if (BackPointsToLook < 1 && FrontPointsToLook < 1)
		{
			GlobalPathPoints[i].SegmentSpeed = MaxSpeed;
			continue;
		}

		// 1) 累计转角(前后相加)
		float CumulativeTurnAngle = CalculateCumulativeTurnAngle_BothSides(i, BackPointsToLook, FrontPointsToLook);

		// 2) 累计视角变化(前后相加)
		float CumulativeViewChange = CalculateCumulativeViewChange_BothSides(i, BackPointsToLook, FrontPointsToLook);

		// 总的“有效样本数” = (BackPointsToLook + FrontPointsToLook)
		float SamplesCount = float(BackPointsToLook + FrontPointsToLook);

		// 3) 求平均转角、平均视角变化
		float AverageTurnAngle = (SamplesCount > 0.f) ? (CumulativeTurnAngle / SamplesCount) : 0.f;
		float AverageViewChange = (SamplesCount > 0.f) ? (CumulativeViewChange / SamplesCount) : 0.f;

		// 4) 转角影响因子
		float TurnAngleFactor = FMath::Clamp(10*AverageTurnAngle / SharpTurnAngle, 0.0f, 1.0f);

		// 5) 视角变化影响因子
		float ViewChangeFactor = FMath::Clamp(10*AverageViewChange / MaxViewChangeAngle, 0.0f, 1.0f);

		// 6) 综合影响因子
		float CombinedFactor = FMath::Clamp(TurnAngleFactor+ ViewChangeFactor,0.0f,1.0f);

		// 应用高Yaw变化区域影响
		if (bSlowRegion[i])
		{
			CombinedFactor = FMath::Clamp(CombinedFactor + 0.5f, 0.0f, 1.0f);
		}

		// 7) 速度插值
		float Speed = FMath::Lerp(MinSpeed, MaxSpeed, 1.0f - CombinedFactor);

		GlobalPathPoints[i].SegmentSpeed = Speed;
	}

	// 可选: 做一次或多次平滑, 让速度在相邻点间过渡更自然
	SmoothSegmentSpeed(4); // 平滑2次(可按需调整)
}


void ADroneActor1::GenerateOrbitFlightPath_Internal()
{
	// 清空现有的路径点和兴趣区域
	// 使用互斥锁保护全局变量
	FScopeLock Lock(&PathMutex);

	TArray<double> TimeArray;
	double StartTime = FPlatformTime::Seconds();
	

	// 清空现有的路径点和兴趣区域
	GlobalPathPoints.Empty();
	InterestAreas.Empty();
	GlobalBestViewPoints.Empty();

	/*FVector CameraLocation;
	FRotator CameraRotation;
	PlayerController->PlayerCameraManager->GetCameraViewPoint(CameraLocation, CameraRotation);
	float MainCameraFOV = PlayerController->PlayerCameraManager->GetFOVAngle();

	FMatrix ViewMatrix, ProjectionMatrix, ViewProjectionMatrix;
	FMinimalViewInfo CamView;

	CamView.Location = CameraLocation;
	CamView.Rotation = CameraRotation;
	CamView.FOV = MainCameraFOV;

	UGameplayStatics::GetViewProjectionMatrix(CamView, ViewMatrix, ProjectionMatrix, ViewProjectionMatrix);*/

	const float HeightPerTurn = 800.0f;  // 每圈旋转时的高度增量

	if (InterestPoints.Num() == 0) return;

	int numInterestPoints = 0;
	int numOptimizedPathPoints = 0;

	//Force3DTilesLoad();
	// 遍历每个兴趣点
	for (int currentAOIIndex = 0;currentAOIIndex < InterestPoints.Num();currentAOIIndex++)
	{
		// 直接使用当前索引计算进度百分比
		float CurrentProgress = (static_cast<float>(currentAOIIndex) / InterestPoints.Num()) * 100.0f;

		AsyncTask(ENamedThreads::GameThread, [this, CurrentProgress, currentAOIIndex]()
			{
				OnPathGenerationProgress.Broadcast(
					CurrentProgress,
					FString::Printf(TEXT("Processing AOI %d of %d..."),
						currentAOIIndex + 1, InterestPoints.Num())
				);
			});

		FCylindricalInterestPoint& InterestPoint = InterestPoints[currentAOIIndex];
		FInterestArea currentInterestArea;
		float OrbitRadius, OrbitRadius1, OrbitRadius2, MinHeight, MaxHeight;
		CalculateOrbitParameters(InterestPoint, 
			OrbitRadius,OrbitRadius1,OrbitRadius2,
			MinHeight, MaxHeight);
		TArray<float> AllRadius = { OrbitRadius,OrbitRadius1};

		TArray<float> Heights;
		Heights.Add(MinHeight);
		Heights.Add(MaxHeight);

		float TotalHeight = MaxHeight - MinHeight;
		int32 NumTurns = FMath::CeilToInt(TotalHeight / HeightPerTurn);
		float TotalAngle = 360.0f * NumTurns;
		float HeightIncrement = TotalHeight / TotalAngle;

		for (float _Radius : AllRadius) {
			int32 DynamicPointsPerCircle = FMath::Clamp(static_cast<int32>(12 * (_Radius / 1000.0f)), 8, 20);
			float AngleIncrementD = 360.0f / DynamicPointsPerCircle;

			float currentHeight = MinHeight;
			float currentAngle = 0.0f;

			// 遍历生成兴趣区域中的路径点
			while (currentHeight < MaxHeight)
			{
				currentHeight += AngleIncrementD;
				currentHeight += HeightIncrement * AngleIncrementD;

				FVector PointLocation(
					InterestPoint.Center.X + _Radius * FMath::Cos(FMath::DegreesToRadians(currentAngle)),
					InterestPoint.Center.Y + _Radius * FMath::Sin(FMath::DegreesToRadians(currentAngle)),
					currentHeight
				);

				FPathPointWithOrientation PathPoint;
				PathPoint.Point = PointLocation;
				PathPoint.Orientation = FRotationMatrix::MakeFromX(InterestPoint.Center - PointLocation).Rotator();
				PathPoint.FOV = CameraComponent->FieldOfView;  // 使用相机的FOV
				PathPoint.AOIIndex = currentAOIIndex;

				// 生成该路径点的所有候选视点
				TArray<FCandidateViewpoint> CandidateViewpoints;
				// 只获取最优视点
				FPathPointWithOrientation BestViewpoint = GenerateCandidateViewpoints(PathPoint,
					PointLocation,
					PathPoint.Orientation,
					currentAOIIndex,
					&CandidateViewpoints
				);
				// 将最优视点添加到路径点
				currentInterestArea.PathPoints.Add(BestViewpoint);

				// 将候选视点添加到当前兴趣区域
				// 其实不需要，这里只是为了调试方便
				currentInterestArea.CandidateViewpoints.Append(CandidateViewpoints);

				currentAngle += AngleIncrementD;
			}
		}
		

		// 测试显示当前兴趣区域的初始路径点
		TestPathPoints.Append(currentInterestArea.PathPoints);

		numInterestPoints += currentInterestArea.PathPoints.Num();

		// 生成并选择最佳视点组合
		TArray<TArray<FPathPointWithOrientation>> BestViewpointCombinations;
		SelectBestViewpointGroups(currentInterestArea.PathPoints, BestViewpointCombinations);

		if (BestViewpointCombinations.Num() == 0) {
			UE_LOG(LogTemp, Error, TEXT("No viewpoint combinations generated!"));
			return;
		}

		// 存储最佳曲线及其代价
		TArray<FPathPointWithOrientation> BestSplinePoints;
		TArray<FPathPointWithOrientation> BestControlPoints;
		TArray<FPathPointWithOrientation> BestCombination;
		float BestCost = FLT_MAX;
		float MaxPathLength = 0.0f;

		// 遍历每个视点组合
		int count = 0;
		UE_LOG(LogTemp, Warning, TEXT("Computing max length"));
		for (const auto& Combination : BestViewpointCombinations)
		{
			//UE_LOG(LogTemp, Warning, TEXT("Processing %dof %d viewpoint combinations"), count+1, BestViewpointCombinations.Num());
			// 生成样条曲线路径点
			TArray<FPathPointWithOrientation> SplinePoints;
			GenerateSplinePath(Combination, SplinePoints,fMinDisBetwenPoints);

			// 更新最大路径长度
			float PathLength = CalculatePathLength(SplinePoints);
			MaxPathLength = FMath::Max(MaxPathLength, PathLength);
			count++;
		} 

		// 遍历每个视点组合
		count = 0;
		for (const auto& Combination : BestViewpointCombinations)
		{
			UE_LOG(LogTemp, Warning, TEXT("Processing %d of %d viewpoint combination"), count+1, Combination.Num());

			// 生成样条曲线路径点
			TArray<FPathPointWithOrientation> SplinePoints;
			TArray<FPathPointWithOrientation> CurrentControlPoints;
			CurrentControlPoints=GenerateSplinePath(Combination, SplinePoints,fMinDisBetwenPoints);

			// 检查路径是否无碰撞
			if (!IsPathCollisionFree(SplinePoints))
			{
				UE_LOG(LogTemp, Warning, TEXT("Path collision detected!"));
				continue;  // 如果发生碰撞,跳过该路径
			};

			// 计算样条曲线的代价
			float LengthCost = CalculateLengthCost(SplinePoints, MaxPathLength);
			float QualityCost = CalculateQualityCost(SplinePoints);
			float SmoothnessCost = CalculateSmoothnessCost(SplinePoints);

			float TotalCost = LengthCost + QualityCost + SmoothnessCost;

			// 更新最佳曲线
			if (TotalCost < BestCost)
			{
				BestCost = TotalCost;
				BestSplinePoints = SplinePoints;
				BestControlPoints =(CurrentControlPoints);
				BestCombination = Combination; // 保存最佳视点组合
			}
			count++;
		}

		// 如果没有找到最佳曲线，先暂时放进去一条测试
		if (BestSplinePoints.Num() == 0&& !BestViewpointCombinations.IsEmpty())
		{
			TArray<FPathPointWithOrientation> Combination = BestViewpointCombinations[0];
			
			TArray<FPathPointWithOrientation> SplinePoints;
			TArray<FPathPointWithOrientation> CurrentControlPoints;
			CurrentControlPoints=GenerateSplinePath(Combination, SplinePoints,fMinDisBetwenPoints);
			if (CurrentControlPoints.Num() == 0)
			{
				UE_LOG(LogTemp, Warning, TEXT("No control points generated!"));
				continue;
			}
			BestControlPoints =(CurrentControlPoints);
			BestSplinePoints = SplinePoints;

			UE_LOG(LogTemp, Warning, TEXT("No optimal path found! Using a test path"));
		}
		else if(BestViewpointCombinations.IsEmpty()){
			UE_LOG(LogTemp, Warning, TEXT("Spline generation failed. Using original path."));
			BestSplinePoints = currentInterestArea.PathPoints;
		}


		// 更新当前兴趣区域的路径点为样条曲线点
		currentInterestArea.PathPoints = BestSplinePoints;
		numOptimizedPathPoints += currentInterestArea.PathPoints.Num();

		// 添加当前兴趣区域路径点到总路径点
		InterestAreas.Add(currentInterestArea);

		// 添加到GlobalPathPoints中
		// 应该要在外部实现
		/*GlobalPathPoints.Append(currentInterestArea.PathPoints);*/

		// 测试用
		GlobalBestSplinePoints.Append(BestSplinePoints);
		GlobalBestKeyPoints.Append(BestCombination);
		GlobalBestControlPoints.Append(BestControlPoints);


		GlobalBestViewPoints.Append(BestControlPoints);

		// 输出现在的点数
		UE_LOG(LogTemp, Warning, TEXT("Generated %d path points for AOI %d"),
			currentInterestArea.PathPoints.Num(), currentAOIIndex);
	}

	double Time1 = FPlatformTime::Seconds();
	double TimeElapsed = Time1 - StartTime;
	TimeArray.Add(TimeElapsed);

	// --------------------------------------------------------------
	// 处理过渡航线和STSP问题
	TArray<FDoubleArray> CostMatrix;
	bool bSuccess = BuildSTSPCostMatrix(CostMatrix);
	if (!bSuccess)
	{
		UE_LOG(LogTemp, Error, TEXT("Failed to build cost matrix."));
	}
	else {
		UE_LOG(LogTemp, Warning, TEXT("Successfully build cost matrix."));
	}

	double Time2 = FPlatformTime::Seconds();
	double TimeElapsed2 = Time2 - Time1;
	TimeArray.Add(TimeElapsed2);

	UMySTSPClass* STSPSolver = NewObject<UMySTSPClass>();
	int NumRegions = InterestAreas.Num();
	TArray<int32> BestOrder = STSPSolver->GeneticAlgorithm(CostMatrix, NumRegions);

	double Time3 = FPlatformTime::Seconds();
	double TimeElapsed3 = Time3 - Time2;
	TimeArray.Add(TimeElapsed3);

	// 将数组转换为字符串
	FString BestOrderString;
	for (int32 Value : BestOrder)
	{
		BestOrderString += FString::FromInt(Value) + TEXT(" ");
	}
	// 输出到日志
	UE_LOG(LogTemp, Log, TEXT("Best Order: %s"), *BestOrderString);

	TArray<FPathPointWithOrientation> FinalPath = BuildFinalPath(BestOrder);
	UE_LOG(LogTemp,Warning, TEXT("Successfully build final path."));

	GlobalPathPoints.Append(FinalPath);

	SmoothGlobalPathPoints_PositionOrientation(2);

	// 计算所有速度
	ComputeSpeedByCurvatureAndViewChange(fMaxFlightSpeed,fMinFlightSpeed);
	UE_LOG(LogTemp, Warning, TEXT("Successfully compute speed."));

	double Time4 = FPlatformTime::Seconds();
	double TimeElapsed4 = Time4 - Time3;
	TimeArray.Add(TimeElapsed4);

	for (int i = 0; i < TimeArray.Num(); i++)
	{
		UE_LOG(LogTemp, Warning, TEXT("Phase %d Time: %f"), i, TimeArray[i]);
	}

	ExportPathPointsToWGS84Txt();

	DisableForce3DTilesLoad();
	// 完成处理
	// 尝试全部交给广播事件处理
	AsyncTask(ENamedThreads::GameThread, [this, numInterestPoints, numOptimizedPathPoints]()
		{
			OnPathGenerationProgress.Broadcast(100.0f, TEXT("Generation complete"));
			OnPathGenerationComplete.Broadcast(true);

			GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Yellow,
				FString::Printf(TEXT("Generated paths: %d initial, %d optimized"),
					numInterestPoints, numOptimizedPathPoints));
		});

	// GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Yellow, FString::Printf(TEXT("Generated initial orbit paths with %d points"), numInterestPoints));
}


// 测试生成候选视点
FPathPointWithOrientation ADroneActor1::TestGenerateCandidateViewpoints(
	const FPathPointWithOrientation& PathPoint
)
{
	FVector PathPointLocation = PathPoint.Point;
	FRotator OriginalRotation = PathPoint.Orientation;
	int32 CurrentAOIIndex = PathPoint.AOIIndex;
	TArray<FCandidateViewpoint> Candidates;

	FPathPointWithOrientation bestPathPoint;

	// 获取当前兴趣点
	check(InterestPoints.IsValidIndex(CurrentAOIIndex));
	const FCylindricalInterestPoint& CurrentAOI = InterestPoints[CurrentAOIIndex];
	FVector TargetWorldLocation = CurrentAOI.Center;

	DrawDebugSphere(GetWorld(), TargetWorldLocation, 50.0f, 8, FColor::Blue, true, 0.1f, 0, 2.0f);

	// 定义理想位置
	TArray<TPair<FVector2D, FString>> IdealPositions = {
		{FVector2D(0.0f, 0.0f), TEXT("Center")},
		{FVector2D(-0.167f, 0.167f), TEXT("TopLeft")},
		{FVector2D(0.167f, 0.167f), TEXT("TopRight")},
		{FVector2D(-0.167f, -0.167f), TEXT("BottomLeft")},
		{FVector2D(0.167f, -0.167f), TEXT("BottomRight")} // 格式是X,Y
	};

	// 定义焦距
	const float FocalLength = 1.0f / FMath::Tan(FMath::DegreesToRadians(CameraComponent->FieldOfView / 2.0f));
	const float AspectRatio = CameraComponent->AspectRatio;
	UE_LOG(LogTemp, Warning, TEXT("FocalLength: %f, AspectRatio: %f"), FocalLength, AspectRatio);

	// 生成候选视点
	for (const auto& IdealPos : IdealPositions)
	{
		FCandidateViewpoint Candidate;
		Candidate.Location = PathPointLocation;
		Candidate.OriginalRotation = OriginalRotation;
		Candidate.AOIIndex = CurrentAOIIndex;
		Candidate.FOV = PathPoint.FOV;
		Candidate.TargetImagePosition = IdealPos.Key;
		FPathPointWithOrientation viewCandidate;

		FVector ToTarget = TargetWorldLocation - PathPointLocation;
		FRotator BaseRotation = ToTarget.Rotation();  // 从目标位置到路径点的位置差生成旋转

		// 计算 Pitch 和 Yaw 的偏移
		// 加上负号就正确了
		double PitchOffset = - FMath::RadiansToDegrees(FMath::Atan2(IdealPos.Key.Y, FocalLength));
		double YawOffset = - FMath::RadiansToDegrees(FMath::Atan2(IdealPos.Key.X* AspectRatio, FocalLength));

		// 更新目标旋转角度
		Candidate.TargetRotation = BaseRotation + FRotator(PitchOffset, YawOffset, 0.0f);

		// 计算成本
		Candidate.CompositionCost = CalculateCompositionCost(Candidate);
		Candidate.VisibilityCost = CalculateVisibilityCost(Candidate);

		// 计算总成本
		const float CompositionWeight = 0.6f;
		const float VisibilityWeight = 0.4f;
		Candidate.TotalCost =
			CompositionWeight * Candidate.CompositionCost +
			VisibilityWeight * Candidate.VisibilityCost;

		// 添加到候选列表
		Candidates.Add(Candidate);

		viewCandidate.Point = Candidate.Location;
		viewCandidate.Orientation = Candidate.TargetRotation;
		viewCandidate.FOV = Candidate.FOV;
		viewCandidate.SegmentSpeed = 10.0f;

		// 将兴趣区域中心投影到屏幕空间
		FVector2D ScreenPosition;
		bool bProjected = ProjectWorldPointToScreen(TargetWorldLocation, ScreenPosition, viewCandidate.Point, viewCandidate.Orientation, PathPoint.FOV);

		if (bProjected)
		{
			UE_LOG(LogTemp, Warning, TEXT("Projected screen position: %s"), *ScreenPosition.ToString());
		}
		
		GlobalPathPoints.Add(viewCandidate);
	}

	// 排序
	Candidates.Sort([](const FCandidateViewpoint& A, const FCandidateViewpoint& B) {
		return A.TotalCost < B.TotalCost;
		});

	// 返回最优视点
	bestPathPoint.Point = Candidates[0].Location;
	bestPathPoint.Orientation = Candidates[0].TargetRotation;
	bestPathPoint.FOV = Candidates[0].FOV;
	bestPathPoint.AOIIndex = Candidates[0].AOIIndex;

	// 输出最佳视角的TargetImagePosition
	FVector2D BestImagePosition = Candidates[0].TargetImagePosition;
	FString CorrespondingName;
	bool bFound = false;

	for (const TPair<FVector2D, FString>& Pair : IdealPositions) {
		if (Pair.Key == BestImagePosition) {
			CorrespondingName = Pair.Value;
			bFound = true;
			break;
		}
	}

	UE_LOG(LogTemp, Warning, TEXT("Best viewpoint: %s with coordiates of: %s"), *CorrespondingName, *BestImagePosition.ToString());

	return bestPathPoint;
}

void ADroneActor1::OnTestGenerateCandidateViewpoints()
{

	if (InterestPoints.Num()== 0) {

		GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Yellow, TEXT("No interest points available!"));
		UE_LOG(LogTemp, Warning, TEXT("No interest points available!"));
		return;
	}

	if (fGenerationFinished == false) {
		
		// 重置当前索引
		currentIndex = 0;
		GlobalPathPoints.Empty();

		bIsSlowMove = true; // 开启慢速移动

		// 选择当前位置作为测试视点
		// 获取当前无人机的位置和朝向
		FVector DronePosition = GetActorLocation();
		FRotator DroneOrientation = GetActorRotation();

		FPathPointWithOrientation RandomPathPoint = { DronePosition ,DroneOrientation, CameraComponent->FieldOfView};
		RandomPathPoint.AOIIndex = 0;

		// 生成候选视点
		FPathPointWithOrientation BestViewpoint = TestGenerateCandidateViewpoints(RandomPathPoint);

		for (int32 i = 0; i < GlobalPathPoints.Num(); ++i)
		{
			const FPathPointWithOrientation& PathPoint = GlobalPathPoints[i];
			FVector OrientationEnd = PathPoint.Point + PathPoint.Orientation.Vector() * 50.0f;
			DrawDebugLine(GetWorld(), PathPoint.Point, OrientationEnd, FColor::Red, true, 0.1f, 0, 0.5f);
		}

		//GlobalPathPoints.Add(BestViewpoint);
		//bShouldDrawPathPoints = true;
		fGenerationFinished = true;

	}
	else {
		bIsSlowMove = false;
		/*bShouldDrawPathPoints = false;
		fGenerationFinished = false;*/
		FlushPersistentDebugLines(GetWorld());

		for (int32 i = 0; i < GlobalPathPoints.Num(); ++i)
		{
			const FPathPointWithOrientation& PathPoint = GlobalPathPoints[i];
			// 绘制航点位置的小圆球
			DrawDebugSphere(GetWorld(), PathPoint.Point, 50.0f, 8, FColor::Blue, true, 0.1f, 0, 2.0f);

			// 绘制相机朝向的线
			FVector OrientationEnd = PathPoint.Point + PathPoint.Orientation.Vector() * 50.0f;
			DrawDebugLine(GetWorld(), PathPoint.Point, OrientationEnd, FColor::Red, true, 0.1f, 0, 2.0f);
			//SplineComponent->AddSplinePoint(PathPoint.Point, ESplineCoordinateSpace::World);
		}
		
		// 清空兴趣点和路径点
		/*InterestPoints.Empty();
		GlobalPathPoints.Empty();
		DestroyPathPoints();*/

		currentIndex = 0;

		//GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Yellow, TEXT("Stop drawing path points"));
	}
	
}


// 生成候选视点
FPathPointWithOrientation ADroneActor1::GenerateCandidateViewpoints(
	const FPathPointWithOrientation& PathPoint,
	const FVector& PathPointLocation,
	const FRotator& OriginalRotation,
	const int32 CurrentAOIIndex,
	TArray<FCandidateViewpoint>* OutAllCandidates = nullptr) // 可选参数，用于存储所有候选点
{
	TArray<FCandidateViewpoint> Candidates;

	FPathPointWithOrientation bestPathPoint;

	// 获取当前兴趣点
	check(InterestPoints.IsValidIndex(CurrentAOIIndex));
	const FCylindricalInterestPoint& CurrentAOI = InterestPoints[CurrentAOIIndex];
	FVector TargetWorldLocation = CurrentAOI.Center;

	// 定义理想位置
	TArray<TPair<FVector2D, FString>> IdealPositions = {
		{FVector2D(0.0f, 0.0f), TEXT("Center")},
		{FVector2D(-0.167f, 0.167f), TEXT("TopLeft")},
		{FVector2D(0.167f, 0.167f), TEXT("TopRight")},
		{FVector2D(-0.167f, -0.167f), TEXT("BottomLeft")},
		{FVector2D(0.167f, -0.167f), TEXT("BottomRight")} // 格式是X,Y
	};

	// 定义焦距
	const float FocalLength = 1.0f / FMath::Tan(FMath::DegreesToRadians(CameraComponent->FieldOfView/ 2.0f));
	const float AspectRatio = CameraComponent->AspectRatio;

	// 生成候选视点
	for (const auto& IdealPos : IdealPositions)
	{
		FCandidateViewpoint Candidate;
		Candidate.Location = PathPointLocation;
		Candidate.OriginalRotation = OriginalRotation;
		Candidate.AOIIndex = CurrentAOIIndex;
		Candidate.FOV = PathPoint.FOV;
		Candidate.TargetImagePosition = IdealPos.Key;

		FVector ToTarget = TargetWorldLocation - PathPointLocation;
		FRotator BaseRotation = ToTarget.Rotation();  // 从目标位置到路径点的位置差生成旋转

		// 计算 Pitch 和 Yaw 的偏移
		float PitchOffset = - FMath::RadiansToDegrees(FMath::Atan2(IdealPos.Key.Y, FocalLength));
		float YawOffset = - FMath::RadiansToDegrees(FMath::Atan2(IdealPos.Key.X*AspectRatio, FocalLength));

		// 更新目标旋转角度
		Candidate.TargetRotation = BaseRotation + FRotator(PitchOffset, YawOffset, 0.0f);

		// 计算成本
		Candidate.CompositionCost = CalculateCompositionCost(Candidate);
		Candidate.VisibilityCost = CalculateVisibilityCost(Candidate);

		// 计算总成本
		const float CompositionWeight = 0.6f;
		const float VisibilityWeight = 0.4f;
		Candidate.TotalCost =
			CompositionWeight * Candidate.CompositionCost +
			VisibilityWeight * Candidate.VisibilityCost;

		// 添加到候选列表
		Candidates.Add(Candidate);
	}

	// 排序
	Candidates.Sort([](const FCandidateViewpoint& A, const FCandidateViewpoint& B) {
		return A.TotalCost < B.TotalCost;
		});

	// 保存所有候选点（如果需要）
	if (OutAllCandidates != nullptr)
	{
		*OutAllCandidates = Candidates;
	}

	// 返回最优视点
	bestPathPoint.Point = Candidates[0].Location;
	bestPathPoint.Orientation = Candidates[0].TargetRotation;
	bestPathPoint.FOV = Candidates[0].FOV;
	bestPathPoint.AOIIndex = Candidates[0].AOIIndex;

	return bestPathPoint;
}

void ADroneActor1::HandleTilesetLoaded()
{
	// 每当一个 Tileset 加载完成，减少计数
	PendingTilesets--;

	// 当所有 Tileset 都加载完成时，设置 Promise
	if (PendingTilesets <= 0 && LoadPromise.IsValid())
	{
		LoadPromise->SetValue();
	}
}

// 强制加载所有瓦片
void ADroneActor1::Force3DTilesLoad() {
	TPromise<void> Promise;
	TFuture<void> Future = Promise.GetFuture();

	AsyncTask(ENamedThreads::GameThread, [this, Promise = MoveTemp(Promise)]() mutable
		{
			TArray<AActor*> FoundActors;
			UGameplayStatics::GetAllActorsOfClass(GetWorld(), ACesium3DTileset::StaticClass(), FoundActors);

			if (FoundActors.Num() > 0)
			{
				for (AActor* Actor : FoundActors)
				{
					if (ACesium3DTileset* Tileset = Cast<ACesium3DTileset>(Actor))
					{
						Tileset->EnableFrustumCulling = false;  // 禁用视锥体剔除，强制加载所有相关瓦片
						Tileset->EnableFogCulling = false;      // 禁用雾剔除
						//Tileset->SetCreatePhysicsMeshes(true);  // 确保物理碰撞体生成

						UE_LOG(LogTemp, Log, TEXT("Tileset updated: %s"), *Tileset->GetName());
					}
				}
			}
			else
			{
				UE_LOG(LogTemp, Warning, TEXT("No Cesium3DTilesets found in the world."));
			}

			// 任务完成，触发事件
			Promise.SetValue();
		});

	// 等待任务完成
	Future.Wait();
	UE_LOG(LogTemp, Log, TEXT("Force3DTilesLoad completed."));
}


// 禁用强制加载3D瓦片	
void ADroneActor1::DisableForce3DTilesLoad() {
	TPromise<void> Promise;
	TFuture<void> Future = Promise.GetFuture();

	AsyncTask(ENamedThreads::GameThread, [this, Promise = MoveTemp(Promise)]() mutable
		{
			TArray<AActor*> FoundActors;
			UGameplayStatics::GetAllActorsOfClass(GetWorld(), ACesium3DTileset::StaticClass(), FoundActors);

			if (FoundActors.Num() > 0)
			{
				for (AActor* Actor : FoundActors)
				{
					ACesium3DTileset* Tileset = Cast<ACesium3DTileset>(Actor);
					if (Tileset)
					{
						Tileset->EnableFrustumCulling = true; // 重新启用视锥体剔除
						Tileset->EnableFogCulling = true;
						Tileset->RefreshTileset(); // 刷新瓦片加载
						UE_LOG(LogTemp, Log, TEXT("Tileset updated: %s"), *Tileset->GetName());
					}
				}
			}
			else
			{
				UE_LOG(LogTemp, Warning, TEXT("No Cesium3DTilesets found in the world."));
			}

			// 任务完成，触发事件
			Promise.SetValue();
		});

	// 等待任务完成
	Future.Wait();
	UE_LOG(LogTemp, Log, TEXT("DisableForce3DTilesLoad completed."));
}

// 渲染视点到RenderTarget
void ADroneActor1::RenderViewpointToRenderTarget(const FPathPointWithOrientation& Viewpoint)
{
	// 保存当前相机状态
	FVector OriginalLocation = CameraComponent->GetComponentLocation();
	FRotator OriginalRotation = CameraComponent->GetComponentRotation();
	float OriginalFOV = SceneCaptureComponent->FOVAngle;

	// 设置新的相机参数
	CameraComponent->SetWorldLocation(Viewpoint.Point);
	CameraComponent->SetWorldRotation(Viewpoint.Orientation);
	SceneCaptureComponent->FOVAngle = Viewpoint.FOV;
	SceneCaptureComponent->UpdateContent();

	// 确保资源加载完成
	//UGameplayStatics::FlushLevelStreaming(GetWorld());

	// 延迟捕获，等待瓦片加载完成
	//FTimerHandle TimerHandle;
	//GetWorld()->GetTimerManager().SetTimer(TimerHandle, [this]()
	//	{
	//		FlushRenderingCommands(); // 强制完成渲染队列
	//		SceneCaptureComponent->CaptureScene();
	//	}, 0.5f, false);

	// 捕获场景
	//FlushRenderingCommands(); // 完成所有挂起的渲染命令
	SceneCaptureComponent->CaptureScene();

	// 恢复原始相机状态
	CameraComponent->SetWorldLocation(OriginalLocation);
	CameraComponent->SetWorldRotation(OriginalRotation);
	SceneCaptureComponent->FOVAngle = OriginalFOV;
}


float ADroneActor1::CalculatePathLength(const TArray<FPathPointWithOrientation>& _PathPoints)
{
	float TotalLength = 0.0f;

	for (int i = 1; i < _PathPoints.Num(); ++i)
	{
		const FVector& PrevPoint = _PathPoints[i - 1].Point;
		const FVector& CurrentPoint = _PathPoints[i].Point;

		float SegmentLength = FVector::Dist(PrevPoint, CurrentPoint);
		TotalLength += SegmentLength;
	}

	return TotalLength;
}


bool ADroneActor1::IsAngleVisible_KeepNegativeAngles(
	const FPathPointWithOrientation& Viewpoint,
	float YawAngleDeg,
	const FCylindricalInterestPoint& AOI)
{
	// 将角度转换为弧度
	float Rad = FMath::DegreesToRadians(YawAngleDeg);

	// 构造2D朝向(在X-Y平面上)
	FVector2D Direction2D(FMath::Cos(Rad), FMath::Sin(Rad));

	// 3D朝向向量(假设忽略Z，水平旋转)
	FVector RayDirection(Direction2D.X, Direction2D.Y, 0.f);

	// 起点：无人机所在位置
	FVector Start = Viewpoint.Point;

	// 终点：大概就是朝该方向打一条线足够远，
	// 或者针对AOI再多一个半径
	float Distance = (AOI.Center - Start).Size();
	FVector End = Start + RayDirection * (Distance + AOI.Radius);

	// 在这里进行遮挡检测(LineTraceSingleByChannel示例)
	FHitResult HitResult;
	FCollisionQueryParams TraceParams(SCENE_QUERY_STAT(DroneVisibilityTrace), true, this);

	bool bHit = GetWorld()->LineTraceSingleByChannel(
		HitResult,
		Start,
		End,
		ECollisionChannel::ECC_Visibility,
		TraceParams
	);

	if (!bHit)
	{
		// 没撞到任何东西 → 没有被遮挡
		return true;
	}
	else
	{
		// 如果想判断“撞到的是否是我们想要的 AOI”，
		// 需要看下HitResult.GetActor()或碰撞组件是否是AOI本身。
		// 但通常 AOI 只是个逻辑体，没有真实碰撞体，
		// 所以只要Hit，就说明被别的障碍物挡住了。
		return false;
	}
}

TArray<FAngleRange> ADroneActor1::BuildVisibleRanges_KeepNegativeAngles(
	const TArray<float>& SortedAngles,
	float AngleStep)
{
	TArray<FAngleRange> Ranges;

	if (SortedAngles.Num() == 0)
	{
		return Ranges;
	}

	// 当前可见区间的起始
	float RangeStart = SortedAngles[0];
	float PrevAngle = SortedAngles[0];

	for (int i = 1; i < SortedAngles.Num(); i++)
	{
		float CurrAngle = SortedAngles[i];
		// 若与上一个可见角的差值大于阈值, 说明出现间断
		if ((CurrAngle - PrevAngle) > (2.f * AngleStep))
		{
			// 结束上一个区间
			FAngleRange NewRange;
			NewRange.LowerBound = RangeStart;
			NewRange.UpperBound = PrevAngle;
			Ranges.Add(NewRange);

			// 开启新的区间
			RangeStart = CurrAngle;
		}
		PrevAngle = CurrAngle;
	}

	// 最后一段
	{
		FAngleRange NewRange;
		NewRange.LowerBound = RangeStart;
		NewRange.UpperBound = PrevAngle;
		Ranges.Add(NewRange);
	}

	return Ranges;
}

// 计算视点覆盖角度
// 考虑到视角是否被遮挡
float ADroneActor1::CalculateViewpointCoverage(FPathPointWithOrientation& _Viewpoint, const FCylindricalInterestPoint& AOI)
{
	// 1. 基本几何计算 ----------------------------------------
	FVector _ViewpointToAOICenter = AOI.Center - _Viewpoint.Point;
	float Distance = _ViewpointToAOICenter.Size();

	// 如果视点在AOI内部
	if (Distance <= AOI.Radius)
	{
		_Viewpoint.AngleRange.LowerBound = -180.f;
		_Viewpoint.AngleRange.UpperBound = 180.f;
		_Viewpoint.CoverageAngle = 360.f;
		return 360.f;
	}

	// 几何上的半覆盖角
	float HalfCoverageAngleDeg = FMath::RadiansToDegrees(
		FMath::Acos(AOI.Radius / Distance)
	);

	// 原视点朝向
	FVector ViewDir = _Viewpoint.Orientation.Vector();
	float ViewAngleDeg = FMath::Atan2(ViewDir.Y, ViewDir.X) * 180.f / PI;

	// *** 去掉这行，让角度保留原值(可能<-180或>180)
	// ViewAngleDeg = FMath::UnwindDegrees(ViewAngleDeg);

	float LowerBound = ViewAngleDeg - HalfCoverageAngleDeg;
	float UpperBound = ViewAngleDeg + HalfCoverageAngleDeg;

	// *** 去掉下面两行，让上下界保持原值
	// LowerBound = FMath::UnwindDegrees(LowerBound);
	// UpperBound = FMath::UnwindDegrees(UpperBound);

	// 2. 做离散采样 + 遮挡检测 --------------------------------
	TArray<float> VisibleAngles;
	const float AngleStep = 1.0f;

	if (LowerBound <= UpperBound)
	{
		for (float CurrAngle = LowerBound; CurrAngle <= UpperBound; CurrAngle += AngleStep)
		{
			// *** 不再做 UnwindDegrees
			// float AngleCandidate = FMath::UnwindDegrees(CurrAngle);
			float AngleCandidate = CurrAngle;

			if (IsAngleVisible_KeepNegativeAngles(_Viewpoint, AngleCandidate, AOI))
			{
				VisibleAngles.Add(AngleCandidate);
			}
		}
	}
	else
	{
		// 跨界情况(类似以前的逻辑，但我们不再把边界都规范到[-180,180])
		// 你可以保留这一段以继续支持“跨过某个极值”的场景

		// A. 从LowerBound -> 一些上限(可能是999.0f, 例如你自己定义)
		//   或者你仍然想用 180.f 这个“截断”
		//   这得根据你的需求改
		for (float CurrAngle = LowerBound; CurrAngle <= 180.f; CurrAngle += AngleStep)
		{
			float AngleCandidate = CurrAngle;
			if (IsAngleVisible_KeepNegativeAngles(_Viewpoint, AngleCandidate, AOI))
			{
				VisibleAngles.Add(AngleCandidate);
			}
		}
		// B. 从-180 -> UpperBound
		//   同理，你可能想改成从 -1800 -> UpperBound, 看你如何定义跨界
		for (float CurrAngle = -180.f; CurrAngle <= UpperBound; CurrAngle += AngleStep)
		{
			float AngleCandidate = CurrAngle;
			if (IsAngleVisible_KeepNegativeAngles(_Viewpoint, AngleCandidate, AOI))
			{
				VisibleAngles.Add(AngleCandidate);
			}
		}
	}

	// 3. 合并区间
	VisibleAngles.Sort();
	TArray<FAngleRange> VisibleRanges = BuildVisibleRanges_KeepNegativeAngles(VisibleAngles, AngleStep);

	// 4. 找到最大覆盖段
	float MaxCoverage = 0.f;
	float BestLower = 0.f;
	float BestUpper = 0.f;

	for (auto& Range : VisibleRanges)
	{
		float Coverage = Range.UpperBound - Range.LowerBound;
		if (Coverage > MaxCoverage)
		{
			MaxCoverage = Coverage;
			BestLower = Range.LowerBound;
			BestUpper = Range.UpperBound;
		}
	}

	// 更新
	_Viewpoint.AngleRange.LowerBound = BestLower;
	_Viewpoint.AngleRange.UpperBound = BestUpper;
	_Viewpoint.CoverageAngle = MaxCoverage;

	return MaxCoverage;
}


void ADroneActor1::SelectBestViewpoints(
	const TArray<FPathPointWithOrientation>& Candidates,
	TArray<FPathPointWithOrientation>& OutBestViewpoints,
	int32 NumRequired = 3)
{
	// 复制并评分候选视点
	TArray<FPathPointWithOrientation> ScoredViewpoints = Candidates;
	for (FPathPointWithOrientation& Viewpoint : ScoredViewpoints)
	{
		// 渲染并获取美学评分
		RenderViewpointToRenderTarget(Viewpoint);
		if (NimaTracker)
		{
			NimaTracker->RunInference(RenderTarget);
			Viewpoint.AestheticScore = NimaTracker->GetNimaScore();
		}

		// 计算覆盖角度
		Viewpoint.CoverageAngle = CalculateViewpointCoverage(Viewpoint, InterestPoints[Viewpoint.AOIIndex]);
	}

	// 筛选美学评分高于阈值的视点
	TArray<FPathPointWithOrientation> FilteredViewpoints;
	for (const FPathPointWithOrientation& Viewpoint : ScoredViewpoints)
	{
		if (Viewpoint.AestheticScore >= AestheticScoreThreshold)
		{
			FilteredViewpoints.Add(Viewpoint);
		}
	}

	// 生成所有可能的视点组合
	TArray<TArray<FPathPointWithOrientation>> ViewpointCombinations;
	GenerateViewpointGroups(FilteredViewpoints, NumRequired, ViewpointCombinations);

	// 筛选满足距离和覆盖度约束的视点组合
	TArray<TArray<FPathPointWithOrientation>> ValidViewpointCombinations;
	for (const auto& Combination : ViewpointCombinations)
	{
		if (IsCoverageSatisfied(Combination) && IsDistanceSatisfied(Combination))
		{
			ValidViewpointCombinations.Add(Combination);
		}
	}

	// 如果没有满足约束条件的视点组合,返回空数组
	if (ValidViewpointCombinations.Num() == 0)
	{
		OutBestViewpoints.Empty();
		return;
	}

	// 在满足约束条件的视点组合中选择美学评分最高的组合
	int32 BestCombinationIndex = 0;
	float BestCombinationScore = CalculateGroupAestheticScore(ValidViewpointCombinations[0]);
	for (int32 i = 1; i < ValidViewpointCombinations.Num(); ++i)
	{
		float CombinationScore = CalculateGroupAestheticScore(ValidViewpointCombinations[i]);
		if (CombinationScore > BestCombinationScore)
		{
			BestCombinationIndex = i;
			BestCombinationScore = CombinationScore;
		}
	}

	OutBestViewpoints = ValidViewpointCombinations[BestCombinationIndex];
}



//void ADroneActor1::SelectBestViewpointGroups(
//	const TArray<FPathPointWithOrientation>& Candidates,
//	TArray<TArray<FPathPointWithOrientation>>& OutViewpointGroups,
//	int32 NumGroups,
//	int32 NumViewpointsPerGroup)
//{
//	// 复制并评分候选视点
//	TArray<FPathPointWithOrientation> ScoredViewpoints = Candidates;
//	for (FPathPointWithOrientation& Viewpoint : ScoredViewpoints)
//	{
//		// 渲染并获取美学评分
//		RenderViewpointToRenderTarget(Viewpoint);
//		if (NimaTracker)
//		{
//			NimaTracker->RunInference(RenderTarget);
//			Viewpoint.AestheticScore = NimaTracker->GetNimaScore();
//		}
//
//		// 计算覆盖角度
//		Viewpoint.CoverageAngle = CalculateViewpointCoverage(Viewpoint, InterestPoints[Viewpoint.AOIIndex]);
//	}
//
//	// 筛选美学评分高于阈值的视点
//	TArray<FPathPointWithOrientation> FilteredViewpoints;
//	for (const FPathPointWithOrientation& Viewpoint : ScoredViewpoints)
//	{
//		if (Viewpoint.AestheticScore >= AestheticScoreThreshold)
//		{
//			FilteredViewpoints.Add(Viewpoint);
//		}
//	}
//
//	// 生成视点组合
//	TArray<TArray<FPathPointWithOrientation>> ViewpointGroups;
//	GenerateViewpointGroups(FilteredViewpoints, NumViewpointsPerGroup, ViewpointGroups);
//
//	// 筛选满足覆盖度和距离约束的视点组合
//	TArray<TArray<FPathPointWithOrientation>> ValidViewpointGroups;
//	for (const auto& Group : ViewpointGroups)
//	{
//		if (IsCoverageSatisfied(Group) && IsDistanceSatisfied(Group))
//		{
//			ValidViewpointGroups.Add(Group);
//		}
//	}
//
//	// 如果没有满足约束条件的视点组合,返回空数组
//	if (ValidViewpointGroups.Num() == 0)
//	{
//		OutViewpointGroups.Empty();
//		return;
//	}
//
//	// 在满足约束条件的视点组合中选择美学评分最高的组合
//	int32 BestGroupIndex = 0;
//	float BestGroupScore = CalculateGroupAestheticScore(ValidViewpointGroups[0]);
//	for (int32 i = 1; i < ValidViewpointGroups.Num(); ++i)
//	{
//		float GroupScore = CalculateGroupAestheticScore(ValidViewpointGroups[i]);
//		if (GroupScore > BestGroupScore)
//		{
//			BestGroupIndex = i;
//			BestGroupScore = GroupScore;
//		}
//	}
//
//	OutViewpointGroups = { ValidViewpointGroups[BestGroupIndex] };
//}


void ADroneActor1::SelectBestViewpointGroups(
	const TArray<FPathPointWithOrientation>& Candidates,
	TArray<TArray<FPathPointWithOrientation>>& OutViewpointGroups,
	int32 NumGroups,
	int32 NumViewpointsPerGroup)
{
	// 复制候选视点
	TArray<FPathPointWithOrientation> ScoredViewpoints = Candidates;

	// 展示候选点数量
	UE_LOG(LogTemp, Warning, TEXT("Number of candidates: %d"), ScoredViewpoints.Num());

	// 创建一个异步任务计数器
	FThreadSafeCounter TaskCounter(ScoredViewpoints.Num());
	NimaTracker->IfSaveImage = true; // 决定是否保存美学评分的图像
	// 为每个视点创建异步推理任务
	for (FPathPointWithOrientation& Viewpoint : ScoredViewpoints)
	{
		AsyncTask(ENamedThreads::GameThread, [this, &Viewpoint, &TaskCounter]()
			{
				// 在游戏线程上调用渲染函数
				RenderViewpointToRenderTarget(Viewpoint);

				// 在游戏线程上运行推理任务
				if (NimaTracker && RenderTarget)
				{
					NimaTracker->RunInference(RenderTarget);
				}

				// 等待异步推理完成,直到获取到有效的美学评分
				while (NimaTracker->GetNimaScore() <= 0)
				{
					FPlatformProcess::Sleep(0.001f);
				}

				Viewpoint.AestheticScore = NimaTracker->GetNimaScore();
				NimaTracker->ResetNimaScore(); // 重置美学评分
				// 计算覆盖角度
				Viewpoint.CoverageAngle = CalculateViewpointCoverage(Viewpoint, InterestPoints[Viewpoint.AOIIndex]);

				// 异步任务完成,减少计数器
				TaskCounter.Decrement();
			});

		while (Viewpoint.AestheticScore <= 0)
		{
			FPlatformProcess::Sleep(0.001f);
		}

	}

	// 等待所有异步任务完成
	while (TaskCounter.GetValue() > 0)
	{
		FPlatformProcess::Sleep(0.001f);
	}

	NimaTracker->IfSaveImage = false;
	UE_LOG(LogTemp, Warning, TEXT("Aesthetic Computation Completed"));

	// ... (其余代码保持不变)

	// 筛选美学评分高于阈值的视点
	// 以及视点是否看到了兴趣点的顶部
	TArray<FPathPointWithOrientation> FilteredViewpoints;
	for (const FPathPointWithOrientation& Viewpoint : ScoredViewpoints)
	{
		if (Viewpoint.AestheticScore >= AestheticScoreThreshold&&DoesViewpointSeeTop(Viewpoint, InterestPoints[Viewpoint.AOIIndex]))
		{
			FilteredViewpoints.Add(Viewpoint);
		}
	}

	// 生成视点组合
	TArray<TArray<FPathPointWithOrientation>> ViewpointGroups;
	GenerateViewpointGroups(FilteredViewpoints, NumViewpointsPerGroup, ViewpointGroups);
	UE_LOG(LogTemp, Warning, TEXT("Number of viewpoint groups: %d"), ViewpointGroups.Num());
	// 筛选满足覆盖度和距离约束的视点组合
	TArray<TArray<FPathPointWithOrientation>> ValidViewpointGroups;
	for (const auto& Group : ViewpointGroups)
	{
		if (IsCoverageSatisfied(Group) && IsDistanceSatisfied(Group))
		{
			ValidViewpointGroups.Add(Group);
		}
	}

	// 如果没有满足约束条件的视点组合,返回空数组
	if (ValidViewpointGroups.Num() == 0)
	{
		OutViewpointGroups.Empty();
		UE_LOG(LogTemp, Warning, TEXT("No valid viewpoint groups found"));
		return;
	}

	// 计算每个视点组合的美学评分
	TArray<TPair<float, TArray<FPathPointWithOrientation>>> ScoredGroups;
	for (const auto& Group : ValidViewpointGroups)
	{
		float GroupScore = CalculateGroupAestheticScore(Group);
		ScoredGroups.Emplace(GroupScore, Group);
	}

	// 按照美学评分降序排列视点组合
	ScoredGroups.Sort([](const TPair<float, TArray<FPathPointWithOrientation>>& A, const TPair<float, TArray<FPathPointWithOrientation>>& B)
		{
			return A.Key > B.Key;
		});

	// 选择美学评分最高的 NumGroups 个组合
	OutViewpointGroups.Empty();
	for (int32 i = 0; i < NumGroups && i < ScoredGroups.Num(); ++i)
	{
		OutViewpointGroups.Add(ScoredGroups[i].Value);
	}
}


void ADroneActor1::GenerateViewpointGroups(
	const TArray<FPathPointWithOrientation>& Viewpoints,
	int32 GroupSize,
	TArray<TArray<FPathPointWithOrientation>>& OutViewpointGroups)
{
	const int32 NumViewpoints = Viewpoints.Num();

	// 生成所有可能的视点组合
	UE_LOG(LogTemp, Warning, TEXT("Number of viewpoints: %d"), NumViewpoints);
	TArray<TArray<int32>> IndexCombinations;
	GenerateCombinationsHelper(NumViewpoints, GroupSize, TArray<int32>(), IndexCombinations);

	// 将索引组合转换为视点组合
	OutViewpointGroups.Reset();
	for (const auto& IndexCombination : IndexCombinations)
	{
		TArray<FPathPointWithOrientation> ViewpointGroup;
		for (int32 Index : IndexCombination)
		{
			ViewpointGroup.Add(Viewpoints[Index]);
		}
		OutViewpointGroups.Add(ViewpointGroup);
	}
}

void ADroneActor1::GenerateCombinationsHelper(
	int32 NumViewpoints,
	int32 GroupSize,
	TArray<int32> CurrentCombination,
	TArray<TArray<int32>>& OutCombinations)
{
	if (CurrentCombination.Num() == GroupSize)
	{
		OutCombinations.Add(CurrentCombination);
		return;
	}

	int32 StartIndex = CurrentCombination.Num() > 0 ? CurrentCombination.Last() + 1 : 0;
	for (int32 i = StartIndex; i < NumViewpoints; ++i)
	{
		CurrentCombination.Add(i);
		GenerateCombinationsHelper(NumViewpoints, GroupSize, CurrentCombination, OutCombinations);
		CurrentCombination.Pop();
	}
}

FVector ComputeSimpleBottomEdgeEndpoint(
	const FVector& ViewpointLocation,
	const FCylindricalInterestPoint& InterestPoint)
{
	// 兴趣区域底部平面高度
	float BottomZ = InterestPoint.BottomCenter.Z;

	// 将视点投影到与底部中心相同高度的平面上
	FVector ViewpointProjected = FVector(ViewpointLocation.X, ViewpointLocation.Y, BottomZ);

	// 计算从底部中心到投影点的水平向量
	FVector Dir = (ViewpointProjected - InterestPoint.BottomCenter);
	// 忽略垂直方向
	Dir.Z = 0.f;

	// 如果方向向量接近零（视点水平位置几乎与底部中心重合），则无法确定方向
	if (Dir.IsNearlyZero())
	{
		// 退而求其次：选择底部中心向外的某一固定方向（例如X轴正方向）
		Dir = FVector(1.f, 0.f, 0.f);
	}
	else
	{
		Dir.Normalize();
	}

	// 将方向向量乘以半径得到边缘点
	// 或许使用中心而不是底部更宽泛一些
	FVector BottomEdgePoint = InterestPoint.BottomCenter + Dir * InterestPoint.Radius;
	// 确保高度与底部一致
	BottomEdgePoint.Z = BottomZ;

	return BottomEdgePoint;
}

FVector ComputeSimpleTopEdgeEndpoint(
	const FVector& ViewpointLocation,
	const FCylindricalInterestPoint& InterestPoint)
{
	// 兴趣区域顶部平面高度
	float TopZ = InterestPoint.BottomCenter.Z+ InterestPoint.Height;

	// 将视点投影到与顶部中心相同高度的平面上
	FVector ViewpointProjected = FVector(ViewpointLocation.X, ViewpointLocation.Y, TopZ);

	// 计算从顶部中心到投影点的水平向量
	FVector Dir = (ViewpointProjected - (InterestPoint.BottomCenter+ InterestPoint.Height));
	// 忽略垂直方向
	Dir.Z = 0.f;

	// 如果方向向量接近零（视点水平位置几乎与顶部中心重合），则无法确定方向
	if (Dir.IsNearlyZero())
	{
		// 退而求其次：选择底部中心向外的某一固定方向（例如X轴正方向）
		Dir = FVector(1.f, 0.f, 0.f);
	}
	else
	{
		Dir.Normalize();
	}

	// 将方向向量乘以半径得到边缘点
	FVector TopEdgePoint = InterestPoint.BottomCenter + InterestPoint.Height + Dir * InterestPoint.Radius;
	// 确保高度与底部一致
	TopEdgePoint.Z = TopZ;

	return TopEdgePoint;
}

bool ADroneActor1::DoesViewpointSeeTopAndBottom(
	const FPathPointWithOrientation& Viewpoint,
	const FCylindricalInterestPoint& InterestPoint)
{
	// 计算兴趣区域的顶部和平面位置
	FVector BottomCenter = InterestPoint.BottomCenter;
	FVector TopPoint = BottomCenter + FVector(0.f, 0.f, InterestPoint.Height+ InterestPoint.MinSafetyDistance);

	// 计算从视点到顶部和平面的方向向量
	FVector DirToTop = (TopPoint - Viewpoint.Point).GetSafeNormal();
	FVector DirToBottomCenter = (BottomCenter - Viewpoint.Point).GetSafeNormal();

	// 获取视点的正前方方向向量
	FVector CameraForward = Viewpoint.Orientation.Vector();

	// 计算视点正前方与指向顶部/底部中心的夹角
	float AngleToTop = abs(FMath::Acos(FVector::DotProduct(CameraForward, DirToTop)) * (180.f / PI));
	float AngleToBottomCenter = abs(FMath::Acos(FVector::DotProduct(CameraForward, DirToBottomCenter)) * (180.f / PI));

	// 假设视点的垂直视场等同于其FOV的一半
	float HalfFOV = Viewpoint.FOV / 2.0f;

	// 检查视角是否同时覆盖顶部和平面中心
	if (!(AngleToTop <= HalfFOV && AngleToBottomCenter <= HalfFOV))
	{
		return false;
	}

	// 使用简化函数计算底部边缘终点，避免穿过兴趣区域内部
	FVector SimpleBottomEndpoint = ComputeSimpleBottomEdgeEndpoint(Viewpoint.Point, InterestPoint);
	FVector SimpleTopEndpoint = ComputeSimpleTopEdgeEndpoint(Viewpoint.Point, InterestPoint);

	// 设置射线检测参数
	FCollisionQueryParams QueryParams;
	QueryParams.bTraceComplex = true;
	QueryParams.AddIgnoredActor(this);

	UWorld* World = GetWorld();
	if (!World) return false;

	FHitResult HitResultTop;
	FHitResult HitResultBottom;

	// 射线检测视点到顶部
	bool bBlockedTop = World->LineTraceSingleByChannel(
		HitResultTop,
		Viewpoint.Point,
		SimpleTopEndpoint,
		ECC_Visibility,
		QueryParams
	);

	// 射线检测视点到底部边缘
	bool bBlockedBottom = World->LineTraceSingleByChannel(
		HitResultBottom,
		Viewpoint.Point,
		SimpleBottomEndpoint,
		ECC_Visibility,
		QueryParams
	);

	// 如果顶部或底部路径上有障碍物，则返回false
	if (bBlockedTop || bBlockedBottom)
	{
		return false;
	}

	// 视角覆盖且无障碍物阻挡，返回true
	return true;
}

bool ADroneActor1::DoesViewpointSeeTop(
	const FPathPointWithOrientation& Viewpoint,
	const FCylindricalInterestPoint& InterestPoint)
{
	// 计算兴趣区域的顶部和平面位置
	FVector BottomCenter = InterestPoint.BottomCenter;
	FVector TopPoint = BottomCenter + FVector(0.f, 0.f, InterestPoint.Height + InterestPoint.MinSafetyDistance);

	// 计算从视点到顶部和平面的方向向量
	FVector DirToTop = (TopPoint - Viewpoint.Point).GetSafeNormal();

	// 获取视点的正前方方向向量
	FVector CameraForward = Viewpoint.Orientation.Vector();

	// 计算视点正前方与指向顶部/底部中心的夹角
	float AngleToTop = abs(FMath::Acos(FVector::DotProduct(CameraForward, DirToTop)) * (180.f / PI));

	// 假设视点的垂直视场等同于其FOV的一半
	float HalfFOV = Viewpoint.FOV / 2.0f;

	// 检查视角是否同时覆盖顶部和平面中心
	if (!(AngleToTop <= HalfFOV ))
	{
		return false;
	}

	// 使用简化函数计算顶部边缘终点，避免穿过兴趣区域内部
	FVector SimpleTopEndpoint = ComputeSimpleTopEdgeEndpoint(Viewpoint.Point, InterestPoint);

	// 设置射线检测参数
	FCollisionQueryParams QueryParams;
	QueryParams.bTraceComplex = true;
	QueryParams.AddIgnoredActor(this);

	UWorld* World = GetWorld();
	if (!World) return false;

	FHitResult HitResultTop;

	// 射线检测视点到顶部
	bool bBlockedTop = World->LineTraceSingleByChannel(
		HitResultTop,
		Viewpoint.Point,
		SimpleTopEndpoint,
		ECC_Visibility,
		QueryParams
	);

	// 如果顶部路径上有障碍物，则返回false
	if (bBlockedTop)
	{
		return false;
	}

	// 视角覆盖且无障碍物阻挡，返回true
	return true;
}


bool ADroneActor1::IsCoverageSatisfied(const TArray<FPathPointWithOrientation>& ViewpointGroup)
{
	if (ViewpointGroup.Num() == 0)
	{
		return false;
	}

	// 假设所有视点围绕同一兴趣区域，取第一个视点的AOIIndex
	int32 targetAOIIndex = ViewpointGroup[0].AOIIndex;
	if (!InterestPoints.IsValidIndex(targetAOIIndex))
	{
		return false;
	}
	const FCylindricalInterestPoint& TargetInterestPoint = InterestPoints[targetAOIIndex];

	// --- 水平覆盖检查 ---
	// 创建一个数组来存储所有视点的覆盖范围
	TArray<FAngleRange> CoverageRanges;

	// 将每个视点的覆盖范围添加到数组中
	for (const FPathPointWithOrientation& Viewpoint : ViewpointGroup)
	{
		CoverageRanges.Add(Viewpoint.AngleRange);
	}

	// 对覆盖范围进行排序,按照下界从小到大排序
	CoverageRanges.Sort([](const FAngleRange& A, const FAngleRange& B) {
		return A.LowerBound < B.LowerBound;
		});

	// 合并重叠的覆盖范围
	TArray<FAngleRange> MergedRanges;
	for (const FAngleRange& Range : CoverageRanges)
	{
		if (MergedRanges.Num() == 0 || Range.LowerBound > MergedRanges.Last().UpperBound)
		{
			MergedRanges.Add(Range);
		}
		else
		{
			MergedRanges.Last().UpperBound = FMath::Max(MergedRanges.Last().UpperBound, Range.UpperBound);
		}
	}
	bool bHorizontalCoverageSatisfied = (MergedRanges.Num() == 1 &&
		MergedRanges[0].UpperBound - MergedRanges[0].LowerBound >= 360.0f);

	// --- 垂直覆盖检查 ---
	int VerticalCoverageCount = 0;
	for (const FPathPointWithOrientation& Viewpoint : ViewpointGroup)
	{
		if (DoesViewpointSeeTopAndBottom(Viewpoint, TargetInterestPoint))
		{
			VerticalCoverageCount++;
			if (VerticalCoverageCount >= 1)
			{
				break;
			}
		}

	}
	bool bVerticalCoverageSatisfied = (VerticalCoverageCount >= 1);

	return bHorizontalCoverageSatisfied && bVerticalCoverageSatisfied;

}

bool ADroneActor1::IsDistanceSatisfied(const TArray<FPathPointWithOrientation>& ViewpointGroup)
{
	// 获取兴趣区域的半径和最小安全距离
	float InterestRadius = InterestPoints[0].Radius;
	float MinSafetyDistance = InterestPoints[0].MinSafetyDistance;

	// 计算最小距离阈值
	float MinDistanceThreshold = InterestRadius;

	// 检查每对视点之间的距离是否大于等于最小距离阈值
	for (int32 i = 0; i < ViewpointGroup.Num() - 1; ++i)
	{
		for (int32 j = i + 1; j < ViewpointGroup.Num(); ++j)
		{
			float Distance = FVector::Dist(ViewpointGroup[i].Point, ViewpointGroup[j].Point);
			if (Distance < MinDistanceThreshold)
			{
				//UE_LOG(LogTemp, Warning, TEXT("Distance not satisfied"));
				return false;
			}
		}
	}

	//UE_LOG(LogTemp, Warning, TEXT("Distance satisfied"));
	return true;
}

float ADroneActor1::CalculateGroupAestheticScore(const TArray<FPathPointWithOrientation>& ViewpointGroup)
{
	// 计算视点组合的总美学评分
	float TotalAestheticScore = 0.0f;
	for (const FPathPointWithOrientation& Viewpoint : ViewpointGroup)
	{
		TotalAestheticScore += Viewpoint.AestheticScore;
	}

	// 返回平均美学评分
	return TotalAestheticScore / ViewpointGroup.Num();
}


// ----------------------------
// 曲线工具函数
void ADroneActor1::SortViewpointsInWrappingOrder(TArray<FPathPointWithOrientation>& Viewpoints)
{
	if (Viewpoints.Num() < 3) return;

	// 计算视点的中心点
	FVector _CenterPoint = FVector::ZeroVector;
	for (const FPathPointWithOrientation& Viewpoint : Viewpoints)
	{
		_CenterPoint += Viewpoint.Point;
	}
	_CenterPoint /= Viewpoints.Num();

	// 根据视点相对于中心点的角度进行排序
	Viewpoints.Sort([_CenterPoint](const FPathPointWithOrientation& A, const FPathPointWithOrientation& B)
		{
			FVector DirA = A.Point - _CenterPoint;
			FVector DirB = B.Point - _CenterPoint;
			float AngleA = FMath::Atan2(DirA.Y, DirA.X);
			float AngleB = FMath::Atan2(DirB.Y, DirB.X);
			return AngleA < AngleB;
		});

	// 检查是否存在高度由大到小的情况,如果是,则反转顺序
	bool bReverseOrder = false;
	for (int32 i = 1; i < Viewpoints.Num(); ++i)
	{
		if (Viewpoints[i].Point.Z < Viewpoints[i - 1].Point.Z)
		{
			bReverseOrder = true;
			break;
		}
	}

	if (bReverseOrder)
	{
		Algo::Reverse(Viewpoints);
	}
}

void ADroneActor1::SortControlPointsInWrappingOrder(TArray<FPathPointWithOrientation>& ControlPoints)
{
	if (ControlPoints.Num() < 3) return;

	// 计算控制点的中心点
	FVector _CenterPoint = FVector::ZeroVector;
	for (const FPathPointWithOrientation& ControlPoint : ControlPoints)
	{
		_CenterPoint += ControlPoint.Point;
	}
	_CenterPoint /= ControlPoints.Num();

	// 根据控制点相对于中心点的角度进行排序
	ControlPoints.Sort([_CenterPoint](const FPathPointWithOrientation& A, const FPathPointWithOrientation& B)
		{
			FVector DirA = A.Point - _CenterPoint;
			FVector DirB = B.Point - _CenterPoint;
			float AngleA = FMath::Atan2(DirA.Y, DirA.X);
			float AngleB = FMath::Atan2(DirB.Y, DirB.X);
			return AngleA < AngleB;
		});
}


FPathPointWithOrientation ADroneActor1::GenerateControlPointTowardsNext(
	const FPathPointWithOrientation& CurrentPoint, const FPathPointWithOrientation& NextPoint, 
	const FVector _PlaneNormal, float ControlPointDistanceRatio)
{
	FPathPointWithOrientation ControlPoint;

	// 获取当前点所属的兴趣区域
	const FCylindricalInterestPoint& AOI = InterestPoints[CurrentPoint.AOIIndex];

	// 计算当前点到兴趣区域中心的向量
	FVector CurrentToAOICenter = AOI.Center - CurrentPoint.Point;
	CurrentToAOICenter.Normalize();

	// 计算位于共面平面上且相切的向量 朝向下一个点
	FVector TangentDirection = FVector::CrossProduct(_PlaneNormal, CurrentToAOICenter);
	TangentDirection.Normalize();

	// 计算控制点的位置
	float Distance = FVector::Distance(CurrentPoint.Point, NextPoint.Point);
	ControlPoint.Point = CurrentPoint.Point + TangentDirection * (Distance * ControlPointDistanceRatio);

	// 设置控制点的初始方向为朝向兴趣区域中心
	ControlPoint.Orientation = FRotationMatrix::MakeFromX(CurrentToAOICenter).Rotator();

	// 获取最优视点
	FPathPointWithOrientation BestViewpoint = GenerateCandidateViewpoints(ControlPoint,
		ControlPoint.Point,
		ControlPoint.Orientation,
		CurrentPoint.AOIIndex
	);

	// 更新控制点的方向为最优视点方向
	ControlPoint.Orientation = BestViewpoint.Orientation;

	ControlPoint.FOV = CurrentPoint.FOV;
	ControlPoint.AOIIndex = CurrentPoint.AOIIndex;
	ControlPoint.AestheticScore = CurrentPoint.AestheticScore;
	ControlPoint.CoverageAngle = CurrentPoint.CoverageAngle;
	return ControlPoint;
}

FPathPointWithOrientation ADroneActor1::GenerateControlPointTowardsPrev(
	const FPathPointWithOrientation& CurrentPoint, const FPathPointWithOrientation& PrevPoint, 
	const FVector _PlaneNormal, float ControlPointDistanceRatio)
{
	FPathPointWithOrientation ControlPoint;

	// 获取当前点所属的兴趣区域
	const FCylindricalInterestPoint& AOI = InterestPoints[CurrentPoint.AOIIndex];

	// 计算当前点到兴趣区域中心的向量
	FVector CurrentToAOICenter = AOI.Center - CurrentPoint.Point;
	CurrentToAOICenter.Normalize();

	// 计算位于共面平面上且相切的向量 朝向前一个点
	FVector TangentDirection = FVector::CrossProduct(CurrentToAOICenter, _PlaneNormal);
	TangentDirection.Normalize();

	// 计算控制点的位置
	float Distance = FVector::Distance(CurrentPoint.Point, PrevPoint.Point);
	ControlPoint.Point = CurrentPoint.Point + TangentDirection * (Distance * ControlPointDistanceRatio);

	// 设置控制点的初始方向为朝向兴趣区域中心
	ControlPoint.Orientation = FRotationMatrix::MakeFromX(CurrentToAOICenter).Rotator();

	// 获取最优视点
	FPathPointWithOrientation BestViewpoint = GenerateCandidateViewpoints(ControlPoint,
		ControlPoint.Point,
		ControlPoint.Orientation,
		CurrentPoint.AOIIndex
	);

	// 更新控制点的方向为最优视点方向
	ControlPoint.Orientation = BestViewpoint.Orientation;

	ControlPoint.FOV = CurrentPoint.FOV;
	ControlPoint.AOIIndex = CurrentPoint.AOIIndex;
	ControlPoint.AestheticScore = CurrentPoint.AestheticScore;
	ControlPoint.CoverageAngle = CurrentPoint.CoverageAngle;
	return ControlPoint;
}


// ----------------------------
// 生成B样条曲线路径点 返回控制点
TArray<FPathPointWithOrientation> ADroneActor1::GenerateSplinePath(
	const TArray<FPathPointWithOrientation>& BestViewpoints,
	TArray<FPathPointWithOrientation>& OutSplinePoints,
	float MinDisBetwPoints)
{
	// 检查输入点数量
	if (BestViewpoints.Num() < 3) return TArray<FPathPointWithOrientation>();

	// 清空输出数组
	OutSplinePoints.Empty();

	// 为五阶B样条曲线准备控制点(需要至少6个控制点)
	TArray<FPathPointWithOrientation> ControlPoints;

	// 确保最佳视点按照环绕顺序排列
	TArray<FPathPointWithOrientation> SortedViewpoints = BestViewpoints;
	SortViewpointsInWrappingOrder(SortedViewpoints);

	// 计算三个点所在平面的法向量
	FVector CurrentToPrev = SortedViewpoints[0].Point - SortedViewpoints[1].Point;
	FVector CurrentToNext = SortedViewpoints[2].Point - SortedViewpoints[1].Point;
	FVector PlaneNormal = FVector::CrossProduct(CurrentToPrev, CurrentToNext);
	PlaneNormal.Normalize();

	// 根据排序后的视点生成控制点
	for (int32 i = 0; i < SortedViewpoints.Num(); ++i)
	{
		const FPathPointWithOrientation& CurrentPoint = SortedViewpoints[i];
		

		if (i == 0)
		{
			// 起点,朝向下一个点生成一个控制点
			const FPathPointWithOrientation& NextPoint = SortedViewpoints[i + 1];
			FPathPointWithOrientation ControlPoint = GenerateControlPointTowardsNext(CurrentPoint, NextPoint, PlaneNormal);
			ControlPoints.Add(CurrentPoint);
			ControlPoints.Add(ControlPoint);
		}
		else if (i == SortedViewpoints.Num() - 1)
		{
			// 终点,朝向前一个点生成一个控制点
			const FPathPointWithOrientation& PrevPoint = SortedViewpoints[i - 1];
			FPathPointWithOrientation ControlPoint = GenerateControlPointTowardsPrev(CurrentPoint, PrevPoint, PlaneNormal);
			ControlPoints.Add(ControlPoint);
			ControlPoints.Add(CurrentPoint);
		}
		else
		{
			// 中间点,朝向前一个点和下一个点各生成一个控制点
			const FPathPointWithOrientation& PrevPoint = SortedViewpoints[i - 1];
			const FPathPointWithOrientation& NextPoint = SortedViewpoints[i + 1];
			FPathPointWithOrientation ControlPoint1 = GenerateControlPointTowardsPrev(CurrentPoint, PrevPoint, PlaneNormal);
			FPathPointWithOrientation ControlPoint2 = GenerateControlPointTowardsNext(CurrentPoint, NextPoint, PlaneNormal);
			ControlPoints.Add(ControlPoint1);
			ControlPoints.Add(CurrentPoint);
			ControlPoints.Add(ControlPoint2);
		}
	}

	// 其实没有必要
	// 对生成的控制点进行排序
	//SortControlPointsInWrappingOrder(ControlPoints);

	// 先添加进去测试
	// GlobalPathPoints=(ControlPoints);

	// 生成B样条曲线点
	// 计算路径的总长度
	float TotalLength = 0.0f;
	for (int32 i = 1; i < BestViewpoints.Num(); ++i) {
		TotalLength += FVector::Dist(BestViewpoints[i - 1].Point, BestViewpoints[i].Point);
	}

	// 根据总长度分配NumSplinePoints
	const int32 NumSplinePoints = FMath::Max(10, FMath::RoundToInt(TotalLength / 200.0f)); // 例如，每100单位长度分配一个点

	// 创建节点向量
	TArray<float> KnotVector;
	const int32 Order = 5;
	const int32 NumKnots = ControlPoints.Num() + Order;

	// 基于弦长参数化(Chord Length Parameterization)的节点向量计算
	// 计算控制点之间的距离
	TArray<float> Distances;
	Distances.Add(0.0f);

	for (int32 i = 1; i < ControlPoints.Num(); ++i)
	{
		float Distance = FVector::Dist(ControlPoints[i].Point, ControlPoints[i - 1].Point);
		Distances.Add(Distance);
	}

	// 计算总距离
	float TotalDistance = 0.0f;
	for (int32 i = 0; i < Distances.Num(); ++i)
	{
		TotalDistance += Distances[i];
	}

	// 根据弦长参数计算参数值 U
	TArray<float> U;
	U.SetNum(ControlPoints.Num());
	U[0] = 0.0f;

	float AccumulatedDistance = 0.0f;
	for (int32 i = 1; i < ControlPoints.Num(); ++i)
	{
		AccumulatedDistance += Distances[i];
		U[i] = AccumulatedDistance / TotalDistance;
	}

	// 构建节点向量
	KnotVector.Empty();

	int32 NumCP = ControlPoints.Num();  // 控制点数为 n+1
	int32 p = Order - 1;                // 阶 p = Order - 1
	int32 m = NumCP - 1;                // m = n，n为最后一个控制点索引

	// 前 p+1 个节点为 0
	for (int32 i = 0; i <= p; ++i)
	{
		KnotVector.Add(0.0f);
	}

	// 中间节点计算
	// i=1,...,m-p:
	// t_{p+i} = (u_i + u_{i+1} + ... + u_{i+p-1}) / p
	for (int32 i = 1; i <= m - p; ++i)
	{
		float sum = 0.0f;
		for (int32 j = i; j <= i + p - 1; ++j)
		{
			sum += U[j];
		}
		float avg = sum / p;
		KnotVector.Add(avg);
	}

	// 后 p+1 个节点为 1
	for (int32 i = 0; i <= p; ++i)
	{
		KnotVector.Add(1.0f);
	}

	// 接下来使用新的KnotVector和CalculateBSplineBasis来计算样条点
	for (int32 i = 0; i < NumSplinePoints; ++i)
	{
		float t = float(i) / float(NumSplinePoints - 1);
		FPathPointWithOrientation SplinePoint;

		SplinePoint.Point = FVector::ZeroVector;

		// 计算B样条基函数并应用控制点加权求和
		for (int32 j = 0; j < ControlPoints.Num(); ++j)
		{
			float Weight = CalculateBSplineBasis(j, Order, t, KnotVector);
			SplinePoint.Point += ControlPoints[j].Point * Weight;
		}

		// 插值方向和FOV等属性
		InterpolatePathPointProperties(SplinePoint, ControlPoints, t, KnotVector);

		OutSplinePoints.Add(SplinePoint);
	}

	UE_LOG(LogTemp, Warning, TEXT("Spline points generated: %d"), OutSplinePoints.Num());

	// 基于距离简化样条点
	TArray<FPathPointWithOrientation> SimplifiedSplinePoints;
	//float MinDisBetwPoints = 50.0f; // 最小点间距离，您可以调整为适合需求的值
	float LocalAccumulatedDistance = 0.0f;
	FVector LastPoint = OutSplinePoints[0].Point; // 上一个保留的点

	// 起点加入简化结果
	SimplifiedSplinePoints.Add(OutSplinePoints[0]);

	// 遍历生成的样条点
	for (int32 i = 1; i < OutSplinePoints.Num(); ++i)
	{
		const FPathPointWithOrientation& CurrentPoint = OutSplinePoints[i];

		// 计算当前点与上一个保留点之间的距离
		float SegmentDistance = FVector::Dist(LastPoint, CurrentPoint.Point);
		LocalAccumulatedDistance += SegmentDistance;

		// 如果累积距离大于等于最小点间距离，记录当前点
		if (LocalAccumulatedDistance >= MinDisBetwPoints)
		{
			SimplifiedSplinePoints.Add(CurrentPoint);

			// 重置累积距离，并更新上一个保留点
			LocalAccumulatedDistance = 0.0f;
			LastPoint = CurrentPoint.Point;
		}
	}

	// 确保终点加入结果
	if (SimplifiedSplinePoints.Last().Point != OutSplinePoints.Last().Point)
	{
		SimplifiedSplinePoints.Add(OutSplinePoints.Last());
	}
	// 将结果覆盖到 OutSplinePoints
	OutSplinePoints = SimplifiedSplinePoints;
	UE_LOG(LogTemp, Warning, TEXT("Simplified spline points: %d"), OutSplinePoints.Num());


	// 异步计算美学评分
	FThreadSafeCounter TaskCounter(OutSplinePoints.Num());
	for (FPathPointWithOrientation& Viewpoint : OutSplinePoints)
	{
		AsyncTask(ENamedThreads::GameThread, [this, &Viewpoint, &TaskCounter]()
			{
				RenderViewpointToRenderTarget(Viewpoint);
				if (NimaTracker && RenderTarget)
				{
					NimaTracker->RunInference(RenderTarget);
				}

				while (NimaTracker->GetNimaScore() <= 0)
				{
					FPlatformProcess::Sleep(0.001f);
				}

				Viewpoint.AestheticScore = NimaTracker->GetNimaScore();
				NimaTracker->ResetNimaScore();

				if (Viewpoint.AOIIndex >= 0 && Viewpoint.AOIIndex < InterestPoints.Num())
				{
					Viewpoint.CoverageAngle = CalculateViewpointCoverage(Viewpoint, InterestPoints[Viewpoint.AOIIndex]);
				}
				else
				{
					Viewpoint.CoverageAngle = 0.0f;
				}

				TaskCounter.Decrement();
			});

		while (Viewpoint.AestheticScore <= 0)
		{
			FPlatformProcess::Sleep(0.001f);
		}
	}

	while (TaskCounter.GetValue() > 0)
	{
		FPlatformProcess::Sleep(0.001f);
	}

	return ControlPoints;
}


float ADroneActor1::CalculateBSplineBasis(int32 i, int32 k, float t, const TArray<float>& KnotVector)
{
	if (k == 1)
	{
		return (t >= KnotVector[i] && t <= KnotVector[i + 1]) ? 1.0f : 0.0f;
	}

	float d1 = 0.0f;
	float d2 = 0.0f;

	if (KnotVector[i + k - 1] != KnotVector[i])
	{
		d1 = (t - KnotVector[i]) / (KnotVector[i + k - 1] - KnotVector[i]);
	}

	if (KnotVector[i + k] != KnotVector[i + 1])
	{
		d2 = (KnotVector[i + k] - t) / (KnotVector[i + k] - KnotVector[i + 1]);
	}

	return d1 * CalculateBSplineBasis(i, k - 1, t, KnotVector) +
		d2 * CalculateBSplineBasis(i + 1, k - 1, t, KnotVector);
}


void ADroneActor1::InterpolatePathPointProperties(FPathPointWithOrientation& SplinePoint,
	const TArray<FPathPointWithOrientation>& ControlPoints, float t, const TArray<float>& U)
{
	// 找到 t 值所在的控制点区间
	int32 Index = 0;
	for (int32 i = 0; i < ControlPoints.Num() - 1; ++i)
	{
		if (t >= U[i] && t <= U[i + 1])
		{
			Index = i;
			break;
		}
	}

	// 计算区间内的相对 t 值
	float USpan = U[Index + 1] - U[Index];
	float LocalT = (USpan > 0.0f) ? (t - U[Index]) / USpan : 0.0f;

	// 获取相机位置和兴趣区域位置
	FVector CameraLocation = SplinePoint.Point;
	FVector AOILocation = InterestPoints[ControlPoints[Index].AOIIndex].Center;

	// 计算物体在前一个控制点和后一个控制点的屏幕位置
	FVector2D PrevScreenPosition = CalculateScreenPosition(ControlPoints[Index]);
	FVector2D NextScreenPosition = CalculateScreenPosition(ControlPoints[Index + 1]);

	// 插值物体在屏幕上的位置
	FVector2D InterpolatedScreenPosition = FMath::Lerp(PrevScreenPosition, NextScreenPosition, LocalT);

	// 根据插值后的屏幕位置计算相机旋转
	FRotator InterpolatedOrientation = CalculateOrientationFromScreenPosition(
		CameraLocation, AOILocation, InterpolatedScreenPosition, SplinePoint.FOV);
	SplinePoint.Orientation = InterpolatedOrientation;

	// 插值其他属性
	SplinePoint.FOV = FMath::Lerp(ControlPoints[Index].FOV, ControlPoints[Index + 1].FOV, LocalT);
	// 之后统一计算而不是插值
	/*SplinePoint.AestheticScore = FMath::Lerp(ControlPoints[Index].AestheticScore, ControlPoints[Index + 1].AestheticScore, LocalT);*/
	/*SplinePoint.CoverageAngle = FMath::Lerp(ControlPoints[Index].CoverageAngle, ControlPoints[Index + 1].CoverageAngle, LocalT);*/
	SplinePoint.AOIIndex = ControlPoints[Index].AOIIndex; // AOI索引不需要插值
}



FVector2D ADroneActor1::CalculateScreenPosition(const FPathPointWithOrientation& PathPoint)
{
	// 获取PathPoint关联的兴趣区域
	const FCylindricalInterestPoint& AOI = InterestPoints[PathPoint.AOIIndex];

	// 计算相机位置和旋转
	FVector CameraLocation = PathPoint.Point;
	FRotator CameraRotation = PathPoint.Orientation;

	// 将兴趣区域中心投影到屏幕空间
	FVector2D ScreenPosition;
	bool bProjected = ProjectWorldPointToScreen(AOI.Center, ScreenPosition, CameraLocation, CameraRotation, PathPoint.FOV);

	if (bProjected)
	{
	}
	else
	{
		// 如果投影失败,将屏幕位置设置为(0.5, 0.5)
		ScreenPosition = FVector2D(0.0f, 0.0f);
	}

	return ScreenPosition;
}

FRotator ADroneActor1::CalculateOrientationFromScreenPosition(
	const FVector& CameraLocation, const FVector& AOILocation, const FVector2D& ScreenPosition, float FOV)
{
	// 定义焦距
	const float FocalLength = 1.0f / FMath::Tan(FMath::DegreesToRadians(FOV / 2.0f));
	const float AspectRatio = CameraComponent->AspectRatio;
	FVector Direction = AOILocation - CameraLocation;
	FRotator _BaseRotation = Direction.Rotation();

	// 计算 Pitch 和 Yaw 的偏移
	float PitchOffset = -FMath::RadiansToDegrees(FMath::Atan2(ScreenPosition.Y, FocalLength));
	float YawOffset = -FMath::RadiansToDegrees(FMath::Atan2(ScreenPosition.X* AspectRatio, FocalLength));
	FRotator _TargetRotation = _BaseRotation + FRotator(PitchOffset, YawOffset, 0.0f);

	return _TargetRotation;
}



bool ADroneActor1::IsPathCollisionFree(const TArray<FPathPointWithOrientation>& Path)
{
	FCollisionQueryParams QueryParams;
	QueryParams.AddIgnoredActor(this);

	for (int32 i = 0; i < Path.Num() - 1; ++i)
	{
		FHitResult HitResult;
		FVector Start = Path[i].Point;
		FVector End = Path[i + 1].Point;

		// 使用 SweepSingleByChannel 检测碰撞
		bool bHit = GetWorld()->SweepSingleByChannel(
			HitResult,
			Start,
			End,
			FQuat::Identity,
			ECC_WorldStatic, // 检测静态物体
			FCollisionShape::MakeSphere(DroneRadius),
			QueryParams
		);

		if (!bHit)
		{
			bHit = GetWorld()->SweepSingleByChannel(
				HitResult,
				Start,
				End,
				FQuat::Identity,
				ECC_WorldDynamic, // 检测动态物体
				FCollisionShape::MakeSphere(DroneRadius),
				QueryParams
			);
		}

		if (bHit && HitResult.GetActor() != this)
		{
			return false;
		}
	}

	return true;
}


float ADroneActor1::CalculateLengthCost(const TArray<FPathPointWithOrientation>& Path, float MaxLength)
{
	float TotalLength = 0.0f;
	for (int32 i = 0; i < Path.Num() - 1; ++i)
	{
		TotalLength += FVector::Dist(Path[i].Point, Path[i + 1].Point);
	}
	// 使用候选路径中的最大长度作为归一化因子
	float NormalizedLength = TotalLength / MaxLength;
	return NormalizedLength * LengthCostWeight;
}


float ADroneActor1::CalculateQualityCost(const TArray<FPathPointWithOrientation>& Path)
{
	float TotalScore = 0.0f;
	for (const auto& Point : Path)
	{
		TotalScore += Point.AestheticScore;
	}
	float AverageScore = TotalScore / Path.Num();
	// 使用归一化的平均美学评分作为代价
	float NormalizedQuality = 1.0f - (AverageScore / MaxAestheticScore);
	return NormalizedQuality * QualityCostWeight;
}


float ADroneActor1::CalculateSmoothnessCost(const TArray<FPathPointWithOrientation>& Path)
{
	float TotalCurvature = 0.0f;
	for (int32 i = 1; i < Path.Num() - 1; ++i)
	{
		FVector Segment1 = Path[i].Point - Path[i - 1].Point;
		FVector Segment2 = Path[i + 1].Point - Path[i].Point;
		float Angle = FMath::Acos(FVector::DotProduct(Segment1.GetSafeNormal(), Segment2.GetSafeNormal()));
		TotalCurvature += Angle;
	}
	// 使用归一化的总曲率作为代价
	float NormalizedCurvature = TotalCurvature / (PI * (Path.Num() - 2));
	return NormalizedCurvature * SmoothnessCostWeight;
}

double ADroneActor1::ComputeTransiPathCost(
	const TArray<FPathPointWithOrientation>& Path,
	int startIndex,
	int targetIndex,
	const FVector& PrevMidPoint,    // 上一条局部航线的中点
	const FVector& NextMidPoint     // 下一条局部航线的中点
)
{
	float Alpha = 1.0f;

	if (Path.Num() < 2)
	{
		return TNumericLimits<double>::Max();
	}

	// 当前过渡航线的起点和终点
	FVector s = Path[0].Point;
	FVector m_prime = Path.Last().Point;

	// 取得当前过渡路径的中间点
	int32 MidIndex = Path.Num() / 2;
	FVector w = Path[MidIndex].Point; // 当前过渡航线的中间点

	// --- 计算各项代价 ---

	// 1. 路径长度 E_length
	float TotalLength = 0.0f;
	for (int32 i = 0; i < Path.Num() - 1; ++i)
	{
		TotalLength += FVector::Dist(Path[i].Point, Path[i + 1].Point);
	}
	float SafeLength = FMath::Max(TotalLength, KINDA_SMALL_NUMBER);
	float E_length = Alpha * TotalLength; // Alpha为类成员变量或预先定义的常量

	// 2. 视图质量代价 E_quality
	float TotalScore = 0.0f;
	for (const auto& P : Path)
	{
		TotalScore += P.AestheticScore;
	}
	float AvgScore = (Path.Num() > 0) ? (TotalScore / Path.Num()) : 0.0f;
	float E_quality = 1.0f - AvgScore; // 假设AestheticScore在[0,1]

	// 3. 相机总视角变化代价 E_rot
	FVector q_s = Path[0].Orientation.Vector().GetSafeNormal();
	FVector q_e = Path.Last().Orientation.Vector().GetSafeNormal();
	float dot_rot = FVector::DotProduct(q_s, q_e);
	float E_rot = (1.0f / SafeLength) * (1.0f - dot_rot);

	// 4. 转弯代价 E_turn
	// 根据论文的定义：
	// d_m: (s - PrevMidPoint).GetSafeNormal()
	// d_mt: (w - s).GetSafeNormal()
	// d_tm': (NextMidPoint - w).GetSafeNormal()
	// d_m': (m_prime - NextMidPoint).GetSafeNormal()
	FVector d_m = (s - PrevMidPoint).GetSafeNormal();
	FVector d_mt = (w - s).GetSafeNormal();
	FVector d_tm_ = (NextMidPoint - w).GetSafeNormal();
	FVector d_m_ = (m_prime - NextMidPoint).GetSafeNormal();

	float E_turn = 0.25f * (2.0f - FVector::DotProduct(d_m, d_mt) - FVector::DotProduct(d_tm_, d_m_));

	// 5. 高度差代价 E_height
	float E_height = Path.Last().Point.Z - Path[0].Point.Z;

	// 总代价
	double E_trans = 0.001 * E_length + E_quality + 10000 * abs(E_rot) + 5 * abs(E_turn) + 0.001 * abs(E_height);

	return E_trans;
}


TArray<float> ADroneActor1::ParameterizePath(const TArray<FPathPointWithOrientation>& Path)
{
	TArray<float> U;
	U.SetNum(Path.Num());
	U[0] = 0.0f;

	for (int32 i = 1; i < Path.Num(); ++i)
	{
		float SegmentLength = FVector::Distance(Path[i].Point, Path[i - 1].Point);
		U[i] = U[i - 1] + SegmentLength;
	}

	return U;
}


bool ADroneActor1::IsEntryNode(int32 NodeIndex) const
{
	return (NodeIndex >= 2) && ((NodeIndex % 2) == 0);
}

bool ADroneActor1::IsExitNode(int32 NodeIndex) const
{
	return (NodeIndex >= 2) && ((NodeIndex % 2) == 1);
}

// 定义一个函数，接受一个“当创建完后要执行的回调”
//void RequestCreateRRTClassAsync(TFunction<void(UMyRRTClass*)> OnCreated)
//{
//	// 当前处于异步线程/后台线程里
//	// 切回游戏线程执行真正的 NewObject
//	AsyncTask(ENamedThreads::GameThread, [OnCreated]()
//		{
//			UMyRRTClass* RRTClass = NewObject<UMyRRTClass>();
//			// 进行初始化等操作
//			RRTClass->SetWorld(GEngine->GetWorldFromContextObjectChecked(GWorld));
//
//			// 创建完成后，调用回调，传出指针
//			if (OnCreated)
//			{
//				OnCreated(RRTClass);
//			}
//		});
//}

bool ADroneActor1::BuildAndProcessPathSegment(
	const FVector& StartPos,
	const FVector& EndPos,
	int32 StartRegionIndex,
	int32 TargetRegionIndex,
	double& OutCost,
	TArray<FPathPointWithOrientation>& OutPath,
	UMyRRTClass* LocalRRTClass
)
{

	FVector PrevMidPoint = FVector::ZeroVector;
	FVector NextMidPoint = FVector::ZeroVector;

	// 从之前的路径和下一段路径延申的额外点，用于作为额外控制点
	FVector PrevExtraPoint = FVector::ZeroVector;
	FVector NextExtraPoint = FVector::ZeroVector;

	// 如果ComputeTransiPathCost需要上下文（如转弯代价）
	// 根据StartRegionIndex，从InterestAreas中获取PrevAreaPath
	if (InterestAreas.IsValidIndex(StartRegionIndex))
	{
		TArray<FPathPointWithOrientation> PrevAreaPath = InterestAreas[StartRegionIndex].PathPoints;
		// 检查方向，如果EndPos距离PrevAreaPath末尾更近，则反转
		float DistToFirst = FVector::Dist(StartPos, PrevAreaPath[0].Point);
		float DistToLast = FVector::Dist(StartPos, PrevAreaPath.Last().Point);
		if (DistToLast > DistToFirst)
		{
			Algo::Reverse(PrevAreaPath);
		}
		int32 MidIdx = PrevAreaPath.Num() / 2;
		PrevMidPoint = PrevAreaPath[MidIdx].Point;

		// 从PrevAreaPath获取额外控制点
		FVector LastNormal = (PrevAreaPath.Last().Point - PrevAreaPath[PrevAreaPath.Num() - 2].Point).GetSafeNormal();
		PrevExtraPoint = PrevAreaPath.Last().Point + LastNormal * 100.0f;
	}

	// 对TargetRegionIndex获取下一路径中点
	if (InterestAreas.IsValidIndex(TargetRegionIndex))
	{
		TArray<FPathPointWithOrientation> NextAreaPath = InterestAreas[TargetRegionIndex].PathPoints;
		float DistToFirst = FVector::Dist(EndPos, NextAreaPath[0].Point);
		float DistToLast = FVector::Dist(EndPos, NextAreaPath.Last().Point);
		if (DistToLast < DistToFirst)
		{
			Algo::Reverse(NextAreaPath);
		}
		int32 MidIdx = NextAreaPath.Num() / 2;
		NextMidPoint = NextAreaPath[MidIdx].Point;

		// 类似的，从NextAreaPath获取额外控制点
		FVector FirstNormal = (NextAreaPath[0].Point - NextAreaPath[1].Point).GetSafeNormal(); // 逆向
		NextExtraPoint = NextAreaPath[0].Point + FirstNormal * 100.0f;
	}

	//TFuture<UMyRRTClass*> MyFuture;
	//{
	//	// 创建Promise
	//	TSharedRef<TPromise<UMyRRTClass*>> Promise = MakeShared<TPromise<UMyRRTClass*>>();
	//	// 拿到对应的Future
	//	MyFuture = Promise->GetFuture();

	//	// 在异步线程里发起请求
	//	AsyncTask(ENamedThreads::GameThread, [Promise]()
	//		{
	//			// 切到主线程执行NewObject
	//			UMyRRTClass* RRTClass = NewObject<UMyRRTClass>();
	//			RRTClass->SetWorld(GEngine->GetWorldFromContextObjectChecked(GWorld));

	//			// 通知Promise
	//			Promise->SetValue(RRTClass);
	//		});
	//}

	//// 这里还是在当前异步线程
	//// 等待或轮询
	//UMyRRTClass* RRTClass = MyFuture.Get(); // Get()会阻塞直到SetValue被调用
	//RRTClass->SetWorld(GetWorld()); // 设置世界 必须！

	TArray<FVector> LocalOptimizedPath;
	TArray<FPathPointWithOrientation> RRTPath = LocalRRTClass->GenerateAndSmoothRRTPath(
		StartPos,
		EndPos,
		InterestPoints,
		LocalOptimizedPath,
		PrevExtraPoint,
		NextExtraPoint,
		fMinDisBetwenPoints
	);
	// 添加到测试路径点集
	/*if (!LocalOptimizedPath.IsEmpty()) {
		GlobalBestSplinePoints.Append(LocalOptimizedPath);
	}*/


	if (RRTPath.Num() == 0)
	{
		OutCost = TNumericLimits<double>::Max();
		return false;
	}

	// 路径参数化
	TArray<float> U = ParameterizePath(RRTPath);
	float TotalLength = U.Last();
	float HalfLength = TotalLength * 0.5f;
	int32 MidKeyIndex = 0;
	for (int32 i = 0; i < U.Num(); ++i)
	{
		if (U[i] >= HalfLength)
		{
			MidKeyIndex = i;
			break;
		}
	}

	TArray<int32> KeyIndices;
	KeyIndices.Add(0);
	if (MidKeyIndex != 0 && MidKeyIndex != RRTPath.Num() - 1)
	{
		KeyIndices.Add(MidKeyIndex);
	}
	KeyIndices.Add(RRTPath.Num() - 1);

	int32 AOIIndex = -1;
	if (TargetRegionIndex >= 0 && TargetRegionIndex < InterestPoints.Num())
	{
		AOIIndex = TargetRegionIndex;
	}
	else if (TargetRegionIndex == -1) {
		//AOIIndex = InterestPoints.Num() - 1; // 最后一个兴趣区域
		AOIIndex = StartRegionIndex; // 保持不变

	}

	// 为关键点选最佳视点
	TArray<FPathPointWithOrientation> ControlPoints;
	for (int32 KeyIdx : KeyIndices)
	{
		FPathPointWithOrientation& BasePoint = RRTPath[KeyIdx];

		FRotator OriginalRotation = FRotator::ZeroRotator;
		if (AOIIndex >= 0)
		{
			FVector AOICenter = InterestPoints[AOIIndex].Center;
			FVector ToAOI = AOICenter - BasePoint.Point;
			OriginalRotation = ToAOI.Rotation();
		}
		else
		{
			if (KeyIdx < RRTPath.Num() - 1)
			{
				FVector Dir = RRTPath[KeyIdx + 1].Point - RRTPath[KeyIdx].Point;
				OriginalRotation = Dir.Rotation();
			}
		}

		TArray<FCandidateViewpoint> AllCandidates;
		FPathPointWithOrientation BestViewpoint = GenerateCandidateViewpoints(
			BasePoint,
			BasePoint.Point,
			OriginalRotation,
			AOIIndex,
			&AllCandidates
		);

		BasePoint = BestViewpoint;
		ControlPoints.Add(BasePoint);
	}

	auto InterpolateSegment = [&](int32 StartControlIdx, int32 EndControlIdx, int32 StartKeyIdx, int32 EndKeyIdx)
		{
			float StartU = U[StartKeyIdx];
			float EndU = U[EndKeyIdx];

			const FPathPointWithOrientation& CPStart = ControlPoints[StartControlIdx];
			const FPathPointWithOrientation& CPEnd = ControlPoints[EndControlIdx];

			FVector2D PrevScreenPosition = CalculateScreenPosition(CPStart);
			FVector2D NextScreenPosition = CalculateScreenPosition(CPEnd);

			int32 AOIIndexLocal = CPStart.AOIIndex;
			FVector AOILocation = (AOIIndexLocal >= 0 && AOIIndexLocal < InterestPoints.Num())
				? InterestPoints[AOIIndexLocal].Center
				: (CPStart.Point + FVector(100, 0, 0));

			for (int32 i = StartKeyIdx + 1; i < EndKeyIdx; ++i)
			{
				float t = U[i];
				float normalizedT = (t - StartU) / (EndU - StartU);

				RRTPath[i].FOV = FMath::Lerp(CPStart.FOV, CPEnd.FOV, normalizedT);
				// 之后统一计算而不是插值
				/*RRTPath[i].AestheticScore = FMath::Lerp(CPStart.AestheticScore, CPEnd.AestheticScore, normalizedT);*/
				/*RRTPath[i].CoverageAngle = FMath::Lerp(CPStart.CoverageAngle, CPEnd.CoverageAngle, normalizedT);*/
				RRTPath[i].AOIIndex = CPStart.AOIIndex;

				FVector2D InterpolatedScreenPosition = FMath::Lerp(PrevScreenPosition, NextScreenPosition, normalizedT);
				FVector CameraLocation = RRTPath[i].Point;
				FRotator InterpolatedOrientation = CalculateOrientationFromScreenPosition(
					CameraLocation,
					AOILocation,
					InterpolatedScreenPosition,
					RRTPath[i].FOV
				);

				RRTPath[i].Orientation = InterpolatedOrientation;
			}
		};

	if (ControlPoints.Num() == 3)
	{
		InterpolateSegment(0, 1, KeyIndices[0], KeyIndices[1]);
		InterpolateSegment(1, 2, KeyIndices[1], KeyIndices[2]);
	}
	else
	{
		InterpolateSegment(0, 1, KeyIndices[0], KeyIndices[1]);
	}

	// 异步计算美学评分
	FThreadSafeCounter TaskCounter(RRTPath.Num());
	for (FPathPointWithOrientation& Viewpoint : RRTPath)
	{
		AsyncTask(ENamedThreads::GameThread, [this, &Viewpoint, &TaskCounter]()
			{
				RenderViewpointToRenderTarget(Viewpoint);
				if (NimaTracker && RenderTarget)
				{
					NimaTracker->RunInference(RenderTarget);
				}

				while (NimaTracker->GetNimaScore() <= 0)
				{
					FPlatformProcess::Sleep(0.001f);
				}

				Viewpoint.AestheticScore = NimaTracker->GetNimaScore();
				NimaTracker->ResetNimaScore();

				if (Viewpoint.AOIIndex >= 0 && Viewpoint.AOIIndex < InterestPoints.Num())
				{
					Viewpoint.CoverageAngle = CalculateViewpointCoverage(Viewpoint, InterestPoints[Viewpoint.AOIIndex]);
				}
				else
				{
					Viewpoint.CoverageAngle = 0.0f;
				}

				TaskCounter.Decrement();
			});

		while (Viewpoint.AestheticScore <= 0)
		{
			FPlatformProcess::Sleep(0.001f);
		}
	}

	while (TaskCounter.GetValue() > 0)
	{
		FPlatformProcess::Sleep(0.001f);
	}

	OutCost = ComputeTransiPathCost(RRTPath, StartRegionIndex, TargetRegionIndex, PrevMidPoint, NextMidPoint);
	// 去除RRTPath中的第一个点，因为它是StartPos
	RRTPath.RemoveAt(0);
	OutPath = RRTPath;

	return true;
}


bool ADroneActor1::BuildLinkRoute(
	const FVector& StartPos,
	const FVector& EndPos,
	int32 StartRegionIndex,
	int32 TargetRegionIndex,
	double& OutCost,
	TArray<FPathPointWithOrientation>& OutPath,
	UMyRRTClass* LocalRRTClass
)
{
	// 使用通用逻辑生成路径段并处理
	return BuildAndProcessPathSegment(StartPos, EndPos, StartRegionIndex, TargetRegionIndex, OutCost, OutPath,LocalRRTClass);
}


bool ADroneActor1::BuildStartToRegionRoute(
	const FVector& StartPos,
	const FVector& EndPos,
	int32 TargetRegionIndex,
	double& OutCost,
	TArray<FPathPointWithOrientation>& OutPath,
	UMyRRTClass* LocalRRTClass
)
{
	// 使用通用逻辑生成路径段并处理
	int32 StartRegionIndex = -1; // Start为-1表示没有region
	return BuildAndProcessPathSegment(StartPos, EndPos, StartRegionIndex, TargetRegionIndex, OutCost, OutPath, LocalRRTClass);
}


bool ADroneActor1::BuildRegionToEndRoute(
	const FVector& StartPos,
	const FVector& EndPos,
	int32 StartRegionIndex,
	double& OutCost,
	TArray<FPathPointWithOrientation>& OutPath,
	UMyRRTClass* LocalRRTClass
)
{

	// 使用通用逻辑生成路径段并处理
	// 这里TargetRegionIndex = -1 因为是End, StartRegionIndex传递为实际RegionIndex
	int32 TargetRegionIndex = -1;
	return BuildAndProcessPathSegment(StartPos, EndPos, StartRegionIndex, TargetRegionIndex, OutCost, OutPath, LocalRRTClass);
}


bool ADroneActor1::BuildSTSPCostMatrix(TArray<FDoubleArray>& OutCostMatrix)
{
	UE_LOG(LogTemp,Warning , TEXT("Building Cost Matrix"));

	int32 NumRegions = InterestAreas.Num();
	int32 NumNodes = 2 + 2 * NumRegions; // 0:Start, 1:End; 2/3:Region0 Entry/Exit; 4/5:Region1 Entry/Exit; etc.

	if (NumRegions == 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("No interest areas found."));
		return false;
	}

	TArray<FVector> NodePositions;
	NodePositions.SetNum(NumNodes);

	// 设置Start和End
	NodePositions[0] = StartLocation;
	NodePositions[1] = EndLocation;

	// 设置各区域Entry和Exit
	for (int32 i = 0; i < NumRegions; ++i)
	{
		const FInterestArea& Area = InterestAreas[i];
		NodePositions[2 + 2 * i] = Area.PathPoints[0].Point;      // Entry
		NodePositions[3 + 2 * i] = Area.PathPoints.Last().Point; // Exit
	}

	// 初始化代价矩阵
	OutCostMatrix.SetNum(NumNodes);
	for (int32 i = 0; i < NumNodes; ++i)
	{
		// 先只调用SetNum设定元素个数，不传第二个参数
		OutCostMatrix[i].Values.SetNum(NumNodes);

		// 然后循环赋值为TNumericLimits<double>::Max()
		for (int32 j = 0; j < NumNodes; ++j)
		{
			OutCostMatrix[i].Values[j] = TNumericLimits<double>::Max();
		}
	}

	// 初始化LinkRoutes
	LinkRoutes.SetNum(NumNodes);
	for (int32 i = 0; i < NumNodes; ++i)
	{
		LinkRoutes[i].SetNum(NumNodes);
	}

	// 辅助函数：获取区域索引
	auto GetRegionIndexFromNode = [&](int32 NodeIndex) -> int32
		{
			if (NodeIndex < 2) return -1; // Start and End have no region
			return (NodeIndex - 2) / 2;
		};

	//TFuture<UMyRRTClass*> MyFuture;
	//{
	//	// 创建Promise
	//	TSharedRef<TPromise<UMyRRTClass*>> Promise = MakeShared<TPromise<UMyRRTClass*>>();
	//	// 拿到对应的Future
	//	MyFuture = Promise->GetFuture();

	//	// 在异步线程里发起请求
	//	AsyncTask(ENamedThreads::GameThread, [Promise,this]()
	//		{
	//			// 切到主线程执行NewObject
	//			UMyRRTClass* RRTClass = NewObject<UMyRRTClass>();
	//			RRTClass->SetWorld(GetWorld());

	//			// 通知Promise
	//			Promise->SetValue(RRTClass);
	//		});
	//}

	//// 等待或轮询
	//UMyRRTClass* RRTClass = MyFuture.Get(); // Get()会阻塞直到SetValue被调用

	// 使用前检查有效性
	if (!GlobalRRTClass)
	{
		UE_LOG(LogTemp, Error, TEXT("RRTClass is not valid"));
		return false;
	}

	UMyRRTClass* RRTClass = GlobalRRTClass;

	int Progress = 0;
	int TotalComputation = NumNodes * NumNodes;
	// 构建代价矩阵
	for (int32 i = 0; i < NumNodes; ++i)
	{
		for (int32 j = 0; j < NumNodes; ++j)
		{
			// 在后台线程中直接操作UI不安全，需要切回GameThread
			float CurrentProgress = (static_cast<float>(Progress) / TotalComputation) * 100.0f;
			int UpdateFrequency = 2; // 每处理100次迭代更新一次
			if (Progress % UpdateFrequency == 0) {
				
				AsyncTask(ENamedThreads::GameThread, [this, CurrentProgress]() {
					OnPathGenerationProgress.Broadcast(CurrentProgress, TEXT("Building STSP Cost Matrix..."));
					});
				UE_LOG(LogTemp, Warning, TEXT("Progress: %.1f%%"), CurrentProgress);
			}
			//UE_LOG(LogTemp, Warning, TEXT("Building STSP Cost Matrix. Progress: %.1f%%"), CurrentProgress);

			if (i == j)
			{
				OutCostMatrix[i].Values[j] = TNumericLimits<double>::Max();
				continue;
			}

			int32 RegionFrom = GetRegionIndexFromNode(i);
			int32 RegionTo = GetRegionIndexFromNode(j);

			// 同一区域内部路径（Entry->Exit或 Exit->Entry）代价设为0
			if (RegionFrom == RegionTo && RegionFrom != -1)
			{
				OutCostMatrix[i].Values[j] = 0.0;

				// 记录路径
				FLinkRoute Route;
				Route.StartNodeIndex = i;
				Route.EndNodeIndex = j;
				Route.Cost = 0.0;

				// 内部路径已预定义，无需具体 PathPoints
				Route.PathPoints.Empty(); // 或根据需求填充

				LinkRoutes[i][j] = Route;
			}
			else
			{
				// 根据 RegionFrom 和 RegionTo 调用相应的路径构建函数
				double LinkCost = TNumericLimits<double>::Max();
				TArray<FPathPointWithOrientation> LinkPath;

				bool bSuccess = false;

				if (RegionFrom == -1 && RegionTo != -1)
				{
					// 从 Start 到区域
					bSuccess = BuildStartToRegionRoute(NodePositions[i], NodePositions[j],
						RegionTo, LinkCost, LinkPath, RRTClass);
				}
				else if (RegionTo == -1 && RegionFrom != -1)
				{
					// 从区域到 End
					bSuccess = BuildRegionToEndRoute(NodePositions[i], NodePositions[j],
						RegionFrom, LinkCost, LinkPath, RRTClass);
				}
				else if (RegionFrom != -1 && RegionTo != -1)
				{
					// 区域到区域
					bSuccess = BuildLinkRoute(NodePositions[i], NodePositions[j],
						RegionFrom, RegionTo, LinkCost, LinkPath, RRTClass);
				}

				if (bSuccess)
				{
					OutCostMatrix[i].Values[j] = LinkCost;

					FLinkRoute Route;
					Route.StartNodeIndex = i;
					Route.EndNodeIndex = j;
					Route.Cost = LinkCost;
					Route.PathPoints = LinkPath;

					LinkRoutes[i][j] = Route;
				}
				else
				{
					OutCostMatrix[i].Values[j] = TNumericLimits<double>::Max();
					// LinkRoutes[i][j] 保持默认状态
				}
			}

			++Progress;
		}
	}

	// 输出代价矩阵
	FString SavePath = FPaths::ProjectDir() + TEXT("CostMatrix.csv");
	FString CSVContent;

	for (int32 i = 0; i < OutCostMatrix.Num(); ++i)
	{
		FString Row;
		for (int32 j = 0; j < OutCostMatrix[i].Values.Num(); ++j)
		{
			Row += FString::Printf(TEXT("%f,"), OutCostMatrix[i].Values[j]);
		}
		Row.RemoveFromEnd(TEXT(",")); // 移除行末尾的逗号
		CSVContent += Row + LINE_TERMINATOR;
	}

	FFileHelper::SaveStringToFile(CSVContent, *SavePath);
	UE_LOG(LogTemp, Warning, TEXT("Cost Matrix saved to %s"), *SavePath);

	return true;
}




TArray<FPathPointWithOrientation> ADroneActor1::BuildFinalPath(const TArray<int32>& BestOrder)
{
	TArray<FPathPointWithOrientation> FinalPath;

	// 构建完整的节点序列：Start -> BestOrder... -> End
	TArray<int32> NodeSequence;
	NodeSequence.Add(0); // Start
	for (int32 Node : BestOrder)
	{
		NodeSequence.Add(Node);
	}
	NodeSequence.Add(1); // End

	// 串联各段路径
	for (int32 i = 0; i < NodeSequence.Num() - 1; ++i)
	{
		int32 FromNode = NodeSequence[i];
		int32 ToNode = NodeSequence[i + 1];

		const FLinkRoute& Route = LinkRoutes[FromNode][ToNode];

		if (Route.PathPoints.Num() == 0)
		{
			// 内部路径代价为0，直接使用 InterestAreas 中的 PathPoints
			// 确保 FromNode 和 ToNode 是同一区域的 Entry 和 Exit
			int32 RegionIndex = (FromNode >= 2) ? (FromNode - 2) / 2 : -1;
			if (RegionIndex != -1 && InterestAreas.IsValidIndex(RegionIndex))
			{
				// 根据 FromNode 是 Entry 还是 Exit，决定使用正向或反向路径
				bool bFromEntry = IsEntryNode(FromNode);
				TArray<FPathPointWithOrientation> InternalPath = InterestAreas[RegionIndex].PathPoints;

				if (!bFromEntry)
				{
					Algo::Reverse(InternalPath);
				}

				// 拼接路径，避免重复添加节点
				if (FinalPath.Num() == 0)
				{
					FinalPath.Append(InternalPath);
				}
				else
				{
					// 跳过第一个点以避免重复
					FinalPath.Append(InternalPath.GetData() + 1, InternalPath.Num() - 1);
				}
			}
		}
		else
		{
			// 外部过渡路径
			if (FinalPath.Num() == 0)
			{
				FinalPath.Append(Route.PathPoints);
			}
			else
			{
				// 跳过第一个点以避免重复
				FinalPath.Append(Route.PathPoints.GetData() + 1, Route.PathPoints.Num() - 1);
			}
		}
	}

	return FinalPath;
}



// 响应生成局部和全局的轨道飞行路径
void ADroneActor1::OnGenerateOrbitFlightPath()
{
	if (fGenerationFinished)
	{
		// 清空路径点
		InterestPoints.Empty();
		GlobalPathPoints.Empty();
		GlobalBestSplinePoints.Empty();
		TestPathPoints.Empty();
		bShouldDrawPathPoints = false;
		fGenerationFinished = false;

		DestroyPathPoints();
	}
	else {
		if (bIsGeneratingPath)
		{
			UE_LOG(LogTemp, Warning, TEXT("Path generation already in progress"));
			return;
		}

		// 绑定事件处理器 - 使用 IsAlreadyBound 检查
		if (!OnPathGenerationProgress.IsAlreadyBound(this, &ADroneActor1::OnPathGenerationProgressUpdate))
		{
			OnPathGenerationProgress.AddDynamic(
				this,
				&ADroneActor1::OnPathGenerationProgressUpdate
			);
		}

		if (!OnPathGenerationComplete.IsAlreadyBound(this, &ADroneActor1::OnPathGenerationCompleted))
		{
			OnPathGenerationComplete.AddDynamic(
				this,
				&ADroneActor1::OnPathGenerationCompleted
			);
		}

		// 先把起点和终点设置为启动点
		StartLocation = GetActorLocation();
		EndLocation = GetActorLocation();

		currentIndex = 0;
		bShouldDrawPathPoints = false; // 确认禁用路径点绘制
		fGenerationFinished = false;

		// 启动异步任务
		GenerateOrbitFlightPathAsync();
	}
}

// 事件处理器：路径生成进度更新
void ADroneActor1::OnPathGenerationProgressUpdate(float Progress, const FString& Status)
{
	// 在UI上更新进度
	GEngine->AddOnScreenDebugMessage(-1, 1.0f, FColor::Green,
		FString::Printf(TEXT("Progress: %.1f%% - %s"), Progress, *Status));
}

void ADroneActor1::UnbindPathGenerationEvents()
{
	OnPathGenerationProgress.RemoveDynamic(this, &ADroneActor1::OnPathGenerationProgressUpdate);
	OnPathGenerationComplete.RemoveDynamic(this, &ADroneActor1::OnPathGenerationCompleted);
}

// 事件处理器：路径生成完成
void ADroneActor1::OnPathGenerationCompleted(bool Success)
{
	if (Success)
	{
		GEngine->AddOnScreenDebugMessage(-1, 5.0f, FColor::Green,
			TEXT("Path generation completed successfully"));

		// 这里可以添加生成完成后的其他操作
		// 例如：更新UI、开始导航等
		bShouldDrawPathPoints = true; // 启用路径点绘制
		fGenerationFinished = true;

	}
	else
	{
		GEngine->AddOnScreenDebugMessage(-1, 5.0f, FColor::Red,
			TEXT("Path generation failed"));
	}

	// 在下一帧解绑事件
	FTimerHandle UnbindTimerHandle;
	GetWorld()->GetTimerManager().SetTimerForNextTick(
		FTimerDelegate::CreateUObject(this, &ADroneActor1::UnbindPathGenerationEvents)
	);
}




// 实现异步生成入口
void ADroneActor1::GenerateOrbitFlightPathAsync()
{
	if (bIsGeneratingPath)
	{
		UE_LOG(LogTemp, Warning, TEXT("Path generation already in progress"));
		return;
	}

	bIsGeneratingPath = true;
	PathGenerationTask = new FAsyncTask<FOrbitPathGenerationTask>(this);

	// 启动异步任务
	PathGenerationTask->StartBackgroundTask(); // 就是调用FOrbitPathGenerationTask::DoWork()函数

	// 开始轮询进度
	GetWorld()->GetTimerManager().SetTimer(
		ProgressCheckTimerHandle,
		this,
		&ADroneActor1::CheckGenerationProgress,
		0.1f,
		true
	);
}

// 修改后的进度检查函数
void ADroneActor1::CheckGenerationProgress()
{
	if (!PathGenerationTask) return;

	if (PathGenerationTask->IsDone())
	{
		// 任务完成，清理资源
		GetWorld()->GetTimerManager().ClearTimer(ProgressCheckTimerHandle);
		bIsGeneratingPath = false;

		// 已经在主任务中处理了
		//// 在游戏线程处理结果
		//AsyncTask(ENamedThreads::GameThread, [this]()
		//	{
		//		OnPathGenerationComplete.Broadcast(true);
		//	});

		delete PathGenerationTask;
		PathGenerationTask = nullptr;
	}
	// 移除else分支，因为进度报告已在GenerateOrbitFlightPath_Internal中处理
}







void ADroneActor1::OnGenerateSimpleFlight() {

	// 假如有兴趣点之后
	if (!fGenerationFinished && InterestPoints.Num() > 0) {
		// 重置索引
		currentIndex = 0;

		GenerateSimpleFlightPath();
		fGenerationFinished = true;
		bShouldDrawPathPoints = true;
	}
	else {
		// 清空兴趣点和路径点
		InterestPoints.Empty();
		GlobalPathPoints.Empty();

		bShouldDrawPathPoints = false;
		fGenerationFinished = false;

	}

}

void ADroneActor1::OnGenerateOptimizedFlight()
{
	// 检查是否有兴趣点
	if (!fGenerationFinished && InterestPoints.Num() > 0)
	{
		GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Yellow, TEXT("Starting to plan optimized flight..."));

		// 重置当前索引
		currentIndex = 0;

		// 获取当前无人机的位置和朝向
		FVector DronePosition = GetActorLocation();
		FRotator DroneOrientation = GetActorRotation();

		// 使用当前无人机位置作为起点和终点
		GenerateOptimizedFlightPath(DronePosition, DronePosition);

		// 启用路径点绘制
		bShouldDrawPathPoints = true;
		fGenerationFinished = true;
	}
	else
	{
		GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Yellow, TEXT("Clearing interest points and path points..."));

		// 清空兴趣点和路径点
		InterestPoints.Empty();
		GlobalPathPoints.Empty();

		// 禁用路径点绘制
		bShouldDrawPathPoints = false;
		fGenerationFinished = false;
	}
}


void ADroneActor1::OnStartFlightAlongPath()
{

	if (!bIsFlyingAlongPath)
	{
		// 检查路径点是否已经生成
		if (GlobalPathPoints.Num() == 0)
		{
			GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Red, TEXT("No path points generated!"));
			return;
		}

		// 禁用 PlayerController 输入
		if (PlayerController)
		{
			PlayerController->DisableInput(PlayerController);
		}

		GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Green, TEXT("Starting flight along the path..."));
		DestroyPathPoints(); // 清除路径点显示

		// 重置当前索引
		currentIndex = 0; // 应该先飞到第一个点	

		// 设置初始点
		OriginalPathPoint.Point = GetActorLocation();
		OriginalPathPoint.Orientation = GetActorRotation();
		OriginalPathPoint.FOV = CameraComponent->FieldOfView;
		StartRotation = GetActorRotation();
		PrecomputeAllDuration();
		StartNewPathPoint();

		bIsFlyingAlongPath = true;
	}
	else
	{
		// 停止飞行
		bIsFlyingAlongPath = false;

		// 恢复 PlayerController 输入
		if (PlayerController)
		{
			PlayerController->EnableInput(PlayerController);
		}

	}


}


void ADroneActor1::OnReadPathFromFile() {
	// 获取项目目录
	FString ProjectDir = FPaths::ProjectDir();

	// 查找所有符合命名规则的文件
	TArray<FString> FoundFiles;
	IFileManager::Get().FindFiles(FoundFiles, *(ProjectDir + TEXT("Path_*.txt")), true, false);

	if (FoundFiles.Num() == 0)
	{
		GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Red, TEXT("No path files found!"));
		return;
	}

	// 按修改时间排序文件
	FoundFiles.Sort([&](const FString& A, const FString& B) {
		FDateTime TimeA = IFileManager::Get().GetTimeStamp(*(ProjectDir + A));
		FDateTime TimeB = IFileManager::Get().GetTimeStamp(*(ProjectDir + B));
		return TimeA > TimeB; // 降序排列，最近的文件在前
		});

	// 获取最近的文件路径
	FString MostRecentFilePath = ProjectDir + FoundFiles[0];

	// 检查文件是否存在
	if (!FPaths::FileExists(MostRecentFilePath))
	{
		GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Red, TEXT("Most recent path file not found!"));
		return;
	}

	if (bIsGeneratingPath)
	{
		UE_LOG(LogTemp, Warning, TEXT("Path generation already in progress"));
		return;
	}

	// 读取路径点
	ImportPathPointsFromTxt(MostRecentFilePath);

	// 启用路径点绘制
	bShouldDrawPathPoints = true;
	fGenerationFinished = true;

	// 显示成功消息
	GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Green, FString::Printf(TEXT("Loaded most recent path file: %s"), *MostRecentFilePath));
}


void ADroneActor1::OnStartTracking()
{
	if (bIsTrackingActive)
	{
		// If tracking is active, stop tracking
		YoloTracker->StopTracking();
		bIsTrackingActive = false;
		UE_LOG(LogTemp, Warning, TEXT("Tracking stopped."));
	}
	else {
		FVector2D MousePosition;

		if (GetWorld()->GetFirstPlayerController()->GetMousePosition(MousePosition.X, MousePosition.Y))
		{
			// 创建或重用 RenderTarget
			if (!RenderTarget)
			{
				// 使用 TSharedPtr 管理 RenderTarget
				//RenderTarget = MakeShareable(NewObject<UTextureRenderTarget2D>(this, TEXT("ScreenShot")));

				RenderTarget = NewObject<UTextureRenderTarget2D>(this, TEXT("ScreenShot"));

				RenderTarget->InitAutoFormat(ViewportWidth, ViewportHeight);
				RenderTarget->UpdateResourceImmediate(true);
				SceneCaptureComponent->TextureTarget = RenderTarget;

				// 配置 SceneCaptureComponent 以确保捕捉正确的渲染内容
				SceneCaptureComponent->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR; // 关键在于使用最终颜色而不是深度或其他
				/*SceneCaptureComponent->PostProcessSettings.bOverride_BloomIntensity = true;
				SceneCaptureComponent->PostProcessSettings.BloomIntensity = 1.0f;*/
				// 根据需要添加更多后处理设置

				UE_LOG(LogTemp, Log, TEXT("RenderTarget initialized with size: %dx%d"), ViewportWidth, ViewportHeight);
			}
			SceneCaptureComponent->CaptureScene();

			// Start tracking using the render target and mouse click position
			YoloTracker->StartTracking(RenderTarget, MousePosition);
			LastTrackedPosition = MousePosition; // Initialize tracking position
			UE_LOG(LogTemp, Warning, TEXT("Tracking started at mouse position (%f, %f)."), MousePosition.X, MousePosition.Y);
			bIsTrackingActive = true;
		}
		else
		{
			UE_LOG(LogTemp, Warning, TEXT("Failed to get mouse position."));
		}
	}
}

void ADroneActor1::OnCaptureScreenshotWithUI(bool ifWithUI)
{
	// 获取当前时间
	FDateTime Now = FDateTime::Now();

	// 格式化时间为字符串（例如：20231025_153045）
	FString Timestamp = Now.ToString(TEXT("%Y%m%d_%H%M%S"));

	// 设置截图文件名，包含时间戳
	FString ScreenshotName = FString::Printf(TEXT("Screenshot_%s.png"), *Timestamp);
	// 选择保存目录：项目的 Saved/CapturedImages 目录
	FString SaveDirectory = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("CapturedImages"));
	IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
	// 检查目录是否存在，如果不存在则创建
	if (!PlatformFile.DirectoryExists(*SaveDirectory))
	{
		bool bCreated = PlatformFile.CreateDirectory(*SaveDirectory);
		if (!bCreated)
		{
			UE_LOG(LogTemp, Error, TEXT("Failed to create directory: %s"), *SaveDirectory);
		}
	}
	FString FullPath = FPaths::Combine(SaveDirectory, ScreenshotName);

	// 请求截图，并启用 UI 捕获
	FScreenshotRequest::RequestScreenshot(FullPath, ifWithUI, false); // 第二个参数为 `bCaptureUI`

	// 确保截图完成
	if (FScreenshotRequest::IsScreenshotRequested())
	{
		UE_LOG(LogTemp, Log, TEXT("Screenshot with UI requested: %s"), *ScreenshotName);
	}
}

//void ADroneActor1::OnLeftMouseClick()
//{
//    APlayerController* PlayerController = UGameplayStatics::GetPlayerController(this, 0);
//    if (PlayerController)
//    {
//        FVector WorldLocation, WorldDirection;
//        if (PlayerController->DeprojectMousePositionToWorld(WorldLocation, WorldDirection))
//        {
//            // 设置射线的起点和终点
//            FVector Start = WorldLocation;
//            FVector End = Start + (WorldDirection * 100000.0f); // 射线长度
//
//            FHitResult HitResult;
//            FCollisionQueryParams Params;
//            Params.AddIgnoredActor(this); // 忽略当前的 DroneActor
//
//            // 执行标准的射线检测，包括物理碰撞
//            bool bHit = GetWorld()->LineTraceSingleByChannel(HitResult, Start, End, ECC_Visibility, Params);
//            if (bHit)
//            {
//                FVector HitLocation = HitResult.Location;
//
//                if (CesiumGeoreference)
//                {
//                    // 将 Unreal 世界坐标转换为地理坐标
//                    FVector GeoLocation = CesiumGeoreference->TransformUnrealPositionToLongitudeLatitudeHeight(HitLocation);
//                    GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Blue, FString::Printf(TEXT("Geographic Location: Longitude=%.6f, Latitude=%.6f, Height=%.2f"), GeoLocation.X, GeoLocation.Y, GeoLocation.Z));
//                }
//                else
//                {
//                    GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Red, TEXT("CesiumGeoreference not found"));
//                }
//            }
//            else
//            {
//                GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Red, TEXT("No hit detected"));
//            }
//        }
//    }
//}


// 点击函数
void ADroneActor1::OnLeftMouseClick()
{
	if (!CesiumGeoreference) return;
	if (!PlayerController) return;

	FVector WorldLocation, WorldDirection;
	if (PlayerController->DeprojectMousePositionToWorld(WorldLocation, WorldDirection))
	{
		FVector Start = WorldLocation;
		FVector End = Start + (WorldDirection * 100000.0f);

		FHitResult HitResult;
		FCollisionQueryParams Params;
		Params.AddIgnoredActor(this);

		if (GetWorld()->LineTraceSingleByChannel(HitResult, Start, End, ECC_Visibility, Params))
		{
			FVector HitLocation = HitResult.Location;
			FVector _ProjectedLocation;

			// 将 Unreal 世界坐标转换为地理坐标	
            FVector GeoLocation = CesiumGeoreference->TransformUnrealPositionToLongitudeLatitudeHeight(HitLocation);
            GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Blue, FString::Printf(TEXT("Geographic Location: Longitude=%.6f, Latitude=%.6f, Height=%.2f"), GeoLocation.X, GeoLocation.Y, GeoLocation.Z));
			UE_LOG(LogTemp, Warning, TEXT("Geographic Location: Longitude=%.6f, Latitude=%.6f, Height=%.2f"), GeoLocation.X, GeoLocation.Y, GeoLocation.Z);

			// 第一次点击：选择顶部的中点
			if (CurrentState == EInterestPointState::Idle)
			{
				CenterPoint = HitLocation; // 将顶部中点设为初始中心点
				CurrentState = EInterestPointState::CenterSelected;
				GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Green, TEXT("Top center point selected"));
			}
			// 第二次点击：选择半径
			else if (CurrentState == EInterestPointState::CenterSelected)
			{
				float PlaneZ = CenterPoint.Z;
				float DistanceToPlane = (PlaneZ - Start.Z) / WorldDirection.Z;
				_ProjectedLocation = Start + DistanceToPlane * WorldDirection;

				SecondPoint = _ProjectedLocation;
				Radius = FVector::Dist(CenterPoint, SecondPoint);
				CurrentState = EInterestPointState::RadiusSelected;

				GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Green, FString::Printf(TEXT("Radius selected: %.2f"), Radius));
			}
			// 第三次点击：选择高度
			else if (CurrentState == EInterestPointState::RadiusSelected)
			{
				// 获取相机位置并定义平面法向量
				FVector CameraLocation = PlayerController->PlayerCameraManager->GetCameraLocation();
				FVector ViewDirection = (CameraLocation - SecondPoint).GetSafeNormal();

				// 计算法向量
				FVector IntermediateVector = FVector::CrossProduct(FVector::UpVector, ViewDirection).GetSafeNormal();
				FVector PlaneNormal = FVector::CrossProduct(IntermediateVector, FVector::UpVector).GetSafeNormal();

				// 计算鼠标射线与该垂直平面的交点
				float DotProduct = FVector::DotProduct(WorldDirection, PlaneNormal);
				if (FMath::Abs(DotProduct) > KINDA_SMALL_NUMBER)
				{
					float DistanceToPlane = FVector::DotProduct((SecondPoint - Start), PlaneNormal) / DotProduct;
					FVector IntersectionPoint = Start + DistanceToPlane * WorldDirection;

					// 设置第三点的 Z 值和高度
					ThirdPoint = FVector(SecondPoint.X, SecondPoint.Y, IntersectionPoint.Z);
					Height = FMath::Abs(ThirdPoint.Z - CenterPoint.Z); // 计算高度
					CurrentState = EInterestPointState::HeightSelected;

					GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Green, FString::Printf(TEXT("Height selected: %.2f"), Height));

					// 判断是否需要将中点移动到底部
					if (ThirdPoint.Z < CenterPoint.Z)
					{
						CenterPoint.Z -= Height; // 将中心点移动到底部
						SecondPoint.Z -= Height; // 将第二点移动到底部
						ThirdPoint.Z += Height; // 将第三点移动到顶部
						GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Blue, TEXT("Center point moved to bottom"));
					}
				}
			}
			else if (CurrentState == EInterestPointState::HeightSelected)
			{
				// 保存兴趣点
				FCylindricalInterestPoint NewInterestPoint;
				NewInterestPoint.BottomCenter = CenterPoint;
				NewInterestPoint.Center = FVector(CenterPoint.X, CenterPoint.Y, CenterPoint.Z + Height / 2); // 兴趣区域体积中心
				NewInterestPoint.Radius = Radius;
				NewInterestPoint.MinSafetyDistance = 100.0f; // 根据需要设置
				NewInterestPoint.Height = Height;
				

				InterestPoints.Add(NewInterestPoint);

				// 重置状态
				CurrentState = EInterestPointState::Idle;

				GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Green, FString::Printf(TEXT("Interest point %d saved"), InterestPoints.Num()));
			}
		}
	}
}


void ADroneActor1::OnRightMouseClick() {

}

// 在到达路径点后，计算下一个路径点所需的旋转速度
void ADroneActor1::UpdateRotationSpeedForNextPathPoint(float minRotatSpeed, float maxRotateSpeed)
{
	if (currentIndex + 1 >= GlobalPathPoints.Num())
		return; // 如果没有下一个路径点则返回

	FVector CurrentLocation;
	FRotator CurrentRotation;
	if (currentIndex != 0) {
		CurrentLocation = GlobalPathPoints[currentIndex].Point;
		CurrentRotation = GlobalPathPoints[currentIndex].Orientation;
	}
	else {
		CurrentLocation = GetActorLocation();
		CurrentRotation = GetActorRotation();
	}

	FVector NextLocation = GlobalPathPoints[currentIndex + 1].Point;

	// 计算到下一个路径点的距离和时间
	float DistanceToNextPoint = FVector::Dist(CurrentLocation, NextLocation);
	float _TimeToNextPoint = DistanceToNextPoint / MoveSpeed;

	// 计算当前朝向和目标朝向的差异角度

	FRotator _TargetRotation = GlobalPathPoints[currentIndex + 1].Orientation;
	float _AngleDifference = FMath::Abs(CurrentRotation.Yaw - _TargetRotation.Yaw);

	// 动态计算旋转速度，以确保在到达时间内平滑旋转到目标朝向
	RotationInterpSpeed = FMath::Clamp(_AngleDifference / _TimeToNextPoint, minRotatSpeed, maxRotateSpeed);
}
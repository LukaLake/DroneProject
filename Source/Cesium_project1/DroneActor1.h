// Fill out your copyright notice in the Description page of Project Settings.

#pragma once


#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/PlayerController.h"
#include "Kismet/GameplayStatics.h"
#include "Camera/CameraComponent.h"
#include "GameFramework/FloatingPawnMovement.h"
#include <Components/SceneCaptureComponent2D.h>
#include "Components/SplineComponent.h"
#include "Components/SplineMeshComponent.h"

// 在 ADroneActor1.h 文件的顶部添加
#include "CesiumGlobeAnchorComponent.h"
#include "EngineUtils.h"

#include "Public/MyTSPClass.h"

#include "Public/MyUtil.h"
#include "DroneActor1.generated.h"

// 添加委托声明
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FOnPathGenerationProgress, float, Progress, const FString&, Status);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnPathGenerationComplete, bool, Success);


class FOrbitPathGenerationTask;

UCLASS()
class CESIUM_PROJECT1_API ADroneActor1 : public APawn
{
	GENERATED_BODY()

public:
	// Sets default values for this actor's properties
	ADroneActor1();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	// Override the function to setup player inputs
	virtual void SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent) override;

	bool GetViewportSize(int32& OutWidth, int32& OutHeight);
	void InitializeRenderTarget();

	// 声明鼠标输入的回调函数
	void OnMouseMove(float DeltaX, float DeltaY);

	// Function to handle the "PredictAction" action
	void OnPredictAction();

	// Start tracking
	void OnStartTracking();

	// 截图函数
	void OnCaptureScreenshotWithUI(bool ifWithUI=true);

	// 规划简单飞行路径
	UFUNCTION(BlueprintCallable, Category = "Flight Path")
	void OnGenerateSimpleFlight();
	// 规划优化飞行路径
	UFUNCTION(BlueprintCallable, Category = "Flight Path")
	void OnGenerateOptimizedFlight();

	// 开始飞行
	UFUNCTION(BlueprintCallable, Category = "Flight Path")
	void OnStartFlightAlongPath();

	UFUNCTION(BlueprintCallable, Category = "Flight Path")
	void OnReadPathFromFile();

	// 鼠标灵敏度变量
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Input")
	float MouseSensitivity;

	/*std::unique_ptr<Ort::Session> currentONNXModel;*/
	// Ort::Session currentONNXModel;

	// 移动组件
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Movement")
	UFloatingPawnMovement* FloatingMovement;

	// 相机组件
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Camera")
	UCameraComponent* CameraComponent;

	// 场景捕捉组件
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Camera")
	USceneCaptureComponent2D* SceneCaptureComponent;

	// 渲染目标
	// 渲染目标
	UPROPERTY()
	UTextureRenderTarget2D* RenderTarget;

	// 在类中添加成员变量
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Cesium")
	UCesiumGlobeAnchorComponent* GlobeAnchorComponent;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UI")
	TSubclassOf<UUserWidget> SpeedDisplayWidgetClass;

	//-------------------------------------------------------------------------------------
	// 测试用函数

	// 计算简单的螺旋飞行路径
	UFUNCTION(BlueprintCallable, Category = "Flight Path")
	void GenerateSimpleFlightPath();

	// 生成优化的飞行路径 针对多个兴趣区域
	UFUNCTION(BlueprintCallable, Category = "Flight Path")
	void GenerateOptimizedFlightPath(const FVector& StartPoint, const FVector& EndPoint);

	//-------------------------------------------------------------------------------------
	// 最优化轨道计算

	/**
	 * 计算轨道参数，包括轨道半径、最小高度和最大高度。
	 *
	 * @param InterestPoint 兴趣点，包含中心、底部中心、半径和高度等信息。
	 * @param OutOrbitRadius 输出参数，计算得到的轨道半径。
	 * @param OutMinHeight 输出参数，计算得到的最小高度。
	 * @param OutMaxHeight 输出参数，计算得到的最大高度。
	 */
	void CalculateOrbitParameters(const FCylindricalInterestPoint& InterestPoint,
		float& OutOrbitRadius, float& OutOrbitRadius1, float& OutOrbitRadius2,
		float& OutMinHeight, float& OutMaxHeight);

	// Calculate the framing cost
	// The framing cost measures the difference between the current normalized position of the target in the image and the ideal position.
	float CalculateCompositionCost(FCandidateViewpoint& Candidate);

	bool ProjectWorldPointToScreen(const FVector& WorldPoint, FVector2D& ScreenPoint,
		const FVector& CameraLocation, const FRotator& CameraRotation, float FOV);

	// Calculate the visibility cost
	// The visibility cost ensures the AOI is fully visible and not cut off in the image.
	float CalculateVisibilityCost(FCandidateViewpoint& Candidate);

	float CalculateDynamicEdgeThreshold(const FCylindricalInterestPoint& InterestPoint);

	/**
	 * 获取兴趣点的边界点。
	 *
	 * @param InterestPoint 兴趣点，包含中心、底部中心、半径和高度等信息。
	 * @param OutPoints 输出参数，存储计算得到的边界点。
	 */
	void GetAOIBoundingPoints(const FCylindricalInterestPoint& InterestPoint, TArray<FVector>& OutPoints);

	// Check if an object is visible from a given viewpoint
	bool IsObjectVisible(const FVector& ViewpointLocation, const FRotator& CameraRotation, const FCylindricalInterestPoint& InterestPoint);

	// ----------------------------------------------------------------------------------------------------
	// 计算所有点速度
	float CalculateTurnAngle(const FVector& PrevPoint, const FVector& CurrentPoint, const FVector& NextPoint);
	float CalculateCumulativeTurnAngle(int32 StartIndex, int32 LookaheadPoints);
	float CalculateCumulativeTurnAngle_BothSides(
		int32 CurrentIndex,
		int32 BackPoints,
		int32 FrontPoints);

	float CalculateViewChange(const FRotator& PrevOrientation, const FRotator& CurrentOrientation,
		const FRotator& NextOrientation, float PrevFOV, float CurrentFOV, float NextFOV);
	float CalculateCumulativeViewChange(int32 CurrentIndex, int32 LookaheadPoints);
	float CalculateCumulativeViewChange_BothSides(
		int32 CurrentIndex,
		int32 BackPointsToLook,
		int32 FrontPointsToLook);
	
	void SmoothSegmentSpeed(int32 Iterations);

	void ExportPathPointsToWGS84Txt();
	bool ImportPathPointsFromTxt(const FString& LoadFilePath);
	void SmoothGlobalPathPoints_PositionOrientation(int32 Iterations =2);
	void ComputeSpeedByCurvatureAndViewChange(float MaxSpeed = 1500.0f, float MinSpeed = 500.0f, float SharpTurnAngle = 120.0f, float MaxViewChangeAngle = 90.0f, int32 LookaheadPoints = 5);

	// ---------------------------------------------------------------------------------------------------

	// 测试
	FPathPointWithOrientation TestGenerateCandidateViewpoints(const FPathPointWithOrientation& PathPoint);
	void OnTestGenerateCandidateViewpoints();

	// Generate candidate viewpoints for each AOI.
	FPathPointWithOrientation GenerateCandidateViewpoints(
		const FPathPointWithOrientation& PathPoint,
		const FVector& PathPointLocation,
		const FRotator& OriginalRotation,
		const int32 CurrentAOIIndex,
		TArray<FCandidateViewpoint>* OutAllCandidates); // 可选参数，用于存储所有候选点

	/**
	 * 生成轨道飞行路径的内部实现。
	 *
	 * 该函数会遍历所有的兴趣点，计算每个兴趣点的轨道参数，并生成路径点。
	 * 然后根据路径点生成候选视点，并选择最佳视点组合，生成样条曲线路径。
	 * 最后将生成的路径点存储到全局路径点数组中。
	 */
	void GenerateOrbitFlightPath_Internal();

	void Force3DTilesLoad();
	void DisableForce3DTilesLoad();
	// 渲染视点到RenderTarget
	void RenderViewpointToRenderTarget(const FPathPointWithOrientation& Viewpoint);

	// 计算路径长度
	float CalculatePathLength(const TArray<FPathPointWithOrientation>& PathPoints);


	// 计算视点覆盖角度
	bool IsAngleVisible_KeepNegativeAngles(
		const FPathPointWithOrientation& Viewpoint,
		float YawAngleDeg,
		const FCylindricalInterestPoint& AOI);
	TArray<FAngleRange> BuildVisibleRanges_KeepNegativeAngles(
		const TArray<float>& SortedAngles,
		float AngleStep);
	float CalculateViewpointCoverage(FPathPointWithOrientation& Viewpoint, const FCylindricalInterestPoint& AOI);

	// 选择单个最佳视点集合
	void SelectBestViewpoints(
		const TArray<FPathPointWithOrientation>& Candidates,
		TArray<FPathPointWithOrientation>& OutBestViewpoints,
		int32 NumRequired);

	/// <summary>
	/// Selects the best viewpoint groups from the given candidates based on aesthetic score and coverage.
	/// </summary>
	/// <param name="Candidates">The array of candidate viewpoints with orientation.</param>
	/// <param name="OutViewpointGroups">The output array of selected viewpoint groups.</param>
	/// <param name="NumGroups">The number of groups to select.</param>
	/// <param name="NumViewpointsPerGroup">The number of viewpoints per group.</param>
	void SelectBestViewpointGroups(
		const TArray<FPathPointWithOrientation>& Candidates,
		TArray<TArray<FPathPointWithOrientation>>& OutViewpointGroups,
		int32 NumGroups = 3,
		int32 NumViewpointsPerGroup = 3);

	void GenerateViewpointGroups(
		const TArray<FPathPointWithOrientation>& Viewpoints,
		int32 GroupSize,
		TArray<TArray<FPathPointWithOrientation>>& OutViewpointGroups);

	void GenerateCombinationsHelper(
		int32 NumViewpoints,
		int32 GroupSize,
		TArray<int32> CurrentCombination,
		TArray<TArray<int32>>& OutCombinations);

	bool IsCoverageSatisfied(const TArray<FPathPointWithOrientation>& ViewpointGroup);

	bool IsDistanceSatisfied(const TArray<FPathPointWithOrientation>& ViewpointGroup);

	float CalculateGroupAestheticScore(const TArray<FPathPointWithOrientation>& ViewpointGroup);

	void SortViewpointsInWrappingOrder(TArray<FPathPointWithOrientation>& Viewpoints);

	void SortControlPointsInWrappingOrder(TArray<FPathPointWithOrientation>& ControlPoints);

	// 生成朝向下一个点的控制点
	/**
	 * 生成朝向下一个路径点的控制点。
	 *
	 * @param CurrentPoint 当前路径点。
	 * @param NextPoint 下一个路径点。
	 * @param _PlaneNormal 控制点所在平面的法线。
	 * @param ControlPointDistanceRatio 控制点距离当前点和下一个点之间距离的比例。
	 * @return 生成的控制点，包含位置和方向。
	 */
	FPathPointWithOrientation GenerateControlPointTowardsNext(
		const FPathPointWithOrientation& CurrentPoint, const FPathPointWithOrientation& NextPoint, 
		const FVector _PlaneNormal, float ControlPointDistanceRatio = 0.1);

	/**
	 * 生成指向前一个点的控制点。
	 */
	FPathPointWithOrientation GenerateControlPointTowardsPrev(
		const FPathPointWithOrientation& CurrentPoint, const FPathPointWithOrientation& PrevPoint,
		const FVector _PlaneNormal, float ControlPointDistanceRatio = 0.1);

	// 生成B样条路径
	TArray<FPathPointWithOrientation> GenerateSplinePath(
		const TArray<FPathPointWithOrientation>& BestViewpoints,
		TArray<FPathPointWithOrientation>& OutSplinePoints,
		float MinDisBetwPoints=500.0f);

	/**
	 * 计算B样条基函数的值。
	 *
	 * @param i 基函数的索引。
	 * @param k 基函数的阶数。
	 * @param t 参数值。
	 * @param KnotVector 节点向量。
	 * @return B样条基函数在给定参数值t处的值。
	 */
	float CalculateBSplineBasis(int32 i, int32 k, float t, const TArray<float>& KnotVector);

	/**
	 * @brief 插值路径点属性
	 *
	 * 该函数用于在路径点之间插值计算属性值。通过弦长参数化方法，计算给定参数 t 对应的路径点属性。
	 *
	 * @param ControlPoints 控制点数组，包含路径点的位置信息。
	 * @param U 弦长参数化的节点向量数组。
	 * @param t 参数 t，表示插值位置，范围为 [0, 1]。
	 * @return FPathPointWithOrientation 插值后的路径点属性。
	 */
	void InterpolatePathPointProperties(FPathPointWithOrientation& SplinePoint,
		const TArray<FPathPointWithOrientation>& ControlPoints, float t, const TArray<float>& U);

	/**
	 * 计算路径点关联的兴趣点在屏幕上的位置。
	 *
	 * @param PathPoint 包含位置和方向的路径点。
	 * @return 屏幕空间坐标，范围在[0, 1]之间。
	 */
	FVector2D CalculateScreenPosition(const FPathPointWithOrientation& PathPoint);

	/// <summary>
	/// 根据屏幕位置计算相机的目标旋转方向。
	/// </summary>
	/// <param name="CameraLocation">相机的位置，类型为 FVector，表示三维空间中的一个点。</param>
	/// <param name="AOILocation">感兴趣区域的位置，类型为 FVector，表示三维空间中的一个点。</param>
	/// <param name="ScreenPosition">屏幕位置，类型为 FVector2D，表示屏幕上的一个点，范围为 [-1.0, 1.0]。</param>
	/// <param name="FOV">相机的视野角度，类型为 float，范围为 [0.0, 180.0]。</param>
	/// <returns>计算得到的目标旋转方向，类型为 FRotator。</returns>
	FRotator CalculateOrientationFromScreenPosition(
		const FVector& CameraLocation, const FVector& AOILocation, const FVector2D& ScreenPosition, float FOV);

	// 碰撞检测
	bool IsPathCollisionFree(const TArray<FPathPointWithOrientation>& Path);

	// 路径长度代价
	float CalculateLengthCost(const TArray<FPathPointWithOrientation>& Path, float MaxLength);

	// 视图质量代价
	float CalculateQualityCost(const TArray<FPathPointWithOrientation>& Path);

	// 平滑性代价
	float CalculateSmoothnessCost(const TArray<FPathPointWithOrientation>& Path);

	// 计算过渡路径代价的函数（可自行实现）
	double ComputeTransiPathCost(
		const TArray<FPathPointWithOrientation>& Path,
		int startIndex,
		int targetIndex,
		const FVector& PrevMidPoint,    // 上一条局部航线的中点
		const FVector& NextMidPoint     // 下一条局部航线的中点
	);

	TArray<float> ParameterizePath(const TArray<FPathPointWithOrientation>& Path);
	
	bool IsEntryNode(int32 NodeIndex) const;

	bool IsExitNode(int32 NodeIndex) const;

	bool BuildAndProcessPathSegment(
		const FVector& StartPos,
		const FVector& EndPos,
		int32 StartRegionIndex,
		int32 TargetRegionIndex,
		double& OutCost,
		TArray<FPathPointWithOrientation>& OutPath
	);

	bool BuildLinkRoute(
		const FVector& StartPos,
		const FVector& EndPos,
		int32 StartRegionIndex,
		int32 TargetRegionIndex,
		double& OutCost,
		TArray<FPathPointWithOrientation>& OutPath
	);

	bool BuildStartToRegionRoute(
		const FVector& StartPos,
		const FVector& EndPos,
		int32 TargetRegionIndex,
		double& OutCost,
		TArray<FPathPointWithOrientation>& OutPath
	);

	bool BuildRegionToEndRoute(
		const FVector& StartPos,
		const FVector& EndPos,
		int32 StartRegionIndex,
		double& OutCost,
		TArray<FPathPointWithOrientation>& OutPath
	);


	// 构建STSP代价矩阵
	bool BuildSTSPCostMatrix(TArray<FDoubleArray>& OutCostMatrix);

	TArray<FPathPointWithOrientation> BuildFinalPath(const TArray<int32>& BestOrder);

	UFUNCTION()
	void OnPathGenerationProgressUpdate(float Progress, const FString& Status);

	UFUNCTION()
	void OnPathGenerationCompleted(bool Success);

	UFUNCTION(BlueprintCallable)
	void OnGenerateOrbitFlightPath();

	//-------------------------------------------------------------------------------------
	// 异步函数

	// 进度和完成回调
	UPROPERTY(BlueprintAssignable, Category = "Path Generation")
	FOnPathGenerationProgress OnPathGenerationProgress;

	UPROPERTY(BlueprintAssignable, Category = "Path Generation")
	FOnPathGenerationComplete OnPathGenerationComplete;

	// 异步生成轨迹的入口函数
	UFUNCTION(BlueprintCallable, Category = "Path Generation")
	void GenerateOrbitFlightPathAsync();

	// 进度检查函数
	void CheckGenerationProgress();

	// 绑定和解绑路径生成事件
	void UnbindPathGenerationEvents();

	// -------------------------------------------------------------------------------------
	// 控制函数

	// 移动控制
	void OnLeftMouseClick(); // 鼠标左键点击事件
	void OnRightMouseClick(); // 鼠标右键点击事件
	void OnRightMousePressed();
	void OnRightMouseReleased();
	void MoveForward(float Value);
	void MoveRight(float Value);
	void MoveUp(float Value);

	// 镜头控制
	void Turn(float Value);
	void LookUp(float Value);

	// 调整飞行速度的函数
	void AdjustFlySpeed(float AxisValue);

	// -------------------------------------------------------------------------------------
	// 兴趣点和路径

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "InterestPoints")
	TArray<FCylindricalInterestPoint> InterestPoints;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "InterestPoints")
	FVector StartLocation; // 起点位置

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "InterestPoints")
	FVector EndLocation;   // 终点位置

	// 储存所有过渡路径
	TArray<TArray<FLinkRoute>> LinkRoutes;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Flight Path")
	TArray<FPathPointWithOrientation> GlobalPathPoints;

	TArray<float> GlobalFlightDurations;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Flight Path")
	TArray<FPathPointWithOrientation> GlobalBestViewPoints;

	//-------------------------------------------------------------------------------------
	// 测试用数据
	TArray<FPathPointWithOrientation> TestPathPoints;

	TArray<FPathPointWithOrientation> GlobalBestSplinePoints;
	TArray<FPathPointWithOrientation> GlobalBestControlPoints;
	TArray<FPathPointWithOrientation> GlobalBestKeyPoints;


	// SplineComponent 用于绘制曲线路径
	UPROPERTY(VisibleAnywhere, Category = "Spline")
	USplineComponent* SplineComponent;

	// 在 ADroneActor1 中添加一个 TArray 来存储 SplineMeshComponents
	TArray<USplineMeshComponent*> SplineMeshComponents;

	// -------------------------------------------------------------------------------------

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Flight Path")
	float fMinHeight;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Flight Path")
	float fMaxHeight;

	// 飞行路径两点之间最小距离
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Flight Path")
	float fMinDisBetwenPoints;

	// 手动移动速度系数
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement")
	float SpeedMultiplier;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Flight")
	float CurrentSpeed = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Flight")
	float Accel = 200.0f; // 加速度（units/s^2）

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Flight")
	float Decel = 200.0f; // 减速度（units/s^2）

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Flight")
	bool bIsShowGlobalPath;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Flight")
	bool bIsShowDebugPoints;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Flight")
	bool bIsShowLocalOriginalPath;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Flight")
	float fMinFlightSpeed;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Flight")
	float fMaxFlightSpeed;


	TSharedPtr<YoloObjectTracker> YoloTracker;

	TSharedPtr<NimaObjectTracker> NimaTracker;

public:
	void UpdateRotationSpeedForNextPathPoint(float minRotatSpeed = 1.0f, float maxRotateSpeed = 15.0f); // 更新到下一个航点的旋转速度

	void UpdateCameraFacing(const FVector& _InterestPoint) {
		if (!CameraComponent) return; // 确保相机组件存在

		// 计算当前位置到兴趣点的方向向量
		FVector Direction = _InterestPoint - GetActorLocation();
		Direction.Normalize();

		// 将方向向量转换为旋转
		FRotator NewRotation = Direction.Rotation();

		// 更新相机组件的旋转来朝向兴趣点
		CameraComponent->SetWorldRotation(NewRotation);
	}

	void PrecomputeAllDuration();
	void StartNewPathPoint();

	void UpdateActorRotation(float DeltaTime);

	// 绘制辅助线和圆的逻辑
	void DrawDebugHelpers(bool bIfShowDebugAreas);

	void ShowPathPoints();
	void DestroyPathPoints();

private:
	bool bIsSlowMove; // 是否慢速移动

	

	bool bIsRightMousePressed;

	float PressedTime; // 鼠标右键按下的时间

	float SingleScore; // 图像分数

	// 目前坐标系
	ACesiumGeoreference* CesiumGeoreference;

	bool bIsTrackingActive = false;
	FVector2D LastTrackedPosition;
	FString ModelPath_Yolo;
	FString ModelPath_Nima;

	float TimeSinceLastInference;
	float InferenceInterval; // 推理的时间间隔（秒）

	int32 ViewportWidth = 0;
	int32 ViewportHeight = 0;

	EInterestPointState CurrentState = EInterestPointState::Idle;
	FVector CenterPoint;
	FVector SecondPoint;
	FVector ThirdPoint;
	float Radius;
	float Height;

	APlayerController* PlayerController;

	// 飞行速度和旋转镜头速度
	float MoveSpeed;
	float RotationInterpSpeed;
	float FOVInterpSpeed;
	bool bIsFlyingAlongPath;
	// 路径点索引
	int currentIndex;
	// 控制航点绘制的标志
	bool bShouldDrawPathPoints;
	bool fGenerationFinished;

	// 控制镜头旋转
	float CurrentRotationTime = 0.0f;
	float TotalRotationDuration = 0.0f;
	// 每段飞行的起始点
	FPathPointWithOrientation OriginalPathPoint;
	FRotator StartRotation;
	FRotator TargetRotation;
	float LastRotationSpeed;
	float CurrentRotationSpeed;

private: // 权重参数
	const float LengthCostWeight = 0.5f;
	const float QualityCostWeight = 1.0f;
	const float SmoothnessCostWeight = 0.8f;

	const float AestheticScoreThreshold = 4.0f;
	const float MaxAestheticScore = 10.0f;

	const float DroneRadius = 30.0f;

private:
	//TArray<FCandidateViewpoint> CandidateViewpoints;
	/*TArray<FCandidateViewpoint> BestViewpoints;*/
	// 兴趣区域局部路径数组
	TArray<FInterestArea> InterestAreas;

	// 任务状态
	FAsyncTask<FOrbitPathGenerationTask>* PathGenerationTask;
	FTimerHandle ProgressCheckTimerHandle;
	bool bIsGeneratingPath;

	// 用于保护路径生成过程中的全局变量
	FCriticalSection PathMutex;
};

// 创建异步任务类
class FOrbitPathGenerationTask : public FNonAbandonableTask
{
public:
	FOrbitPathGenerationTask(ADroneActor1* InOwner)
		: Owner(InOwner)
	{}

	void DoWork()
	{
		// 在后台线程执行轨迹生成
		Owner->GenerateOrbitFlightPath_Internal();
	}

	FORCEINLINE TStatId GetStatId() const
	{
		RETURN_QUICK_DECLARE_CYCLE_STAT(FOrbitPathGenerationTask, STATGROUP_ThreadPoolAsyncTasks);
	}

private:
	ADroneActor1* Owner;
};
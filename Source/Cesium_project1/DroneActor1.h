// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/PlayerController.h"
#include "Kismet/GameplayStatics.h"
#include "Camera/CameraComponent.h"
#include "GameFramework/FloatingPawnMovement.h"
#include <Components/SceneCaptureComponent2D.h>
#include "MovieSceneCapture.h"
#include "Components/SplineComponent.h"
#include "Components/SplineMeshComponent.h"
#include "CesiumGlobeAnchorComponent.h"
#include "EngineUtils.h"
#include "Public/MyTSPClass.h"
#include "Public/MyUtil.h"
#include "Public/MyRRTClass.h"
#include "DroneActor1.generated.h"

// 委托声明
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FOnPathGenerationProgress, float, Progress, const FString&, Status);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnPathGenerationComplete, bool, Success);

// 前向声明
class FOrbitPathGenerationTask;
class FTraditionalOrbitGenerationTask;

UCLASS()
class CESIUM_PROJECT1_API ADroneActor1 : public APawn
{
    GENERATED_BODY()

public:
    //====================================================================================
    // 构造函数和生命周期函数
    //====================================================================================

    // 设置默认值
    ADroneActor1();

    // 游戏开始或生成时调用
    virtual void BeginPlay() override;

    // 每帧调用
    virtual void Tick(float DeltaTime) override;

    // 设置玩家输入
    virtual void SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent) override;

    //====================================================================================
    // 组件和属性
    //====================================================================================

    // 相机组件
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Camera")
    UCameraComponent* CameraComponent;

    // 场景捕捉组件
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Camera")
    USceneCaptureComponent2D* SceneCaptureComponent;

    // 渲染目标
    UPROPERTY()
    UTextureRenderTarget2D* RenderTarget;

    // 移动组件
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Movement")
    UFloatingPawnMovement* FloatingMovement;

    // Cesium组件
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Cesium")
    UCesiumGlobeAnchorComponent* GlobeAnchorComponent;

    // UI组件
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UI")
    TSubclassOf<UUserWidget> SpeedDisplayWidgetClass;

    // Spline组件
    UPROPERTY(VisibleAnywhere, Category = "Spline")
    USplineComponent* SplineComponent;

    // SplineMesh组件数组
    TArray<USplineMeshComponent*> SplineMeshComponents;

    //====================================================================================
    // 路径相关数据
    //====================================================================================

    // 兴趣点
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "InterestPoints")
    TArray<FCylindricalInterestPoint> InterestPoints;

    // 起点和终点
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "InterestPoints")
    FVector StartLocation;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "InterestPoints")
    FVector EndLocation;

    // 全局路径点
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Flight Path")
    TArray<FPathPointWithOrientation> GlobalPathPoints;

    // 飞行持续时间
    TArray<float> GlobalFlightDurations;

    // 最佳视点
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Flight Path")
    TArray<FPathPointWithOrientation> GlobalBestViewPoints;

    // 测试数据
    TArray<FPathPointWithOrientation> TestPathPoints;
    TArray<FPathPointWithOrientation> GlobalBestSplinePoints;
    TArray<FPathPointWithOrientation> GlobalBestControlPoints;
    TArray<FPathPointWithOrientation> GlobalBestKeyPoints;

    // 兴趣区域数组
    TArray<FInterestArea> InterestAreas;

    // 过渡路径
    TArray<TArray<FLinkRoute>> LinkRoutes;

    //====================================================================================
    // 飞行参数
    //====================================================================================

    // 高度限制
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Flight Path")
    float fMinHeight;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Flight Path")
    float fMaxHeight;

    // 路径点最小距离
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Flight Path")
    float fMinDisBetwenPoints;

    // 移动速度
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement")
    float SpeedMultiplier;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Flight")
    float CurrentSpeed = 0.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Flight")
    float Accel = 200.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Flight")
    float Decel = 200.0f;

    // 飞行速度范围
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Flight")
    float fMinFlightSpeed;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Flight")
    float fMaxFlightSpeed;

    // 显示选项
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Flight")
    bool bIsShowGlobalPath;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Flight")
    bool bIsShowDebugPoints;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Flight")
    bool bIsShowLocalOriginalPath;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Flight")
    bool bIsSavePhotos;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Flight")
    bool bIsSaveImageForPrediction;

    // 输入参数
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Input")
    float MouseSensitivity;

    //====================================================================================
    // 模型与跟踪
    //====================================================================================

    // YOLO模型
    TSharedPtr<YoloObjectTracker> YoloTracker;

    // NIMA模型
    TSharedPtr<NimaObjectTracker> NimaTracker;

    // 模型选择
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Model")
	FString ModelPath;

    // RRT类
    UPROPERTY()
    UMyRRTClass* GlobalRRTClass;

    //====================================================================================
    // 委托事件
    //====================================================================================

    // 路径生成进度和完成事件
    UPROPERTY(BlueprintAssignable, Category = "Path Generation")
    FOnPathGenerationProgress OnPathGenerationProgress;

    UPROPERTY(BlueprintAssignable, Category = "Path Generation")
    FOnPathGenerationComplete OnPathGenerationComplete;

    //====================================================================================
    // 基础工具函数
    //====================================================================================

	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

    virtual void BeginDestroy() override;

    // 获取视口大小
    bool GetViewportSize(int32& OutWidth, int32& OutHeight);

    // 初始化渲染目标
    void InitializeRenderTarget();

    void PrewarmNimaModelAfterCleanup();
    // 渲染视点到RenderTarget
    void RenderViewpointToRenderTarget(const FPathPointWithOrientation& Viewpoint);

    // 强制加载与禁用3DTiles
    void Force3DTilesLoad();
    void DisableForce3DTilesLoad();
    void HandleTilesetLoaded();

    //====================================================================================
    // 路径规划核心函数
    //====================================================================================

    // 路径生成入口
    UFUNCTION(BlueprintCallable, Category = "Path Generation")
    void OnGenerateOrbitFlightPath();

    // 异步生成路径
    UFUNCTION(BlueprintCallable, Category = "Path Generation")
    void GenerateOrbitFlightPathAsync();

    // 路径生成内部实现
    void GenerateOrbitFlightPath_Internal();

    // 轨道参数计算
    void CalculateOrbitParameters(const FCylindricalInterestPoint& InterestPoint,
        float& OutOrbitRadius, float& OutOrbitRadius1, float& OutOrbitRadius2,
        float& OutMinHeight, float& OutMaxHeight);

    // 预计算所有飞行持续时间
    void PrecomputeAllDuration();

    //====================================================================================
    // 视点与候选点相关函数
    //====================================================================================

    // 生成候选视点
    FPathPointWithOrientation GenerateCandidateViewpoints(
        const FPathPointWithOrientation& PathPoint,
        const FVector& PathPointLocation,
        const FRotator& OriginalRotation,
        const int32 CurrentAOIIndex,
        TArray<FCandidateViewpoint>* OutAllCandidates);

    // 计算构图代价
    float CalculateCompositionCost(FCandidateViewpoint& Candidate);

    // 计算可见性代价
    float CalculateVisibilityCost(FCandidateViewpoint& Candidate);

    // 计算动态边缘阈值
    float CalculateDynamicEdgeThreshold(const FCylindricalInterestPoint& InterestPoint);

    // 世界点投影到屏幕
    bool ProjectWorldPointToScreen(const FVector& WorldPoint, FVector2D& ScreenPoint,
        const FVector& CameraLocation, const FRotator& CameraRotation, float FOV);

    // 获取兴趣点边界点
    void GetAOIBoundingPoints(const FCylindricalInterestPoint& InterestPoint, TArray<FVector>& OutPoints);

    // 物体可见性检查
    bool IsObjectVisible(const FVector& ViewpointLocation, const FRotator& CameraRotation, const FCylindricalInterestPoint& InterestPoint);

    //====================================================================================
    // 视点选择与覆盖检查
    //====================================================================================

    // 计算视点覆盖角度
    float CalculateViewpointCoverage(FPathPointWithOrientation& Viewpoint, const FCylindricalInterestPoint& AOI);

    // 角度可见性检查
    bool IsAngleVisible_KeepNegativeAngles(
        const FPathPointWithOrientation& Viewpoint,
        float YawAngleDeg,
        const FCylindricalInterestPoint& AOI);

    // 构建可见角度范围
    TArray<FAngleRange> BuildVisibleRanges_KeepNegativeAngles(
        const TArray<float>& SortedAngles,
        float AngleStep);

    // 视点覆盖检查
    bool DoesViewpointSeeTop(
        const FPathPointWithOrientation& Viewpoint,
        const FCylindricalInterestPoint& InterestPoint);

    bool DoesViewpointSeeTopAndBottom(
        const FPathPointWithOrientation& Viewpoint,
        const FCylindricalInterestPoint& InterestPoint);

    // 选择最佳视点
    void SelectBestViewpoints(
        const TArray<FPathPointWithOrientation>& Candidates,
        TArray<FPathPointWithOrientation>& OutBestViewpoints,
        int32 NumRequired);

    // 选择最佳视点组合
    void SelectBestViewpointGroups(
        const TArray<FPathPointWithOrientation>& Candidates,
        TArray<TArray<FPathPointWithOrientation>>& OutViewpointGroups,
        int32 NumGroups = 3,
        int32 NumViewpointsPerGroup = 3);

    // 生成视点组合
    void GenerateViewpointGroups(
        const TArray<FPathPointWithOrientation>& Viewpoints,
        int32 GroupSize,
        TArray<TArray<FPathPointWithOrientation>>& OutViewpointGroups,
        int32 MaxCombinations = 400);

    // 组合生成辅助函数
    void GenerateCombinationsHelper(
        int32 NumViewpoints,
        int32 GroupSize,
        TArray<int32> CurrentCombination,
        TArray<TArray<int32>>& OutCombinations);

    // 覆盖满足检查
    bool IsCoverageSatisfied(const TArray<FPathPointWithOrientation>& ViewpointGroup);

    // 距离满足检查
    bool IsDistanceSatisfied(const TArray<FPathPointWithOrientation>& ViewpointGroup);

    // 计算组美学评分
    float CalculateGroupAestheticScore(const TArray<FPathPointWithOrientation>& ViewpointGroup);

    //====================================================================================
    // 路径生成与优化
    //====================================================================================

    // 生成B样条路径
    TArray<FPathPointWithOrientation> GenerateSplinePath(
        const TArray<FPathPointWithOrientation>& BestViewpoints,
        TArray<FPathPointWithOrientation>& OutSplinePoints,
        float MinDisBetwPoints = 500.0f);

    // 计算B样条基函数
    float CalculateBSplineBasis(int32 i, int32 k, float t, const TArray<float>& KnotVector);

    // 插值路径点属性
    void InterpolatePathPointProperties(FPathPointWithOrientation& SplinePoint,
        const TArray<FPathPointWithOrientation>& ControlPoints, float t, const TArray<float>& U);

    // 计算屏幕位置
    FVector2D CalculateScreenPosition(const FPathPointWithOrientation& PathPoint);

    // 计算方向
    FRotator CalculateOrientationFromScreenPosition(
        const FVector& CameraLocation, const FVector& AOILocation, const FVector2D& ScreenPosition, float FOV);

    // 生成控制点
    FPathPointWithOrientation GenerateControlPointTowardsNext(
        const FPathPointWithOrientation& CurrentPoint, const FPathPointWithOrientation& NextPoint,
        const FVector _PlaneNormal, float ControlPointDistanceRatio = 0.1);

    FPathPointWithOrientation GenerateControlPointTowardsPrev(
        const FPathPointWithOrientation& CurrentPoint, const FPathPointWithOrientation& PrevPoint,
        const FVector _PlaneNormal, float ControlPointDistanceRatio = 0.1);

    // 视点排序
    void SortViewpointsInWrappingOrder(TArray<FPathPointWithOrientation>& Viewpoints);
    void SortControlPointsInWrappingOrder(TArray<FPathPointWithOrientation>& ControlPoints);

    // 障碍物处理
    float GetClosestObstacleDistance(const FVector& Position,
        const TArray<FCylindricalInterestPoint>& AllInterestPoints,
        int32 CurrentAOIIndex, float DetectionRadius = 1000.0f);

    bool AdjustPathPointForObstacles(FPathPointWithOrientation& PathPoint, const FCylindricalInterestPoint& InterestPoint);

    // 碰撞检测
    bool IsPathCollisionFree(const TArray<FPathPointWithOrientation>& Path);
    bool IsPositionSafe(const FVector& Position, float SafetyRadius,
        const TArray<FCylindricalInterestPoint>& AllInterestPoints,
        int32 CurrentAOIIndex);

	// 路径安全距离分析
    void AnalyzePathSafetyDistances(const TArray<FPathPointWithOrientation>& Path);
	// 保存安全距离到CSV
    void SaveSafetyDistancesToCSV(const TArray<float>& Distances, int32 OutOfRangeCount, float MaxRange);



    //====================================================================================
    // 路径代价计算
    //====================================================================================

    // 计算路径长度
    float CalculatePathLength(const TArray<FPathPointWithOrientation>& PathPoints);

    // 路径代价计算
    float CalculateLengthCost(const TArray<FPathPointWithOrientation>& Path, float MaxLength);
    float CalculateQualityCost(const TArray<FPathPointWithOrientation>& Path);
    float CalculateSmoothnessCost(const TArray<FPathPointWithOrientation>& Path);
    double ComputeTransiPathCost(
        const TArray<FPathPointWithOrientation>& Path,
        int startIndex,
        int targetIndex,
        const FVector& PrevMidPoint,
        const FVector& NextMidPoint
    );

    // 路径参数化
    TArray<float> ParameterizePath(const TArray<FPathPointWithOrientation>& Path);

    // 节点类型检查
    bool IsEntryNode(int32 NodeIndex) const;
    bool IsExitNode(int32 NodeIndex) const;

    //====================================================================================
    // 路径段构建
    //====================================================================================

    // 构建路径段
    bool BuildAndProcessPathSegment(
        const FVector& StartPos,
        const FVector& EndPos,
        int32 StartRegionIndex,
        int32 TargetRegionIndex,
        double& OutCost,
        TArray<FPathPointWithOrientation>& OutPath,
        UMyRRTClass* LocalRRTClass
    );

    // 构建连接路径
    bool BuildLinkRoute(
        const FVector& StartPos,
        const FVector& EndPos,
        int32 StartRegionIndex,
        int32 TargetRegionIndex,
        double& OutCost,
        TArray<FPathPointWithOrientation>& OutPath,
        UMyRRTClass* LocalRRTClass
    );

    // 构建起点到区域路径
    bool BuildStartToRegionRoute(
        const FVector& StartPos,
        const FVector& EndPos,
        int32 TargetRegionIndex,
        double& OutCost,
        TArray<FPathPointWithOrientation>& OutPath,
        UMyRRTClass* LocalRRTClass
    );

    // 构建区域到终点路径
    bool BuildRegionToEndRoute(
        const FVector& StartPos,
        const FVector& EndPos,
        int32 StartRegionIndex,
        double& OutCost,
        TArray<FPathPointWithOrientation>& OutPath,
        UMyRRTClass* LocalRRTClass
    );

    // 构建STSP代价矩阵
    bool BuildSTSPCostMatrix(TArray<FDoubleArray>& OutCostMatrix);

    // 构建最终路径
    TArray<FPathPointWithOrientation> BuildFinalPath(const TArray<int32>& BestOrder);

    //====================================================================================
    // 速度计算与平滑
    //====================================================================================

    // 计算转弯角度
    float CalculateTurnAngle(const FVector& PrevPoint, const FVector& CurrentPoint, const FVector& NextPoint);

    // 计算累积转弯角度
    float CalculateCumulativeTurnAngle(int32 StartIndex, int32 LookaheadPoints);
    float CalculateCumulativeTurnAngle_BothSides(int32 CurrentIndex, int32 BackPoints, int32 FrontPoints);

    // 计算视角变化
    float CalculateViewChange(const FRotator& PrevOrientation, const FRotator& CurrentOrientation,
        const FRotator& NextOrientation, float PrevFOV, float CurrentFOV, float NextFOV);

    // 计算累积视角变化
    float CalculateCumulativeViewChange(int32 CurrentIndex, int32 LookaheadPoints);
    float CalculateCumulativeViewChange_BothSides(int32 CurrentIndex, int32 BackPointsToLook, int32 FrontPointsToLook);

    // 平滑段速度
    void SmoothSegmentSpeed(int32 Iterations);

    // 基于曲率和视角变化计算速度
    void ComputeSpeedByCurvatureAndViewChange(float MaxSpeed = 1500.0f, float MinSpeed = 500.0f,
        float SharpTurnAngle = 120.0f, float MaxViewChangeAngle = 90.0f, int32 LookaheadPoints = 3);

    // 平滑全局路径点
    void SmoothGlobalPathPoints_PositionOrientation(int32 Iterations = 2);
    
    void OptimizePathPoints();

    //====================================================================================
    // 测试与示例函数
    //====================================================================================

    // 生成传统环绕航线
    UFUNCTION(BlueprintCallable, Category = "Path Generation")
    void GenerateTraditionalOrbitPath_Internal();
    void CheckTraditionalOrbitProgress();
    void GenerateTraditionalOrbitAsync();
    int32 CalculatePointsForOrbitRadius(float Radius);
    void AdjustFOVBasedOnDistanceAndHeight(FPathPointWithOrientation& PathPoint,
        const FCylindricalInterestPoint& InterestPoint,
        float Radius, float HeightOffset);


    // 生成简单飞行路径
    UFUNCTION(BlueprintCallable, Category = "Flight Path")
    void GenerateSimpleFlightPath();

    // 生成优化飞行路径
    UFUNCTION(BlueprintCallable, Category = "Flight Path")
    void GenerateOptimizedFlightPath(const FVector& StartPoint, const FVector& EndPoint);

    // 测试候选视点生成
    FPathPointWithOrientation TestGenerateCandidateViewpoints(const FPathPointWithOrientation& PathPoint);
    void OnTestGenerateCandidateViewpoints();

    //====================================================================================
    // 输入与控制函数
    //====================================================================================

    // 鼠标控制
    void OnMouseMove(float DeltaX, float DeltaY);
    void OnLeftMouseClick();
    void OnRightMouseClick();
    void OnRightMousePressed();
    void OnRightMouseReleased();

    // 移动控制
    void MoveForward(float Value);
    void MoveRight(float Value);
    void MoveUp(float Value);

    // 视角控制
    void Turn(float Value);
    void LookUp(float Value);

    // 速度调整
    void AdjustFlySpeed(float AxisValue);

    // 更新旋转速度
    void UpdateRotationSpeedForNextPathPoint(float minRotatSpeed = 1.0f, float maxRotateSpeed = 15.0f);
    void UpdateActorRotation(float DeltaTime);

    // 更新相机朝向
    void UpdateCameraFacing(const FVector& _InterestPoint);

    //====================================================================================
    // 路径点与飞行控制
    //====================================================================================

    // 开始新路径点
    void StartNewPathPoint();

    // 开始沿路径飞行
    UFUNCTION(BlueprintCallable, Category = "Flight Path")
    void OnStartFlightAlongPath();

    // 绘制辅助线和路径点
    void DrawDebugHelpers(bool bIfShowDebugAreas);
    void ShowPathPoints();
    void DestroyPathPoints();

	// ======================================================================================
    // 手动记录路径点的函数
    UFUNCTION(BlueprintCallable, Category = "Drone|Path")
    void ToggleManualPathRecording();

    UFUNCTION(BlueprintCallable, Category = "Drone|Path")
    void RecordCurrentPositionAsPathPoint();

    UFUNCTION(BlueprintCallable, Category = "Drone|Path")
    void FinishManualPathRecording();

    //====================================================================================
    // 文件操作与其他工具函数
    //====================================================================================

    // 导出/导入路径点到WGS84格式
    void ExportPathPointsToWGS84Txt();
    bool ImportPathPointsFromTxt(const FString& LoadFilePath);

    // 读取路径文件
    UFUNCTION(BlueprintCallable, Category = "Flight Path")
    void OnReadPathFromFile();

    // 截图功能
    void OnCaptureScreenshotWithUI(bool ifWithUI = true);
    // 新增回调处理函数
    void HandleScreenshotProcessed();
    void CaptureScreenshotLowOverhead(bool ifWithUI);

	// 视频录制
	void OnRecordVideo();
    UPROPERTY(Transient)
    UMovieSceneCapture* MovieSceneCaptureInstance;

    UFUNCTION(BlueprintCallable, Category = "Recording")
    void StartRecording();

    UFUNCTION(BlueprintCallable, Category = "Recording")
    void StopRecording();

    // YOLO模型跟踪
    void OnStartTracking();

    // 预测操作
    void OnPredictAction();

    // 生成简单飞行
    UFUNCTION(BlueprintCallable, Category = "Flight Path")
    void OnGenerateSimpleFlight();

	void OnGenerateTraditionalOrbit();

    // 生成优化飞行
    UFUNCTION(BlueprintCallable, Category = "Flight Path")
    void OnGenerateOptimizedFlight();

    //====================================================================================
    // 异步任务管理
    //====================================================================================

    // 检查生成进度
    void CheckGenerationProgress();

    // 路径生成事件处理
    UFUNCTION()
    void OnPathGenerationProgressUpdate(float Progress, const FString& Status);

    UFUNCTION()
    void OnPathGenerationCompleted(bool Success);

    // 解绑路径生成事件
    void UnbindPathGenerationEvents();

private:
    //====================================================================================
    // 内部状态变量
    //====================================================================================

    // 移动状态
    bool bIsSlowMove;
    bool bIsRightMousePressed;
    float PressedTime;

    // 评分相关
    float SingleScore;

    // 地理引用
    ACesiumGeoreference* CesiumGeoreference;

    // 跟踪相关
    bool bIsTrackingActive = false;
    FVector2D LastTrackedPosition;
    FString ModelPath_Yolo;
    FString ModelPath_Nima;
    float TimeSinceLastInference;
    float InferenceInterval;

    // 视口尺寸
    int32 ViewportWidth = 0;
    int32 ViewportHeight = 0;

    // 兴趣点状态
    EInterestPointState CurrentState = EInterestPointState::Idle;
    FVector CenterPoint;
    FVector SecondPoint;
    FVector ThirdPoint;
    float Radius;
    float Height;

    // 控制器相关
    APlayerController* PlayerController;

    // 飞行控制
    float MoveSpeed;
    float RotationInterpSpeed;
    float FOVInterpSpeed;
    bool bIsFlyingAlongPath;
    int currentIndex;
    bool bShouldDrawPathPoints;
    bool fGenerationFinished;

    // 镜头旋转控制
    float CurrentRotationTime = 0.0f;
    float TotalRotationDuration = 0.0f;
    FPathPointWithOrientation OriginalPathPoint;
    FRotator StartRotation;
    FRotator TargetRotation;
    float LastRotationSpeed;
    float CurrentRotationSpeed;

    // 截图控制
    FTimerHandle ScreenshotTimerHandle;
    int32 ScreenshotCounter = 0;
    const float ScreenshotCooldown = 0.2f; // 截图冷却时间（秒）
    bool bScreenshotInProgress = false;

    // 视频录制相关
    bool bIsRecording = false;
    // 录制完成回调
    void OnRecordingFinished();

    FString RecordingSessionID;

    //====================================================================================
    // 权重参数
    //====================================================================================

    const float LengthCostWeight = 0.5f;
    const float QualityCostWeight = 1.0f;
    const float SmoothnessCostWeight = 0.8f;
    const float AestheticScoreThreshold = 4.2f;
    const float MaxAestheticScore = 10.0f;
    const float DroneRadius = 30.0f;

    //====================================================================================
    // 3DTiles加载
    //====================================================================================

    int32 PendingTilesets = 0;
    TSharedPtr<TPromise<void>> LoadPromise;

    //====================================================================================
    // 异步任务控制
    //====================================================================================

    FAsyncTask<FOrbitPathGenerationTask>* PathGenerationTask;
    FTimerHandle ProgressCheckTimerHandle;
    bool bIsGeneratingPath;

    // 传统环绕路径生成异步任务
    FAsyncTask<FTraditionalOrbitGenerationTask>* TraditionalOrbitTask;
    FTimerHandle TraditionalOrbitTimerHandle;
    bool bIsGeneratingTraditionalOrbit;

    // 互斥锁
    FCriticalSection PathMutex;

    // 模型初始化
    double WarmupStartTime;
    const float WarmupTimeout = 10.0f;

	// ====================================================================================
    // 手动记录的路径点集合
    TArray<FPathPointWithOrientation> ManualPathPoints;

    // 是否处于手动记录路径点模式
    bool bIsManualRecordingPath;

    // 记录点的最小间隔时间(秒)，防止重复记录
    // 用于模型预热的计时器句柄和相关变量
    FTimerHandle WarmupTimerHandle;
    float ManualRecordCooldown;
    float LastRecordTime;
};

// 异步任务类定义
class FOrbitPathGenerationTask : public FNonAbandonableTask
{
public:
    FOrbitPathGenerationTask(ADroneActor1* InOwner) : Owner(InOwner) {}

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


class FTraditionalOrbitGenerationTask : public FNonAbandonableTask
{
public:
    FTraditionalOrbitGenerationTask(
        ADroneActor1* InOwner)
        : Owner(InOwner)
    {
    }

    void DoWork()
    {
        if (Owner)
        {
            Owner->GenerateTraditionalOrbitPath_Internal();
        }
    }

    FORCEINLINE TStatId GetStatId() const
    {
        RETURN_QUICK_DECLARE_CYCLE_STAT(FTraditionalOrbitGenerationTask, STATGROUP_ThreadPoolAsyncTasks);
    }

private:
    ADroneActor1* Owner;
};

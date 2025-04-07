// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "MyUtil.h"


#include "MyRRTClass.generated.h"

struct RRTNode
{
    FVector Point;
    RRTNode* Parent;
    double Cost;

    RRTNode(const FVector& p, RRTNode* parent, double cost)
        : Point(p), Parent(parent), Cost(cost) {}
};

struct FBiRRTMetrics
{
    float ForwardTreeSize = 0;
    float BackwardTreeSize = 0;
    float BestBridgeDistance = TNumericLimits<float>::Max();
    float LastImprovementIteration = 0;
    int32 ConnectionAttempts = 0;
    int32 SuccessfulConnections = 0;
};

class FBiRRTTree
{
public:
    TArray<RRTNode*> ForwardTree;
    TArray<RRTNode*> BackwardTree;
    FVector StartLocation;
    FVector GoalLocation;
    FCriticalSection TreeLock;
    FBiRRTMetrics Metrics;

    // 默认构造函数
    FBiRRTTree()
        : StartLocation(FVector::ZeroVector)
        , GoalLocation(FVector::ZeroVector)
    {
    }

    FBiRRTTree(const FVector& Start, const FVector& Goal)
        : StartLocation(Start)
        , GoalLocation(Goal)
    {
        Initialize(Start, Goal);
    }

    ~FBiRRTTree()
    {
        for (auto* Node : ForwardTree) delete Node;
        for (auto* Node : BackwardTree) delete Node;
    }

    // 初始化方法
    void Initialize(const FVector& Start, const FVector& Goal)
    {
        StartLocation = Start;
        GoalLocation = Goal;
        ForwardTree.Add(new RRTNode(Start, nullptr, 0.0));
        BackwardTree.Add(new RRTNode(Goal, nullptr, 0.0));
    }
};

/**
 * 
 */
UCLASS()
class CESIUM_PROJECT1_API UMyRRTClass : public UObject
{
	GENERATED_BODY()

public:	
    UMyRRTClass();
    ~UMyRRTClass();

public:
    void SetWorld(UWorld* InWorld) { World = InWorld; }

    UFUNCTION(BlueprintCallable, Category = "RRT")
    void AddNode(const FPathPointWithOrientation& NewNode);

    UFUNCTION(BlueprintCallable, Category = "RRT")
    FPathPointWithOrientation GetRandomNode();

    UFUNCTION(BlueprintCallable, Category = "RRT")
    FPathPointWithOrientation GetNearestNode(const FPathPointWithOrientation& Node);

    UFUNCTION(BlueprintCallable, Category = "RRT")
    TArray<FPathPointWithOrientation> GenerateRRTPath(const FVector& StartLocation, 
        const FVector& EndLocation, const TArray<FCylindricalInterestPoint>& InterestPoints);

    UFUNCTION(BlueprintCallable, Category = "RRT")
    float ComputePathLength(const TArray<FVector>& Path);

    UFUNCTION(BlueprintCallable, Category = "RRT")
    TArray<float> ComputeCurvature(const TArray<FVector>& Path);

	// 三次样条插值
    UFUNCTION(BlueprintCallable, Category = "RRT")
    void CubicSplineInterpolation(const TArray<float>& InputX, const TArray<float>& InputY, const TArray<float>& OutputX, TArray<float>& OutputY);

	// B样条插值
    UFUNCTION(BlueprintCallable, Category = "RRT")
    FVector EvaluateBSplinePoint(int32 Degree, const TArray<FVector>& ControlPoints, const TArray<float>& KnotVector, float T);

    UFUNCTION(BlueprintCallable, Category = "RRT")
    TArray<FVector> SmoothPathWithBSplineDynamic(const TArray<FVector>& Path, int32 Degree, int32 NumControlPoints, float Tension);


    UFUNCTION(BlueprintCallable, Category = "RRT")
    TArray<FVector> SmoothPathWithExtraConstraints(
        const TArray<FVector>& Path,
        int32 Degree,
        int32 NumControlPoints,
        float Tension,
        const FVector& StartLocation,
        const FVector& ExtraStartLocation,
        const FVector& EndLocation,
        const FVector& ExtraEndLocation
    );

    UFUNCTION(BlueprintCallable, Category = "RRT")
    TArray<FVector> InterpolateSegment(const FVector& P1, const FVector& P2, float Threshold);

    bool IsPointInCylinder(const FVector& Point, const FCylindricalInterestPoint& Cylinder) const;

	// 根据最小距离过滤路径
    TArray<FVector> FilterPathByMinDistance(const TArray<FVector>& Path, float MinDisBetweenPoints);
    TArray<FVector> MergeSegment(const TArray<FVector>& Segment, float MinDisBetweenPoints);
    TArray<FVector> FilterPathByMinDistanceWithCollisionCheck(
        const TArray<FVector>& Path,
        float MinDisBetweenPoints,
        const TArray<FCylindricalInterestPoint>& InterestPoints);

    /**
     * 检查给定的轨迹是否与任何障碍物发生碰撞。
     *
     * @param Trajectory 需要检查的轨迹点数组。
     * @param Obstacles 圆柱形障碍物的数组。
     * @param Threshold = 0.5f 采样点之间的距离阈值。
     * @return 如果轨迹没有与任何障碍物发生碰撞，则返回true；否则返回false。
     */
    UFUNCTION(BlueprintCallable, Category = "RRT")
    bool IsTrajectoryCollisionFree(const TArray<FVector>& Trajectory, const TArray<FCylindricalInterestPoint>& Obstacles, float Threshold = 10.0f);

    bool CheckCollisionBatch(UWorld* World, const TArray<FVector>& Points,
        int32 StartIndex, int32 EndIndex, float Threshold);

    /*UFUNCTION(BlueprintCallable, Category = "RRT")
    TArray<FVector> SmoothAndValidatePath(const TArray<FVector>& Path, const TArray<FCylindricalInterestPoint>& Obstacles, int32 MaxRetries = 20);*/

    /// <summary>
    /// Generates and smooths a Rapidly-exploring Random Tree (RRT) path between the start and end locations.
    /// </summary>
    /// <param name="StartLocation">The starting location of the path.</param>
    /// <param name="EndLocation">The ending location of the path.</param>
    /// <param name="InterestPoints">A list of cylindrical interest points that the path should consider.</param>
    /// <param name="ExtraStartPoint">An optional extra start point for the path.</param>
    /// <param name="ExtraEndPoint">An optional extra end point for the path.</param>
    /// <param name="MinDisBetweenPoints">The minimum distance between points in the path.</param>
    /// <param name="MaxRetries">The maximum number of retries for smoothing the path.</param>
    /// <param name="BaseStepSize">The base step size for the RRT algorithm.</param>
    /// <param name="CurvatureFactor">The curvature factor for smoothing the path.</param>
    /// <param name="MaxSegmentLength">The maximum segment length for the path.</param>
    /// <param name="MinPoints">The minimum number of points in the path.</param>
    /// <param name="StepSize">The step size for the RRT algorithm.</param>
    /// <param name="NeighborRadius">The neighbor radius for the RRT algorithm.</param>
    /// <returns>A smoothed path represented as an array of FPathPointWithOrientation.</returns>
	UFUNCTION(BlueprintCallable, Category = "RRT")
    TArray<FPathPointWithOrientation> GenerateAndSmoothRRTPath(
        const FVector& StartLocation,
        const FVector& EndLocation,
        const TArray<FCylindricalInterestPoint>& InterestPoints,
        TArray<FVector>& TestPoints, // 默认空值
        const FVector& ExtraStartPoint = FVector::ZeroVector,
        const FVector& ExtraEndPoint = FVector::ZeroVector,
        float MinDisBetweenPoints = 500.0f,
        int32 MaxRetries =10,
        float CurvatureFactor =3.0f,
        float MaxSegmentLength =200.0f,
        int32 MinPoints =10,
        double StepSize = 500.0f,
        double NeighborRadius =20000.0f
    );

    // RRT 算法
    TArray<FPathPointWithOrientation> GenerateAndSmoothRRTPath_BackUp(
        const FVector& StartLocation,
        const FVector& EndLocation,
        const TArray<FCylindricalInterestPoint>& InterestPoints,
        int32 MaxRetries =5,
        float BaseStepSize =50.0f,
        float CurvatureFactor =1.0f,
        float MaxSegmentLength =200.0f,
        int32 MinPoints =10
    );

private:

    UWorld* World;

    UPROPERTY()
    TArray<FPathPointWithOrientation> Nodes;

    bool IsInObstacleLocal(const FVector& Point, const TArray<FCylindricalInterestPoint>& Obstacles,
        float Threshold = 10.0f) const;
    bool LineIntersectsObstacles(const FVector& Start, const FVector& End,
        const TArray<FCylindricalInterestPoint>& Obstacles,
        float Threshold = 10.0f) const;
    float ComputeLineObstacleDistance(const FVector& Start, const FVector& End,
        const FCylindricalInterestPoint& Obstacle) const;

    void SolveTridiagonalSystem(const TArray<float>& A, const TArray<float>& B, const TArray<float>& C, TArray<float>& D, TArray<float>& X);

    // -------------------------------------双向RRT*算法--------------------------------------------------------
    RRTNode* ExtendBiRRTTree(TArray<RRTNode*>& ActiveTree, const FVector& Target,
        double StepSize, const TArray<FCylindricalInterestPoint>& Obstacles, double NeighborRadius);

    TArray<FVector> ExtractPath(RRTNode* ForwardNode, RRTNode* BackwardNode, bool bReverse);
    /*void OptimizeTreeConnection(TArray<RRTNode*>& TreeA, TArray<RRTNode*>& TreeB,
        const TArray<FCylindricalInterestPoint>& Obstacles);*/


    // 
    FVector HeuristicSample(
        const TArray<RRTNode*>& ForwardTree,
        const TArray<RRTNode*>& BackwardTree,
        const FBox& SearchSpace,
        float GoalBiasProb,
        const FVector& StartLocation,
        const FVector& EndLocation);

    double GetAdaptiveConnectThreshold(
        double BaseThreshold,
        int32 CurrentIter,
        int32 MaxIter,
        float CurrentSuccessRate);


    // 搜索状态跟踪
    int32 TotalIterations;
    int32 CurrentIteration;
    float SuccessRate;
    int32 SuccessfulExtensions;

    // 上一次连接尝试的距离
    double LastConnectDistance;

};
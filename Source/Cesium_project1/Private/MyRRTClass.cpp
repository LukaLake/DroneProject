// Fill out your copyright notice in the Description page of Project Settings.


#include "MyRRTClass.h"
#include "Kismet/GameplayStatics.h"
#include "Engine/World.h"
#include "CollisionQueryParams.h"
#include "Engine/StaticMeshActor.h"
#include "Async/ParallelFor.h"
#include "HAL/CriticalSection.h"


int NearestNodeIndex(const TArray<RRTNode*>& Tree, const FVector& RandPoint)
{
    double MinDist = TNumericLimits<double>::Max();
    int NearestIndex = -1;
    for (int i = 0; i < Tree.Num(); ++i)
    {
        double dist = FVector::DistSquared(Tree[i]->Point, RandPoint);
        if (dist < MinDist)
        {
            MinDist = dist;
            NearestIndex = i;
        }
    }
    return NearestIndex;
}

TArray<int> RadiusSearch(const TArray<RRTNode*>& Tree, const FVector& NewPoint, double Radius)
{
    TArray<int> Result;
    double RadiusSq = Radius * Radius;
    for (int i = 0; i < Tree.Num(); ++i)
    {
        double dist = FVector::DistSquared(Tree[i]->Point, NewPoint);
        if (dist <= RadiusSq)
        {
            Result.Add(i);
        }
    }
    return Result;
}

TArray<FPathPointWithOrientation> BacktrackRRTPath(RRTNode* goalNode)
{
    TArray<FPathPointWithOrientation> Result;
    RRTNode* current = goalNode;
    while (current != nullptr)
    {
        FPathPointWithOrientation WP;
        WP.Point = current->Point;
        WP.Orientation = FRotator::ZeroRotator;
        WP.FOV = 90.0f;
        WP.AestheticScore = 0.0f;
        WP.CoverageAngle = 0.0f;
        WP.AOIIndex = -1;
        WP.AngleRange = FAngleRange();
        Result.Insert(WP, 0);
        current = current->Parent;
    }
    return Result;
}


UMyRRTClass::UMyRRTClass()
{
}

UMyRRTClass::~UMyRRTClass()
{
}

void UMyRRTClass::AddNode(const FPathPointWithOrientation& NewNode)
{
    Nodes.Add(NewNode);
}

FPathPointWithOrientation UMyRRTClass::GetRandomNode()
{
    int32 RandomIndex = FMath::RandRange(0, Nodes.Num() - 1);
    return Nodes[RandomIndex];
}

FPathPointWithOrientation UMyRRTClass::GetNearestNode(const FPathPointWithOrientation& Node)
{
    float MinDistance = TNumericLimits<float>::Max();
    FPathPointWithOrientation NearestNode;

    for (const FPathPointWithOrientation& ExistingNode : Nodes)
    {
        float Distance = FVector::Distance(Node.Point, ExistingNode.Point);
        if (Distance < MinDistance)
        {
            MinDistance = Distance;
            NearestNode = ExistingNode;
        }
    }

    return NearestNode;
}

TArray<FPathPointWithOrientation> UMyRRTClass::GenerateRRTPath(const FVector& StartLocation, 
    const FVector& EndLocation, const TArray<FCylindricalInterestPoint>& InterestPoints)
{
    Nodes.Empty();

    FPathPointWithOrientation StartNode;
    StartNode.Point = StartLocation;
    // 设置起点的方向、FOV和其他属性...
    AddNode(StartNode);

    // 计算起点和终点的包围盒
    FBox BoundingBox(StartLocation, EndLocation);

    // 计算搜索空间的边界,适当扩大包围盒
    float Expansion = FVector::Distance(BoundingBox.Min, BoundingBox.Max) / 5.0f;
    FVector BoxMin = BoundingBox.Min - FVector(Expansion);
    FVector BoxMax = BoundingBox.Max + FVector(Expansion);
    FBox SearchSpace(BoxMin, BoxMax);

    // 自动设置参数
    float StepSize =  100.0f;
    int32 MaxIterations = FMath::Max(10 * FVector::Distance(StartLocation, EndLocation), 5000);
    float GoalBiasProb = FMath::Clamp(1.0f / (1.0f + InterestPoints.Num()), 0.05f, 0.2f);

    for (int32 i = 0; i < MaxIterations; ++i)
    {
        FPathPointWithOrientation RandomNode;

        // 以一定概率直接选择目标点作为随机节点
        if (FMath::FRand() < GoalBiasProb)
        {
            RandomNode.Point = EndLocation;
        }
        else
        {
            RandomNode.Point = FMath::RandPointInBox(SearchSpace);
        }

        FPathPointWithOrientation NearestNode = GetNearestNode(RandomNode);

        FVector Direction = (RandomNode.Point - NearestNode.Point).GetSafeNormal();
        FPathPointWithOrientation NewNode;
        NewNode.Point = NearestNode.Point + Direction * StepSize;
        // 设置新节点的方向、FOV和其他属性...

        // 检查新节点是否与兴趣点发生碰撞
        bool bCollision = false;
        for (const FCylindricalInterestPoint& InterestPoint : InterestPoints)
        {
            if (IsPointInCylinder(NewNode.Point, InterestPoint))
            {
                bCollision = true;
                break;
            }
        }

        if (!bCollision)
        {
            AddNode(NewNode);
        }

        if (FVector::Distance(NewNode.Point, EndLocation) <= StepSize)
        {
            FPathPointWithOrientation EndNode;
            EndNode.Point = EndLocation;
            // 设置终点的方向、FOV和其他属性...
            AddNode(EndNode);
            break;
        }
    }

    return Nodes;
}

float UMyRRTClass::ComputePathLength(const TArray<FVector>& Path)
{
    float TotalLength = 0.0f;
    for (int32 i = 1; i < Path.Num(); ++i)
    {
        TotalLength += FVector::Distance(Path[i], Path[i - 1]);
    }
    return TotalLength;
}

TArray<float> UMyRRTClass::ComputeCurvature(const TArray<FVector>& Path)
{
    int32 n = Path.Num();
    TArray<float> Curvature;
    Curvature.Init(0.0f, n);

    for (int32 i = 1; i < n - 1; ++i)
    {
        FVector PPrev = Path[i - 1];
        FVector PCurr = Path[i];
        FVector PNext = Path[i + 1];

        FVector DP1 = PPrev - PCurr;
        FVector DP2 = PNext - PCurr;

        FVector CrossProduct = FVector::CrossProduct(DP1, DP2);

        float NormCross = CrossProduct.Size();
        float NormDP1 = DP1.Size();
        float NormDP2 = DP2.Size();

        Curvature[i] = NormCross / (NormDP1 * NormDP2 + 1e-8f);
    }

    return Curvature;
}

void UMyRRTClass::SolveTridiagonalSystem(const TArray<float>& A, const TArray<float>& B, const TArray<float>& C, TArray<float>& D, TArray<float>& X)
{
    int32 n = D.Num();
    TArray<float> CStar;
    TArray<float> DStar;
    CStar.Init(0.0f, n);
    DStar.Init(0.0f, n);

    CStar[0] = C[0] / B[0];
    DStar[0] = D[0] / B[0];

    for (int32 i = 1; i < n; ++i)
    {
        float m = B[i] - A[i] * CStar[i - 1];
        CStar[i] = C[i] / m;
        DStar[i] = (D[i] - A[i] * DStar[i - 1]) / m;
    }

    X[n - 1] = DStar[n - 1];
    for (int32 i = n - 2; i >= 0; --i)
    {
        X[i] = DStar[i] - CStar[i] * X[i + 1];
    }
}

void UMyRRTClass::CubicSplineInterpolation(const TArray<float>& InputX, const TArray<float>& InputY, const TArray<float>& OutputX, TArray<float>& OutputY)
{
    int32 NumInputPoints = InputX.Num();
    TArray<float> a, b, c, d, h;
    a.Init(0.0f, NumInputPoints);
    b.Init(0.0f, NumInputPoints);
    c.Init(0.0f, NumInputPoints);
    d.Init(0.0f, NumInputPoints);
    h.Init(0.0f, NumInputPoints - 1);

    for (int32 i = 0; i < NumInputPoints - 1; ++i)
    {
        h[i] = InputX[i + 1] - InputX[i];
    }

    for (int32 i = 1; i < NumInputPoints - 1; ++i)
    {
        a[i] = h[i - 1];
        b[i] = 2.0f * (h[i - 1] + h[i]);
        c[i] = h[i];
        d[i] = 3.0f * ((InputY[i + 1] - InputY[i]) / h[i] - (InputY[i] - InputY[i - 1]) / h[i - 1]);
    }

    b[0] = 1.0f;
    b[NumInputPoints - 1] = 1.0f;
    d[0] = 0.0f;
    d[NumInputPoints - 1] = 0.0f;

    TArray<float> SecondDerivatives;
    SecondDerivatives.Init(0.0f, NumInputPoints);
    SolveTridiagonalSystem(a, b, c, d, SecondDerivatives);

    for (int32 i = 0; i < NumInputPoints - 1; ++i)
    {
        float hi = h[i];
        a[i] = InputY[i];
        b[i] = (InputY[i + 1] - InputY[i]) / hi - hi * (SecondDerivatives[i + 1] + 2.0f * SecondDerivatives[i]) / 3.0f;
        d[i] = (SecondDerivatives[i + 1] - SecondDerivatives[i]) / (3.0f * hi);
        c[i] = SecondDerivatives[i];
    }

    OutputY.Empty();
    for (int32 i = 0; i < NumInputPoints - 1; ++i)
    {
        float xi = InputX[i];
        float xi1 = InputX[i + 1];
        float hi = h[i];

        for (int32 j = 0; j < OutputX.Num(); ++j)
        {
            float x = OutputX[j];
            if (x >= xi && x <= xi1)
            {
                float t = (x - xi) / hi;
                float y = a[i] + b[i] * (x - xi) + c[i] * FMath::Pow(x - xi, 2) + d[i] * FMath::Pow(x - xi, 3);
                OutputY.Add(y);
            }
        }
    }

    OutputY.Add(InputY.Last());
}


FVector UMyRRTClass::EvaluateBSplinePoint(int32 Degree, const TArray<FVector>& ControlPoints, const TArray<float>& KnotVector, float T)
{
    const int32 n = ControlPoints.Num();
    const int32 k = Degree;

    // 找到参数T所属的节点区间
    int32 span = -1;
    for (int32 i = k; i < n; ++i)
    {
        if (T >= KnotVector[i] && T <= KnotVector[i + 1])
        {
            span = i;
            break;
        }
    }

    // 超出范围的情况处理
    if (span == -1)
    {
        if (T <= KnotVector[k])
        {
            return ControlPoints[0];
        }
        else
        {
            return ControlPoints.Last();
        }
    }

    // De Boor算法
    TArray<FVector> d;
    d.SetNumUninitialized(n);
    for (int32 i = 0; i < n; ++i)
    {
        d[i] = ControlPoints[i];
    }

    for (int32 r = 1; r <= k; ++r)
    {
        for (int32 i = span; i >= span - k + r; --i)
        {
            float alpha = (T - KnotVector[i]) / (KnotVector[i + k + 1 - r] - KnotVector[i]);
            alpha = FMath::Clamp(alpha, 0.0f, 1.0f);
            d[i] = (1.0f - alpha) * d[i - 1] + alpha * d[i];
        }
    }

    return d[span];
}


TArray<FVector> UMyRRTClass::SmoothPathWithBSplineDynamic(const TArray<FVector>& Path, int32 Degree, int32 NumControlPoints, float Tension)
{
    if (Path.Num() < 2 || NumControlPoints < 2)
    {
        return Path;
    }

    // 1. 基于弦长进行参数化
    TArray<float> Distances;
    Distances.Reserve(Path.Num());
    Distances.Add(0.0f);
    for (int32 i = 1; i < Path.Num(); ++i)
    {
        float segLength = FVector::Distance(Path[i], Path[i - 1]);
        Distances.Add(Distances[i - 1] + segLength);
    }

    float TotalLength = Distances.Last();
    for (float& val : Distances)
    {
        val /= TotalLength;
    }

    // 2. 构建初步控制点 (Raw Control Points)
    TArray<FVector> RawControlPoints;
    RawControlPoints.Reserve(NumControlPoints);
    float Increment = 1.0f / (NumControlPoints - 1);
    for (int32 i = 0; i < NumControlPoints; ++i)
    {
        float tVal = i * Increment;
        int32 Index = Algo::UpperBound(Distances, tVal) - 1;
        Index = FMath::Clamp(Index, 0, Distances.Num() - 2);
        float LocalAlpha = (tVal - Distances[Index]) / (Distances[Index + 1] - Distances[Index]);
        FVector RawPos = FMath::Lerp(Path[Index], Path[Index + 1], LocalAlpha);
        RawControlPoints.Add(RawPos);
    }

    // 3. 使用Tension对控制点进行二次平滑处理
    //    当Tension > 1时，说明可以更加平滑，尝试将控制点稍微拉向前后控制点的中点
    //    当Tension < 1时，更忠实原路径，不做或者少做平滑处理。
    //    当Tension = 1时，为标准B样条无额外偏移。
    TArray<FVector> ControlPoints;
    ControlPoints.SetNum(NumControlPoints);

    for (int32 i = 0; i < NumControlPoints; ++i)
    {
        FVector CP = RawControlPoints[i];

        // 邻点平均
        FVector PrevPos = (i > 0) ? RawControlPoints[i - 1] : RawControlPoints[i];
        FVector NextPos = (i < NumControlPoints - 1) ? RawControlPoints[i + 1] : RawControlPoints[i];
        FVector Mid = (PrevPos + NextPos) * 0.5f;

        // 定义Tension映射策略
        // 当Tension > 1时，让CP向Mid插值
        // 当Tension < 1时，向CP自身靠拢，不做额外平滑
        float ExtraSmoothFactor = (Tension > 1.0f) ? (Tension - 1.0f) : 0.0f;
        // 根据项目需求可进一步调优ExtraSmoothFactor
        CP = FMath::Lerp(CP, Mid, ExtraSmoothFactor);

        ControlPoints[i] = CP;
    }

    // 4. 构建Knot Vector （B样条标准节点向量）
    TArray<float> KnotVector;
    KnotVector.Reserve(NumControlPoints + Degree + 1);

    for (int32 i = 0; i <= Degree; ++i)
    {
        KnotVector.Add(0.0f);
    }

    int32 InnerKnots = NumControlPoints - Degree - 1;
    for (int32 i = 1; i <= InnerKnots; ++i)
    {
        float kv = (float)i / (InnerKnots + 1);
        KnotVector.Add(kv);
    }

    for (int32 i = 0; i <= Degree; ++i)
    {
        KnotVector.Add(1.0f);
    }

    // 5. 生成平滑路径点
    int32 NumPoints = FMath::Max(10, Path.Num());
    TArray<FVector> SmoothedPath;
    SmoothedPath.Reserve(NumPoints);
    float TStep = 1.0f / (NumPoints - 1);

    for (int32 i = 0; i < NumPoints; ++i)
    {
        float CurrentT = i * TStep;
        FVector Pt = EvaluateBSplinePoint(Degree, ControlPoints, KnotVector, CurrentT);
        SmoothedPath.Add(Pt);
    }

    return SmoothedPath;
}


TArray<FVector> UMyRRTClass::SmoothPathWithExtraConstraints(
    const TArray<FVector>& Path,
    int32 Degree,
    int32 NumControlPoints,
    float Tension,
    const FVector& StartLocation,
    const FVector& ExtraStartLocation,
    const FVector& EndLocation,
    const FVector& ExtraEndLocation
) {
    if (Path.Num() < 2 || NumControlPoints < 2) {
        return Path;
    }

    // 1. 基于弦长进行参数化
    TArray<float> Distances;
    Distances.Reserve(Path.Num());
    Distances.Add(0.0f);
    for (int32 i = 1; i < Path.Num(); ++i) {
        float segLength = FVector::Distance(Path[i], Path[i - 1]);
        Distances.Add(Distances[i - 1] + segLength);
    }

    float TotalLength = Distances.Last();
    for (float& val : Distances) {
        val /= TotalLength;
    }

    // 2. 构建控制点
    TArray<FVector> ControlPoints;
    ControlPoints.Reserve(NumControlPoints);

    // 增强起点权重
    ControlPoints.Add(StartLocation);
    if (!ExtraStartLocation.IsZero()) {
        FVector DirectionStart = (StartLocation - ExtraStartLocation).GetSafeNormal();
        FVector ConstrainedStart = StartLocation + DirectionStart * (TotalLength * 0.05f);
        ControlPoints.Add(ConstrainedStart);
    }
    else {
        ControlPoints.Add(StartLocation);
    }

    // 中间点（从 Path 中选取）
    float Increment = 1.0f / (NumControlPoints - 1);
    for (int32 i = 0; i < NumControlPoints; ++i) {
        float tVal = i * Increment;
        int32 Index = Algo::UpperBound(Distances, tVal) - 1;
        Index = FMath::Clamp(Index, 0, Distances.Num() - 2);
        float LocalAlpha = (tVal - Distances[Index]) / (Distances[Index + 1] - Distances[Index]);
        FVector RawPos = FMath::Lerp(Path[Index], Path[Index + 1], LocalAlpha);
        ControlPoints.Add(RawPos);
    }

    // 增强终点权重
    if (!ExtraEndLocation.IsZero()) {
        FVector DirectionEnd = (ExtraEndLocation - EndLocation).GetSafeNormal();
        FVector ConstrainedEnd = EndLocation + DirectionEnd * (TotalLength * 0.05f);
        ControlPoints.Add(ConstrainedEnd);
    }
    else {
        ControlPoints.Add(EndLocation);
    }
    ControlPoints.Add(EndLocation);

    // 3. 构建 Knot Vector（B样条标准节点向量）
    TArray<float> KnotVector;
    KnotVector.Reserve(ControlPoints.Num() + Degree + 1);

    // 起点和终点的权重增加（节点重复）
    for (int32 i = 0; i <= Degree; ++i) {
        KnotVector.Add(0.0f);
    }

    int32 InnerKnots = ControlPoints.Num() - Degree - 1;
    for (int32 i = 1; i <= InnerKnots; ++i) {
        float kv = (float)i / (InnerKnots + 1);
        KnotVector.Add(kv);
    }

    for (int32 i = 0; i <= Degree; ++i) {
        KnotVector.Add(1.0f);
    }

    // 4. 生成平滑路径点
    float LocalTotalLength = 0.0f;
    for (int32 i = 1; i < Path.Num(); ++i) {
        LocalTotalLength += FVector::Dist(Path[i - 1], Path[i]);
    }
    // 根据总长度分配NumSplinePoints
    const int32 NumPoints = FMath::Max(10, FMath::RoundToInt(LocalTotalLength / 200.0f)); // 例如，每100单位长度分配一个点
    //int32 NumPoints = FMath::Max(10, Path.Num() * 2);

    // 生成B样条曲线点
    TArray<FVector> SmoothedPath;
    SmoothedPath.Reserve(NumPoints);
    float TStep = 1.0f / (NumPoints - 1);

    for (int32 i = 0; i < NumPoints; ++i) {
        float CurrentT = i * TStep;
        FVector Pt = EvaluateBSplinePoint(Degree, ControlPoints, KnotVector, CurrentT);
        SmoothedPath.Add(Pt);
    }

    return SmoothedPath;
}






TArray<FVector> UMyRRTClass::InterpolateSegment(const FVector& P1, const FVector& P2, float Threshold)
{
    float Distance = FVector::Distance(P2, P1);
    int32 NumPoints = FMath::Max(static_cast<int32>(Distance / Threshold), 1);
    TArray<FVector> InterpolatedPoints;
    InterpolatedPoints.Reserve(NumPoints + 1);

    FVector Direction = (P2 - P1).GetSafeNormal();
    float StepSize = Distance / NumPoints;
    for (int32 i = 0; i <= NumPoints; ++i)
    {
        InterpolatedPoints.Add(P1 + Direction * (i * StepSize));
    }

    return InterpolatedPoints;
}

// 检查点是否在圆柱体内
bool UMyRRTClass::IsPointInCylinder(const FVector& Point, const FCylindricalInterestPoint& Cylinder) const
{
    // 计算点相对于圆柱体底部中心的向量
    FVector CylinderToPoint = Point - Cylinder.BottomCenter;

    // 计算点在 XY 平面的欧几里得距离（忽略 Z 轴）
    float DistanceXY = FVector::DistXY(Point, Cylinder.BottomCenter);

    // 计算点在 Z 轴上的高度差
    float DistanceZ = CylinderToPoint.Z;

    // 检查点是否在圆柱体的半径加上安全距离内
    bool bWithinRadius = DistanceXY <= (Cylinder.Radius + Cylinder.MinSafetyDistance);

    // 检查点是否在圆柱体的高度范围内
    bool bWithinHeight = (DistanceZ >= 0.0f) && (DistanceZ <= Cylinder.Height);

    // 点在圆柱体内且满足高度范围
    return bWithinRadius && bWithinHeight;
}


//bool UMyRRTClass::IsTrajectoryCollisionFree(const TArray<FVector>& Trajectory,
//    const TArray<FCylindricalInterestPoint>& Obstacles,
//    float Threshold)
//{
//    // 使用 TWeakObjectPtr 安全地引用 World
//    TWeakObjectPtr<UWorld> WeakWorld = World;
//
//    if (!WeakWorld.IsValid())
//    {
//        UE_LOG(LogTemp, Warning, TEXT("World is nullptr"));
//        return false;
//    }
//
//    std::atomic<bool> bCollisionDetected(false);
//    FCriticalSection WorldMutex; // 用于保护对 World 的访问
//
//    // 遍历轨迹段
//    for (int32 i = 1; i < Trajectory.Num() && !bCollisionDetected.load(); ++i)
//    {
//        FVector StartPoint = Trajectory[i - 1];
//        FVector EndPoint = Trajectory[i];
//        FVector Segment = EndPoint - StartPoint;
//        float SegmentLength = Segment.Size();
//
//        if (SegmentLength == 0.0f)
//        {
//            continue;
//        }
//
//        int32 NumSteps = FMath::CeilToInt(SegmentLength / Threshold);
//
//        // 存储采样点
//        TArray<FVector> SamplePoints;
//        SamplePoints.Reserve(NumSteps + 1);
//
//        // 使用 ParallelFor 生成采样点并检测圆柱体碰撞
//        ParallelFor(NumSteps + 1, [&](int32 Step)
//            {
//                if (bCollisionDetected.load())
//                {
//                    return;
//                }
//
//                float t = static_cast<float>(Step) / static_cast<float>(NumSteps);
//                FVector SamplePoint = FMath::Lerp(StartPoint, EndPoint, t);
//
//                // 检查与圆柱体的碰撞
//                for (const FCylindricalInterestPoint& Obstacle : Obstacles)
//                {
//                    if (IsPointInCylinder(SamplePoint, Obstacle))
//                    {
//                        bCollisionDetected.store(true);
//                        UE_LOG(LogTemp, Warning, TEXT("Collision with Cylinder Detected"));
//                        return;
//                    }
//                }
//
//                // 将采样点加入列表
//                SamplePoints.Add(SamplePoint);
//            });
//
//        // 如果在并行过程中检测到圆柱体碰撞，则提前退出
//        if (bCollisionDetected.load())
//        {
//            UE_LOG(LogTemp, Warning, TEXT("Collision detected during parallel processing."));
//            return false;
//        }
//
//        // 批量调度物理碰撞检测到主线程
//        TSharedRef<TPromise<bool>> Promise = MakeShared<TPromise<bool>>();
//        TFuture<bool> Future = Promise->GetFuture();
//
//        AsyncTask(ENamedThreads::GameThread, [WeakWorld, Promise, SamplePoints, Threshold, &WorldMutex]() mutable
//            {
//                bool CollisionDetected = false;
//
//                // 使用 FScopeLock 保护对 World 的访问
//                FScopeLock Lock(&WorldMutex);
//
//                if (!WeakWorld.IsValid())
//                {
//                    UE_LOG(LogTemp, Error, TEXT("World is no longer valid in AsyncTask"));
//                    Promise->SetValue(false); // 如果 World 无效，直接返回 false
//                    return;
//                }
//
//                UWorld* World = WeakWorld.Get();
//                for (const FVector& SamplePoint : SamplePoints)
//                {
//                    if (CollisionDetected)
//                    {
//                        break;
//                    }
//
//                    FHitResult HitResult;
//                    FVector TraceStart = SamplePoint - FVector(1.0f, 1.0f, 1.0f) * Threshold;
//                    FVector TraceEnd = SamplePoint + FVector(1.0f, 1.0f, 1.0f) * Threshold;
//
//                    FCollisionQueryParams LocalQueryParams;
//                    float SphereRadius = 200.0f; // 设置球体半径
//                    if (World->SweepSingleByChannel(HitResult,
//                        SamplePoint, SamplePoint, FQuat::Identity, 
//                        ECC_WorldDynamic, FCollisionShape::MakeSphere(SphereRadius), LocalQueryParams))
//                    {
//                        CollisionDetected = true;
//                        //UE_LOG(LogTemp, Warning, TEXT("Collision with World detected at point: %s"), *SamplePoint.ToString());
//                        break;
//                    }
//                }
//
//                Promise->SetValue(CollisionDetected);
//            });
//
//        // 在后台线程等待主线程任务完成
//		bool bCollisionDetectedInSegment = Future.Get();
//        bCollisionDetected.store(bCollisionDetectedInSegment);
//
//        if (bCollisionDetected.load())
//        {
//            //UE_LOG(LogTemp, Warning, TEXT("Some kind of collision detected."));
//            return false;
//        }
//    }
//
//    return !bCollisionDetected.load();
//}


bool UMyRRTClass::IsTrajectoryCollisionFree(const TArray<FVector>& Trajectory,
    const TArray<FCylindricalInterestPoint>& Obstacles,
    float Threshold)
{
    // 使用 TWeakObjectPtr 安全地引用 World
    TWeakObjectPtr<UWorld> WeakWorld = World;
    if (!WeakWorld.IsValid())
    {
        UE_LOG(LogTemp, Warning, TEXT("World is nullptr"));
        return false;
    }

    // 1. 首先执行快速的几何碰撞检测（基于圆柱体的）
    for (int32 i = 1; i < Trajectory.Num(); ++i)
    {
        FVector StartPoint = Trajectory[i - 1];
        FVector EndPoint = Trajectory[i];

        // 检查端点是否在障碍物内
        for (const FCylindricalInterestPoint& Obstacle : Obstacles)
        {
            if (IsPointInCylinder(StartPoint, Obstacle) || IsPointInCylinder(EndPoint, Obstacle))
            {
                return false; // 轨迹点在障碍物内，碰撞失败
            }
        }

        // 检查线段与圆柱体的相交情况
        for (const FCylindricalInterestPoint& Obstacle : Obstacles)
        {
            float MinDist = ComputeLineObstacleDistance(StartPoint, EndPoint, Obstacle);
            if (MinDist <= (Obstacle.Radius + Threshold))
            {
                return false; // 线段与圆柱体相交，碰撞失败
            }
        }
    }

    // 2. 物理碰撞检测需要在游戏线程上执行，但我们需要避免递归调用FlushRenderingCommands
    // 使用一个标志来同步异步操作的结果
    bool bResult = true;
    FThreadSafeBool bResultReady = false;

    // 准备批次采样点
    TArray<FVector> AllSamplePoints;
    for (int32 i = 1; i < Trajectory.Num(); ++i)
    {
        FVector StartPoint = Trajectory[i - 1];
        FVector EndPoint = Trajectory[i];
        float SegmentLength = FVector::Dist(StartPoint, EndPoint);

        if (SegmentLength < KINDA_SMALL_NUMBER)
            continue;

        // 使用自适应采样密度
        int32 NumSteps = FMath::CeilToInt(SegmentLength / Threshold);
        NumSteps = FMath::Clamp(NumSteps, 1, 20); // 限制采样点数量

        for (int32 Step = 0; Step <= NumSteps; ++Step)
        {
            float t = static_cast<float>(Step) / static_cast<float>(NumSteps);
            FVector SamplePoint = FMath::Lerp(StartPoint, EndPoint, t);
            AllSamplePoints.Add(SamplePoint);
        }
    }

    // 在游戏线程上执行碰撞检测，但不等待结果
    AsyncTask(ENamedThreads::GameThread, [WeakWorld, AllSamplePoints, Threshold, &bResult, &bResultReady]()
        {
            if (!WeakWorld.IsValid())
            {
                bResult = false;
                bResultReady = true;
                return;
            }

            UWorld* LocalWorld = WeakWorld.Get();

            // 批量处理碰撞检测
            const int32 BatchSize = 20;
            const int32 NumBatches = FMath::CeilToInt(AllSamplePoints.Num() / static_cast<float>(BatchSize));

            bool bCollisionDetected = false;

            for (int32 BatchIndex = 0; BatchIndex < NumBatches && !bCollisionDetected; ++BatchIndex)
            {
                int32 StartIndex = BatchIndex * BatchSize;
                int32 EndIndex = FMath::Min((BatchIndex + 1) * BatchSize, AllSamplePoints.Num());

                for (int32 i = StartIndex; i < EndIndex && !bCollisionDetected; ++i)
                {
                    const FVector& SamplePoint = AllSamplePoints[i];

                    FHitResult HitResult;
                    FCollisionQueryParams QueryParams;

                    float SphereRadius = FMath::Min(200.0f, Threshold * 2.0f);

                    if (LocalWorld->SweepSingleByChannel(
                        HitResult,
                        SamplePoint,
                        SamplePoint, // 相同的起点和终点表示静态检查
                        FQuat::Identity,
                        ECC_WorldDynamic,
                        FCollisionShape::MakeSphere(SphereRadius),
                        QueryParams))
                    {
                        bCollisionDetected = true;
                        break;
                    }
                }
            }

            bResult = !bCollisionDetected;
            bResultReady = true;
        });

    // 3. 如果调用者不需要立即结果，可以设置一个回调而不是等待
    // 但对于这个函数，我们需要同步等待结果
    // 重要：使用一个轻量级的轮询而不是阻塞等待，避免递归FlushRenderingCommands

    // 轮询等待结果，最多等待5秒
    const float MaxWaitTime = 5.0f;
    float ElapsedTime = 0.0f;
    const float SleepInterval = 0.001f;

    while (!bResultReady && ElapsedTime < MaxWaitTime)
    {
        FPlatformProcess::Sleep(SleepInterval);
        ElapsedTime += SleepInterval;
    }

    if (!bResultReady)
    {
        // 超时，保守地返回碰撞
        UE_LOG(LogTemp, Warning, TEXT("Collision check timed out after %f seconds"), MaxWaitTime);
        return false;
    }

    return bResult;
}


FVector UMyRRTClass::HeuristicSample(
    const TArray<RRTNode*>& ForwardTree,
    const TArray<RRTNode*>& BackwardTree,
    const FBox& SearchSpace,
    float GoalBiasProb,
    const FVector& StartLocation,
    const FVector& EndLocation)
{
    // 计算搜索进度
    float SearchProgress = (float)CurrentIteration / (float)TotalIterations;

    // 动态调整目标偏向概率
    float AdjustedGoalBiasProb = GoalBiasProb;
    if (SearchProgress < 0.3f) {
        // 搜索初期降低目标偏向
        AdjustedGoalBiasProb *= 0.7f;
    }
    else if (SearchProgress > 0.7f) {
        // 搜索后期提高目标偏向
        AdjustedGoalBiasProb = FMath::Min(AdjustedGoalBiasProb * 1.5f, 0.5f);
    }
    else {
        // 搜索中期根据成功率调整
        if (SuccessRate < 0.3f) {
            AdjustedGoalBiasProb *= 0.8f; // 成功率低时降低目标偏向，增强探索
        }
    }

    // 使用动态目标偏向采样
    if (FMath::FRand() < AdjustedGoalBiasProb) {
        return EndLocation; // 目标偏向
    }

    // 计算启发式采样概率
    float HeuristicProb = 0.4f;
    if (SuccessRate < 0.3f) {
        HeuristicProb = 0.6f; // 成功率低时增加启发式采样比例
    }

    // 在两树之间区域采样 
    if (FMath::FRand() < HeuristicProb && ForwardTree.Num() > 0 && BackwardTree.Num() > 0) {
        // 随机选择两棵树中的节点
        int32 ForwardIdx = FMath::RandRange(0, ForwardTree.Num() - 1);
        int32 BackwardIdx = FMath::RandRange(0, BackwardTree.Num() - 1);

        FVector ForwardPoint = ForwardTree[ForwardIdx]->Point;
        FVector BackwardPoint = BackwardTree[BackwardIdx]->Point;

        // 在连线附近随机采样
        float Alpha = FMath::FRand(); // 随机插值参数
        FVector MidPoint = FMath::Lerp(ForwardPoint, BackwardPoint, Alpha);

        // 在中点附近添加随机扰动
        float MaxDeviation = FVector::Distance(ForwardPoint, BackwardPoint) * 0.3f;
        FVector RandomOffset(
            FMath::FRandRange(-MaxDeviation, MaxDeviation),
            FMath::FRandRange(-MaxDeviation, MaxDeviation),
            FMath::FRandRange(-MaxDeviation, MaxDeviation)
        );

        return MidPoint + RandomOffset;
    }

    // 默认随机采样
    return FMath::RandPointInBox(SearchSpace);
}


double UMyRRTClass::GetAdaptiveConnectThreshold(
    double BaseThreshold,
    int32 CurrentIter,
    int32 MaxIter,
    float CurrentSuccessRate)
{
    // 1. 根据迭代进度调整
    float ProgressFactor = 1.0f + 0.5f * (static_cast<float>(CurrentIter) / static_cast<float>(MaxIter));

    // 2. 根据成功率调整
    float SuccessFactor = 1.0f;
    if (CurrentSuccessRate < 0.2f) {
        // 成功率低时放宽阈值
        SuccessFactor = 1.2f;
    }
    else if (CurrentSuccessRate > 0.6f) {
        // 成功率高时收紧阈值
        SuccessFactor = 0.9f;
    }

    // 3. 根据上次连接尝试的接近程度调整
    float DistanceFactor = 1.0f;
    if (LastConnectDistance < BaseThreshold * 5.0f && LastConnectDistance > BaseThreshold) {
        // 如果接近但未成功，适当放宽阈值
        DistanceFactor = 1.1f;
    }

    return BaseThreshold * ProgressFactor * SuccessFactor * DistanceFactor;
}


RRTNode* UMyRRTClass::ExtendBiRRTTree(TArray<RRTNode*>& ActiveTree, const FVector& Target,
    double StepSize, const TArray<FCylindricalInterestPoint>& Obstacles, double NeighborRadius)
{
    // 找到离目标最近的节点
    int32 NearestIdx = NearestNodeIndex(ActiveTree, Target);
    RRTNode* NearestNode = ActiveTree[NearestIdx];

    // 计算扩展方向并生成新点
    FVector Direction = (Target - NearestNode->Point).GetSafeNormal();
    FVector NewPoint = NearestNode->Point + Direction * StepSize;

    // 检查新点是否在障碍物中
    if (IsInObstacleLocal(NewPoint, Obstacles))
    {
        return nullptr;
    }

    // 检查 NearestNode 和 NewPoint 之间的路径段是否与障碍物相交
    if (LineIntersectsObstacles(NearestNode->Point, NewPoint, Obstacles))
    {
        return nullptr; // 如果路径段与障碍物相交，拒绝这个新节点
    }

    // 初始化新节点的成本
    double NewCost = NearestNode->Cost + FVector::Distance(NearestNode->Point, NewPoint);
    RRTNode* NewNode = new RRTNode(NewPoint, NearestNode, NewCost);

    // 查找邻域内的节点
    TArray<int32> Neighbors = RadiusSearch(ActiveTree, NewPoint, NeighborRadius);

    // 选择最优父节点
    for (int32 ni : Neighbors)
    {
        RRTNode* Neighbor = ActiveTree[ni];
        double PotentialCost = Neighbor->Cost + FVector::Distance(Neighbor->Point, NewPoint);

        if (PotentialCost < NewNode->Cost && !LineIntersectsObstacles(Neighbor->Point, NewPoint, Obstacles))
        {
            NewNode->Parent = Neighbor;
            NewNode->Cost = PotentialCost;
        }
    }

    // 添加新节点到树
    ActiveTree.Add(NewNode);

    // 更新成功计数
    SuccessfulExtensions++;

    // 重接邻域内的节点，以优化路径成本
    for (int32 ni : Neighbors)
    {
        RRTNode* Neighbor = ActiveTree[ni];
        double PotentialCost = NewNode->Cost + FVector::Distance(NewNode->Point, Neighbor->Point);

        if (PotentialCost < Neighbor->Cost && !LineIntersectsObstacles(NewNode->Point, Neighbor->Point, Obstacles))
        {
            Neighbor->Parent = NewNode;
            Neighbor->Cost = PotentialCost;
        }
    }

    return NewNode;
}


TArray<FVector> UMyRRTClass::ExtractPath(RRTNode* ForwardNode, RRTNode* BackwardNode, bool bReverse)
{
    TArray<FVector> CompletePath;

    // 1. 从ForwardNode回溯到起点，收集前向路径
    TArray<RRTNode*> ForwardNodes;
    RRTNode* Current = ForwardNode;
    while (Current)
    {
        ForwardNodes.Insert(Current, 0); // 插入到开头，保持正确顺序
        Current = Current->Parent;
    }

    // 2. 从BackwardNode回溯到终点，收集后向路径
    TArray<RRTNode*> BackwardNodes;
    Current = BackwardNode;
    while (Current)
    {
        BackwardNodes.Add(Current); // 添加到末尾
        Current = Current->Parent;
    }

    // 3. 构建完整路径
    // 添加前向路径点
    for (RRTNode* Node : ForwardNodes)
    {
        FPathPointWithOrientation WP;
        WP.Point = Node->Point;
        WP.Orientation = FRotator::ZeroRotator;
        WP.FOV = 90.0f;
        WP.AestheticScore = 0.0f;
        WP.CoverageAngle = 0.0f;
        WP.AOIIndex = -1;
        WP.AngleRange = FAngleRange();
        CompletePath.Add(WP.Point);
		//DrawDebugSphere(World, WP.Point, 50.0f, 12, FColor::Emerald, true, 5.0f);
    }

    // 只有当前向和后向节点不重合时才添加连接点
    float ConnectionDistance = FVector::Distance(ForwardNode->Point, BackwardNode->Point);
    if (ConnectionDistance > 1.0f) // 使用一个小的阈值来判断是否重合
    {
        // 添加连接点
        FVector ConnectionPoint = (ForwardNode->Point + BackwardNode->Point) * 0.5f;
        FPathPointWithOrientation ConnectionWP;
        ConnectionWP.Point = ConnectionPoint;
        ConnectionWP.Orientation = FRotator::ZeroRotator;
        ConnectionWP.FOV = 90.0f;
        ConnectionWP.AestheticScore = 0.0f;
        ConnectionWP.CoverageAngle = 0.0f;
        ConnectionWP.AOIIndex = -1;
        ConnectionWP.AngleRange = FAngleRange();
        CompletePath.Add(ConnectionWP.Point);
    }

    // 添加后向路径点（如果需要反转）
    if (bReverse)
    {
        // 跳过第一个节点，因为它与连接点太近
        for (int32 i = 1; i < BackwardNodes.Num(); ++i)
        {
            FPathPointWithOrientation WP;
            WP.Point = BackwardNodes[i]->Point;
            WP.Orientation = FRotator::ZeroRotator;
            WP.FOV = 90.0f;
            WP.AestheticScore = 0.0f;
            WP.CoverageAngle = 0.0f;
            WP.AOIIndex = -1;
            WP.AngleRange = FAngleRange();
            CompletePath.Add(WP.Point);
        }
    }
    else
    {
        // 如果不需要反转，则反向添加后向路径点
        for (int32 i = BackwardNodes.Num() - 2; i >= 0; --i)
        {
            FPathPointWithOrientation WP;
            WP.Point = BackwardNodes[i]->Point;
            WP.Orientation = FRotator::ZeroRotator;
            WP.FOV = 90.0f;
            WP.AestheticScore = 0.0f;
            WP.CoverageAngle = 0.0f;
            WP.AOIIndex = -1;
            WP.AngleRange = FAngleRange();
            CompletePath.Add(WP.Point);
        }
    }

    // 验证路径的完整性
    if (CompletePath.Num() < 2)
    {
        UE_LOG(LogTemp, Warning, TEXT("ExtractPath generated invalid path with less than 2 points"));
        return TArray<FVector>();
    }

    // 验证起点和终点
    float StartDistance = FVector::Distance(CompletePath[0], ForwardNodes[0]->Point);
    float EndDistance = FVector::Distance(CompletePath.Last(), BackwardNodes.Last()->Point);

    if (StartDistance > 1.0f || EndDistance > 1.0f)
    {
        UE_LOG(LogTemp, Warning, TEXT("ExtractPath: Path endpoints don't match original points. Start diff: %f, End diff: %f"),
            StartDistance, EndDistance);
    }

    // 移除可能的重复点
    for (int32 i = CompletePath.Num() - 2; i >= 0; --i)
    {
        if (FVector::Distance(CompletePath[i], CompletePath[i + 1]) < 1.0f)
        {
            CompletePath.RemoveAt(i);
        }
    }

    return CompletePath;
}


bool UMyRRTClass::IsInObstacleLocal(const FVector& Point,
    const TArray<FCylindricalInterestPoint>& Obstacles,
    float Threshold) const
{
    // 检查几何障碍物（圆柱体）
    for (const FCylindricalInterestPoint& Obstacle : Obstacles)
    {
        if (IsPointInCylinder(Point, Obstacle))
        {
            return true; // 点在圆柱体障碍物内
        }
    }

    // 使用 TWeakObjectPtr 安全地引用 World
    TWeakObjectPtr<UWorld> WeakWorld = World;

    if (!WeakWorld.IsValid())
    {
        UE_LOG(LogTemp, Warning, TEXT("World is invalid"));
        return false;
    }

    UWorld* LocalWorld = WeakWorld.Get();

    // 定义 SweepSingleByChannel 的碰撞形状
    FHitResult HitResult;
    FVector TraceStart = Point - FVector(1.0f, 1.0f, 1.0f) * Threshold;
    FVector TraceEnd = Point + FVector(1.0f, 1.0f, 1.0f) * Threshold;

    FCollisionQueryParams QueryParams;
    QueryParams.bTraceComplex = true; // 检测复杂碰撞对象

    float SphereRadius = 200.0f; // 动态设置球体半径

    // 执行碰撞检测
    if (LocalWorld->SweepSingleByChannel(
        HitResult,
        TraceStart,
        TraceEnd,
        FQuat::Identity,       // 没有旋转
        ECC_WorldDynamic,        // 默认可见性通道
        FCollisionShape::MakeSphere(SphereRadius), // 定义球体形状
        QueryParams))
    {
        //UE_LOG(LogTemp, Warning, TEXT("Point is within an obstacle detected by SweepSingleByChannel"));
        return true; // 检测到碰撞
    }

    return false; // 未检测到碰撞
}


bool UMyRRTClass::LineIntersectsObstacles(const FVector& Start, const FVector& End,
    const TArray<FCylindricalInterestPoint>& Obstacles,
    float Threshold) const
{
    // 检查端点是否在障碍物内
    if (IsInObstacleLocal(Start, Obstacles) || IsInObstacleLocal(End, Obstacles))
    {
        return true;
    }

    // 检查线段是否过短
    if (FVector::Distance(Start, End) < 1.0f)
    {
        return false; // 线段过短，不可能发生碰撞
    }

    // 使用 TWeakObjectPtr 安全地引用 World
    TWeakObjectPtr<UWorld> WeakWorld = World;

    if (!WeakWorld.IsValid())
    {
        UE_LOG(LogTemp, Warning, TEXT("World is invalid"));
        return false;
    }

    UWorld* LocalWorld = WeakWorld.Get();

    // 检查与圆柱体障碍物的碰撞
    for (const FCylindricalInterestPoint& Obstacle : Obstacles)
    {
        float MinDist = ComputeLineObstacleDistance(Start, End, Obstacle);
        if (MinDist <= (Obstacle.Radius + Threshold))
        {
            return true;
        }
    }

    float SphereRadius = 200.0f; // 动态调整球体半径
    FHitResult HitResult;
    FCollisionQueryParams LocalQueryParams;

    if (LocalWorld->SweepSingleByChannel(
        HitResult,
        Start,
        End,
        FQuat::Identity,
        ECC_WorldDynamic,
        FCollisionShape::MakeSphere(SphereRadius),
        LocalQueryParams))
    {
        //UE_LOG(LogTemp, Warning, TEXT("Collision detected at: %s"), *HitResult.ImpactPoint.ToString());
        return true;
    }

    return false;
}


float UMyRRTClass::ComputeLineObstacleDistance(const FVector& Start, const FVector& End,
    const FCylindricalInterestPoint& Obstacle) const
{
    // 获取线段向量
    FVector LineDir = (End - Start).GetSafeNormal();

    // 圆柱体底部中心到起点的向量
    FVector ToStart = Start - Obstacle.BottomCenter;

    // 计算垂直投影
    float DotProduct = FVector::DotProduct(ToStart, LineDir);
    FVector Projection = Start + LineDir * FMath::Clamp(-DotProduct, 0.0f, FVector::Distance(Start, End));

    // 计算投影点到圆柱体中心的水平距离
    FVector ToCylinder = Projection - Obstacle.BottomCenter;
    float HorizontalDist = FMath::Sqrt(ToCylinder.X * ToCylinder.X + ToCylinder.Y * ToCylinder.Y);

    // 考虑高度约束
    float VerticalDist = ToCylinder.Z;
    if (VerticalDist < 0.0f || VerticalDist > Obstacle.Height)
    {
        // 如果在圆柱体高度范围外，返回一个很大的距离
        return TNumericLimits<float>::Max();
    }

    return HorizontalDist;
}


TArray<FPathPointWithOrientation> UMyRRTClass::GenerateAndSmoothRRTPath_BackUp(
    const FVector& StartLocation,
    const FVector& EndLocation,
    const TArray<FCylindricalInterestPoint>& InterestPoints,
    int32 MaxRetries /*=5*/,
    float BaseStepSize /*=0.5f*/,
    float CurvatureFactor /*=1.0f*/,
    float MaxSegmentLength /*=100.0f*/,
    int32 MinPoints /*=10*/
)
{
    // 使用RRT生成初始路径
    TArray<FPathPointWithOrientation> RawPath = GenerateRRTPath(StartLocation, EndLocation, InterestPoints);

    if (RawPath.Num() <= 1)
    {
        return RawPath;
    }

    // 转换为FVector数组计算长度
    TArray<FVector> RawVectorPath;
    RawVectorPath.Reserve(RawPath.Num());
    for (const FPathPointWithOrientation& Node : RawPath)
    {
        RawVectorPath.Add(Node.Point);
    }

    // 计算路径总长度
    float TotalLength = 0.0f;
    for (int32 i = 1; i < RawVectorPath.Num(); ++i)
    {
        TotalLength += FVector::Dist(RawVectorPath[i], RawVectorPath[i - 1]);
    }

    // 根据总长度和MaxSegmentLength计算需要的点数
    int32 NumPoints = FMath::CeilToInt(TotalLength / MaxSegmentLength) + 1;
    // 确保不低于MinPoints
    NumPoints = FMath::Max(NumPoints, MinPoints);

    // 对路径进行样条插值和平滑
    TArray<FVector> SmoothedVectorPath = SmoothPathWithBSplineDynamic(RawVectorPath, BaseStepSize,  NumPoints, CurvatureFactor);

    int32 RetryCount = 0;
    while (!IsTrajectoryCollisionFree(SmoothedVectorPath, InterestPoints) && RetryCount < MaxRetries)
    {
        CurvatureFactor *= 0.9f;
        // 曲率变化可能需要重新计算NumPoints吗？
        // 一般不需改变点数策略，可保持NumPoints不变或者再算一次。
        // 保持简化，这里不再改变NumPoints。
        SmoothedVectorPath = SmoothPathWithBSplineDynamic(RawVectorPath, BaseStepSize, NumPoints, CurvatureFactor);
        ++RetryCount;
    }

    // 若仍有碰撞，则退回原始路径
    if (!IsTrajectoryCollisionFree(SmoothedVectorPath, InterestPoints))
    {
        UE_LOG(LogTemp, Warning, TEXT("Failed to smooth path after %d retries!"), RetryCount);
        return RawPath;
    }

    // 转换回FPathPointWithOrientation数组
    TArray<FPathPointWithOrientation> SmoothedPathWithOrientation;
    SmoothedPathWithOrientation.Reserve(SmoothedVectorPath.Num());

    for (int32 i = 0; i < SmoothedVectorPath.Num(); ++i)
    {
        FPathPointWithOrientation NewNode;
        NewNode.Point = SmoothedVectorPath[i];
        NewNode.Orientation = FRotator::ZeroRotator;
        NewNode.FOV = 90.0f;
        NewNode.AestheticScore = 0.0f;
        NewNode.CoverageAngle = 0.0f;
        NewNode.AOIIndex = -1;
        NewNode.AngleRange = FAngleRange();
        SmoothedPathWithOrientation.Add(NewNode);
    }

    return SmoothedPathWithOrientation;
}


TArray<FVector> UMyRRTClass::FilterPathByMinDistance(const TArray<FVector>& Path, 
    float MinDisBetweenPoints)
{
    TArray<FVector> FilteredPath;
    if (Path.Num() == 0) return FilteredPath;

    FilteredPath.Add(Path[0]); // 保留第一个点
    FVector LastPoint = Path[0];

    for (int32 i = 1; i < Path.Num(); ++i)
    {
        float Distance = FVector::Dist(LastPoint, Path[i]);
        if (Distance >= MinDisBetweenPoints)
        {
            FilteredPath.Add(Path[i]);
            LastPoint = Path[i]; // 更新上一个点
        }
    }

    // 确保保留最后一个点
    if (FilteredPath.Last() != Path.Last())
    {
        FilteredPath.Add(Path.Last());
    }

    return FilteredPath;
}


TArray<FVector> UMyRRTClass::MergeSegment(const TArray<FVector>& Segment, float MinDisBetweenPoints)
{
    TArray<FVector> MergedSegment;
    if (Segment.Num() == 0) return MergedSegment;

    MergedSegment.Add(Segment[0]); // 保留段的起点
    FVector LastPoint = Segment[0];

    for (int32 i = 1; i < Segment.Num(); ++i)
    {
        FVector CurrentPoint = Segment[i];
        float Distance = FVector::Dist(LastPoint, CurrentPoint);

        // 仅在距离超过阈值时保留点
        if (Distance >= MinDisBetweenPoints)
        {
            MergedSegment.Add(CurrentPoint);
            LastPoint = CurrentPoint;
        }
        else
        {
            // 如果距离小于阈值，检查路径的曲率或角度变化
            if (i < Segment.Num() - 1)
            {
                FVector NextPoint = Segment[i + 1];
                FVector Dir1 = (CurrentPoint - LastPoint).GetSafeNormal();
                FVector Dir2 = (NextPoint - CurrentPoint).GetSafeNormal();
                float DotProduct = FVector::DotProduct(Dir1, Dir2);

                // 如果角度变化较大，保留该点
                if (DotProduct < 0.9) // 0.9 是一个阈值，可以根据实际情况调整
                {
                    MergedSegment.Add(CurrentPoint);
                    LastPoint = CurrentPoint;
                }
            }
        }
    }

    // 确保段的终点保留
    if (MergedSegment.Last() != Segment.Last())
    {
        MergedSegment.Add(Segment.Last());
    }

    return MergedSegment;
}


TArray<FVector> UMyRRTClass::FilterPathByMinDistanceWithCollisionCheck(
    const TArray<FVector>& Path,
    float MinDisBetweenPoints,
    const TArray<FCylindricalInterestPoint>& InterestPoints)
{
    TArray<FVector> FilteredPath;
    if (Path.Num() == 0) return FilteredPath;

    // 初始化
    FilteredPath.Add(Path[0]); // 保留第一个点
    FVector LastPoint = Path[0];

    // 当前段初始化为空
    TArray<FVector> CurrentSegment;

    for (int32 i = 1; i < Path.Num(); ++i)
    {
        FVector CurrentPoint = Path[i];
        float Distance = FVector::Dist(LastPoint, CurrentPoint);

        // 如果点间距超过 MinDisBetweenPoints，尝试处理当前段
        if (Distance >= MinDisBetweenPoints)
        {
            // 如果当前段只有一个点，直接加入最终路径
            if (CurrentSegment.Num() == 1)
            {
                FilteredPath.Add(CurrentSegment[0]);
            }
            else if (CurrentSegment.Num() > 1)
            {
                // 尝试合并当前段
                TArray<FVector> MergedSegment = MergeSegment(CurrentSegment, MinDisBetweenPoints);

                // 检测合并后的段是否无碰撞
                if (!IsTrajectoryCollisionFree(MergedSegment, InterestPoints))
                {
                    // 若发生碰撞，保留原始段
                    FilteredPath.Append(CurrentSegment);
                }
                else
                {
                    // 若无碰撞，加入合并后的段
                    FilteredPath.Append(MergedSegment);
                }
            }

            // 开始新的段
            CurrentSegment.Empty();
            CurrentSegment.Add(CurrentPoint); // 当前点为新段起点
        }
        else
        {
            // 当前点仍在当前段内
            CurrentSegment.Add(CurrentPoint);
        }

        LastPoint = CurrentPoint; // 更新上一个点
    }

    // 处理最后一个段
    if (CurrentSegment.Num() == 1)
    {
        FilteredPath.Add(CurrentSegment[0]);
    }
    else if (CurrentSegment.Num() > 1)
    {
        TArray<FVector> MergedSegment = MergeSegment(CurrentSegment, MinDisBetweenPoints);
        if (!IsTrajectoryCollisionFree(MergedSegment, InterestPoints))
        {
            FilteredPath.Append(CurrentSegment);
        }
        else
        {
            FilteredPath.Append(MergedSegment);
        }
    }

    // 确保终点保留
    if (FilteredPath.Last() != Path.Last())
    {
        FilteredPath.Add(Path.Last());
    }

    return FilteredPath;
}


TArray<FPathPointWithOrientation> UMyRRTClass::GenerateAndSmoothRRTPath(
    const FVector& StartLocation,
    const FVector& EndLocation,
    const TArray<FCylindricalInterestPoint>& InterestPoints,
    TArray<FVector>& TestPoints,
    const FVector& ExtraStartPoint,
    const FVector& ExtraEndPoint,
    float MinDisBetweenPoints,
    int32 MaxRetries,
    float CurvatureFactor,
    float MaxSegmentLength,
    int32 MinPoints,
    double StepSize,
    double NeighborRadius)
{
    // 重置跟踪变量
    CurrentIteration = 0;
    SuccessfulExtensions = 0;
    SuccessRate = 1.0f; // 初始假设成功率高

    FBiRRTTree BiRRT;
    FVector LocalStartLocation;
    FVector LocalEndLocation;
    if (ExtraStartPoint != FVector::ZeroVector && ExtraEndPoint != FVector::ZeroVector) {
		BiRRT.Initialize(ExtraStartPoint, ExtraEndPoint);
		LocalStartLocation = ExtraStartPoint;
        LocalEndLocation = ExtraEndPoint;
    }
    else {
        BiRRT.Initialize(StartLocation, EndLocation);
		LocalStartLocation = StartLocation;
		LocalEndLocation = EndLocation;
    }
    
    // 设置搜索空间
    FBox BoundingBox(StartLocation, EndLocation);
    // 获取长方体的半尺寸
    FVector BoxExtent = BoundingBox.GetExtent();

    // 定义扩展比例和固定高度扩展值
    float HorizontalExpansionRatio = 0.2f; // 水平方向扩展比例（X 和 Y）
    float FixedVerticalExpansion = 2000.0f; // 高度方向固定扩展值

    // 计算水平方向的扩展值
    FVector Expansion(
        BoxExtent.X * HorizontalExpansionRatio,
        BoxExtent.Y * HorizontalExpansionRatio,
        FixedVerticalExpansion // 高度方向直接使用固定值
    );
    FVector BoxMin = BoundingBox.Min - FVector(Expansion);
    FVector BoxMax = BoundingBox.Max + FVector(Expansion);
    FBox SearchSpace(BoxMin, BoxMax);

    // 设置RRT参数
    TotalIterations = FMath::Max(10 * FVector::Distance(StartLocation, EndLocation), 50000);
    float BaseGoalBias = 0.2f;
    double BaseConnectThreshold = StepSize * 0.1;
    LastConnectDistance = TNumericLimits<float>::Max();

    RRTNode* ConnectingForwardNode = nullptr;
    RRTNode* ConnectingBackwardNode = nullptr;
    float BestDistance = TNumericLimits<float>::Max();

    for (int32 Iter = 0; Iter < TotalIterations; ++Iter)
    {
        CurrentIteration = Iter;

        // 更新成功率
        if (Iter > 0) {
            SuccessRate = static_cast<float>(SuccessfulExtensions) / static_cast<float>(Iter);
        }

        // 计算树平衡概率，倾向于扩展小树
        float ForwardProb = static_cast<float>(BiRRT.BackwardTree.Num()) /
            (BiRRT.ForwardTree.Num() + BiRRT.BackwardTree.Num());

        bool bGrowForward = FMath::FRand() < ForwardProb;
        TArray<RRTNode*>& ActiveTree = bGrowForward ? BiRRT.ForwardTree : BiRRT.BackwardTree;
        TArray<RRTNode*>& OtherTree = bGrowForward ? BiRRT.BackwardTree : BiRRT.ForwardTree;

        // 使用启发式采样
        FVector Target = HeuristicSample(
            BiRRT.ForwardTree,
            BiRRT.BackwardTree,
            SearchSpace,
            BaseGoalBias,
            LocalStartLocation,
            LocalEndLocation
        );

        // 扩展树
        RRTNode* NewNode = ExtendBiRRTTree(ActiveTree, Target, StepSize, InterestPoints, NeighborRadius);
        if (!NewNode) continue;

        // 自适应连接阈值
        double CurrentConnectThreshold = GetAdaptiveConnectThreshold(
            BaseConnectThreshold,
            Iter,
            TotalIterations,
            SuccessRate
        );

        // 尝试连接
        int32 NearestOtherIdx = NearestNodeIndex(OtherTree, NewNode->Point);
        RRTNode* NearestOther = OtherTree[NearestOtherIdx];
        float Distance = FVector::Distance(NewNode->Point, NearestOther->Point);
        LastConnectDistance = Distance; // 记录连接尝试距离

        if (Distance < CurrentConnectThreshold &&
            !LineIntersectsObstacles(NewNode->Point, NearestOther->Point, InterestPoints))
        {
            if (Distance < BestDistance)
            {
                BestDistance = Distance;
                ConnectingForwardNode = bGrowForward ? NewNode : NearestOther;
                ConnectingBackwardNode = bGrowForward ? NearestOther : NewNode;

                // 如果距离很小，直接结束搜索
                if (Distance < StepSize * 0.5f)
                {
                    break;
                }
            }
        }

        // 周期性检查是否可以结束搜索
        if (Iter % 100 == 0 && ConnectingForwardNode != nullptr)
        {
            if (BestDistance < StepSize)
            {
                break;
            }
        }
    }

    // 检查是否找到有效路径
    if (!ConnectingForwardNode || !ConnectingBackwardNode ||
        BestDistance > BaseConnectThreshold * 2.0f)
    {
        UE_LOG(LogTemp, Warning, TEXT("BiRRT failed to find path (Distance: %f), using backup method"),
            BestDistance);
        return GenerateAndSmoothRRTPath_BackUp(StartLocation, EndLocation, InterestPoints);
    }

    // 提取路径
    TArray<FVector> Path = ExtractPath(ConnectingForwardNode, ConnectingBackwardNode, true);

    if (!IsTrajectoryCollisionFree(Path, InterestPoints))
    {
		UE_LOG(LogTemp, Error, TEXT("BiRRT path contains collisions!"));
    }

    // 路径平滑
	// 也利用额外点进行平滑
    TArray<FVector> RawVectorPath;
	if (LocalStartLocation != StartLocation && LocalEndLocation != EndLocation)
	{
		RawVectorPath.Reserve(Path.Num() + 2);
		RawVectorPath.Add(StartLocation);
		for (const auto& Node : Path)
		{
			RawVectorPath.Add(Node);
		}
		RawVectorPath.Add(EndLocation);
	}
	else
	{
		RawVectorPath.Reserve(Path.Num() );
		for (const auto& Node : Path)
		{
			RawVectorPath.Add(Node);
		}
	}

    
    int32 NumPoints = FMath::Max(
        FMath::CeilToInt(ComputePathLength(RawVectorPath) / MaxSegmentLength * (1 + CurvatureFactor)),
        MinPoints
    );
    int32 Degree = (CurvatureFactor > 1.5f) ? 4 : 3; // 曲率较大时使用4阶

    TArray<FVector> SmoothedVectorPath;
    int32 RetryCount = 0;
    while (RetryCount < MaxRetries)
    {
        SmoothedVectorPath = SmoothPathWithExtraConstraints(RawVectorPath, Degree, NumPoints, CurvatureFactor,
            StartLocation,ExtraStartPoint,
            EndLocation,ExtraEndPoint);
        if (IsTrajectoryCollisionFree(SmoothedVectorPath, InterestPoints))
        {
            break;
        }
        CurvatureFactor *= 0.9f;
        ++RetryCount;
    }

    // 对 SmoothedVectorPath 进行基于距离和碰撞检测的逐段优化
    TArray<FVector> OptimizedPath = FilterPathByMinDistanceWithCollisionCheck(SmoothedVectorPath, MinDisBetweenPoints, InterestPoints);

    // 检查筛选后的路径是否无碰撞
    if (!IsTrajectoryCollisionFree(OptimizedPath, InterestPoints))
    {
        UE_LOG(LogTemp, Warning, TEXT("Filtered path contains collisions!"));
        // 输出当前起点和终点
		//UE_LOG(LogTemp, Warning, TEXT("Start: %s, End: %s"), *StartLocation.ToString(), *EndLocation.ToString());
        if (!IsTrajectoryCollisionFree(SmoothedVectorPath, InterestPoints)) {
			UE_LOG(LogTemp, Warning, TEXT("Even Smoothed path contains collisions?"));
        }

		TestPoints = OptimizedPath;
        OptimizedPath = SmoothedVectorPath;
    }

    // 转换回FPathPointWithOrientation
    TArray<FPathPointWithOrientation> SmoothedPath;
    SmoothedPath.Reserve(SmoothedVectorPath.Num());

    for (const FVector& Point : OptimizedPath)
    {
        FPathPointWithOrientation NewNode;
        NewNode.Point = Point;
        NewNode.Orientation = FRotator::ZeroRotator;
        NewNode.FOV = 90.0f;
        NewNode.AestheticScore = 0.0f;
        NewNode.CoverageAngle = 0.0f;
        NewNode.AOIIndex = -1;
        NewNode.AngleRange = FAngleRange();
        SmoothedPath.Add(NewNode);
    }

    return SmoothedPath;
}

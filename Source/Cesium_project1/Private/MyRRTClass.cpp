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
    // �������ķ���FOV����������...
    AddNode(StartNode);

    // ���������յ�İ�Χ��
    FBox BoundingBox(StartLocation, EndLocation);

    // ���������ռ�ı߽�,�ʵ������Χ��
    float Expansion = FVector::Distance(BoundingBox.Min, BoundingBox.Max) / 5.0f;
    FVector BoxMin = BoundingBox.Min - FVector(Expansion);
    FVector BoxMax = BoundingBox.Max + FVector(Expansion);
    FBox SearchSpace(BoxMin, BoxMax);

    // �Զ����ò���
    float StepSize =  100.0f;
    int32 MaxIterations = FMath::Max(10 * FVector::Distance(StartLocation, EndLocation), 5000);
    float GoalBiasProb = FMath::Clamp(1.0f / (1.0f + InterestPoints.Num()), 0.05f, 0.2f);

    for (int32 i = 0; i < MaxIterations; ++i)
    {
        FPathPointWithOrientation RandomNode;

        // ��һ������ֱ��ѡ��Ŀ�����Ϊ����ڵ�
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
        // �����½ڵ�ķ���FOV����������...

        // ����½ڵ��Ƿ�����Ȥ�㷢����ײ
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
            // �����յ�ķ���FOV����������...
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

    // �ҵ�����T�����Ľڵ�����
    int32 span = -1;
    for (int32 i = k; i < n; ++i)
    {
        if (T >= KnotVector[i] && T <= KnotVector[i + 1])
        {
            span = i;
            break;
        }
    }

    // ������Χ���������
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

    // De Boor�㷨
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

    // 1. �����ҳ����в�����
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

    // 2. �����������Ƶ� (Raw Control Points)
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

    // 3. ʹ��Tension�Կ��Ƶ���ж���ƽ������
    //    ��Tension > 1ʱ��˵�����Ը���ƽ�������Խ����Ƶ���΢����ǰ����Ƶ���е�
    //    ��Tension < 1ʱ������ʵԭ·����������������ƽ������
    //    ��Tension = 1ʱ��Ϊ��׼B�����޶���ƫ�ơ�
    TArray<FVector> ControlPoints;
    ControlPoints.SetNum(NumControlPoints);

    for (int32 i = 0; i < NumControlPoints; ++i)
    {
        FVector CP = RawControlPoints[i];

        // �ڵ�ƽ��
        FVector PrevPos = (i > 0) ? RawControlPoints[i - 1] : RawControlPoints[i];
        FVector NextPos = (i < NumControlPoints - 1) ? RawControlPoints[i + 1] : RawControlPoints[i];
        FVector Mid = (PrevPos + NextPos) * 0.5f;

        // ����Tensionӳ�����
        // ��Tension > 1ʱ����CP��Mid��ֵ
        // ��Tension < 1ʱ����CP����£����������ƽ��
        float ExtraSmoothFactor = (Tension > 1.0f) ? (Tension - 1.0f) : 0.0f;
        // ������Ŀ����ɽ�һ������ExtraSmoothFactor
        CP = FMath::Lerp(CP, Mid, ExtraSmoothFactor);

        ControlPoints[i] = CP;
    }

    // 4. ����Knot Vector ��B������׼�ڵ�������
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

    // 5. ����ƽ��·����
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

    // 1. �����ҳ����в�����
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

    // 2. �������Ƶ�
    TArray<FVector> ControlPoints;
    ControlPoints.Reserve(NumControlPoints);

    // ��ǿ���Ȩ��
    ControlPoints.Add(StartLocation);
    if (!ExtraStartLocation.IsZero()) {
        FVector DirectionStart = (StartLocation - ExtraStartLocation).GetSafeNormal();
        FVector ConstrainedStart = StartLocation + DirectionStart * (TotalLength * 0.05f);
        ControlPoints.Add(ConstrainedStart);
    }
    else {
        ControlPoints.Add(StartLocation);
    }

    // �м�㣨�� Path ��ѡȡ��
    float Increment = 1.0f / (NumControlPoints - 1);
    for (int32 i = 0; i < NumControlPoints; ++i) {
        float tVal = i * Increment;
        int32 Index = Algo::UpperBound(Distances, tVal) - 1;
        Index = FMath::Clamp(Index, 0, Distances.Num() - 2);
        float LocalAlpha = (tVal - Distances[Index]) / (Distances[Index + 1] - Distances[Index]);
        FVector RawPos = FMath::Lerp(Path[Index], Path[Index + 1], LocalAlpha);
        ControlPoints.Add(RawPos);
    }

    // ��ǿ�յ�Ȩ��
    if (!ExtraEndLocation.IsZero()) {
        FVector DirectionEnd = (ExtraEndLocation - EndLocation).GetSafeNormal();
        FVector ConstrainedEnd = EndLocation + DirectionEnd * (TotalLength * 0.05f);
        ControlPoints.Add(ConstrainedEnd);
    }
    else {
        ControlPoints.Add(EndLocation);
    }
    ControlPoints.Add(EndLocation);

    // 3. ���� Knot Vector��B������׼�ڵ�������
    TArray<float> KnotVector;
    KnotVector.Reserve(ControlPoints.Num() + Degree + 1);

    // �����յ��Ȩ�����ӣ��ڵ��ظ���
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

    // 4. ����ƽ��·����
    float LocalTotalLength = 0.0f;
    for (int32 i = 1; i < Path.Num(); ++i) {
        LocalTotalLength += FVector::Dist(Path[i - 1], Path[i]);
    }
    // �����ܳ��ȷ���NumSplinePoints
    const int32 NumPoints = FMath::Max(10, FMath::RoundToInt(LocalTotalLength / 200.0f)); // ���磬ÿ100��λ���ȷ���һ����
    //int32 NumPoints = FMath::Max(10, Path.Num() * 2);

    // ����B�������ߵ�
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

// �����Ƿ���Բ������
bool UMyRRTClass::IsPointInCylinder(const FVector& Point, const FCylindricalInterestPoint& Cylinder) const
{
    // ����������Բ����ײ����ĵ�����
    FVector CylinderToPoint = Point - Cylinder.BottomCenter;

    // ������� XY ƽ���ŷ����þ��루���� Z �ᣩ
    float DistanceXY = FVector::DistXY(Point, Cylinder.BottomCenter);

    // ������� Z ���ϵĸ߶Ȳ�
    float DistanceZ = CylinderToPoint.Z;

    // �����Ƿ���Բ����İ뾶���ϰ�ȫ������
    bool bWithinRadius = DistanceXY <= (Cylinder.Radius + Cylinder.MinSafetyDistance);

    // �����Ƿ���Բ����ĸ߶ȷ�Χ��
    bool bWithinHeight = (DistanceZ >= 0.0f) && (DistanceZ <= Cylinder.Height);

    // ����Բ������������߶ȷ�Χ
    return bWithinRadius && bWithinHeight;
}


//bool UMyRRTClass::IsTrajectoryCollisionFree(const TArray<FVector>& Trajectory,
//    const TArray<FCylindricalInterestPoint>& Obstacles,
//    float Threshold)
//{
//    // ʹ�� TWeakObjectPtr ��ȫ������ World
//    TWeakObjectPtr<UWorld> WeakWorld = World;
//
//    if (!WeakWorld.IsValid())
//    {
//        UE_LOG(LogTemp, Warning, TEXT("World is nullptr"));
//        return false;
//    }
//
//    std::atomic<bool> bCollisionDetected(false);
//    FCriticalSection WorldMutex; // ���ڱ����� World �ķ���
//
//    // �����켣��
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
//        // �洢������
//        TArray<FVector> SamplePoints;
//        SamplePoints.Reserve(NumSteps + 1);
//
//        // ʹ�� ParallelFor ���ɲ����㲢���Բ������ײ
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
//                // �����Բ�������ײ
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
//                // ������������б�
//                SamplePoints.Add(SamplePoint);
//            });
//
//        // ����ڲ��й����м�⵽Բ������ײ������ǰ�˳�
//        if (bCollisionDetected.load())
//        {
//            UE_LOG(LogTemp, Warning, TEXT("Collision detected during parallel processing."));
//            return false;
//        }
//
//        // ��������������ײ��⵽���߳�
//        TSharedRef<TPromise<bool>> Promise = MakeShared<TPromise<bool>>();
//        TFuture<bool> Future = Promise->GetFuture();
//
//        AsyncTask(ENamedThreads::GameThread, [WeakWorld, Promise, SamplePoints, Threshold, &WorldMutex]() mutable
//            {
//                bool CollisionDetected = false;
//
//                // ʹ�� FScopeLock ������ World �ķ���
//                FScopeLock Lock(&WorldMutex);
//
//                if (!WeakWorld.IsValid())
//                {
//                    UE_LOG(LogTemp, Error, TEXT("World is no longer valid in AsyncTask"));
//                    Promise->SetValue(false); // ��� World ��Ч��ֱ�ӷ��� false
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
//                    float SphereRadius = 200.0f; // ��������뾶
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
//        // �ں�̨�̵߳ȴ����߳��������
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
    // ʹ�� TWeakObjectPtr ��ȫ������ World
    TWeakObjectPtr<UWorld> WeakWorld = World;
    if (!WeakWorld.IsValid())
    {
        UE_LOG(LogTemp, Warning, TEXT("World is nullptr"));
        return false;
    }

    // 1. ����ִ�п��ٵļ�����ײ��⣨����Բ����ģ�
    for (int32 i = 1; i < Trajectory.Num(); ++i)
    {
        FVector StartPoint = Trajectory[i - 1];
        FVector EndPoint = Trajectory[i];

        // ���˵��Ƿ����ϰ�����
        for (const FCylindricalInterestPoint& Obstacle : Obstacles)
        {
            if (IsPointInCylinder(StartPoint, Obstacle) || IsPointInCylinder(EndPoint, Obstacle))
            {
                return false; // �켣�����ϰ����ڣ���ײʧ��
            }
        }

        // ����߶���Բ������ཻ���
        for (const FCylindricalInterestPoint& Obstacle : Obstacles)
        {
            float MinDist = ComputeLineObstacleDistance(StartPoint, EndPoint, Obstacle);
            if (MinDist <= (Obstacle.Radius + Threshold))
            {
                return false; // �߶���Բ�����ཻ����ײʧ��
            }
        }
    }

    // 2. ������ײ�����Ҫ����Ϸ�߳���ִ�У���������Ҫ����ݹ����FlushRenderingCommands
    // ʹ��һ����־��ͬ���첽�����Ľ��
    bool bResult = true;
    FThreadSafeBool bResultReady = false;

    // ׼�����β�����
    TArray<FVector> AllSamplePoints;
    for (int32 i = 1; i < Trajectory.Num(); ++i)
    {
        FVector StartPoint = Trajectory[i - 1];
        FVector EndPoint = Trajectory[i];
        float SegmentLength = FVector::Dist(StartPoint, EndPoint);

        if (SegmentLength < KINDA_SMALL_NUMBER)
            continue;

        // ʹ������Ӧ�����ܶ�
        int32 NumSteps = FMath::CeilToInt(SegmentLength / Threshold);
        NumSteps = FMath::Clamp(NumSteps, 1, 20); // ���Ʋ���������

        for (int32 Step = 0; Step <= NumSteps; ++Step)
        {
            float t = static_cast<float>(Step) / static_cast<float>(NumSteps);
            FVector SamplePoint = FMath::Lerp(StartPoint, EndPoint, t);
            AllSamplePoints.Add(SamplePoint);
        }
    }

    // ����Ϸ�߳���ִ����ײ��⣬�����ȴ����
    AsyncTask(ENamedThreads::GameThread, [WeakWorld, AllSamplePoints, Threshold, &bResult, &bResultReady]()
        {
            if (!WeakWorld.IsValid())
            {
                bResult = false;
                bResultReady = true;
                return;
            }

            UWorld* LocalWorld = WeakWorld.Get();

            // ����������ײ���
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
                        SamplePoint, // ��ͬ�������յ��ʾ��̬���
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

    // 3. ��������߲���Ҫ�����������������һ���ص������ǵȴ�
    // ���������������������Ҫͬ���ȴ����
    // ��Ҫ��ʹ��һ������������ѯ�����������ȴ�������ݹ�FlushRenderingCommands

    // ��ѯ�ȴ���������ȴ�5��
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
        // ��ʱ�����صط�����ײ
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
    // ������������
    float SearchProgress = (float)CurrentIteration / (float)TotalIterations;

    // ��̬����Ŀ��ƫ�����
    float AdjustedGoalBiasProb = GoalBiasProb;
    if (SearchProgress < 0.3f) {
        // �������ڽ���Ŀ��ƫ��
        AdjustedGoalBiasProb *= 0.7f;
    }
    else if (SearchProgress > 0.7f) {
        // �����������Ŀ��ƫ��
        AdjustedGoalBiasProb = FMath::Min(AdjustedGoalBiasProb * 1.5f, 0.5f);
    }
    else {
        // �������ڸ��ݳɹ��ʵ���
        if (SuccessRate < 0.3f) {
            AdjustedGoalBiasProb *= 0.8f; // �ɹ��ʵ�ʱ����Ŀ��ƫ����ǿ̽��
        }
    }

    // ʹ�ö�̬Ŀ��ƫ�����
    if (FMath::FRand() < AdjustedGoalBiasProb) {
        return EndLocation; // Ŀ��ƫ��
    }

    // ��������ʽ��������
    float HeuristicProb = 0.4f;
    if (SuccessRate < 0.3f) {
        HeuristicProb = 0.6f; // �ɹ��ʵ�ʱ��������ʽ��������
    }

    // ������֮��������� 
    if (FMath::FRand() < HeuristicProb && ForwardTree.Num() > 0 && BackwardTree.Num() > 0) {
        // ���ѡ���������еĽڵ�
        int32 ForwardIdx = FMath::RandRange(0, ForwardTree.Num() - 1);
        int32 BackwardIdx = FMath::RandRange(0, BackwardTree.Num() - 1);

        FVector ForwardPoint = ForwardTree[ForwardIdx]->Point;
        FVector BackwardPoint = BackwardTree[BackwardIdx]->Point;

        // �����߸����������
        float Alpha = FMath::FRand(); // �����ֵ����
        FVector MidPoint = FMath::Lerp(ForwardPoint, BackwardPoint, Alpha);

        // ���е㸽���������Ŷ�
        float MaxDeviation = FVector::Distance(ForwardPoint, BackwardPoint) * 0.3f;
        FVector RandomOffset(
            FMath::FRandRange(-MaxDeviation, MaxDeviation),
            FMath::FRandRange(-MaxDeviation, MaxDeviation),
            FMath::FRandRange(-MaxDeviation, MaxDeviation)
        );

        return MidPoint + RandomOffset;
    }

    // Ĭ���������
    return FMath::RandPointInBox(SearchSpace);
}


double UMyRRTClass::GetAdaptiveConnectThreshold(
    double BaseThreshold,
    int32 CurrentIter,
    int32 MaxIter,
    float CurrentSuccessRate)
{
    // 1. ���ݵ������ȵ���
    float ProgressFactor = 1.0f + 0.5f * (static_cast<float>(CurrentIter) / static_cast<float>(MaxIter));

    // 2. ���ݳɹ��ʵ���
    float SuccessFactor = 1.0f;
    if (CurrentSuccessRate < 0.2f) {
        // �ɹ��ʵ�ʱ�ſ���ֵ
        SuccessFactor = 1.2f;
    }
    else if (CurrentSuccessRate > 0.6f) {
        // �ɹ��ʸ�ʱ�ս���ֵ
        SuccessFactor = 0.9f;
    }

    // 3. �����ϴ����ӳ��ԵĽӽ��̶ȵ���
    float DistanceFactor = 1.0f;
    if (LastConnectDistance < BaseThreshold * 5.0f && LastConnectDistance > BaseThreshold) {
        // ����ӽ���δ�ɹ����ʵ��ſ���ֵ
        DistanceFactor = 1.1f;
    }

    return BaseThreshold * ProgressFactor * SuccessFactor * DistanceFactor;
}


RRTNode* UMyRRTClass::ExtendBiRRTTree(TArray<RRTNode*>& ActiveTree, const FVector& Target,
    double StepSize, const TArray<FCylindricalInterestPoint>& Obstacles, double NeighborRadius)
{
    // �ҵ���Ŀ������Ľڵ�
    int32 NearestIdx = NearestNodeIndex(ActiveTree, Target);
    RRTNode* NearestNode = ActiveTree[NearestIdx];

    // ������չ���������µ�
    FVector Direction = (Target - NearestNode->Point).GetSafeNormal();
    FVector NewPoint = NearestNode->Point + Direction * StepSize;

    // ����µ��Ƿ����ϰ�����
    if (IsInObstacleLocal(NewPoint, Obstacles))
    {
        return nullptr;
    }

    // ��� NearestNode �� NewPoint ֮���·�����Ƿ����ϰ����ཻ
    if (LineIntersectsObstacles(NearestNode->Point, NewPoint, Obstacles))
    {
        return nullptr; // ���·�������ϰ����ཻ���ܾ�����½ڵ�
    }

    // ��ʼ���½ڵ�ĳɱ�
    double NewCost = NearestNode->Cost + FVector::Distance(NearestNode->Point, NewPoint);
    RRTNode* NewNode = new RRTNode(NewPoint, NearestNode, NewCost);

    // ���������ڵĽڵ�
    TArray<int32> Neighbors = RadiusSearch(ActiveTree, NewPoint, NeighborRadius);

    // ѡ�����Ÿ��ڵ�
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

    // ����½ڵ㵽��
    ActiveTree.Add(NewNode);

    // ���³ɹ�����
    SuccessfulExtensions++;

    // �ؽ������ڵĽڵ㣬���Ż�·���ɱ�
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

    // 1. ��ForwardNode���ݵ���㣬�ռ�ǰ��·��
    TArray<RRTNode*> ForwardNodes;
    RRTNode* Current = ForwardNode;
    while (Current)
    {
        ForwardNodes.Insert(Current, 0); // ���뵽��ͷ��������ȷ˳��
        Current = Current->Parent;
    }

    // 2. ��BackwardNode���ݵ��յ㣬�ռ�����·��
    TArray<RRTNode*> BackwardNodes;
    Current = BackwardNode;
    while (Current)
    {
        BackwardNodes.Add(Current); // ��ӵ�ĩβ
        Current = Current->Parent;
    }

    // 3. ��������·��
    // ���ǰ��·����
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

    // ֻ�е�ǰ��ͺ���ڵ㲻�غ�ʱ��������ӵ�
    float ConnectionDistance = FVector::Distance(ForwardNode->Point, BackwardNode->Point);
    if (ConnectionDistance > 1.0f) // ʹ��һ��С����ֵ���ж��Ƿ��غ�
    {
        // ������ӵ�
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

    // ��Ӻ���·���㣨�����Ҫ��ת��
    if (bReverse)
    {
        // ������һ���ڵ㣬��Ϊ�������ӵ�̫��
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
        // �������Ҫ��ת��������Ӻ���·����
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

    // ��֤·����������
    if (CompletePath.Num() < 2)
    {
        UE_LOG(LogTemp, Warning, TEXT("ExtractPath generated invalid path with less than 2 points"));
        return TArray<FVector>();
    }

    // ��֤�����յ�
    float StartDistance = FVector::Distance(CompletePath[0], ForwardNodes[0]->Point);
    float EndDistance = FVector::Distance(CompletePath.Last(), BackwardNodes.Last()->Point);

    if (StartDistance > 1.0f || EndDistance > 1.0f)
    {
        UE_LOG(LogTemp, Warning, TEXT("ExtractPath: Path endpoints don't match original points. Start diff: %f, End diff: %f"),
            StartDistance, EndDistance);
    }

    // �Ƴ����ܵ��ظ���
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
    // ��鼸���ϰ��Բ���壩
    for (const FCylindricalInterestPoint& Obstacle : Obstacles)
    {
        if (IsPointInCylinder(Point, Obstacle))
        {
            return true; // ����Բ�����ϰ�����
        }
    }

    // ʹ�� TWeakObjectPtr ��ȫ������ World
    TWeakObjectPtr<UWorld> WeakWorld = World;

    if (!WeakWorld.IsValid())
    {
        UE_LOG(LogTemp, Warning, TEXT("World is invalid"));
        return false;
    }

    UWorld* LocalWorld = WeakWorld.Get();

    // ���� SweepSingleByChannel ����ײ��״
    FHitResult HitResult;
    FVector TraceStart = Point - FVector(1.0f, 1.0f, 1.0f) * Threshold;
    FVector TraceEnd = Point + FVector(1.0f, 1.0f, 1.0f) * Threshold;

    FCollisionQueryParams QueryParams;
    QueryParams.bTraceComplex = true; // ��⸴����ײ����

    float SphereRadius = 200.0f; // ��̬��������뾶

    // ִ����ײ���
    if (LocalWorld->SweepSingleByChannel(
        HitResult,
        TraceStart,
        TraceEnd,
        FQuat::Identity,       // û����ת
        ECC_WorldDynamic,        // Ĭ�Ͽɼ���ͨ��
        FCollisionShape::MakeSphere(SphereRadius), // ����������״
        QueryParams))
    {
        //UE_LOG(LogTemp, Warning, TEXT("Point is within an obstacle detected by SweepSingleByChannel"));
        return true; // ��⵽��ײ
    }

    return false; // δ��⵽��ײ
}


bool UMyRRTClass::LineIntersectsObstacles(const FVector& Start, const FVector& End,
    const TArray<FCylindricalInterestPoint>& Obstacles,
    float Threshold) const
{
    // ���˵��Ƿ����ϰ�����
    if (IsInObstacleLocal(Start, Obstacles) || IsInObstacleLocal(End, Obstacles))
    {
        return true;
    }

    // ����߶��Ƿ����
    if (FVector::Distance(Start, End) < 1.0f)
    {
        return false; // �߶ι��̣������ܷ�����ײ
    }

    // ʹ�� TWeakObjectPtr ��ȫ������ World
    TWeakObjectPtr<UWorld> WeakWorld = World;

    if (!WeakWorld.IsValid())
    {
        UE_LOG(LogTemp, Warning, TEXT("World is invalid"));
        return false;
    }

    UWorld* LocalWorld = WeakWorld.Get();

    // �����Բ�����ϰ������ײ
    for (const FCylindricalInterestPoint& Obstacle : Obstacles)
    {
        float MinDist = ComputeLineObstacleDistance(Start, End, Obstacle);
        if (MinDist <= (Obstacle.Radius + Threshold))
        {
            return true;
        }
    }

    float SphereRadius = 200.0f; // ��̬��������뾶
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
    // ��ȡ�߶�����
    FVector LineDir = (End - Start).GetSafeNormal();

    // Բ����ײ����ĵ���������
    FVector ToStart = Start - Obstacle.BottomCenter;

    // ���㴹ֱͶӰ
    float DotProduct = FVector::DotProduct(ToStart, LineDir);
    FVector Projection = Start + LineDir * FMath::Clamp(-DotProduct, 0.0f, FVector::Distance(Start, End));

    // ����ͶӰ�㵽Բ�������ĵ�ˮƽ����
    FVector ToCylinder = Projection - Obstacle.BottomCenter;
    float HorizontalDist = FMath::Sqrt(ToCylinder.X * ToCylinder.X + ToCylinder.Y * ToCylinder.Y);

    // ���Ǹ߶�Լ��
    float VerticalDist = ToCylinder.Z;
    if (VerticalDist < 0.0f || VerticalDist > Obstacle.Height)
    {
        // �����Բ����߶ȷ�Χ�⣬����һ���ܴ�ľ���
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
    // ʹ��RRT���ɳ�ʼ·��
    TArray<FPathPointWithOrientation> RawPath = GenerateRRTPath(StartLocation, EndLocation, InterestPoints);

    if (RawPath.Num() <= 1)
    {
        return RawPath;
    }

    // ת��ΪFVector������㳤��
    TArray<FVector> RawVectorPath;
    RawVectorPath.Reserve(RawPath.Num());
    for (const FPathPointWithOrientation& Node : RawPath)
    {
        RawVectorPath.Add(Node.Point);
    }

    // ����·���ܳ���
    float TotalLength = 0.0f;
    for (int32 i = 1; i < RawVectorPath.Num(); ++i)
    {
        TotalLength += FVector::Dist(RawVectorPath[i], RawVectorPath[i - 1]);
    }

    // �����ܳ��Ⱥ�MaxSegmentLength������Ҫ�ĵ���
    int32 NumPoints = FMath::CeilToInt(TotalLength / MaxSegmentLength) + 1;
    // ȷ��������MinPoints
    NumPoints = FMath::Max(NumPoints, MinPoints);

    // ��·������������ֵ��ƽ��
    TArray<FVector> SmoothedVectorPath = SmoothPathWithBSplineDynamic(RawVectorPath, BaseStepSize,  NumPoints, CurvatureFactor);

    int32 RetryCount = 0;
    while (!IsTrajectoryCollisionFree(SmoothedVectorPath, InterestPoints) && RetryCount < MaxRetries)
    {
        CurvatureFactor *= 0.9f;
        // ���ʱ仯������Ҫ���¼���NumPoints��
        // һ�㲻��ı�������ԣ��ɱ���NumPoints�����������һ�Ρ�
        // ���ּ򻯣����ﲻ�ٸı�NumPoints��
        SmoothedVectorPath = SmoothPathWithBSplineDynamic(RawVectorPath, BaseStepSize, NumPoints, CurvatureFactor);
        ++RetryCount;
    }

    // ��������ײ�����˻�ԭʼ·��
    if (!IsTrajectoryCollisionFree(SmoothedVectorPath, InterestPoints))
    {
        UE_LOG(LogTemp, Warning, TEXT("Failed to smooth path after %d retries!"), RetryCount);
        return RawPath;
    }

    // ת����FPathPointWithOrientation����
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

    FilteredPath.Add(Path[0]); // ������һ����
    FVector LastPoint = Path[0];

    for (int32 i = 1; i < Path.Num(); ++i)
    {
        float Distance = FVector::Dist(LastPoint, Path[i]);
        if (Distance >= MinDisBetweenPoints)
        {
            FilteredPath.Add(Path[i]);
            LastPoint = Path[i]; // ������һ����
        }
    }

    // ȷ���������һ����
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

    MergedSegment.Add(Segment[0]); // �����ε����
    FVector LastPoint = Segment[0];

    for (int32 i = 1; i < Segment.Num(); ++i)
    {
        FVector CurrentPoint = Segment[i];
        float Distance = FVector::Dist(LastPoint, CurrentPoint);

        // ���ھ��볬����ֵʱ������
        if (Distance >= MinDisBetweenPoints)
        {
            MergedSegment.Add(CurrentPoint);
            LastPoint = CurrentPoint;
        }
        else
        {
            // �������С����ֵ�����·�������ʻ�Ƕȱ仯
            if (i < Segment.Num() - 1)
            {
                FVector NextPoint = Segment[i + 1];
                FVector Dir1 = (CurrentPoint - LastPoint).GetSafeNormal();
                FVector Dir2 = (NextPoint - CurrentPoint).GetSafeNormal();
                float DotProduct = FVector::DotProduct(Dir1, Dir2);

                // ����Ƕȱ仯�ϴ󣬱����õ�
                if (DotProduct < 0.9) // 0.9 ��һ����ֵ�����Ը���ʵ���������
                {
                    MergedSegment.Add(CurrentPoint);
                    LastPoint = CurrentPoint;
                }
            }
        }
    }

    // ȷ���ε��յ㱣��
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

    // ��ʼ��
    FilteredPath.Add(Path[0]); // ������һ����
    FVector LastPoint = Path[0];

    // ��ǰ�γ�ʼ��Ϊ��
    TArray<FVector> CurrentSegment;

    for (int32 i = 1; i < Path.Num(); ++i)
    {
        FVector CurrentPoint = Path[i];
        float Distance = FVector::Dist(LastPoint, CurrentPoint);

        // ������೬�� MinDisBetweenPoints�����Դ���ǰ��
        if (Distance >= MinDisBetweenPoints)
        {
            // �����ǰ��ֻ��һ���㣬ֱ�Ӽ�������·��
            if (CurrentSegment.Num() == 1)
            {
                FilteredPath.Add(CurrentSegment[0]);
            }
            else if (CurrentSegment.Num() > 1)
            {
                // ���Ժϲ���ǰ��
                TArray<FVector> MergedSegment = MergeSegment(CurrentSegment, MinDisBetweenPoints);

                // ���ϲ���Ķ��Ƿ�����ײ
                if (!IsTrajectoryCollisionFree(MergedSegment, InterestPoints))
                {
                    // ��������ײ������ԭʼ��
                    FilteredPath.Append(CurrentSegment);
                }
                else
                {
                    // ������ײ������ϲ���Ķ�
                    FilteredPath.Append(MergedSegment);
                }
            }

            // ��ʼ�µĶ�
            CurrentSegment.Empty();
            CurrentSegment.Add(CurrentPoint); // ��ǰ��Ϊ�¶����
        }
        else
        {
            // ��ǰ�����ڵ�ǰ����
            CurrentSegment.Add(CurrentPoint);
        }

        LastPoint = CurrentPoint; // ������һ����
    }

    // �������һ����
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

    // ȷ���յ㱣��
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
    // ���ø��ٱ���
    CurrentIteration = 0;
    SuccessfulExtensions = 0;
    SuccessRate = 1.0f; // ��ʼ����ɹ��ʸ�

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
    
    // ���������ռ�
    FBox BoundingBox(StartLocation, EndLocation);
    // ��ȡ������İ�ߴ�
    FVector BoxExtent = BoundingBox.GetExtent();

    // ������չ�����͹̶��߶���չֵ
    float HorizontalExpansionRatio = 0.2f; // ˮƽ������չ������X �� Y��
    float FixedVerticalExpansion = 2000.0f; // �߶ȷ���̶���չֵ

    // ����ˮƽ�������չֵ
    FVector Expansion(
        BoxExtent.X * HorizontalExpansionRatio,
        BoxExtent.Y * HorizontalExpansionRatio,
        FixedVerticalExpansion // �߶ȷ���ֱ��ʹ�ù̶�ֵ
    );
    FVector BoxMin = BoundingBox.Min - FVector(Expansion);
    FVector BoxMax = BoundingBox.Max + FVector(Expansion);
    FBox SearchSpace(BoxMin, BoxMax);

    // ����RRT����
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

        // ���³ɹ���
        if (Iter > 0) {
            SuccessRate = static_cast<float>(SuccessfulExtensions) / static_cast<float>(Iter);
        }

        // ������ƽ����ʣ���������չС��
        float ForwardProb = static_cast<float>(BiRRT.BackwardTree.Num()) /
            (BiRRT.ForwardTree.Num() + BiRRT.BackwardTree.Num());

        bool bGrowForward = FMath::FRand() < ForwardProb;
        TArray<RRTNode*>& ActiveTree = bGrowForward ? BiRRT.ForwardTree : BiRRT.BackwardTree;
        TArray<RRTNode*>& OtherTree = bGrowForward ? BiRRT.BackwardTree : BiRRT.ForwardTree;

        // ʹ������ʽ����
        FVector Target = HeuristicSample(
            BiRRT.ForwardTree,
            BiRRT.BackwardTree,
            SearchSpace,
            BaseGoalBias,
            LocalStartLocation,
            LocalEndLocation
        );

        // ��չ��
        RRTNode* NewNode = ExtendBiRRTTree(ActiveTree, Target, StepSize, InterestPoints, NeighborRadius);
        if (!NewNode) continue;

        // ����Ӧ������ֵ
        double CurrentConnectThreshold = GetAdaptiveConnectThreshold(
            BaseConnectThreshold,
            Iter,
            TotalIterations,
            SuccessRate
        );

        // ��������
        int32 NearestOtherIdx = NearestNodeIndex(OtherTree, NewNode->Point);
        RRTNode* NearestOther = OtherTree[NearestOtherIdx];
        float Distance = FVector::Distance(NewNode->Point, NearestOther->Point);
        LastConnectDistance = Distance; // ��¼���ӳ��Ծ���

        if (Distance < CurrentConnectThreshold &&
            !LineIntersectsObstacles(NewNode->Point, NearestOther->Point, InterestPoints))
        {
            if (Distance < BestDistance)
            {
                BestDistance = Distance;
                ConnectingForwardNode = bGrowForward ? NewNode : NearestOther;
                ConnectingBackwardNode = bGrowForward ? NearestOther : NewNode;

                // ��������С��ֱ�ӽ�������
                if (Distance < StepSize * 0.5f)
                {
                    break;
                }
            }
        }

        // �����Լ���Ƿ���Խ�������
        if (Iter % 100 == 0 && ConnectingForwardNode != nullptr)
        {
            if (BestDistance < StepSize)
            {
                break;
            }
        }
    }

    // ����Ƿ��ҵ���Ч·��
    if (!ConnectingForwardNode || !ConnectingBackwardNode ||
        BestDistance > BaseConnectThreshold * 2.0f)
    {
        UE_LOG(LogTemp, Warning, TEXT("BiRRT failed to find path (Distance: %f), using backup method"),
            BestDistance);
        return GenerateAndSmoothRRTPath_BackUp(StartLocation, EndLocation, InterestPoints);
    }

    // ��ȡ·��
    TArray<FVector> Path = ExtractPath(ConnectingForwardNode, ConnectingBackwardNode, true);

    if (!IsTrajectoryCollisionFree(Path, InterestPoints))
    {
		UE_LOG(LogTemp, Error, TEXT("BiRRT path contains collisions!"));
    }

    // ·��ƽ��
	// Ҳ���ö�������ƽ��
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
    int32 Degree = (CurvatureFactor > 1.5f) ? 4 : 3; // ���ʽϴ�ʱʹ��4��

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

    // �� SmoothedVectorPath ���л��ھ������ײ��������Ż�
    TArray<FVector> OptimizedPath = FilterPathByMinDistanceWithCollisionCheck(SmoothedVectorPath, MinDisBetweenPoints, InterestPoints);

    // ���ɸѡ���·���Ƿ�����ײ
    if (!IsTrajectoryCollisionFree(OptimizedPath, InterestPoints))
    {
        UE_LOG(LogTemp, Warning, TEXT("Filtered path contains collisions!"));
        // �����ǰ�����յ�
		//UE_LOG(LogTemp, Warning, TEXT("Start: %s, End: %s"), *StartLocation.ToString(), *EndLocation.ToString());
        if (!IsTrajectoryCollisionFree(SmoothedVectorPath, InterestPoints)) {
			UE_LOG(LogTemp, Warning, TEXT("Even Smoothed path contains collisions?"));
        }

		TestPoints = OptimizedPath;
        OptimizedPath = SmoothedVectorPath;
    }

    // ת����FPathPointWithOrientation
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

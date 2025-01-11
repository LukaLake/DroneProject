// Fill out your copyright notice in the Description page of Project Settings.


#include "MyTSPClass.h"

UMySTSPClass::UMySTSPClass()
{
    NumRegions = 8;
    MutationRate = 0.05;
    CrossoverRate = 0.9;
    PopulationSize = 200;
    Generations = 500;
}

UMySTSPClass::~UMySTSPClass()
{
}


TArray<int32> UMySTSPClass::GeneticAlgorithm(const TArray<FDoubleArray>& CostMatrix, int32 InNumRegions)
{
    NumRegions = InNumRegions;

    // ��ʼ����Ⱥ
    TArray<TArray<int32>> Population;
    Population.Reserve(PopulationSize);
    for (int32 i = 0; i < PopulationSize; ++i)
    {
        Population.Add(GenerateIndividual(NumRegions));
    }

    // �Ŵ�����
    for (int32 Generation = 0; Generation < Generations; ++Generation)
    {
        TArray<TArray<int32>> SelectedPopulation = SelectPopulation(Population, CostMatrix);
        TArray<TArray<int32>> NextGeneration;

        while (NextGeneration.Num() < PopulationSize)
        {
            int32 ParentIndex1 = FMath::RandRange(0, SelectedPopulation.Num() - 1);
            int32 ParentIndex2 = FMath::RandRange(0, SelectedPopulation.Num() - 1);

            TArray<int32> Child;
            if (RandDouble() < CrossoverRate)
            {
                Child = Crossover(SelectedPopulation[ParentIndex1], SelectedPopulation[ParentIndex2]);
            }
            else
            {
                Child = SelectedPopulation[ParentIndex1];
            }

            Mutate(Child);
            NextGeneration.Add(Child);
        }

        Population = NextGeneration;
    }

    // Ѱ����ѽ�
    TArray<int32> BestPath;
    double BestCost = TNumericLimits<double>::Max();

    for (const TArray<int32>& Individual : Population)
    {
        double Cost = ComputePathCost(Individual, CostMatrix);
        if (Cost < BestCost)
        {
            BestCost = Cost;
            BestPath = Individual;
        }
    }

    return BestPath;
}

TArray<int32> UMySTSPClass::GenerateIndividual(int32 _NumRegions)
{
    TArray<int32> Individual;
    // Individual ������ Start (0) �� End (1)
    // ÿ�������������ڵ㣺Entry (2 + 2*i) �� Exit (3 + 2*i)

    // Step 1: ��������ڵ�ԣ����������ÿ�Ե��ڲ�˳��
    TArray<TArray<int32>> RegionNodePairs;
    RegionNodePairs.Reserve(_NumRegions);

    for (int32 i = 0; i < _NumRegions; ++i)
    {
        int32 Entry = 2 + 2 * i;
        int32 Exit = Entry + 1;

        if (FMath::RandBool())
        {
            RegionNodePairs.Add({ Entry, Exit });
        }
        else
        {
            RegionNodePairs.Add({ Exit, Entry });
        }
    }

    // Step 2: ʹ�� Fisher-Yates �㷨��������ڵ�Ե�˳��
    for (int32 i = RegionNodePairs.Num() - 1; i > 0; --i)
    {
        int32 j = FMath::RandRange(0, i);
        RegionNodePairs.Swap(i, j);
    }

    // Step 3: չ������ڵ�ԣ��γ����յĸ�������
    for (const TArray<int32>& Pair : RegionNodePairs)
    {
        Individual.Append(Pair);
    }

    return Individual;
}


TArray<TArray<int32>> UMySTSPClass::SelectPopulation(const TArray<TArray<int32>>& Population, const TArray<FDoubleArray>& CostMatrix)
{
    TArray<TPair<double, TArray<int32>>> Fitness;
    for (const TArray<int32>& Individual : Population)
    {
        double Cost = ComputePathCost(Individual, CostMatrix);
        Fitness.Emplace(Cost, Individual);
    }

    Fitness.Sort([](const TPair<double, TArray<int32>>& A, const TPair<double, TArray<int32>>& B) {
        return A.Key < B.Key;
        });

    TArray<TArray<int32>> SelectedPopulation;
    SelectedPopulation.Reserve(Fitness.Num() / 2);

    for (int32 i = 0; i < Fitness.Num() / 2; ++i)
    {
        SelectedPopulation.Add(Fitness[i].Value);
    }

    return SelectedPopulation;
}

// ����һ���ڵ�� Pair��������˳�������̶���
// �������ý�С�Ľڵ���ǰ
TArray<int32> NormalizePair(const TArray<int32>& Pair)
{
    check(Pair.Num() == 2);

    if (Pair[0] <= Pair[1])
    {
        return Pair;
    }
    else
    {
        return { Pair[1], Pair[0] };
    }
}

// �� ChildPairs ���ѳ��ֵ� Pair�����Ƿ�� Parent2Index ָ��� Pair �ǡ�ͬһ�����򡿡�
bool AlreadyHasSameRegion(const TArray<TArray<int32>>& ChildPairs, const TArray<int32>& PairToCheck)
{
    TArray<int32> NormalizedPair = NormalizePair(PairToCheck);

    for (const TArray<int32>& CP : ChildPairs)
    {
        // �����յĻ򳤶� != 2 ��
        if (CP.Num() != 2)
        {
            continue;
        }

        TArray<int32> NormalizedCP = NormalizePair(CP);
        if (NormalizedCP[0] == NormalizedPair[0] && NormalizedCP[1] == NormalizedPair[1])
        {
            return true;
        }
    }

    return false;
}

TArray<int32> UMySTSPClass::Crossover(const TArray<int32>& Parent1, const TArray<int32>& Parent2)
{
    int32 NumPairs = Parent1.Num() / 2; // ÿ�����ڵ�Ϊһ��
    TArray<TArray<int32>> Parent1Pairs;
    TArray<TArray<int32>> Parent2Pairs;
    TArray<TArray<int32>> ChildPairs;
    ChildPairs.SetNum(NumPairs);

    // ��ȡ�����Ľڵ��
    for (int32 i = 0; i < Parent1.Num(); i += 2)
    {
        Parent1Pairs.Add({ Parent1[i], Parent1[i + 1] });
        Parent2Pairs.Add({ Parent2[i], Parent2[i + 1] });
    }

    // ���򽻲棨OX��
    int32 CutPoint1 = FMath::RandRange(0, NumPairs - 1);
    int32 CutPoint2 = FMath::RandRange(0, NumPairs - 1);

    if (CutPoint1 > CutPoint2)
    {
        Swap(CutPoint1, CutPoint2);
    }

    // �� Parent1 ����һ���ֵ� Child
    for (int32 i = CutPoint1; i <= CutPoint2; ++i)
    {
        ChildPairs[i] = Parent1Pairs[i];
    }

    // ���ʣ��Ľڵ������ Parent2�������Ѵ��ڵĶ�
    int32 Parent2Index = 0;
    for (int32 i = 0; i < NumPairs; ++i)
    {
        if (i >= CutPoint1 && i <= CutPoint2)
        {
            continue;
        }

        while (Parent2Index < NumPairs && AlreadyHasSameRegion(ChildPairs, Parent2Pairs[Parent2Index]))
        {
            Parent2Index++;
        }

        if (Parent2Index < NumPairs)
        {
            ChildPairs[i] = Parent2Pairs[Parent2Index];
            Parent2Index++;
        }
    }

    // չ���ڵ���γ� Child
    TArray<int32> Child;
    Child.Reserve(NumPairs * 2);
    for (const TArray<int32>& Pair : ChildPairs)
    {
        Child.Append(Pair);
    }

    return Child;
}


void UMySTSPClass::Mutate(TArray<int32>& Individual)
{
    if (FMath::FRand() < MutationRate)
    {
        int32 NumPairs = Individual.Num() / 2;
        int32 PairIndex1 = FMath::RandRange(0, NumPairs - 1);
        int32 PairIndex2 = FMath::RandRange(0, NumPairs - 1);

        if (PairIndex1 != PairIndex2)
        {
            // ���������ڵ��
            for (int32 i = 0; i < 2; ++i)
            {
                Individual.Swap(PairIndex1 * 2 + i, PairIndex2 * 2 + i);
            }
        }
    }
}


double UMySTSPClass::ComputePathCost(const TArray<int32>& Individual, const TArray<FDoubleArray>& CostMatrix)
{
    double TotalCost = 0.0;

    // Start node (0)
    int32 PreviousNode = 0;

    // Traverse through the individual's node sequence
    for (int32 CurrentNode : Individual)
    {
        TotalCost += CostMatrix[PreviousNode].Values[CurrentNode];
        PreviousNode = CurrentNode;
    }

    // Finally, go to End node (1)
    TotalCost += CostMatrix[PreviousNode].Values[1];

    return TotalCost;
}

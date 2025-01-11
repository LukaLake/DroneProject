// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "MyTSPClass.generated.h"

// 定义一个包含 TArray<double> 的自定义结构体
USTRUCT()
struct FDoubleArray
{
    GENERATED_BODY()

    UPROPERTY()
    TArray<double> Values;
};

/**
 * 
 */
UCLASS()
class CESIUM_PROJECT1_API UMySTSPClass : public UObject
{
    GENERATED_BODY()
public:
    UMySTSPClass();
    ~UMySTSPClass();

    // 输入参数：CostMatrix为构建好的代价矩阵，NumRegions为区域数
    // 输出：一个访问区域的序列（如：[2,0,1,...]）
    TArray<int32> GeneticAlgorithm(const TArray<FDoubleArray>& CostMatrix, int32 NumRegions);

protected:
    int32 NumRegions;
    double MutationRate;
    double CrossoverRate;
    int32 PopulationSize;
    int32 Generations;

    TArray<int32> GenerateIndividual(int32 _NumRegions);
    TArray<TArray<int32>> SelectPopulation(const TArray<TArray<int32>>& Population, const TArray<FDoubleArray>& CostMatrix);
    TArray<int32> Crossover(const TArray<int32>& Parent1, const TArray<int32>& Parent2);
    void Mutate(TArray<int32>& Individual);

    double ComputePathCost(const TArray<int32>& Individual, const TArray<FDoubleArray>& CostMatrix);

    // 工具函数
    double RandDouble() { return FMath::FRand(); }
};
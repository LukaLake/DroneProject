// Fill out your copyright notice in the Description page of Project Settings.


#include "MyLevelScriptActor.h"

void AMyLevelScriptActor::BeginPlay()
{
    Super::BeginPlay();
    
}

void AMyLevelScriptActor::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);

    // 在每一帧执行的代码
}

void AMyLevelScriptActor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    Super::EndPlay(EndPlayReason);

    // 在关卡结束时执行的代码
}

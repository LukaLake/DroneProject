// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Engine/LevelScriptActor.h"



#include "MyLevelScriptActor.generated.h"

/**
 * 
 */
UCLASS()
class CESIUM_PROJECT1_API AMyLevelScriptActor : public ALevelScriptActor
{
	GENERATED_BODY()

	protected:
	virtual void BeginPlay() override;

	virtual void Tick(float DeltaSeconds) override;

	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	
};

// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/HUD.h"
#include "AHUD.generated.h"

/**
 * 
 */
UCLASS()
class CESIUM_PROJECT1_API AAHUD : public AHUD
{
	GENERATED_BODY()

protected:
    virtual void DrawHUD() override;

    // Function to convert model coordinates to screen coordinates
    FVector2D ConvertModelCoordToScreen(float ModelX, float ModelY);

private:
    bool bHasLoggedWarning = false;
	bool bHasLoggedReminder = false;
    // Get the class name from the class ID
	FString GetClassName(int32 ClassID); 
};

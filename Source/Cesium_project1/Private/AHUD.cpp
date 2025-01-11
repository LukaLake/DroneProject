// Fill out your copyright notice in the Description page of Project Settings.


#include "AHUD.h"
#include "../DroneActor1.h"
#include "Kismet/GameplayStatics.h"
#include "Engine/Canvas.h"

void AAHUD::DrawHUD()
{
    Super::DrawHUD();

    if (!Canvas)
    {
        UE_LOG(LogTemp, Error, TEXT("Canvas is invalid."));
        return;
    }

    // 获取玩家的 Pawn 并转换为 ADroneActor1
    ADroneActor1* DroneActor = Cast<ADroneActor1>(UGameplayStatics::GetPlayerPawn(GetWorld(), 0));

    if (DroneActor)
    {
        if (DroneActor->YoloTracker.IsValid())
        {
            // 检查是否正在跟踪
            if (DroneActor->YoloTracker->IsTracking()&&(!DroneActor->YoloTracker->ShouldStopTracking()))
            {
				if (!bHasLoggedReminder)
				{
					GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Green, TEXT("Tracking"));
					bHasLoggedReminder = true;
				}
                
                // 获取跟踪的检测结果
                const FDetectionResult& Detection = DroneActor->YoloTracker->GetTrackedDetection();

                // 将模型坐标系中的检测结果转换为屏幕坐标系
                FVector2D ScreenLeftTop = FVector2D(Detection.x1, Detection.y1);
                FVector2D ScreenRightBottom = FVector2D(Detection.x2, Detection.y2);

                // 计算物体边界框的中心点
                FVector2D ObjectCenter = (ScreenLeftTop + ScreenRightBottom) / 2.0f;

                // 计算边界框的宽度和高度
                float ScreenWidth = ScreenRightBottom.X - ScreenLeftTop.X;
                float ScreenHeight = ScreenRightBottom.Y - ScreenLeftTop.Y;

                // 绘制物体边界框的中心点
                float PointSize = 5.0f;
                DrawRect(FLinearColor::Red, ObjectCenter.X - PointSize / 2, ObjectCenter.Y - PointSize / 2, PointSize, PointSize);

                // 绘制边界框
                FLinearColor BoxColor = FLinearColor::Green;
                float LineThickness = 2.0f;
                DrawLine(ScreenLeftTop.X, ScreenLeftTop.Y, ScreenRightBottom.X, ScreenLeftTop.Y, BoxColor, LineThickness); // 上边
                DrawLine(ScreenRightBottom.X, ScreenLeftTop.Y, ScreenRightBottom.X, ScreenRightBottom.Y, BoxColor, LineThickness); // 右边
                DrawLine(ScreenRightBottom.X, ScreenRightBottom.Y, ScreenLeftTop.X, ScreenRightBottom.Y, BoxColor, LineThickness); // 下边
                DrawLine(ScreenLeftTop.X, ScreenRightBottom.Y, ScreenLeftTop.X, ScreenLeftTop.Y, BoxColor, LineThickness); // 左边

                // 绘制类别名称和置信度
                FString ClassName = GetClassName(Detection.ClassID);
                FString Label = FString::Printf(TEXT("%s: %.2f"), *ClassName, Detection.Confidence);
                UFont* Font = GEngine->GetMediumFont();
                float TextScale = 1.0f;
                DrawText(Label, BoxColor, ScreenLeftTop.X, ScreenLeftTop.Y - 20.0f, Font, TextScale);

                // 输出Label
				//UE_LOG(LogTemp, Warning, TEXT("Label: %s"), *Label);
            }
            else
            {
				bHasLoggedReminder = false;

                // 未在跟踪，可以根据需要添加日志或其他处理
                //UE_LOG(LogTemp, Warning, TEXT("YoloTracker is not tracking."));
            }
        }
        else
        {
            //UE_LOG(LogTemp, Error, TEXT("YoloTracker is invalid."));
        }
    }
    else
    {
        //UE_LOG(LogTemp, Error, TEXT("DroneActor is invalid."));
    }
}

// 假如需要的话，从模型坐标系转换为屏幕坐标系
FVector2D AAHUD::ConvertModelCoordToScreen(float ModelX, float ModelY)
{
    float ScreenX = (ModelX / 640.0f) * Canvas->SizeX;
    float ScreenY = (ModelY / 640.0f) * Canvas->SizeY;
    return FVector2D(ScreenX, ScreenY);
}

/* 
 * 根据类别ID获取类别名称

*/
FString AAHUD::GetClassName(int32 ClassID)
{
    static const TArray<FString> ClassNames = {
        TEXT("person"), TEXT("bicycle"), TEXT("car"), TEXT("motorbike"), TEXT("aeroplane"), TEXT("bus"),
        TEXT("train"), TEXT("truck"), TEXT("boat"), TEXT("traffic light"), TEXT("fire hydrant"),
        TEXT("stop sign"), TEXT("parking meter"), TEXT("bench"), TEXT("bird"), TEXT("cat"), TEXT("dog"),
        TEXT("horse"), TEXT("sheep"), TEXT("cow"), TEXT("elephant"), TEXT("bear"), TEXT("zebra"), TEXT("giraffe"),
        TEXT("backpack"), TEXT("umbrella"), TEXT("handbag"), TEXT("tie"), TEXT("suitcase"), TEXT("frisbee"),
        TEXT("skis"), TEXT("snowboard"), TEXT("sports ball"), TEXT("kite"), TEXT("baseball bat"),
        TEXT("baseball glove"), TEXT("skateboard"), TEXT("surfboard"), TEXT("tennis racket"),
        TEXT("bottle"), TEXT("wine glass"), TEXT("cup"), TEXT("fork"), TEXT("knife"), TEXT("spoon"), TEXT("bowl"),
        TEXT("banana"), TEXT("apple"), TEXT("sandwich"), TEXT("orange"), TEXT("broccoli"), TEXT("carrot"),
        TEXT("hot dog"), TEXT("pizza"), TEXT("donut"), TEXT("cake"), TEXT("chair"), TEXT("sofa"), TEXT("potted plant"),
        TEXT("bed"), TEXT("dining table"), TEXT("toilet"), TEXT("tvmonitor"), TEXT("laptop"), TEXT("mouse"),
        TEXT("remote"), TEXT("keyboard"), TEXT("cell phone"), TEXT("microwave"), TEXT("oven"), TEXT("toaster"),
        TEXT("sink"), TEXT("refrigerator"), TEXT("book"), TEXT("clock"), TEXT("vase"), TEXT("scissors"),
        TEXT("teddy bear"), TEXT("hair drier"), TEXT("toothbrush")
    };

    if (ClassNames.IsValidIndex(ClassID))
    {
        return ClassNames[ClassID];
    }
    else
    {
        return TEXT("unknown");
    }
}


// Copyright Epic Games, Inc. All Rights Reserved.
using System.IO;
using UnrealBuildTool;

public class Cesium_project1 : ModuleRules
{
    public Cesium_project1(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

        PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore","RenderCore","UMG","OpenCV","OpenCVHelper",
        "CesiumRuntime","HeadMountedDisplay", "EnhancedInput", "NNE","NNERuntimeORT" });

        //// 设置包含目录的相对路径
        //string IncludeDirectory = Path.Combine(ModuleDirectory, "..", "..", "packages", "Microsoft.ML.OnnxRuntime.Gpu.1.16.3", "build", "native", "include");
        //PrivateIncludePaths.Add(IncludeDirectory);

        //// 设置库文件的相对路径
        //string LibDirectory = Path.Combine(ModuleDirectory, "..", "..", "packages", "Microsoft.ML.OnnxRuntime.Gpu.1.16.3", "runtimes", "win-x64", "native");
        //PublicSystemLibraryPaths.Add(LibDirectory);
        //string LibraryPath = Path.Combine(LibDirectory, "onnxruntime.lib");
        //PublicAdditionalLibraries.Add(LibraryPath);
        //LibraryPath = Path.Combine(LibDirectory, "onnxruntime_providers_cuda.lib");
        //PublicAdditionalLibraries.Add(LibraryPath);
        //LibraryPath = Path.Combine(LibDirectory, "onnxruntime_providers_shared.lib");
        //PublicAdditionalLibraries.Add(LibraryPath);

        //string CudaPath = "C:\\Program Files\\NVIDIA GPU Computing Toolkit\\CUDA\\v11.8";
        //// string CudnnPath = "C:/Program Files/NVIDIA/CUDNN/v8.0";
        //PublicIncludePaths.AddRange(new string[] { Path.Combine(CudaPath, "include") });
        //string CudnnPath = Path.Combine(CudaPath, "lib", "x64");
        //PublicSystemLibraryPaths.Add(CudnnPath);
        //PublicAdditionalLibraries.AddRange(new string[] { Path.Combine(CudnnPath, "cudart_static.lib"), Path.Combine(CudnnPath, "cudnn.lib") });



        PrivateDependencyModuleNames.AddRange(new string[] { });

        // Uncomment if you are using Slate UI
        // PrivateDependencyModuleNames.AddRange(new string[] { "Slate", "SlateCore" });

        // Uncomment if you are using online features
        // PrivateDependencyModuleNames.Add("OnlineSubsystem");

        // To include OnlineSubsystemSteam, add it to the plugins section in your uproject file with the Enabled attribute set to true

        // 设置RTTI
        bUseRTTI = true;




    }
}

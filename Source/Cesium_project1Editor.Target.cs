// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;
using System.Collections.Generic;

public class Cesium_project1EditorTarget : TargetRules
{
	public Cesium_project1EditorTarget( TargetInfo Target) : base(Target)
	{
		Type = TargetType.Editor;
		DefaultBuildSettings = BuildSettingsVersion.V5;
		ExtraModuleNames.AddRange( new string[] { "Cesium_project1" } );
        bOverrideBuildEnvironment = true; // 添加这一行
    }
}

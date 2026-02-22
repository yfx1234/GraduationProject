// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class GraduationProject : ModuleRules
{
	public GraduationProject(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
	
		PublicDependencyModuleNames.AddRange(new string[] {
			"Core", "CoreUObject", "Engine", "InputCore", "EnhancedInput",
			"Sockets", "Networking",
			"Json", "JsonUtilities",
			"RenderCore", "RHI", "ImageWrapper",
			"CinematicCamera"
		});

		PrivateDependencyModuleNames.AddRange(new string[] {
			"Slate", "SlateCore", "UMG"
		});
	}
}

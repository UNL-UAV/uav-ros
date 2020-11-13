outputdir = "%{cfg.buildcfg}/%{cfg.system}-%{cfg.architecture}"

includeDir = {}

ROS_DISTRO = os.getenv("ROS_DISTRO")
ROS_DIR = "/opt/ros/"..ROS_DISTRO

projectName = "Project"

workspace "uav-ros" -- ChangeProject
	location "workspace"
	architecture "x64"

	startproject "Core" -- Change this to what's ever the starting project is 

	configurations{
		"Debug",
		"Release",
		"Dist"
	}

	includedirs{
		"include",
		ROS_DIR.."/include"
	}

	libdirs{
		ROS_DIR.."/lib"
	}
	
	links{
		"roscpp",
		"rostime",
		"rosconsole",
		"roscpp_serialization",
		"boost_system"
	}
	
	pchheader "include/pch.hpp"
	pchsource "src/pch.cpp"

	targetdir ("bin/" .. outputdir)
	objdir ("bin-int/" .. outputdir)

	filter "system:windows"
		defines "_WINDOWS"
	filter "system:linux"
		defines "_LINUX"
		libdirs{
			"/usr/lib/x86_64-linux-gnu"
		}
	filter "system:macosx"
		defines "_OSX"
		
    filter "configurations:Debug"
        defines "_DEBUG"
        symbols "On"
    filter "configurations:Dist"
        defines "_DIST"
        optimize "On"
    filter "configurations:Release"
        defines "_RELEASE"
        optimize "On"

project "Core" -- Default Project
	language "C++"
	cppdialect "C++17"
	kind "SharedLib" -- ConsoleApp / SharedLib
	defines "_DLL" -- If SharedLib

	files{
		"src/core/**.cpp",
		"src/core/**.c"
	}

	includedirs{
	}

	links{

	}

project "Preflight"
	language "C++"
	cppdialect "C++17"
	kind "ConsoleApp"
	
	files{
		"src/Preflight/**.cpp",
		"src/Preflight/**.c"
	}

	includedirs{
		"include/Preflight"
	}

	links{
		"Core"
	}

project "newtakeoff" -- Default Project
	language "C++"
	cppdialect "C++17"
	kind "ConsoleApp" -- ConsoleApp / SharedLib
	-- defines "_DLL" -- If SharedLib

	files{
		"src/newtakeoff/**.cpp",
		"src/newtakeoff/**.c"
	}

	includedirs{
	}

	links{
		"Core"
	}

project "test" -- Default Project
	language "C++"
	cppdialect "C++17"
	kind "ConsoleApp" -- ConsoleApp / SharedLib
	-- defines "_DLL" -- If SharedLib

	files{
		"src/test/**.cpp",
		"src/test/**.c"
	}

	includedirs{
	}

	links{
		"Core"
	}

--[[
project "" -- Default Project
	language "C++"
	cppdialect "C++17"
	kind "" -- ConsoleApp / SharedLib
	-- defines "_DLL" -- If SharedLib

	files{
		"src/**.cpp",
		"src/**.c"
	}

	includedirs{
		"include"
	}

	links{

	}
]]--

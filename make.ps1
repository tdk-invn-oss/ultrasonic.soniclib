<#
.SYNOPSIS
Script to assist with CMake configuraiton and build.

.DESCRIPTION
Use this script to easily configure and build several variants of the soniclib
library. Both chx01 (whitney) and icu (shasta) variants are built.

.PARAMETER Clean
Set to clean the project (no configure/build triggered).

.PARAMETER BuildTypes
Specify the build types to configure and build as a list of strings. Default is
to build Release, MinSizeRel, RelWithDebInfo, and Debug.

.PARAMETER InitFwOptions
(shasta-only) Specify which init FW to use in the build. Options are FULL,
NO_TX_OPTIMIZATION, and NONE. Default is to build all.

.PARAMETER CMakeToolchainFile
Specify the path to the CMakeToolchain file. The deafult is arm-non-eabi-m4.cmake.

.PARAMETER Generator
The CMake generator to use. Default is "MinGW Makefiles".

.PARAMETER ExternalAlgoOptions

Options to use for INCLDUE_ALGO_EXTERNAL. Must be list of OFF, ON.

.PARAMETER SensorTypes

List of sensor types to build for. Options are SHASTA (ICU-x0201) and WHITNEY
(CH-x01).

.PARAMETER Install

Define to run the install step. Currently this just copies the compile_comamnds.json
file to the directory containing the main CMakeLists.txt

.PARAMETER BuildDir

The build directory to create (or use if it already exists).

.INPUTS

None.

.OUTPUTS

None.

.EXAMPLE

PS> .\make.ps1

#>
param(
    [switch]$Clean,
    [string[]]$BuildTypes=("Release", "MinSizeRel", "RelWithDebInfo", "Debug"),
    [string[]]$InitFwOptions=("FULL", "NO_TX_OPTIMIZATION", "NONE"),
    [string]$CMakeToolchainFile="CMakeToolchains/arm-none-eabi-m4.cmake",
    [string]$Generator="MinGW Makefiles",
    [string[]]$ExternalAlgoOptions=("OFF", "ON"),
    [switch]$Install,
    [string[]]$SensorTypes=("SHASTA", "WHITNEY"),
    [string]$BuildDir="build"
)

function Build-Project {
    param(
        $BuildDir
    )
    cmake --build "$BuildDir" -- -j4
    if ($LASTEXITCODE) { throw "make failed with status $LASTEXITCODE." }
}

if ($Clean) {
    Remove-Item -Recurse -Force "$BuildDir" -ErrorAction SilentlyContinue
    exit 0
}

New-Item -ItemType Directory $BuildDir -ErrorAction SilentlyContinue

ForEach ($BuildType in $BuildTypes) {
    # shasta build
    if ($SensorTypes -Contains "SHASTA") {
        ForEach ($InitFwOption in $InitFwOptions) {
            ForEach ($ExternalAlgoOption in $ExternalAlgoOptions) {
                $SubBuildDir = "shasta_$InitFwOption`_extalgo$ExternalAlgoOption`_$BuildType".ToLower()
                $SubBuildDir = Join-Path "$BuildDir" "$SubBuildDir"
                cmake -G "$Generator" `
                    -DCMAKE_TOOLCHAIN_FILE="$CMakeToolchainFile" `
                    -DINCLUDE_SHASTA_SUPPORT:BOOL=ON `
                    -DCMAKE_BUILD_TYPE:STRING="$BuildType" `
                    -DCHIRP_INIT_FW_TYPE="$InitFwOption" `
                    -B "$SubBuildDir" `
                    -DINCLUDE_ALGO_EXTERNAL:BOOL="$ExternalAlgoOption" `
                    -DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=ON
                if ($LASTEXITCODE) { exit $LASTEXITCODE }
                try {
                    Build-Project "$SubBuildDir"
                    if ($Install) {
                        cmake --install "$SubBuildDir"
                    }
                } catch {
                    Write-Output $_
                    exit 1
                }
            }
        }
    }
    # whitney build
    if ($SensorTypes -Contains "WHITNEY") {
        $SubBuildDir = "whitney_$BuildType".ToLower()
        $SubBuildDir = Join-Path "$BuildDir" "$SubBuildDir"
        cmake -G "$Generator" `
            -DCMAKE_TOOLCHAIN_FILE="$CMakeToolchainFile" `
            -DCMAKE_BUILD_TYPE:STRING="$BuildType" `
            -B "$SubBuildDir" `
            -DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=ON
        if ($LASTEXITCODE) { exit $LASTEXITCODE }
        try {
            Build-Project "$SubBuildDir"
            if ($Install) {
                cmake --install "$SubBuildDir"
            }
        } catch {
            Write-Output $_
            exit 1
        }
    }
}

exit 0

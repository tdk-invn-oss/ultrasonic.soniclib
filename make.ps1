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

.PARAMETER RunTests

Define to run the unit tests by calling CTest.

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
    [string[]]$BuildTypes = ("Release", "MinSizeRel", "RelWithDebInfo", "Debug"),
    [string[]]$InitFwOptions = ("FULL", "NO_TX_OPTIMIZATION", "NONE"),
    [string]$CMakeToolchainFile = "CMakeToolchains/arm-none-eabi-m4.cmake",
    [string]$Generator = "MinGW Makefiles",
    [string[]]$ExternalAlgoOptions = ("OFF", "ON"),
    [string[]]$LogLevels = ("Disabled", "Min", "Max"),
    [switch]$Install,
    [switch]$RunTests,
    [switch]$Coverage,
    [string[]]$SensorTypes = ("SHASTA", "WHITNEY"),
    [string]$BuildDir = "build",
    [string]$MingwGccPath
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

$source_dir = "./invn/soniclib/"
$MingwGccPath = $MingwGccPath -replace '\\', '/'

# Copied from ch_log.h
enum ChLogLevels
{
    CH_LOG_LEVEL_TRACE   = 0
    CH_LOG_LEVEL_DEBUG   = 1
    CH_LOG_LEVEL_INFO    = 2
    CH_LOG_LEVEL_WARNING = 3
    CH_LOG_LEVEL_ERROR   = 4
    CH_LOG_LEVEL_APP     = 5
    CH_LOG_LEVEL_DISABLE = 6
}

$RUN_UNIT_TEST = ''
if ($RunTests) {
    $RUN_UNIT_TEST = "-DRUN_UNIT_TEST:BOOL=ON"
}

if ($Coverage) {
    $coverage_dir = "$BuildDir/coverage"
    New-Item -ItemType Directory -Path $coverage_dir -Force | Out-Null

    python -m gcovr `
        --merge-mode-functions=separate `
        --branches `
        --calls `
        --decisions `
        --sort-percentage `
        --html-details "$coverage_dir/coverage.html" `
        --add-tracefile "$BuildDir/**/*_coverage.json" `
        --root $source_dir
    Write-Output "`nMerged coverages available in $coverage_dir/coverage.html"
    exit 0
}

ForEach ($BuildType in $BuildTypes) {
    # shasta build
    if ($SensorTypes -Contains "SHASTA") {
        ForEach ($InitFwOption in $InitFwOptions) {
            ForEach ($ExternalAlgoOption in $ExternalAlgoOptions) {
                ForEach ($LogLevel in $LogLevels) {
                    if ($BuildType -eq "Debug") {
                        Switch($LogLevel) {
                            "Disabled" {$LogLevelInt = ([ChLogLevels]::CH_LOG_LEVEL_DISABLE).value__}
                            "Min" {$LogLevelInt = ([ChLogLevels]::CH_LOG_LEVEL_ERROR).value__}
                            "Max" {$LogLevelInt = ([ChLogLevels]::CH_LOG_LEVEL_TRACE).value__}
                        }
                        $LogLevelInt = ([ChLogLevels]::CH_LOG_LEVEL_ERROR).value__
                    } else {
                        # For other builds, always use the default log level
                        if ($LogLevel -eq "Min") {
                            $LogLevelInt = ([ChLogLevels]::CH_LOG_LEVEL_ERROR).value__
                        } else {
                            # return # continue
                            continue
                        }
                    }
                    $BuildConfig = "shasta_$InitFwOption`_extalgo$ExternalAlgoOption`_$BuildType`_log$LogLevel".ToLower()
                    $SubBuildDir = Join-Path "$BuildDir" "$BuildConfig"
                    cmake -G "$Generator" `
                        -DCMAKE_TOOLCHAIN_FILE="$CMakeToolchainFile" `
                        -DINCLUDE_SHASTA_SUPPORT:BOOL=ON `
                        -DCMAKE_BUILD_TYPE:STRING="$BuildType" `
                        -DCHIRP_INIT_FW_TYPE="$InitFwOption" `
                        -B "$SubBuildDir" `
                        -DINCLUDE_ALGO_EXTERNAL:BOOL="$ExternalAlgoOption" `
                        -DCHIRP_LOG_LEVEL="$LogLevelInt" `
                        -DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=ON `
                        $RUN_UNIT_TEST
                    if ($LASTEXITCODE) { exit $LASTEXITCODE }
                    try {
                        Build-Project "$SubBuildDir"
                        if ($RunTests) {
                            # ctest --test-dir "$SubBuildDir/test/unit_test" --output-junit "junitoutput.xml"  # CMake 3.21+
                            ctest --test-dir "$SubBuildDir/test/unit_test" -VV --output-on-failure --no-compress-output "-D" "ExperimentalTest"

                            # Generate coverage report
                            python -m gcovr `
                                --print-summary `
                                --branches `
                                --calls `
                                --decisions `
                                --json "$SubBuildDir/$BuildConfig`_coverage.json" --json-pretty `
                                --gcov-executable "$MingwGccPath/bin/gcov.exe" `
                                --root $source_dir `
                                 "$SubBuildDir/CMakeFiles/soniclib.dir/invn/soniclib"
                        }
                        if ($Install) {
                            cmake --install "$SubBuildDir"
                        }
                    }
                    catch {
                        Write-Output $_
                        exit 1
                    }
                }
            }
        }
    }
    # whitney build
    if ($SensorTypes -Contains "WHITNEY") {
        ForEach ($LogLevel in $LogLevels) {
            if ($BuildType -eq "Debug") {
                Switch($LogLevel) {
                    "Disabled" {$LogLevelInt = ([ChLogLevels]::CH_LOG_LEVEL_DISABLE).value__}
                    "Min" {$LogLevelInt = ([ChLogLevels]::CH_LOG_LEVEL_ERROR).value__}
                    "Max" {$LogLevelInt = ([ChLogLevels]::CH_LOG_LEVEL_TRACE).value__}
                }
                $LogLevelInt = ([ChLogLevels]::CH_LOG_LEVEL_ERROR).value__
            } else {
                # For other builds, always use the default log level
                if ($LogLevel -eq "Min") {
                    $LogLevelInt = ([ChLogLevels]::CH_LOG_LEVEL_ERROR).value__
                } else {
                    # return # continue
                    continue
                }
            }
            $BuildConfig = "whitney_$BuildType`_log$LogLevel".ToLower()
            $SubBuildDir = Join-Path "$BuildDir" "$BuildConfig"
            cmake -G "$Generator" `
                -DCMAKE_TOOLCHAIN_FILE="$CMakeToolchainFile" `
                -DCMAKE_BUILD_TYPE:STRING="$BuildType" `
                -B "$SubBuildDir" `
                -DCHIRP_LOG_LEVEL="$LogLevelInt" `
                -DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=ON `
                $RUN_UNIT_TEST
            if ($LASTEXITCODE) { exit $LASTEXITCODE }
            try {
                Build-Project "$SubBuildDir"
                if ($RunTests) {
                    # ctest --test-dir "$SubBuildDir/test/unit_test" --output-junit "junitoutput.xml"  # CMake 3.21+
                    ctest --test-dir "$SubBuildDir/test/unit_test" --no-compress-output "-D" "ExperimentalTest"

                    # Generate coverage report
                    python -m gcovr `
                        --print-summary `
                        --branches `
                        --calls `
                        --decisions `
                        --json "$SubBuildDir/$BuildConfig`_coverage.json" --json-pretty `
                        --gcov-executable "$MingwGccPath/bin/gcov.exe" `
                        --root $source_dir `
                         "$SubBuildDir/CMakeFiles/soniclib.dir/invn/soniclib"
                }
                if ($Install) {
                    cmake --install "$SubBuildDir"
                }
            }
            catch {
                Write-Output $_
                exit 1
            }
        }
    }
}

exit 0

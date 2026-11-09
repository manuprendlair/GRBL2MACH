# build_installer.ps1 - builds grbl2mach.exe and its Windows installer in release\
#
# Usage: powershell -File packaging\build_installer.ps1 [-Tools <folder>]
#
# -Tools: folder holding WinLibs (winlibs\mingw64\bin), CMake and Inno Setup (innosetup\ISCC.exe).
# Default: the "tools" folder next to the repository.

param([string]$Tools = (Join-Path $PSScriptRoot "..\..\tools"))

$ErrorActionPreference = "Stop"
$repo = Resolve-Path (Join-Path $PSScriptRoot "..")
$Tools = Resolve-Path $Tools

$env:PATH = (Join-Path $Tools "winlibs\mingw64\bin") + ";" + $env:PATH
$cmake = Get-ChildItem $Tools -Recurse -Filter cmake.exe | Select-Object -First 1 -ExpandProperty FullName
$iscc = Join-Path $Tools "innosetup\ISCC.exe"
if (-not $cmake) { throw "cmake.exe not found in $Tools" }
if (-not (Test-Path $iscc)) { throw "ISCC.exe not found: $iscc" }

$match = Select-String -Path (Join-Path $repo "CMakeLists.txt") -Pattern "project\(GRBL2MACH VERSION ([0-9.]+)"
if (-not $match) { throw "No version in CMakeLists.txt" }
$version = $match.Matches[0].Groups[1].Value

$build = Join-Path $repo "build"
& $cmake -S $repo -B $build -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=gcc
if ($LASTEXITCODE) { throw "CMake configuration failed" }
& $cmake --build $build
if ($LASTEXITCODE) { throw "Build failed" }

$out = Join-Path $repo "release"
New-Item -ItemType Directory -Force $out | Out-Null
& $iscc "/DAppVersion=$version" "/DBuildDir=$build" "/DOutDir=$out" (Join-Path $PSScriptRoot "grbl2mach.iss")
if ($LASTEXITCODE) { throw "Inno Setup failed" }

Get-Item (Join-Path $out "GRBL2MACH-$version-setup.exe") | Select-Object Name, Length, LastWriteTime | Format-List

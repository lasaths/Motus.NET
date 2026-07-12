param([string]$Rid = "win-x64")
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$buildDir = Join-Path $root "native/build-full-$Rid"
cmake -S (Join-Path $root "native") -B $buildDir -DMOTUS_USE_OMPL=ON -DMOTUS_USE_FCL=ON
cmake --build $buildDir --config Release
$dest = Join-Path $root "src/Motus.Native/runtimes/$Rid/native"
New-Item -ItemType Directory -Force -Path $dest | Out-Null
Copy-Item (Join-Path $buildDir "Release/motus_native.dll") $dest -ErrorAction SilentlyContinue
if (-not (Test-Path (Join-Path $dest "motus_native.dll"))) {
    Copy-Item (Join-Path $buildDir "motus_native.dll") $dest
}
Write-Host "Full native -> $dest"

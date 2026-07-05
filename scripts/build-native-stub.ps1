param([string]$Rid = "win-x64")
$ErrorActionPreference = "Stop"
$root = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$buildDir = Join-Path $root "native\build-stub-$Rid"
cmake -S (Join-Path $root "native") -B $buildDir -DMOTUS_USE_OMPL=OFF -DMOTUS_USE_FCL=OFF
cmake --build $buildDir --config Release
$dest = Join-Path $root "src\Motus.Native\runtimes\$Rid\native"
New-Item -ItemType Directory -Force -Path $dest | Out-Null
Copy-Item (Join-Path $buildDir "Release\motus_native.dll") (Join-Path $dest "motus_native.dll") -Force
Write-Host "Stub -> $dest"

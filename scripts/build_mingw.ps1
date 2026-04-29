param(
    [string]$Configuration = "Release"
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$out = Join-Path $root "build-local"
New-Item -ItemType Directory -Force $out | Out-Null

$sources = @(
    "src/barrowman.cpp",
    "src/csv.cpp",
    "src/dispersion.cpp",
    "src/io.cpp",
    "src/simulator.cpp"
)

function Invoke-Gxx {
    & g++ @args
    if ($LASTEXITCODE -ne 0) {
        throw "g++ failed with exit code $LASTEXITCODE"
    }
}

$coreSources = @($sources | ForEach-Object { Join-Path $root $_ })
Invoke-Gxx -std=c++20 -O2 "-I$root/include" @coreSources (Join-Path $root "src/main_cli.cpp") -o (Join-Path $out "hrocket_cli.exe")
Invoke-Gxx -std=c++20 -O2 "-I$root/include" @coreSources (Join-Path $root "tests/core_tests.cpp") -o (Join-Path $out "hrocket_tests.exe")
Invoke-Gxx -std=c++20 -O2 -mwindows -municode "-I$root/include" @coreSources (Join-Path $root "src/gui_win32.cpp") -lcomdlg32 -lgdi32 -luser32 -o (Join-Path $out "hrocket_gui.exe")

& (Join-Path $out "hrocket_tests.exe")
Write-Host "Built binaries in $out"

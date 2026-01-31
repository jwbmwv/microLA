# MicroLA Simple Benchmark Runner
# Requires: Visual Studio Build Tools or full Visual Studio installation

Write-Host "`n=== MicroLA Benchmark Setup ===`n" -ForegroundColor Cyan

# Check for Visual Studio installations
$vsPaths = @(
    "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat",
    "C:\Program Files\Microsoft Visual Studio\2022\Professional\VC\Auxiliary\Build\vcvars64.bat",
    "C:\Program Files\Microsoft Visual Studio\2022\Enterprise\VC\Auxiliary\Build\vcvars64.bat",
    "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Auxiliary\Build\vcvars64.bat",
    "C:\Program Files (x86)\Microsoft Visual Studio\2019\Professional\VC\Auxiliary\Build\vcvars64.bat"
)

$vcvarsPath = $null
foreach ($path in $vsPaths) {
    if (Test-Path $path) {
        $vcvarsPath = $path
        break
    }
}

if ($vcvarsPath) {
    Write-Host "Found Visual Studio at: $vcvarsPath" -ForegroundColor Green
    Write-Host "`nTo build and run benchmarks, use a Developer Command Prompt:" -ForegroundColor Yellow
    Write-Host "1. Open 'Developer Command Prompt for VS'" -ForegroundColor White
    Write-Host "2. Navigate to: cd C:\microla\benchmarks" -ForegroundColor White
    Write-Host "3. Compile: cl /EHsc /O2 /std:c++14 /I..\include simple_bench.cpp" -ForegroundColor White
    Write-Host "4. Run: .\simple_bench.exe`n" -ForegroundColor White
} else {
    Write-Host "Visual Studio not found!" -ForegroundColor Red
    Write-Host "`nPlease install one of the following:" -ForegroundColor Yellow
    Write-Host "- Visual Studio 2019 or 2022 (Community, Professional, or Enterprise)" -ForegroundColor White
    Write-Host "- Visual Studio Build Tools: https://visualstudio.microsoft.com/downloads/#build-tools-for-visual-studio-2022" -ForegroundColor White
    Write-Host "`nOr use MinGW-w64:" -ForegroundColor Yellow
    Write-Host "- Install: winget install mingw-w64" -ForegroundColor White
    Write-Host "- Then run: g++ -std=c++14 -O2 -I../include simple_bench.cpp -o simple_bench.exe`n" -ForegroundColor White
}

# Check for alternative compilers
Write-Host "Checking for alternative compilers..." -ForegroundColor Cyan

$hasGcc = Get-Command gcc -ErrorAction SilentlyContinue
$hasClang = Get-Command clang++ -ErrorAction SilentlyContinue
$hasMingw = Test-Path "C:\msys64\mingw64\bin\g++.exe"

if ($hasGcc) {
    Write-Host "[OK] GCC found: $(gcc --version | Select-Object -First 1)" -ForegroundColor Green
    Write-Host "Try: gcc -std=c++14 -O2 -I../include simple_bench.cpp -o simple_bench.exe" -ForegroundColor White
} elseif ($hasMingw) {
    Write-Host "[OK] MinGW found in MSYS2" -ForegroundColor Green
    Write-Host "Try: C:\msys64\mingw64\bin\g++.exe -std=c++14 -O2 -I../include simple_bench.cpp -o simple_bench.exe" -ForegroundColor White
} elseif ($hasClang) {
    Write-Host "[WARN] Clang found but requires Visual Studio libraries" -ForegroundColor Yellow
} else {
    Write-Host "[NO] No compatible C++ compiler found" -ForegroundColor Red
}

Write-Host "`n=== Quick Test (Header-only Check) ===`n" -ForegroundColor Cyan

# Create a minimal test to verify the headers
$testCode = @'
#define MICROLA_LINEAR_HEADER_ONLY
#include "../include/microla/microla.hpp"
int main() {
    microla::Vec3f v{1.0f, 2.0f, 3.0f};
    auto len = v.length();
    microla::SquareMat<float, 3> m = microla::SquareMat<float, 3>::identity();
    return 0;
}
'@

Set-Content -Path "test_compile.cpp" -Value $testCode
Write-Host "Created minimal test file: test_compile.cpp" -ForegroundColor Gray

Write-Host "`nBenchmark file ready at: $PWD\simple_bench.cpp" -ForegroundColor Green
Write-Host "Install a compiler to build and run the benchmarks.`n" -ForegroundColor Yellow

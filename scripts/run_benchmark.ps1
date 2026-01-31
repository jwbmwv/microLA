param(
    [string]$BuildDir = "build/benchmarks",
    [string]$OutDir = "build/benchmarks/results"
)

if (-not (Test-Path $OutDir)) { New-Item -ItemType Directory -Path $OutDir | Out-Null }

Start-Process -FilePath (Join-Path $BuildDir "benchmarks.exe") -ArgumentList @('--benchmark_format=json', "--benchmark_out=$OutDir\results.json") -Wait

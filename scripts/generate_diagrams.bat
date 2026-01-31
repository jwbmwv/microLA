@echo off
setlocal enabledelayedexpansion

set "SCRIPT_DIR=%~dp0"
set "REPO_ROOT=%SCRIPT_DIR%.."

where plantuml >nul 2>&1
if errorlevel 1 (
  echo Error: plantuml not found in PATH. Install PlantUML or adjust PATH.
  exit /b 1
)

for %%D in ("%REPO_ROOT%\design" "%REPO_ROOT%\examples") do (
  if exist "%%~fD" (
    echo Rendering diagrams in %%~fD
    for %%F in ("%%~fD\*.puml") do (
      echo   -> %%~fF
      plantuml -tpng "%%~fF"
      plantuml -tsvg "%%~fF"
      plantuml -tpdf "%%~fF"
      for %%P in ("%%~dpnF.pdf") do (
        if not exist "%%~fP" (
          echo Error: PDF output missing for %%~fF
          exit /b 1
        )
        if %%~zP EQU 0 (
          echo Error: PDF output is empty for %%~fF
          del /q "%%~fP" >nul 2>&1
          exit /b 1
        )
      )
    )
  )
)

echo Diagram rendering complete (png, svg, pdf).

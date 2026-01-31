@echo off
REM Generate PlantUML diagrams for MicroLA
REM Requires: PlantUML 1.2025.10 and Batik 1.19

setlocal

set PLANTUML_JAR=C:\Users\think\bin\plantuml.jar
set BATIK_PATH=C:\Users\think\bin\batik-1.19

echo Generating PlantUML diagrams...

REM Generate PNG diagrams with Batik for better rendering
java -jar "%PLANTUML_JAR%" -tpng -DPLANTUML_BATIK="%BATIK_PATH%" design\*.puml

REM Generate SVG diagrams (vector graphics)
java -jar "%PLANTUML_JAR%" -tsvg design\*.puml

echo.
echo Diagrams generated in design\ directory:
echo - PNG files (for documentation)
echo - SVG files (for scaling/print)
echo.
echo Done!

endlocal

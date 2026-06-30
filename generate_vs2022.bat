@echo off
REM ---------------------------------------------------------------------------
REM gltf2md5 -- Visual Studio 2022 solution generator
REM
REM Creates build\gltf2md5.sln using the project's CMakeLists.txt.
REM Run from the project root (the folder containing CMakeLists.txt and src\).
REM
REM Requirements:
REM   - CMake 3.15 or newer on PATH  (winget install Kitware.CMake)
REM   - Visual Studio 2022 with the "Desktop development with C++" workload
REM ---------------------------------------------------------------------------

setlocal

REM Sanity check: make sure we're at the project root
if not exist CMakeLists.txt (
    echo ERROR: CMakeLists.txt not found in current directory.
    echo Run this script from the project root.
    pause
    exit /b 1
)

REM Make sure cmake is reachable
where cmake >nul 2>nul
if errorlevel 1 (
    echo ERROR: cmake not found on PATH.
    echo Install CMake 3.15+ and add it to PATH, then re-run.
    pause
    exit /b 1
)

REM Create build directory
if not exist build mkdir build

echo.
echo Generating Visual Studio 2022 x64 solution in .\build ...
echo.

cmake -S . -B build -G "Visual Studio 17 2022" -A x64

if errorlevel 1 (
    echo.
    echo ERROR: CMake generation failed.
    pause
    exit /b 1
)

echo.
echo ============================================================
echo   Solution generated: build\gltf2md5.sln
echo   Binary will land at: bin\Release\gltf2md5.exe
echo ============================================================
echo.

REM Auto-open the solution in Visual Studio. Comment out if not wanted.
start "" "build\gltf2md5.sln"

endlocal

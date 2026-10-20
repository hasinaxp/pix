@echo off
rem pix build script - works from a plain cmd / PowerShell window.
rem
rem   build                 build every example in examples\
rem   build hello_3d        build one example (examples\hello_3d.cpp)
rem   build hello_3d run    build it and run it
rem   build single          regenerate dist\pix.h and test-compile the examples against it
rem   build test            build and run tests\build_check.cpp
rem   build debug <name>    debug build (no optimisation, debug info)
rem
rem Your own game needs nothing but:   cl /std:c++14 /O2 /I src game.cpp
rem (or /I dist with the single header) - pix links its own system libraries.
setlocal enabledelayedexpansion
cd /d "%~dp0"

rem ---- find a compiler: cl on PATH, else locate Visual Studio with vswhere ----
where cl >nul 2>nul
if errorlevel 1 (
    set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
    if not exist "!VSWHERE!" (
        echo pix: MSVC not found. Install Visual Studio or the Build Tools with the C++ workload.
        exit /b 1
    )
    for /f "usebackq tokens=*" %%i in (`"!VSWHERE!" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VSDIR=%%i"
    if not defined VSDIR (
        echo pix: Visual Studio found, but without the C++ tools.
        exit /b 1
    )
    call "!VSDIR!\VC\Auxiliary\Build\vcvars64.bat" >nul
)

if not exist build mkdir build
set "FLAGS=/nologo /std:c++14 /EHsc /O2 /W3 /I src"
set "RUN="

if /i "%~1"=="debug" (
    set "FLAGS=/nologo /std:c++14 /EHsc /Od /Zi /W3 /I src"
    shift
)
if /i "%~1"=="single" (
    python compile.py --test
    exit /b !errorlevel!
)
if /i "%~1"=="test" (
    rem unoptimised on purpose: an /O2 build of this file trips a Windows Defender
    rem heuristic on some machines, and the checks do not need the speed
    cl /nologo /std:c++14 /EHsc /Od /W3 /I src /Fe:build\build_check.exe /Fo:build\ tests\build_check.cpp || exit /b 1
    build\build_check.exe
    exit /b !errorlevel!
)
if /i "%~2"=="run" set "RUN=1"

if "%~1"=="" (
    for %%f in (examples\*.cpp) do (
        echo building %%~nf
        cl %FLAGS% /Fe:build\%%~nf.exe /Fo:build\ "%%f" || exit /b 1
    )
    exit /b 0
)

if not exist "examples\%~1.cpp" (
    echo pix: no examples\%~1.cpp
    exit /b 1
)
cl %FLAGS% /Fe:build\%~1.exe /Fo:build\ "examples\%~1.cpp" || exit /b 1
if defined RUN build\%~1.exe
exit /b 0

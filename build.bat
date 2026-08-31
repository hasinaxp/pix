@echo off
rem Build with MSVC. Run from a "x64 Native Tools Command Prompt for VS".
if not exist build mkdir build
cl /nologo /std:c++14 /EHsc /O2 /W3 /Fe:build\pix.exe /Fo:build\ src\main.cpp

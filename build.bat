@echo off
echo ===================================================
echo [1/3] Setting up environment variables for Qt6/MinGW...
echo ===================================================
set "PATH=D:\QT\Tools\Ninja;D:\QT\6.11.1\mingw_64\bin;D:\QT\Tools\mingw1310_64\bin;%PATH%"

echo ===================================================
echo [2/3] Configuring project using CMake and Ninja...
echo ===================================================
if exist build rmdir /s /q build
mkdir build
cd build
D:\QT\Tools\CMake_64\bin\cmake.exe -S .. -B . -G Ninja -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++ -DCMAKE_CXX_FLAGS="-O3 -march=native"

if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] CMake configuration failed.
    exit /b %ERRORLEVEL%
)

echo ===================================================
echo [3/3] Compiling optimized binaries...
echo ===================================================
D:\QT\Tools\CMake_64\bin\cmake.exe --build .

if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] Compilation failed.
    exit /b %ERRORLEVEL%
)

echo ===================================================
echo [SUCCESS] Compilation completed successfully!
echo Binary located at: d:\c++big_homework\build\QtTest.exe
echo ===================================================

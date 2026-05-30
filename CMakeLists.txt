cmake_minimum_required(VERSION 3.5)
project(QtTest VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 启用 Qt MOC (元对象编译器) 以支持信号和槽槽函数
set(CMAKE_AUTOMOC ON)

# 告诉 CMake 你的 Qt 安装路径
set(CMAKE_PREFIX_PATH "D:/QT/6.11.1/mingw_64" CACHE PATH "")

find_package(Qt6 COMPONENTS Widgets OpenGLWidgets REQUIRED)

# 源代码列表
set(SOURCES
    main.cpp
    src/core/SpatialHash.cpp
    src/core/World.cpp
    src/core/fluid/SPHSolver.cpp
    src/core/rigidbody/Collider.cpp
    src/core/rigidbody/Solver.cpp
    src/gui/RenderCanvas.cpp
    src/gui/MainWindow.cpp
)

add_executable(QtTest ${SOURCES})
target_link_libraries(QtTest PRIVATE Qt6::Widgets Qt6::OpenGLWidgets)
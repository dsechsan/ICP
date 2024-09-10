cmake_minimum_required(VERSION 3.24)
project(pointclouds)

set(CMAKE_CXX_STANDARD 17)
set(Open3D_DIR "${HOME}/open3d_install/lib/cmake/Open3D/")
find_package(Open3D REQUIRED HINTS "${Open3D_DIR}")

add_executable(pointclouds
        src/ICPAlgorithm.cpp
        src/main.cpp
        src/ICPVisualization.cpp
        src/utils.cpp
)

target_link_libraries(pointclouds PRIVATE Open3D::Open3D)
target_include_directories(pointclouds PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(/usr/local/include)
include_directories(${Open3D_INCLUDE_DIRS})
include_directories(/opt/homebrew/include)


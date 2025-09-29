cmake_minimum_required(VERSION 3.10)
project(trajectory_optimizer)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set build type if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Compiler flags for MinGW numerical stability
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# Windows/MinGW specific flags for numerical consistency
if(WIN32 AND CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffloat-store -fexcess-precision=standard")
endif()

# Find required packages
# 直接设置Eigen3路径
set(EIGEN3_INCLUDE_DIR "D:/CPP_library/eigen-3.3.8")

# 验证Eigen3安装
if(NOT EXISTS "${EIGEN3_INCLUDE_DIR}/Eigen/Core")
    message(FATAL_ERROR "Eigen3 not found at ${EIGEN3_INCLUDE_DIR}. Please verify the installation path.")
endif()

# 创建Eigen3目标
add_library(Eigen3::Eigen INTERFACE IMPORTED)
set_target_properties(Eigen3::Eigen PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIR}"
)

# Include directories
include_directories(
    include
    ${EIGEN3_INCLUDE_DIR}
)

# Create library for trajectory optimization
add_library(traj_opt_lib
    src/traj_opt_perching.cc
)

# Set target properties
target_include_directories(traj_opt_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIR}
)

# 为数值一致性链接数学库
if(WIN32)
    target_link_libraries(traj_opt_lib PRIVATE m)
endif()

# Create example executables
add_executable(traj_opt_example examples/basic_example.cpp)
target_link_libraries(traj_opt_example traj_opt_lib)

add_executable(precision_test examples/precision_test.cpp)
target_link_libraries(precision_test traj_opt_lib)
cmake_minimum_required(VERSION 3.14)
project(policy_depoly)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# 通过 CMake 传入根目录
add_definitions(-DPROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

# 同时设置 libtorch 和 raisim 的路径
set(CMAKE_PREFIX_PATH "$ENV{LIBTORCH_DIR};$ENV{RAISIM_DIR}")

# 查找依赖库
find_package(Torch REQUIRED)

# 查找 yaml-cpp
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp)

find_package(Eigen3 REQUIRED)

find_package(raisim CONFIG REQUIRED)
message(STATUS "Found Raisim. " ${RAISIM_DIR})

# 递归收集所有 .cpp 文件
file(GLOB_RECURSE SRC_FILES
    ${CMAKE_SOURCE_DIR}/src/*.cpp
)

# 头文件路径（你项目头文件 + 第三方头文件）
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${YAMLCPP_INCLUDE_DIRS} 
    ${EIGEN3_INCLUDE_DIRS}
    ${RAISIM_DIR}
)

# 设置库目录
link_directories(
    ${YAMLCPP_LIBRARY_DIRS}
)

# 添加可执行文件
add_executable(deploy_mujoco
    exec/deploy_mujoco.cpp
    ${SRC_FILES}
)

# 链接所需库
target_link_libraries(deploy_mujoco
    ${TORCH_LIBRARIES}
    yaml-cpp
    raisim::raisim
)


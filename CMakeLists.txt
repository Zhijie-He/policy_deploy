cmake_minimum_required(VERSION 3.14)
project(policy_depoly)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-DPROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

# # 启用 AddressSanitizer 调试（可选）
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -g -O1")
# set(CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} -fsanitize=address")

# 同时设置 libtorch 和 raisim 的路径
set(CMAKE_PREFIX_PATH "$ENV{LIBTORCH_DIR};$ENV{RAISIM_DIR}")
set(MUJOCO_DIR $ENV{MUJOCO_DIR})

# 查找依赖库
find_package(Torch REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp)
find_package(Eigen3 REQUIRED)
pkg_check_modules(GLFW REQUIRED glfw3) # GLFW
find_package(raisim CONFIG REQUIRED)
message(STATUS "Found Raisim. " ${RAISIM_DIR})

# 设置 Unitree SDK 根目录
set(UNITREE_SDK2_DIR $ENV{UNITREE_SDK2_DIR})

# 递归收集所有 .cpp 文件
file(GLOB_RECURSE SRC_FILES
    ${CMAKE_SOURCE_DIR}/src/*.cpp
)

include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${YAMLCPP_INCLUDE_DIRS} 
    ${EIGEN3_INCLUDE_DIRS}
    ${RAISIM_DIR}
    ${MUJOCO_DIR}/include
    ${GLFW_INCLUDE_DIRS}

    ${UNITREE_SDK2_DIR}/include
    ${UNITREE_SDK2_DIR}/thirdparty/include
    ${UNITREE_SDK2_DIR}/thirdparty/include/ddscxx
    ${CMAKE_CURRENT_SOURCE_DIR}/exec
)

# 设置库目录
link_directories(
    ${YAMLCPP_LIBRARY_DIRS}
    ${GLFW_LIBRARY_DIRS}
    ${MUJOCO_DIR}/lib 

    ${UNITREE_SDK2_DIR}/lib/x86_64
    ${UNITREE_SDK2_DIR}/thirdparty/lib/x86_64
)

set(COMMON_LIBS
    ${TORCH_LIBRARIES}
    yaml-cpp
    raisim::raisim
    ${GLFW_LIBRARIES}
    mujoco

    unitree_sdk2
    ddscxx
    ddsc
)

# 添加 Raisim 可执行程序
# add_executable(deploy_raisim exec/deploy_raisim.cpp ${SRC_FILES})
# target_link_libraries(deploy_raisim ${COMMON_LIBS})

# MuJoCo 版本执行文件
add_executable(deploy_mujoco exec/deploy_mujoco.cpp ${SRC_FILES})
target_link_libraries(deploy_mujoco ${COMMON_LIBS})

# G1真机部署
add_executable(deploy_real exec/deploy_real.cpp ${SRC_FILES})
target_link_libraries(deploy_real ${COMMON_LIBS})
cmake_minimum_required(VERSION 3.14)
project(policy_depoly)
## Set compiler to use c++ 17 features
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-DPROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

# only use unitree sdk in linux system
if (${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
    set(USE_UNITREE_SDK_DEFAULT ON)
else()
    set(USE_UNITREE_SDK_DEFAULT OFF)
endif()

option(USE_UNITREE_SDK "Enable Unitree SDK support (only for Linux)" ${USE_UNITREE_SDK_DEFAULT})

# 启用 AddressSanitizer 调试（可选）
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -g -O1")
# set(CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} -fsanitize=address")

set(CMAKE_PREFIX_PATH "$ENV{LIBTORCH_DIR}") # 设置 libtorch path 
set(MUJOCO_DIR $ENV{MUJOCO_DIR})  # Set Mujoco path
set(UNITREE_SDK2_DIR ${CMAKE_CURRENT_SOURCE_DIR}/library/unitree_sdk2) # set Unitree SDK path

# 查找依赖库
find_package(Torch REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp)
pkg_check_modules(GLFW REQUIRED glfw3) # GLFW

message(STATUS "System: ${CMAKE_SYSTEM_NAME}")
message(STATUS "USE_UNITREE_SDK: ${USE_UNITREE_SDK}")

# 递归收集所有 .cpp 文件
file(GLOB_RECURSE ALL_SRC_FILES ${CMAKE_SOURCE_DIR}/src/*.cpp)

# 排除 sim2/real 文件夹下的文件
if (NOT USE_UNITREE_SDK)
    list(FILTER ALL_SRC_FILES EXCLUDE REGEX ".*/real/.*")
endif()

set(SRC_FILES ${ALL_SRC_FILES})

# 包含头文件
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${YAMLCPP_INCLUDE_DIRS}
    ${MUJOCO_DIR}/include
    ${GLFW_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/exec
    ${CMAKE_CURRENT_SOURCE_DIR}/library
)

# 链接库目录
link_directories(
    ${YAMLCPP_LIBRARY_DIRS}
    ${GLFW_LIBRARY_DIRS}
    ${MUJOCO_DIR}/lib
)

# 公共库列表
set(COMMON_LIBS
    ${TORCH_LIBRARIES}
    yaml-cpp
    ${GLFW_LIBRARIES}
    mujoco
    pthread
)

# 如果启用 Unitree SDK
if (USE_UNITREE_SDK)
    include_directories(
        ${UNITREE_SDK2_DIR}/include
        ${UNITREE_SDK2_DIR}/thirdparty/include
        ${UNITREE_SDK2_DIR}/thirdparty/include/ddscxx
    )
    link_directories(
        ${UNITREE_SDK2_DIR}/lib/x86_64
        ${UNITREE_SDK2_DIR}/thirdparty/lib/x86_64
    )
    list(APPEND COMMON_LIBS
        unitree_sdk2
        ddscxx
        ddsc
    )
endif()

# ===== 添加可执行文件 =====

# MuJoCo 版本执行文件 For non-linux system
if (NOT USE_UNITREE_SDK)
    add_executable(deploy_mujoco exec/deploy_mujoco.cpp ${SRC_FILES})
    target_link_libraries(deploy_mujoco ${COMMON_LIBS})
endif()

# 真机部署（仅在启用Unitree SDK时编译）
if (USE_UNITREE_SDK)
    add_executable(g1 exec/g1.cpp ${SRC_FILES})
    target_link_libraries(g1 ${COMMON_LIBS})

    add_executable(simulate_robot exec/simulate_robot.cpp ${SRC_FILES})
    target_link_libraries(simulate_robot ${COMMON_LIBS})

    add_executable(check_G1Robot_input exec/check_G1Robot_input.cpp ${SRC_FILES})
    target_link_libraries(check_G1Robot_input ${COMMON_LIBS})
endif()

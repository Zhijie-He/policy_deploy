cmake_minimum_required(VERSION 3.14)
project(policy_depoly)

# === 编译器设置 ===
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wno-deprecated-declarations)

add_definitions(-DPROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

# === 彩色日志函数 ===
option(ENABLE_COLOR_LOG "Enable colorized message output" ON)

function(color_message TYPE COLOR TEXT)
    if(NOT ENABLE_COLOR_LOG)
        message(${TYPE} "${TEXT}")
        return()
    endif()

    string(ASCII 27 Esc)
    set(RESET "${Esc}[0m")

    if(COLOR STREQUAL "red")
        set(COLOR_CODE "${Esc}[1;31m")
    elseif(COLOR STREQUAL "green")
        set(COLOR_CODE "${Esc}[1;32m")
    elseif(COLOR STREQUAL "yellow")
        set(COLOR_CODE "${Esc}[1;33m")
    elseif(COLOR STREQUAL "blue")
        set(COLOR_CODE "${Esc}[1;34m")
    elseif(COLOR STREQUAL "cyan")
        set(COLOR_CODE "${Esc}[1;36m")
    else()
        set(COLOR_CODE "")
    endif()

    message(${TYPE} "${COLOR_CODE}${TEXT}${RESET}")
endfunction()

# ========== 平台识别（仅限 macOS / Linux） ==========
if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
    set(CURRENT_PLATFORM "macos")
elseif(CMAKE_SYSTEM_NAME MATCHES "Linux")
    if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
        set(CURRENT_PLATFORM "linux-x86_64")
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
        set(CURRENT_PLATFORM "linux-aarch64")
    else()
        color_message(FATAL_ERROR red "[ERROR] Unsupported Linux architecture: ${CMAKE_SYSTEM_PROCESSOR}")
    endif()
else()
    color_message(FATAL_ERROR red "[ERROR] Unsupported platform: ${CMAKE_SYSTEM_NAME}")
endif()

color_message(STATUS cyan "[INFO] System: ${CMAKE_SYSTEM_NAME}")
color_message(STATUS cyan "[INFO] Detected platform: ${CURRENT_PLATFORM}")

# === AddressSanitizer 选项（默认关闭）===
option(ENABLE_ASAN "Enable AddressSanitizer" OFF)
if (ENABLE_ASAN)
    color_message(STATUS cyan "[INFO] AddressSanitizer enabled")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -g -O1")
    set(CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} -fsanitize=address")
else()
    color_message(STATUS cyan "[INFO] AddressSanitizer disabled (enable with -DENABLE_ASAN=ON)")
endif()

set(CMAKE_PREFIX_PATH $ENV{LIBTORCH_DIR} ${CMAKE_PREFIX_PATH})      # Add libtorch path
set(MUJOCO_DIR ${CMAKE_SOURCE_DIR}/library/mujoco)                  # Set Mujoco path
set(TENSORRT_DIR $ENV{TENSORRT_DIR})                                # Set TensorRT path
set(CUDA_DIR $ENV{CUDA_DIR})                                        # Set CUDA path
set(FASTDDS_DIR ${CMAKE_SOURCE_DIR}/library/fastdds)                # Set FastDDS path
set(WLROBOT_DIR ${CMAKE_SOURCE_DIR}/library/wlrobot_sdk)            # Set wlrobot_sdk path

# 根据 TENSORRT_DIR 是否存在自动判断是否使用 TensorRT
if(DEFINED TENSORRT_DIR AND EXISTS "${TENSORRT_DIR}/lib")
    set(USE_TENSORRT ON)
    add_definitions(-DUSE_TENSORRT)
    color_message(STATUS cyan "[INFO] USE_TENSORRT: ${USE_TENSORRT} (${TENSORRT_DIR})")
else()
    set(USE_TENSORRT OFF)
    color_message(STATUS cyan "[INFO] USE_TENSORRT: ${USE_TENSORRT} (TENSORRT_DIR not set or invalid ${TENSORRT_DIR})")
endif()

# 查找依赖库
find_package(Torch REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(SDL2 REQUIRED)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp)
pkg_check_modules(GLFW REQUIRED glfw3)

# 递归收集所有 .cpp 文件
file(GLOB_RECURSE ALL_SRC_FILES ${CMAKE_SOURCE_DIR}/src/*.cpp)

if (NOT USE_TENSORRT)
    list(FILTER ALL_SRC_FILES EXCLUDE REGEX ".*/tensorrt/.*")
endif()

set(SRC_FILES ${ALL_SRC_FILES})

# 包含头文件
include_directories(
    ${MUJOCO_DIR}/include
    ${YAMLCPP_INCLUDE_DIRS}
    ${GLFW_INCLUDE_DIRS}
    ${SDL2_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/exec
    ${CMAKE_SOURCE_DIR}/library
    ${FASTDDS_DIR}/include
    ${WLROBOT_DIR}/include
)

# 链接库目录
link_directories(
    ${YAMLCPP_LIBRARY_DIRS}
    ${GLFW_LIBRARY_DIRS}
    ${MUJOCO_DIR}/lib/${CURRENT_PLATFORM}
    ${FASTDDS_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR}
)

# 公共库列表
set(COMMON_LIBS
    ${TORCH_LIBRARIES}
    yaml-cpp
    ${GLFW_LIBRARIES}
    mujoco
    pthread
    ${SDL2_LIBRARIES}
)

# 如果启动TensorRT
if (USE_TENSORRT)
    include_directories(
        ${TENSORRT_DIR}/include
        ${CUDA_DIR}/include
    )
    link_directories(
        ${TENSORRT_DIR}/lib
        ${CUDA_DIR}/lib64
    )
    list(APPEND COMMON_LIBS
        nvinfer
        nvonnxparser
        nvparsers
        nvinfer_plugin
        cudart
    )
endif()

# # ===== 添加可执行文件 =====

# 递归收集所有 .cpp 文件
file(GLOB_RECURSE WLROBOT_SDK_SRC_FILES ${WLROBOT_DIR}/src/*.cpp)

add_executable(run exec/run.cpp ${SRC_FILES} ${WLROBOT_SDK_SRC_FILES})
target_link_libraries(run fastdds fastcdr ${COMMON_LIBS})

# sync with real robot
add_executable(robot_sim_sync exec/robot_sim_sync.cpp ${SRC_FILES} ${WLROBOT_SDK_SRC_FILES})
target_link_libraries(robot_sim_sync fastdds fastcdr ${COMMON_LIBS})

add_executable(simulate_robot exec/simulate_robot.cpp ${SRC_FILES} ${WLROBOT_SDK_SRC_FILES})
target_link_libraries(simulate_robot fastdds fastcdr ${COMMON_LIBS})

add_executable(start_low_state exec/start_low_state.cpp ${SRC_FILES} ${WLROBOT_SDK_SRC_FILES})
target_link_libraries(start_low_state fastdds fastcdr ${COMMON_LIBS})

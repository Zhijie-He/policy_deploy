cmake_minimum_required(VERSION 3.10)
project(robot_message)

# 如果用的是 GCC 或 Clang 编译器，开启警告选项，提高代码质量。
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED) # ROS 2 构建系统（ament）基础包，所有 ROS 包都需要。
find_package(rosidl_default_generators REQUIRED) # 提供 rosidl_generate_interfaces() 宏，用于生成 .msg 的 C++/Python 接口代码。

set(msg_files
  "msg/MotorState.msg"
  "msg/MotorStateData.msg"
  "msg/MotorCmd.msg"
  "msg/MotorControl.msg"
  "msg/JointControlGroup.msg"
  "msg/JointStateGroup.msg"
  "msg/ImuData.msg"
)

# 自动生成消息代码
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

# 导出依赖（让其他包可以使用） 把生成后的接口暴露给其他依赖你的包（如 import robot_message.msg）。
# ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#  声明是 ROS 包
ament_package()


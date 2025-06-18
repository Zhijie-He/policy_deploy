cmake_minimum_required(VERSION 3.10)
project(neural_policy_deploy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED) # 标准消息库
find_package(sensor_msgs REQUIRED)
find_package(robot_message REQUIRED)

# 2. 封装通用工具库
file(GLOB_RECURSE UTILITY_SRC_LIST "src/utility/*.cpp")
add_library(utility SHARED ${UTILITY_SRC_LIST}) # 把你写的 src/utility/ 目录下的所有 .cpp 文件打包成一个共享库（动态链接库），名字叫 utility
# SHARED 表示是一个 动态链接库（.so / .dylib）。 ${UTILITY_SRC_LIST} 是你前面通过 file(GLOB_RECURSE ...) 收集到的所有 .cpp 文件。
# 添加头文件搜索路径
target_include_directories(utility PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utility
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 3. 可执行文件
add_executable(joint_monkey exec/joint_monkey.cpp src/ros_node/RobotInterface.cpp src/state_machine/StateMachine.cpp)
ament_target_dependencies(joint_monkey rclcpp std_msgs sensor_msgs robot_message)
target_include_directories(joint_monkey PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(joint_monkey 
          utility 
          yaml-cpp
          Eigen3::Eigen
)

# 4. 安装可执行文件
install(TARGETS joint_monkey
  DESTINATION lib/${PROJECT_NAME}
)

# 5. 测试支持
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_package()


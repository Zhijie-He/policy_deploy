cmake_minimum_required(VERSION 3.8)
project(robot_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_message REQUIRED)

# 添加头文件目录
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include/robot_manager
)

# 添加可执行文件 motor_manager，指定源文件路径
file(GLOB MOTOR_MANAGER_SRCS "src/motor_manager_node/*.cpp")
add_executable(motor_manager 
			${MOTOR_MANAGER_SRCS})
ament_target_dependencies(motor_manager 
	rclcpp
	std_msgs  # 连接 std_msgs 库
	robot_message	#消息库
)
target_link_libraries(motor_manager "${cpp_typesupport_target}" ${LIBXML2_LIBRARIES})


# 添加可执行文件 joint_dispose，指定源文件路径
file(GLOB JOINT_DISPOSE_SRCS "src/joint_dispose_node/*.cpp")
add_executable(joint_dispose
      ${JOINT_DISPOSE_SRCS})
ament_target_dependencies(joint_dispose 
	rclcpp
	std_msgs  # 连接 std_msgs 库
	robot_message	#消息库
)
target_link_libraries(joint_dispose "${cpp_typesupport_target}")


# 添加可执行文件 imu_manager，指定源文件路径
file(GLOB IMU_MANAGER_SRCS "src/imu_manager_node/*.cpp")
add_executable(imu_manager
      ${IMU_MANAGER_SRCS})
ament_target_dependencies(imu_manager 
	rclcpp
	std_msgs  # 连接 std_msgs 库
	robot_message	#消息库
)
target_link_libraries(imu_manager "${cpp_typesupport_target}" ${LIBXML2_LIBRARIES})

# 安装可执行文件到指定路径，确保可以通过 ros2 run 运行
install(TARGETS
  motor_manager                    # motor_manager节点执行文件
  joint_dispose                    # joint_dispose节点执行文件
  imu_manager       			         # imu_manager节点执行文件

  DESTINATION lib/${PROJECT_NAME} # 安装路径：lib/robot_manager
)

# 安装 XML 配置文件到指定目录
install(DIRECTORY config/
        DESTINATION share/${PROJECT_NAME}/config)

# 安装 setup_ros2_distributed.sh 脚本
install(
  PROGRAMS scripts/setup_ros2_distributed.sh
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件到指定路径
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch  # 安装路径：share/robot_manager/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

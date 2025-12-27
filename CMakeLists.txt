cmake_minimum_required(VERSION 3.8)
project(teb_controller_plugin)

# 设置 C++ 标准 # 编译器警告选项（针对 GCC/Clang）
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找 ROS 2 依赖包
find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED) # 处理消息与 tf2 类型转换的转换器
find_package(pluginlib REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

# 添加自定义的 CMake 模块路径  # 添加自定义查找模块路径
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/teb_algorithm/cmake")

# 查找 G2O（使用自带的 FindG2O.cmake）
find_package(G2O REQUIRED)

# 如果上面找不到，手动设置路径
if(NOT G2O_FOUND)
  message(STATUS "Using manual G2O paths")
  set(G2O_INCLUDE_DIRS "/usr/local/include")
  set(G2O_LIBRARIES 
    g2o_core 
    g2o_stuff 
    g2o_types_slam2d
    g2o_solver_eigen
  )
endif()

message(STATUS "G2O_INCLUDE_DIRS: ${G2O_INCLUDE_DIRS}")
message(STATUS "G2O_LIBRARIES: ${G2O_LIBRARIES}")

# ==================================
# 编译 TEB 算法库
# ==================================

set(TEB_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/teb_algorithm/src/planner_manager.cpp
)
# 创建名为 teb_algorithm 的静态库
add_library(teb_algorithm STATIC ${TEB_SOURCES})
set_target_properties(teb_algorithm PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(teb_algorithm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/teb_algorithm/include>
  $<INSTALL_INTERFACE:include/teb_algorithm>
  ${G2O_INCLUDE_DIRS}
)
# 链接 G2O 和 Eigen3
target_link_libraries(teb_algorithm 
  PUBLIC
    ${G2O_LIBRARIES}
    Eigen3::Eigen
)

target_compile_features(teb_algorithm PUBLIC cxx_std_17)

# ==================================
# 编译控制器插件  创建共享库（.so 文件），作为 ROS 2 插件
# ==================================

add_library(${PROJECT_NAME} SHARED
  src/teb_controller.cpp
)
# 创建别名（方便引用）
add_library(teb_controller_plugin::teb_controller_plugin ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  teb_algorithm   # 链接上面的静态库
  Boost::thread 

)
# ROS 2 特有的依赖声明方式
ament_target_dependencies(${PROJECT_NAME}
  nav2_common
  nav2_core
  nav2_costmap_2d
  nav2_util
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  nav_msgs
  tf2_geometry_msgs 
  tf2_ros
  tf2
  pluginlib
)

# ==================================
# 安装配置
# ==================================

install(TARGETS ${PROJECT_NAME} teb_algorithm
  EXPORT ${PROJECT_NAME}_targets  # 导出目标，供其他项目使用
  ARCHIVE DESTINATION lib         # 静态库 (.a)
  LIBRARY DESTINATION lib         # 共享库 (.so)
  RUNTIME DESTINATION bin         # 可执行文件
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY teb_algorithm/include/
  DESTINATION include/teb_algorithm
)

pluginlib_export_plugin_description_file(nav2_core teb_controller_plugin.xml)

# ==================================
# 导出配置  让其他 ROS 2 包能够 find_package(teb_controller_plugin) 找到这个包
# ==================================

ament_export_targets(${PROJECT_NAME}_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  nav2_common
  nav2_core
  nav2_costmap_2d
  nav2_util
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  nav_msgs
  tf2_geometry_msgs 
  tf2_ros
  tf2
  pluginlib
  Eigen3
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
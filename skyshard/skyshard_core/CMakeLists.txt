cmake_minimum_required(VERSION 3.8)
project(skyshard_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(skyshard_msgs REQUIRED)

include_directories(include)

# Router Node
add_executable(router_node src/router_node.cpp)
ament_target_dependencies(router_node
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  skyshard_msgs
)
install(TARGETS router_node DESTINATION lib/${PROJECT_NAME})

# Scenario Manager Node
add_executable(scenario_manager src/scenario_manager.cpp)
ament_target_dependencies(scenario_manager
  rclcpp
  skyshard_msgs
)
install(TARGETS scenario_manager DESTINATION lib/${PROJECT_NAME})

# Başlık dosyalarının kurulumu
install(DIRECTORY include/
  DESTINATION include
)

ament_package()

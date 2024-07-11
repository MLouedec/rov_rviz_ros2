cmake_minimum_required(VERSION 3.5)
project(rov_rviz_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(message2 REQUIRED)
find_package(geometry_msgs REQUIRED)

if(jsoncpp_FOUND)
  message("jsoncpp found")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(json_data_reader src/json_data_reader.cpp)
add_executable(bag2csv src/bag2csv.cpp)
add_executable(replay src/replay.cpp)

ament_target_dependencies(json_data_reader rclcpp std_msgs tf2 tf2_ros visualization_msgs)
ament_target_dependencies(bag2csv rclcpp std_msgs tf2 nav_msgs mavros_msgs)
ament_target_dependencies(replay rclcpp std_msgs tf2 tf2_ros nav_msgs mavros_msgs visualization_msgs message2 geometry_msgs)

target_link_libraries(json_data_reader ${cpp_typesupport_target} jsoncpp_lib)
target_link_libraries(bag2csv ${cpp_typesupport_target} Eigen3::Eigen)
target_link_libraries(replay ${cpp_typesupport_target} Eigen3::Eigen)

target_include_directories(json_data_reader  PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_include_directories(bag2csv  PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_include_directories(replay PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

ament_package()

install(TARGETS json_data_reader bag2csv replay
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config meshs
        DESTINATION share/${PROJECT_NAME}/)



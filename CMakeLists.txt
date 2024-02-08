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
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)

#get_target_property(JSON_INC_PATH jsoncpp INTERFACE_INCLUDE_DIRECTORIES)
#include_directories(${JSON_INC_PATH})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(json_data_reader src/json_data_reader.cpp)
ament_target_dependencies(json_data_reader rclcpp std_msgs tf2 tf2_ros visualization_msgs)

target_link_libraries(json_data_reader "${cpp_typesupport_target}" ${JSONCPP_LIBRARIES})

target_include_directories(data_reader PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

ament_package()

install(TARGETS json_data_reader
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config meshs
        DESTINATION share/${PROJECT_NAME}/)



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

if(jsoncpp_FOUND)
  message("jsoncpp found")
  #~ include_directories("${JSONCPP_INCLUDE_DIRS}")
endif()

#~ find_package(PkgConfig REQUIRED)
#~ pkg_check_modules(JSONCPP jsoncpp)

#~ get_target_property(JSON_INC_PATH jsoncpp INTERFACE_INCLUDE_DIRECTORIES)
#~ include_directories(${JSON_INC_PATH})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(json_data_reader src/json_data_reader.cpp)
add_executable(bag2csv src/bag2csv.cpp)

ament_target_dependencies(json_data_reader rclcpp std_msgs tf2 tf2_ros visualization_msgs)# jsoncpp)
ament_target_dependencies(bag2csv rclcpp std_msgs tf2 nav_msgs mavros_msgs)

target_link_libraries(json_data_reader ${cpp_typesupport_target} jsoncpp_lib)# ${JSONCPP_LIBRARIES})
#~ target_link_libraries(json_data_reader "${cpp_typesupport_target}" "${JSONCPP_LINK_LIBRARIES}")
target_link_libraries(bag2csv ${cpp_typesupport_target} Eigen3::Eigen)

target_include_directories(json_data_reader  PUBLIC
        #${JSONCPP_INCLUDE_DIR}
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_include_directories(bag2csv  PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

ament_package()

install(TARGETS json_data_reader bag2csv
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config meshs
        DESTINATION share/${PROJECT_NAME}/)



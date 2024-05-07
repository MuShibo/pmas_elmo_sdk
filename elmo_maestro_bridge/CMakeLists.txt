cmake_minimum_required(VERSION 3.5)
project(elmo_maestro_bridge)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14) # Need C++14 for lib to compile
endif()

set(CMAKE_CXX_FLAGS "-O3 -Wall -fPIC -no-pie -pthread")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(elmo_maestro_api REQUIRED)
find_package(elmo_motor_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

#Build
include_directories(
  include
)
add_library(${PROJECT_NAME}_lib
  "src/pmas_node.cpp"
  "src/function_test.cpp"
)
set(DEPENDENCIES
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "elmo_maestro_api"
  "elmo_motor_bridge"
)

add_executable(pmas_node src/pmas_node.cpp)
target_link_libraries(pmas_node ${PROJECT_NAME}_lib)

add_executable(joy_test src/joy_test.cpp)
target_link_libraries(joy_test ${PROJECT_NAME}_lib)

ament_target_dependencies(${PROJECT_NAME}_lib ${DEPENDENCIES})

install(TARGETS 
  pmas_node
  joy_test
  DESTINATION lib/${PROJECT_NAME}
)
install(
    DIRECTORY include/
    DESTINATION include
)

  
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
  )
  ament_lint_auto_find_test_dependencies()
endif()
ament_export_include_directories(include)

ament_package()

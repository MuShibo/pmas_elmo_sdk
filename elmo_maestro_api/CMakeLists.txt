cmake_minimum_required(VERSION 3.8)
project(elmo_maestro_api)

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
# find_package(Threads)

set(
  maestro_Libs
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libMMC_APP_LinuxRpc64.a
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libMMCPP_LinuxRpc64.a
)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_INSTALL_PREFIX}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} INTERFACE
  ${CMAKE_INSTALL_PREFIX}/lib/libMMC_APP_LinuxRpc64.a
  ${CMAKE_INSTALL_PREFIX}/lib/libMMCPP_LinuxRpc64.a)
  # ${CMAKE_THREAD_LIBS_INIT})

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)
install(
  FILES ${maestro_Libs}
  DESTINATION lib
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)


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

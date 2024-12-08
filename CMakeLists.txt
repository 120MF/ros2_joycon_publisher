cmake_minimum_required(VERSION 3.8)
project(joycon_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(HARDWARE_INCLUDE_DEPENDS
        rclcpp
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBEVDEV REQUIRED libevdev)

foreach(Dependency IN ITEMS ${HARDWARE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_executable(joycon_publisher src/joycon_publisher.cpp)
ament_target_dependencies(joycon_publisher PUBLIC ${HARDWARE_INCLUDE_DEPENDS})
target_include_directories(joycon_publisher PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
        ${LIBEVDEV_INCLUDE_DIRS})

target_link_libraries(joycon_publisher PUBLIC ${LIBEVDEV_LIBRARIES})

target_compile_features(joycon_publisher PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS joycon_publisher
        DESTINATION lib/${PROJECT_NAME})

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
cmake_minimum_required(VERSION 3.5)
project(cpp01_topic)

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
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(base_interface_demo REQUIRED)

add_executable(demo01_talker_str src/demo01_talker_str.cpp)
target_include_directories(demo01_talker_str PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  demo01_talker_str
  "std_msgs"
  "rclcpp"
  "base_interface_demo"
)

install(TARGETS demo01_talker_str
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

# build outputs and install + logs
# 最关键的是排除 build/ install/ log/，这些目录通常很大且不应推到远程。
/build/
/install/
/log/
/.rosinstall
/.colcon_cache/
/.venv/
/venv/
/__pycache__/
/*.pyc

# Editor/IDE
.vscode/
.idea/
*.swp
*.swo
*~ 

# OS
.DS_Store

# CMake temporary
CMakeCache.txt
CMakeFiles/
cmake_install.cmake
CTestTestfile.cmake
Makefile

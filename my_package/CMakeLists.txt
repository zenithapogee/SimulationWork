



cmake_minimum_required(VERSION 3.5)
project(my_package)

# Add support for C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Set C++ standard required
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#set(Python3_EXECUTABLE "/Users/matthew.huang/miniconda3/envs/humble_env/bin/python")
#set(Python3_NumPy_INCLUDE_DIRS "/Users/matthew.huang/miniconda3/envs/humble_env/lib/python3.10/site-packages/numpy/core/include")

find_package(gazebo REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
#For CV
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})

################## Computer Vision for cpp #############
# Add executable only for cpp
#add_executable(image_listener src/image_listener.cpp)

# Link dependencies
#ament_target_dependencies(image_listener
  #rclcpp
 #sensor_msgs
  #cv_bridge
  #OpenCV
#)





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

################## Plugins #############
add_library(WavePlugin SHARED ${CMAKE_CURRENT_SOURCE_DIR}/plugins/WavePlugin.cpp)
target_link_libraries(WavePlugin ${GAZEBO_LIBRARIES})

ament_export_include_directories(include)
ament_export_dependencies(gazebo_ros)
ament_export_libraries(WavePlugin)

#Install wave plugin
install(
  TARGETS WavePlugin
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

# Install CV script for cpp
#install(TARGETS
  #image_listener
  #DESTINATION lib/${PROJECT_NAME}
#)

# Install CV script for python
install(
    PROGRAMS
    src/image_listener.py
    DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY plugins launch worlds description meshes src include #Update everytime a new folder is added in package
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
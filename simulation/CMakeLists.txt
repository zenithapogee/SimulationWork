cmake_minimum_required(VERSION 3.5)
project(simulation)

# Add support for C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Set C++ standard required
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(gazebo REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

### For CV ###
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)

### For Camera_Sim ###
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(GLEW REQUIRED)
find_package(OGRE REQUIRED)
find_package(rosidl_default_generators REQUIRED)

### For FSM Communication ###
find_package(ros2_interfaces REQUIRED)
include_directories(${ros2_interfaces_INCLUDE_DIRS})


# Set GL preference to quiet build warning
set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)
message(STATUS "FOUND QT 5")
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Xml REQUIRED)
if (Qt5_POSITION_INDEPENDENT_CODE)
  SET(CMAKE_POSITION_INDEPENDENT_CODE ON)
endif()
include_directories(SYSTEM "${Qt5Widgets_INCLUDE_DIRS}")
include_directories(SYSTEM "${Qt5Xml_INCLUDE_DIRS}")
set(QT_QTXML_LIBRARY Qt5::Xml)
set(QT_QTCORE_LIBRARY Qt5::Widgets)

include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${rclcpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
include_directories(${geometry_msgs_INCLUDE_DIRS})

### Camera_sim ###
option(ENABLE_PROFILER "Enable/disable Ignition Profiler" OFF)
if(ENABLE_PROFILER)
  find_package(ignition-common3 COMPONENTS profiler)
  if(ignition-common3_FOUND)
    add_definitions("-DIGN_PROFILER_ENABLE=1" "-DIGN_PROFILER_REMOTERY=1")
    message(STATUS "Profiler is active")
  else()
    message(WARNING "Can't find Ignition common3. Profiler will not be actived")
  endif()
endif()

# Camera_sim: Generate messages in the 'msg' folder
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/ShaderParamUpdate.msg
  DEPENDENCIES
  std_msgs
)

# mallan: add plugins env
ament_environment_hooks(${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/65.irg_gazebo_plugins.dsv.in)

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

################## Plugins ##################

### Laser Plugin ###
add_library(LaserPlugin SHARED ${CMAKE_CURRENT_SOURCE_DIR}/plugins/LaserPlugin.cpp)
target_link_libraries(LaserPlugin ${GAZEBO_LIBRARIES} ${rclcpp_LIBRARIES} ${std_msgs_LIBRARIES} ${ros2_interfaces_LIBRARIES})

### Wave Plugin ###
add_library(WavePlugin SHARED ${CMAKE_CURRENT_SOURCE_DIR}/plugins/WavePlugin.cpp)
target_link_libraries(WavePlugin ${GAZEBO_LIBRARIES} ${rclcpp_LIBRARIES} ${std_msgs_LIBRARIES} ${ros2_interfaces_LIBRARIES})

ament_export_include_directories(include)
ament_export_dependencies(gazebo_ros)
ament_export_libraries(WavePlugin)
ament_export_libraries(LaserPlugin)

#Camera_sim plugin
ament_export_dependencies(rclcpp std_msgs rosidl_default_generators rosidl_default_runtime tf2 tf2_ros)
ament_export_dependencies(ros2_interfaces)

#Install materials for Camera_sim
install(
  DIRECTORY materials
  DESTINATION share/${PROJECT_NAME}/
)

# Install Laser Plugin
install(
  TARGETS LaserPlugin 
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

#Install wave plugin
install(
  TARGETS WavePlugin
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

# Install CV script for python
install(
    PROGRAMS
    src/image_listener.py
    DESTINATION lib/${PROJECT_NAME}
)

#Initialise cmakelist.txt for IRGCameraSimPlugins
add_subdirectory(IRGCameraSimPlugins)

install(DIRECTORY IRGCameraSimPlugins plugins msg launch worlds description materials meshes src #Update everytime a new folder is added in package
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
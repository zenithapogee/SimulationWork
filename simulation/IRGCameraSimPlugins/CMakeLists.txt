set(TARGET_NAME IRGCameraSim)
add_library(${TARGET_NAME}SensorPlugin SHARED 
  ${CMAKE_CURRENT_SOURCE_DIR}/CameraCompositor.h
  ${CMAKE_CURRENT_SOURCE_DIR}/CameraCompositorListener.h
  ${CMAKE_CURRENT_SOURCE_DIR}/CameraSimSensorPlugin.h
  ${CMAKE_CURRENT_SOURCE_DIR}/CameraCompositor.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/CameraCompositorListener.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/CameraSimSensorPlugin.cpp
)

add_library(${TARGET_NAME}VisualPlugin SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/CameraCompositor.h
  ${CMAKE_CURRENT_SOURCE_DIR}/CameraCompositorListener.h
  ${CMAKE_CURRENT_SOURCE_DIR}/CameraSimVisualPlugin.h
  ${CMAKE_CURRENT_SOURCE_DIR}/CameraCompositor.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/CameraCompositorListener.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/CameraSimVisualPlugin.cpp
)

ament_target_dependencies(${TARGET_NAME}SensorPlugin
  gazebo_ros
  rclcpp
  std_msgs
)
target_link_libraries(${TARGET_NAME}SensorPlugin ${GAZEBO_LIBRARIES})
set_target_properties(${TARGET_NAME}SensorPlugin PROPERTIES COMPILE_FLAGS "${GAZEBO_CXX_FLAGS}")

ament_target_dependencies(${TARGET_NAME}VisualPlugin
  gazebo_ros
  rclcpp
  std_msgs
)
target_link_libraries(${TARGET_NAME}VisualPlugin ${GAZEBO_LIBRARIES})
set_target_properties(${TARGET_NAME}VisualPlugin PROPERTIES COMPILE_FLAGS "${GAZEBO_CXX_FLAGS}")

install(TARGETS
  ${TARGET_NAME}SensorPlugin
  ${TARGET_NAME}VisualPlugin
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  #update my_package name whenever package name changes
)


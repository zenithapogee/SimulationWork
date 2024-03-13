ament_export_dependencies(gazebo_ros)
ament_export_include_directories(include)
ament_export_libraries(my_wave_plugin)

install(
  DIRECTORY include/
  DESTINATION include
)

add_library(my_wave_plugin SHARED plugins/WavePlugin.cpp)
target_include_directories(my_wave_plugin PRIVATE include)
ament_target_dependencies(my_wave_plugin gazebo_ros)

install(
  TARGETS my_wave_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

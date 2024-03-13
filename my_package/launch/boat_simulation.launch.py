import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'my_package'
    file_subpath = 'description/dock.urdf.xacro'

    # Use xacro to process the file of robot_1 (pier)
    xacro_file_robot_1 = os.path.join(get_package_share_directory(pkg_name), file_subpath)
    robot_1_description_raw = xacro.process_file(xacro_file_robot_1).toxml()


    world_file_path = os.path.join(get_package_share_directory(pkg_name), 'worlds/ocean.world')
    # Configure the node for robot state publisher_1 (Pier)
    node_robot_state_publisher_1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        namespace='robot_1',
        parameters=[{'robot_description': robot_1_description_raw, 'use_sim_time': True}]
    )



    # Spawn the robot_1 (dock) entity in Gazebo
    spawn_entity_robot_1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        namespace='robot_1',
        arguments=['-topic', 'robot_description', '-entity', 'robot1'],
        output='screen'
    )



    #World configuration
    #Configure the Gazebo launch with your custom world file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments={'world': world_file_path}.items(),
    )
    water_surface_height_param = DeclareLaunchArgument(
        'water_surface_height',
        default_value='4', 
        description='Height of the water surface'
    )

    #  Start image pipeline node for camera 1. Arguments: topic, center_offset_x,center_offset_y
    listener_node_1 = Node(
        package="my_package",
        executable="image_listener.py",
        # namespace="robot_1",
        output="screen",
        arguments=['/camera_1/image_raw','0','0']
       
    )

    # Launch the Gazebo simulation along with other nodes
    return LaunchDescription([
        water_surface_height_param,
        gazebo,
        node_robot_state_publisher_1,
        spawn_entity_robot_1,
        listener_node_1,
    ])
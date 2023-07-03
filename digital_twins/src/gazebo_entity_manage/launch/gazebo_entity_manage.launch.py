import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
# from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    package_name = "gazebo_entity_manage"
    default_world_name = "test_site"
    
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    
    default_world_file = os.path.join(pkg_share, f'world/{default_world_name}.world')
    world_file = LaunchConfiguration('world_file', default=default_world_file)
    
    models_folder = LaunchConfiguration('models_folder', default=os.path.join(pkg_share, "urdf"))
    
    ld = LaunchDescription()
    
    gazebo_cmd =  ExecuteProcess(
        cmd=['gazebo', world_file, '--verbose','-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )
    
    entity_manager_node = Node(
        name="entity_manager",
        package='gazebo_entity_manage',
        executable='entity_manage_node',
        parameters=[{'models_folder': models_folder}]
    )
    
    rosbridge_server_node = Node(
        name = "rosbridge_server",
        package = "rosbridge_server",
        executable = "rosbridge_websocket"
    )

    ld.add_action(gazebo_cmd)
    ld.add_action(entity_manager_node)
    ld.add_action(rosbridge_server_node)
    
    return ld

if __name__ == '__main__':
    generate_launch_description()
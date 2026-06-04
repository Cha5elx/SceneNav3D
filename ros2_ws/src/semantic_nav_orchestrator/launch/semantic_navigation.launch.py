from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros
import os


def generate_launch_description():
    package_dir = get_package_share_directory('semantic_nav_orchestrator')
    params = os.path.join(package_dir, 'config', 'semantic_nav_params.yaml')
    locations = os.path.join(package_dir, 'config', 'named_locations.yaml')
    objects = os.path.join(package_dir, 'config', 'object_map.yaml')
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='semantic_nav_orchestrator',
            executable='semantic_nav_node',
            name='semantic_nav',
            parameters=[
                params,
                {
                    'locations_file': locations,
                    'objects_file': objects,
                    'use_sim_time': True,
                },
            ],
            output='screen',
        ),
    ])

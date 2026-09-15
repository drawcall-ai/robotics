from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    root = Path(__file__).parent
    params = str(root / 'nav2.yaml')
    nodes = [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'use_sim_time': True, 'robot_description': (root / 'robot.urdf').read_text()}]),
        Node(package='nav2_controller', executable='controller_server', name='controller_server', parameters=[params]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server', parameters=[params]),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server', parameters=[params]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', parameters=[params]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation',
             parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': [
                 'controller_server', 'planner_server', 'behavior_server', 'bt_navigator']}]),
    ]
    slam = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
        Path(get_package_share_directory('slam_toolbox')) / 'launch/online_async_launch.py')),
        launch_arguments={'use_sim_time': 'true', 'slam_params_file': str(root / 'slam.yaml')}.items())
    return LaunchDescription([
        RegisterEventHandler(OnProcessExit(on_exit=[
            EmitEvent(event=Shutdown(reason='SLAM or navigation process exited'))])),
        slam, *nodes,
    ])

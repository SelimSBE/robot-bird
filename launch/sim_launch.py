from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('robotic_bird')
    
    # Get paths
    world_path = os.path.join(pkg_share, 'worlds', 'robot_bird.world')
    urdf_path = os.path.join(pkg_share, 'urdf', 'robotic_bird.urdf.xacro')

    # Environment variables
    env_vars = [
        SetEnvironmentVariable('GAZEBO_MASTER_URI', 'http://localhost:11345'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', os.path.join(pkg_share, 'models')),
        SetEnvironmentVariable('IGN_RENDERING_ENGINE_PATH', ''),
        SetEnvironmentVariable('DISPLAY', ''),
        # Disable audio completely
        SetEnvironmentVariable('AUDIODEV', '/dev/null'),
        SetEnvironmentVariable('SDL_AUDIODRIVER', 'dummy'),
        SetEnvironmentVariable('ALSA_CONFIG_PATH', '/dev/null'),
        # Set OpenGL to use software rendering
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
        SetEnvironmentVariable('OGRE_RTT_MODE', 'Copy'),
    ]

    # Include Gazebo server
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 
                        'launch', 'gzserver.launch.py')
        ]),
        launch_arguments={
            'world': world_path,
            'verbose': 'true',
            'gui': 'false',
            'server_required': 'true',
            'port': '11345',
            'headless': 'true',
            'enable_audio': 'false',
            'enable_rendering': 'false'
        }.items()
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[urdf_path]
    )

    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-entity', 'robotic_bird',
            '-file', urdf_path,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.0'
        ]
    )

    return LaunchDescription(
        env_vars + [
            gazebo,
            robot_state_publisher,
            spawn_robot,
            Node(
                package="robotic_bird",
                executable="flight_controller",
                name="flight_controller",
                output="screen"),
            Node(
                package="robotic_bird",
                executable="navigator",
                name="navigator",
                output="screen"),
        ]
    )
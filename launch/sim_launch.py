from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('robotic_bird')
    
    # Define paths
    world_path = os.path.join(pkg_share, 'worlds', 'robot_bird.world')
    urdf_path = os.path.join(pkg_share, 'urdf', 'robotic_bird.urdf.xacro')
    
    # Kill any existing Gazebo server
    os.system('pkill gzserver')
    
    # Environment variables
    env_vars = [
        # Gazebo settings
        SetEnvironmentVariable('GAZEBO_MASTER_URI', 'http://localhost:11345'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', os.path.join(pkg_share, 'models')),
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', 
                             os.path.join(get_package_share_directory('robotic_bird'), 'lib')),
        
        # Force software rendering
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
        SetEnvironmentVariable('GALLIUM_DRIVER', 'llvmpipe'),
        SetEnvironmentVariable('DISPLAY', ':0'),
        
        # RViz specific settings
        SetEnvironmentVariable('OGRE_RTT_MODE', 'Copy'),
        SetEnvironmentVariable('MESA_GL_VERSION_OVERRIDE', '3.3'),
        SetEnvironmentVariable('MESA_GLSL_VERSION_OVERRIDE', '330'),
        
        # Disable hardware acceleration
        SetEnvironmentVariable('LIBGL_ALWAYS_INDIRECT', '0'),
        
        # Qt settings
        SetEnvironmentVariable('QT_X11_NO_MITSHM', '1'),
        SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),
        
        # X11 settings
        SetEnvironmentVariable('XAUTHORITY', os.path.expanduser('~/.Xauthority')),
        SetEnvironmentVariable('XDG_RUNTIME_DIR', '/tmp/runtime-root'),
    ]

    # Delay RViz start to ensure robot model is loaded
    rviz_delay = ExecuteProcess(
        cmd=['sleep', '2'],
        output='screen'
    )
    # Include Gazebo server with different port
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
            'port': '11346',  # Changed port
            'headless': 'false'  # Changed to false for visualization
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
            '-z', '0.0'
        ]
    )

    # odom_to_tf node
    odom_to_tf = Node(
        package='robotic_bird',
        executable='nodes/odom_to_tf.py',  # path relative to package
        name='odom_to_tf',
        output='screen'
    )
    target_publisher = Node(
        package='robotic_bird',
        executable='nodes/target_publisher.py',
        name='target_publisher',
        output='screen'
    )

    # Add RViz with modified settings
    rviz_config = os.path.join(pkg_share, 'rviz', 'robot_bird.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        output='screen',
        parameters=[{'use_sim_time': True}],
        additional_env={
            'LIBGL_DEBUG': 'verbose',
            'OGRE_RENDERING_PLUGIN': 'GL',
            'LIBGL_ALWAYS_SOFTWARE': '1'
        }
    )

    return LaunchDescription(
        env_vars + [
            gazebo,
            robot_state_publisher,
            target_publisher,
            odom_to_tf,
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
            rviz
        ]
    )
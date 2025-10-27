import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from xacro import process_file

def generate_launch_description():
    # Path to your SDF file
    xacro_file = os.path.join(
        get_package_share_directory('ardupilot_gazebo'),
        'models/iris_with_gimbal',
        'model.urdf.xacro'
    )
    

    robot_desc = process_file(xacro_file).toxml()

    # Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 
                        'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'gimbal_small_3d'
        ],
        output='screen'
    )
    
    # Bridge for joint states (Gazebo -> ROS 2)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/empty/model/gimbal_small_3d/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        remappings=[
            ('/world/empty/model/gimbal_small_3d/joint_state', '/joint_states')
        ],
        output='screen'
    )
    
    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }]
    )

    # RViz.
    pkg_project_bringup = get_package_share_directory("ardupilot_gz_bringup")
   
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", f'{Path(pkg_project_bringup) / "rviz" / "iris.rviz"}'],
    )
    return LaunchDescription([
        gazebo,
        spawn_entity,
        bridge,
        robot_state_publisher,
        rviz,
    ])
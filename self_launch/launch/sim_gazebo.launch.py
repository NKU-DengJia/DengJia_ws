import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


package_name='learning_gazebo'
def generate_launch_description():
    self_load_urdf_into_gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','self_load_urdf_into_gazebo.launch.py'
                )])  
                ) 

    chassis_sim_NodeChassis = Node(package='chassis_sim',
                            executable='NodeChassis',
                            output='screen',
                            parameters=[{'use_sim_time': True}]  # 使用仿真时间
                            )
    
    traj_publisher_Node = Node(package='module_planner',
                            executable='NodeTrajPublisher',
                            output='screen',
                            parameters=[{'use_sim_time': True}]  # 使用仿真时间
                            )
    
    main_controller_Node = Node(package='module_controller',
                            executable='ChassisCmdPublisherNode',
                            output='screen',
                            parameters=[{'use_sim_time': True}]  # 使用仿真时间
                            )
    
    return LaunchDescription([
        self_load_urdf_into_gazebo,
        traj_publisher_Node,
        chassis_sim_NodeChassis,
        main_controller_Node,
    ])
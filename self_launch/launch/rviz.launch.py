from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # 启动节点, 发布路网marker
    NodeReadtxtAndPublish = Node(
        package='module_planner',
        executable='NodeReadtxtAndPublish',
        output='screen',
        parameters=[{'use_sim_time': True}]  # 使用仿真时间
    )
    # 启动节点, 发布自车marker
    NodeEgoMarkerPublish = Node(
        package='module_planner',
        executable='NodeEgoMarkerPublish',
        output='screen',
        parameters=[{'use_sim_time': True}]  # 使用仿真时间
    )
    # 启动节点, 发布轨迹marker
    NodeTrajMarkerPublisher = Node(
        package='module_planner',
        executable='NodeTrajMarkerPublisher',
        output='screen',
        parameters=[{'use_sim_time': True}]  # 使用仿真时间
    )

    # 启动 RViz
    rviz_config_dir = os.path.join(
        os.path.dirname(__file__),
        'rviz',
        '/home/dengjia/DengJia_ws/src/self_launch/rviz/self_test.rviz'  
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_dir] 
    )

    return LaunchDescription([
        NodeReadtxtAndPublish,
        NodeEgoMarkerPublish,
        NodeTrajMarkerPublisher,
        rviz_node
    ])

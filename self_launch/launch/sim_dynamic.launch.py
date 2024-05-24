from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    chassis_sim_NodeDynamic = Node(package='chassis_sim',
                            executable='NodeDynamic',
                            output='screen',
                            parameters=[{'use_sim_time': False}]  
                            )
    
    traj_publisher_Node = Node(package='module_planner',
                            executable='NodeTrajPublisher',
                            output='screen',
                            parameters=[{'use_sim_time': False}]  
                            )
    
    main_controller_Node = Node(package='module_controller',
                            executable='ChassisCmdPublisherNode',
                            output='screen',
                            parameters=[{'use_sim_time': False}]  
                            )
    
    Node_record_csv = Node(package='data_plot',
                            executable='node_record_csv',
                            output='screen',
                            parameters=[{'use_sim_time': False}]  
                            )
    
    
    return LaunchDescription([
        chassis_sim_NodeDynamic,
        traj_publisher_Node,
        main_controller_Node,
        Node_record_csv,
    ])
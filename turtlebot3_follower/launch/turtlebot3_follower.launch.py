import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    number_of_follower=2
    follower = Node(
        package='turtlebot3_follower',
        executable='follower',
        output='screen',
        arguments=[str(number_of_follower)],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    nodes=[]
    nodes.append(follower)
    for i in range(number_of_follower):
        namespace = f"TB3_{i+2}"  # TB3_2, TB3_3, ...

        ctrl_yaml_path = os.path.join(
            get_package_share_directory('turtlebot3_follower'),
            'param',
            f'controll_server{i+1}.yaml'
        )
        ctrl_node = Node(
            package='nav2_controller',
            executable='controller_server',
            namespace=namespace,
            name='controller_server',
            output='screen',
            parameters=[
                ctrl_yaml_path,
                {'use_sim_time': use_sim_time},
            ],
        )

        lifecycle_node = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            namespace=namespace,
            name='lifecycle_manager_controller',
            output='screen',
            parameters=[
                {'autostart': True},
                {'use_sim_time': use_sim_time},
                {'node_names': ['controller_server']}
            ]
        )
        nodes.append(ctrl_node)
        nodes.append(lifecycle_node)


    return LaunchDescription(nodes)

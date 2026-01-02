from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # 1. Start the ROS-TCP Endpoint (Bridge to Unity)
        Node(
            package='ros_tcp_endpoint',
            executable='default_server_endpoint',
            parameters=[{'ROS_IP': '0.0.0.0'}, {'ROS_TCP_PORT': 10000}],
        ),

        # 2. Start the Neutral Consciousness Engine (SNN)
        Node(
            package='neutral_consciousness',
            executable='cortex_node',  # You will define this entry point
            name='cortex_core',
            output='screen'
        ),

        # 3. (Optional) Auto-launch Unity Build if available
        # ExecuteProcess(
        #     cmd=['./unity_project/Builds/Linux/NeutralBody.x86_64'],
        #     output='screen'
        # )
    ])

"""
Master System Launch File - Neutral Consciousness Engine

Launches the complete "Mind" system:
1. ROS-TCP Endpoint (Unity Bridge)
2. Visual Cortex SNN (Predictive Coding)
3. Dream Engine (Generative Model)
4. Neural Firewall (Brainjacking Defense)
5. Homomorphic Encryption Node (Satellite Security)
6. Latency Injector (OISL Simulation)
7. Split Brain Test (Hemispheric Protocol)

Usage:
    ros2 launch neutral_consciousness master_system.launch.py

    # With custom parameters:
    ros2 launch neutral_consciousness master_system.launch.py round_trip_time_ms:=50.0 dream_mode:=true encryption_enabled:=true

Prerequisites:
    - Clone ROS-TCP-Endpoint: https://github.com/Unity-Technologies/ROS-TCP-Endpoint
      into ros2_ws/src/ and build with colcon
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Launch arguments
    rtt_arg = DeclareLaunchArgument(
        'round_trip_time_ms',
        default_value='20.0',
        description='Simulated OISL round-trip time in milliseconds'
    )
    
    dream_mode_arg = DeclareLaunchArgument(
        'dream_mode',
        default_value='false',
        description='Start Dream Engine in dream mode (free-running generation)'
    )
    
    encryption_arg = DeclareLaunchArgument(
        'encryption_enabled',
        default_value='true',
        description='Enable homomorphic encryption for satellite communication'
    )

    return LaunchDescription([
        rtt_arg,
        dream_mode_arg,
        encryption_arg,
        
        # ============================================================
        # 1. ROS-TCP Endpoint (Bridge to Unity)
        # NOTE: Requires cloning from Unity-Technologies/ROS-TCP-Endpoint
        # ============================================================
        Node(
            package='ros_tcp_endpoint',
            executable='default_server_endpoint',
            name='ros_tcp_endpoint',
            parameters=[
                {'ROS_IP': '0.0.0.0'},
                {'ROS_TCP_PORT': 10000}
            ],
            output='screen'
        ),

        # ============================================================
        # 2. Visual Cortex SNN (The "Brain" - Predictive Coding)
        # Receives: unity/camera/raw
        # Publishes: /neural_data/prediction_error, /synchronization_health
        # ============================================================
        Node(
            package='neutral_consciousness',
            executable='cortex_node',
            name='visual_cortex',
            output='screen'
        ),

        # ============================================================
        # 3. Dream Engine (Generative Model)
        # Receives: cortex/visual/activity, dream/enable
        # Publishes: dream/prediction, dream/prediction_error
        # ============================================================
        Node(
            package='neutral_consciousness',
            executable='dream_node',
            name='dream_engine',
            parameters=[
                {'prediction_rate_hz': 30.0},
                {'dream_mode': LaunchConfiguration('dream_mode')}
            ],
            output='screen'
        ),

        # ============================================================
        # 4. Neural Firewall (Brainjacking Defense - Hybrid TEE)
        # Receives: /satellite_uplink/incoming_data
        # Publishes: /verified_neural_stream, firewall/kill_switch
        # ============================================================
        Node(
            package='neutral_consciousness',
            executable='firewall_node',
            name='neural_firewall',
            output='screen'
        ),

        # ============================================================
        # 5. Homomorphic Encryption Node (Satellite Security)
        # Receives: /neural_stream/outgoing, /satellite_downlink/encrypted
        # Publishes: /satellite_uplink/encrypted, /neural_stream/decrypted
        # Ensures satellite cannot read neural data (CKKS scheme)
        # ============================================================
        Node(
            package='neutral_consciousness',
            executable='he_node',
            name='homomorphic_encryption',
            parameters=[
                {'encryption_enabled': LaunchConfiguration('encryption_enabled')},
                {'log_metrics': True}
            ],
            output='screen'
        ),

        # ============================================================
        # 6. Latency Injector (OISL Delay Simulation)
        # Receives: /neural_stream/generated
        # Publishes: /neural_stream/delayed
        # Validates against Libet's 500ms limit
        # ============================================================
        Node(
            package='neutral_consciousness',
            executable='latency_injector_node',
            name='latency_injector',
            parameters=[
                {'round_trip_time_ms': LaunchConfiguration('round_trip_time_ms')}
            ],
            output='screen'
        ),

        # ============================================================
        # 7. Split Brain Test (Uni-hemispheric Subjective Protocol)
        # Receives: /camera/left_eye, /camera/right_eye, /synchronization_health
        # Publishes: /conscious_output/unified_field
        # Service: /trigger_hemispheric_switch
        # ============================================================
        Node(
            package='neutral_consciousness',
            executable='split_brain_node',
            name='split_brain_test',
            output='screen'
        ),

        # ============================================================
        # 8. (Optional) Auto-launch Unity Build if available
        # Uncomment and adjust path for your platform
        # ============================================================
        # ExecuteProcess(
        #     cmd=['./unity_project/Builds/Linux/NeutralBody.x86_64'],
        #     output='screen'
        # )
    ])

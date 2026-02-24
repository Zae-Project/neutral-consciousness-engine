"""
Master System Launch File - Neutral Consciousness Engine

Launches the complete "Mind" system:
1. ROS-TCP Endpoint (Unity Bridge)
2. Visual Cortex SNN (Isomorphic Topographic Predictive Coding)
3. Sensory Tectum (Fast Multisensory Convergence — Step 1 Consciousness)
4. Limbic Node (Affective Valence & Dopamine Modulation)
5. Reentrant Processor (Bidirectional Binding — Consciousness Gate)
6. Dream Engine (Generative Model / Pallium — Step 2 Consciousness)
7. Neural Firewall (Brainjacking Defense)
8. Homomorphic Encryption Node (Satellite Security)
9. Latency Injector (OISL Simulation)
10. Split Brain Test (Hemispheric Protocol)

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
        # Receives: unity/camera/raw, dream/top_down_prediction
        # Publishes: /neural_data/prediction_error, /neural_data/cortex_activity, /synchronization_health
        # ============================================================
        Node(
            package='neutral_consciousness',
            executable='cortex_node',
            name='visual_cortex',
            output='screen'
        ),

        # ============================================================
        # 3. Sensory Tectum (Fast Multisensory Convergence — Step 1)
        # Receives: /neural_data/cortex_activity, /neural_data/spatial_error_map,
        #           /unity/proprioception
        # Publishes: /tectum/unified_map, /tectum/salience_map, /tectum/reflex_motor
        # Architecture: 2,000 LIF neurons (5ms tau_rc, ultra-fast)
        # ============================================================
        Node(
            package='neutral_consciousness',
            executable='tectum_node',
            name='sensory_tectum',
            output='screen'
        ),

        # ============================================================
        # 4. Limbic Node (Affective Valence & Dopamine Modulation)
        # Receives: /unity/battery_level, /unity/collision_damage,
        #           /unity/goal_proximity, /unity/threat_distance,
        #           /neural_data/prediction_error
        # Publishes: /consciousness/affective/valence, /consciousness/affective/dopamine,
        #            /consciousness/affective/state, /consciousness/affective/reward_prediction_error
        # Architecture: 500 LIF neurons (VTA + PAG + Habenula)
        # ============================================================
        Node(
            package='neutral_consciousness',
            executable='limbic_node',
            name='limbic_node',
            output='screen'
        ),

        # ============================================================
        # 5. Reentrant Processor (Bidirectional Binding — Consciousness Gate)
        # Receives: /tectum/unified_map, dream/top_down_prediction,
        #           /consciousness/affective/dopamine
        # Publishes: /reentrant/bound_representation, /reentrant/binding_error,
        #            /reentrant/converged, /reentrant/error_map
        # Architecture: 4,000 LIF neurons (bottom-up + top-down + binding + convergence)
        # ============================================================
        Node(
            package='neutral_consciousness',
            executable='reentrant_node',
            name='reentrant_processor',
            parameters=[
                {'convergence_threshold': 0.15},
                {'max_iterations': 10},
                {'binding_rate_hz': 100.0}
            ],
            output='screen'
        ),

        # ============================================================
        # 6. Dream Engine (Generative Model / Pallium — Step 2)
        # Receives: /neural_data/cortex_activity, /neural_data/prediction_error,
        #           /tectum/unified_map, /consciousness/affective/dopamine,
        #           /reentrant/bound_representation
        # Publishes: dream/top_down_prediction, dream/semantic_state,
        #            dream/topographic_reconstruction
        # Architecture: 10,000 neurons (Compression, Semantic, Cleanup, Decompression)
        # ============================================================
        Node(
            package='neutral_consciousness',
            executable='dream_node',
            name='dream_engine',
            parameters=[
                {'prediction_rate_hz': 30.0},
                {'dream_mode': LaunchConfiguration('dream_mode')},
                {'tau_slow': 100.0}  # MTRNN slow dynamics (ms)
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

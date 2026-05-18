from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Launch file for Project TRINITY Swarm Coordination Engine.
    
    Launches:
    1. Centrally-distributed Swarm Commander Node
    2. 3x Mapped UAV Controller instances isolated inside namespaces (/uav_1, /uav_2, /uav_3)
    """
    ld = LaunchDescription()
    
    # 1. Centrally-distributed Swarm Commander Node
    commander_node = Node(
        package='trinity_swarm',
        executable='swarm_commander',
        name='swarm_commander',
        parameters=[{
            'default_formation': 'V-SHAPE',
            'target_x': 120.0,
            'target_y': 120.0,
            'target_z': 15.0
        }]
    )
    ld.add_action(commander_node)
    
    # 2. Spawn 3x UAV Controllers mapped under namespaces
    num_drones = 3
    for i in range(num_drones):
        uav_id = f"uav_{i+1}"
        is_leader = (i == 0)
        
        controller_node = Node(
            package='trinity_swarm',
            executable='uav_controller',
            namespace=uav_id,
            name='swarm_controller',
            parameters=[{
                'uav_id': uav_id,
                'is_leader': is_leader
            }]
        )
        ld.add_action(controller_node)
        
    # 3. Spawn Mock SITL Physics & Telemetry bridge
    mock_sitl_node = Node(
        package='trinity_swarm',
        executable='mock_sitl',
        name='mock_sitl'
    )
    ld.add_action(mock_sitl_node)
    
    # 4. Spawn 3D RViz Visualizer Node
    visualizer_node = Node(
        package='trinity_swarm',
        executable='swarm_visualizer',
        name='swarm_visualizer'
    )
    ld.add_action(visualizer_node)
    
    # 5. Launch Pre-Configured RViz2 Workspace
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', '/mnt/d/Drone_Projects/Project_ICARUS/Project_TRINITY/launch/trinity_config.rviz']
    )
    ld.add_action(rviz_node)
        
    return ld

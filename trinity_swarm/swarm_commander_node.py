#!/usr/bin/env python3
# ==============================================================================
# 🛸 Project TRINITY: Swarm Commander ROS 2 Node
# ==============================================================================
# Broadcasts active flight targets, dynamic formation commands, and monitors
# individual UAV telemetry heartbeats to trigger self-healing mesh structures.
# ==============================================================================

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

class SwarmCommander(Node):
    def __init__(self):
        super().__init__('swarm_commander')
        
        # ROS 2 publishers
        self.formation_pub = self.create_publisher(String, '/swarm/formation_cmd', 10)
        self.target_pub = self.create_publisher(Point, '/swarm/target_waypoint', 10)
        self.roster_pub = self.create_publisher(String, '/swarm/active_roster', 10)
        
        # ROS 2 subscribers
        self.mesh_sub = self.create_subscription(
            DiagnosticArray,
            '/swarm/mesh_status',
            self.mesh_callback,
            10
        )
        
        # State parameters
        self.declare_parameter('default_formation', 'V-SHAPE')
        self.declare_parameter('target_x', 50.0)
        self.declare_parameter('target_y', 50.0)
        self.declare_parameter('target_z', 10.0)
        
        self.current_formation = self.get_parameter('default_formation').value
        self.target_coords = [
            self.get_parameter('target_x').value,
            self.get_parameter('target_y').value,
            self.get_parameter('target_z').value
        ]
        
        self.active_uavs = set()
        self.last_heartbeats = {} # Maps UAV ID -> timestamp
        
        # Timers
        self.cmd_timer = self.create_timer(1.0, self.broadcast_directives)
        self.monitor_timer = self.create_timer(0.5, self.monitor_mesh_health)
        
        self.get_logger().info('=========================================')
        self.get_logger().info('🛸 TRINITY: Swarm Commander initialized.')
        self.get_logger().info(f'✓ Initial Formation: {self.current_formation}')
        self.get_logger().info(f'✓ Global Waypoint Target: {self.target_coords}')
        self.get_logger().info('=========================================')

    def broadcast_directives(self):
        """Periodically publishes the mission objective targets and current formations."""
        # 1. Publish target coordinate
        t_msg = Point()
        t_msg.x = self.target_coords[0]
        t_msg.y = self.target_coords[1]
        t_msg.z = self.target_coords[2]
        self.target_pub.publish(t_msg)
        
        # 2. Publish formation structure
        f_msg = String()
        f_msg.data = self.current_formation
        self.formation_pub.publish(f_msg)
        
        # 3. Broadcast active roster
        r_msg = String()
        r_msg.data = ','.join(sorted(list(self.active_uavs)))
        self.roster_pub.publish(r_msg)

    def mesh_callback(self, msg: DiagnosticArray):
        """Processes high-frequency heartbeats from active UAV nodes."""
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        for status in msg.status:
            uav_id = status.name
            if status.message == 'OK':
                self.last_heartbeats[uav_id] = current_time
                if uav_id not in self.active_uavs:
                    self.active_uavs.add(uav_id)
                    self.get_logger().info(f'[MESH] New node detected: {uav_id}. Recalculating offsets.')

    def monitor_mesh_health(self):
        """Monitors network heartbeats to trigger self-healing mesh re-indexing."""
        current_time = self.get_clock().now().nanoseconds / 1e9
        timeout = 2.5 # Lost link threshold [s]
        
        offline_nodes = []
        for uav_id, last_t in list(self.last_heartbeats.items()):
            if current_time - last_t > timeout:
                offline_nodes.append(uav_id)
                
        if offline_nodes:
            for node in offline_nodes:
                if node in self.active_uavs:
                    self.active_uavs.remove(node)
                    self.get_logger().warn(f'⚡ [MESH FAILURE] Heartbeat lost for {node}!')
                    
                    # Dynamically switch formation to CIRCLE for safety if a failure is caught
                    if self.current_formation != 'CIRCLE':
                        self.current_formation = 'CIRCLE'
                        self.get_logger().info('🔄 [MESH] Triggering self-healing... Moving to CIRCLE formation!')
                    
                del self.last_heartbeats[node]

def main(args=None):
    rclpy.init(args=args)
    node = SwarmCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

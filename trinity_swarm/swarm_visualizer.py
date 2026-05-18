#!/usr/bin/env python3
# ==============================================================================
# 🛸 Project TRINITY: Swarm 3D RViz Visualization Publisher
# ==============================================================================
# Translates multi-UAV coordinate telemetry streams into high-fidelity
# 3D spatial markers, dynamic forcefield vector links, and target indicators.
# ==============================================================================

import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import String
from geometry_msgs.msg import Point
from px4_msgs.msg import VehicleOdometry
from visualization_msgs.msg import Marker, MarkerArray

class SwarmVisualizer(Node):
    def __init__(self):
        super().__init__('swarm_visualizer')
        
        # State variables
        self.uav_positions = {} # Maps uav_id -> [x, y, z]
        self.target_coords = [120.0, 120.0, 15.0]
        self.active_formation = 'V-SHAPE'
        self.active_roster = []
        
        # Publishers
        self.marker_pub = self.create_publisher(MarkerArray, '/swarm/rviz_markers', 10)
        
        # Subscriptions
        self.create_subscription(Point, '/swarm/target_waypoint', self.target_callback, 10)
        self.create_subscription(String, '/swarm/formation_cmd', self.formation_callback, 10)
        self.create_subscription(String, '/swarm/active_roster', self.roster_callback, 10)
        
        # Create subscriptions for 3 drones' telemetries
        for i in range(3):
            uav_id = f"uav_{i+1}"
            self.create_subscription(
                VehicleOdometry,
                f'/{uav_id}/fmu/out/vehicle_odometry',
                self.make_telemetry_callback(uav_id),
                10
            )
            
        # 10Hz visual publisher loop
        self.viz_timer = self.create_timer(0.1, self.publish_markers)
        
        self.get_logger().info('========================================================')
        self.get_logger().info('🛸 TRINITY: 3D RViz Spatial Visualizer Node initialized.')
        self.get_logger().info('✓ Drawing glowing boid meshes & dynamic vector lasers...')
        self.get_logger().info('========================================================')

    def make_telemetry_callback(self, uav_id):
        return lambda msg: self.telemetry_callback(uav_id, msg)

    def telemetry_callback(self, uav_id, msg: VehicleOdometry):
        self.uav_positions[uav_id] = [msg.position[0], msg.position[1], msg.position[2]]

    def target_callback(self, msg: Point):
        self.target_coords = [msg.x, msg.y, msg.z]

    def formation_callback(self, msg: String):
        self.active_formation = msg.data

    def roster_callback(self, msg: String):
        self.active_roster = msg.data.split(',') if msg.data else []

    def publish_markers(self):
        """Generates and publishes 3D visual markers for RViz display."""
        ma = MarkerArray()
        current_time = self.get_clock().now().to_msg()
        
        # 1. Spawning quadcopter visual markers (Sphere representation with high-contrast rings)
        for i in range(3):
            uav_id = f"uav_{i+1}"
            if uav_id not in self.uav_positions:
                continue
                
            pos = self.uav_positions[uav_id]
            is_active = uav_id in self.active_roster
            
            # Quadcopter main body sphere
            m_body = Marker()
            m_body.header.frame_id = "map"
            m_body.header.stamp = current_time
            m_body.ns = "uav_bodies"
            m_body.id = i
            m_body.type = Marker.SPHERE
            m_body.action = Marker.ADD
            m_body.pose.position.x = pos[0]
            m_body.pose.position.y = pos[1]
            m_body.pose.position.z = pos[2]
            
            # Size mapping
            m_body.scale.x = 2.0
            m_body.scale.y = 2.0
            m_body.scale.z = 0.8
            
            # Color mapping (Active: glowing cyan; Failed: glowing red)
            if is_active:
                m_body.color.r = 0.0
                m_body.color.g = 0.95
                m_body.color.b = 1.0
                m_body.color.a = 0.85
            else:
                m_body.color.r = 1.0
                m_body.color.g = 0.1
                m_body.color.b = 0.2
                m_body.color.a = 0.9
                
            ma.markers.append(m_body)
            
            # Text tag marker for UAV identifier and health
            m_text = Marker()
            m_text.header.frame_id = "map"
            m_text.header.stamp = current_time
            m_text.ns = "uav_labels"
            m_text.id = i
            m_text.type = Marker.TEXT_VIEW_FACING
            m_text.action = Marker.ADD
            m_text.pose.position.x = pos[0]
            m_text.pose.position.y = pos[1]
            m_text.pose.position.z = pos[2] + 2.0
            m_text.scale.z = 1.2
            
            m_text.color.r = 1.0
            m_text.color.g = 1.0
            m_text.color.b = 1.0
            m_text.color.a = 1.0
            
            if is_active:
                m_text.text = f"{uav_id.upper()} [ACTIVE]"
            else:
                m_text.text = f"{uav_id.upper()} [ENGINE SEIZURE - OFFLINE]"
            ma.markers.append(m_text)

        # 2. Draw glowing dynamic laser vector mesh links (connecting the active swarm wingmen)
        active_uavs = [f"uav_{i+1}" for i in range(3) if f"uav_{i+1}" in self.uav_positions and f"uav_{i+1}" in self.active_roster]
        
        if len(active_uavs) >= 2:
            m_lines = Marker()
            m_lines.header.frame_id = "map"
            m_lines.header.stamp = current_time
            m_lines.ns = "swarm_forcefields"
            m_lines.id = 0
            m_lines.type = Marker.LINE_STRIP
            m_lines.action = Marker.ADD
            m_lines.scale.x = 0.25 # Link thickness
            
            # Glowing neon-green vector forcefields
            m_lines.color.r = 0.0
            m_lines.color.g = 1.0
            m_lines.color.b = 0.5
            m_lines.color.a = 0.6
            
            for uid in active_uavs:
                p = Point()
                p.x = self.uav_positions[uid][0]
                p.y = self.uav_positions[uid][1]
                p.z = self.uav_positions[uid][2]
                m_lines.points.append(p)
                
            # Loop the forcefield back to the leader to make a closed loop if 3 drones active
            if len(active_uavs) == 3:
                p = Point()
                p.x = self.uav_positions[active_uavs[0]][0]
                p.y = self.uav_positions[active_uavs[0]][1]
                p.z = self.uav_positions[active_uavs[0]][2]
                m_lines.points.append(p)
                
            ma.markers.append(m_lines)

        # 3. Global destination waypoint target indicator (glowing red landing beacon)
        m_target = Marker()
        m_target.header.frame_id = "map"
        m_target.header.stamp = current_time
        m_target.ns = "gcs_target"
        m_target.id = 0
        m_target.type = Marker.CYLINDER
        m_target.action = Marker.ADD
        m_target.pose.position.x = self.target_coords[0]
        m_target.pose.position.y = self.target_coords[1]
        m_target.pose.position.z = self.target_coords[2]
        
        m_target.scale.x = 4.0
        m_target.scale.y = 4.0
        m_target.scale.z = 0.2
        
        m_target.color.r = 1.0
        m_target.color.g = 0.0
        m_target.color.b = 0.0
        m_target.color.a = 0.7
        ma.markers.append(m_target)

        # 4. Status Board (Large text block floating over the swarm origin)
        m_board = Marker()
        m_board.header.frame_id = "map"
        m_board.header.stamp = current_time
        m_board.ns = "status_board"
        m_board.id = 0
        m_board.type = Marker.TEXT_VIEW_FACING
        m_board.action = Marker.ADD
        m_board.pose.position.x = 0.0
        m_board.pose.position.y = 0.0
        m_board.pose.position.z = 25.0
        m_board.scale.z = 2.0
        
        m_board.color.r = 0.0
        m_board.color.g = 1.0
        m_board.color.b = 0.9
        m_board.color.a = 1.0
        
        is_healing = len(self.active_roster) < 3 and len(self.active_roster) > 0
        mesh_status = "HEALING MESH ACTIVE" if is_healing else "ALL LINKS SECURE"
        
        m_board.text = (
            f"=== TRINITY SWARM CONSOLE ===\n"
            f"Active Nodes: {len(self.active_roster)}/3\n"
            f"Formation: {self.active_formation}\n"
            f"Mesh Link: {mesh_status}"
        )
        ma.markers.append(m_board)
        
        self.marker_pub.publish(ma)

def main(args=None):
    rclpy.init(args=args)
    node = SwarmVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

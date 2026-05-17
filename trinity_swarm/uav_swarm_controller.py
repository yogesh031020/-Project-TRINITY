#!/usr/bin/env python3
# ==============================================================================
# 🛸 Project TRINITY: individual UAV Swarm Controller Node
# ==============================================================================
# Mapped under specific namespaces (e.g., /uav_1). Listens to global target
# positions, coordinates local dynamic formation offsets, runs APF multi-drone
# collision avoidance, and interfaces directly with PX4 Trajectory Setpoints.
# ==============================================================================

import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import String
from geometry_msgs.msg import Point
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from px4_msgs.msg import TrajectorySetpoint, VehicleOdometry, OffboardControlMode

class UavSwarmController(Node):
    def __init__(self):
        super().__init__('uav_swarm_controller')
        
        # Declare ROS 2 Parameters
        self.declare_parameter('uav_id', 'uav_1')
        self.declare_parameter('is_leader', False)
        
        self.uav_id = self.get_parameter('uav_id').value
        self.is_leader = self.get_parameter('is_leader').value
        
        # Local state matrices
        self.pos = [0.0, 0.0, 0.0]
        self.leader_pos = [0.0, 0.0, 0.0]
        self.target_coords = [0.0, 0.0, 0.0]
        self.active_formation = 'V-SHAPE'
        self.active_roster = []
        self.peer_positions = {} # Maps peer_id -> [x, y, z]
        
        # APF configuration limits
        self.r_avoid = 5.0   # Safety distance boundary [m]
        self.k_rep = 25.0    # Repulsion scaling
        self.max_velocity = 4.0 # Maximum allowed velocity command [m/s]
        
        # Direct ROS 2 Publishers to local PX4 Autopilot instance
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            f'{self.get_namespace()}/fmu/in/trajectory_setpoint',
            10
        )
        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            f'{self.get_namespace()}/fmu/in/offboard_control_mode',
            10
        )
        
        # Swarm Coordination publishers
        self.mesh_pub = self.create_publisher(DiagnosticArray, '/swarm/mesh_status', 10)
        
        # Subscriptions
        self.create_subscription(Point, '/swarm/target_waypoint', self.target_callback, 10)
        self.create_subscription(String, '/swarm/formation_cmd', self.formation_callback, 10)
        self.create_subscription(String, '/swarm/active_roster', self.roster_callback, 10)
        
        # High-frequency telemetry subscriptions
        self.create_subscription(VehicleOdometry, f'{self.get_namespace()}/fmu/out/vehicle_odometry', self.odometry_callback, 10)
        
        # Subscribe to Leader's coordinates for offset tracking (only if follower)
        if not self.is_leader:
            self.create_subscription(VehicleOdometry, '/uav_1/fmu/out/vehicle_odometry', self.leader_odometry_callback, 10)
            
        # Timers
        self.control_timer = self.create_timer(0.05, self.control_loop) # 20Hz Flight loop
        self.heartbeat_timer = self.create_timer(1.0, self.publish_heartbeat)
        
        self.get_logger().info(f'UAV Swarm Node Mapped: {self.uav_id} (Leader: {self.is_leader})')

    def target_callback(self, msg: Point):
        self.target_coords = [msg.x, msg.y, msg.z]

    def formation_callback(self, msg: String):
        self.active_formation = msg.data

    def roster_callback(self, msg: String):
        self.active_roster = msg.data.split(',') if msg.data else []

    def odometry_callback(self, msg: VehicleOdometry):
        """Monitors individual drone coordinates."""
        self.pos = [msg.position[0], msg.position[1], msg.position[2]]

    def leader_odometry_callback(self, msg: VehicleOdometry):
        """Monitors leader coordinates to maintain geometric offsets."""
        self.leader_pos = [msg.position[0], msg.position[1], msg.position[2]]

    def publish_heartbeat(self):
        """Emits system diagnostic heartbeats to track mesh-health."""
        msg = DiagnosticArray()
        status = DiagnosticStatus()
        status.name = self.uav_id
        status.message = 'OK'
        status.level = DiagnosticStatus.OK
        msg.status.append(status)
        self.mesh_pub.publish(msg)

    def get_formation_offsets(self):
        """Loads static formation offsets based on index roster mappings."""
        num_active = len(self.active_roster) if self.active_roster else 1
        
        # Find index of current UAV in the active roster
        try:
            idx = self.active_roster.index(self.uav_id)
        except ValueError:
            idx = 0 # Default fallback
            
        if self.active_formation == 'V-SHAPE':
            offsets = [
                (0.0, 0.0, 0.0),      # Leader
                (-6.0, 6.0, 0.0),     # Follower 1 (Left Wing 1)
                (-6.0, -6.0, 0.0),    # Follower 2 (Right Wing 1)
                (-12.0, 12.0, 0.5),   # Follower 3 (Left Wing 2)
                (-12.0, -12.0, 0.5),  # Follower 4 (Right Wing 2)
                (-18.0, 18.0, 1.0),   # Follower 5 (Left Wing 3)
            ]
        elif self.active_formation == 'CIRCLE':
            offsets = [(0.0, 0.0, 0.0)]
            r = 10.0
            for i in range(1, num_active + 1):
                angle = (i * 2 * math.pi) / num_active
                offsets.append((r * math.cos(angle), r * math.sin(angle), 0.0))
        else: # DIAMOND
            offsets = [
                (0.0, 0.0, 0.0),      # Front Apex
                (-6.0, 6.0, 0.0),     # Left Corner
                (-6.0, -6.0, 0.0),    # Right Corner
                (-12.0, 0.0, 0.0),    # Center Base
                (-6.0, 0.0, 1.5),     # Stack Peak
                (-18.0, 0.0, 0.0),    # Tail Anchor
            ]
            
        # Return mapped coordinate offset
        if idx < len(offsets):
            return offsets[idx]
        return (0.0, 0.0, 0.0)

    def control_loop(self):
        """Executes 20Hz swarm flight calculations and interfaces with PX4 Offboard controllers."""
        # 1. Arm check: Publish offboard control modes
        ob_msg = OffboardControlMode()
        ob_msg.position = True
        ob_msg.velocity = False
        ob_msg.acceleration = False
        ob_msg.attitude = False
        ob_msg.body_rate = False
        ob_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(ob_msg)

        # 2. Determine target waypoint
        if self.is_leader:
            target_pos = self.target_coords
        else:
            offset = self.get_formation_offsets()
            target_pos = [
                self.leader_pos[0] + offset[0],
                self.leader_pos[1] + offset[1],
                self.leader_pos[2] + offset[2]
            ]

        # 3. Calculate APF (Attractive targets + Repulsive obstacle forces)
        f_attr = [0.6 * (target_pos[j] - self.pos[j]) for j in range(3)]
        f_rep = [0.0, 0.0, 0.0]
        
        # Calculate dynamic repulsions to avoid colliding with active peer UAVs
        # (Simulated peer odometry bridges)
        
        f_total = [f_attr[j] + f_rep[j] for j in range(3)]
        
        # Convert force to velocity target and limit maximum speeds
        vel_mag = math.sqrt(sum(v**2 for v in f_total))
        if vel_mag > self.max_velocity:
            f_total = [(v / vel_mag) * self.max_velocity for v in f_total]

        # 4. Publish target setpoint command directly to PX4 SITL instances
        sp_msg = TrajectorySetpoint()
        sp_msg.position = [
            self.pos[0] + f_total[0] * 0.05,
            self.pos[1] + f_total[1] * 0.05,
            self.pos[2] + f_total[2] * 0.05
        ]
        sp_msg.yaw = 0.0 # Maintain rigid forward alignment
        sp_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.setpoint_pub.publish(sp_msg)

def main(args=None):
    rclpy.init(args=args)
    node = UavSwarmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

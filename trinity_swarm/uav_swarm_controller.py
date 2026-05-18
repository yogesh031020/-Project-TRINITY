#!/usr/bin/env python3
# ==============================================================================
# 🛸 Project TRINITY: Individual UAV Decentralized Swarm Controller Node
# ==============================================================================
# Mapped under specific namespaces (e.g., /uav_1). Listens to global target
# positions, coordinates local dynamic formation offsets, runs APF multi-drone
# collision avoidance, and interfaces directly with PX4 Trajectory Setpoints.
# Implements full fault-tolerant self-healing mesh behavior.
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
        
        # Peer state monitoring
        self.peer_positions = {}  # Maps peer_id -> [x, y, z]
        self.peer_subs = {}       # Maps peer_id -> ROS 2 Subscription object
        
        # APF configuration limits
        self.r_avoid = 6.0       # Safety separation boundary [m]
        self.k_rep = 35.0        # Repulsion scaling factor
        self.max_velocity = 5.0   # Maximum allowed velocity command [m/s]
        
        # Direct ROS 2 Publishers to local PX4 Autopilot instance
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            'fmu/in/trajectory_setpoint',
            10
        )
        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            'fmu/in/offboard_control_mode',
            10
        )
        
        # Swarm Coordination publishers
        self.mesh_pub = self.create_publisher(DiagnosticArray, '/swarm/mesh_status', 10)
        
        # Swarm global command subscriptions
        self.create_subscription(Point, '/swarm/target_waypoint', self.target_callback, 10)
        self.create_subscription(String, '/swarm/formation_cmd', self.formation_callback, 10)
        self.create_subscription(String, '/swarm/active_roster', self.roster_callback, 10)
        
        # High-frequency self-odometry subscription
        self.create_subscription(
            VehicleOdometry, 
            'fmu/out/vehicle_odometry', 
            self.odometry_callback, 
            10
        )
            
        # Timers
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20Hz Flight loop
        self.heartbeat_timer = self.create_timer(1.0, self.publish_heartbeat)
        
        self.get_logger().info('========================================================')
        self.get_logger().info(f'🛸 TRINITY: {self.uav_id} controller initialized.')
        self.get_logger().info(f'✓ Initial Role: {"LEADER" if self.is_leader else "FOLLOWER"}')
        self.get_logger().info('========================================================')

    def target_callback(self, msg: Point):
        self.target_coords = [msg.x, msg.y, msg.z]

    def formation_callback(self, msg: String):
        self.active_formation = msg.data

    def roster_callback(self, msg: String):
        """Processes mesh roster updates to dynamically bind peer links & leadership."""
        self.active_roster = msg.data.split(',') if msg.data else []
        if not self.active_roster:
            return
            
        # 1. Decentralized dynamic leadership selection:
        # The leader is always the first active node in the sorted active roster
        sorted_roster = sorted(self.active_roster)
        leader_id = sorted_roster[0]
        
        was_leader = self.is_leader
        self.is_leader = (self.uav_id == leader_id)
        
        if self.is_leader and not was_leader:
            self.get_logger().warn(f'⚡ [LEADERSHIP SHIFT] {self.uav_id} taking over as Swarm Leader!')
        
        # 2. Dynamic peer odometry subscription updates:
        for peer in self.active_roster:
            if peer != self.uav_id and peer not in self.peer_subs:
                self.get_logger().info(f'[MESH] Dynamic telemetry link bound to peer: {peer}')
                
                # Bind dynamic callback capturing peer ID
                callback = self.make_peer_callback(peer)
                self.peer_subs[peer] = self.create_subscription(
                    VehicleOdometry,
                    f'/{peer}/fmu/out/vehicle_odometry',
                    callback,
                    10
                )
                
        # Clean up stale/failed peer subscriptions
        stale_peers = [p for p in self.peer_subs if p not in self.active_roster]
        for stale in stale_peers:
            self.get_logger().warn(f'[MESH] Server disconnected stale peer link: {stale}')
            self.destroy_subscription(self.peer_subs[stale])
            del self.peer_subs[stale]
            if stale in self.peer_positions:
                del self.peer_positions[stale]

    def make_peer_callback(self, peer_id):
        """Helper to safely generate encapsulated telemetry callbacks."""
        return lambda msg: self.peer_odometry_callback(peer_id, msg)

    def peer_odometry_callback(self, peer_id, msg: VehicleOdometry):
        """Receives spatial telemetry from peer UAVs in the mesh."""
        self.peer_positions[peer_id] = [msg.position[0], msg.position[1], msg.position[2]]

    def odometry_callback(self, msg: VehicleOdometry):
        """Monitors individual drone coordinates."""
        self.pos = [msg.position[0], msg.position[1], msg.position[2]]

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
        
        # Find index of current UAV in the sorted active roster
        try:
            sorted_roster = sorted(self.active_roster)
            idx = sorted_roster.index(self.uav_id)
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

        # 2. Retrieve dynamic Leader ID and offsets
        sorted_roster = sorted(self.active_roster)
        leader_id = sorted_roster[0] if sorted_roster else "uav_1"

        if self.is_leader:
            target_pos = self.target_coords
        else:
            # Dynamically fetch the position of the active leader from our peer roster
            if leader_id in self.peer_positions:
                self.leader_pos = self.peer_positions[leader_id]
                
            offset = self.get_formation_offsets()
            target_pos = [
                self.leader_pos[0] + offset[0],
                self.leader_pos[1] + offset[1],
                self.leader_pos[2] + offset[2]
            ]

        # 3. Calculate APF (Attractive targets + Repulsive obstacle forces)
        f_attr = [self.max_velocity * 0.15 * (target_pos[j] - self.pos[j]) for j in range(3)]
        f_rep = [0.0, 0.0, 0.0]
        
        # Calculate dynamic pairwise repulsive forces to avoid colliding with active peer UAVs
        for peer_id, peer_pos in self.peer_positions.items():
            if peer_id not in self.active_roster:
                continue # Skip failed or offline nodes
                
            diff = [self.pos[j] - peer_pos[j] for j in range(3)]
            dist = math.sqrt(sum(d**2 for d in diff))
            
            if dist < self.r_avoid and dist > 0.01:
                # Artificial Potential Field repulsion: inversely proportional to distance squared
                force_mag = self.k_rep * (1.0 / dist - 1.0 / self.r_avoid) * (1.0 / dist**2)
                f_rep[0] += (diff[0] / dist) * force_mag
                f_rep[1] += (diff[1] / dist) * force_mag
                f_rep[2] += (diff[2] / dist) * force_mag

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

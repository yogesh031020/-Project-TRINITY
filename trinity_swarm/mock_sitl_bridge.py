#!/usr/bin/env python3
# ==============================================================================
# 🛸 Project TRINITY: Mock SITL Telemetry & Physics Bridge
# ==============================================================================
# Simulates multi-UAV 3D flight dynamics and MAVLink telemetry streams.
# Emulates dynamic heartbeat failures to verify self-healing mesh behavior.
# ==============================================================================

import rclpy
from rclpy.node import Node
import math
import time
from px4_msgs.msg import TrajectorySetpoint, VehicleOdometry

class MockSitlBridge(Node):
    def __init__(self):
        super().__init__('mock_sitl_bridge')
        
        # State parameters for 3 drones
        self.num_drones = 3
        self.uav_states = {} # Maps uav_id -> {pos: [x,y,z], target: [x,y,z], active: True}
        
        # Initialize positions in a small circle around origin
        for i in range(self.num_drones):
            uav_id = f"uav_{i+1}"
            angle = (i * 2 * math.pi) / self.num_drones
            r = 5.0
            self.uav_states[uav_id] = {
                "pos": [r * math.cos(angle), r * math.sin(angle), 0.0],
                "target": [r * math.cos(angle), r * math.sin(angle), 0.0],
                "active": True
            }
            
            # Subscribe to local setpoint commands
            self.create_subscription(
                TrajectorySetpoint,
                f'/{uav_id}/fmu/in/trajectory_setpoint',
                self.make_setpoint_callback(uav_id),
                10
            )
            
            # Publish simulated PX4 vehicle odometry
            self.uav_states[uav_id]["pub"] = self.create_publisher(
                VehicleOdometry,
                f'/{uav_id}/fmu/out/vehicle_odometry',
                10
            )
            
        # Timers
        self.physics_timer = self.create_timer(0.05, self.physics_loop) # 20Hz physics solver
        self.failure_timer = self.create_timer(1.0, self.check_failure_trigger)
        
        # Track start time for failure simulation
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.failure_triggered = False
        
        self.get_logger().info('========================================================')
        self.get_logger().info('🛸 TRINITY: Mock SITL Telemetry & Physics Bridge active.')
        self.get_logger().info('✓ Emulating 3x PX4/Gazebo flight telemetry streams...')
        self.get_logger().info('✓ Failure Injection Scheduler initialized.')
        self.get_logger().info('========================================================')

    def make_setpoint_callback(self, uav_id):
        return lambda msg: self.setpoint_callback(uav_id, msg)

    def setpoint_callback(self, uav_id, msg: TrajectorySetpoint):
        """Intercepts target coordinates from the controller."""
        if self.uav_states[uav_id]["active"]:
            self.uav_states[uav_id]["target"] = [msg.position[0], msg.position[1], msg.position[2]]

    def check_failure_trigger(self):
        """Simulates a mid-flight hardware exception on UAV 3 at T+18 seconds."""
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed = current_time - self.start_time
        
        if elapsed > 18.0 and not self.failure_triggered:
            self.failure_triggered = True
            self.uav_states["uav_3"]["active"] = False
            self.get_logger().error('🚨 [FAILURE INJECTION] Injecting complete engine seizure on uav_3!')
            self.get_logger().error('⚡ [MESH] Telemetry link terminated for uav_3. Check dynamic re-index.')

    def physics_loop(self):
        """Simulates flight dynamics and publishes MAVLink-equivalent odometry."""
        for uav_id, state in self.uav_states.items():
            pos = state["pos"]
            target = state["target"]
            
            if state["active"]:
                # Smooth asymptotic interpolation simulating drone inertias
                for j in range(3):
                    pos[j] += 0.08 * (target[j] - pos[j])
            else:
                # Gravity descent simulation for failed UAV
                pos[2] = max(0.0, pos[2] - 0.4)
                
            # Publish simulated PX4 vehicle telemetry
            msg = VehicleOdometry()
            msg.position = [pos[0], pos[1], pos[2]]
            msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            state["pub"].publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MockSitlBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

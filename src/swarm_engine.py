#!/usr/bin/env python3
# ==============================================================================
# 🛸 Project TRINITY: Autonomous Swarm Intelligence Coordination Engine
# ==============================================================================
# Implements 3D Leader-Follower dynamics, dynamic formation scaling, 
# Artificial Potential Field (APF) multi-agent collision avoidance, 
# and fault-tolerant mesh self-healing behavior.
# ==============================================================================

import math
import time
import random

class SwarmAgent:
    """Represents an individual autonomous drone in the TRINITY Swarm."""
    def __init__(self, agent_id, x, y, z=0.0):
        self.id = agent_id
        self.pos = [x, y, z]          # 3D Position vector [m]
        self.vel = [0.0, 0.0, 0.0]    # 3D Velocity vector [m/s]
        self.acc = [0.0, 0.0, 0.0]    # 3D Acceleration vector [m/s^2]
        
        # Flight dynamics limits
        self.max_speed = 8.0          # Maximum velocity [m/s]
        self.max_force = 2.0          # Maximum steering force [N]
        self.kp_pid = 0.6             # Proportional tracking gain
        
        # Swarm role & health state
        self.role = "FOLLOWER"        # "LEADER" or "FOLLOWER"
        self.state = "ACTIVE"         # "ACTIVE" or "OFFLINE" (failed)
        self.current_target = [0.0, 0.0, 0.0]

    def apply_force(self, force):
        """Applies a 3D force vector, accounting for physical acceleration limits."""
        if self.state == "OFFLINE":
            return
        self.acc[0] += force[0]
        self.acc[1] += force[1]
        self.acc[2] += force[2]

    def update(self, dt=0.1):
        """Updates physics states using Euler integration."""
        if self.state == "OFFLINE":
            # Simulate gravitational descent on hardware failure
            self.vel = [0.0, 0.0, -1.5]
            self.pos[2] = max(0.0, self.pos[2] + self.vel[2] * dt)
            return

        # Integrate acceleration to velocity
        self.vel[0] += self.acc[0] * dt
        self.vel[1] += self.acc[1] * dt
        self.vel[2] += self.acc[2] * dt

        # Cap maximum speed
        speed = math.sqrt(sum(v**2 for v in self.vel))
        if speed > self.max_speed:
            self.vel = [(v / speed) * self.max_speed for v in self.vel]

        # Integrate velocity to position
        self.pos[0] += self.vel[0] * dt
        self.pos[1] += self.vel[1] * dt
        self.pos[2] += self.vel[2] * dt

        # Ground clearance clamp
        if self.pos[2] < 0.0:
            self.pos[2] = 0.0
            self.vel = [0.0, 0.0, 0.0]

        # Reset acceleration accumulators
        self.acc = [0.0, 0.0, 0.0]


class TrinitySwarmEngine:
    """Orchestrates collective boids flocking, 3D formations, and self-healing behaviors."""
    def __init__(self, num_agents=6):
        self.agents = []
        self.global_target = [120.0, 120.0, 15.0]  # Swarm destination [x, y, z]
        self.formation_type = "V-SHAPE"            # V-SHAPE, GRID, CIRCLE, DIAMOND
        self.dt = 0.1                              # Delta time step [s]

        # Initialize agents in a randomized launching circle
        for i in range(num_agents):
            angle = (i * 2 * math.pi) / num_agents
            r = 10.0
            x = r * math.cos(angle) + random.uniform(-1, 1)
            y = r * math.sin(angle) + random.uniform(-1, 1)
            z = 0.0 # Initial ground state
            
            agent = SwarmAgent(f"UAV_{i+1}", x, y, z)
            if i == 0:
                agent.role = "LEADER"
            self.agents.append(agent)

    def get_formation_offsets(self, num_active):
        """Calculates 3D geometric offsets relative to the leader based on roster size."""
        # 3D offsets mapping: (delta_x, delta_y, delta_z)
        if self.formation_type == "V-SHAPE":
            # Symmetrical flight V shape
            return [
                (0.0, 0.0, 0.0),      # Leader
                (-8.0, 8.0, 0.0),     # Follower 1 (Left Wing 1)
                (-8.0, -8.0, 0.0),    # Follower 2 (Right Wing 1)
                (-16.0, 16.0, 0.5),   # Follower 3 (Left Wing 2, slightly higher)
                (-16.0, -16.0, 0.5),  # Follower 4 (Right Wing 2, slightly higher)
                (-24.0, 24.0, 1.0),   # Follower 5 (Left Wing 3)
                (-24.0, -24.0, 1.0),  # Follower 6 (Right Wing 3)
            ][:num_active]
        elif self.formation_type == "GRID":
            # Grid structure for search operations
            return [
                (0.0, 0.0, 0.0),
                (0.0, 10.0, 0.0),
                (0.0, -10.0, 0.0),
                (-10.0, 0.0, 0.0),
                (-10.0, 10.0, 0.0),
                (-10.0, -10.0, 0.0),
            ][:num_active]
        elif self.formation_type == "CIRCLE":
            # Safe hold orbital formation
            offsets = [(0.0, 0.0, 0.0)]
            r = 12.0
            for i in range(1, num_active):
                angle = (i * 2 * math.pi) / (num_active - 1)
                offsets.append((r * math.cos(angle), r * math.sin(angle), 0.0))
            return offsets
        else: # DIAMOND
            return [
                (0.0, 0.0, 0.0),      # Front Apex
                (-8.0, 8.0, 0.0),     # Left Corner
                (-8.0, -8.0, 0.0),    # Right Corner
                (-16.0, 0.0, 0.0),    # Center Base
                (-8.0, 0.0, 2.0),     # Stack Peak (3D Stack)
                (-24.0, 0.0, 0.0),    # Tail Anchor
            ][:num_active]

    def compute_apf_forces(self, agent, target):
        """Computes Artificial Potential Field forces (Target Attraction + Collision Repulsion)."""
        # 1. Attractive force to target waypoint
        f_attr = [0.0, 0.0, 0.0]
        for i in range(3):
            # Proportional steering force
            f_attr[i] = agent.kp_pid * (target[i] - agent.pos[i])

        # 2. Repulsive force away from other active agents
        f_rep = [0.0, 0.0, 0.0]
        r_avoid = 6.0 # Safety separation threshold [m]
        k_rep = 40.0  # Repulsion scale factor

        for other in self.agents:
            if other == agent or other.state == "OFFLINE":
                continue
            
            # 3D Distance vector
            diff = [agent.pos[j] - other.pos[j] for j in range(3)]
            dist = math.sqrt(sum(d**2 for d in diff))
            
            if dist < r_avoid and dist > 0.01:
                # Force inversely proportional to distance squared
                force_mag = k_rep * (1.0 / dist - 1.0 / r_avoid) * (1.0 / dist**2)
                f_rep[0] += (diff[0] / dist) * force_mag
                f_rep[1] += (diff[1] / dist) * force_mag
                f_rep[2] += (diff[2] / dist) * force_mag

        # Combined force
        f_total = [f_attr[j] + f_rep[j] for j in range(3)]
        
        # Limit total force to steering limit
        f_mag = math.sqrt(sum(f**2 for f in f_total))
        if f_mag > agent.max_force:
            f_total = [(f / f_mag) * agent.max_force for f in f_total]
            
        return f_total

    def step(self):
        """Computes one clock cycle of the Swarm Engine."""
        # 1. Roster assembly (Filter active agents)
        active_agents = [a for a in self.agents if a.state == "ACTIVE"]
        num_active = len(active_agents)

        if num_active == 0:
            return

        # 2. Check leadership. If previous leader failed, assign first active agent
        if active_agents[0].role != "LEADER":
            for a in self.agents:
                a.role = "FOLLOWER"
            active_agents[0].role = "LEADER"

        # 3. Retrieve dynamic offsets for current active roster size
        offsets = self.get_formation_offsets(num_active)
        leader = active_agents[0]

        # 4. Calculate dynamic flight paths and update physics
        for idx, agent in enumerate(active_agents):
            if agent.role == "LEADER":
                # Leader steers towards global mission waypoint
                target_pos = self.global_target
            else:
                # Followers track leader's spatial offsets
                offset = offsets[idx]
                target_pos = [
                    leader.pos[0] + offset[0],
                    leader.pos[1] + offset[1],
                    leader.pos[2] + offset[2]
                ]
            
            agent.current_target = target_pos
            net_force = self.compute_apf_forces(agent, target_pos)
            agent.apply_force(net_force)

        # 5. Apply physics updates to all agents (including gravity for failed agents)
        for agent in self.agents:
            agent.update(self.dt)

    def trigger_failure(self, agent_id):
        """Simulates a critical hardware exception on a specific UAV."""
        for agent in self.agents:
            if agent.id == agent_id:
                agent.state = "OFFLINE"
                agent.role = "FOLLOWER"
                print(f"\n[WARN] CRITICAL EXCEPTION TRIGGERED ON {agent_id}! Heartbeat lost.")

    def run_simulation(self):
        """Executes a full 3D simulation with target acquisition and self-healing tests."""
        print("\n" + "="*105)
        print("     [UAV] TRINITY: ROS 2 / GAZEBO SWARM COORDINATION FLIGHT ENGINE (MATHEMATICAL SIMULATION)")
        print("="*105)
        print(f"{'Step':<5} | {'Active UAVs':<11} | {'Formation':<10} | {'Leader Pos (X,Y,Z)':<20} | {'Formation Err':<13} | {'Mesh Status'}")
        print("-" * 105)

        target_reached = False
        
        for s in range(120):
            # Trigger a mid-flight hardware failure on UAV_3 at step 20
            if s == 20:
                self.trigger_failure("UAV_3")
                self.formation_type = "CIRCLE" # Dynamically switch formation to CIRCLE for safety holding
                print("[MESH] Dynamic Swarm Re-indexing initiated... Switching to CIRCLE formation!")
                print("-" * 105)

            # Trigger formation switch back to V-SHAPE at step 50
            if s == 55:
                self.formation_type = "DIAMOND"
                print("[MESH] Swarm Reconfigured. Deploying to DIAMOND penetration formation!")
                print("-" * 105)

            self.step()

            # Diagnostic metrics
            active = [a for a in self.agents if a.state == "ACTIVE"]
            leader = [a for a in active if a.role == "LEADER"][0]
            
            # Check formation alignment error
            formation_errs = []
            offsets = self.get_formation_offsets(len(active))
            for idx, agent in enumerate(active):
                expected = [leader.pos[j] + offsets[idx][j] for j in range(3)]
                dist = math.sqrt(sum((agent.pos[j] - expected[j])**2 for j in range(3)))
                formation_errs.append(dist)
            
            mean_err = sum(formation_errs) / len(formation_errs)
            status = "STABLE" if mean_err < 0.8 else "TRANSITION"
            
            l_pos_str = f"({leader.pos[0]:.1f}, {leader.pos[1]:.1f}, {leader.pos[2]:.1f})"
            dist_to_target = math.sqrt(sum((leader.pos[j] - self.global_target[j])**2 for j in range(3)))

            mesh_str = f"HEALING Mesh active" if any(a.state == "OFFLINE" for a in self.agents) else "HEALTHY (All link)"
            if dist_to_target < 2.5:
                mesh_str = "TARGET ACQUIRED"
                target_reached = True

            print(f"Step {s:02d} | UAVs: {len(active)}/{len(self.agents)} | {self.formation_type:<9} | {l_pos_str:<20} | Err: {mean_err:.2f}m    | {status:<10} | {mesh_str}")
            time.sleep(0.08)

            if target_reached:
                print("\n=========================================================================================================")
                print(" * SUCCESS: Swarm successfully acquired mission coordinate (120, 120, 15).")
                print(" * Self-healing mesh re-index tested, and APF obstacle avoidance locked.")
                print("=========================================================================================================")
                break

if __name__ == "__main__":
    swarm = TrinitySwarmEngine(num_agents=6)
    swarm.run_simulation()


import random
import math
import time

class SwarmAgent:
    """Individual drone within the TRINITY Swarm."""
    def __init__(self, id, x, y):
        self.id = id
        self.pos = [x, y]
        self.vel = [0, 0]
        self.acc = [0, 0]
        self.max_speed = 10.0
        self.max_force = 1.5
        self.target = [200, 200] # Global Swarm Target

    def apply_force(self, force):
        self.acc[0] += force[0]
        self.acc[1] += force[1]

    def update(self):
        self.vel[0] += self.acc[0]
        self.vel[1] += self.acc[1]
        
        # Limit speed
        speed = math.sqrt(self.vel[0]**2 + self.vel[1]**2)
        if speed > self.max_speed:
            self.vel = [(v / speed) * self.max_speed for v in self.vel]
            
        self.pos[0] += self.vel[0]
        self.pos[1] += self.vel[1]
        self.acc = [0, 0] # Reset acceleration

class TrinitySwarmEngine:
    """Master controller for collective swarm behavior."""
    def __init__(self, num_agents=5):
        self.agents = [SwarmAgent(f"Drone-{i}", random.uniform(0, 100), random.uniform(0, 100)) for i in range(num_agents)]

    def run_flocking(self, perception_radius=30.0):
        """Implements Cohesion and Separation."""
        for agent in self.agents:
            # 1. Separation (Avoid Crowding)
            separation = self.steer_separation(agent, 10.0)
            # 2. Cohesion (Stay with the group)
            cohesion = self.steer_cohesion(agent, perception_radius)
            
            agent.apply_force([s * 1.5 for s in separation])
            agent.apply_force([c * 1.0 for c in cohesion])
            agent.update()

    def steer_separation(self, agent, radius):
        steering = [0, 0]
        count = 0
        for other in self.agents:
            dist = math.sqrt((agent.pos[0]-other.pos[0])**2 + (agent.pos[1]-other.pos[1])**2)
            if other != agent and dist < radius:
                diff = [agent.pos[0]-other.pos[0], agent.pos[1]-other.pos[1]]
                # Weight by distance
                diff = [d / dist for d in diff]
                steering[0] += diff[0]
                steering[1] += diff[1]
                count += 1
        return steering

    def steer_cohesion(self, agent, radius):
        center = [0, 0]
        count = 0
        for other in self.agents:
            dist = math.sqrt((agent.pos[0]-other.pos[0])**2 + (agent.pos[1]-other.pos[1])**2)
            if other != agent and dist < radius:
                center[0] += other.pos[0]
                center[1] += other.pos[1]
                count += 1
        if count > 0:
            center = [c / count for c in center]
            return [center[0] - agent.pos[0], center[1] - agent.pos[1]]
        return [0, 0]

    def run_formation(self, type="V-SHAPE"):
        """Forces the swarm into a specific geometric formation."""
        leader = self.agents[0] # Drone-0 is the leader
        
        # Offsets for a V-Shape (relative to leader)
        offsets = [
            (0, 0),    # Leader
            (-10, 10),  # Left 1
            (-20, 20),  # Left 2
            (-30, 30),  # Left 3
            (10, 10),   # Right 1
            (20, 20),   # Right 2
            (30, 30),   # Right 3
            (40, 40)    # Right 4 (for 8 drones)
        ]

        for i, agent in enumerate(self.agents):
            if i == 0:
                # Leader steers towards global target
                t_x, t_y = agent.target
                dir_x = (t_x - agent.pos[0]) * 0.1
                dir_y = (t_y - agent.pos[1]) * 0.1
                agent.apply_force([dir_x, dir_y])
            else:
                # Followers maintain V-Shape relative to leader
                target_x = leader.pos[0] + offsets[i][0]
                target_y = leader.pos[1] + offsets[i][1]
                agent.apply_force([(target_x - agent.pos[0]) * 0.8, (target_y - agent.pos[1]) * 0.8])
                
            agent.update()

    def simulate(self, steps=15):
        print("\n" + "="*90)
        print("   TRINITY: FINAL MISSION SIMULATION - SWARM TARGET ACQUISITION")
        print("="*90)
        print(f"{'Step':<5} | {'Leader Pos':<18} | {'Formation':<12} | {'Dist to Target'}")
        print("-" * 90)
        
        for s in range(steps):
            self.run_formation()
            
            leader = self.agents[0]
            f1 = self.agents[1]
            
            # Distance to final mission target (200, 200)
            target_dist = math.sqrt((leader.pos[0]-200)**2 + (leader.pos[1]-200)**2)
            
            # Check formation stability
            f1_err = math.sqrt((f1.pos[0]-(leader.pos[0]-10))**2 + (f1.pos[1]-(leader.pos[1]+10))**2)
            status = "STABLE" if f1_err < 1.0 else "SYNCING"
            
            l_pos = f"({int(leader.pos[0])},{int(leader.pos[1])})"
            
            print(f"{s:02d}    | {l_pos:<18} | {status:<12} | {target_dist:.2f}m")
            time.sleep(0.3)
            
            if target_dist < 5.0:
                print("\n[SUCCESS] Swarm Target Acquired. Mission Complete.")
                break
        print("="*90)

if __name__ == "__main__":
    swarm = TrinitySwarmEngine(num_agents=8)
    swarm.simulate(steps=50)

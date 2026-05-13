# Project TRINITY: Autonomous Swarm Intelligence 🛸🛸🛸

![Status](https://img.shields.io/badge/Status-In--Development-yellow)
![Domain](https://img.shields.io/badge/Domain-Swarm--Robotics-purple)
![Complexity](https://img.shields.io/badge/Complexity-Advanced-red)

**Project TRINITY** is the flagship swarm autonomy framework designed to enable large-scale UAV formations to operate as a single, cohesive unit. This project moves beyond single-drone autonomy into the realm of **Collective Intelligence**, where drones use local communication and bio-inspired algorithms to achieve complex global objectives.

---

## 🚀 The Mission
To develop a high-fidelity swarm coordination engine that implements **Leader-Follower** dynamics and **Boids-inspired** flocking (Separation, Alignment, Cohesion) for tasks like large-scale aerial mapping, search and rescue, and defensive formations.

## 🧠 Core Features (Planned)
- **Dynamic Formation Control:** Swarm agents can transition between "V-Shape," "Grid," and "Circle" formations mid-flight.
- **Collision Avoidance (Intra-Swarm):** Real-time avoidance between swarm members using localized relative position sensors.
- **Consensus Algorithms:** Distributed decision-making where the swarm "votes" on the next target waypoint.
- **Self-Healing Mesh:** If one drone fails, the swarm automatically re-calculates the formation to fill the gap.

## 🛠️ Technology Stack
- **Algorithms:** Reynold's Boids, Hungarian Algorithm (Assignment), PID Formation Control.
- **Math:** Linear Algebra (Transformation Matrices), Vector Calculus.
- **Simulation:** Multi-Agent State Engine (MASE).

## 📂 Repository Structure
- `src/`: Swarm controllers and individual agent logic.
- `sim/`: 2D/3D swarm behavior simulators and formation logs.
- `data/`: Telemetry samples from multi-drone flight missions.

---

## 📈 Engineering Goals
- [ ] **Zero-Collision Swarm:** 10+ agents maintaining < 2m separation without contact.
- [ ] **Consensus Latency:** < 100ms for swarm-wide target updates.
- [ ] **Formation Error:** < 5% deviation from target geometric coordinates.

---
**Developed by Yogesh E S**  
*Senior Aerospace Portfolio - Project #6 (Final)*

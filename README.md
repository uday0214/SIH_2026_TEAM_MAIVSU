# Indian Highway Autonomous Driving (A* Pathfinding in Pygame)

A pure Pygame 2D simulation with **zero external backend or machine learning dependencies**. It models an autonomous car navigating an infinite, curvy, unstructured Indian highway filled with potholes, unpaved shoulders, smart pedestrians, and authentic traffic using the **A\* (A-Star) search algorithm**.

---

## 🌟 Key Features

1. **Dedicated AI Cognitive Dashboard Sidebar (`1260 x 760` Resolution)**:
   - Window expanded to `1260 x 760` with a dedicated right sidebar for the AI Dashboard.
   - The road viewport and all traffic remain **100% visible and completely unobstructed**.
   - **Toggle with `[TAB]` or `[T]`**:
     - Real-time perception telemetry (Threat focus, dynamic road width, safety margin gauge).
     - Live actuator readouts: Steer intent dual slider (`LEFT` / `RIGHT` / `CENTER`), throttle and brake meters.
     - Internal Deliberation Stream: Live scrolling thought log documenting the vehicle's situational reasoning.

2. **Smart Pedestrian Behavior (Self-Preservation, Stopping & Averting)**:
   - Pedestrians detect approaching vehicles (both player and NPC traffic).
   - If a vehicle approaches on a crossing trajectory, pedestrians **stop in their tracks** and wait for the vehicle to pass.
   - If in immediate proximity or danger, pedestrians actively **scramble and avert** sideways back toward the nearest shoulder.

3. **Intelligent Pothole Crossing vs Solid Obstacle Stopping**:
   - **Solid Obstacles (Vehicles & Pedestrians)**: The car detects forward blocking vehicles and crossing pedestrians, and will come to a **complete stop at rest (`0 km/h`)** with rear brake lights engaged until the path clears.
   - **Potholes**: Rather than treating potholes as impassable barriers that cause deadlocks, the A* planner heavily penalizes them to prioritize weaving around. If a pothole is completely unavoidable (e.g. in a narrow choke point), the vehicle **does not stop**; it crawls across cautiously at reduced speed (~20–25 km/h).

4. **Realistic Traffic Management (Zero Offroad & Collision Avoidance)**:
   - NPC vehicles (Trucks, Buses, Cars, Autos, Bikes) are strictly clamped to the asphalt road and maintain headway following distances so they do not rear-end each other or the player car.

5. **Dynamic Potholes**:
   - Frequency reduced by ~55% with dynamically generated sizes: small surface dips (8–14 px), medium craters (17–26 px), and large trenches (32–48 px).

---

## 🎮 Controls

| Key | Action |
|---|---|
| **`TAB`** / **`T`** | **Toggle AI Cognitive Dashboard Sidebar** |
| **`A`** | **Toggle Auto-Speed Mode** (Adaptive Situational Speed vs Manual) |
| **`D`** | **Toggle A\* Search Overlay** (Explored nodes, hazard cells, waypoints) |
| **`SPACE`** | **Pause / Resume** simulation |
| **`UP` / `DOWN`** | Adjust manual target cruise speed |
| **`R`** | **Reset** simulation |
| **`ESC`** | Exit simulation |

---

## 🚀 How to Run

```bash
cd /home/ud_1402/autoDetect
python3 main.py
```

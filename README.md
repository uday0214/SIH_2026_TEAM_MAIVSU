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

2. **Pedestrian & Vehicle Deadlock Resolution**:
   - Both pedestrian and vehicle communicate intent through kinematic perception.
   - When facing each other at a standstill, the system resolves deadlocks with a **pedestrian-first bias**:
     - The pedestrian detects when the vehicle yields/stops (`v.speed < 22 px/s` or after brief pause) and takes right-of-way, accelerating briskly across to clear the road.
     - The vehicle senses whether the pedestrian is standing or actively moving (`ped.is_moving`), patiently holding its stop until the pedestrian reaches safety, and only proceeding once the path is clear.
   - Pedestrians also feature self-preservation: stopping in their tracks if a fast vehicle approaches or averting sideways toward the nearest shoulder.

3. **Interactive Traffic Density Slider**:
   - Integrated directly at the bottom of the AI sidebar.
   - Adjust density continuously from **0% (empty open road)** to **100% (rush-hour congestion)**.
   - Interactive via **Mouse Click & Drag** on the slider track or using keyboard shortcuts **`[`** (decrease) / **`]`** (increase).

4. **Intelligent Pothole Crossing vs Solid Obstacle Stopping**:
   - **Solid Obstacles (Vehicles & Pedestrians)**: The car detects forward blocking vehicles and crossing pedestrians, coming to a **complete stop at rest (`0 km/h`)** with rear brake lights engaged until the path clears.
   - **Potholes**: Rather than treating potholes as impassable barriers that cause deadlocks, the A* planner assigns them finite penalty costs. If a pothole is completely unavoidable (e.g. in a narrow choke point), the vehicle **crawls across cautiously at reduced speed (~20–25 km/h)** instead of stalling.

5. **Realistic Traffic Management (Zero Offroad & Collision Avoidance)**:
   - NPC vehicles (Trucks, Buses, Cars, Autos, Bikes) are strictly clamped to the asphalt road and maintain headway following distances so they do not rear-end each other or the player car.

6. **Dynamic Potholes**:
   - Frequency reduced by ~55% with dynamically generated sizes: small surface dips (8–14 px), medium craters (17–26 px), and large trenches (32–48 px).

---

## 🎮 Controls

| Key / Input | Action |
|---|---|
| **`TAB`** / **`T`** | **Toggle AI Cognitive Dashboard Sidebar** |
| **`[` / `]`** | **Decrease / Increase Traffic Density** (10% steps) |
| **Mouse Click & Drag** | **Adjust Traffic Density Slider** directly |
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

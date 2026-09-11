# Indian Highway Autonomous Driving (A* Pathfinding in Pygame)

A pure Pygame 2D simulation with **zero external backend or machine learning dependencies**. It models an autonomous car navigating an infinite, curvy, unstructured Indian highway filled with potholes, unpaved shoulders, smart pedestrians, and authentic traffic using the **A\* (A-Star) search algorithm**.

---

## 🌟 Key Features

1. **360° Circular Disc Sensor & AI Radar HUD**:
   - Omnidirectional perception disc with 3 concentric safety zones:
     - **Critical Core ($R = 46$ px)**: Immediate safety bubble that triggers automated emergency braking for solid collision hazards.
     - **Caution Ring ($R = 105$ px)**: Reactive zone providing continuous lateral repulsion steering (`repulsion_steer`) to smoothly nudge the vehicle away from flank obstacles.
     - **Perception Disc ($R = 180$ px)**: Wide-angle situational awareness across 8 radial sectors (`FRONT`, `FR`, `RIGHT`, `RR`, `REAR`, `RL`, `LEFT`, `FL`).
   - Visually renders concentric zones, a rotating radar beam, pulsing sensor rings, and contact blip dots on the road.
   - Embeds a real-time **360° mini radar HUD** inside the AI Cognitive Dashboard displaying sensor status, closest threat distance, and lateral repulsion percentages.

2. **Cows Motivated by Horns & Deadlocks**:
   - Cows react dynamically to acoustic horn pulses (within 210 px) and stopped vehicle deadlocks.
   - When honked at or blocked, resting cows stand up, perk their ears, display an alert badge (`!`), and walk out of the drivable road lanes toward the shoulder (`state = 'CLEARING_ROAD'`).
   - Autonomous vehicles and traffic vehicles automatically pulse their horns when blocked by bovine obstacles, smoothly clearing traffic jams.

3. **Strict Safe Distance Cushion Between All Entities**:
   - All vehicles maintain generous headway safety cushions:
     - Vehicle-to-vehicle: 34 px bumper safety margin.
     - Vehicle-to-pedestrian: 50 px margin.
     - Vehicle-to-cow: 60 px margin.
   - Traffic vehicles stop smoothly behind obstacles, preventing vehicle-vehicle and vehicle-pedestrian collisions.

4. **Road-Aligned A\* Path Tracer (Curvature Direction Tracking)**:
   - Built on a moving **Frenet spatial lattice** parameterized by longitudinal progress along the road and lateral offset from the road centerline.
   - Forward search transitions naturally follow the road's organic curve tangent.
   - The active path tracer line **points directly in the direction of the road** ahead, curving through bends and swerves rather than pointing straight up to the window top.

2. **Indian Bovines (Cows) with Early Threat Prediction**:
   - Realistic top-down cattle models with prominent Zebu dorsal humps, curved horns, ears, and animated tail swishes across 4 authentic breeds (White Zebu, Brown Desi, Spotted, Grey Gyr).
   - **Edge Grazing Herds**: Herds of max 4 cows walking peacefully along the road shoulders and edges (~10–16 px/s).
   - **Middle-of-Road Resting**: Cows calmly sitting down in the middle of the asphalt chewing cud, unbothered by honking or traffic.
   - **Long-Range Prediction & Early Avoidance**: Both the autonomous car and NPC traffic detect resting and grazing cows from far off (up to 220+ px), smoothly planning wide avoidance arcs well ahead of time. The car brings speed to a complete halt if the lane is directly blocked.

3. **Decoupled Pedestrian Quota & Lively Highway Vehicle Flow**:
   - Pedestrians have their own independent quota and spawn cycle, completely decoupled from traffic density.
   - Fixed empty road bug by recycling fast vehicles that drive far ahead, pre-populating traffic at startup/reset, and scaling active vehicles from 8 (sparse) to 28 (rush hour).
   - The highway feels constantly vibrant and alive with trucks, buses, taxis, auto-rickshaws, and motorcycles.

4. **Pedestrian & Vehicle Deadlock Resolution**:
   - Both pedestrian and vehicle communicate intent through kinematic perception.
   - When facing each other at a standstill, the system resolves deadlocks with a **pedestrian-first bias**:
     - The pedestrian detects when the vehicle yields/stops (`v.speed < 22 px/s` or after brief pause) and takes right-of-way, accelerating briskly across to clear the road.
     - The vehicle senses whether the pedestrian is standing or actively moving (`ped.is_moving`), patiently holding its stop until the pedestrian reaches safety, and only proceeding once the path is clear.
   - Pedestrians also feature self-preservation: stopping in their tracks if a fast vehicle approaches or averting sideways toward the nearest shoulder.

5. **Dedicated AI Cognitive Dashboard Sidebar (`1260 x 760` Resolution)**:
   - Dedicated sidebar for perception telemetry, live steer/throttle/brake meters, and internal deliberation stream.
   - Integrated Traffic Density Slider (click & drag or `[` / `]` shortcuts).
   - Road viewport and traffic remain 100% visible and unobstructed.

6. **Intelligent Pothole Crossing vs Solid Obstacle Stopping**:
   - Solid obstacles (vehicles, pedestrians, resting cows): complete stop at rest (`0 km/h`).
   - Potholes: assigned finite high penalties; if completely unavoidable in narrow pinches, the car crawls across cautiously (~20–25 km/h) instead of stalling.

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

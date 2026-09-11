# Indian Highway Autonomous Driving (A* Pathfinding in Pygame)

A pure Pygame 2D simulation with **zero external backend or machine learning dependencies**. It models an autonomous car navigating an infinite, curvy, unstructured Indian highway filled with potholes, unpaved shoulders, unpredictable pedestrians, and authentic traffic using the **A\* (A-Star) search algorithm**.

---

## 🌟 Key Features

1. **Intelligent Traffic Management & Zero Off-Road Guarantee**:
   - **Strict Road Clamping**: All vehicles (Trucks, Buses, Cars, Autos, Bikes) are strictly clamped within the drivable asphalt road; no vehicles go offroad onto dirt shoulders.
   - **Inter-Vehicle Collision Avoidance**: NPC vehicles continuously check forward headway. If blocked by another vehicle, pedestrian, or player car, they automatically decelerate or come to a **complete stop at rest (speed = 0.0)**.
   - **Gap-Checked Lane Cuts**: Before executing a lane change, NPC vehicles verify that adjacent spaces are clear.

2. **Complete Stop-at-Rest Capability (`Speed = 0.0`)**:
   - Both the main autonomous vehicle and NPC vehicles are capable of coming to a complete stop (`0 km/h`) whenever blocked by crossing pedestrians or stopped traffic ahead, activating rear brake lights and restarting smoothly when the road clears.

3. **Dynamic Potholes (Reduced Probability by 55–60%)**:
   - Pothole spawn frequency has been reduced significantly (~55% less frequent).
   - Sizes are dynamically generated across three distinct categories:
     - **Small Surface Potholes** (9–14 px)
     - **Medium Road Craters** (17–26 px)
     - **Large Hazardous Trenches** (32–48 px)

4. **AI Cognitive Dashboard (Toggle with `[TAB]` or `[T]`)**:
   - Live perception metrics: Safety margin progress gauge, focus threat indicator, dynamic road width.
   - Dual-sided steering intent slider (`LEFT` / `RIGHT` / `CENTER`) and throttle/braking intensity bars.
   - Internal Deliberation Stream: Live timestamped thought log explaining the vehicle's real-time decisions (e.g., pre-braking for curves, honking at pedestrians, coming to full stop behind traffic, navigating bottlenecks).

5. **5 Distinct Vehicle Varieties with Custom Driving Profiles**:
   - 🚚 **Truck**: Heavy multi-axle carrier. Slow speed (80–105 px/s), heavy inertia, very gentle lateral transitions (~22 px/s), wide curves.
   - 🚌 **Bus**: Long passenger bus with luggage rack. High inertia, stays centered/left, slow steady curves.
   - 🚗 **Car**: Passenger sedan/hatchback. Balanced cruising speed (~125–165 px/s).
   - 🛺 **Auto-Rickshaw**: 3-wheeler (22x36 px) with green body & yellow canopy. Moderate speed, weaves through inner shoulders.
   - 🏍️ **Bike**: Agile motorcycle with rider helmet (13x26 px). Fastest (145–190 px/s) and most nimble.

6. **Realistic Highway Steering Constraints on Primary Vehicle**:
   - Angular turn rate capped at `1.45 rad/s` with vehicular yaw damping.
   - Off-axis heading clamped within ~20.5° (0.38 rad) of the road tangent, preventing sharp or unrealistic side-swerves.

7. **Dynamic Road Width & Sparse Markings**:
   - Road narrows down to single-lane bottlenecks (~205 px) and expands to open stretches (~420 px).
   - Weathered, broken white center markings appear only sparsely (~15% of the road).

---

## 🎮 Controls

| Key | Action |
|---|---|
| **`TAB`** / **`T`** | **Toggle AI Cognitive Dashboard** (Thoughts, observations, actuator gauges) |
| **`A`** | **Toggle Auto-Speed Mode** (Adaptive Speed vs Manual Speed) |
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

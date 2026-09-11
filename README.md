# Indian Highway Autonomous Driving (A* Pathfinding in Pygame)

A pure Pygame 2D simulation with **zero external backend or machine learning dependencies**. It models an autonomous car navigating an infinite, curvy, unstructured Indian highway filled with potholes, unpaved shoulders, unpredictable pedestrians, and authentic traffic using the **A\* (A-Star) search algorithm**.

---

## 🌟 Features

1. **AI Cognitive Dashboard (Toggle with `[TAB]` or `[T]`)**:
   - Displays real-time internal observations and perception metrics:
     - **Safety Margin** gauge (40% - 99%).
     - **Steering Intent** dynamic centered dual slider (`LEFT` / `RIGHT` / `CENTER`).
     - **Throttle & Braking** intensity progress bars.
     - **Internal Deliberation Stream**: Live timestamped thought log explaining the vehicle's real-time decisions (e.g., pre-braking for curves, honking at pedestrians, overtaking slow trucks, easing throttle in bottlenecks).

2. **5 Authentic Indian Traffic Varieties with Custom Driving Dynamics**:
   - 🚚 **Truck**: Heavy multi-axle freight carrier. Very gentle lateral speed (~24 px/s), heavy inertia, rare cuts (~20% probability), wide sweeping curves.
   - 🚌 **Bus**: Long State Transport passenger bus with luggage roof rack. Steady cruising, high inertia, stays centered/left.
   - 🚗 **Car**: Standard sedan/hatchback. Balanced cruising speed (~130-170 px/s) and moderate lane changes.
   - 🛺 **Auto-Rickshaw**: Nimble 3-wheeler with green body & yellow canopy. Moderate speed, weaves through inner shoulders and gaps.
   - 🏍️ **Bike**: Fastest and most agile (~150-195 px/s). Frequently weaves nimbly through tight openings with high lateral agility.

3. **Smooth Kinematic Lane Cuts (Zero Teleportation)**:
   - NPC vehicles change lanes using smooth lateral acceleration physics (`self.vx`) and dynamic yaw angle tilt rather than snapping or jumping across positions.
   - Sharp cut frequency reduced by 25–30%.

4. **Realistic Highway Steering Constraints for Primary Vehicle**:
   - Angular turn rate capped at `1.45 rad/s` with realistic vehicular yaw damping.
   - Strict heading constraint: Off-axis yaw is clamped within ~20.5° of the road tangent, preventing unrealistic sharp side-swerves.
   - Extended lookahead pursuit distance (65–125 px) enforces graceful forward curves.

5. **Dynamic Road Width & Sparse Markings**:
   - Organically narrows down into single-lane bridges / bottlenecks (~205 px) and expands into broad stretches (~420 px).
   - Weathered, broken white center markings appear only sparsely on isolated stretches (~15% of the road).

6. **Random Jaywalking Pedestrians**:
   - Pedestrians walking along the shoulders unpredictably step out and cross the road without warning. The car detects, pre-brakes, and sounds its horn.

7. **Adaptive Auto-Speed Mode (`[A]`)**:
   - Situationally modulates throttle based on road width, upcoming curve severity, pedestrian crossing hazards, and traffic gaps.

---

## 🎮 Controls

| Key | Action |
|---|---|
| **`TAB`** / **`T`** | **Toggle AI Cognitive Dashboard** (Internal thoughts, observations, actuator gauges) |
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

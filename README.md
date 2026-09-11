# Indian Highway Autonomous Driving (A* Pathfinding in Pygame)

A pure Pygame 2D simulation with **zero external backend or machine learning dependencies**. It models an autonomous car navigating an infinite, curvy, unstructured Indian highway filled with potholes, roadside shoulders, and authentic traffic (auto-rickshaws, colorful trucks, cars, and scooters) using the **A\* (A-Star) search algorithm**.

---

## 🌟 Key Features

1. **Procedural Infinite Indian Road**:
   - Organically winding curves using multi-octave sinusoidal modulation.
   - Dynamic width variations (choke points and wide stretches).
   - Worn asphalt patches, faded/broken center divider lines, unpaved dirt shoulders, and roadside NH milestones.

2. **A\* Local Lattice Path Planner**:
   - Discretizes a dynamic lookahead grid ahead of the vehicle into a spatial lattice.
   - **Cost Function $g(n)$**:
     - Exponential off-road / shoulder penalty.
     - Hard collision exclusion + repulsive potential field around potholes.
     - Dynamic traffic collision avoidance with velocity and arrival-time projection.
     - Gentle centerline affinity to maintain natural lane positioning.
   - **Admissible Heuristic $h(n)$**: Euclidean distance to the optimal road spine waypoint at the planning horizon.
   - **Chaikin Path Smoothing**: Converts raw grid steps into curved, driveable trajectories.

3. **Autonomous Vehicle Controller**:
   - **Pure Pursuit Steering**: Smooth heading adjustments with realistic angular turn rate limits.
   - **Dynamic Speed Adaptation**: Automatically decelerates into tight curves or congested traffic, and accelerates on open straights.
   - **Procedural Vehicle Aesthetics**: Headlight projection beam cone, responsive brake lights, suspension bump shudder upon hitting potholes, and visual horn soundwave indicators.

4. **Dynamic Obstacles & Traffic**:
   - **Potholes**: Irregular, jagged craters with deep cavities and crumbling rims.
   - **Traffic Vehicles**: Distinct vehicle types with realistic behaviors:
     - 🛺 **Auto-Rickshaws**: Slower cruising, drifts near road edges.
     - 🚚 **Highway Trucks**: Large, decorative multi-axle freight carriers.
     - 🚗 **Passenger Cars**: Moderate cruising sedans and hatchbacks.
     - 🛵 **Scooters**: Nimble two-wheelers.

5. **Interactive Telemetry HUD & Controls**:
   - Real-time speedometer (km/h), distance traveled, FPS, A* nodes evaluated per cycle, potholes safely dodged, and traffic overtaken.
   - Live visual debugging overlay showing the A* search tree (explored nodes), hazard grid cells, and glowing planned path.

---

## 🎮 Controls

| Key | Action |
|---|---|
| `D` | Toggle A* Path & Search Lattice Debug Overlay |
| `SPACE` | Pause / Resume simulation |
| `UP` / `DOWN` | Increase / Decrease target cruise speed |
| `R` | Reset / Restart simulation |
| `ESC` | Exit simulation |

---

## 🚀 How to Run

Ensure Python 3 and Pygame are installed:

```bash
# Run the interactive simulation
python3 main.py
```

### Headless Mode (for automated testing / CI)
```bash
SDL_VIDEODRIVER=dummy python3 main.py --frames=600
```

---

## 📁 Codebase Architecture

- [`config.py`](file:///home/ud_1402/autoDetect/config.py): Global simulation parameters, display resolution, physics constants, colors, and planner tuning.
- [`road.py`](file:///home/ud_1402/autoDetect/road.py): Analytical procedural generator for the infinite curvy road, shoulders, and asphalt patches.
- [`obstacles.py`](file:///home/ud_1402/autoDetect/obstacles.py): Pothole and traffic vehicle classes, procedural spawning, and collision shapes.
- [`planner.py`](file:///home/ud_1402/autoDetect/planner.py): High-performance A* search algorithm over local spatio-temporal lattice with Chaikin path smoothing.
- [`car.py`](file:///home/ud_1402/autoDetect/car.py): Autonomous vehicle kinematics, pure pursuit tracking, dynamic throttle/braking, and rendering.
- [`main.py`](file:///home/ud_1402/autoDetect/main.py): Game loop, smooth camera tracking, roadside props, telemetry HUD, and event dispatching.

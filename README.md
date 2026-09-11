# Indian Highway Autonomous Driving (A* Pathfinding in Pygame)

A pure Pygame 2D simulation with **zero external backend or machine learning dependencies**. It models an autonomous car navigating an infinite, curvy, unstructured Indian highway filled with potholes, unpaved shoulders, unpredictable pedestrians, and chaotic traffic using the **A\* (A-Star) search algorithm**.

---

## 🌟 Features Implemented

1. **Sparse Road Markings**:
   - Uniform center lines removed. Faded, weathered dashes appear only sparsely here and there (~15% of the highway) while the rest remains raw, unmarked asphalt.

2. **Dynamic Road Width (Bottlenecks & Open Highway)**:
   - Width dynamically fluctuates between wide open highway stretches (~420 px) and narrow choke points / culverts / single-lane squeezes (~205 px).

3. **Random Jaywalking Pedestrians**:
   - Pedestrians walk along the dirt shoulders and unpredictably decide to cut directly across the asphalt road without warning or indicators.
   - The car's A* pathfinder anticipates pedestrian walking vectors, brakes when necessary, and honks its horn.

4. **Controlled Vehicle Steering Constraints**:
   - Realistic yaw damping and angular turn-rate constraints prevent random or erratic steering snaps. The vehicle smoothly tracks the road curvature with stable vehicular momentum.

5. **Chaotic NPC Vehicle Lane-Cutting**:
   - NPC vehicles (Auto-rickshaws, Trucks, Cars, Scooters) randomly cut into different lanes and sides of the road without warning or indicators.

6. **Adaptive Auto-Speed Mode (Keymap: `[A]`)**:
   - Dynamically scales cruise speed based on real-time driving conditions:
     - **Accelerates** on wide, straight, open roads (up to ~75-80 km/h).
     - **Slows down** in narrow bottlenecks and choke points.
     - **Pre-brakes** when approaching sharp upcoming curves.
     - **Crawls or stops** when pedestrians cross or traffic suddenly cuts ahead.

---

## 🎮 Controls

| Key | Action |
|---|---|
| `A` | **Toggle Auto-Speed Mode** (Adaptive Speed vs Manual Speed) |
| `D` | **Toggle A\* Search Overlay** (Explored nodes, hazard cells, waypoints) |
| `SPACE` | **Pause / Resume** simulation |
| `UP` / `DOWN` | Adjust Target Cruise Speed |
| `R` | **Reset** simulation |
| `ESC` | Exit simulation |

---

## 🚀 How to Run

```bash
cd /home/ud_1402/autoDetect
python3 main.py
```

### Headless Verification Mode
```bash
SDL_VIDEODRIVER=dummy python3 main.py --frames=600
```

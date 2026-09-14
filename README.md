# Autonomous Driving for Unstructured Indian Roads

An intuitive, software-driven solution designed to help self-driving vehicles navigate the unique, unpredictable conditions of Indian roads—where painted lane markers are missing, traffic flows freely from all directions, and road hazards like potholes and roaming animals are common.

This project contains **two separate simulations**, each addressing a critical part of the self-driving challenge:
1. **The Autonomous Vehicle Path Navigation Simulation**: Demonstrates how the car plans its journey, invents its own driving lanes, and coordinates its senses to dodge road hazards in real time.
2. **The Demo Sensor Comparison Simulation**: Tests, grades, and compares different electronic "eyes and ears" to find the most cost-effective and reliable sensor package for tough road conditions.

---

## Simulation 1: Autonomous Vehicle Path Navigation

This simulation places a virtual car onto a busy, winding Indian highway. The car has no human driver and must safely drive forward while dealing with sudden obstacles, unorganized traffic, resting cows, crossing pedestrians, and deep potholes.

```
       [ Long-Range Planner: Looks Far Ahead (Big Picture Path) ]
                                 │
                                 ▼
       [ Short-Range Planner: Reacts Up Close (Instant Dodges)  ]
                                 │
       ┌─────────────────────────┼─────────────────────────┐
       ▼                         ▼                         ▼
 [ Invisible Lanes ]    [ Teamwork of Senses ]    [ Finding Missing Ground ]
 (Draws its own paths)  (Cameras + Lasers + Radar)   (Spotting Potholes)
```

### 1. The Two-Layer Algorithm: Long-Range Mapping vs. Short-Range Reaction
Human drivers naturally think on two levels: you look far down the highway to see where the road bends, but you also watch right in front of your tires to dodge unexpected bumps. The car does the exact same thing using a **two-layer pathfinding system**:

* **Looking Far Ahead (Long-Range Planning)**:
  * The car scans several car lengths into the distance to understand the general curve of the highway.
  * It maps out a smooth, gradual corridor to follow so the ride stays comfortable and never jerky.
* **Reacting Up Close (Short-Range Rapid Dodging)**:
  * While the long-range planner keeps the overall journey on track, a second, faster reaction system constantly inspects the space immediately around the car's bumper.
  * If an auto-rickshaw suddenly swerves, a pedestrian steps out, or a pothole appears, this quick-reaction brain instantly steers the car around the danger in milliseconds, without losing the main road direction.

### 2. Creating Invisible Lanes (Virtual Lane Navigation)
In many countries, autonomous cars rely heavily on painted white lines. On Indian roads, lane markings are often faded, broken, or completely absent. 

* Instead of searching for painted lines that aren't there, the car continuously checks the **open, drivable width of the road** using RGB Cameras to mathematically approximate width of the road.
* It divides this open space in its imagination into **neat, invisible driving lanes**.
* It naturally prefers to stay in the safest imaginary lane on the left (following standard traffic habits), while automatically drifting or nudging around road edges when obstacles appear.
* For two-way traffic, the car virtually approximates the centerline of the road, creating a continuous, smooth curved trace.

### 3. Teamwork of Senses (Multimodal Sensor Fusion)
No single electronic sense can see everything perfectly. A regular camera can get blinded by bright headlights or fail in heavy rain; a simple radar cannot read surface details. To solve this, the car combines three complementary senses:

* **Color(RGB) Cameras (Like Human Eyes)**:
  * Captures colors, shapes, and surface patterns. It recognizes vehicles, pedestrians, and boundaries.
* **Laser Light Scanners (LiDARs)**:
  * Shoots out millions of invisible light beams every second and clocks how fast they bounce back.
  * This builds an exact 3D physical map of everything around the car, measuring precise distances down to the centimeter.
* **Special Super-Radar (4D Imaging Radar - Weatherproof Vision & Speed Sensing)**:
  * Uses radio waves that easily cut through thick dust, monsoon rain, dense fog, and nighttime darkness.
  * Unlike older radars, it can tell how high an object is and instantly senses how fast oncoming or trailing vehicles are moving toward or away from us.

Combining these three sensors input, we achieve multi-modality, which is useful in solving negative-obstacle detection(potholes, trenches, elevated road edges)

### 4. Spotting Potholes using LiDAR Dataset Voids and Multimodality
Most self-driving systems only look for objects sticking *up* from the ground (like other cars or walls). Potholes and ditches are "negative hazards"—places where the ground drops *down*.

* **How the Lasers Spot a Hole**: When the car beams laser light at flat road, the light bounces back in a steady, predictable blanket. But when there is a pothole, the ground suddenly drops away. The light beams either disappear into the hole or return much later than expected.
* **Finding the Empty Gap**: This creates a clear **"blank spot" or "empty void"** in the laser data where solid road ought to be.
* **Confirming with the Camera**: The car's computer pairs that empty laser gap with the dark, jagged shape seen by the color camera, infering the presence of potholes
* **Crawling Condition**: At high driving speeds, the car treats the pothole as a solid obstacle and steers smoothly around it. If traffic is heavily jammed and swerving is unsafe, the car gently slows down to a crawl so it can roll through without damaging tires or suspension.

We also expect false voids in the LiDAR dataset, and thus use a confidence score based approach, integrating all three sensors together, to cross-verify the signal.

---

## Simulation 2: Sensor Simulation and Cost Approximation

Before spending huge sums of money physically buying and installing expensive hardware on real cars, this second simulation approximately tests, compares, and grades different sensor setups in computer-generated safety tests, to verify the simulations. 
The primary purpose of this web program is to **be able to visualize the activity of different sensors** to achieve our purpose of "multi-modal integration"

```
 [ Sensor Options ] ──▶ [ Test Distances ] ──▶ [ Diverse Scenarios ] ──▶ [ Combined Score ]
 (Cameras, Lasers,      (Close, Medium,         (Day, Night, Rain,      (Best Safety for
  Radars, Thermal)       Far Ranges)             Muddy Water Pools)      Every Rupee Spent)
```

### 1. Testing Effectiveness at Different Distances
A sensor that works great close to the bumper might be completely useless for spotting hazards far ahead, and vice versa:
* **Close Range (3 to 6 meters)**: Crucial for inspecting the road surface directly under the front wheels to catch sudden dips and potholes.
* **Medium Range (6 to 10 meters)**: Gives the car enough time to decide whether to gently steer around a hazard or slow down.
* **Far Range (10 to 15+ meters)**: Necessary for high-speed highway driving so the vehicle can spot large blockages early and brake smoothly.

The simulation runs every sensor across all of these distances to measure where it is strongest and where it goes blind, including multiple angular settings

### 2. Testing in Tough Real-World Conditions
The simulation tests each sensor under realistic conditions:
* **Day vs. Pitch-Black Night**: Checking how well cameras perform under headlights compared to radars and lasers that carry their own light and signals.
* **Dry Road vs. Rain Puddles**: Water-filled potholes are notoriously dangerous because water reflects the sky like a mirror, tricking normal cameras into thinking the road is flat. The simulation tests how radar waves and specialized lasers penetrate or bounce off water surfaces.

### 3. Equipment Cost vs. Real-World Value
High-end aerospace sensors can cost tens of lakhs of rupees, making a production car unaffordable. Simple cameras are very cheap, but fail in the dark or fog.

* The simulation assigns realistic price tags (in Indian Rupees) to every piece of hardware.
* It measures how much **genuine safety protection** each sensor delivers per thousand rupees invested.

### 4. The Combined Teamwork Score (Benchmarking)
Instead of guessing which sensor is "the best", the simulation brings all the results together into a single, comprehensive **Combined Score (Report Card Grade)**:
* It balances four key ingredients:
  1. How reliably the sensor spots hazards across all distances.
  2. How well it functions in rain, dust, and darkness.
  3. How affordable it is to manufacture and install.
  4. How well it shares information with the other sensors (teamwork capability).
* **Benchmarking (Ranking)**: The simulation ranks different equipment combinations side by side. It shows manufacturers exactly which setup provides maximum passenger safety at an affordable, realistic price tag for the Indian market.

### 5. The Live Data Feed
The simulation includes an interactive visualizer that acts as a continuous **live feed**:
* It simulates the steady stream of information flowing from the car's sensors into the decision-making brain.
* As you move a virtual hazard closer or change the weather from clear day to midnight rain, the live feed displays in real time how each sensor's confidence rises or falls, demonstrating why multi-sensor teamwork is essential for survival on Indian roads.

---

# How to Run the Simulations

## Running Simulation 1 (Autonomous Path Navigation)
To launch the 2D highway pathfinding and driving simulation:

```bash
git clone https://github.com/uday0214/SIH_2026_TEAM_MAIVSU.git

cd SIH_2026_TEAM_MAIVSU/path_tracing_simulation

python3 main.py
```

* **Controls during the simulation**:
  * `TAB` or `T`: Open / close the live dashboard panel.
  * `[` / `]`: Increase or decrease surrounding traffic.
  * `A`: Toggle automatic speed adjustment.
  * `D`: Show / hide the pathfinding search grid and hazard markers.
  * `SPACE`: Pause or resume the simulation.
  * `R`: Reset the car to the starting point.

---

### Running Simulation 2 (Sensor Benchmarking and Testing)

* **To run the automated benchmark calculation and print the score report**:
  ```bash
  git clone https://github.com/uday0214/SIH_2026_TEAM_MAIVSU.git
  ```

* **To open the interactive visual sensor feed in your web browser**:
  Open the file `sensor_simulations/lidar_sim.html` in any web browser (such as Chrome or Firefox) to interactively adjust distance sliders, toggle day/night, change water levels in potholes, and observe live sensor scores.
